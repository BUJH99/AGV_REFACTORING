#include "agv/render_ipc_server.hpp"
#include "agv/simulation_engine.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

agv::core::ScenarioConfig make_single_phase_custom_scenario() {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};
    return scenario;
}

agv::core::ScenarioConfig make_realtime_idle_scenario() {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Realtime;
    scenario.speedMultiplier = 0.0;
    scenario.realtimeParkChance = 0;
    scenario.realtimeExitChance = 0;
    return scenario;
}

void configure_engine(
    agv::core::SimulationEngine& engine,
    agv::core::PathAlgo algorithm,
    const agv::core::ScenarioConfig& scenario,
    int map_id = 1) {
    engine.setSeed(7);
    engine.loadMap(map_id);
    engine.setAlgorithm(algorithm);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
}

void merge_agent_updates(
    std::vector<agv::core::AgentRenderState>& agents,
    const std::vector<agv::core::AgentRenderState>& updates) {
    for (const auto& update : updates) {
        auto it = std::find_if(agents.begin(), agents.end(),
            [&update](const auto& agent) { return agent.id == update.id; });
        if (it == agents.end()) {
            agents.push_back(update);
        } else {
            *it = update;
        }
    }
}

void merge_goal_updates(
    std::vector<agv::core::GoalRenderState>& goals,
    const std::vector<agv::core::GoalRenderState>& updates) {
    for (const auto& update : updates) {
        auto it = std::find_if(goals.begin(), goals.end(),
            [&update](const auto& goal) {
                return goal.position.x == update.position.x && goal.position.y == update.position.y;
            });
        if (it == goals.end()) {
            goals.push_back(update);
        } else {
            *it = update;
        }
    }
}

void apply_delta(
    agv::core::RenderFrameSnapshot& frame,
    const agv::core::RenderFrameDelta& delta) {
    frame.sessionId = delta.sessionId;
    frame.sceneVersion = delta.sceneVersion;
    frame.frameId = delta.toFrameId;
    frame.lastLogSeq = delta.lastLogSeq;
    if (delta.hudChanged) {
        frame.hud = delta.hud;
    }
    merge_agent_updates(frame.agents, delta.agentUpdates);
    merge_goal_updates(frame.goalStates, delta.goalStateChanges);
    frame.logsTail.insert(frame.logsTail.end(), delta.newLogs.begin(), delta.newLogs.end());
    if (delta.overlayChanged) {
        frame.plannerOverlay = delta.plannerOverlay;
    }
}

bool has_ansi_escape(const std::string& text) {
    return text.find('\x1b') != std::string::npos;
}

TEST(RenderModelTest, StaticSceneRemainsStableWithinSessionAndReloadsWithNewSession) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario(), 1);

    const auto first = engine.snapshotStaticScene();
    const auto second = engine.snapshotStaticScene();

    EXPECT_EQ(first.sessionId, second.sessionId);
    EXPECT_EQ(first.sceneVersion, second.sceneVersion);
    EXPECT_EQ(first.baseTiles, second.baseTiles);
    EXPECT_EQ(first.goalCells.size(), second.goalCells.size());
    EXPECT_EQ(first.homeCells.size(), second.homeCells.size());

    engine.loadMap(3);
    const auto third = engine.snapshotStaticScene();
    EXPECT_GT(third.sessionId, first.sessionId);
    EXPECT_EQ(third.sceneVersion, 1u);
    EXPECT_NE(third.baseTiles, first.baseTiles);
}

TEST(RenderModelTest, RenderFrameAndDeltaAdvanceWithSimulationSteps) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario(), 1);

    const auto first = engine.snapshotRenderFrame();
    EXPECT_EQ(first.frameId, 0u);

    engine.step();
    const auto second = engine.snapshotRenderFrame();
    const auto delta = engine.snapshotRenderDelta(first.frameId);

    EXPECT_GT(second.frameId, first.frameId);
    EXPECT_FALSE(delta.requiresFullResync);
    EXPECT_EQ(delta.fromFrameId, first.frameId);
    EXPECT_EQ(delta.toFrameId, second.frameId);
    EXPECT_FALSE(delta.agentUpdates.empty());
    EXPECT_TRUE(delta.hudChanged);
}

TEST(RenderModelTest, GoalStateChangesCaptureParkingCompletion) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario(), 1);

    agv::core::RenderFrameSnapshot frame = engine.snapshotRenderFrame();
    bool found_parking_delta = false;

    for (int step = 0; step < 256 && !found_parking_delta; ++step) {
        engine.step();
        const auto delta = engine.snapshotRenderDelta(frame.frameId);
        if (std::any_of(delta.goalStateChanges.begin(), delta.goalStateChanges.end(),
                [](const auto& goal) { return goal.isParked; })) {
            found_parking_delta = true;
        }
        frame = engine.snapshotRenderFrame();
    }

    EXPECT_TRUE(found_parking_delta);
}

TEST(RenderModelTest, StructuredLogsAreSanitizedAndCategorized) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario(), 1);

    engine.step();
    const auto frame = engine.snapshotRenderFrame();

    ASSERT_FALSE(frame.logsTail.empty());
    EXPECT_FALSE(std::all_of(frame.logsTail.begin(), frame.logsTail.end(),
        [](const auto& entry) { return entry.category == "GENERAL"; }));
    EXPECT_TRUE(std::all_of(frame.logsTail.begin(), frame.logsTail.end(),
        [](const auto& entry) { return !has_ansi_escape(entry.text); }));
}

TEST(RenderModelTest, PlannerOverlayProvidesCoordinateBasedPaths) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario(), 1);

    engine.step();
    agv::core::RenderQueryOptions options;
    options.plannerOverlay = true;
    const auto frame = engine.snapshotRenderFrame(options);

    EXPECT_TRUE(frame.plannerOverlay.available);
    EXPECT_EQ(frame.plannerOverlay.horizon, frame.hud.whcaHorizon);
    EXPECT_FALSE(frame.plannerOverlay.plannedPaths.empty());
    EXPECT_TRUE(std::all_of(frame.plannerOverlay.plannedPaths.begin(), frame.plannerOverlay.plannedPaths.end(),
        [](const auto& path) { return !path.cells.empty(); }));
}

TEST(RenderModelTest, StaleFrameRequestRequiresFullResyncAfterHistoryWindow) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_realtime_idle_scenario(), 1);

    const auto initial = engine.snapshotRenderFrame();
    for (int step = 0; step < 300; ++step) {
        engine.step();
    }

    const auto delta = engine.snapshotRenderDelta(initial.frameId);
    EXPECT_TRUE(delta.requiresFullResync);
}

TEST(RenderModelTest, FullSnapshotPlusDeltaReplayReconstructsLatestState) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_realtime_idle_scenario(), 1);

    agv::core::RenderFrameSnapshot replayed = engine.snapshotRenderFrame();
    for (int step = 0; step < 6; ++step) {
        engine.step();
        const auto delta = engine.snapshotRenderDelta(replayed.frameId);
        ASSERT_FALSE(delta.requiresFullResync);
        apply_delta(replayed, delta);
    }

    const auto latest = engine.snapshotRenderFrame();
    EXPECT_EQ(replayed.frameId, latest.frameId);
    EXPECT_EQ(replayed.hud.step, latest.hud.step);
    EXPECT_EQ(replayed.goalStates.size(), latest.goalStates.size());
    EXPECT_EQ(replayed.agents.size(), latest.agents.size());

    std::unordered_map<int, agv::core::GridCoord> latest_positions;
    for (const auto& agent : latest.agents) {
        latest_positions.emplace(agent.id, agent.position);
    }
    for (const auto& agent : replayed.agents) {
        const auto it = latest_positions.find(agent.id);
        ASSERT_NE(it, latest_positions.end());
        EXPECT_EQ(agent.position.x, it->second.x);
        EXPECT_EQ(agent.position.y, it->second.y);
    }
}

TEST(RenderModelTest, IpcServerSupportsStaticSceneDeltaFlowWithMockClient) {
    std::stringstream input;
    input
        << R"({"command":"loadMap","mapId":1})" << '\n'
        << R"({"command":"setAlgorithm","algorithm":"default"})" << '\n'
        << R"({"command":"configureScenario","scenario":{"mode":"custom","speedMultiplier":0.0,"phases":[{"type":"park","taskCount":1}]}})" << '\n'
        << R"({"command":"subscribeFrameDelta","enabled":true})" << '\n'
        << R"({"command":"getStaticScene"})" << '\n'
        << R"({"command":"getFrame"})" << '\n'
        << R"({"command":"step"})" << '\n'
        << R"({"command":"getDelta","sinceFrameId":0})" << '\n';

    std::stringstream output;
    agv::ipc::RenderIpcServer server;
    server.run(input, output);

    bool saw_scene = false;
    bool saw_event = false;
    bool saw_delta = false;

    std::string line;
    while (std::getline(output, line)) {
        if (line.empty()) {
            continue;
        }
        const auto message = nlohmann::json::parse(line);
        if (message.value("command", std::string{}) == "getStaticScene") {
            saw_scene = true;
            EXPECT_TRUE(message.contains("scene"));
            EXPECT_EQ(message["scene"]["width"], 82);
        }
        if (message.value("event", std::string{}) == "frameDeltaAvailable") {
            saw_event = true;
            EXPECT_GE(message["frameId"].get<std::uint64_t>(), 1u);
        }
        if (message.value("command", std::string{}) == "getDelta") {
            saw_delta = true;
            EXPECT_FALSE(message["delta"]["requiresFullResync"].get<bool>());
            EXPECT_GE(message["delta"]["toFrameId"].get<std::uint64_t>(), 1u);
        }
    }

    EXPECT_TRUE(saw_scene);
    EXPECT_TRUE(saw_event);
    EXPECT_TRUE(saw_delta);
}

}  // namespace
