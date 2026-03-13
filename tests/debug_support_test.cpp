#include "agv/console_shell.hpp"
#include "agv/simulation_engine.hpp"
#include "agv/internal/engine_internal.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

namespace {

agv::core::ScenarioConfig make_debug_scenario() {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};
    return scenario;
}

void configure_debug_engine(agv::core::SimulationEngine& engine) {
    engine.setSeed(7);
    engine.loadMap(1);
    engine.setAlgorithm(agv::core::PathAlgo::Default);
    engine.configureScenario(make_debug_scenario());
    engine.setSuppressOutput(true);
}

TEST(DebugSupportTest, InternalRecentLogCollectionPreservesChronologicalOrderAcrossWraparound) {
    Logger logger;

    for (int index = 0; index < LOG_BUFFER_LINES + 2; ++index) {
        logger.appendLine("line-" + std::to_string(index));
    }

    const auto lines = collect_recent_logs(&logger);
    ASSERT_EQ(lines.size(), static_cast<std::size_t>(LOG_BUFFER_LINES));
    EXPECT_EQ(lines.front(), "line-2");
    EXPECT_EQ(lines.back(), "line-6");
}

TEST(DebugSupportTest, StructuredLogCollectionPreservesRepeatedMessagesBeyondLegacyTail) {
    Logger logger;
    logger.setContext(7, 0, -1);

    for (int index = 0; index < LOG_BUFFER_LINES + 3; ++index) {
        logger.appendLine("[CTRL] repeated");
    }

    const auto entries = collect_structured_logs(&logger, 0, RENDER_LOG_BATCH_LIMIT);
    ASSERT_EQ(entries.size(), static_cast<std::size_t>(LOG_BUFFER_LINES + 3));
    EXPECT_EQ(entries.front().seq, 1u);
    EXPECT_EQ(entries.back().seq, static_cast<std::uint64_t>(LOG_BUFFER_LINES + 3));
    EXPECT_TRUE(std::all_of(entries.begin(), entries.end(), [](const auto& entry) {
        return entry.step == 7 && entry.category == "Control" && entry.text == "[CTRL] repeated";
    }));
}

TEST(DebugSupportTest, PublicDebugSnapshotCapturesLogsAgentsAndFrame) {
    agv::core::SimulationEngine engine;
    configure_debug_engine(engine);

    engine.runUntilComplete();
    const agv::core::DebugSnapshot snapshot = engine.snapshotDebugState(true);

    EXPECT_GE(snapshot.metrics.recordedSteps, 1);
    EXPECT_GE(snapshot.metrics.tasksCompletedTotal, 1u);
    EXPECT_GE(snapshot.frame.hud.step, 1);
    EXPECT_FALSE(snapshot.frame.agents.empty());
    EXPECT_FALSE(snapshot.recentLogs.empty());
    EXPECT_LE(snapshot.recentLogs.size(), static_cast<std::size_t>(LOG_BUFFER_LINES));
    EXPECT_FALSE(snapshot.agents.empty());
    EXPECT_GE(snapshot.runtime.currentStep, 1);
    EXPECT_GE(snapshot.runtime.lastStepCpuTimeMs, 0.0);
    EXPECT_GE(snapshot.runtime.plannedMoveCount, snapshot.runtime.finalMoveCount);
    EXPECT_GE(snapshot.runtime.noMovementStreak, 0);

    const auto active_agent = std::find_if(
        snapshot.agents.begin(),
        snapshot.agents.end(),
        [](const agv::core::AgentDebugSnapshot& agent) { return agent.isActive; });
    ASSERT_NE(active_agent, snapshot.agents.end());
    EXPECT_GE(active_agent->posX, 0);
    EXPECT_GE(active_agent->posY, 0);
}

TEST(DebugSupportTest, DebugReportCanBeWrittenToDisk) {
    agv::core::SimulationEngine engine;
    configure_debug_engine(engine);

    engine.step();

    const std::filesystem::path report_path =
        std::filesystem::current_path() / "agv_debug_report_test.txt";
    std::error_code cleanup_error;
    std::filesystem::remove(report_path, cleanup_error);

    ASSERT_TRUE(agv::console::writeDebugReport(engine, report_path.string(), true));
    ASSERT_TRUE(std::filesystem::exists(report_path));

    std::ifstream input(report_path);
    ASSERT_TRUE(input.is_open());
    const std::string contents{
        std::istreambuf_iterator<char>(input),
        std::istreambuf_iterator<char>()};
    input.close();

    EXPECT_NE(contents.find("Simulation Debug Report"), std::string::npos);
    EXPECT_NE(contents.find("Runtime Snapshot"), std::string::npos);
    EXPECT_NE(contents.find("Tasks/AGV"), std::string::npos);
    EXPECT_NE(contents.find("AGV Fairness"), std::string::npos);
    EXPECT_NE(contents.find("cv/min-max"), std::string::npos);
    EXPECT_NE(contents.find("AGV A : tasks="), std::string::npos);
    EXPECT_NE(contents.find("Planner Derived KPIs"), std::string::npos);
    EXPECT_NE(contents.find("Recent Logs"), std::string::npos);
    EXPECT_NE(contents.find("Agent Snapshots"), std::string::npos);
    EXPECT_NE(contents.find("Render Frame"), std::string::npos);

    std::filesystem::remove(report_path, cleanup_error);
}

}  // namespace
