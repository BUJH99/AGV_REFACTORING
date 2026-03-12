#include "agv/render_ipc_server.hpp"

#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>

namespace agv::ipc {

namespace {

using nlohmann::json;

std::string to_string(core::PathAlgo algorithm) {
    switch (algorithm) {
        case core::PathAlgo::AStarSimple:
            return "astar";
        case core::PathAlgo::DStarBasic:
            return "dstar";
        case core::PathAlgo::Default:
        default:
            return "default";
    }
}

std::string to_string(core::SimulationMode mode) {
    return (mode == core::SimulationMode::Realtime) ? "realtime" : "custom";
}

std::string to_string(core::PhaseType phase) {
    return (phase == core::PhaseType::Exit) ? "exit" : "park";
}

std::string to_string(core::AgentState state) {
    switch (state) {
        case core::AgentState::GoingToPark:
            return "going_to_park";
        case core::AgentState::ReturningHomeEmpty:
            return "returning_home_empty";
        case core::AgentState::GoingToCollect:
            return "going_to_collect";
        case core::AgentState::ReturningWithCar:
            return "returning_with_car";
        case core::AgentState::GoingToCharge:
            return "going_to_charge";
        case core::AgentState::Charging:
            return "charging";
        case core::AgentState::ReturningHomeMaintenance:
            return "returning_home_maintenance";
        case core::AgentState::Idle:
        default:
            return "idle";
    }
}

core::PathAlgo parse_algorithm(const json& value) {
    if (value.is_number_integer()) {
        const int algorithm = value.get<int>();
        if (algorithm == 2) {
            return core::PathAlgo::AStarSimple;
        }
        if (algorithm == 3) {
            return core::PathAlgo::DStarBasic;
        }
        return core::PathAlgo::Default;
    }

    const std::string name = value.get<std::string>();
    if (name == "astar" || name == "AStarSimple") {
        return core::PathAlgo::AStarSimple;
    }
    if (name == "dstar" || name == "DStarBasic") {
        return core::PathAlgo::DStarBasic;
    }
    return core::PathAlgo::Default;
}

core::ScenarioConfig parse_scenario(const json& value) {
    core::ScenarioConfig config;
    if (!value.is_object()) {
        return config;
    }

    if (value.contains("mode")) {
        const std::string mode = value.at("mode").get<std::string>();
        config.mode = (mode == "realtime") ? core::SimulationMode::Realtime : core::SimulationMode::Custom;
    }
    if (value.contains("speedMultiplier")) {
        config.speedMultiplier = value.at("speedMultiplier").get<double>();
    }
    if (value.contains("realtimeParkChance")) {
        config.realtimeParkChance = value.at("realtimeParkChance").get<int>();
    }
    if (value.contains("realtimeExitChance")) {
        config.realtimeExitChance = value.at("realtimeExitChance").get<int>();
    }
    if (value.contains("phases") && value.at("phases").is_array()) {
        config.phases.clear();
        for (const json& phase : value.at("phases")) {
            if (!phase.is_object()) {
                continue;
            }
            core::PhaseConfig item;
            const std::string type = phase.value("type", std::string("park"));
            item.type = (type == "exit") ? core::PhaseType::Exit : core::PhaseType::Park;
            item.taskCount = phase.value("taskCount", 1);
            config.phases.push_back(item);
        }
    }
    if (config.mode == core::SimulationMode::Custom && config.phases.empty()) {
        config.phases.push_back({core::PhaseType::Park, 1});
    }
    return config;
}

core::RenderQueryOptions parse_render_options(const json& request, bool paused) {
    core::RenderQueryOptions options;
    options.paused = paused;
    if (!request.contains("options") || !request.at("options").is_object()) {
        return options;
    }

    const json& raw = request.at("options");
    options.paused = raw.value("paused", paused);
    options.logsTail = raw.value("logsTail", true);
    options.maxLogEntries = raw.value("maxLogEntries", static_cast<std::size_t>(5));
    options.plannerOverlay = raw.value("plannerOverlay", false);
    return options;
}

json to_json_value(const core::GridCoord& coord) {
    return json{{"x", coord.x}, {"y", coord.y}};
}

json to_json_value(const core::HomeCellSnapshot& home) {
    return json{
        {"agentId", home.agentId},
        {"symbol", std::string(1, home.symbol)},
        {"position", to_json_value(home.position)},
    };
}

json to_json_value(const core::GoalRenderState& goal) {
    return json{
        {"position", to_json_value(goal.position)},
        {"isGoal", goal.isGoal},
        {"isParked", goal.isParked},
        {"reservedByAgent", goal.reservedByAgent},
    };
}

json to_json_value(const core::StructuredLogEntry& entry) {
    return json{
        {"seq", entry.seq},
        {"step", entry.step},
        {"category", entry.category},
        {"level", entry.level},
        {"text", entry.text},
    };
}

json to_json_value(const core::OverlayWaitEdge& edge) {
    return json{
        {"fromAgentId", edge.fromAgentId},
        {"toAgentId", edge.toAgentId},
        {"timeOffset", edge.timeOffset},
        {"cause", edge.cause},
        {"from", to_json_value(edge.from)},
        {"to", to_json_value(edge.to)},
    };
}

json to_json_value(const core::OverlayPlannedPath& path) {
    json cells = json::array();
    for (const auto& cell : path.cells) {
        cells.push_back(to_json_value(cell));
    }
    return json{
        {"agentId", path.agentId},
        {"cells", std::move(cells)},
    };
}

json to_json_value(const core::PlannerOverlaySnapshot& overlay) {
    json wait_edges = json::array();
    for (const auto& edge : overlay.waitEdges) {
        wait_edges.push_back(to_json_value(edge));
    }

    json planned_paths = json::array();
    for (const auto& path : overlay.plannedPaths) {
        planned_paths.push_back(to_json_value(path));
    }

    json cbs_paths = json::array();
    for (const auto& path : overlay.cbsPaths) {
        cbs_paths.push_back(to_json_value(path));
    }

    return json{
        {"available", overlay.available},
        {"algorithm", to_string(overlay.algorithm)},
        {"horizon", overlay.horizon},
        {"waitEdgeCount", overlay.waitEdgeCount},
        {"leaderAgentId", overlay.leaderAgentId},
        {"usedCbs", overlay.usedCbs},
        {"sccParticipantAgentIds", overlay.sccParticipantAgentIds},
        {"yieldAgentIds", overlay.yieldAgentIds},
        {"pullOverAgentIds", overlay.pullOverAgentIds},
        {"waitEdges", std::move(wait_edges)},
        {"plannedPaths", std::move(planned_paths)},
        {"cbsPaths", std::move(cbs_paths)},
    };
}

json to_json_value(const core::HudSnapshot& hud) {
    return json{
        {"mapId", hud.mapId},
        {"algorithm", to_string(hud.algorithm)},
        {"mode", to_string(hud.mode)},
        {"step", hud.step},
        {"paused", hud.paused},
        {"speedMultiplier", hud.speedMultiplier},
        {"currentPhaseIndex", hud.currentPhaseIndex},
        {"totalPhases", hud.totalPhases},
        {"currentPhaseType", to_string(hud.currentPhaseType)},
        {"phaseTaskTarget", hud.phaseTaskTarget},
        {"phaseTasksCompleted", hud.phaseTasksCompleted},
        {"phaseRemainingTasks", hud.phaseRemainingTasks},
        {"queuedTaskCount", hud.queuedTaskCount},
        {"inFlightTaskCount", hud.inFlightTaskCount},
        {"outstandingTaskCount", hud.outstandingTaskCount},
        {"parkedCars", hud.parkedCars},
        {"totalGoalCount", hud.totalGoalCount},
        {"lastStepCpuTimeMs", hud.lastStepCpuTimeMs},
        {"avgCpuTimeMs", hud.avgCpuTimeMs},
        {"totalCpuTimeMs", hud.totalCpuTimeMs},
        {"plannerWaitEdges", hud.plannerWaitEdges},
        {"plannerSccCount", hud.plannerSccCount},
        {"plannerCbsSucceeded", hud.plannerCbsSucceeded},
        {"plannerCbsExpansions", hud.plannerCbsExpansions},
        {"whcaHorizon", hud.whcaHorizon},
    };
}

json to_json_value(const core::AgentRenderState& agent) {
    return json{
        {"id", agent.id},
        {"symbol", std::string(1, agent.symbol)},
        {"state", to_string(agent.state)},
        {"isActive", agent.isActive},
        {"position", to_json_value(agent.position)},
        {"lastPosition", to_json_value(agent.lastPosition)},
        {"home", to_json_value(agent.home)},
        {"goal", to_json_value(agent.goal)},
        {"totalDistanceTraveled", agent.totalDistanceTraveled},
        {"chargeTimer", agent.chargeTimer},
        {"actionTimer", agent.actionTimer},
        {"rotationWait", agent.rotationWait},
        {"stuckSteps", agent.stuckSteps},
        {"oscillationSteps", agent.oscillationSteps},
    };
}

json to_json_value(const core::StaticSceneSnapshot& scene) {
    json goals = json::array();
    for (const auto& goal : scene.goalCells) {
        goals.push_back(to_json_value(goal));
    }

    json chargers = json::array();
    for (const auto& charger : scene.chargerCells) {
        chargers.push_back(to_json_value(charger));
    }

    json homes = json::array();
    for (const auto& home : scene.homeCells) {
        homes.push_back(to_json_value(home));
    }

    return json{
        {"sessionId", scene.sessionId},
        {"sceneVersion", scene.sceneVersion},
        {"mapId", scene.mapId},
        {"width", scene.width},
        {"height", scene.height},
        {"baseTiles", scene.baseTiles},
        {"goalCells", std::move(goals)},
        {"chargerCells", std::move(chargers)},
        {"homeCells", std::move(homes)},
    };
}

json to_json_value(const core::RenderFrameSnapshot& frame) {
    json agents = json::array();
    for (const auto& agent : frame.agents) {
        agents.push_back(to_json_value(agent));
    }

    json goals = json::array();
    for (const auto& goal : frame.goalStates) {
        goals.push_back(to_json_value(goal));
    }

    json logs = json::array();
    for (const auto& log : frame.logsTail) {
        logs.push_back(to_json_value(log));
    }

    return json{
        {"sessionId", frame.sessionId},
        {"sceneVersion", frame.sceneVersion},
        {"frameId", frame.frameId},
        {"lastLogSeq", frame.lastLogSeq},
        {"hud", to_json_value(frame.hud)},
        {"agents", std::move(agents)},
        {"goalStates", std::move(goals)},
        {"logsTail", std::move(logs)},
        {"plannerOverlay", to_json_value(frame.plannerOverlay)},
    };
}

json to_json_value(const core::RenderFrameDelta& delta) {
    json agents = json::array();
    for (const auto& agent : delta.agentUpdates) {
        agents.push_back(to_json_value(agent));
    }

    json goals = json::array();
    for (const auto& goal : delta.goalStateChanges) {
        goals.push_back(to_json_value(goal));
    }

    json logs = json::array();
    for (const auto& log : delta.newLogs) {
        logs.push_back(to_json_value(log));
    }

    return json{
        {"sessionId", delta.sessionId},
        {"sceneVersion", delta.sceneVersion},
        {"fromFrameId", delta.fromFrameId},
        {"toFrameId", delta.toFrameId},
        {"lastLogSeq", delta.lastLogSeq},
        {"requiresFullResync", delta.requiresFullResync},
        {"hudChanged", delta.hudChanged},
        {"overlayChanged", delta.overlayChanged},
        {"hud", to_json_value(delta.hud)},
        {"agentUpdates", std::move(agents)},
        {"goalStateChanges", std::move(goals)},
        {"newLogs", std::move(logs)},
        {"plannerOverlay", to_json_value(delta.plannerOverlay)},
    };
}

json ack_response(std::string_view command) {
    return json{{"type", "response"}, {"command", command}, {"ok", true}};
}

}  // namespace

RenderIpcServer::RenderIpcServer() {
    engine_.setSuppressOutput(true);
}

std::vector<json> RenderIpcServer::processRequest(const json& request) {
    const std::string command = request.value("command", std::string{});
    if (command.empty()) {
        throw std::runtime_error("missing command");
    }

    std::vector<json> responses;

    if (command == "loadMap") {
        engine_.loadMap(request.at("mapId").get<int>());
        responses.push_back(ack_response(command));
    } else if (command == "setAlgorithm") {
        engine_.setAlgorithm(parse_algorithm(request.at("algorithm")));
        responses.push_back(ack_response(command));
    } else if (command == "configureScenario") {
        engine_.configureScenario(parse_scenario(request.at("scenario")));
        responses.push_back(ack_response(command));
    } else if (command == "step") {
        engine_.step();
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"complete", engine_.isComplete()},
            {"frameId", engine_.snapshotRenderFrame(core::RenderQueryOptions{paused_, false, 0, false}).frameId},
        });
        if (subscribed_) {
            responses.push_back(buildFrameEvent());
        }
    } else if (command == "run") {
        const int max_steps = request.value("maxSteps", -1);
        int executed = 0;
        while (!paused_ && !engine_.isComplete() && (max_steps < 0 || executed < max_steps)) {
            engine_.step();
            ++executed;
        }
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"executedSteps", executed},
            {"complete", engine_.isComplete()},
            {"frameId", engine_.snapshotRenderFrame(core::RenderQueryOptions{paused_, false, 0, false}).frameId},
        });
        if (subscribed_ && executed > 0) {
            responses.push_back(buildFrameEvent());
        }
    } else if (command == "pause") {
        paused_ = true;
        responses.push_back(ack_response(command));
    } else if (command == "resume") {
        paused_ = false;
        responses.push_back(ack_response(command));
    } else if (command == "getStaticScene") {
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"scene", to_json_value(engine_.snapshotStaticScene())},
        });
    } else if (command == "getFrame") {
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"frame", to_json_value(engine_.snapshotRenderFrame(parse_render_options(request, paused_)))},
        });
    } else if (command == "getDelta") {
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"delta", to_json_value(engine_.snapshotRenderDelta(request.at("sinceFrameId").get<std::uint64_t>(), parse_render_options(request, paused_)))},
        });
    } else if (command == "subscribeFrameDelta") {
        subscribed_ = request.value("enabled", true);
        responses.push_back(json{
            {"type", "response"},
            {"command", command},
            {"ok", true},
            {"subscribed", subscribed_},
        });
    } else {
        throw std::runtime_error("unknown command: " + command);
    }

    return responses;
}

json RenderIpcServer::buildFrameEvent() {
    core::RenderQueryOptions options;
    options.paused = paused_;
    options.logsTail = false;
    const core::RenderFrameSnapshot frame = engine_.snapshotRenderFrame(options);
    return json{
        {"type", "event"},
        {"event", "frameDeltaAvailable"},
        {"sessionId", frame.sessionId},
        {"sceneVersion", frame.sceneVersion},
        {"frameId", frame.frameId},
    };
}

void RenderIpcServer::run(std::istream& input, std::ostream& output) {
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        try {
            const json request = json::parse(line);
            const std::vector<json> responses = processRequest(request);
            for (const json& response : responses) {
                output << response.dump() << '\n';
            }
        } catch (const std::exception& error) {
            output << json{
                {"type", "response"},
                {"ok", false},
                {"error", error.what()},
            }.dump() << '\n';
        }
    }
}

}  // namespace agv::ipc
