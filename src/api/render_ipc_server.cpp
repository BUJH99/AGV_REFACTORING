#include "agv/render_ipc_server.hpp"
#include "agv/internal/engine_internal.hpp"
#include "agv/internal/launch_ui_metadata.hpp"

#include <algorithm>
#include <chrono>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace agv::ipc {

namespace {

using nlohmann::json;

constexpr int kProtocolVersion = 1;

class ProtocolError final : public std::runtime_error {
public:
    ProtocolError(std::string code, std::string message)
        : std::runtime_error(std::move(message)),
          code_(std::move(code)) {}

    const std::string& code() const { return code_; }

private:
    std::string code_;
};

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

std::string to_string(core::AgentWaitReason reason) {
    switch (reason) {
        case core::AgentWaitReason::Idle:
            return "idle";
        case core::AgentWaitReason::Charging:
            return "charging";
        case core::AgentWaitReason::GoalAction:
            return "goal_action";
        case core::AgentWaitReason::Rotation:
            return "rotation";
        case core::AgentWaitReason::BlockedByStationary:
            return "blocked_by_stationary";
        case core::AgentWaitReason::PriorityYield:
            return "priority_yield";
        case core::AgentWaitReason::Stuck:
            return "stuck";
        case core::AgentWaitReason::Oscillating:
            return "oscillating";
        case core::AgentWaitReason::None:
        default:
            return "none";
    }
}

json request_id_or_null(const json& request) {
    return (request.is_object() && request.contains("requestId")) ? request.at("requestId") : json(nullptr);
}

void require_protocol_request(const json& request) {
    if (!request.is_object()) {
        throw ProtocolError("invalid_request", "Request must be a JSON object.");
    }
    if (!request.contains("protocolVersion")) {
        throw ProtocolError("invalid_request", "Missing protocolVersion.");
    }
    if (request.at("protocolVersion").get<int>() != kProtocolVersion) {
        throw ProtocolError("invalid_request", "Unsupported protocolVersion.");
    }
    if (!request.contains("requestId")) {
        throw ProtocolError("invalid_request", "Missing requestId.");
    }
    if (!request.contains("command") || !request.at("command").is_string()) {
        throw ProtocolError("invalid_request", "Missing command.");
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
    return config;
}

core::LaunchConfig parse_launch_config(const json& value) {
    if (!value.is_object()) {
        throw ProtocolError("invalid_request", "launchConfig must be an object.");
    }

    core::LaunchConfig config;
    config.seed = value.value("seed", static_cast<std::uint32_t>(0));
    config.mapId = value.value("mapId", 1);
    if (value.contains("algorithm")) {
        config.algorithm = parse_algorithm(value.at("algorithm"));
    }
    if (value.contains("scenario")) {
        config.scenario = parse_scenario(value.at("scenario"));
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
    options.maxLogEntries = raw.value("maxLogEntries", static_cast<std::size_t>(RENDER_LOG_TAIL_LINES));
    options.plannerOverlay = raw.value("plannerOverlay", false);
    return options;
}

json to_json_value(const core::LaunchConfig& config) {
    json phases = json::array();
    for (const auto& phase : config.scenario.phases) {
        phases.push_back({
            {"type", to_string(phase.type)},
            {"taskCount", phase.taskCount},
        });
    }

    return json{
        {"seed", config.seed},
        {"mapId", config.mapId},
        {"algorithm", to_string(config.algorithm)},
        {"scenario", {
            {"mode", to_string(config.scenario.mode)},
            {"speedMultiplier", config.scenario.speedMultiplier},
            {"realtimeParkChance", config.scenario.realtimeParkChance},
            {"realtimeExitChance", config.scenario.realtimeExitChance},
            {"phases", std::move(phases)},
        }},
    };
}

json to_json_value(const core::ValidationIssue& issue) {
    return json{
        {"field", issue.field},
        {"code", issue.code},
        {"message", issue.message},
    };
}

json to_json_value(const agv::internal::launch_ui::MapOption& option) {
    return json{
        {"id", option.id},
        {"key", option.key},
        {"label", option.label},
        {"description", option.description},
        {"capacity", option.capacity},
    };
}

json to_json_value(const agv::internal::launch_ui::AlgorithmOption& option) {
    return json{
        {"id", option.key},
        {"label", option.label},
        {"description", option.description},
    };
}

json to_json_value(const agv::internal::launch_ui::ModeOption& option) {
    return json{
        {"id", option.key},
        {"label", option.label},
        {"description", option.description},
    };
}

json to_json_value(const agv::internal::launch_ui::WizardStepOption& option) {
    return json{
        {"id", option.key},
        {"title", option.title},
        {"description", option.description},
    };
}

json to_json_value(const core::SessionDescriptor& session) {
    return json{
        {"sessionId", session.sessionId},
        {"sceneVersion", session.sceneVersion},
        {"frameId", session.frameId},
        {"launchConfig", to_json_value(session.launchConfig)},
    };
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
    json value{
        {"seq", entry.seq},
        {"step", entry.step},
        {"frameId", entry.frameId},
        {"category", entry.category},
        {"level", entry.level},
        {"text", entry.text},
    };
    value["agentId"] = entry.agentId ? json(*entry.agentId) : json(nullptr);
    value["phaseIndex"] = entry.phaseIndex ? json(*entry.phaseIndex) : json(nullptr);
    return value;
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
        {"readyIdleAgentCount", hud.readyIdleAgentCount},
        {"activeGoalActionCount", hud.activeGoalActionCount},
        {"waitingAgentCount", hud.waitingAgentCount},
        {"stuckAgentCount", hud.stuckAgentCount},
        {"oscillatingAgentCount", hud.oscillatingAgentCount},
        {"noMovementStreak", hud.noMovementStreak},
        {"maxNoMovementStreak", hud.maxNoMovementStreak},
        {"lastTaskCompletionStep", hud.lastTaskCompletionStep},
        {"stepsSinceLastTaskCompletion", hud.stepsSinceLastTaskCompletion},
        {"oldestQueuedRequestAge", hud.oldestQueuedRequestAge},
        {"parkedCars", hud.parkedCars},
        {"totalGoalCount", hud.totalGoalCount},
        {"lastStepCpuTimeMs", hud.lastStepCpuTimeMs},
        {"lastPlanningTimeMs", hud.lastPlanningTimeMs},
        {"avgCpuTimeMs", hud.avgCpuTimeMs},
        {"totalCpuTimeMs", hud.totalCpuTimeMs},
        {"plannedMoveCount", hud.plannedMoveCount},
        {"postRotationMoveCount", hud.postRotationMoveCount},
        {"postBlockerMoveCount", hud.postBlockerMoveCount},
        {"finalMoveCount", hud.finalMoveCount},
        {"rotationCanceledCount", hud.rotationCanceledCount},
        {"blockerCanceledCount", hud.blockerCanceledCount},
        {"orderCanceledCount", hud.orderCanceledCount},
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
        {"taskActive", agent.taskActive},
        {"taskAgeSteps", agent.taskAgeSteps},
        {"taskDistance", agent.taskDistance},
        {"taskTurns", agent.taskTurns},
        {"goalActionInProgress", agent.goalActionInProgress},
        {"movedLastStep", agent.movedLastStep},
        {"waitReason", to_string(agent.waitReason)},
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

json to_json_value(const core::DistributionSummary& summary) {
    return json{
        {"min", summary.min},
        {"avg", summary.avg},
        {"max", summary.max},
        {"stddev", summary.stddev},
        {"coefficientOfVariation", summary.coefficientOfVariation},
        {"minMaxRatio", summary.minMaxRatio},
    };
}

json to_json_value(const core::AgentFairnessSnapshot& fairness) {
    return json{
        {"id", fairness.id},
        {"symbol", std::string(1, fairness.symbol)},
        {"tasksCompleted", fairness.tasksCompleted},
        {"distanceCells", fairness.distanceCells},
        {"idleSteps", fairness.idleSteps},
    };
}

json to_json_value(const core::MetricsSnapshot& metrics) {
    json fairness = json::array();
    for (const auto& item : metrics.agentFairnessBreakdown) {
        fairness.push_back(to_json_value(item));
    }

    return json{
        {"seed", metrics.seed},
        {"mapId", metrics.mapId},
        {"algorithm", to_string(metrics.algorithm)},
        {"mode", to_string(metrics.mode)},
        {"activeAgents", metrics.activeAgents},
        {"recordedSteps", metrics.recordedSteps},
        {"tasksCompletedTotal", metrics.tasksCompletedTotal},
        {"throughput", metrics.throughput},
        {"tasksPerAgent", metrics.tasksPerAgent},
        {"totalMovementCost", metrics.totalMovementCost},
        {"deadlockCount", metrics.deadlockCount},
        {"totalCpuTimeMs", metrics.totalCpuTimeMs},
        {"avgCpuTimeMs", metrics.avgCpuTimeMs},
        {"maxStepCpuTimeMs", metrics.maxStepCpuTimeMs},
        {"avgCpuTimePerTaskMs", metrics.avgCpuTimePerTaskMs},
        {"tasksPerCpuSecond", metrics.tasksPerCpuSecond},
        {"totalPlanningTimeMs", metrics.totalPlanningTimeMs},
        {"avgPlanningTimeMs", metrics.avgPlanningTimeMs},
        {"maxPlanningTimeMs", metrics.maxPlanningTimeMs},
        {"planningCpuShare", metrics.planningCpuShare},
        {"avgPlanningTimePerTaskMs", metrics.avgPlanningTimePerTaskMs},
        {"tasksPerPlanningSecond", metrics.tasksPerPlanningSecond},
        {"memoryUsageSumKb", metrics.memoryUsageSumKb},
        {"avgMemoryUsageKb", metrics.avgMemoryUsageKb},
        {"memoryUsagePeakKb", metrics.memoryUsagePeakKb},
        {"avgMemoryUsagePerAgentKb", metrics.avgMemoryUsagePerAgentKb},
        {"algoNodesExpandedTotal", metrics.algoNodesExpandedTotal},
        {"algoHeapMovesTotal", metrics.algoHeapMovesTotal},
        {"algoGeneratedNodesTotal", metrics.algoGeneratedNodesTotal},
        {"algoValidExpansionsTotal", metrics.algoValidExpansionsTotal},
        {"validExpansionRatio", metrics.validExpansionRatio},
        {"avgNodesExpandedPerStep", metrics.avgNodesExpandedPerStep},
        {"avgNodesExpandedPerTask", metrics.avgNodesExpandedPerTask},
        {"nodesExpandedPerPlanningMs", metrics.nodesExpandedPerPlanningMs},
        {"heapMovesPerNodeExpanded", metrics.heapMovesPerNodeExpanded},
        {"requestsCreatedTotal", metrics.requestsCreatedTotal},
        {"requestWaitTicksSum", metrics.requestWaitTicksSum},
        {"avgRequestWaitTicks", metrics.avgRequestWaitTicks},
        {"avgMovementPerTask", metrics.avgMovementPerTask},
        {"nonTaskMovementCost", metrics.nonTaskMovementCost},
        {"taskMovementCoverageRatio", metrics.taskMovementCoverageRatio},
        {"nonTaskMovementRatio", metrics.nonTaskMovementRatio},
        {"avgTaskDistance", metrics.avgTaskDistance},
        {"avgTaskTurns", metrics.avgTaskTurns},
        {"avgTaskSteps", metrics.avgTaskSteps},
        {"avgTaskStepsPerCell", metrics.avgTaskStepsPerCell},
        {"avgTaskTurnsPer100Cells", metrics.avgTaskTurnsPer100Cells},
        {"stepsWithMovement", metrics.stepsWithMovement},
        {"stallStepCount", metrics.stallStepCount},
        {"movementStepRatio", metrics.movementStepRatio},
        {"stallStepRatio", metrics.stallStepRatio},
        {"maxNoMovementStreak", metrics.maxNoMovementStreak},
        {"lastTaskCompletionStep", metrics.lastTaskCompletionStep},
        {"stepsSinceLastTaskCompletion", metrics.stepsSinceLastTaskCompletion},
        {"queuedTaskCount", metrics.queuedTaskCount},
        {"inFlightTaskCount", metrics.inFlightTaskCount},
        {"outstandingTaskCount", metrics.outstandingTaskCount},
        {"avgOutstandingTaskCount", metrics.avgOutstandingTaskCount},
        {"peakOutstandingTaskCount", metrics.peakOutstandingTaskCount},
        {"avgOutstandingTasksPerAgent", metrics.avgOutstandingTasksPerAgent},
        {"peakOutstandingTasksPerAgent", metrics.peakOutstandingTasksPerAgent},
        {"oldestQueuedRequestAge", metrics.oldestQueuedRequestAge},
        {"avgOldestQueuedRequestAge", metrics.avgOldestQueuedRequestAge},
        {"peakOldestQueuedRequestAge", metrics.peakOldestQueuedRequestAge},
        {"plannerWaitEdgesSum", metrics.plannerWaitEdgesSum},
        {"plannerConflictCycleTotal", metrics.plannerConflictCycleTotal},
        {"plannerWaitEdgeStepCount", metrics.plannerWaitEdgeStepCount},
        {"plannerCycleStepCount", metrics.plannerCycleStepCount},
        {"plannerWaitEdgesPerStep", metrics.plannerWaitEdgesPerStep},
        {"plannerWaitEdgesPerConflictStep", metrics.plannerWaitEdgesPerConflictStep},
        {"plannerCycleStepRatio", metrics.plannerCycleStepRatio},
        {"plannerCbsAttemptCount", metrics.plannerCbsAttemptCount},
        {"plannerCbsSuccessCount", metrics.plannerCbsSuccessCount},
        {"plannerCbsFailureCount", metrics.plannerCbsFailureCount},
        {"plannerCbsAttemptRate", metrics.plannerCbsAttemptRate},
        {"plannerCbsSuccessRate", metrics.plannerCbsSuccessRate},
        {"plannerCbsFailureRate", metrics.plannerCbsFailureRate},
        {"tasksPerAgentSpread", to_json_value(metrics.tasksPerAgentSpread)},
        {"distancePerAgentSpread", to_json_value(metrics.distancePerAgentSpread)},
        {"idleStepsPerAgentSpread", to_json_value(metrics.idleStepsPerAgentSpread)},
        {"agentFairnessBreakdown", std::move(fairness)},
        {"remainingParkedVehicles", metrics.remainingParkedVehicles},
    };
}

json to_json_value(const core::AgentDebugSnapshot& agent) {
    return json{
        {"id", agent.id},
        {"symbol", std::string(1, agent.symbol)},
        {"state", to_string(agent.state)},
        {"isActive", agent.isActive},
        {"posX", agent.posX},
        {"posY", agent.posY},
        {"lastPosX", agent.lastPosX},
        {"lastPosY", agent.lastPosY},
        {"homeX", agent.homeX},
        {"homeY", agent.homeY},
        {"goalX", agent.goalX},
        {"goalY", agent.goalY},
        {"totalDistanceTraveled", agent.totalDistanceTraveled},
        {"chargeTimer", agent.chargeTimer},
        {"actionTimer", agent.actionTimer},
        {"rotationWait", agent.rotationWait},
        {"taskActive", agent.taskActive},
        {"taskAgeSteps", agent.taskAgeSteps},
        {"taskDistance", agent.taskDistance},
        {"taskTurns", agent.taskTurns},
        {"stuckSteps", agent.stuckSteps},
        {"oscillationSteps", agent.oscillationSteps},
    };
}

json to_json_value(const core::RuntimeDebugSnapshot& runtime) {
    return json{
        {"currentStep", runtime.currentStep},
        {"hasActivePhase", runtime.hasActivePhase},
        {"inCleanupRegion", runtime.inCleanupRegion},
        {"currentPhaseIndex", runtime.currentPhaseIndex},
        {"totalPhases", runtime.totalPhases},
        {"currentPhaseType", to_string(runtime.currentPhaseType)},
        {"phaseTaskTarget", runtime.phaseTaskTarget},
        {"phaseTasksCompleted", runtime.phaseTasksCompleted},
        {"phaseRemainingTasks", runtime.phaseRemainingTasks},
        {"queuedTaskCount", runtime.queuedTaskCount},
        {"inFlightTaskCount", runtime.inFlightTaskCount},
        {"outstandingTaskCount", runtime.outstandingTaskCount},
        {"readyIdleAgentCount", runtime.readyIdleAgentCount},
        {"activeGoalActionCount", runtime.activeGoalActionCount},
        {"lastActiveAgentCount", runtime.lastActiveAgentCount},
        {"lastWaitingAgentCount", runtime.lastWaitingAgentCount},
        {"lastStuckAgentCount", runtime.lastStuckAgentCount},
        {"lastOscillatingAgentCount", runtime.lastOscillatingAgentCount},
        {"noMovementStreak", runtime.noMovementStreak},
        {"maxNoMovementStreak", runtime.maxNoMovementStreak},
        {"lastTaskCompletionStep", runtime.lastTaskCompletionStep},
        {"stepsSinceLastTaskCompletion", runtime.stepsSinceLastTaskCompletion},
        {"oldestQueuedRequestAge", runtime.oldestQueuedRequestAge},
        {"lastStepCpuTimeMs", runtime.lastStepCpuTimeMs},
        {"lastPlanningTimeMs", runtime.lastPlanningTimeMs},
        {"plannedMoveCount", runtime.plannedMoveCount},
        {"postRotationMoveCount", runtime.postRotationMoveCount},
        {"postBlockerMoveCount", runtime.postBlockerMoveCount},
        {"finalMoveCount", runtime.finalMoveCount},
        {"rotationCanceledAgentIds", runtime.rotationCanceledAgentIds},
        {"blockerCanceledAgentIds", runtime.blockerCanceledAgentIds},
        {"orderCanceledAgentIds", runtime.orderCanceledAgentIds},
        {"plannerWaitEdges", runtime.plannerWaitEdges},
        {"plannerSccCount", runtime.plannerSccCount},
        {"plannerCbsSucceeded", runtime.plannerCbsSucceeded},
        {"plannerCbsExpansions", runtime.plannerCbsExpansions},
        {"whcaHorizon", runtime.whcaHorizon},
    };
}

json to_json_value(const core::DeadlockSnapshot& deadlock) {
    return json{
        {"hasEvent", deadlock.hasEvent},
        {"step", deadlock.step},
        {"deadlockCount", deadlock.deadlockCount},
        {"phaseIndex", deadlock.phaseIndex},
        {"phaseTaskTarget", deadlock.phaseTaskTarget},
        {"phaseTasksCompleted", deadlock.phaseTasksCompleted},
        {"pendingTaskCount", deadlock.pendingTaskCount},
        {"activeAgentCount", deadlock.activeAgentCount},
        {"waitingAgentCount", deadlock.waitingAgentCount},
        {"stuckAgentCount", deadlock.stuckAgentCount},
        {"plannerWaitEdges", deadlock.plannerWaitEdges},
        {"plannerSccCount", deadlock.plannerSccCount},
        {"plannerCbsSucceeded", deadlock.plannerCbsSucceeded},
        {"plannerCbsExpansions", deadlock.plannerCbsExpansions},
        {"whcaHorizon", deadlock.whcaHorizon},
        {"plannedMoveCount", deadlock.plannedMoveCount},
        {"postRotationMoveCount", deadlock.postRotationMoveCount},
        {"postBlockerMoveCount", deadlock.postBlockerMoveCount},
        {"finalMoveCount", deadlock.finalMoveCount},
        {"rotationCanceledAgentIds", deadlock.rotationCanceledAgentIds},
        {"blockerCanceledAgentIds", deadlock.blockerCanceledAgentIds},
        {"orderCanceledAgentIds", deadlock.orderCanceledAgentIds},
        {"participantAgentIds", deadlock.participantAgentIds},
        {"reason", deadlock.reason},
    };
}

json to_json_value(const core::DebugSnapshot& snapshot) {
    json agents = json::array();
    for (const auto& agent : snapshot.agents) {
        agents.push_back(to_json_value(agent));
    }

    return json{
        {"metrics", to_json_value(snapshot.metrics)},
        {"runtime", to_json_value(snapshot.runtime)},
        {"frameText", snapshot.frame.text},
        {"recentLogs", snapshot.recentLogs},
        {"agents", std::move(agents)},
        {"deadlock", to_json_value(snapshot.deadlock)},
    };
}

json base_response(const json& request, std::string_view command, bool ok) {
    return json{
        {"type", "response"},
        {"protocolVersion", kProtocolVersion},
        {"requestId", request_id_or_null(request)},
        {"command", command},
        {"ok", ok},
    };
}

json base_error_response(const json& request, std::string error_code, std::string message) {
    const std::string command = (request.is_object() && request.contains("command") && request.at("command").is_string())
        ? request.at("command").get<std::string>()
        : std::string{};
    json response = base_response(request, command, false);
    response["errorCode"] = std::move(error_code);
    response["message"] = std::move(message);
    return response;
}

}  // namespace

RenderIpcServer::RenderIpcServer() {
    engine_.setTerminalOutputEnabled(false);
}

std::vector<json> RenderIpcServer::processRequest(const json& request) {
    require_protocol_request(request);
    const std::string command = request.at("command").get<std::string>();
    std::vector<json> responses;

    auto require_session = [&]() -> std::uint64_t {
        if (!request.contains("sessionId")) {
            throw ProtocolError("session_not_started", "Missing sessionId.");
        }
        if (current_session_id_ == 0) {
            throw ProtocolError("session_not_started", "No active session.");
        }
        const std::uint64_t session_id = request.at("sessionId").get<std::uint64_t>();
        if (session_id != current_session_id_) {
            throw ProtocolError("session_stale", "The provided sessionId is stale.");
        }
        return session_id;
    };

    if (command == "getCapabilities") {
        json maps = json::array();
        for (const auto& option : agv::internal::launch_ui::map_options()) {
            maps.push_back(to_json_value(option));
        }

        json algorithms = json::array();
        for (const auto& option : agv::internal::launch_ui::algorithm_options()) {
            algorithms.push_back(to_json_value(option));
        }

        json modes = json::array();
        for (const auto& option : agv::internal::launch_ui::mode_options()) {
            modes.push_back(to_json_value(option));
        }

        json wizard_steps = json::array();
        for (const auto& option : agv::internal::launch_ui::wizard_steps()) {
            wizard_steps.push_back(to_json_value(option));
        }

        const auto now = std::chrono::system_clock::now();
        const auto seed = static_cast<std::uint32_t>(std::chrono::system_clock::to_time_t(now));
        const core::LaunchConfig recommended = agv::internal::launch_ui::recommended_launch_config(seed);
        json response = base_response(request, command, true);
        response["capabilities"] = {
            {"platform", "windows"},
            {"protocolVersion", kProtocolVersion},
            {"commands", json::array({
                "getCapabilities",
                "startSession",
                "getStaticScene",
                "getFrame",
                "getDelta",
                "getLogs",
                "runBurst",
                "setPaused",
                "setSpeedMultiplier",
                "getMetrics",
                "getDebugSnapshot",
                "subscribeFrameDelta",
                "shutdown",
                "loadMap",
                "setAlgorithm",
                "configureScenario",
                "step",
                "run",
                "pause",
                "resume",
            })},
            {"mapIdRange", {{"min", 1}, {"max", 7}}},
            {"recommendedLaunchConfig", to_json_value(recommended)},
            {"recommendedPreset", {
                {"id", "recommended"},
                {"label", "Recommended"},
                {"description", "Balanced starter setup for quick validation runs."},
                {"seedStrategy", "timestamp_seconds"},
            }},
            {"wizardFlow", json::array({"map", "algorithm", "mode", "scenario", "speed", "seed", "summary"})},
            {"wizardSteps", std::move(wizard_steps)},
            {"maps", std::move(maps)},
            {"algorithms", std::move(algorithms)},
            {"modes", std::move(modes)},
            {"launchSchema", {
                {"seed", {
                    {"type", "uint32"},
                    {"min", 0},
                    {"max", 4294967295ull},
                    {"note", "Same seed and config reproduce the same random run."},
                    {"defaultStrategy", "timestamp_seconds"},
                }},
                {"speedMultiplier", {
                    {"type", "double"},
                    {"min", 0.0},
                    {"max", static_cast<double>(MAX_SPEED_MULTIPLIER)},
                    {"default", 0.0},
                    {"note", "0.0 keeps the simulation at full speed without deliberate sleep."},
                }},
                {"customPhases", {
                    {"minCount", 0},
                    {"maxCount", MAX_PHASES},
                    {"emptyBehavior", "normalize_to_single_park_x1"},
                    {"taskCountUsesSelectedMapCapacity", true},
                    {"taskCountCapacityField", "maps[].capacity"},
                }},
                {"realtimeChances", {
                    {"min", 0},
                    {"max", 100},
                    {"sumMax", 100},
                }},
                {"persistence", {
                    {"lastUsedVersion", 1},
                    {"pathHint", "%LOCALAPPDATA%\\\\AGVRefactor\\\\last_launch.json"},
                }},
            }},
            {"maxSpeedMultiplier", static_cast<double>(MAX_SPEED_MULTIPLIER)},
            {"supportsFrameDeltaEvent", true},
            {"supportsStructuredLogs", true},
        };
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "startSession") {
        if (!request.contains("launchConfig")) {
            throw ProtocolError("invalid_request", "Missing launchConfig.");
        }
        const core::LaunchConfig launch_config = parse_launch_config(request.at("launchConfig"));
        const core::ValidationResult validation = core::validateLaunchConfig(launch_config);
        if (!validation.ok()) {
            json response = base_error_response(request, "validation_failed", "Launch config validation failed.");
            json issues = json::array();
            for (const auto& issue : validation.errors) {
                issues.push_back(to_json_value(issue));
            }
            response["issues"] = std::move(issues);
            responses.push_back(std::move(response));
            return responses;
        }

        engine_.setTerminalOutputEnabled(false);
        engine_.configureLaunch(validation.normalizedConfig);
        const core::SessionDescriptor session = engine_.startConfiguredSession();
        current_session_id_ = session.sessionId;
        paused_ = false;

        json response = base_response(request, command, true);
        response["session"] = to_json_value(session);
        if (!validation.warnings.empty()) {
            json warnings = json::array();
            for (const auto& issue : validation.warnings) {
                warnings.push_back(to_json_value(issue));
            }
            response["warnings"] = std::move(warnings);
        }
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "loadMap") {
        engine_.loadMap(request.at("mapId").get<int>());
        current_session_id_ = 0;
        paused_ = false;
        responses.push_back(base_response(request, command, true));
        return responses;
    }
    if (command == "setAlgorithm") {
        engine_.setAlgorithm(parse_algorithm(request.at("algorithm")));
        current_session_id_ = 0;
        paused_ = false;
        responses.push_back(base_response(request, command, true));
        return responses;
    }
    if (command == "configureScenario") {
        engine_.configureScenario(parse_scenario(request.at("scenario")));
        current_session_id_ = 0;
        paused_ = false;
        responses.push_back(base_response(request, command, true));
        return responses;
    }

    if (command == "subscribeFrameDelta") {
        subscribed_ = request.value("enabled", true);
        json response = base_response(request, command, true);
        response["subscribed"] = subscribed_;
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "setPaused" || command == "pause" || command == "resume") {
        require_session();
        if (command == "pause") {
            paused_ = true;
        } else if (command == "resume") {
            paused_ = false;
        } else {
            paused_ = request.value("paused", false);
        }
        json response = base_response(request, command, true);
        response["paused"] = paused_;
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "setSpeedMultiplier") {
        require_session();
        const double multiplier = request.at("speedMultiplier").get<double>();
        try {
            engine_.setSpeedMultiplier(multiplier);
        } catch (const std::invalid_argument& error) {
            throw ProtocolError("validation_failed", error.what());
        }
        json response = base_response(request, command, true);
        response["sessionId"] = current_session_id_;
        response["speedMultiplier"] = multiplier;
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getStaticScene") {
        require_session();
        json response = base_response(request, command, true);
        response["scene"] = to_json_value(engine_.snapshotStaticScene());
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getFrame") {
        require_session();
        json response = base_response(request, command, true);
        response["frame"] = to_json_value(engine_.snapshotRenderFrame(parse_render_options(request, paused_)));
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getDelta") {
        require_session();
        json response = base_response(request, command, true);
        response["delta"] = to_json_value(
            engine_.snapshotRenderDelta(request.at("sinceFrameId").get<std::uint64_t>(), parse_render_options(request, paused_)));
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getLogs") {
        require_session();
        const std::uint64_t since = request.value("sinceSeq", static_cast<std::uint64_t>(0));
        const std::size_t max_entries = request.value("maxEntries", static_cast<std::size_t>(RENDER_LOG_BATCH_LIMIT));
        json logs = json::array();
        std::uint64_t last_seq = since;
        for (const auto& entry : engine_.snapshotStructuredLogs(since, max_entries)) {
            last_seq = std::max(last_seq, entry.seq);
            logs.push_back(to_json_value(entry));
        }
        json response = base_response(request, command, true);
        response["sinceSeq"] = since;
        response["lastLogSeq"] = last_seq;
        response["logs"] = std::move(logs);
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getMetrics") {
        require_session();
        json response = base_response(request, command, true);
        response["metrics"] = to_json_value(engine_.snapshotMetrics());
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "getDebugSnapshot") {
        require_session();
        json response = base_response(request, command, true);
        response["snapshot"] = to_json_value(engine_.snapshotDebugState(paused_));
        responses.push_back(std::move(response));
        return responses;
    }

    if (command == "step") {
        if (current_session_id_ == 0) {
            current_session_id_ = engine_.startConfiguredSession().sessionId;
        } else {
            require_session();
        }
        engine_.step();
        const core::RenderFrameSnapshot frame = engine_.snapshotRenderFrame(core::RenderQueryOptions{paused_, false, 0, false});
        json response = base_response(request, command, true);
        response["sessionId"] = frame.sessionId;
        response["complete"] = engine_.isComplete();
        response["frameId"] = frame.frameId;
        response["lastLogSeq"] = frame.lastLogSeq;
        responses.push_back(std::move(response));
        if (subscribed_) {
            responses.push_back(buildFrameEvent());
        }
        return responses;
    }

    if (command == "runBurst") {
        require_session();
        const int max_steps = request.value("maxSteps", 0);
        const int max_duration_ms = request.value("maxDurationMs", 0);
        if (max_steps <= 0 && max_duration_ms <= 0) {
            throw ProtocolError("invalid_request", "runBurst requires a positive maxSteps or maxDurationMs.");
        }

        json response = base_response(request, command, true);
        response["sessionId"] = current_session_id_;
        response["paused"] = paused_;
        if (paused_) {
            const auto frame = engine_.snapshotRenderFrame(core::RenderQueryOptions{paused_, false, 0, false});
            response["executedSteps"] = 0;
            response["complete"] = engine_.isComplete();
            response["frameId"] = frame.frameId;
            response["lastLogSeq"] = frame.lastLogSeq;
            responses.push_back(std::move(response));
            return responses;
        }

        const core::BurstRunResult burst = engine_.runBurst(max_steps, max_duration_ms);
        response["executedSteps"] = burst.executedSteps;
        response["complete"] = burst.complete;
        response["frameId"] = burst.frameId;
        response["lastLogSeq"] = burst.lastLogSeq;
        responses.push_back(std::move(response));
        if (subscribed_ && burst.executedSteps > 0) {
            responses.push_back(buildFrameEvent());
        }
        return responses;
    }

    if (command == "run") {
        if (current_session_id_ == 0) {
            current_session_id_ = engine_.startConfiguredSession().sessionId;
        } else {
            require_session();
        }
        const int max_steps = request.value("maxSteps", -1);
        int executed = 0;
        while (!paused_ && !engine_.isComplete() && (max_steps < 0 || executed < max_steps)) {
            engine_.step();
            ++executed;
        }
        const auto frame = engine_.snapshotRenderFrame(core::RenderQueryOptions{paused_, false, 0, false});
        json response = base_response(request, command, true);
        response["sessionId"] = frame.sessionId;
        response["executedSteps"] = executed;
        response["complete"] = engine_.isComplete();
        response["frameId"] = frame.frameId;
        response["lastLogSeq"] = frame.lastLogSeq;
        response["paused"] = paused_;
        responses.push_back(std::move(response));
        if (subscribed_ && executed > 0) {
            responses.push_back(buildFrameEvent());
        }
        return responses;
    }

    if (command == "shutdown") {
        json response = base_response(request, command, true);
        response["shuttingDown"] = true;
        responses.push_back(std::move(response));
        shutting_down_ = true;
        return responses;
    }

    throw ProtocolError("unsupported_command", "Unknown command: " + command);
}

json RenderIpcServer::buildFrameEvent() {
    core::RenderQueryOptions options;
    options.paused = paused_;
    options.logsTail = false;
    const core::RenderFrameSnapshot frame = engine_.snapshotRenderFrame(options);
    return json{
        {"type", "event"},
        {"protocolVersion", kProtocolVersion},
        {"event", "frameDeltaAvailable"},
        {"eventId", next_event_id_++},
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

        json request;
        try {
            request = json::parse(line);
            const std::vector<json> responses = processRequest(request);
            for (const json& response : responses) {
                output << response.dump() << '\n';
            }
            if (shutting_down_) {
                break;
            }
        } catch (const json::parse_error& error) {
            output << base_error_response(request, "invalid_request", error.what()).dump() << '\n';
        } catch (const ProtocolError& error) {
            output << base_error_response(request, error.code(), error.what()).dump() << '\n';
        } catch (const std::exception& error) {
            output << base_error_response(request, "internal_error", error.what()).dump() << '\n';
        }
    }
}

}  // namespace agv::ipc
