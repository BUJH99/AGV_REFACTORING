#include "agv/simulation_engine.hpp"

#include "agv/internal/engine_internal.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace agv::core {

namespace {

::PathAlgo toInternalAlgo(PathAlgo algorithm) {
    switch (algorithm) {
        case PathAlgo::AStarSimple:
            return ::PathAlgo::AStarSimple;
        case PathAlgo::DStarBasic:
            return ::PathAlgo::DStarBasic;
        case PathAlgo::Default:
        default:
            return ::PathAlgo::Default;
    }
}

PathAlgo fromInternalAlgo(::PathAlgo algorithm) {
    switch (algorithm) {
        case ::PathAlgo::AStarSimple:
            return PathAlgo::AStarSimple;
        case ::PathAlgo::DStarBasic:
            return PathAlgo::DStarBasic;
        case ::PathAlgo::Default:
        default:
            return PathAlgo::Default;
    }
}

::SimulationMode toInternalMode(SimulationMode mode) {
    return mode == SimulationMode::Realtime ? ::SimulationMode::Realtime : ::SimulationMode::Custom;
}

SimulationMode fromInternalMode(::SimulationMode mode) {
    return mode == ::SimulationMode::Realtime ? SimulationMode::Realtime : SimulationMode::Custom;
}

AgentState fromInternalAgentState(::AgentState state) {
    switch (state) {
        case ::AgentState::GoingToPark:
            return AgentState::GoingToPark;
        case ::AgentState::ReturningHomeEmpty:
            return AgentState::ReturningHomeEmpty;
        case ::AgentState::GoingToCollect:
            return AgentState::GoingToCollect;
        case ::AgentState::ReturningWithCar:
            return AgentState::ReturningWithCar;
        case ::AgentState::GoingToCharge:
            return AgentState::GoingToCharge;
        case ::AgentState::Charging:
            return AgentState::Charging;
        case ::AgentState::ReturningHomeMaintenance:
            return AgentState::ReturningHomeMaintenance;
        case ::AgentState::Idle:
        default:
            return AgentState::Idle;
    }
}

::PhaseType toInternalPhase(PhaseType phase) {
    return phase == PhaseType::Exit ? ::PhaseType::Exit : ::PhaseType::Park;
}

MetricsSnapshot toMetricsSnapshot(const RunSummary& legacy) {
    MetricsSnapshot snapshot;
    snapshot.seed = legacy.seed;
    snapshot.mapId = legacy.map_id;
    snapshot.algorithm = fromInternalAlgo(legacy.path_algo);
    snapshot.mode = fromInternalMode(legacy.mode);
    snapshot.activeAgents = legacy.active_agents;
    snapshot.recordedSteps = legacy.recorded_steps;
    snapshot.tasksCompletedTotal = legacy.tasks_completed_total;
    snapshot.throughput = legacy.throughput;
    snapshot.totalMovementCost = legacy.total_movement_cost;
    snapshot.deadlockCount = legacy.deadlock_count;
    snapshot.totalCpuTimeMs = legacy.total_cpu_time_ms;
    snapshot.avgCpuTimeMs = legacy.avg_cpu_time_ms;
    snapshot.totalPlanningTimeMs = legacy.total_planning_time_ms;
    snapshot.avgPlanningTimeMs = legacy.avg_planning_time_ms;
    snapshot.memoryUsageSumKb = legacy.memory_usage_sum_kb;
    snapshot.avgMemoryUsageKb = legacy.avg_memory_usage_kb;
    snapshot.memoryUsagePeakKb = legacy.memory_usage_peak_kb;
    snapshot.algoNodesExpandedTotal = legacy.algo_nodes_expanded_total;
    snapshot.algoHeapMovesTotal = legacy.algo_heap_moves_total;
    snapshot.algoGeneratedNodesTotal = legacy.algo_generated_nodes_total;
    snapshot.algoValidExpansionsTotal = legacy.algo_valid_expansions_total;
    snapshot.validExpansionRatio = legacy.valid_expansion_ratio;
    snapshot.requestsCreatedTotal = legacy.requests_created_total;
    snapshot.requestWaitTicksSum = legacy.request_wait_ticks_sum;
    snapshot.remainingParkedVehicles = legacy.remaining_parked_vehicles;
    return snapshot;
}

int node_x(const Node* node) {
    return node ? node->x : -1;
}

int node_y(const Node* node) {
    return node ? node->y : -1;
}

std::vector<AgentDebugSnapshot> collectAgentDebugSnapshots(const Simulation& simulation) {
    std::vector<AgentDebugSnapshot> agents;
    if (!simulation.agent_manager) {
        return agents;
    }

    agents.reserve(MAX_AGENTS);
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = simulation.agent_manager->agents[index];
        if (!agent.pos && !agent.home_base && !agent.goal) {
            continue;
        }

        AgentDebugSnapshot snapshot;
        snapshot.id = agent.id;
        snapshot.symbol = agent.symbol;
        snapshot.state = fromInternalAgentState(agent.state);
        snapshot.isActive = agent.pos != nullptr;
        snapshot.posX = node_x(agent.pos);
        snapshot.posY = node_y(agent.pos);
        snapshot.lastPosX = node_x(agent.last_pos);
        snapshot.lastPosY = node_y(agent.last_pos);
        snapshot.homeX = node_x(agent.home_base);
        snapshot.homeY = node_y(agent.home_base);
        snapshot.goalX = node_x(agent.goal);
        snapshot.goalY = node_y(agent.goal);
        snapshot.totalDistanceTraveled = agent.total_distance_traveled;
        snapshot.chargeTimer = agent.charge_timer;
        snapshot.actionTimer = agent.action_timer;
        snapshot.stuckSteps = agent.stuck_steps;
        snapshot.oscillationSteps = agent.oscillation_steps;
        agents.push_back(snapshot);
    }

    return agents;
}

DeadlockSnapshot collectDeadlockSnapshot(const Simulation& simulation) {
    DeadlockSnapshot snapshot;
    const DeadlockEventRecord& event = simulation.last_deadlock_event;
    if (!event.valid) {
        return snapshot;
    }

    snapshot.hasEvent = true;
    snapshot.step = event.step;
    snapshot.deadlockCount = event.deadlock_count;
    snapshot.phaseIndex = event.phase_index;
    snapshot.phaseTaskTarget = event.phase_task_target;
    snapshot.phaseTasksCompleted = event.phase_tasks_completed;
    snapshot.pendingTaskCount = event.pending_task_count;
    snapshot.activeAgentCount = event.active_agent_count;
    snapshot.waitingAgentCount = event.waiting_agent_count;
    snapshot.stuckAgentCount = event.stuck_agent_count;
    snapshot.plannerWaitEdges = event.planner_wait_edges;
    snapshot.plannerSccCount = event.planner_scc_count;
    snapshot.plannerCbsSucceeded = event.planner_cbs_succeeded;
    snapshot.plannerCbsExpansions = event.planner_cbs_expansions;
    snapshot.whcaHorizon = event.whca_horizon;
    snapshot.plannedMoveCount = event.planned_move_count;
    snapshot.postRotationMoveCount = event.post_rotation_move_count;
    snapshot.postBlockerMoveCount = event.post_blocker_move_count;
    snapshot.finalMoveCount = event.final_move_count;
    snapshot.reason = event.reason;
    snapshot.rotationCanceledAgentIds.reserve(static_cast<std::size_t>(event.rotation_canceled_count));
    for (int index = 0; index < event.rotation_canceled_count; ++index) {
        snapshot.rotationCanceledAgentIds.push_back(event.rotation_canceled_agent_ids[index]);
    }
    snapshot.blockerCanceledAgentIds.reserve(static_cast<std::size_t>(event.blocker_canceled_count));
    for (int index = 0; index < event.blocker_canceled_count; ++index) {
        snapshot.blockerCanceledAgentIds.push_back(event.blocker_canceled_agent_ids[index]);
    }
    snapshot.orderCanceledAgentIds.reserve(static_cast<std::size_t>(event.order_canceled_count));
    for (int index = 0; index < event.order_canceled_count; ++index) {
        snapshot.orderCanceledAgentIds.push_back(event.order_canceled_agent_ids[index]);
    }
    snapshot.participantAgentIds.reserve(static_cast<std::size_t>(event.participant_count));
    for (int index = 0; index < event.participant_count; ++index) {
        snapshot.participantAgentIds.push_back(event.participant_agent_ids[index]);
    }
    return snapshot;
}

std::string_view path_algo_label(PathAlgo algorithm) {
    switch (algorithm) {
        case PathAlgo::AStarSimple:
            return "AStarSimple";
        case PathAlgo::DStarBasic:
            return "DStarBasic";
        case PathAlgo::Default:
        default:
            return "Default";
    }
}

std::string_view simulation_mode_label(SimulationMode mode) {
    return mode == SimulationMode::Realtime ? "Realtime" : "Custom";
}

std::string_view agent_state_label(AgentState state) {
    switch (state) {
        case AgentState::GoingToPark:
            return "GoingToPark";
        case AgentState::ReturningHomeEmpty:
            return "ReturningHomeEmpty";
        case AgentState::GoingToCollect:
            return "GoingToCollect";
        case AgentState::ReturningWithCar:
            return "ReturningWithCar";
        case AgentState::GoingToCharge:
            return "GoingToCharge";
        case AgentState::Charging:
            return "Charging";
        case AgentState::ReturningHomeMaintenance:
            return "ReturningHomeMaintenance";
        case AgentState::Idle:
        default:
            return "Idle";
    }
}

std::string buildDebugReportText(const DebugSnapshot& snapshot) {
    std::ostringstream report;
    report << "Simulation Debug Report\n";
    report << "=======================\n";
    report << "Seed: " << snapshot.metrics.seed << "\n";
    report << "Map: " << snapshot.metrics.mapId << "\n";
    report << "Algorithm: " << path_algo_label(snapshot.metrics.algorithm) << "\n";
    report << "Mode: " << simulation_mode_label(snapshot.metrics.mode) << "\n";
    report << "Recorded Steps: " << snapshot.metrics.recordedSteps << "\n";
    report << "Tasks Completed: " << snapshot.metrics.tasksCompletedTotal << "\n";
    report << "Requests Created: " << snapshot.metrics.requestsCreatedTotal << "\n";
    report << "Deadlocks: " << snapshot.metrics.deadlockCount << "\n";
    report << "Movement Cost: " << snapshot.metrics.totalMovementCost << "\n";
    report << "Active Agents: " << snapshot.metrics.activeAgents << "\n";
    report << "\nLast Deadlock Event\n";
    report << "-------------------\n";
    if (!snapshot.deadlock.hasEvent) {
        report << "(no deadlock event recorded)\n";
    } else {
        report << "Step: " << snapshot.deadlock.step << "\n";
        report << "Deadlock Count: " << snapshot.deadlock.deadlockCount << "\n";
        report << "Phase Index: " << snapshot.deadlock.phaseIndex << "\n";
        report << "Phase Progress: " << snapshot.deadlock.phaseTasksCompleted
               << "/" << snapshot.deadlock.phaseTaskTarget << "\n";
        report << "Pending Task Count: " << snapshot.deadlock.pendingTaskCount << "\n";
        report << "Active/Waiting/Stuck Agents: "
               << snapshot.deadlock.activeAgentCount << "/"
               << snapshot.deadlock.waitingAgentCount << "/"
               << snapshot.deadlock.stuckAgentCount << "\n";
        report << "Planner Metrics: wait_edges=" << snapshot.deadlock.plannerWaitEdges
               << " scc=" << snapshot.deadlock.plannerSccCount
               << " cbs_ok=" << snapshot.deadlock.plannerCbsSucceeded
               << " cbs_exp=" << snapshot.deadlock.plannerCbsExpansions
               << " whca_h=" << snapshot.deadlock.whcaHorizon << "\n";
        report << "Move Pipeline: plan=" << snapshot.deadlock.plannedMoveCount
               << " post_rotation=" << snapshot.deadlock.postRotationMoveCount
               << " post_blocker=" << snapshot.deadlock.postBlockerMoveCount
               << " final=" << snapshot.deadlock.finalMoveCount << "\n";
        report << "Canceled By Rotation:";
        if (snapshot.deadlock.rotationCanceledAgentIds.empty()) {
            report << " none";
        } else {
            for (const int agent_id : snapshot.deadlock.rotationCanceledAgentIds) {
                report << " " << agent_id;
            }
        }
        report << "\n";
        report << "Canceled By Stationary Blocker:";
        if (snapshot.deadlock.blockerCanceledAgentIds.empty()) {
            report << " none";
        } else {
            for (const int agent_id : snapshot.deadlock.blockerCanceledAgentIds) {
                report << " " << agent_id;
            }
        }
        report << "\n";
        report << "Canceled By Final Conflict Resolve:";
        if (snapshot.deadlock.orderCanceledAgentIds.empty()) {
            report << " none";
        } else {
            for (const int agent_id : snapshot.deadlock.orderCanceledAgentIds) {
                report << " " << agent_id;
            }
        }
        report << "\n";
        report << "Participants:";
        if (snapshot.deadlock.participantAgentIds.empty()) {
            report << " none";
        } else {
            for (const int agent_id : snapshot.deadlock.participantAgentIds) {
                report << " " << agent_id;
            }
        }
        report << "\n";
        report << "Reason: " << snapshot.deadlock.reason << "\n";
    }

    report << "\nRecent Logs\n";
    report << "-----------\n";
    if (snapshot.recentLogs.empty()) {
        report << "(no logs captured)\n";
    } else {
        for (const std::string& line : snapshot.recentLogs) {
            report << line << "\n";
        }
    }

    report << "\nAgent Snapshots\n";
    report << "---------------\n";
    if (snapshot.agents.empty()) {
        report << "(no tracked agents)\n";
    } else {
        for (const AgentDebugSnapshot& agent : snapshot.agents) {
            report << "Agent " << (agent.symbol ? agent.symbol : '?') << " [id=" << agent.id << "] "
                   << agent_state_label(agent.state)
                   << " pos=(" << agent.posX << "," << agent.posY << ")"
                   << " goal=(" << agent.goalX << "," << agent.goalY << ")"
                   << " home=(" << agent.homeX << "," << agent.homeY << ")"
                   << " last=(" << agent.lastPosX << "," << agent.lastPosY << ")"
                   << " stuck=" << agent.stuckSteps
                   << " oscillation=" << agent.oscillationSteps
                   << " charge=" << agent.chargeTimer
                   << " action=" << agent.actionTimer
                   << " distance=" << agent.totalDistanceTraveled
                   << "\n";
        }
    }

    report << "\nRender Frame\n";
    report << "------------\n";
    if (snapshot.frame.text.empty()) {
        report << "(empty frame)\n";
    } else {
        report << snapshot.frame.text;
        if (snapshot.frame.text.back() != '\n') {
            report << "\n";
        }
    }
    return report.str();
}

}  // namespace

struct SimulationEngine::Impl {
    std::unique_ptr<Simulation> simulation;
    SimulationConfig config{};
    bool dirty{true};
 
    Impl()
        : config(default_simulation_config()) {
    }

    Simulation& createFreshSimulation() {
        simulation = std::make_unique<Simulation>();
        return *simulation;
    }

    Simulation& requireInitialized(std::string_view operation) const {
        if (simulation == nullptr) {
            throw std::runtime_error(std::string(operation));
        }
        return *simulation;
    }

    Simulation& rebuildSimulationIfNeeded() {
        if (!dirty && simulation != nullptr) {
            return *simulation;
        }

        createFreshSimulation();
        if (!apply_simulation_config(simulation.get(), config)) {
            throw std::runtime_error("apply_simulation_config failed");
        }

        dirty = false;
        return *simulation;
    }
};

SimulationEngine::SimulationEngine()
    : impl_(std::make_unique<Impl>()) {}

SimulationEngine::~SimulationEngine() = default;

SimulationEngine::SimulationEngine(SimulationEngine&&) noexcept = default;

SimulationEngine& SimulationEngine::operator=(SimulationEngine&&) noexcept = default;

void SimulationEngine::setSeed(std::uint32_t seed) {
    impl_->config.seed = seed;
    impl_->dirty = true;
}

void SimulationEngine::loadMap(int mapId) {
    impl_->config.map_id = mapId;
    impl_->dirty = true;
}

void SimulationEngine::setAlgorithm(PathAlgo algorithm) {
    impl_->config.path_algo = toInternalAlgo(algorithm);
    impl_->dirty = true;
}

void SimulationEngine::configureScenario(const ScenarioConfig& config) {
    impl_->config.mode = toInternalMode(config.mode);
    impl_->config.speed_multiplier = static_cast<float>(config.speedMultiplier);
    impl_->config.realtime_park_chance = config.realtimeParkChance;
    impl_->config.realtime_exit_chance = config.realtimeExitChance;
    impl_->config.num_phases = std::min<int>(static_cast<int>(config.phases.size()), MAX_PHASES);

    std::fill(impl_->config.phases.begin(), impl_->config.phases.end(), ConfigPhase{::PhaseType::Park, 1});
    for (int i = 0; i < impl_->config.num_phases; ++i) {
        const auto& phase = config.phases[static_cast<std::size_t>(i)];
        impl_->config.phases[i].type = toInternalPhase(phase.type);
        impl_->config.phases[i].task_count = phase.taskCount;
    }

    impl_->dirty = true;
}

void SimulationEngine::setSuppressOutput(bool suppress) {
    impl_->config.suppress_stdout = suppress;
    impl_->dirty = true;
}

void SimulationEngine::prepareConsole() {
    agv_prepare_console();
}

bool SimulationEngine::interactiveSetup() {
    Simulation& simulation = impl_->createFreshSimulation();
    impl_->dirty = false;
    return simulation_setup(&simulation) != 0;
}

void SimulationEngine::runInteractiveConsole() {
    Simulation& simulation = impl_->requireInitialized("interactiveSetup must be called first");

    ui_enter_alt_screen();
    try {
        simulation.run();
    } catch (...) {
        ui_leave_alt_screen();
        throw;
    }
    ui_leave_alt_screen();
}

void SimulationEngine::printPerformanceSummary() const {
    impl_->requireInitialized("simulation is not initialized").printPerformanceSummary();
}

void SimulationEngine::step() {
    execute_headless_step(&impl_->rebuildSimulationIfNeeded());
}

void SimulationEngine::runUntilComplete() {
    Simulation& simulation = impl_->rebuildSimulationIfNeeded();
    if (!run_simulation_to_completion(&simulation)) {
        throw std::runtime_error("run_simulation_to_completion failed");
    }
}

bool SimulationEngine::isComplete() {
    return impl_->rebuildSimulationIfNeeded().isComplete();
}

MetricsSnapshot SimulationEngine::snapshotMetrics() {
    return toMetricsSnapshot(collect_run_summary(&impl_->rebuildSimulationIfNeeded()));
}

RenderFrame SimulationEngine::snapshotFrame(bool paused) {
    RenderFrame frame;
    frame.text = build_render_frame_text(&impl_->rebuildSimulationIfNeeded(), paused);
    return frame;
}

std::vector<std::string> SimulationEngine::snapshotRecentLogs() {
    return collect_recent_logs(impl_->rebuildSimulationIfNeeded().logger);
}

DebugSnapshot SimulationEngine::snapshotDebugState(bool paused) {
    Simulation& simulation = impl_->rebuildSimulationIfNeeded();

    DebugSnapshot snapshot;
    snapshot.metrics = toMetricsSnapshot(collect_run_summary(&simulation));
    snapshot.frame.text = build_render_frame_text(&simulation, paused);
    snapshot.recentLogs = collect_recent_logs(simulation.logger);
    snapshot.agents = collectAgentDebugSnapshots(simulation);
    snapshot.deadlock = collectDeadlockSnapshot(simulation);
    return snapshot;
}

std::string SimulationEngine::buildDebugReport(bool paused) {
    return buildDebugReportText(snapshotDebugState(paused));
}

bool SimulationEngine::writeDebugReport(const std::string& path, bool paused) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    output << buildDebugReport(paused);
    return output.good();
}

}  // namespace agv::core
