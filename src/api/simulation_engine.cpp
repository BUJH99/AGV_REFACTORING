#include "agv/simulation_engine.hpp"

#include "agv/internal/engine_internal.hpp"
#include "agv/internal/simulation_engine_access.hpp"

#include <algorithm>
#include <chrono>
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

PhaseType fromInternalPhase(::PhaseType phase) {
    return phase == ::PhaseType::Exit ? PhaseType::Exit : PhaseType::Park;
}

LaunchConfig default_launch_config() {
    LaunchConfig config;
    config.seed = static_cast<std::uint32_t>(std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()));
    config.mapId = 1;
    config.algorithm = PathAlgo::Default;
    config.scenario.mode = SimulationMode::Custom;
    config.scenario.speedMultiplier = 0.0;
    config.scenario.realtimeParkChance = 0;
    config.scenario.realtimeExitChance = 0;
    config.scenario.phases = {{PhaseType::Park, 1}};
    return config;
}

SimulationConfig to_internal_config(const LaunchConfig& launch_config, bool terminal_output_enabled) {
    SimulationConfig config{};
    config.seed = launch_config.seed;
    config.map_id = launch_config.mapId;
    config.path_algo = toInternalAlgo(launch_config.algorithm);
    config.mode = toInternalMode(launch_config.scenario.mode);
    config.speed_multiplier = static_cast<float>(launch_config.scenario.speedMultiplier);
    config.realtime_park_chance = launch_config.scenario.realtimeParkChance;
    config.realtime_exit_chance = launch_config.scenario.realtimeExitChance;
    config.num_phases = std::min<int>(static_cast<int>(launch_config.scenario.phases.size()), MAX_PHASES);
    std::fill(config.phases.begin(), config.phases.end(), ConfigPhase{::PhaseType::Park, 1});
    for (int index = 0; index < config.num_phases; ++index) {
        config.phases[index].type = toInternalPhase(launch_config.scenario.phases[static_cast<std::size_t>(index)].type);
        config.phases[index].task_count = launch_config.scenario.phases[static_cast<std::size_t>(index)].taskCount;
    }
    config.suppress_stdout = !terminal_output_enabled;
    return config;
}

std::string join_validation_messages(const std::vector<ValidationIssue>& issues) {
    std::ostringstream message;
    for (std::size_t index = 0; index < issues.size(); ++index) {
        if (index > 0) {
            message << "; ";
        }
        message << issues[index].field << ": " << issues[index].message;
    }
    return message.str();
}

int countInFlightTasks(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const ::AgentState state = manager->agents[index].state;
        if (state == ::AgentState::GoingToPark ||
            state == ::AgentState::ReturningHomeEmpty ||
            state == ::AgentState::GoingToCollect ||
            state == ::AgentState::ReturningWithCar) {
            count++;
        }
    }
    return count;
}

int countReadyIdleAgents(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = manager->agents[index];
        if (agent.pos && agent.state == ::AgentState::Idle) {
            count++;
        }
    }
    return count;
}

int countGoalActionAgents(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = manager->agents[index];
        if (agent.action_timer > 0 &&
            (agent.state == ::AgentState::GoingToPark || agent.state == ::AgentState::GoingToCollect)) {
            count++;
        }
    }
    return count;
}

int currentPhaseRemainingTasks(const Simulation& simulation) {
    if (!simulation.scenario_manager) {
        return 0;
    }

    const ScenarioManager& scenario = *simulation.scenario_manager;
    if (scenario.mode != ::SimulationMode::Custom ||
        scenario.current_phase_index < 0 ||
        scenario.current_phase_index >= scenario.num_phases) {
        return 0;
    }

    const DynamicPhase& phase = scenario.phases[scenario.current_phase_index];
    return std::max(phase.task_count - scenario.tasks_completed_in_phase, 0);
}

int oldestQueuedRequestAge(const ScenarioManager* scenario) {
    if (!scenario || scenario->task_queue.empty()) {
        return 0;
    }

    int oldest = 0;
    for (const TaskNode& task : scenario->task_queue) {
        oldest = std::max(oldest, std::max(scenario->time_step - task.created_at_step, 0));
    }
    return oldest;
}

int currentOutstandingTaskCount(const Simulation& simulation) {
    if (!simulation.scenario_manager) {
        return 0;
    }

    if (simulation.scenario_manager->mode == ::SimulationMode::Custom) {
        return currentPhaseRemainingTasks(simulation);
    }

    return simulation.scenario_manager->task_count + countInFlightTasks(simulation.agent_manager);
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
    snapshot.tasksPerAgent = legacy.tasks_per_agent;
    snapshot.totalMovementCost = legacy.total_movement_cost;
    snapshot.deadlockCount = legacy.deadlock_count;
    snapshot.totalCpuTimeMs = legacy.total_cpu_time_ms;
    snapshot.avgCpuTimeMs = legacy.avg_cpu_time_ms;
    snapshot.maxStepCpuTimeMs = legacy.max_step_cpu_time_ms;
    snapshot.avgCpuTimePerTaskMs = legacy.avg_cpu_time_per_task_ms;
    snapshot.tasksPerCpuSecond = legacy.tasks_per_cpu_second;
    snapshot.totalPlanningTimeMs = legacy.total_planning_time_ms;
    snapshot.avgPlanningTimeMs = legacy.avg_planning_time_ms;
    snapshot.maxPlanningTimeMs = legacy.max_planning_time_ms;
    snapshot.planningCpuShare = legacy.planning_cpu_share;
    snapshot.avgPlanningTimePerTaskMs = legacy.avg_planning_time_per_task_ms;
    snapshot.tasksPerPlanningSecond = legacy.tasks_per_planning_second;
    snapshot.memoryUsageSumKb = legacy.memory_usage_sum_kb;
    snapshot.avgMemoryUsageKb = legacy.avg_memory_usage_kb;
    snapshot.memoryUsagePeakKb = legacy.memory_usage_peak_kb;
    snapshot.avgMemoryUsagePerAgentKb = legacy.avg_memory_usage_per_agent_kb;
    snapshot.algoNodesExpandedTotal = legacy.algo_nodes_expanded_total;
    snapshot.algoHeapMovesTotal = legacy.algo_heap_moves_total;
    snapshot.algoGeneratedNodesTotal = legacy.algo_generated_nodes_total;
    snapshot.algoValidExpansionsTotal = legacy.algo_valid_expansions_total;
    snapshot.validExpansionRatio = legacy.valid_expansion_ratio;
    snapshot.avgNodesExpandedPerStep = legacy.avg_nodes_expanded_per_step;
    snapshot.avgNodesExpandedPerTask = legacy.avg_nodes_expanded_per_task;
    snapshot.nodesExpandedPerPlanningMs = legacy.nodes_expanded_per_planning_ms;
    snapshot.heapMovesPerNodeExpanded = legacy.heap_moves_per_node_expanded;
    snapshot.requestsCreatedTotal = legacy.requests_created_total;
    snapshot.requestWaitTicksSum = legacy.request_wait_ticks_sum;
    snapshot.avgRequestWaitTicks = legacy.avg_request_wait_ticks;
    snapshot.avgMovementPerTask = legacy.avg_movement_per_task;
    snapshot.nonTaskMovementCost = legacy.non_task_movement_cost;
    snapshot.taskMovementCoverageRatio = legacy.task_movement_coverage_ratio;
    snapshot.nonTaskMovementRatio = legacy.non_task_movement_ratio;
    snapshot.avgTaskDistance = legacy.avg_task_distance;
    snapshot.avgTaskTurns = legacy.avg_task_turns;
    snapshot.avgTaskSteps = legacy.avg_task_steps;
    snapshot.avgTaskStepsPerCell = legacy.avg_task_steps_per_cell;
    snapshot.avgTaskTurnsPer100Cells = legacy.avg_task_turns_per_100_cells;
    snapshot.stepsWithMovement = legacy.steps_with_movement;
    snapshot.stallStepCount = legacy.stall_step_count;
    snapshot.movementStepRatio = legacy.movement_step_ratio;
    snapshot.stallStepRatio = legacy.stall_step_ratio;
    snapshot.maxNoMovementStreak = legacy.max_no_movement_streak;
    snapshot.lastTaskCompletionStep = legacy.last_task_completion_step;
    snapshot.stepsSinceLastTaskCompletion = legacy.steps_since_last_task_completion;
    snapshot.queuedTaskCount = legacy.queued_task_count;
    snapshot.inFlightTaskCount = legacy.in_flight_task_count;
    snapshot.outstandingTaskCount = legacy.outstanding_task_count;
    snapshot.avgOutstandingTaskCount = legacy.avg_outstanding_task_count;
    snapshot.peakOutstandingTaskCount = legacy.peak_outstanding_task_count;
    snapshot.avgOutstandingTasksPerAgent = legacy.avg_outstanding_tasks_per_agent;
    snapshot.peakOutstandingTasksPerAgent = legacy.peak_outstanding_tasks_per_agent;
    snapshot.oldestQueuedRequestAge = legacy.oldest_queued_request_age;
    snapshot.avgOldestQueuedRequestAge = legacy.avg_oldest_queued_request_age;
    snapshot.peakOldestQueuedRequestAge = legacy.peak_oldest_queued_request_age;
    snapshot.plannerWaitEdgesSum = legacy.planner_wait_edges_sum;
    snapshot.plannerConflictCycleTotal = legacy.planner_conflict_cycle_total;
    snapshot.plannerWaitEdgeStepCount = legacy.planner_wait_edge_step_count;
    snapshot.plannerCycleStepCount = legacy.planner_cycle_step_count;
    snapshot.plannerWaitEdgesPerStep = legacy.planner_wait_edges_per_step;
    snapshot.plannerWaitEdgesPerConflictStep = legacy.planner_wait_edges_per_conflict_step;
    snapshot.plannerCycleStepRatio = legacy.planner_cycle_step_ratio;
    snapshot.plannerCbsAttemptCount = legacy.planner_cbs_attempt_count;
    snapshot.plannerCbsSuccessCount = legacy.planner_cbs_success_count;
    snapshot.plannerCbsFailureCount = legacy.planner_cbs_failure_count;
    snapshot.plannerCbsAttemptRate = legacy.planner_cbs_attempt_rate;
    snapshot.plannerCbsSuccessRate = legacy.planner_cbs_success_rate;
    snapshot.plannerCbsFailureRate = legacy.planner_cbs_failure_rate;
    snapshot.tasksPerAgentSpread.min = legacy.tasks_per_agent_spread.min;
    snapshot.tasksPerAgentSpread.avg = legacy.tasks_per_agent_spread.avg;
    snapshot.tasksPerAgentSpread.max = legacy.tasks_per_agent_spread.max;
    snapshot.tasksPerAgentSpread.stddev = legacy.tasks_per_agent_spread.stddev;
    snapshot.tasksPerAgentSpread.coefficientOfVariation = legacy.tasks_per_agent_spread.coefficient_of_variation;
    snapshot.tasksPerAgentSpread.minMaxRatio = legacy.tasks_per_agent_spread.min_max_ratio;
    snapshot.distancePerAgentSpread.min = legacy.distance_per_agent_spread.min;
    snapshot.distancePerAgentSpread.avg = legacy.distance_per_agent_spread.avg;
    snapshot.distancePerAgentSpread.max = legacy.distance_per_agent_spread.max;
    snapshot.distancePerAgentSpread.stddev = legacy.distance_per_agent_spread.stddev;
    snapshot.distancePerAgentSpread.coefficientOfVariation = legacy.distance_per_agent_spread.coefficient_of_variation;
    snapshot.distancePerAgentSpread.minMaxRatio = legacy.distance_per_agent_spread.min_max_ratio;
    snapshot.idleStepsPerAgentSpread.min = legacy.idle_steps_per_agent_spread.min;
    snapshot.idleStepsPerAgentSpread.avg = legacy.idle_steps_per_agent_spread.avg;
    snapshot.idleStepsPerAgentSpread.max = legacy.idle_steps_per_agent_spread.max;
    snapshot.idleStepsPerAgentSpread.stddev = legacy.idle_steps_per_agent_spread.stddev;
    snapshot.idleStepsPerAgentSpread.coefficientOfVariation = legacy.idle_steps_per_agent_spread.coefficient_of_variation;
    snapshot.idleStepsPerAgentSpread.minMaxRatio = legacy.idle_steps_per_agent_spread.min_max_ratio;
    snapshot.agentFairnessBreakdown.reserve(legacy.agent_fairness_breakdown.size());
    for (const AgentFairnessRecord& record : legacy.agent_fairness_breakdown) {
        AgentFairnessSnapshot fairness;
        fairness.id = record.id;
        fairness.symbol = record.symbol;
        fairness.tasksCompleted = record.tasks_completed;
        fairness.distanceCells = record.distance_cells;
        fairness.idleSteps = record.idle_steps;
        snapshot.agentFairnessBreakdown.push_back(fairness);
    }
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

    const int current_step = simulation.scenario_manager ? simulation.scenario_manager->time_step : 0;

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
        snapshot.rotationWait = agent.rotation_wait;
        snapshot.taskActive = agent.metrics_task_active;
        snapshot.taskAgeSteps = agent.metrics_task_active ? std::max(current_step - agent.metrics_task_start_step, 0) : 0;
        snapshot.taskDistance = agent.metrics_task_active
            ? std::max(agent.total_distance_traveled - agent.metrics_distance_at_start, 0.0)
            : 0.0;
        snapshot.taskTurns = agent.metrics_turns_current;
        snapshot.stuckSteps = agent.stuck_steps;
        snapshot.oscillationSteps = agent.oscillation_steps;
        agents.push_back(snapshot);
    }

    return agents;
}

void appendAgentIds(std::vector<int>& output, const std::array<int, MAX_AGENTS>& ids, int count) {
    output.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        if (ids[index] >= 0) {
            output.push_back(ids[index]);
        }
    }
}

RuntimeDebugSnapshot collectRuntimeDebugSnapshot(const Simulation& simulation) {
    RuntimeDebugSnapshot snapshot;
    snapshot.currentStep = simulation.total_executed_steps > 0
        ? simulation.total_executed_steps
        : (simulation.scenario_manager ? simulation.scenario_manager->time_step : 0);
    snapshot.noMovementStreak = simulation.no_movement_streak;
    snapshot.maxNoMovementStreak = simulation.max_no_movement_streak;
    snapshot.lastTaskCompletionStep = simulation.last_task_completion_step;
    snapshot.stepsSinceLastTaskCompletion = (simulation.last_task_completion_step > 0)
        ? std::max(snapshot.currentStep - simulation.last_task_completion_step, 0)
        : snapshot.currentStep;
    snapshot.lastStepCpuTimeMs = simulation.last_step_cpu_time_ms;
    snapshot.lastPlanningTimeMs = simulation.last_planning_time_ms;
    snapshot.plannedMoveCount = simulation.step_scratch.planned_move_count;
    snapshot.postRotationMoveCount = simulation.step_scratch.post_rotation_move_count;
    snapshot.postBlockerMoveCount = simulation.step_scratch.post_blocker_move_count;
    snapshot.finalMoveCount = simulation.step_scratch.final_move_count;
    snapshot.plannerWaitEdges = simulation.planner_metrics.wf_edges_last;
    snapshot.plannerSccCount = simulation.planner_metrics.scc_last;
    snapshot.plannerCbsSucceeded = simulation.planner_metrics.cbs_ok_last;
    snapshot.plannerCbsExpansions = simulation.planner_metrics.cbs_exp_last;
    snapshot.whcaHorizon = simulation.planner_metrics.whca_h;
    appendAgentIds(snapshot.rotationCanceledAgentIds,
        simulation.step_scratch.rotation_canceled_agent_ids,
        simulation.step_scratch.rotation_canceled_count);
    appendAgentIds(snapshot.blockerCanceledAgentIds,
        simulation.step_scratch.blocker_canceled_agent_ids,
        simulation.step_scratch.blocker_canceled_count);
    appendAgentIds(snapshot.orderCanceledAgentIds,
        simulation.step_scratch.order_canceled_agent_ids,
        simulation.step_scratch.order_canceled_count);

    if (!simulation.scenario_manager) {
        return snapshot;
    }

    const ScenarioManager& scenario = *simulation.scenario_manager;
    snapshot.hasActivePhase = scenario.mode == ::SimulationMode::Custom &&
        scenario.current_phase_index >= 0 &&
        scenario.current_phase_index < scenario.num_phases;
    snapshot.inCleanupRegion = scenario.mode == ::SimulationMode::Custom &&
        scenario.current_phase_index >= scenario.num_phases;
    snapshot.currentPhaseIndex = scenario.current_phase_index;
    snapshot.totalPhases = scenario.num_phases;
    if (snapshot.hasActivePhase) {
        const DynamicPhase& phase = scenario.phases[scenario.current_phase_index];
        snapshot.currentPhaseType = fromInternalPhase(phase.type);
        snapshot.phaseTaskTarget = phase.task_count;
        snapshot.phaseTasksCompleted = scenario.tasks_completed_in_phase;
        snapshot.phaseRemainingTasks = std::max(phase.task_count - scenario.tasks_completed_in_phase, 0);
    }

    snapshot.queuedTaskCount = scenario.task_count;
    snapshot.inFlightTaskCount = countInFlightTasks(simulation.agent_manager);
    snapshot.outstandingTaskCount = currentOutstandingTaskCount(simulation);
    snapshot.readyIdleAgentCount = countReadyIdleAgents(simulation.agent_manager);
    snapshot.activeGoalActionCount = countGoalActionAgents(simulation.agent_manager);
    snapshot.lastActiveAgentCount = simulation.last_active_agent_count;
    snapshot.lastWaitingAgentCount = simulation.last_waiting_agent_count;
    snapshot.lastStuckAgentCount = simulation.last_stuck_agent_count;
    snapshot.lastOscillatingAgentCount = simulation.last_oscillating_agent_count;
    snapshot.oldestQueuedRequestAge = oldestQueuedRequestAge(simulation.scenario_manager);
    return snapshot;
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

}  // namespace

ValidationResult validateLaunchConfig(const LaunchConfig& config) {
    ValidationResult result;
    result.normalizedConfig = config;

    if (result.normalizedConfig.seed == 0) {
        result.normalizedConfig.seed = default_launch_config().seed;
    }

    if (result.normalizedConfig.mapId < 1 || result.normalizedConfig.mapId > 7) {
        result.errors.push_back({"mapId", "out_of_range", "Map id must be in the range 1..7."});
    }

    if (result.normalizedConfig.scenario.speedMultiplier < 0.0 ||
        result.normalizedConfig.scenario.speedMultiplier > static_cast<double>(MAX_SPEED_MULTIPLIER)) {
        result.errors.push_back({
            "scenario.speedMultiplier",
            "out_of_range",
            "Speed multiplier must be in the range 0.0..10000.0.",
        });
    }

    if (result.normalizedConfig.scenario.realtimeParkChance < 0 ||
        result.normalizedConfig.scenario.realtimeParkChance > 100) {
        result.errors.push_back({
            "scenario.realtimeParkChance",
            "out_of_range",
            "Realtime park chance must be in the range 0..100.",
        });
    }

    if (result.normalizedConfig.scenario.realtimeExitChance < 0 ||
        result.normalizedConfig.scenario.realtimeExitChance > 100) {
        result.errors.push_back({
            "scenario.realtimeExitChance",
            "out_of_range",
            "Realtime exit chance must be in the range 0..100.",
        });
    }

    if ((result.normalizedConfig.scenario.realtimeParkChance +
            result.normalizedConfig.scenario.realtimeExitChance) > 100) {
        result.errors.push_back({
            "scenario.realtimeChanceSum",
            "sum_exceeds_100",
            "Realtime park chance plus exit chance must not exceed 100.",
        });
    }

    if (result.normalizedConfig.scenario.phases.size() > static_cast<std::size_t>(MAX_PHASES)) {
        result.errors.push_back({
            "scenario.phases",
            "too_many_phases",
            "Custom scenarios support at most 20 phases.",
        });
    }

    for (std::size_t index = 0; index < result.normalizedConfig.scenario.phases.size(); ++index) {
        if (result.normalizedConfig.scenario.phases[index].taskCount <= 0) {
            result.errors.push_back({
                "scenario.phases[" + std::to_string(index) + "].taskCount",
                "non_positive_task_count",
                "Each phase task count must be at least 1.",
            });
        }
    }

    if (result.normalizedConfig.scenario.mode == SimulationMode::Custom &&
        result.normalizedConfig.scenario.phases.empty()) {
        result.warnings.push_back({
            "scenario.phases",
            "defaulted_single_park",
            "Empty custom phases were normalized to a single Park x1 phase.",
        });
        result.normalizedConfig.scenario.phases.push_back({PhaseType::Park, 1});
    }

    return result;
}

struct SimulationEngine::Impl {
    std::unique_ptr<Simulation> simulation;
    LaunchConfig staged_launch_config{};
    CaptureLevel capture_level{CaptureLevel::None};
    bool terminal_output_enabled{true};
    bool dirty{true};
    std::uint64_t session_counter{0};
 
    Impl()
        : staged_launch_config(default_launch_config()) {
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

    ValidationResult validateStagedLaunchConfig() const {
        return validateLaunchConfig(staged_launch_config);
    }

    Simulation& rebuildSimulationIfNeeded() {
        if (!dirty && simulation != nullptr) {
            return *simulation;
        }

        const ValidationResult validation = validateStagedLaunchConfig();
        if (!validation.ok()) {
            throw std::invalid_argument(join_validation_messages(validation.errors));
        }

        createFreshSimulation();
        if (!apply_simulation_config(
                simulation.get(),
                to_internal_config(validation.normalizedConfig, terminal_output_enabled))) {
            throw std::runtime_error("apply_simulation_config failed");
        }
        simulation->capture_level = capture_level;
        render_model_reset(simulation.get(), ++session_counter);
        staged_launch_config = validation.normalizedConfig;
        dirty = false;
        return *simulation;
    }

    SessionDescriptor makeSessionDescriptor(const LaunchConfig& launch_config) const {
        SessionDescriptor descriptor;
        if (simulation != nullptr) {
            descriptor.sessionId = simulation->render_model.session_id;
            descriptor.sceneVersion = simulation->render_model.scene_version;
            descriptor.frameId = simulation->render_model.frame_id;
        }
        descriptor.launchConfig = launch_config;
        return descriptor;
    }

    SessionDescriptor startConfiguredSession() {
        dirty = true;
        (void)rebuildSimulationIfNeeded();
        return makeSessionDescriptor(staged_launch_config);
    }
};

SimulationEngine::SimulationEngine()
    : impl_(std::make_unique<Impl>()) {}

SimulationEngine::~SimulationEngine() = default;

SimulationEngine::SimulationEngine(SimulationEngine&&) noexcept = default;

SimulationEngine& SimulationEngine::operator=(SimulationEngine&&) noexcept = default;

void SimulationEngine::configureLaunch(const LaunchConfig& config) {
    const ValidationResult validation = validateLaunchConfig(config);
    if (!validation.ok()) {
        throw std::invalid_argument(join_validation_messages(validation.errors));
    }

    impl_->staged_launch_config = validation.normalizedConfig;
    impl_->dirty = true;
}

SessionDescriptor SimulationEngine::startConfiguredSession() {
    return impl_->startConfiguredSession();
}

void SimulationEngine::setCaptureLevel(CaptureLevel level) {
    impl_->capture_level = level;
    if (impl_->simulation != nullptr) {
        impl_->simulation->capture_level = level;
        render_model_reset(impl_->simulation.get(), impl_->simulation->render_model.session_id);
    }
}

void SimulationEngine::setTerminalOutputEnabled(bool enabled) {
    impl_->terminal_output_enabled = enabled;
    if (impl_->simulation != nullptr) {
        impl_->simulation->suppress_stdout = !enabled;
        impl_->simulation->render_state.suppress_flush = !enabled;
    }
}

void SimulationEngine::setSpeedMultiplier(double multiplier) {
    if (multiplier < 0.0 || multiplier > static_cast<double>(MAX_SPEED_MULTIPLIER)) {
        throw std::invalid_argument("scenario.speedMultiplier: Speed multiplier must be in the range 0.0..10000.0.");
    }

    impl_->staged_launch_config.scenario.speedMultiplier = multiplier;
    if (impl_->simulation != nullptr && !impl_->dirty) {
        simulation_set_speed_multiplier(impl_->simulation.get(), multiplier);
    }
}

void SimulationEngine::setSeed(std::uint32_t seed) {
    impl_->staged_launch_config.seed = seed;
    impl_->dirty = true;
}

void SimulationEngine::loadMap(int mapId) {
    impl_->staged_launch_config.mapId = mapId;
    impl_->dirty = true;
}

void SimulationEngine::setAlgorithm(PathAlgo algorithm) {
    impl_->staged_launch_config.algorithm = algorithm;
    impl_->dirty = true;
}

void SimulationEngine::configureScenario(const ScenarioConfig& config) {
    impl_->staged_launch_config.scenario = config;
    impl_->dirty = true;
}

void SimulationEngine::setSuppressOutput(bool suppress) {
    setTerminalOutputEnabled(!suppress);
}

void SimulationEngine::step() {
    execute_headless_step(&impl_->rebuildSimulationIfNeeded(), false);
}

BurstRunResult SimulationEngine::runBurst(int maxSteps, int maxDurationMs) {
    if (maxSteps <= 0 && maxDurationMs <= 0) {
        throw std::invalid_argument("runBurst requires a positive maxSteps or maxDurationMs.");
    }

    Simulation& simulation = impl_->rebuildSimulationIfNeeded();
    BurstRunResult result;
    const auto start = std::chrono::steady_clock::now();

    while (!simulation.isComplete()) {
        if (maxSteps > 0 && result.executedSteps >= maxSteps) {
            break;
        }
        if (maxDurationMs > 0) {
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            if (elapsed.count() >= maxDurationMs) {
                break;
            }
        }

        const bool complete_after_step = execute_headless_step(&simulation, false);
        ++result.executedSteps;
        if (complete_after_step) {
            break;
        }
    }

    const RenderFrameSnapshot frame = snapshot_render_frame(&simulation, RenderQueryOptions{});
    result.complete = simulation.isComplete();
    result.frameId = frame.frameId;
    result.lastLogSeq = frame.lastLogSeq;
    return result;
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

StaticSceneSnapshot SimulationEngine::snapshotStaticScene() {
    return snapshot_static_scene(&impl_->rebuildSimulationIfNeeded());
}

RenderFrameSnapshot SimulationEngine::snapshotRenderFrame(const RenderQueryOptions& options) {
    return snapshot_render_frame(&impl_->rebuildSimulationIfNeeded(), options);
}

RenderFrameDelta SimulationEngine::snapshotRenderDelta(
    std::uint64_t sinceFrameId,
    const RenderQueryOptions& options) {
    return snapshot_render_delta(&impl_->rebuildSimulationIfNeeded(), sinceFrameId, options);
}

std::vector<StructuredLogEntry> SimulationEngine::snapshotStructuredLogs(
    std::uint64_t sinceSeq,
    std::size_t maxEntries) {
    return snapshot_structured_logs(&impl_->rebuildSimulationIfNeeded(), sinceSeq, maxEntries);
}

std::vector<std::string> SimulationEngine::snapshotRecentLogs() {
    return collect_recent_logs(impl_->rebuildSimulationIfNeeded().logger);
}

DebugSnapshot SimulationEngine::snapshotDebugState(bool paused) {
    Simulation& simulation = impl_->rebuildSimulationIfNeeded();

    DebugSnapshot snapshot;
    snapshot.metrics = toMetricsSnapshot(collect_run_summary(&simulation));
    snapshot.runtime = collectRuntimeDebugSnapshot(simulation);
    RenderQueryOptions frame_options;
    frame_options.paused = paused;
    frame_options.logsTail = true;
    frame_options.maxLogEntries = RENDER_LOG_TAIL_LINES;
    frame_options.plannerOverlay = true;
    snapshot.frame = snapshot_render_frame(&simulation, frame_options);
    snapshot.recentLogs = collect_recent_logs(simulation.logger);
    snapshot.agents = collectAgentDebugSnapshots(simulation);
    snapshot.deadlock = collectDeadlockSnapshot(simulation);
    return snapshot;
}

}  // namespace agv::core

namespace agv::internal {

Simulation& SimulationEngineAccess::rebuild(core::SimulationEngine& engine) {
    return engine.impl_->rebuildSimulationIfNeeded();
}

Simulation& SimulationEngineAccess::requireInitialized(core::SimulationEngine& engine, std::string_view operation) {
    return engine.impl_->requireInitialized(operation);
}

}  // namespace agv::internal
