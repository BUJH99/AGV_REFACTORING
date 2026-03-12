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

PhaseType fromInternalPhase(::PhaseType phase) {
    return phase == ::PhaseType::Exit ? PhaseType::Exit : PhaseType::Park;
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

std::string_view phase_type_label(PhaseType phase) {
    return phase == PhaseType::Exit ? "Exit" : "Park";
}

std::string buildDebugReportText(const DebugSnapshot& snapshot) {
    std::ostringstream report;
    auto append_id_list = [&report](std::string_view label, const std::vector<int>& ids) {
        report << label << ":";
        if (ids.empty()) {
            report << " none";
        } else {
            for (const int agent_id : ids) {
                report << " " << agent_id;
            }
        }
        report << "\n";
    };

    report << "Simulation Debug Report\n";
    report << "=======================\n";
    report << "Seed: " << snapshot.metrics.seed << "\n";
    report << "Map: " << snapshot.metrics.mapId << "\n";
    report << "Algorithm: " << path_algo_label(snapshot.metrics.algorithm) << "\n";
    report << "Mode: " << simulation_mode_label(snapshot.metrics.mode) << "\n";
    report << "Recorded Steps: " << snapshot.metrics.recordedSteps << "\n";
    report << "Tasks Completed: " << snapshot.metrics.tasksCompletedTotal << "\n";
    report << "Tasks/AGV: " << snapshot.metrics.tasksPerAgent << "\n";
    report << "Deadlocks: " << snapshot.metrics.deadlockCount << "\n";
    report << "Movement Cost: " << snapshot.metrics.totalMovementCost
           << " (avg/task=" << snapshot.metrics.avgMovementPerTask << ")\n";
    report << "Active Agents: " << snapshot.metrics.activeAgents << "\n";
    report << "Throughput: " << snapshot.metrics.throughput << " task/step\n";
    report << "CPU ms (total/avg/max): " << snapshot.metrics.totalCpuTimeMs
           << "/" << snapshot.metrics.avgCpuTimeMs
           << "/" << snapshot.metrics.maxStepCpuTimeMs << "\n";
    report << "CPU/task and Tasks/CPU-s: " << snapshot.metrics.avgCpuTimePerTaskMs
           << " ms / " << snapshot.metrics.tasksPerCpuSecond << "\n";
    report << "Planning ms (total/avg/max/share): " << snapshot.metrics.totalPlanningTimeMs
           << "/" << snapshot.metrics.avgPlanningTimeMs
           << "/" << snapshot.metrics.maxPlanningTimeMs
           << "/" << snapshot.metrics.planningCpuShare * 100.0 << "%\n";
    report << "Planning/task and Tasks/Planning-s: " << snapshot.metrics.avgPlanningTimePerTaskMs
           << " ms / " << snapshot.metrics.tasksPerPlanningSecond << "\n";
    report << "Movement/Stall Steps: " << snapshot.metrics.stepsWithMovement
           << " (" << snapshot.metrics.movementStepRatio * 100.0 << "%)"
           << " / " << snapshot.metrics.stallStepCount
           << " (" << snapshot.metrics.stallStepRatio * 100.0 << "%)\n";
    report << "Task Efficiency (distance/turns/steps): "
           << snapshot.metrics.avgTaskDistance << "/"
           << snapshot.metrics.avgTaskTurns << "/"
           << snapshot.metrics.avgTaskSteps << "\n";
    report << "Task Pace and Turn Density: "
           << snapshot.metrics.avgTaskStepsPerCell << " step/cell"
           << " / " << snapshot.metrics.avgTaskTurnsPer100Cells << " turns/100 cells\n";
    report << "Task-Captured Movement / Non-Task Movement: "
           << snapshot.metrics.taskMovementCoverageRatio * 100.0 << "% / "
           << snapshot.metrics.nonTaskMovementCost << " cells ("
           << snapshot.metrics.nonTaskMovementRatio * 100.0 << "%)\n";
    report << "Backlog (queued/in-flight/outstanding): "
           << snapshot.metrics.queuedTaskCount << "/"
           << snapshot.metrics.inFlightTaskCount << "/"
           << snapshot.metrics.outstandingTaskCount
           << " (avg=" << snapshot.metrics.avgOutstandingTaskCount
           << ", peak=" << snapshot.metrics.peakOutstandingTaskCount << ")\n";
    report << "Outstanding Tasks per AGV (avg/peak): "
           << snapshot.metrics.avgOutstandingTasksPerAgent << "/"
           << snapshot.metrics.peakOutstandingTasksPerAgent << "\n";
    if (snapshot.metrics.mode == SimulationMode::Realtime) {
        report << "Queued Request Age (current/avg/peak): "
               << snapshot.metrics.oldestQueuedRequestAge << "/"
               << snapshot.metrics.avgOldestQueuedRequestAge << "/"
               << snapshot.metrics.peakOldestQueuedRequestAge << "\n";
        report << "Requests Created / Avg Wait Ticks: " << snapshot.metrics.requestsCreatedTotal
               << " / " << snapshot.metrics.avgRequestWaitTicks << "\n";
    } else {
        report << "External Request Queue: N/A (custom phase-driven)\n";
    }
    report << "Planner Totals: nodes=" << snapshot.metrics.algoNodesExpandedTotal
           << " valid_ratio=" << snapshot.metrics.validExpansionRatio
           << " wait_edges(sum/steps)=" << snapshot.metrics.plannerWaitEdgesSum
           << "/" << snapshot.metrics.plannerWaitEdgeStepCount
           << " scc(sum/steps)=" << snapshot.metrics.plannerConflictCycleTotal
           << "/" << snapshot.metrics.plannerCycleStepCount
           << " cbs(attempt/success/fail)=" << snapshot.metrics.plannerCbsAttemptCount
           << "/" << snapshot.metrics.plannerCbsSuccessCount
           << "/" << snapshot.metrics.plannerCbsFailureCount
           << "\n";
    report << "Planner Derived KPIs: nodes/planning_ms=" << snapshot.metrics.nodesExpandedPerPlanningMs
           << " heap/node=" << snapshot.metrics.heapMovesPerNodeExpanded
           << " wait_edges/step=" << snapshot.metrics.plannerWaitEdgesPerStep
           << " wait_edges/conflict_step=" << snapshot.metrics.plannerWaitEdgesPerConflictStep
           << " cycle_step_ratio=" << snapshot.metrics.plannerCycleStepRatio * 100.0 << "%"
           << " cbs_attempt/step=" << snapshot.metrics.plannerCbsAttemptRate
           << " cbs_fail_rate=" << snapshot.metrics.plannerCbsFailureRate * 100.0 << "%\n";
    report << "AGV Fairness: tasks(min/avg/max/stddev/cv/min-max)="
           << snapshot.metrics.tasksPerAgentSpread.min << "/"
           << snapshot.metrics.tasksPerAgentSpread.avg << "/"
           << snapshot.metrics.tasksPerAgentSpread.max << "/"
           << snapshot.metrics.tasksPerAgentSpread.stddev << "/"
           << snapshot.metrics.tasksPerAgentSpread.coefficientOfVariation << "/"
           << snapshot.metrics.tasksPerAgentSpread.minMaxRatio
           << " distance(min/avg/max/stddev/cv/min-max)="
           << snapshot.metrics.distancePerAgentSpread.min << "/"
           << snapshot.metrics.distancePerAgentSpread.avg << "/"
           << snapshot.metrics.distancePerAgentSpread.max << "/"
           << snapshot.metrics.distancePerAgentSpread.stddev << "/"
           << snapshot.metrics.distancePerAgentSpread.coefficientOfVariation << "/"
           << snapshot.metrics.distancePerAgentSpread.minMaxRatio
           << " idle(min/avg/max/stddev/cv/min-max)="
           << snapshot.metrics.idleStepsPerAgentSpread.min << "/"
           << snapshot.metrics.idleStepsPerAgentSpread.avg << "/"
           << snapshot.metrics.idleStepsPerAgentSpread.max << "/"
           << snapshot.metrics.idleStepsPerAgentSpread.stddev << "/"
           << snapshot.metrics.idleStepsPerAgentSpread.coefficientOfVariation << "/"
           << snapshot.metrics.idleStepsPerAgentSpread.minMaxRatio << "\n";
    for (const AgentFairnessSnapshot& fairness : snapshot.metrics.agentFairnessBreakdown) {
        report << "  AGV " << fairness.symbol
               << " : tasks=" << fairness.tasksCompleted
               << ", distance=" << fairness.distanceCells
               << ", idle=" << fairness.idleSteps << "\n";
    }

    report << "\nRuntime Snapshot\n";
    report << "----------------\n";
    report << "Current Step: " << snapshot.runtime.currentStep << "\n";
    if (snapshot.runtime.hasActivePhase) {
        report << "Phase: " << (snapshot.runtime.currentPhaseIndex + 1)
               << "/" << snapshot.runtime.totalPhases
               << " (" << phase_type_label(snapshot.runtime.currentPhaseType) << ") "
               << snapshot.runtime.phaseTasksCompleted
               << "/" << snapshot.runtime.phaseTaskTarget
               << " remaining=" << snapshot.runtime.phaseRemainingTasks << "\n";
    } else if (snapshot.runtime.inCleanupRegion) {
        report << "Phase: cleanup after custom scenario phases\n";
    } else {
        report << "Phase: realtime / no active custom phase\n";
    }
    report << "Ready Idle Agents: " << snapshot.runtime.readyIdleAgentCount << "\n";
    report << "Goal Actions In Progress: " << snapshot.runtime.activeGoalActionCount << "\n";
    report << "Last Step CPU/Planning: " << snapshot.runtime.lastStepCpuTimeMs
           << "/" << snapshot.runtime.lastPlanningTimeMs << " ms\n";
    report << "Last Step Active/Waiting/Stuck/Oscillating: "
           << snapshot.runtime.lastActiveAgentCount << "/"
           << snapshot.runtime.lastWaitingAgentCount << "/"
           << snapshot.runtime.lastStuckAgentCount << "/"
           << snapshot.runtime.lastOscillatingAgentCount << "\n";
    report << "Last Step Move Pipeline: plan=" << snapshot.runtime.plannedMoveCount
           << " post_rotation=" << snapshot.runtime.postRotationMoveCount
           << " post_blocker=" << snapshot.runtime.postBlockerMoveCount
           << " final=" << snapshot.runtime.finalMoveCount << "\n";
    append_id_list("Rotation Canceled Agents", snapshot.runtime.rotationCanceledAgentIds);
    append_id_list("Blocker Canceled Agents", snapshot.runtime.blockerCanceledAgentIds);
    append_id_list("Order Canceled Agents", snapshot.runtime.orderCanceledAgentIds);
    report << "Last Step Planner: wait_edges=" << snapshot.runtime.plannerWaitEdges
           << " scc=" << snapshot.runtime.plannerSccCount
           << " cbs_ok=" << snapshot.runtime.plannerCbsSucceeded
           << " cbs_exp=" << snapshot.runtime.plannerCbsExpansions
           << " whca_h=" << snapshot.runtime.whcaHorizon << "\n";
    report << "No-Movement Streak (current/max): "
           << snapshot.runtime.noMovementStreak << "/"
           << snapshot.runtime.maxNoMovementStreak << "\n";
    report << "Completion Freshness: last_step=" << snapshot.runtime.lastTaskCompletionStep
           << " idle_for=" << snapshot.runtime.stepsSinceLastTaskCompletion << " step(s)\n";
    report << "Oldest Queued Request Age: " << snapshot.runtime.oldestQueuedRequestAge << "\n";

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
        report << "Outstanding Task Count: " << snapshot.deadlock.pendingTaskCount << "\n";
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
        append_id_list("Canceled By Rotation", snapshot.deadlock.rotationCanceledAgentIds);
        append_id_list("Canceled By Stationary Blocker", snapshot.deadlock.blockerCanceledAgentIds);
        append_id_list("Canceled By Final Conflict Resolve", snapshot.deadlock.orderCanceledAgentIds);
        append_id_list("Participants", snapshot.deadlock.participantAgentIds);
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
                   << " rotation_wait=" << agent.rotationWait
                   << " action=" << agent.actionTimer
                   << " task_active=" << (agent.taskActive ? "yes" : "no")
                   << " task_age=" << agent.taskAgeSteps
                   << " task_distance=" << agent.taskDistance
                   << " task_turns=" << agent.taskTurns
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
    snapshot.runtime = collectRuntimeDebugSnapshot(simulation);
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
