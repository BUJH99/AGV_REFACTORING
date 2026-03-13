#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

#include <algorithm>

namespace {

using agv::core::AgentRenderState;
using agv::core::GoalRenderState;
using agv::core::GridCoord;
using agv::core::HomeCellSnapshot;
using agv::core::HudSnapshot;
using agv::core::OverlayPlannedPath;
using agv::core::OverlayWaitEdge;
using agv::core::PlannerOverlaySnapshot;
using agv::core::RenderFrameDelta;
using agv::core::RenderFrameSnapshot;
using agv::core::RenderQueryOptions;
using agv::core::StaticSceneSnapshot;
using agv::core::StructuredLogEntry;

GridCoord node_coord(const Node* node) {
    if (!node) {
        return {};
    }
    return GridCoord{node->x, node->y};
}

int current_step(const Simulation* simulation) {
    if (!simulation) {
        return 0;
    }
    if (simulation->total_executed_steps > 0) {
        return simulation->total_executed_steps;
    }
    return simulation->scenario_manager ? simulation->scenario_manager->time_step : 0;
}

bool action_in_progress(const Agent& agent);

int count_in_flight_tasks(const AgentManager* manager) {
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
            ++count;
        }
    }
    return count;
}

int count_ready_idle_agents(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = manager->agents[index];
        if (agent.pos && agent.state == ::AgentState::Idle) {
            ++count;
        }
    }
    return count;
}

int count_goal_action_agents(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        if (action_in_progress(manager->agents[index])) {
            ++count;
        }
    }
    return count;
}

int current_phase_remaining_tasks(const Simulation* simulation) {
    if (!simulation || !simulation->scenario_manager) {
        return 0;
    }

    const ScenarioManager& scenario = *simulation->scenario_manager;
    if (scenario.mode != ::SimulationMode::Custom ||
        scenario.current_phase_index < 0 ||
        scenario.current_phase_index >= scenario.num_phases) {
        return 0;
    }

    const DynamicPhase& phase = scenario.phases[scenario.current_phase_index];
    return std::max(phase.task_count - scenario.tasks_completed_in_phase, 0);
}

int current_outstanding_task_count(const Simulation* simulation) {
    if (!simulation || !simulation->scenario_manager) {
        return 0;
    }

    if (simulation->scenario_manager->mode == ::SimulationMode::Custom) {
        return current_phase_remaining_tasks(simulation);
    }

    return simulation->scenario_manager->task_count + count_in_flight_tasks(simulation->agent_manager);
}

double average_cpu_ms(const Simulation* simulation) {
    if (!simulation) {
        return 0.0;
    }

    const int steps = std::max(current_step(simulation), 1);
    return simulation->total_cpu_time_ms / static_cast<double>(steps);
}

std::vector<StructuredLogEntry> collect_log_tail(const Logger* logger, std::size_t max_entries) {
    std::vector<StructuredLogEntry> logs;
    if (!logger || logger->structured_logs.empty() || max_entries == 0) {
        return logs;
    }

    const std::size_t count = std::min<std::size_t>(max_entries, logger->structured_logs.size());
    logs.reserve(count);
    const std::size_t start = logger->structured_logs.size() - count;
    for (std::size_t index = start; index < logger->structured_logs.size(); ++index) {
        logs.push_back(logger->structured_logs[index]);
    }
    return logs;
}

std::vector<StructuredLogEntry> collect_logs_after(const Logger* logger, std::uint64_t last_seq) {
    return collect_structured_logs(logger, last_seq, RENDER_LOG_BATCH_LIMIT);
}

bool action_in_progress(const Agent& agent) {
    return agent.action_timer > 0 &&
        (agent.state == ::AgentState::GoingToPark || agent.state == ::AgentState::GoingToCollect);
}

bool cancellation_contains(
    const std::array<int, MAX_AGENTS>& values,
    int count,
    int agent_id) {
    for (int index = 0; index < count; ++index) {
        if (values[index] == agent_id) {
            return true;
        }
    }
    return false;
}

bool moved_last_step(const Simulation* simulation, int agent_id) {
    if (!simulation || agent_id < 0 || agent_id >= MAX_AGENTS) {
        return false;
    }

    const Node* previous = simulation->step_scratch.previous_positions[agent_id];
    const Node* next = simulation->step_scratch.next_positions[agent_id];
    return previous != nullptr && next != nullptr && previous != next;
}

agv::core::AgentWaitReason classify_wait_reason(
    const Simulation* simulation,
    const Agent& agent,
    bool goal_action_in_progress) {
    if (!agent.pos || agent.state == ::AgentState::Idle) {
        return agv::core::AgentWaitReason::Idle;
    }

    if (agent.state == ::AgentState::Charging || agent.charge_timer > 0) {
        return agv::core::AgentWaitReason::Charging;
    }

    if (goal_action_in_progress) {
        return agv::core::AgentWaitReason::GoalAction;
    }

    if (agent.rotation_wait > 0) {
        return agv::core::AgentWaitReason::Rotation;
    }

    if (simulation) {
        const StepScratch& scratch = simulation->step_scratch;
        if (cancellation_contains(scratch.blocker_canceled_agent_ids, scratch.blocker_canceled_count, agent.id)) {
            return agv::core::AgentWaitReason::BlockedByStationary;
        }
        if (cancellation_contains(scratch.order_canceled_agent_ids, scratch.order_canceled_count, agent.id) ||
            simulation->planner_capture.yield_agents.contains(agent.id)) {
            return agv::core::AgentWaitReason::PriorityYield;
        }
    }

    if (agent.oscillation_steps > 0) {
        return agv::core::AgentWaitReason::Oscillating;
    }

    if (agent.stuck_steps > 0) {
        return agv::core::AgentWaitReason::Stuck;
    }

    if (agent.pos && agent.goal && agent.pos == agent.goal) {
        return agv::core::AgentWaitReason::GoalAction;
    }

    return agv::core::AgentWaitReason::None;
}

AgentRenderState build_agent_render_state(const Simulation* simulation, const Agent& agent) {
    AgentRenderState snapshot;
    snapshot.id = agent.id;
    snapshot.symbol = agent.symbol;
    snapshot.state = static_cast<agv::core::AgentState>(agent.state);
    snapshot.isActive = agent.pos != nullptr;
    snapshot.position = node_coord(agent.pos);
    snapshot.lastPosition = node_coord(agent.last_pos);
    snapshot.home = node_coord(agent.home_base);
    snapshot.goal = node_coord(agent.goal);
    snapshot.totalDistanceTraveled = agent.total_distance_traveled;
    snapshot.chargeTimer = agent.charge_timer;
    snapshot.actionTimer = agent.action_timer;
    snapshot.rotationWait = agent.rotation_wait;
    snapshot.taskActive = agent.metrics_task_active;
    snapshot.taskAgeSteps = agent.metrics_task_active
        ? std::max(current_step(simulation) - agent.metrics_task_start_step, 0)
        : 0;
    snapshot.taskDistance = agent.metrics_task_active
        ? std::max(agent.total_distance_traveled - agent.metrics_distance_at_start, 0.0)
        : 0.0;
    snapshot.taskTurns = agent.metrics_turns_current;
    snapshot.goalActionInProgress = action_in_progress(agent);
    snapshot.movedLastStep = moved_last_step(simulation, agent.id);
    snapshot.waitReason = classify_wait_reason(simulation, agent, snapshot.goalActionInProgress);
    snapshot.stuckSteps = agent.stuck_steps;
    snapshot.oscillationSteps = agent.oscillation_steps;
    return snapshot;
}

GoalRenderState build_goal_render_state(const Node* goal) {
    GoalRenderState snapshot;
    snapshot.position = node_coord(goal);
    snapshot.isGoal = goal ? goal->is_goal : false;
    snapshot.isParked = goal ? goal->is_parked : false;
    snapshot.reservedByAgent = goal ? goal->reserved_by_agent : -1;
    return snapshot;
}

OverlayPlannedPath build_overlay_planned_path(int agent_id, const TimedNodePlan& plan) {
    OverlayPlannedPath snapshot;
    snapshot.agentId = agent_id;

    GridCoord last{};
    bool has_last = false;
    for (const Node* node : plan) {
        if (!node) {
            continue;
        }

        const GridCoord coord = node_coord(node);
        if (has_last && coord.x == last.x && coord.y == last.y) {
            continue;
        }

        snapshot.cells.push_back(coord);
        last = coord;
        has_last = true;
    }

    return snapshot;
}

PlannerOverlaySnapshot build_planner_overlay_snapshot(const Simulation* simulation, const RenderQueryOptions& options) {
    PlannerOverlaySnapshot snapshot;
    if (!simulation || !options.plannerOverlay) {
        return snapshot;
    }

    const PlannerOverlayCapture& capture = simulation->planner_capture;
    if (!capture.valid) {
        return snapshot;
    }

    snapshot.available = true;
    snapshot.algorithm = static_cast<agv::core::PathAlgo>(capture.algorithm);
    snapshot.horizon = capture.horizon;
    snapshot.waitEdgeCount = capture.wait_edge_count;
    snapshot.leaderAgentId = capture.leader_agent_id;
    snapshot.usedCbs = capture.used_cbs;

    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (capture.scc_agents.contains(agent_id)) {
            snapshot.sccParticipantAgentIds.push_back(agent_id);
        }
        if (capture.yield_agents.contains(agent_id)) {
            snapshot.yieldAgentIds.push_back(agent_id);
        }
        if (capture.pull_over_agents.contains(agent_id)) {
            snapshot.pullOverAgentIds.push_back(agent_id);
        }
    }

    snapshot.waitEdges.reserve(static_cast<std::size_t>(capture.wait_edges.count));
    for (int index = 0; index < capture.wait_edges.count; ++index) {
        const WaitEdge& edge = capture.wait_edges.edges[index];
        OverlayWaitEdge item;
        item.fromAgentId = edge.from_id;
        item.toAgentId = edge.to_id;
        item.timeOffset = edge.t;
        item.cause = (edge.cause == CauseType::Swap) ? "swap" : "vertex";
        item.from = GridCoord{edge.x1, edge.y1};
        item.to = GridCoord{edge.x2, edge.y2};
        snapshot.waitEdges.push_back(item);
    }

    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        OverlayPlannedPath planned = build_overlay_planned_path(agent_id, capture.planned_paths[agent_id]);
        if (!planned.cells.empty()) {
            snapshot.plannedPaths.push_back(std::move(planned));
        }

        OverlayPlannedPath cbs = build_overlay_planned_path(agent_id, capture.cbs_paths[agent_id]);
        if (!cbs.cells.empty()) {
            snapshot.cbsPaths.push_back(std::move(cbs));
        }
    }

    return snapshot;
}

HudSnapshot build_hud_snapshot(const Simulation* simulation, const RenderQueryOptions& options) {
    HudSnapshot snapshot;
    if (!simulation) {
        return snapshot;
    }

    snapshot.mapId = simulation->map_id;
    snapshot.algorithm = static_cast<agv::core::PathAlgo>(simulation->path_algo);
    snapshot.mode = simulation->scenario_manager && simulation->scenario_manager->mode == ::SimulationMode::Realtime
        ? agv::core::SimulationMode::Realtime
        : agv::core::SimulationMode::Custom;
    snapshot.step = current_step(simulation);
    snapshot.paused = options.paused;
    snapshot.speedMultiplier = simulation->scenario_manager ? simulation->scenario_manager->speed_multiplier : 0.0;
    snapshot.currentPhaseIndex = simulation->scenario_manager ? simulation->scenario_manager->current_phase_index : -1;
    snapshot.totalPhases = simulation->scenario_manager ? simulation->scenario_manager->num_phases : 0;
    if (simulation->scenario_manager &&
        simulation->scenario_manager->mode == ::SimulationMode::Custom &&
        simulation->scenario_manager->current_phase_index >= 0 &&
        simulation->scenario_manager->current_phase_index < simulation->scenario_manager->num_phases) {
        const DynamicPhase& phase = simulation->scenario_manager->phases[simulation->scenario_manager->current_phase_index];
        snapshot.currentPhaseType = phase.type == ::PhaseType::Exit
            ? agv::core::PhaseType::Exit
            : agv::core::PhaseType::Park;
        snapshot.phaseTaskTarget = phase.task_count;
        snapshot.phaseTasksCompleted = simulation->scenario_manager->tasks_completed_in_phase;
        snapshot.phaseRemainingTasks = std::max(phase.task_count - simulation->scenario_manager->tasks_completed_in_phase, 0);
    }
    snapshot.queuedTaskCount = simulation->scenario_manager ? simulation->scenario_manager->task_count : 0;
    snapshot.inFlightTaskCount = count_in_flight_tasks(simulation->agent_manager);
    snapshot.outstandingTaskCount = current_outstanding_task_count(simulation);
    snapshot.readyIdleAgentCount = simulation->agent_manager ? count_ready_idle_agents(simulation->agent_manager) : 0;
    snapshot.activeGoalActionCount = simulation->agent_manager ? count_goal_action_agents(simulation->agent_manager) : 0;
    snapshot.waitingAgentCount = simulation->last_waiting_agent_count;
    snapshot.stuckAgentCount = simulation->last_stuck_agent_count;
    snapshot.oscillatingAgentCount = simulation->last_oscillating_agent_count;
    snapshot.noMovementStreak = simulation->no_movement_streak;
    snapshot.maxNoMovementStreak = simulation->max_no_movement_streak;
    snapshot.lastTaskCompletionStep = simulation->last_task_completion_step;
    snapshot.stepsSinceLastTaskCompletion = (simulation->last_task_completion_step > 0)
        ? std::max(current_step(simulation) - simulation->last_task_completion_step, 0)
        : current_step(simulation);
    snapshot.oldestQueuedRequestAge = simulation->oldest_request_age_last;
    snapshot.parkedCars = simulation->agent_manager ? simulation->agent_manager->total_cars_parked : 0;
    snapshot.totalGoalCount = simulation->map ? simulation->map->num_goals : 0;
    snapshot.lastStepCpuTimeMs = simulation->last_step_cpu_time_ms;
    snapshot.lastPlanningTimeMs = simulation->last_planning_time_ms;
    snapshot.avgCpuTimeMs = average_cpu_ms(simulation);
    snapshot.totalCpuTimeMs = simulation->total_cpu_time_ms;
    snapshot.plannedMoveCount = simulation->step_scratch.planned_move_count;
    snapshot.postRotationMoveCount = simulation->step_scratch.post_rotation_move_count;
    snapshot.postBlockerMoveCount = simulation->step_scratch.post_blocker_move_count;
    snapshot.finalMoveCount = simulation->step_scratch.final_move_count;
    snapshot.rotationCanceledCount = simulation->step_scratch.rotation_canceled_count;
    snapshot.blockerCanceledCount = simulation->step_scratch.blocker_canceled_count;
    snapshot.orderCanceledCount = simulation->step_scratch.order_canceled_count;
    snapshot.plannerWaitEdges = simulation->planner_metrics.wf_edges_last;
    snapshot.plannerSccCount = simulation->planner_metrics.scc_last;
    snapshot.plannerCbsSucceeded = simulation->planner_metrics.cbs_ok_last != 0;
    snapshot.plannerCbsExpansions = simulation->planner_metrics.cbs_exp_last;
    snapshot.whcaHorizon = simulation->runtime_tuning.whca_horizon;
    return snapshot;
}

RenderFrameSnapshot build_frame_snapshot(Simulation* simulation, const RenderQueryOptions& options) {
    RenderFrameSnapshot snapshot;
    if (!simulation) {
        return snapshot;
    }

    const RenderModelCache& cache = simulation->render_model;
    snapshot.sessionId = cache.session_id;
    snapshot.sceneVersion = cache.scene_version;
    snapshot.frameId = cache.frame_id;
    snapshot.lastLogSeq = (!simulation->logger || simulation->logger->structured_logs.empty())
        ? 0
        : simulation->logger->structured_logs.back().seq;
    snapshot.hud = build_hud_snapshot(simulation, options);

    if (simulation->agent_manager) {
        snapshot.agents.reserve(MAX_AGENTS);
        for (int index = 0; index < MAX_AGENTS; ++index) {
            const Agent& agent = simulation->agent_manager->agents[index];
            if (!agent.pos && !agent.home_base && !agent.goal) {
                continue;
            }
            snapshot.agents.push_back(build_agent_render_state(simulation, agent));
        }
    }

    if (simulation->map) {
        snapshot.goalStates.reserve(static_cast<std::size_t>(simulation->map->num_goals));
        for (int index = 0; index < simulation->map->num_goals; ++index) {
            snapshot.goalStates.push_back(build_goal_render_state(simulation->map->goals[index]));
        }
    }

    if (options.logsTail) {
        snapshot.logsTail = collect_log_tail(simulation->logger, options.maxLogEntries);
    }
    snapshot.plannerOverlay = build_planner_overlay_snapshot(simulation, options);
    return snapshot;
}

bool hud_equal(const HudSnapshot& lhs, const HudSnapshot& rhs) {
    return lhs.mapId == rhs.mapId &&
        lhs.algorithm == rhs.algorithm &&
        lhs.mode == rhs.mode &&
        lhs.step == rhs.step &&
        lhs.paused == rhs.paused &&
        lhs.speedMultiplier == rhs.speedMultiplier &&
        lhs.currentPhaseIndex == rhs.currentPhaseIndex &&
        lhs.totalPhases == rhs.totalPhases &&
        lhs.currentPhaseType == rhs.currentPhaseType &&
        lhs.phaseTaskTarget == rhs.phaseTaskTarget &&
        lhs.phaseTasksCompleted == rhs.phaseTasksCompleted &&
        lhs.phaseRemainingTasks == rhs.phaseRemainingTasks &&
        lhs.queuedTaskCount == rhs.queuedTaskCount &&
        lhs.inFlightTaskCount == rhs.inFlightTaskCount &&
        lhs.outstandingTaskCount == rhs.outstandingTaskCount &&
        lhs.readyIdleAgentCount == rhs.readyIdleAgentCount &&
        lhs.activeGoalActionCount == rhs.activeGoalActionCount &&
        lhs.waitingAgentCount == rhs.waitingAgentCount &&
        lhs.stuckAgentCount == rhs.stuckAgentCount &&
        lhs.oscillatingAgentCount == rhs.oscillatingAgentCount &&
        lhs.noMovementStreak == rhs.noMovementStreak &&
        lhs.maxNoMovementStreak == rhs.maxNoMovementStreak &&
        lhs.lastTaskCompletionStep == rhs.lastTaskCompletionStep &&
        lhs.stepsSinceLastTaskCompletion == rhs.stepsSinceLastTaskCompletion &&
        lhs.oldestQueuedRequestAge == rhs.oldestQueuedRequestAge &&
        lhs.parkedCars == rhs.parkedCars &&
        lhs.totalGoalCount == rhs.totalGoalCount &&
        lhs.lastStepCpuTimeMs == rhs.lastStepCpuTimeMs &&
        lhs.lastPlanningTimeMs == rhs.lastPlanningTimeMs &&
        lhs.avgCpuTimeMs == rhs.avgCpuTimeMs &&
        lhs.totalCpuTimeMs == rhs.totalCpuTimeMs &&
        lhs.plannedMoveCount == rhs.plannedMoveCount &&
        lhs.postRotationMoveCount == rhs.postRotationMoveCount &&
        lhs.postBlockerMoveCount == rhs.postBlockerMoveCount &&
        lhs.finalMoveCount == rhs.finalMoveCount &&
        lhs.rotationCanceledCount == rhs.rotationCanceledCount &&
        lhs.blockerCanceledCount == rhs.blockerCanceledCount &&
        lhs.orderCanceledCount == rhs.orderCanceledCount &&
        lhs.plannerWaitEdges == rhs.plannerWaitEdges &&
        lhs.plannerSccCount == rhs.plannerSccCount &&
        lhs.plannerCbsSucceeded == rhs.plannerCbsSucceeded &&
        lhs.plannerCbsExpansions == rhs.plannerCbsExpansions &&
        lhs.whcaHorizon == rhs.whcaHorizon;
}

bool agent_equal(const AgentRenderState& lhs, const AgentRenderState& rhs) {
    return lhs.id == rhs.id &&
        lhs.symbol == rhs.symbol &&
        lhs.state == rhs.state &&
        lhs.isActive == rhs.isActive &&
        lhs.position.x == rhs.position.x &&
        lhs.position.y == rhs.position.y &&
        lhs.lastPosition.x == rhs.lastPosition.x &&
        lhs.lastPosition.y == rhs.lastPosition.y &&
        lhs.home.x == rhs.home.x &&
        lhs.home.y == rhs.home.y &&
        lhs.goal.x == rhs.goal.x &&
        lhs.goal.y == rhs.goal.y &&
        lhs.totalDistanceTraveled == rhs.totalDistanceTraveled &&
        lhs.chargeTimer == rhs.chargeTimer &&
        lhs.actionTimer == rhs.actionTimer &&
        lhs.rotationWait == rhs.rotationWait &&
        lhs.taskActive == rhs.taskActive &&
        lhs.taskAgeSteps == rhs.taskAgeSteps &&
        lhs.taskDistance == rhs.taskDistance &&
        lhs.taskTurns == rhs.taskTurns &&
        lhs.goalActionInProgress == rhs.goalActionInProgress &&
        lhs.movedLastStep == rhs.movedLastStep &&
        lhs.waitReason == rhs.waitReason &&
        lhs.stuckSteps == rhs.stuckSteps &&
        lhs.oscillationSteps == rhs.oscillationSteps;
}

bool goal_equal(const GoalRenderState& lhs, const GoalRenderState& rhs) {
    return lhs.position.x == rhs.position.x &&
        lhs.position.y == rhs.position.y &&
        lhs.isGoal == rhs.isGoal &&
        lhs.isParked == rhs.isParked &&
        lhs.reservedByAgent == rhs.reservedByAgent;
}

bool planned_path_equal(const OverlayPlannedPath& lhs, const OverlayPlannedPath& rhs) {
    if (lhs.agentId != rhs.agentId || lhs.cells.size() != rhs.cells.size()) {
        return false;
    }
    for (std::size_t index = 0; index < lhs.cells.size(); ++index) {
        if (lhs.cells[index].x != rhs.cells[index].x || lhs.cells[index].y != rhs.cells[index].y) {
            return false;
        }
    }
    return true;
}

bool wait_edge_equal(const OverlayWaitEdge& lhs, const OverlayWaitEdge& rhs) {
    return lhs.fromAgentId == rhs.fromAgentId &&
        lhs.toAgentId == rhs.toAgentId &&
        lhs.timeOffset == rhs.timeOffset &&
        lhs.cause == rhs.cause &&
        lhs.from.x == rhs.from.x &&
        lhs.from.y == rhs.from.y &&
        lhs.to.x == rhs.to.x &&
        lhs.to.y == rhs.to.y;
}

bool overlay_equal(const PlannerOverlaySnapshot& lhs, const PlannerOverlaySnapshot& rhs) {
    if (lhs.available != rhs.available ||
        lhs.algorithm != rhs.algorithm ||
        lhs.horizon != rhs.horizon ||
        lhs.waitEdgeCount != rhs.waitEdgeCount ||
        lhs.leaderAgentId != rhs.leaderAgentId ||
        lhs.usedCbs != rhs.usedCbs ||
        lhs.sccParticipantAgentIds != rhs.sccParticipantAgentIds ||
        lhs.yieldAgentIds != rhs.yieldAgentIds ||
        lhs.pullOverAgentIds != rhs.pullOverAgentIds ||
        lhs.waitEdges.size() != rhs.waitEdges.size() ||
        lhs.plannedPaths.size() != rhs.plannedPaths.size() ||
        lhs.cbsPaths.size() != rhs.cbsPaths.size()) {
        return false;
    }

    for (std::size_t index = 0; index < lhs.waitEdges.size(); ++index) {
        if (!wait_edge_equal(lhs.waitEdges[index], rhs.waitEdges[index])) {
            return false;
        }
    }
    for (std::size_t index = 0; index < lhs.plannedPaths.size(); ++index) {
        if (!planned_path_equal(lhs.plannedPaths[index], rhs.plannedPaths[index])) {
            return false;
        }
    }
    for (std::size_t index = 0; index < lhs.cbsPaths.size(); ++index) {
        if (!planned_path_equal(lhs.cbsPaths[index], rhs.cbsPaths[index])) {
            return false;
        }
    }
    return true;
}

RenderFrameDelta build_frame_delta(
    const Simulation* simulation,
    const RenderFrameSnapshot& previous,
    const RenderFrameSnapshot& current) {
    RenderFrameDelta delta;
    delta.sessionId = current.sessionId;
    delta.sceneVersion = current.sceneVersion;
    delta.fromFrameId = previous.frameId;
    delta.toFrameId = current.frameId;
    delta.lastLogSeq = current.lastLogSeq;

    if (!hud_equal(previous.hud, current.hud)) {
        delta.hudChanged = true;
        delta.hud = current.hud;
    }

    for (const AgentRenderState& candidate : current.agents) {
        auto previous_it = std::find_if(
            previous.agents.begin(),
            previous.agents.end(),
            [&candidate](const AgentRenderState& item) { return item.id == candidate.id; });
        if (previous_it == previous.agents.end() || !agent_equal(*previous_it, candidate)) {
            delta.agentUpdates.push_back(candidate);
        }
    }

    for (const GoalRenderState& candidate : current.goalStates) {
        auto previous_it = std::find_if(
            previous.goalStates.begin(),
            previous.goalStates.end(),
            [&candidate](const GoalRenderState& item) {
                return item.position.x == candidate.position.x && item.position.y == candidate.position.y;
            });
        if (previous_it == previous.goalStates.end() || !goal_equal(*previous_it, candidate)) {
            delta.goalStateChanges.push_back(candidate);
        }
    }

    delta.newLogs = collect_logs_after(simulation ? simulation->logger : nullptr, previous.lastLogSeq);
    if (!overlay_equal(previous.plannerOverlay, current.plannerOverlay)) {
        delta.overlayChanged = current.plannerOverlay.available;
        delta.plannerOverlay = current.plannerOverlay;
    }

    return delta;
}

void push_delta(RenderModelCache& cache, RenderFrameDelta delta) {
    cache.recent_deltas.push_back(std::move(delta));
    while (cache.recent_deltas.size() > RenderModelCache::kDeltaHistoryLimit) {
        cache.recent_deltas.pop_front();
    }
}

void merge_agent_update(std::vector<AgentRenderState>& output, const AgentRenderState& update) {
    auto it = std::find_if(output.begin(), output.end(),
        [&update](const AgentRenderState& item) { return item.id == update.id; });
    if (it == output.end()) {
        output.push_back(update);
    } else {
        *it = update;
    }
}

void merge_goal_update(std::vector<GoalRenderState>& output, const GoalRenderState& update) {
    auto it = std::find_if(output.begin(), output.end(),
        [&update](const GoalRenderState& item) {
            return item.position.x == update.position.x && item.position.y == update.position.y;
        });
    if (it == output.end()) {
        output.push_back(update);
    } else {
        *it = update;
    }
}

RenderFrameDelta filter_delta_for_options(RenderFrameDelta delta, const RenderQueryOptions& options) {
    if (!options.logsTail) {
        delta.newLogs.clear();
    }
    if (!options.plannerOverlay) {
        delta.overlayChanged = false;
        delta.plannerOverlay = {};
    }
    return delta;
}

RenderFrameSnapshot filter_frame_for_options(RenderFrameSnapshot snapshot, const RenderQueryOptions& options) {
    if (!options.logsTail) {
        snapshot.logsTail.clear();
    }
    if (!options.plannerOverlay) {
        snapshot.plannerOverlay = {};
    }
    return snapshot;
}

RenderQueryOptions full_capture_options() {
    RenderQueryOptions options;
    options.logsTail = true;
    options.maxLogEntries = RENDER_LOG_TAIL_LINES;
    options.plannerOverlay = true;
    return options;
}

}  // namespace

void render_model_reset(Simulation* sim, std::uint64_t session_id) {
    if (!sim) {
        return;
    }

    sim->render_model.reset(session_id);
    sim->render_model.frame_id = static_cast<std::uint64_t>(std::max(current_step(sim), 0));
    if (sim->logger) {
        sim->logger->setContext(current_step(sim), sim->render_model.frame_id,
            sim->scenario_manager ? sim->scenario_manager->current_phase_index : -1);
    }
    sim->render_model.last_advanced_frame = build_frame_snapshot(sim, full_capture_options());
    sim->render_model.has_last_advanced_frame = true;
}

void render_model_capture_advanced_frame(Simulation* sim) {
    if (!sim) {
        return;
    }

    RenderModelCache& cache = sim->render_model;
    RenderFrameSnapshot current = build_frame_snapshot(sim, full_capture_options());
    if (cache.has_last_advanced_frame && current.frameId >= cache.last_advanced_frame.frameId) {
        push_delta(cache, build_frame_delta(sim, cache.last_advanced_frame, current));
    }
    cache.last_advanced_frame = std::move(current);
    cache.has_last_advanced_frame = true;
}

StaticSceneSnapshot snapshot_static_scene(const Simulation* sim) {
    StaticSceneSnapshot snapshot;
    if (!sim) {
        return snapshot;
    }

    snapshot.sessionId = sim->render_model.session_id;
    snapshot.sceneVersion = sim->render_model.scene_version;
    snapshot.mapId = sim->map_id;
    snapshot.width = GRID_WIDTH;
    snapshot.height = GRID_HEIGHT;
    snapshot.baseTiles.reserve(static_cast<std::size_t>(GRID_WIDTH * GRID_HEIGHT));

    if (sim->map) {
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                snapshot.baseTiles.push_back(sim->map->grid[y][x].is_obstacle ? '+' : '.');
            }
        }

        snapshot.goalCells.reserve(static_cast<std::size_t>(sim->map->num_goals));
        for (int index = 0; index < sim->map->num_goals; ++index) {
            snapshot.goalCells.push_back(node_coord(sim->map->goals[index]));
        }

        snapshot.chargerCells.reserve(static_cast<std::size_t>(sim->map->num_charge_stations));
        for (int index = 0; index < sim->map->num_charge_stations; ++index) {
            snapshot.chargerCells.push_back(node_coord(sim->map->charge_stations[index]));
        }
    }

    if (sim->agent_manager) {
        snapshot.homeCells.reserve(MAX_AGENTS);
        for (int index = 0; index < MAX_AGENTS; ++index) {
            const Agent& agent = sim->agent_manager->agents[index];
            if (!agent.home_base) {
                continue;
            }
            HomeCellSnapshot home;
            home.agentId = agent.id;
            home.symbol = agent.symbol;
            home.position = node_coord(agent.home_base);
            snapshot.homeCells.push_back(home);
        }
    }

    return snapshot;
}

RenderFrameSnapshot snapshot_render_frame(Simulation* sim, const RenderQueryOptions& options) {
    if (!sim) {
        return {};
    }
    return filter_frame_for_options(build_frame_snapshot(sim, options), options);
}

RenderFrameDelta snapshot_render_delta(
    Simulation* sim,
    std::uint64_t since_frame_id,
    const RenderQueryOptions& options) {
    RenderFrameDelta aggregated;
    if (!sim) {
        aggregated.requiresFullResync = true;
        return aggregated;
    }

    const RenderModelCache& cache = sim->render_model;
    aggregated.sessionId = cache.session_id;
    aggregated.sceneVersion = cache.scene_version;
    aggregated.fromFrameId = since_frame_id;
    aggregated.toFrameId = cache.frame_id;
    aggregated.lastLogSeq = cache.last_advanced_frame.lastLogSeq;

    if (since_frame_id == cache.frame_id) {
        return filter_delta_for_options(aggregated, options);
    }

    if (!cache.has_last_advanced_frame || since_frame_id > cache.frame_id || cache.recent_deltas.empty()) {
        aggregated.requiresFullResync = true;
        return aggregated;
    }

    const std::uint64_t oldest_from = cache.recent_deltas.front().fromFrameId;
    if (since_frame_id < oldest_from) {
        aggregated.requiresFullResync = true;
        return aggregated;
    }

    bool started = false;
    for (const RenderFrameDelta& delta : cache.recent_deltas) {
        if (delta.toFrameId <= since_frame_id) {
            continue;
        }

        if (!started) {
            if (delta.fromFrameId != since_frame_id) {
                aggregated.requiresFullResync = true;
                return aggregated;
            }
            started = true;
            aggregated.fromFrameId = delta.fromFrameId;
        } else if (aggregated.toFrameId != delta.fromFrameId) {
            aggregated.requiresFullResync = true;
            return aggregated;
        }

        aggregated.toFrameId = delta.toFrameId;
        aggregated.lastLogSeq = delta.lastLogSeq;
        if (delta.hudChanged) {
            aggregated.hudChanged = true;
            aggregated.hud = delta.hud;
        }
        for (const AgentRenderState& update : delta.agentUpdates) {
            merge_agent_update(aggregated.agentUpdates, update);
        }
        for (const GoalRenderState& update : delta.goalStateChanges) {
            merge_goal_update(aggregated.goalStateChanges, update);
        }
        aggregated.newLogs.insert(
            aggregated.newLogs.end(),
            delta.newLogs.begin(),
            delta.newLogs.end());
        if (delta.overlayChanged) {
            aggregated.overlayChanged = true;
            aggregated.plannerOverlay = delta.plannerOverlay;
        }
    }

    if (!started) {
        aggregated.requiresFullResync = true;
        return aggregated;
    }

    return filter_delta_for_options(aggregated, options);
}

std::vector<StructuredLogEntry> snapshot_structured_logs(
    Simulation* sim,
    std::uint64_t since_seq,
    std::size_t max_entries) {
    if (!sim) {
        return {};
    }

    const std::size_t limit = (max_entries == 0) ? RENDER_LOG_BATCH_LIMIT : max_entries;
    return collect_structured_logs(sim->logger, since_seq, limit);
}
