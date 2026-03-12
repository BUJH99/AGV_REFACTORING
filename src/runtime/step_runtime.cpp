#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <ctime>
#include <string>

#include "agv/internal/engine_internal.hpp"

void agent_manager_update_charge_state(AgentManager* m, GridMap* map, Logger* lg);
void agent_manager_update_state_after_move(AgentManager* m, ScenarioManager* sc, GridMap* map, Logger* lg, Simulation* sim);
void sort_agents_by_priority(AgentManager* m, AgentOrder& order);

namespace {

constexpr int kMemorySampleIntervalSteps = 8;

inline int node_flat_index_local(const Node* node) {
    return node ? (node->y * GRID_WIDTH + node->x) : -1;
}

inline bool node_is_charge_station_local(const GridMap* map, const Node* node) {
    if (!map || !node) return false;
    for (int index = 0; index < map->num_charge_stations; ++index) {
        if (map->charge_stations[index] == node) {
            return true;
        }
    }
    return false;
}

inline AgentDir dir_from_delta_local(int dx, int dy) {
    if (dx == 1 && dy == 0) return AgentDir::Right;
    if (dx == -1 && dy == 0) return AgentDir::Left;
    if (dx == 0 && dy == -1) return AgentDir::Up;
    if (dx == 0 && dy == 1) return AgentDir::Down;
    return AgentDir::None;
}

inline int dir_turn_steps_local(AgentDir from, AgentDir to) {
    if (from == AgentDir::None || to == AgentDir::None) return 0;
    int diff = ((int)to - (int)from + 4) % 4;
    return diff <= 2 ? diff : 4 - diff;
}

inline void agent_apply_rotation_and_step_local(Agent* agent, Node* current, Node* desired, Node** out_next) {
    if (!agent || !current || !out_next) return;
    *out_next = current;
    if (!desired || desired == current) return;

    int dx = desired->x - current->x;
    int dy = desired->y - current->y;
    AgentDir new_heading = dir_from_delta_local(dx, dy);
    if (new_heading == AgentDir::None) return;

    if (agent->heading == AgentDir::None) {
        agent->heading = new_heading;
        *out_next = desired;
        return;
    }

    int turn_steps = dir_turn_steps_local(agent->heading, new_heading);
    if (turn_steps == 1) {
        agent->rotation_wait = TURN_90_WAIT - 1;
        agent->heading = new_heading;
        agent->metrics_turns_current++;
        return;
    }
    agent->heading = new_heading;
    *out_next = desired;
}

void resolve_conflicts_by_order_local(
    AgentManager* manager,
    const AgentOrder& order,
    AgentNodeSlots& next_positions,
    StepScratch& scratch) {
    scratch.clearTouchedCellOwner();

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int index = order[oi];
        if (!next_positions[index]) continue;
        int next_idx = node_flat_index_local(next_positions[index]);
        if (next_idx < 0) continue;
        if (scratch.cell_owner[next_idx] != -1) {
            next_positions[index] = manager->agents[index].pos;
            continue;
        }
        scratch.setCellOwner(next_idx, index);
    }

    scratch.clearTouchedCellOwner();
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!manager->agents[i].pos) continue;
        int current_idx = node_flat_index_local(manager->agents[i].pos);
        if (current_idx >= 0) scratch.setCellOwner(current_idx, i);
    }

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int index = order[oi];
        if (!next_positions[index]) continue;
        int dest_idx = node_flat_index_local(next_positions[index]);
        int other = (dest_idx >= 0) ? scratch.cell_owner[dest_idx] : -1;
        if (other == -1 || other == index || !next_positions[other]) continue;
        if (next_positions[other] == manager->agents[index].pos) {
            next_positions[index] = manager->agents[index].pos;
            next_positions[other] = manager->agents[other].pos;
        } else if (next_positions[other] == manager->agents[other].pos &&
            next_positions[index] == manager->agents[other].pos) {
            next_positions[index] = manager->agents[index].pos;
        }
    }

    scratch.clearTouchedCellOwner();
}

bool agent_is_active_for_deadlock_local(const Agent* agent) {
    return agent &&
        agent->pos &&
        agent->state != AgentState::Idle &&
        agent->state != AgentState::Charging;
}

bool agent_has_goal_action_in_progress_local(const Agent* agent) {
    return agent &&
        agent->action_timer > 0 &&
        (agent->state == AgentState::GoingToPark ||
            agent->state == AgentState::GoingToCollect);
}

bool any_goal_action_in_progress_local(const AgentManager* manager) {
    if (!manager) {
        return false;
    }

    for (int index = 0; index < MAX_AGENTS; ++index) {
        if (agent_has_goal_action_in_progress_local(&manager->agents[index])) {
            return true;
        }
    }

    return false;
}

int count_in_flight_tasks_local(const AgentManager* manager) {
    if (!manager) {
        return 0;
    }

    int count = 0;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const AgentState state = manager->agents[index].state;
        if (state == AgentState::GoingToPark ||
            state == AgentState::ReturningHomeEmpty ||
            state == AgentState::GoingToCollect ||
            state == AgentState::ReturningWithCar) {
            count++;
        }
    }

    return count;
}

int current_phase_remaining_tasks_local(const Simulation* sim) {
    if (!sim || !sim->scenario_manager) {
        return 0;
    }

    const ScenarioManager* scenario = sim->scenario_manager;
    if (scenario->mode != SimulationMode::Custom ||
        scenario->current_phase_index < 0 ||
        scenario->current_phase_index >= scenario->num_phases) {
        return 0;
    }

    const DynamicPhase& phase = scenario->phases[scenario->current_phase_index];
    return std::max(phase.task_count - scenario->tasks_completed_in_phase, 0);
}

int current_oldest_request_age_local(const ScenarioManager* scenario) {
    if (!scenario || scenario->task_queue.empty()) {
        return 0;
    }

    int oldest_age = 0;
    for (const TaskNode& task : scenario->task_queue) {
        const int age = std::max(scenario->time_step - task.created_at_step, 0);
        if (age > oldest_age) {
            oldest_age = age;
        }
    }

    return oldest_age;
}

int current_outstanding_task_count_local(const Simulation* sim) {
    if (!sim || !sim->scenario_manager) {
        return 0;
    }

    const ScenarioManager* scenario = sim->scenario_manager;
    if (scenario->mode == SimulationMode::Custom) {
        return current_phase_remaining_tasks_local(sim);
    }

    return scenario->task_count + count_in_flight_tasks_local(sim->agent_manager);
}

void record_runtime_backlog_snapshot_local(Simulation* sim) {
    if (!sim || !sim->scenario_manager) {
        return;
    }

    const int outstanding = current_outstanding_task_count_local(sim);
    sim->outstanding_task_sum += static_cast<unsigned long long>(outstanding);
    sim->outstanding_task_samples++;
    if (outstanding > sim->outstanding_task_peak) {
        sim->outstanding_task_peak = outstanding;
    }

    const int oldest_age = current_oldest_request_age_local(sim->scenario_manager);
    sim->oldest_request_age_last = oldest_age;
    sim->oldest_request_age_sum += static_cast<unsigned long long>(oldest_age);
    if (oldest_age > sim->oldest_request_age_peak) {
        sim->oldest_request_age_peak = oldest_age;
    }
}

void record_step_agent_state_local(
    Simulation* sim,
    const AgentNodeSlots& next_positions,
    const AgentNodeSlots& previous_positions) {
    if (!sim || !sim->agent_manager) {
        return;
    }

    sim->last_active_agent_count = 0;
    sim->last_waiting_agent_count = 0;
    sim->last_stuck_agent_count = 0;
    sim->last_oscillating_agent_count = 0;
    sim->last_action_agent_count = 0;

    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = sim->agent_manager->agents[index];
        if (agent_has_goal_action_in_progress_local(&agent)) {
            sim->last_action_agent_count++;
        }
        if (!agent_is_active_for_deadlock_local(&agent)) {
            continue;
        }

        sim->last_active_agent_count++;
        if (next_positions[index] == previous_positions[index]) {
            sim->last_waiting_agent_count++;
        }
        if (agent.stuck_steps > 0) {
            sim->last_stuck_agent_count++;
        }
        if (agent.oscillation_steps > 0) {
            sim->last_oscillating_agent_count++;
        }
    }
}

void mark_idle_agents_at_step_start_local(
    const AgentManager* manager,
    std::array<unsigned char, MAX_AGENTS>& idle_flags) {
    idle_flags.fill(0);
    if (!manager) {
        return;
    }

    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = manager->agents[index];
        if (agent.pos && agent.state == AgentState::Idle) {
            idle_flags[index] = 1;
        }
    }
}

void record_idle_agents_that_remained_idle_local(
    AgentManager* manager,
    const std::array<unsigned char, MAX_AGENTS>& idle_flags) {
    if (!manager) {
        return;
    }

    for (int index = 0; index < MAX_AGENTS; ++index) {
        Agent& agent = manager->agents[index];
        if (idle_flags[index] != 0 && agent.pos && agent.state == AgentState::Idle) {
            agent.metrics_idle_steps_total++;
        }
    }
}

void append_unique_agent_id_local(std::array<int, MAX_AGENTS>& ids, int& count, int agent_id) {
    if (count >= MAX_AGENTS) {
        return;
    }
    for (int index = 0; index < count; ++index) {
        if (ids[index] == agent_id) {
            return;
        }
    }
    ids[count++] = agent_id;
}

int count_pipeline_moves_local(const AgentManager* manager, const AgentNodeSlots& positions, const AgentNodeSlots& previous_positions) {
    if (!manager) return 0;

    int count = 0;
    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (!manager->agents[agent_id].pos || !positions[agent_id]) {
            continue;
        }
        if (positions[agent_id] != previous_positions[agent_id]) {
            count++;
        }
    }
    return count;
}

void record_pipeline_cancellations_local(
    const AgentManager* manager,
    const AgentNodeSlots& before_positions,
    const AgentNodeSlots& after_positions,
    const AgentNodeSlots& previous_positions,
    std::array<int, MAX_AGENTS>& out_ids,
    int& out_count) {
    out_ids.fill(-1);
    out_count = 0;
    if (!manager) return;

    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (!manager->agents[agent_id].pos || !before_positions[agent_id] || !after_positions[agent_id]) {
            continue;
        }
        const bool had_move = before_positions[agent_id] != previous_positions[agent_id];
        const bool lost_move = after_positions[agent_id] == previous_positions[agent_id];
        if (had_move && lost_move) {
            append_unique_agent_id_local(out_ids, out_count, agent_id);
        }
    }
}

void append_deadlock_participant_local(DeadlockEventRecord& event, int agent_id) {
    if (event.participant_count >= MAX_AGENTS) {
        return;
    }
    for (int index = 0; index < event.participant_count; ++index) {
        if (event.participant_agent_ids[index] == agent_id) {
            return;
        }
    }
    event.participant_agent_ids[event.participant_count++] = agent_id;
}

std::string build_deadlock_reason_local(
    const Simulation* sim,
    int waiting_agent_count,
    int stuck_agent_count) {
    const PlannerMetricsState& metrics = sim->planner_metrics;
    if (metrics.scc_last > 0) {
        return "Wait-for graph cycle detected with no movement in this step.";
    }
    if (metrics.wf_edges_last > 0 && metrics.cbs_ok_last > 0) {
        return "CBS attempted to break the conflict, but a standstill still remained.";
    }
    if (metrics.wf_edges_last > 0 && metrics.cbs_exp_last > 0) {
        return "Planner reported unresolved wait edges and no active agent moved.";
    }
    if (stuck_agent_count > 0) {
        return "Multiple active agents remained stationary or oscillating while work was unresolved.";
    }
    if (waiting_agent_count > 0) {
        return "All active agents waited in place while unresolved work remained.";
    }
    return "Deadlock counter increased because no active movement occurred before task completion.";
}

void record_deadlock_event_local(
    Simulation* sim,
    const AgentNodeSlots& next_positions,
    bool is_custom_mode) {
    if (!sim || !sim->scenario_manager || !sim->agent_manager) {
        return;
    }

    DeadlockEventRecord event{};
    event.valid = true;
    event.step = sim->scenario_manager->time_step + 1;
    event.deadlock_count = sim->deadlock_count;
    event.phase_index = sim->scenario_manager->current_phase_index;
    event.pending_task_count = current_outstanding_task_count_local(sim);
    event.planner_wait_edges = sim->planner_metrics.wf_edges_last;
    event.planner_scc_count = sim->planner_metrics.scc_last;
    event.planner_cbs_succeeded = sim->planner_metrics.cbs_ok_last;
    event.planner_cbs_expansions = sim->planner_metrics.cbs_exp_last;
    event.whca_horizon = sim->planner_metrics.whca_h;
    event.planned_move_count = sim->step_scratch.planned_move_count;
    event.post_rotation_move_count = sim->step_scratch.post_rotation_move_count;
    event.post_blocker_move_count = sim->step_scratch.post_blocker_move_count;
    event.final_move_count = sim->step_scratch.final_move_count;
    event.rotation_canceled_count = sim->step_scratch.rotation_canceled_count;
    event.rotation_canceled_agent_ids = sim->step_scratch.rotation_canceled_agent_ids;
    event.blocker_canceled_count = sim->step_scratch.blocker_canceled_count;
    event.blocker_canceled_agent_ids = sim->step_scratch.blocker_canceled_agent_ids;
    event.order_canceled_count = sim->step_scratch.order_canceled_count;
    event.order_canceled_agent_ids = sim->step_scratch.order_canceled_agent_ids;

    if (is_custom_mode &&
        event.phase_index >= 0 &&
        event.phase_index < sim->scenario_manager->num_phases) {
        const DynamicPhase& phase = sim->scenario_manager->phases[event.phase_index];
        event.phase_task_target = phase.task_count;
        event.phase_tasks_completed = sim->scenario_manager->tasks_completed_in_phase;
    }

    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = sim->agent_manager->agents[index];
        if (!agent_is_active_for_deadlock_local(&agent)) {
            continue;
        }

        event.active_agent_count++;
        if (next_positions[index] == agent.pos) {
            event.waiting_agent_count++;
        }
        if (agent.stuck_steps > 0 || agent.oscillation_steps > 0) {
            event.stuck_agent_count++;
        }
        if (agent.stuck_steps >= DEADLOCK_THRESHOLD ||
            agent.oscillation_steps > 0 ||
            next_positions[index] == agent.pos) {
            append_deadlock_participant_local(event, agent.id);
        }
    }

    event.reason = build_deadlock_reason_local(sim, event.waiting_agent_count, event.stuck_agent_count);
    sim->last_deadlock_event = event;
}

void force_idle_cleanup_local(AgentManager* manager, Simulation* sim, Logger* logger) {
    if (!manager) return;
    int changed = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (!agent->pos) continue;
        if (agent->state == AgentState::Idle) continue;
        if (agent->pos->reserved_by_agent == agent->id) {
            agent->pos->reserved_by_agent = -1;
        }
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = nullptr;
        }
        agent->pf.reset();
        agent->rotation_wait = 0;
        agent->stuck_steps = 0;
        agent->oscillation_steps = 0;
        agent->action_timer = 0;
        agent->state = AgentState::Idle;
        changed = 1;
    }
    if (changed && logger) {
        logger_log(logger, "[%sCleanup%s] Forced unfinished agents back to IDLE before shutdown.", C_B_CYN, C_NRM);
    }
}

struct StepExecutionFrame final {
    int phase_idx_for_step{0};
    int step_label{0};
    bool is_custom_mode{false};
    bool phase_active{false};
    bool cleanup_region{false};
    clock_t step_start_cpu{0};
};

void record_step_phase_accounting(
    Simulation* sim,
    const StepExecutionFrame& frame,
    double step_time_ms) {
    if (!frame.is_custom_mode) return;

    if (frame.phase_active) {
        int index = frame.phase_idx_for_step;
        if (index >= 0 && index < MAX_PHASES) {
            if (sim->phase_step_counts[index] == 0) {
                sim->phase_first_step[index] = frame.step_label;
            }
            sim->phase_last_step[index] = frame.step_label;
            sim->phase_step_counts[index]++;
            sim->phase_cpu_time_ms[index] += step_time_ms;
        }
        return;
    }

    if (!frame.cleanup_region) return;

    if (sim->post_phase_step_count == 0) {
        sim->post_phase_first_step = frame.step_label;
    }
    sim->post_phase_last_step = frame.step_label;
    sim->post_phase_step_count++;
    sim->post_phase_cpu_time_ms += step_time_ms;
    if (sim->post_phase_step_count >= CLEANUP_FORCE_IDLE_AFTER_STEPS) {
        force_idle_cleanup_local(sim->agent_manager, sim, sim->logger);
    }
}

class StepExecutorService final {
public:
    void execute(Simulation* sim, bool is_paused) const {
        if (!sim) return;

        StepExecutionFrame frame = begin_frame(sim);
        agent_manager_update_charge_state(sim->agent_manager, sim->map, sim->logger);
        std::array<unsigned char, MAX_AGENTS> idle_agents_at_step_start{};
        mark_idle_agents_at_step_start_local(sim->agent_manager, idle_agents_at_step_start);
        agv_update_task_dispatch(sim);

        StepScratch& scratch = sim->step_scratch;
        AgentNodeSlots& next_positions = scratch.next_positions;
        AgentNodeSlots& previous_positions = scratch.previous_positions;
        prepare_movement_plan(sim, next_positions, previous_positions, scratch);

        const bool moved_this_step = agv_apply_moves_and_update_stuck(sim, next_positions, previous_positions);
        finalize_move_state(sim, frame.step_label);
        record_idle_agents_that_remained_idle_local(sim->agent_manager, idle_agents_at_step_start);
        finalize_frame(sim, frame, moved_this_step, is_paused);
    }

private:
    StepExecutionFrame begin_frame(Simulation* sim) const {
        StepExecutionFrame frame;
        frame.phase_idx_for_step = sim->scenario_manager->current_phase_index;
        frame.step_label = sim->scenario_manager->time_step + 1;
        frame.is_custom_mode = (sim->scenario_manager->mode == SimulationMode::Custom);
        frame.phase_active = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= 0 &&
            frame.phase_idx_for_step < sim->scenario_manager->num_phases);
        frame.cleanup_region = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= sim->scenario_manager->num_phases);
        frame.step_start_cpu = clock();
        if (sim->logger) {
            sim->logger->setContext(frame.step_label, 0, frame.phase_idx_for_step);
        }
        return frame;
    }

    void prepare_movement_plan(Simulation* sim, AgentNodeSlots& next_positions, AgentNodeSlots& previous_positions, StepScratch& scratch) const {
        AgentManager* agents = sim->agent_manager;
        AgentOrder& order = scratch.priority_order;

        for (int i = 0; i < MAX_AGENTS; i++) {
            previous_positions[i] = agents->agents[i].pos;
        }

        scratch.resetPlanDebug();

        sim->planStep(scratch.planner_positions);
        next_positions = scratch.planner_positions;
        scratch.planned_move_count = count_pipeline_moves_local(agents, next_positions, previous_positions);

        if (sim->path_algo == PathAlgo::Default) {
            scratch.post_rotation_positions = next_positions;
            scratch.post_rotation_move_count = scratch.planned_move_count;
            scratch.post_blocker_positions = next_positions;
            scratch.post_blocker_move_count = scratch.planned_move_count;
            scratch.final_move_count = scratch.planned_move_count;
            return;
        }

        apply_rotation_stage(agents, next_positions);
        scratch.post_rotation_positions = next_positions;
        scratch.post_rotation_move_count = count_pipeline_moves_local(agents, next_positions, previous_positions);
        record_pipeline_cancellations_local(
            agents,
            scratch.planner_positions,
            scratch.post_rotation_positions,
            previous_positions,
            scratch.rotation_canceled_agent_ids,
            scratch.rotation_canceled_count);

        resolve_stationary_blockers(agents, next_positions, scratch);
        scratch.post_blocker_positions = next_positions;
        scratch.post_blocker_move_count = count_pipeline_moves_local(agents, next_positions, previous_positions);
        record_pipeline_cancellations_local(
            agents,
            scratch.post_rotation_positions,
            scratch.post_blocker_positions,
            previous_positions,
            scratch.blocker_canceled_agent_ids,
            scratch.blocker_canceled_count);

        sort_agents_by_priority(agents, order);
        resolve_conflicts_by_order_local(agents, order, next_positions, scratch);
        scratch.final_move_count = count_pipeline_moves_local(agents, next_positions, previous_positions);
        record_pipeline_cancellations_local(
            agents,
            scratch.post_blocker_positions,
            next_positions,
            previous_positions,
            scratch.order_canceled_agent_ids,
            scratch.order_canceled_count);
    }

    void apply_rotation_stage(AgentManager* agents, AgentNodeSlots& next_positions) const {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &agents->agents[i];
            if (agent->state == AgentState::Charging) continue;
            Node* current = agent->pos;
            if (!current || !next_positions[i]) continue;
            if (agent->rotation_wait > 0) {
                next_positions[i] = current;
                agent->rotation_wait--;
                continue;
            }
            Node* adjusted = current;
            agent_apply_rotation_and_step_local(agent, current, next_positions[i], &adjusted);
            next_positions[i] = adjusted;
        }
    }

    void resolve_stationary_blockers(AgentManager* agents, AgentNodeSlots& next_positions, StepScratch& scratch) const {
        scratch.clearTouchedCellOwner();
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* blocker = &agents->agents[i];
            if (!blocker->pos || !next_positions[i]) continue;
            if (blocker->rotation_wait > 0 || next_positions[i] == blocker->pos) {
                int blocked_idx = node_flat_index_local(blocker->pos);
                if (blocked_idx >= 0) scratch.setCellOwner(blocked_idx, i);
            }
        }

        for (int j = 0; j < MAX_AGENTS; j++) {
            Agent* mover = &agents->agents[j];
            if (!mover->pos || !next_positions[j] || next_positions[j] == mover->pos) continue;
            int blocked_idx = node_flat_index_local(next_positions[j]);
            if (blocked_idx < 0) continue;
            int blocker = scratch.cell_owner[blocked_idx];
            if (blocker != -1 && blocker != j) {
                next_positions[j] = mover->pos;
            }
        }

        scratch.clearTouchedCellOwner();
    }

    void finalize_move_state(Simulation* sim, int step_label) const {
        unsigned long long prev_completed_tasks = sim->tasks_completed_total;
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->map, sim->logger, sim);
        if (sim->tasks_completed_total != prev_completed_tasks) {
            sim->last_task_completion_step = step_label;
        }
    }

    void finalize_frame(Simulation* sim, const StepExecutionFrame& frame, bool moved_this_step, bool is_paused) const {
        clock_t step_end_cpu = clock();
        double step_time_ms = ((double)(step_end_cpu - frame.step_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
        sim->last_step_cpu_time_ms = step_time_ms;
        sim->total_cpu_time_ms += step_time_ms;
        if (step_time_ms > sim->max_step_cpu_time_ms) {
            sim->max_step_cpu_time_ms = step_time_ms;
        }

        record_step_phase_accounting(sim, frame, step_time_ms);
        record_step_agent_state_local(sim, sim->step_scratch.next_positions, sim->step_scratch.previous_positions);
        agv_update_deadlock_counter(sim, sim->step_scratch.next_positions, moved_this_step, frame.is_custom_mode);
        agv_accumulate_wait_ticks_if_realtime(sim);
        record_runtime_backlog_snapshot_local(sim);
        if (frame.step_label == 1 || (frame.step_label % kMemorySampleIntervalSteps) == 0) {
            sim->collectMemorySampleAlgo();
            sim->collectMemorySample();
        }
        sim->total_executed_steps = frame.step_label;
        sim->render_model.frame_id = static_cast<std::uint64_t>(frame.step_label);
        render_model_capture_advanced_frame(sim);
        if (sim->logger) {
            sim->logger->setContext(frame.step_label, sim->render_model.frame_id, frame.phase_idx_for_step);
        }
        if (!sim->render_state.suppress_flush) {
            sim->renderer.drawFrame(sim, is_paused);
        }
    }
};

const StepExecutorService kStepExecutorService{};

}  // namespace

bool agv_apply_moves_and_update_stuck(Simulation* sim, AgentNodeSlots& next_positions, AgentNodeSlots& previous_positions) {
    bool moved_this_step = false;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &sim->agent_manager->agents[i];
        if (agent->state != AgentState::Charging && next_positions[i]) {
            Node* current = agent->pos;
            const bool immediate_backtrack =
                current &&
                agent->last_pos &&
                next_positions[i] == agent->last_pos &&
                current != agent->last_pos;
            if (agent->pos != next_positions[i]) {
                if (current &&
                    current->reserved_by_agent == agent->id &&
                    node_is_charge_station_local(sim->map, current)) {
                    current->reserved_by_agent = -1;
                }
                agent->total_distance_traveled += 1.0;
                agent->metrics_total_distance_all_time += 1.0;
                sim->total_movement_cost += 1.0;
                moved_this_step = true;
                agent->last_pos = current;
                agent->oscillation_steps = immediate_backtrack ? (agent->oscillation_steps + 1) : 0;
            } else {
                agent->oscillation_steps = 0;
            }
            agent->pos = next_positions[i];
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &sim->agent_manager->agents[i];
        if (agent->state == AgentState::Charging || agent->state == AgentState::Idle || agent->action_timer > 0) {
            agent->stuck_steps = 0;
            agent->oscillation_steps = 0;
            continue;
        }
        if (agent->pos == previous_positions[i] || agent->oscillation_steps > 0) agent->stuck_steps++;
        else agent->stuck_steps = 0;
    }

    return moved_this_step;
}

void agv_update_deadlock_counter(Simulation* sim, const AgentNodeSlots& next_positions, bool moved_this_step, bool is_custom_mode) {
    ScenarioManager* scenario = sim->scenario_manager;
    if (moved_this_step) {
        sim->steps_with_movement++;
        sim->no_movement_streak = 0;
        return;
    }
    int unresolved = 0;
    if (is_custom_mode) {
        if (scenario->current_phase_index < scenario->num_phases) {
            const DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
            if (scenario->tasks_completed_in_phase < phase->task_count) unresolved = 1;
        }
    } else if (scenario->mode == SimulationMode::Realtime) {
        if (scenario->task_count > 0) unresolved = 1;
    }
    if (!unresolved) {
        sim->no_movement_streak = 0;
        return;
    }

    if (any_goal_action_in_progress_local(sim->agent_manager)) {
        sim->no_movement_streak = 0;
        return;
    }

    sim->no_movement_streak++;
    sim->stall_step_count++;
    if (sim->no_movement_streak > sim->max_no_movement_streak) {
        sim->max_no_movement_streak = sim->no_movement_streak;
    }
    if (sim->no_movement_streak == DEADLOCK_THRESHOLD) {
        sim->deadlock_count++;
        record_deadlock_event_local(sim, next_positions, is_custom_mode);
    }
}

void agv_accumulate_wait_ticks_if_realtime(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;
    if (scenario->mode == SimulationMode::Realtime && scenario->task_count > 0) {
        sim->request_wait_ticks_sum += static_cast<unsigned long long>(scenario->task_queue.size());
    }
}

void agv_execute_step_service(Simulation* sim, bool is_paused) {
    kStepExecutorService.execute(sim, is_paused);
}
