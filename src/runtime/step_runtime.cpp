#define _CRT_SECURE_NO_WARNINGS

#include <ctime>

#include "agv/internal/engine_internal.hpp"

#ifndef CLEANUP_FORCE_IDLE_AFTER_STEPS
#define CLEANUP_FORCE_IDLE_AFTER_STEPS 11
#endif

#ifndef TURN_90_WAIT
#define TURN_90_WAIT 2
#endif

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_B_CYN "\x1b[1;36m"
#endif

void agent_manager_update_charge_state(AgentManager* m, GridMap* map, Logger* lg);
void agent_manager_update_state_after_move(AgentManager* m, ScenarioManager* sc, GridMap* map, Logger* lg, Simulation* sim);
void sort_agents_by_priority(AgentManager* m, int order[MAX_AGENTS]);

namespace {

inline int node_flat_index_local(const Node* node) {
    return node ? (node->y * GRID_WIDTH + node->x) : -1;
}

inline AgentDir dir_from_delta_local(int dx, int dy) {
    if (dx == 1 && dy == 0) return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == -1) return DIR_UP;
    if (dx == 0 && dy == 1) return DIR_DOWN;
    return DIR_NONE;
}

inline int dir_turn_steps_local(AgentDir from, AgentDir to) {
    if (from == DIR_NONE || to == DIR_NONE) return 0;
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
    if (new_heading == DIR_NONE) return;

    if (agent->heading == DIR_NONE) {
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
    const int order[MAX_AGENTS],
    Node* next_pos[MAX_AGENTS],
    StepScratch& scratch) {
    scratch.resetCellOwner(-1);

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int index = order[oi];
        if (!next_pos[index]) continue;
        int next_idx = node_flat_index_local(next_pos[index]);
        if (next_idx < 0) continue;
        if (scratch.cell_owner[next_idx] != -1) {
            next_pos[index] = manager->agents[index].pos;
            continue;
        }
        scratch.cell_owner[next_idx] = index;
    }

    scratch.resetCellOwner(-1);
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!manager->agents[i].pos) continue;
        int current_idx = node_flat_index_local(manager->agents[i].pos);
        if (current_idx >= 0) scratch.cell_owner[current_idx] = i;
    }

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int index = order[oi];
        if (!next_pos[index]) continue;
        int dest_idx = node_flat_index_local(next_pos[index]);
        int other = (dest_idx >= 0) ? scratch.cell_owner[dest_idx] : -1;
        if (other == -1 || other == index || !next_pos[other]) continue;
        if (next_pos[other] == manager->agents[index].pos) {
            next_pos[other] = manager->agents[other].pos;
        } else if (next_pos[other] == manager->agents[other].pos &&
            next_pos[index] == manager->agents[other].pos) {
            next_pos[index] = manager->agents[index].pos;
        }
    }
}

void force_idle_cleanup_local(AgentManager* manager, Simulation* sim, Logger* logger) {
    if (!manager) return;
    int changed = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (!agent->pos) continue;
        if (agent->state == IDLE) continue;
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = nullptr;
        }
        agent->pf.reset();
        agent->rotation_wait = 0;
        agent->stuck_steps = 0;
        agent->action_timer = 0;
        agent->state = IDLE;
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
    void execute(Simulation* sim, int is_paused) const {
        if (!sim) return;

        StepExecutionFrame frame = begin_frame(sim);
        agent_manager_update_charge_state(sim->agent_manager, sim->map, sim->logger);
        agv_update_task_dispatch(sim);

        StepScratch& scratch = sim->step_scratch;
        Node** next_pos = scratch.next_positions.data();
        Node** prev_pos = scratch.previous_positions.data();
        prepare_movement_plan(sim, next_pos, prev_pos, scratch);

        int moved_this_step = agv_apply_moves_and_update_stuck(sim, next_pos, prev_pos);
        finalize_move_state(sim, frame.step_label);
        finalize_frame(sim, frame, moved_this_step, is_paused);
    }

private:
    StepExecutionFrame begin_frame(Simulation* sim) const {
        StepExecutionFrame frame;
        frame.phase_idx_for_step = sim->scenario_manager->current_phase_index;
        frame.step_label = sim->scenario_manager->time_step + 1;
        frame.is_custom_mode = (sim->scenario_manager->mode == MODE_CUSTOM);
        frame.phase_active = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= 0 &&
            frame.phase_idx_for_step < sim->scenario_manager->num_phases);
        frame.cleanup_region = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= sim->scenario_manager->num_phases);
        frame.step_start_cpu = clock();
        return frame;
    }

    void prepare_movement_plan(Simulation* sim, Node** next_pos, Node** prev_pos, StepScratch& scratch) const {
        AgentManager* agents = sim->agent_manager;
        int* order = scratch.priority_order.data();

        for (int i = 0; i < MAX_AGENTS; i++) {
            prev_pos[i] = agents->agents[i].pos;
        }

        sim->planStep(next_pos);
        apply_rotation_stage(agents, next_pos);
        resolve_stationary_blockers(agents, next_pos, scratch);
        sort_agents_by_priority(agents, order);
        resolve_conflicts_by_order_local(agents, order, next_pos, scratch);
    }

    void apply_rotation_stage(AgentManager* agents, Node** next_pos) const {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* agent = &agents->agents[i];
            if (agent->state == CHARGING) continue;
            Node* current = agent->pos;
            if (!current || !next_pos[i]) continue;
            if (agent->rotation_wait > 0) {
                next_pos[i] = current;
                agent->rotation_wait--;
                continue;
            }
            Node* adjusted = current;
            agent_apply_rotation_and_step_local(agent, current, next_pos[i], &adjusted);
            next_pos[i] = adjusted;
        }
    }

    void resolve_stationary_blockers(AgentManager* agents, Node** next_pos, StepScratch& scratch) const {
        scratch.resetCellOwner(-1);
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* blocker = &agents->agents[i];
            if (!blocker->pos || !next_pos[i]) continue;
            if (blocker->rotation_wait > 0 || next_pos[i] == blocker->pos) {
                int blocked_idx = node_flat_index_local(blocker->pos);
                if (blocked_idx >= 0) scratch.cell_owner[blocked_idx] = i;
            }
        }

        for (int j = 0; j < MAX_AGENTS; j++) {
            Agent* mover = &agents->agents[j];
            if (!mover->pos || !next_pos[j] || next_pos[j] == mover->pos) continue;
            int blocked_idx = node_flat_index_local(next_pos[j]);
            if (blocked_idx < 0) continue;
            int blocker = scratch.cell_owner[blocked_idx];
            if (blocker != -1 && blocker != j) {
                next_pos[j] = mover->pos;
            }
        }
    }

    void finalize_move_state(Simulation* sim, int step_label) const {
        unsigned long long prev_completed_tasks = sim->tasks_completed_total;
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->map, sim->logger, sim);
        if (sim->tasks_completed_total != prev_completed_tasks) {
            sim->last_task_completion_step = step_label;
        }
    }

    void finalize_frame(Simulation* sim, const StepExecutionFrame& frame, int moved_this_step, int is_paused) const {
        clock_t step_end_cpu = clock();
        double step_time_ms = ((double)(step_end_cpu - frame.step_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
        sim->last_step_cpu_time_ms = step_time_ms;
        sim->total_cpu_time_ms += step_time_ms;
        if (step_time_ms > sim->max_step_cpu_time_ms) {
            sim->max_step_cpu_time_ms = step_time_ms;
        }

        record_step_phase_accounting(sim, frame, step_time_ms);
        agv_update_deadlock_counter(sim, moved_this_step, frame.is_custom_mode);
        agv_accumulate_wait_ticks_if_realtime(sim);
        sim->collectMemorySampleAlgo();
        sim->collectMemorySample();
        sim->total_executed_steps = frame.step_label;
        if (!sim->render_state.suppress_flush) {
            sim->renderer.drawFrame(sim, is_paused);
        }
    }
};

const StepExecutorService kStepExecutorService{};

}  // namespace

int agv_apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]) {
    int moved_this_step = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &sim->agent_manager->agents[i];
        if (agent->state != CHARGING && next_pos[i]) {
            if (agent->pos != next_pos[i]) {
                agent->total_distance_traveled += 1.0;
                sim->total_movement_cost += 1.0;
                moved_this_step = 1;
            }
            agent->pos = next_pos[i];
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &sim->agent_manager->agents[i];
        if (agent->state == CHARGING || agent->state == IDLE || agent->action_timer > 0) {
            agent->stuck_steps = 0;
            continue;
        }
        if (agent->pos == prev_pos[i]) agent->stuck_steps++;
        else agent->stuck_steps = 0;
    }

    return moved_this_step;
}

void agv_update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode) {
    ScenarioManager* scenario = sim->scenario_manager;
    if (moved_this_step) return;
    int unresolved = 0;
    if (is_custom_mode) {
        if (scenario->current_phase_index < scenario->num_phases) {
            const DynamicPhase* phase = &scenario->phases[scenario->current_phase_index];
            if (scenario->tasks_completed_in_phase < phase->task_count) unresolved = 1;
        }
    } else if (scenario->mode == MODE_REALTIME) {
        if (scenario->task_count > 0) unresolved = 1;
    }
    if (unresolved) sim->deadlock_count++;
}

void agv_accumulate_wait_ticks_if_realtime(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;
    if (scenario->mode == MODE_REALTIME && scenario->task_count > 0) {
        sim->request_wait_ticks_sum += static_cast<unsigned long long>(scenario->task_queue.size());
    }
}

void agv_execute_step_service(Simulation* sim, int is_paused) {
    kStepExecutorService.execute(sim, is_paused);
}
