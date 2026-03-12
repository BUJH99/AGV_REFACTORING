#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"
#include "agent_goal_support.hpp"

AgentManager::AgentManager() {
    for (int i = 0; i < MAX_AGENTS; i++) {
        agents[i].id = i;
        agents[i].symbol = 'A' + i;
        agents[i].state = AgentState::Idle;
        agents[i].last_pos = nullptr;
        agents[i].heading = AgentDir::None;
        agents[i].rotation_wait = 0;
        agents[i].action_timer = 0;
        agents[i].pf.reset();
        agents[i].stuck_steps = 0;
        agents[i].oscillation_steps = 0;
        agents[i].metrics_task_active = false;
        agents[i].metrics_task_start_step = 0;
        agents[i].metrics_distance_at_start = 0.0;
        agents[i].metrics_turns_current = 0;
    }
}

void AgentManager::releasePathfinders() {
    for (int i = 0; i < MAX_AGENTS; i++) {
        agents[i].pf.reset();
    }
}

AgentManager::~AgentManager() {
    releasePathfinders();
}

namespace {

void metrics_finalize_task_if_active_local(Simulation* sim, Agent* agent) {
    if (!sim || !agent || !agent->metrics_task_active) return;
    int steps_now = (sim->scenario_manager ? sim->scenario_manager->time_step : 0);
    double d_move = agent->total_distance_traveled - agent->metrics_distance_at_start;
    int turns = agent->metrics_turns_current;
    double t_task = (double)(steps_now - agent->metrics_task_start_step);
    if (t_task < 0) t_task = 0;
    sim->metrics_task_count++;
    sim->metrics_sum_dmove += d_move;
    sim->metrics_sum_turns += turns;
    sim->metrics_sum_ttask += t_task;
    agent->metrics_task_active = false;
    agent->metrics_turns_current = 0;
}

void broadcast_cell_change_local(AgentManager* agents, GridMap* map, Node* changed) {
    if (!map || !changed) return;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agents->agents[i].pf) {
            agents->agents[i].pf->notifyCellChange(map, agents, changed);
        }
    }
}

AgentWorkloadSnapshot collect_agent_workload_local(const AgentManager* agents) {
    AgentWorkloadSnapshot snapshot{};
    if (!agents) return snapshot;

    for (int i = 0; i < MAX_AGENTS; ++i) {
        const Agent* agent = &agents->agents[i];
        switch (agent->state) {
        case AgentState::GoingToPark:
        case AgentState::ReturningHomeEmpty:
            snapshot.active_park_agents++;
            break;
        case AgentState::GoingToCollect:
        case AgentState::ReturningWithCar:
            snapshot.active_exit_agents++;
            break;
        default:
            break;
        }
    }

    return snapshot;
}

void record_completed_phase_task_local(ScenarioManager* scenario, Simulation* sim, PhaseType phase_type) {
    if (scenario &&
        scenario->mode == SimulationMode::Custom &&
        scenario->current_phase_index < scenario->num_phases &&
        scenario->phases[scenario->current_phase_index].type == phase_type) {
        scenario->tasks_completed_in_phase++;
        if (sim) {
            const int phase_index = scenario->current_phase_index;
            if (phase_index >= 0 && phase_index < MAX_PHASES) {
                sim->phase_completed_tasks[phase_index]++;
            }
            sim->tasks_completed_total++;
        }
        return;
    }

    if (sim) {
        sim->tasks_completed_total++;
    }
}

int advance_goal_action_timer_local(Agent* agent, Logger* logger) {
    if (!agent) return 0;
    if (agent->state != AgentState::GoingToPark && agent->state != AgentState::GoingToCollect) return 1;

    if (agent->action_timer <= 0) {
        agent->action_timer = TASK_ACTION_TICKS;
        logger_log(logger, "[%sTask%s] Agent %c, %s task started (%d ticks).", C_YEL, C_NRM, agent->symbol,
            agent->state == AgentState::GoingToPark ? "parking" : "exiting", agent->action_timer);
        return 0;
    }

    agent->action_timer--;
    return agent->action_timer <= 0;
}

void clear_goal_reservation_local(Agent* agent, Node* reached) {
    if (!agent || !reached) return;
    if (agent->state != AgentState::GoingToCharge) {
        reached->reserved_by_agent = -1;
    }
    agent->goal = nullptr;
}

bool reached_temporary_return_waypoint_local(Logger* logger, Agent* agent, Node* reached, const char* reason) {
    if (!agent) return false;
    if (agent->home_base && reached && reached != agent->home_base) {
        logger_log(logger, "[%sInfo%s] Agent %c reached a temporary holding goal at (%d,%d). Resuming the %s path.",
            C_CYN, C_NRM, agent->symbol, reached->x, reached->y, reason);
        agent->pf.reset();
        agent->stuck_steps = 0;
        return true;
    }
    return false;
}

void finish_parking_goal_local(
    AgentManager* agents,
    ScenarioManager* scenario,
    GridMap* map,
    Logger* logger,
    Simulation* sim,
    Agent* agent,
    Node* reached) {
    reached->is_parked = true;
    agents->total_cars_parked++;
    broadcast_cell_change_local(agents, map, reached);
    logger_log(logger, "[%sPark%s] Agent %c parked a vehicle at (%d,%d).", C_GRN, C_NRM, agent->symbol, reached->x, reached->y);
    record_completed_phase_task_local(scenario, sim, PhaseType::Park);
    agent->state = AgentState::ReturningHomeEmpty;
    agent->pf.reset();
}

void finish_return_home_empty_local(Logger* logger, Simulation* sim, Agent* agent, Node* reached) {
    if (!agent) return;
    if (reached_temporary_return_waypoint_local(logger, agent, reached, "return home")) {
        return;
    }

    logger_log(logger, "[%sInfo%s] Agent %c returned home after parking.", C_CYN, C_NRM, agent->symbol);
    agent->state = AgentState::Idle;
    agent->pf.reset();
    metrics_finalize_task_if_active_local(sim, agent);
}

void finish_collect_goal_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent, Node* reached) {
    logger_log(logger, "[%sExit%s] Agent %c picked up a parked vehicle at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached->x, reached->y);
    reached->is_parked = false;
    agents->total_cars_parked--;
    broadcast_cell_change_local(agents, map, reached);
    agent->state = AgentState::ReturningWithCar;
    agent->pf.reset();
}

void finish_return_with_car_local(ScenarioManager* scenario, Logger* logger, Simulation* sim, Agent* agent) {
    logger_log(logger, "[%sExit%s] Agent %c completed the retrieval task.", C_GRN, C_NRM, agent->symbol);
    record_completed_phase_task_local(scenario, sim, PhaseType::Exit);
    agent->state = AgentState::Idle;
    agent->pf.reset();
    metrics_finalize_task_if_active_local(sim, agent);
}

void finish_charge_arrival_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent) {
    logger_log(logger, "[%sCharge%s] Agent %c started charging (%d steps).", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
    agent->state = AgentState::Charging;
    agent->charge_timer = CHARGE_TIME;
    if (agent->pos) {
        broadcast_cell_change_local(agents, map, agent->pos);
    }
}

void finish_return_maintenance_local(Logger* logger, Agent* agent) {
    if (reached_temporary_return_waypoint_local(logger, agent, agent->pos, "post-charge return home")) {
        return;
    }

    logger_log(logger, "[%sInfo%s] Agent %c returned home after charging.", C_CYN, C_NRM, agent->symbol);
    agent->state = AgentState::Idle;
    agent->pf.reset();
}

void complete_agent_goal_local(
    AgentManager* agents,
    ScenarioManager* scenario,
    GridMap* map,
    Logger* logger,
    Simulation* sim,
    Agent* agent,
    Node* reached) {
    switch (agent->state) {
    case AgentState::GoingToPark:
        finish_parking_goal_local(agents, scenario, map, logger, sim, agent, reached);
        break;
    case AgentState::ReturningHomeEmpty:
        finish_return_home_empty_local(logger, sim, agent, reached);
        break;
    case AgentState::GoingToCollect:
        finish_collect_goal_local(agents, map, logger, agent, reached);
        break;
    case AgentState::ReturningWithCar:
        finish_return_with_car_local(scenario, logger, sim, agent);
        break;
    case AgentState::GoingToCharge:
        finish_charge_arrival_local(agents, map, logger, agent);
        break;
    case AgentState::ReturningHomeMaintenance:
        finish_return_maintenance_local(logger, agent);
        break;
    default:
        break;
    }
}

int tick_charge_timer_local(Agent* agent) {
    if (!agent || agent->state != AgentState::Charging) return 0;
    agent->charge_timer--;
    return agent->charge_timer <= 0;
}

void reset_goal_and_path_local(Agent* agent) {
    if (!agent) return;
    agent->goal = nullptr;
    agent->pf.reset();
}

void finish_charging_cycle_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent) {
    logger_log(logger, "[%sCharge%s] Agent %c finished charging.", C_B_GRN, C_NRM, agent->symbol);
    agent->total_distance_traveled = 0.0;
    agent->state = AgentState::ReturningHomeMaintenance;
    if (agent->pos) {
        broadcast_cell_change_local(agents, map, agent->pos);
    }
    reset_goal_and_path_local(agent);
    agent->stuck_steps = 0;
}

}  // namespace

void AgentManager::updateStateAfterMove(ScenarioManager* scenario, GridMap* map, Logger* logger, Simulation* sim) {
    AgentManager* agents = this;

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &agents->agents[i];
        if (agent->state == AgentState::Idle || agent->state == AgentState::Charging || !agent->goal || agent->pos != agent->goal) continue;

        if (!advance_goal_action_timer_local(agent, logger)) {
            continue;
        }

        Node* reached = agent->goal;
        clear_goal_reservation_local(agent, reached);
        complete_agent_goal_local(agents, scenario, map, logger, sim, agent, reached);
        agent->stuck_steps = 0;
    }
}

void agent_manager_update_state_after_move(AgentManager* manager, ScenarioManager* scenario, GridMap* map, Logger* logger, Simulation* sim) {
    if (manager) manager->updateStateAfterMove(scenario, map, logger, sim);
}

void AgentManager::updateChargeState(GridMap* map, Logger* logger) {
    AgentManager* agents = this;

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &agents->agents[i];
        if (!tick_charge_timer_local(agent)) {
            continue;
        }
        finish_charging_cycle_local(agents, map, logger, agent);
    }
}

void agent_manager_update_charge_state(AgentManager* manager, GridMap* map, Logger* logger) {
    if (manager) manager->updateChargeState(map, logger);
}

AgentWorkloadSnapshot agv_collect_agent_workload(const AgentManager* agents) {
    return collect_agent_workload_local(agents);
}

Node* agv_select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    return agent_runtime_select_best_charge_station(agent, map, agents, logger);
}

void agv_set_goal_if_needed(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    agent_runtime_assign_goal_if_needed(agent, map, agents, logger);
}
