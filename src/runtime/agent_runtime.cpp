#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"
#include "agent_goal_support.hpp"

#ifndef TASK_ACTION_TICKS
#define TASK_ACTION_TICKS 10
#endif

#ifndef CHARGE_TIME
#define CHARGE_TIME 20
#endif

#ifndef DISTANCE_BEFORE_CHARGE
#define DISTANCE_BEFORE_CHARGE 300.0
#endif

#ifndef INF
#define INF 1e18
#endif

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_GRN "\x1b[32m"
#define C_YEL "\x1b[33m"
#define C_CYN "\x1b[36m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#endif

void logger_log(Logger* logger, const char* format, ...);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);

AgentManager::AgentManager() {
    for (int i = 0; i < MAX_AGENTS; i++) {
        agents[i].id = i;
        agents[i].symbol = 'A' + i;
        agents[i].state = IDLE;
        agents[i].heading = DIR_NONE;
        agents[i].rotation_wait = 0;
        agents[i].action_timer = 0;
        agents[i].pf.reset();
        agents[i].stuck_steps = 0;
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
            pathfinder_notify_cell_change(agents->agents[i].pf.get(), map, agents, changed);
        }
    }
}

AgentWorkloadSnapshot collect_agent_workload_local(const AgentManager* agents) {
    AgentWorkloadSnapshot snapshot{};
    if (!agents) return snapshot;

    for (int i = 0; i < MAX_AGENTS; ++i) {
        const Agent* agent = &agents->agents[i];
        switch (agent->state) {
        case GOING_TO_PARK:
        case RETURNING_HOME_EMPTY:
            snapshot.active_park_agents++;
            break;
        case GOING_TO_COLLECT:
        case RETURNING_WITH_CAR:
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
        scenario->mode == MODE_CUSTOM &&
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
    if (agent->state != GOING_TO_PARK && agent->state != GOING_TO_COLLECT) return 1;

    if (agent->action_timer <= 0) {
        agent->action_timer = TASK_ACTION_TICKS;
        logger_log(logger, "[%sTask%s] Agent %c, %s task started (%d ticks).", C_YEL, C_NRM, agent->symbol,
            agent->state == GOING_TO_PARK ? "parking" : "exiting", agent->action_timer);
        return 0;
    }

    agent->action_timer--;
    return agent->action_timer <= 0;
}

void clear_goal_reservation_local(Agent* agent, Node* reached) {
    if (!agent || !reached) return;
    if (agent->state != GOING_TO_CHARGE) {
        reached->reserved_by_agent = -1;
    }
    agent->goal = nullptr;
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
    record_completed_phase_task_local(scenario, sim, PARK_PHASE);
    agent->state = RETURNING_HOME_EMPTY;
    agent->pf.reset();
}

void finish_return_home_empty_local(Logger* logger, Simulation* sim, Agent* agent, Node* reached) {
    if (!agent) return;
    if (agent->home_base && reached && reached != agent->home_base) {
        logger_log(logger, "[%sInfo%s] Agent %c reached a temporary holding goal at (%d,%d). Resuming the return home path.",
            C_CYN, C_NRM, agent->symbol, reached->x, reached->y);
        agent->pf.reset();
        agent->stuck_steps = 0;
        return;
    }

    logger_log(logger, "[%sInfo%s] Agent %c returned home after parking.", C_CYN, C_NRM, agent->symbol);
    agent->state = IDLE;
    agent->pf.reset();
    metrics_finalize_task_if_active_local(sim, agent);
}

void finish_collect_goal_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent, Node* reached) {
    logger_log(logger, "[%sExit%s] Agent %c picked up a parked vehicle at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached->x, reached->y);
    reached->is_parked = false;
    agents->total_cars_parked--;
    broadcast_cell_change_local(agents, map, reached);
    agent->state = RETURNING_WITH_CAR;
    agent->pf.reset();
}

void finish_return_with_car_local(ScenarioManager* scenario, Logger* logger, Simulation* sim, Agent* agent) {
    logger_log(logger, "[%sExit%s] Agent %c completed the retrieval task.", C_GRN, C_NRM, agent->symbol);
    record_completed_phase_task_local(scenario, sim, EXIT_PHASE);
    agent->state = IDLE;
    agent->pf.reset();
    metrics_finalize_task_if_active_local(sim, agent);
}

void finish_charge_arrival_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent) {
    logger_log(logger, "[%sCharge%s] Agent %c started charging (%d steps).", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
    agent->state = CHARGING;
    agent->charge_timer = CHARGE_TIME;
    if (agent->pos) {
        broadcast_cell_change_local(agents, map, agent->pos);
    }
}

void finish_return_maintenance_local(Logger* logger, Agent* agent) {
    logger_log(logger, "[%sInfo%s] Agent %c returned home after charging.", C_CYN, C_NRM, agent->symbol);
    agent->state = IDLE;
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
    case GOING_TO_PARK:
        finish_parking_goal_local(agents, scenario, map, logger, sim, agent, reached);
        break;
    case RETURNING_HOME_EMPTY:
        finish_return_home_empty_local(logger, sim, agent, reached);
        break;
    case GOING_TO_COLLECT:
        finish_collect_goal_local(agents, map, logger, agent, reached);
        break;
    case RETURNING_WITH_CAR:
        finish_return_with_car_local(scenario, logger, sim, agent);
        break;
    case GOING_TO_CHARGE:
        finish_charge_arrival_local(agents, map, logger, agent);
        break;
    case RETURNING_HOME_MAINTENANCE:
        finish_return_maintenance_local(logger, agent);
        break;
    default:
        break;
    }
}

int tick_charge_timer_local(Agent* agent) {
    if (!agent || agent->state != CHARGING) return 0;
    agent->charge_timer--;
    return agent->charge_timer <= 0;
}

void clear_position_reservation_local(Agent* agent) {
    if (agent && agent->pos) {
        agent->pos->reserved_by_agent = -1;
    }
}

void reset_goal_and_path_local(Agent* agent) {
    if (!agent) return;
    agent->goal = nullptr;
    agent->pf.reset();
}

void finish_charging_cycle_local(AgentManager* agents, GridMap* map, Logger* logger, Agent* agent) {
    logger_log(logger, "[%sCharge%s] Agent %c finished charging.", C_B_GRN, C_NRM, agent->symbol);
    agent->total_distance_traveled = 0.0;
    agent->state = RETURNING_HOME_MAINTENANCE;
    clear_position_reservation_local(agent);
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
        if (agent->state == IDLE || agent->state == CHARGING || !agent->goal || agent->pos != agent->goal) continue;

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
