#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

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
Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_destroy(Pathfinder* pf);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
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
        agents[i].metrics_task_active = 0;
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
    agent->metrics_task_active = 0;
    agent->metrics_turns_current = 0;
}

void broadcast_cell_change_local(AgentManager* agents, GridMap* map, Node* changed) {
    if (!map || !changed) return;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agents->agents[i].pf) {
            pathfinder_notify_cell_change(agents->agents[i].pf, map, agents, changed);
        }
    }
}

double calculate_path_cost_temp_pf_local(Agent* agent, Node* goal, GridMap* map, AgentManager* agents) {
    if (!agent || !agent->pos || !goal || !map || !agents) return INF;
    if (agent->pos == goal) return 0.0;
    if (goal->is_obstacle) return INF;

    Pathfinder* pathfinder = pathfinder_create(agent->pos, goal, agent);
    if (!pathfinder) return INF;

    pathfinder_compute_shortest_path(pathfinder, map, agents);
    double cost = pathfinder->cells[agent->pos->y][agent->pos->x].g;
    pathfinder_destroy(pathfinder);

    if (cost >= INF * 0.5) return INF;
    return cost;
}

Node* select_best_from_list_local(
    Agent* agent,
    GridMap* map,
    AgentManager* agents,
    Node** list,
    int count,
    int require_parked,
    int check_reserved,
    int toggle_parked_during_eval,
    double* out_best_cost) {
    double best = INF;
    Node* best_node = NULL;
    for (int i = 0; i < count; i++) {
        Node* node = list[i];
        if (!node) continue;
        if (require_parked == 1 && !node->is_parked) continue;
        if (require_parked == 0 && node->is_parked) continue;
        if (check_reserved && (node->reserved_by_agent != -1 && node->reserved_by_agent != agent->id)) continue;

        int restored = 0;
        if (toggle_parked_during_eval && node->is_parked) {
            node->is_parked = FALSE;
            restored = 1;
        }
        double cost = calculate_path_cost_temp_pf_local(agent, node, map, agents);
        if (restored) node->is_parked = TRUE;

        if (cost < best) {
            best = cost;
            best_node = node;
        }
    }
    if (out_best_cost) *out_best_cost = best;
    return best_node;
}

enum GoalTypeLocal { GOAL_PARKING_LOCAL, GOAL_PARKED_CAR_LOCAL, GOAL_CHARGE_LOCAL };

Node* select_best_goal_local(
    Agent* agent,
    GridMap* map,
    AgentManager* agents,
    Logger* logger,
    GoalTypeLocal type,
    double* out_cost) {
    (void)logger;
    Node** list = NULL;
    int count = 0;
    int require_parked = -1;
    int check_reserved = 1;
    int toggle_parked = 0;

    switch (type) {
    case GOAL_PARKING_LOCAL:
        list = map->goals;
        count = map->num_goals;
        require_parked = 0;
        break;
    case GOAL_PARKED_CAR_LOCAL:
        list = map->goals;
        count = map->num_goals;
        require_parked = 1;
        toggle_parked = 1;
        break;
    case GOAL_CHARGE_LOCAL:
        list = map->charge_stations;
        count = map->num_charge_stations;
        break;
    }

    return select_best_from_list_local(
        agent, map, agents, list, count, require_parked, check_reserved, toggle_parked, out_cost);
}

Node* select_best_parking_spot_local(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    double best_cost = INF;
    Node* best = select_best_goal_local(agent, map, agents, logger, GOAL_PARKING_LOCAL, &best_cost);
    if (best) {
        logger_log(logger, "[%sPlan%s] Agent %c selected parking goal (%d,%d) (cost %.1f)",
            C_CYN, C_NRM, agent->symbol, best->x, best->y, best_cost);
    }
    return best;
}

Node* select_best_parked_car_local(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    double best_cost = INF;
    Node* best = select_best_goal_local(agent, map, agents, logger, GOAL_PARKED_CAR_LOCAL, &best_cost);
    if (best) {
        logger_log(logger, "[%sPlan%s] Agent %c selected retrieval target (%d,%d) (cost %.1f)",
            C_CYN, C_NRM, agent->symbol, best->x, best->y, best_cost);
    }
    return best;
}

Node* select_best_charge_station_local(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    double best_cost = INF;
    Node* best = select_best_goal_local(agent, map, agents, logger, GOAL_CHARGE_LOCAL, &best_cost);
    if (best) {
        logger_log(logger, "[%sPlan%s] Agent %c selected charge station (%d,%d) (cost: %.1f)",
            C_CYN, C_NRM, agent->symbol, best->x, best->y, best_cost);
    }
    return best;
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

void agent_set_goal_local(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    if (!agent || !agent->pos) {
        agent->goal = NULL;
        agent->state = IDLE;
        return;
    }

    if (agent->state == RETURNING_HOME_EMPTY &&
        agent->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
        if (agent->goal) {
            agent->goal->reserved_by_agent = -1;
            agent->goal = NULL;
        }
        logger_log(logger, "[%sCharge%s] Agent %c exceeded the mileage threshold while returning home. Switching to charge mode.",
            C_B_YEL, C_NRM, agent->symbol);
        agent->state = GOING_TO_CHARGE;
    }

    if (agent->state == IDLE || agent->state == CHARGING || agent->goal) return;

    Node* new_goal = NULL;
    switch (agent->state) {
    case GOING_TO_PARK:
        new_goal = select_best_parking_spot_local(agent, map, agents, logger);
        break;
    case RETURNING_HOME_EMPTY:
    case RETURNING_WITH_CAR:
    case RETURNING_HOME_MAINTENANCE:
        new_goal = agent->home_base;
        break;
    case GOING_TO_COLLECT:
        new_goal = select_best_parked_car_local(agent, map, agents, logger);
        break;
    case GOING_TO_CHARGE:
        new_goal = select_best_charge_station_local(agent, map, agents, logger);
        break;
    default:
        break;
    }

    if (new_goal) {
        if (agent->goal && agent->goal != new_goal) agent->goal->reserved_by_agent = -1;
        agent->goal = new_goal;
        agent->goal->reserved_by_agent = agent->id;
    } else {
        if (agent->state == RETURNING_HOME_EMPTY ||
            agent->state == RETURNING_WITH_CAR ||
            agent->state == RETURNING_HOME_MAINTENANCE) {
            if (!agent->home_base) {
                agent->state = IDLE;
                logger_log(logger, "[%sWarn%s] Agent %c: no home position is configured. Switching to IDLE.",
                    C_B_RED, C_NRM, agent->symbol);
            }
        } else {
            agent->state = IDLE;
            logger_log(logger, "[%sInfo%s] Agent %c: no valid goal found. Waiting.", C_YEL, C_NRM, agent->symbol);
        }
    }
}

}  // namespace

void AgentManager::updateStateAfterMove(ScenarioManager* scenario, GridMap* map, Logger* logger, Simulation* sim) {
    AgentManager* agents = this;

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &agents->agents[i];
        if (agent->state == IDLE || agent->state == CHARGING || !agent->goal || agent->pos != agent->goal) continue;

        if (agent->state == GOING_TO_PARK || agent->state == GOING_TO_COLLECT) {
            if (agent->action_timer <= 0) {
                agent->action_timer = TASK_ACTION_TICKS;
                logger_log(logger, "[%sTask%s] Agent %c, %s task started (%d ticks).", C_YEL, C_NRM, agent->symbol,
                    agent->state == GOING_TO_PARK ? "parking" : "exiting", agent->action_timer);
                continue;
            } else {
                agent->action_timer--;
                if (agent->action_timer > 0) {
                    continue;
                }
            }
        }

        Node* reached = agent->goal;
        if (agent->state != GOING_TO_CHARGE) reached->reserved_by_agent = -1;
        agent->goal = NULL;

        switch (agent->state) {
        case GOING_TO_PARK:
            reached->is_parked = TRUE;
            agents->total_cars_parked++;
            broadcast_cell_change_local(agents, map, reached);
            logger_log(logger, "[%sPark%s] Agent %c parked a vehicle at (%d,%d).", C_GRN, C_NRM, agent->symbol, reached->x, reached->y);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases &&
                scenario->phases[scenario->current_phase_index].type == PARK_PHASE) {
                scenario->tasks_completed_in_phase++;
                if (sim) {
                    int phase_idx = scenario->current_phase_index;
                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
                        sim->phase_completed_tasks[phase_idx]++;
                    }
                    sim->tasks_completed_total++;
                }
            } else if (sim) {
                sim->tasks_completed_total++;
            }
            agent->state = RETURNING_HOME_EMPTY;
            agent->pf.reset();
            break;
        case RETURNING_HOME_EMPTY:
            logger_log(logger, "[%sInfo%s] Agent %c returned home after parking.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            agent->pf.reset();
            metrics_finalize_task_if_active_local(sim, agent);
            break;
        case GOING_TO_COLLECT:
            logger_log(logger, "[%sExit%s] Agent %c picked up a parked vehicle at (%d,%d).", C_YEL, C_NRM, agent->symbol, reached->x, reached->y);
            reached->is_parked = FALSE;
            agents->total_cars_parked--;
            broadcast_cell_change_local(agents, map, reached);
            agent->state = RETURNING_WITH_CAR;
            agent->pf.reset();
            break;
        case RETURNING_WITH_CAR:
            logger_log(logger, "[%sExit%s] Agent %c completed the retrieval task.", C_GRN, C_NRM, agent->symbol);
            if (scenario->mode == MODE_CUSTOM && scenario->current_phase_index < scenario->num_phases &&
                scenario->phases[scenario->current_phase_index].type == EXIT_PHASE) {
                scenario->tasks_completed_in_phase++;
                if (sim) {
                    int phase_idx = scenario->current_phase_index;
                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
                        sim->phase_completed_tasks[phase_idx]++;
                    }
                    sim->tasks_completed_total++;
                }
            } else if (sim) {
                sim->tasks_completed_total++;
            }
            agent->state = IDLE;
            agent->pf.reset();
            metrics_finalize_task_if_active_local(sim, agent);
            break;
        case GOING_TO_CHARGE:
            logger_log(logger, "[%sCharge%s] Agent %c started charging (%d steps).", C_B_YEL, C_NRM, agent->symbol, CHARGE_TIME);
            agent->state = CHARGING;
            agent->charge_timer = CHARGE_TIME;
            if (agent->pos) broadcast_cell_change_local(agents, map, agent->pos);
            break;
        case RETURNING_HOME_MAINTENANCE:
            logger_log(logger, "[%sInfo%s] Agent %c returned home after charging.", C_CYN, C_NRM, agent->symbol);
            agent->state = IDLE;
            agent->pf.reset();
            break;
        default:
            break;
        }
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
        if (agent->state == CHARGING) {
            agent->charge_timer--;
            if (agent->charge_timer <= 0) {
                logger_log(logger, "[%sCharge%s] Agent %c finished charging.", C_B_GRN, C_NRM, agent->symbol);
                agent->total_distance_traveled = 0.0;
                agent->state = RETURNING_HOME_MAINTENANCE;
                if (agent->pos) agent->pos->reserved_by_agent = -1;
                if (agent->pos) broadcast_cell_change_local(agents, map, agent->pos);
                agent->goal = NULL;
                agent->pf.reset();
                agent->stuck_steps = 0;
            }
        }
    }
}

void agent_manager_update_charge_state(AgentManager* manager, GridMap* map, Logger* logger) {
    if (manager) manager->updateChargeState(map, logger);
}

AgentWorkloadSnapshot agv_collect_agent_workload(const AgentManager* agents) {
    return collect_agent_workload_local(agents);
}

Node* agv_select_best_charge_station(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    return select_best_charge_station_local(agent, map, agents, logger);
}

void agv_set_goal_if_needed(Agent* agent, GridMap* map, AgentManager* agents, Logger* logger) {
    if (agent && agent->goal == NULL && agent->state != IDLE && agent->state != CHARGING) {
        agent_set_goal_local(agent, map, agents, logger);
    }
}
