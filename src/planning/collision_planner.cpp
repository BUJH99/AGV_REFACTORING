#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <cstring>

#include "agv/internal/engine_internal.hpp"
#include "collision_planner_default_support.hpp"
#include "collision_planner_support.hpp"
#include "collision_planner_whca_support.hpp"

#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#define C_B_CYN "\x1b[1;36m"
#endif

int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const struct Agent_* agent);
Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
void pathfinder_update_start(Pathfinder* pf, Node* new_start);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current_node);
void logger_log(Logger* logger, const char* format, ...);

namespace {
}  // namespace

void agent_manager_plan_and_resolve_collisions_core(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) next_pos[i] = m->agents[i].pos;

    int order[MAX_AGENTS];
    sort_agents_by_priority(m, order);

    ReservationTable rt;
    ReservationTable_clear(&rt);
    ReservationTable_seedCurrent(&rt, m);
    WaitEdge wf_edges[kPlannerMaxWaitEdges];
    int wf_cnt = 0;

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        Agent* agent = &m->agents[order[oi]];
        if (!default_planner_agent_is_active(agent)) continue;

        if (default_planner_agent_is_busy_at_goal(agent)) {
            default_planner_reserve_waiting_agent_path(&rt, agent, next_pos);
            continue;
        }

        default_planner_plan_whca_path_for_agent(m, map, &rt, wf_edges, &wf_cnt, agent, next_pos);
    }

    default_planner_record_first_step_conflicts(m, next_pos, wf_edges, &wf_cnt);

    const int scc_mask = build_scc_mask_from_edges(wf_edges, wf_cnt);
    agv_record_wf_scc_metrics(wf_cnt, scc_mask ? 1 : 0);

    default_planner_apply_fallbacks(m, map, lg, &rt, scc_mask, next_pos);

    default_planner_resolve_pairwise_first_step_conflicts(m, lg, next_pos);

    WHCA_adjustHorizon(wf_cnt, scc_mask ? 1 : 0, lg);
}

int agent_should_skip_ordered_planning_local(const Agent* agent) {
    return !agent ||
        agent->rotation_wait > 0 ||
        agent->action_timer > 0 ||
        agent->state == IDLE ||
        agent->state == CHARGING ||
        agent->goal == nullptr ||
        !agent->pos;
}

void run_ordered_planning_core_local(
    AgentManager* manager,
    GridMap* map,
    OrderedPlanningMetric metric_kind,
    Node* next_pos[MAX_AGENTS]) {
    int order[MAX_AGENTS];
    sort_agents_by_priority(manager, order);

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        const int agent_id = order[oi];
        Agent* agent = &manager->agents[agent_id];
        Node* current_pos = agent->pos;

        if (agent->rotation_wait > 0) {
            agent->rotation_wait--;
            continue;
        }
        if (agent_should_skip_ordered_planning_local(agent)) continue;

        Node* desired_move = current_pos;
        ensure_pathfinder_for_agent(agent);
        TempObstacleScope temp_scope(agent->pf, map, manager, true);
        temp_scope.markOrderBlockers(manager, order, oi, next_pos);
        int goal_was_parked = temporarily_unpark_goal(agent, agent->pf, map, manager);

        if (agent->pf) {
            desired_move = compute_ordered_pathfinder_move(agent, map, manager, metric_kind);
        }
        restore_temporarily_unparked_goal(agent, agent->pf, map, manager, goal_was_parked);
        apply_rotation_and_step(agent, current_pos, desired_move, &next_pos[agent_id]);
    }

    resolve_conflicts_by_order(manager, order, next_pos);
}

void agent_manager_plan_and_resolve_collisions_astar_core(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    (void)logger;
    run_ordered_planning_core_local(manager, map, ORDERED_PLANNING_ASTAR, next_pos);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic_core(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    (void)logger;
    run_ordered_planning_core_local(manager, map, ORDERED_PLANNING_DSTAR, next_pos);
}

void assign_goals_for_active_agents_local(AgentManager* manager, GridMap* map, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        agv_set_goal_if_needed(agent, map, manager, logger);
    }
}

void seed_next_positions_from_current_local(AgentManager* manager, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        next_pos[i] = manager->agents[i].pos;
    }
}

using PlannerCoreFnLocal = void (*)(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]);

void run_planner_entry_local(
    AgentManager* manager,
    GridMap* map,
    Logger* logger,
    Node* next_pos[MAX_AGENTS],
    PlannerCoreFnLocal planner_core,
    int seed_next_positions) {
    assign_goals_for_active_agents_local(manager, map, logger);
    if (seed_next_positions) {
        seed_next_positions_from_current_local(manager, next_pos);
    }
    planner_core(manager, map, logger, next_pos);
}

void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    run_planner_entry_local(manager, map, logger, next_pos, agent_manager_plan_and_resolve_collisions_core, FALSE);
}

void agent_manager_plan_and_resolve_collisions_astar(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    run_planner_entry_local(manager, map, logger, next_pos, agent_manager_plan_and_resolve_collisions_astar_core, TRUE);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    run_planner_entry_local(manager, map, logger, next_pos, agent_manager_plan_and_resolve_collisions_dstar_basic_core, TRUE);
}

namespace {

class FunctionPlannerStrategy final : public PlannerStrategy {
public:
    explicit FunctionPlannerStrategy(PlannerCoreFnLocal plan_step)
        : plan_step_(plan_step) {}

    void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const override {
        plan_step_(agents, map, logger, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<FunctionPlannerStrategy>(plan_step_);
    }

private:
    PlannerCoreFnLocal plan_step_{nullptr};
};

Planner planner_make_local(PlannerCoreFnLocal plan_step) {
    return Planner(std::make_unique<FunctionPlannerStrategy>(plan_step));
}

}  // namespace

Planner planner_from_pathalgo(PathAlgo algo) {
    switch (algo) {
    case PATHALGO_ASTAR_SIMPLE:
        return planner_make_local(agent_manager_plan_and_resolve_collisions_astar);
    case PATHALGO_DSTAR_BASIC:
        return planner_make_local(agent_manager_plan_and_resolve_collisions_dstar_basic);
    case PATHALGO_DEFAULT:
    default:
        return planner_make_local(agent_manager_plan_and_resolve_collisions);
    }
}
