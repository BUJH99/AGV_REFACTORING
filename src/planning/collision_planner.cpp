#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <cstring>

#include "agv/internal/engine_internal.hpp"
#include "collision_planner_default_support.hpp"
#include "collision_planner_support.hpp"
#include "collision_planner_whca_support.hpp"

namespace {

PlannerMetricsState& planner_metrics(const PlanningContext& context) {
    return *context.planner_metrics;
}

void accumulate_whca_dstar_metrics(const PlanningContext& context, const Pathfinder& pathfinder) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.whca_dstar_nodes_expanded_this_step += pathfinder.nodes_expanded_this_call;
    metrics.whca_dstar_heap_moves_this_step += pathfinder.heap_moves_this_call;
    metrics.whca_dstar_generated_nodes_this_step += pathfinder.nodes_generated_this_call;
    metrics.whca_dstar_valid_expansions_this_step += pathfinder.valid_expansions_this_call;
}

void accumulate_ordered_planning_metrics(
    const PlanningContext& context,
    OrderedPlanningMetric metric_kind,
    const Pathfinder& pathfinder) {
    PlannerMetricsState& metrics = planner_metrics(context);
    if (metric_kind == OrderedPlanningMetric::AStar) {
        metrics.astar_nodes_expanded_this_step += pathfinder.nodes_expanded_this_call;
        metrics.astar_heap_moves_this_step += pathfinder.heap_moves_this_call;
        metrics.astar_generated_nodes_this_step += pathfinder.nodes_generated_this_call;
        metrics.astar_valid_expansions_this_step += pathfinder.valid_expansions_this_call;
        return;
    }

    metrics.dstar_nodes_expanded_this_step += pathfinder.nodes_expanded_this_call;
    metrics.dstar_heap_moves_this_step += pathfinder.heap_moves_this_call;
    metrics.dstar_generated_nodes_this_step += pathfinder.nodes_generated_this_call;
    metrics.dstar_valid_expansions_this_step += pathfinder.valid_expansions_this_call;
}

void record_wf_scc_metrics(const PlanningContext& context, int wf_edges, int scc) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.wf_edges_last = wf_edges;
    metrics.wf_edges_sum += wf_edges;
    metrics.scc_last = scc;
    metrics.scc_sum += scc;
}
}  // namespace

void agent_manager_plan_and_resolve_collisions_core(const PlanningContext& context, AgentNodeSlots& next_positions) {
    AgentManager* m = context.agents;
    GridMap* map = context.map;
    Logger* lg = context.logger;

    for (int i = 0; i < MAX_AGENTS; i++) next_positions[i] = m->agents[i].pos;

    AgentOrder order{};
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
            default_planner_reserve_waiting_agent_path(context, &rt, agent, next_positions);
            continue;
        }

        default_planner_plan_whca_path_for_agent(context, &rt, wf_edges, &wf_cnt, agent, next_positions);
    }

    default_planner_record_first_step_conflicts(m, next_positions, wf_edges, &wf_cnt);

    const int scc_mask = build_scc_mask_from_edges(wf_edges, wf_cnt);
    record_wf_scc_metrics(context, wf_cnt, scc_mask ? 1 : 0);

    int fallback_leader = -1;
    int pull_over_mask = 0;
    default_planner_apply_fallbacks(context, m, &rt, scc_mask, next_positions, &fallback_leader, &pull_over_mask);

    default_planner_resolve_pairwise_first_step_conflicts(m, lg, next_positions, fallback_leader, pull_over_mask);

    WHCA_adjustHorizon(context, wf_cnt, scc_mask ? 1 : 0, lg);
}

int agent_should_skip_ordered_planning_local(const Agent* agent) {
    return !agent ||
        agent->rotation_wait > 0 ||
        agent->action_timer > 0 ||
        agent->state == AgentState::Idle ||
        agent->state == AgentState::Charging ||
        agent->goal == nullptr ||
        !agent->pos;
}

void run_ordered_planning_core_local(
    const PlanningContext& context,
    OrderedPlanningMetric metric_kind,
    AgentNodeSlots& next_positions) {
    AgentManager* manager = context.agents;
    GridMap* map = context.map;

    AgentOrder order{};
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
        TempObstacleScope temp_scope(agent->pf.get(), map, manager, true);
        temp_scope.markOrderBlockers(manager, order, oi, next_positions);
        int goal_was_parked = temporarily_unpark_goal(agent, agent->pf.get(), map, manager);

        if (agent->pf) {
            desired_move = compute_ordered_pathfinder_move(agent, map, manager, metric_kind);
            accumulate_ordered_planning_metrics(context, metric_kind, *agent->pf);
        }
        restore_temporarily_unparked_goal(agent, agent->pf.get(), map, manager, goal_was_parked);
        apply_rotation_and_step(agent, current_pos, desired_move, &next_positions[agent_id]);
    }

    resolve_conflicts_by_order(manager, order, next_positions);
}

void agent_manager_plan_and_resolve_collisions_astar_core(const PlanningContext& context, AgentNodeSlots& next_positions) {
    run_ordered_planning_core_local(context, OrderedPlanningMetric::AStar, next_positions);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic_core(const PlanningContext& context, AgentNodeSlots& next_positions) {
    run_ordered_planning_core_local(context, OrderedPlanningMetric::DStar, next_positions);
}

void assign_goals_for_active_agents_local(AgentManager* manager, GridMap* map, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        agv_set_goal_if_needed(agent, map, manager, logger);
    }
}

void seed_next_positions_from_current_local(AgentManager* manager, AgentNodeSlots& next_positions) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        next_positions[i] = manager->agents[i].pos;
    }
}

using PlannerCoreFnLocal = void (*)(const PlanningContext& context, AgentNodeSlots& next_positions);

void run_planner_entry_local(
    const PlanningContext& context,
    AgentNodeSlots& next_positions,
    PlannerCoreFnLocal planner_core,
    int seed_next_positions) {
    assign_goals_for_active_agents_local(context.agents, context.map, context.logger);
    if (seed_next_positions) {
        seed_next_positions_from_current_local(context.agents, next_positions);
    }
    planner_core(context, next_positions);
}

void agent_manager_plan_and_resolve_collisions(const PlanningContext& context, AgentNodeSlots& next_positions) {
    run_planner_entry_local(context, next_positions, agent_manager_plan_and_resolve_collisions_core, false);
}

void agent_manager_plan_and_resolve_collisions_astar(const PlanningContext& context, AgentNodeSlots& next_positions) {
    run_planner_entry_local(context, next_positions, agent_manager_plan_and_resolve_collisions_astar_core, true);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic(const PlanningContext& context, AgentNodeSlots& next_positions) {
    run_planner_entry_local(context, next_positions, agent_manager_plan_and_resolve_collisions_dstar_basic_core, true);
}

namespace {

class FunctionPlannerStrategy final : public PlannerStrategy {
public:
    explicit FunctionPlannerStrategy(PlannerCoreFnLocal plan_step)
        : plan_step_(plan_step) {}

    void planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const override {
        plan_step_(context, next_positions);
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
    case PathAlgo::AStarSimple:
        return planner_make_local(agent_manager_plan_and_resolve_collisions_astar);
    case PathAlgo::DStarBasic:
        return planner_make_local(agent_manager_plan_and_resolve_collisions_dstar_basic);
    case PathAlgo::Default:
    default:
        return planner_make_local(agent_manager_plan_and_resolve_collisions);
    }
}
