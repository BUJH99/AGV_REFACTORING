#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"
#include "collision_planner_default_support.hpp"
#include "collision_planner_support.hpp"

namespace {

PlannerMetricsState& planner_metrics(const PlanningContext& context) {
    return *context.planner_metrics;
}

void accumulate_ordered_planning_metrics(
    const PlanningContext& context,
    OrderedPlanningMetric metric_kind,
    const Pathfinder& pathfinder) {
    const PathfinderRunMetrics& run_metrics = pathfinder.lastRunMetrics();
    PlannerMetricsState& metrics = planner_metrics(context);
    if (metric_kind == OrderedPlanningMetric::AStar) {
        metrics.astar_nodes_expanded_this_step += run_metrics.nodes_expanded;
        metrics.astar_heap_moves_this_step += run_metrics.heap_moves;
        metrics.astar_generated_nodes_this_step += run_metrics.generated_nodes;
        metrics.astar_valid_expansions_this_step += run_metrics.valid_expansions;
        return;
    }

    metrics.dstar_nodes_expanded_this_step += run_metrics.nodes_expanded;
    metrics.dstar_heap_moves_this_step += run_metrics.heap_moves;
    metrics.dstar_generated_nodes_this_step += run_metrics.generated_nodes;
    metrics.dstar_valid_expansions_this_step += run_metrics.valid_expansions;
}

void record_wf_scc_metrics(const PlanningContext& context, int wf_edges, int scc) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.wf_edges_last = wf_edges;
    metrics.wf_edges_sum += wf_edges;
    metrics.scc_last = scc;
    metrics.scc_sum += scc;
}

bool should_skip_ordered_planning(const Agent* agent) {
    return !agent ||
        agent->rotation_wait > 0 ||
        agent->action_timer > 0 ||
        agent->state == AgentState::Idle ||
        agent->state == AgentState::Charging ||
        agent->goal == nullptr ||
        !agent->pos;
}

void assign_goals_for_active_agents(AgentManager* manager, GridMap* map, Logger* logger) {
    for (int index = 0; index < MAX_AGENTS; ++index) {
        agv_set_goal_if_needed(&manager->agents[index], map, manager, logger);
    }
}

void seed_next_positions_from_current(AgentManager* manager, AgentNodeSlots& next_positions) {
    for (int index = 0; index < MAX_AGENTS; ++index) {
        next_positions[index] = manager->agents[index].pos;
    }
}

class OrderedPlannerBase : public PlannerStrategy {
public:
    explicit OrderedPlannerBase(OrderedPlanningMetric metric_kind)
        : metric_kind_(metric_kind),
          toolkit_(metric_kind) {}

    void planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const override {
        assign_goals_for_active_agents(context.agents, context.map, context.logger);
        seed_next_positions_from_current(context.agents, next_positions);
        runOrderedPlanning(context, next_positions);
    }

protected:
    void runOrderedPlanning(const PlanningContext& context, AgentNodeSlots& next_positions) const {
        AgentManager* manager = context.agents;
        GridMap* map = context.map;

        AgentOrder order{};
        sort_agents_by_priority(manager, order);

        for (int order_index = 0; order_index < MAX_AGENTS; ++order_index) {
            const int agent_id = order[order_index];
            Agent* agent = &manager->agents[agent_id];
            Node* current_pos = agent->pos;

            if (agent->rotation_wait > 0) {
                agent->rotation_wait--;
                continue;
            }
            if (should_skip_ordered_planning(agent)) continue;

            Node* desired_move = current_pos;
            toolkit_.preparePathfinder(agent);
            TempObstacleScope temp_scope(agent->pf.get(), map, manager, true);
            temp_scope.markOrderBlockers(manager, order, order_index, next_positions);
            TemporaryGoalStateScope goal_scope(agent, agent->pf.get(), map, manager);

            if (agent->pf) {
                desired_move = toolkit_.computeDesiredMove(agent, map, manager);
                accumulate_ordered_planning_metrics(context, metric_kind_, *agent->pf);
            }
            toolkit_.applyRotation(agent, current_pos, desired_move, &next_positions[agent_id]);
        }

        conflict_resolution_.resolve(manager, order, next_positions);
    }

private:
    OrderedPlanningMetric metric_kind_;
    OrderedPlannerToolkit toolkit_;
    mutable ConflictResolutionPolicy conflict_resolution_{};
};

class AStarOrderedPlanner final : public OrderedPlannerBase {
public:
    AStarOrderedPlanner()
        : OrderedPlannerBase(OrderedPlanningMetric::AStar) {}

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<AStarOrderedPlanner>();
    }
};

class DStarOrderedPlanner final : public OrderedPlannerBase {
public:
    DStarOrderedPlanner()
        : OrderedPlannerBase(OrderedPlanningMetric::DStar) {}

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DStarOrderedPlanner>();
    }
};

class DefaultPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const override {
        assign_goals_for_active_agents(context.agents, context.map, context.logger);

        if (!scratch_) {
            scratch_ = std::make_unique<DefaultPlannerScratch>();
        }

        DefaultPlannerSession session(context, next_positions, *scratch_);
        session.execute();
        record_wf_scc_metrics(context, session.waitEdgeCount(), session.hasConflictCycle() ? 1 : 0);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DefaultPlannerStrategy>();
    }

private:
    mutable std::unique_ptr<DefaultPlannerScratch> scratch_{};
};

}  // namespace

Planner planner_from_pathalgo(PathAlgo algo) {
    switch (algo) {
    case PathAlgo::AStarSimple:
        return Planner(std::make_unique<AStarOrderedPlanner>());
    case PathAlgo::DStarBasic:
        return Planner(std::make_unique<DStarOrderedPlanner>());
    case PathAlgo::Default:
    default:
        return Planner(std::make_unique<DefaultPlannerStrategy>());
    }
}
