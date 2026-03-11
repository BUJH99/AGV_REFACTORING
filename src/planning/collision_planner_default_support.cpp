#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_default_support.hpp"

#include "collision_planner_support.hpp"
#include "collision_planner_whca_support.hpp"

namespace {

constexpr int kReturnHomeEscapePriorityStuck = 20;
constexpr int kReturnHomeEscapeBoost = 250;
constexpr int kReturnHomeBayMoveBoost = 1000;

PlannerMetricsState& planner_metrics(const PlanningContext& context) {
    return *context.planner_metrics;
}

void accumulate_whca_dstar_metrics(const PlanningContext& context, const Pathfinder& pathfinder) {
    const PathfinderRunMetrics& metrics_snapshot = pathfinder.lastRunMetrics();
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.whca_dstar_nodes_expanded_this_step += metrics_snapshot.nodes_expanded;
    metrics.whca_dstar_heap_moves_this_step += metrics_snapshot.heap_moves;
    metrics.whca_dstar_generated_nodes_this_step += metrics_snapshot.generated_nodes;
    metrics.whca_dstar_valid_expansions_this_step += metrics_snapshot.valid_expansions;
}

bool default_planner_agent_is_active(const Agent* agent) {
    return agent && agent->state != AgentState::Idle && agent->state != AgentState::Charging && agent->goal != nullptr;
}

bool default_planner_agent_is_busy_at_goal(const Agent* agent) {
    return agent &&
        agent->action_timer > 0 &&
        agent->pos &&
        agent->goal &&
        agent->pos == agent->goal;
}

void seed_next_positions_from_current(AgentManager* manager, AgentNodeSlots& next_positions) {
    for (int index = 0; index < MAX_AGENTS; ++index) {
        next_positions[index] = manager->agents[index].pos;
    }
}

void reserve_goal_tail(
    const PlanningContext& context,
    ReservationTable& table,
    int start_step,
    Node* goal,
    int agent_id,
    TimedNodePlan& plan) {
    for (int step = start_step; step <= context.whcaHorizon(); ++step) {
        table.setOccupant(step, goal, agent_id, context.whcaHorizon());
        plan[step] = goal;
    }
}

int collect_fallback_group(const AgentManager* manager, const AgentOrder& order, std::array<int, MAX_CBS_GROUP>& group_ids) {
    group_ids.fill(-1);
    int group_size = 0;
    for (int order_index = 0; order_index < MAX_AGENTS && group_size < MAX_CBS_GROUP; ++order_index) {
        const int agent_id = order[order_index];
        const Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_is_active(agent) || default_planner_agent_is_busy_at_goal(agent)) continue;
        group_ids[group_size++] = agent_id;
    }
    return group_size;
}

int collect_masked_group(const AgentManager* manager, AgentMask mask, std::array<int, MAX_CBS_GROUP>& group_ids) {
    group_ids.fill(-1);
    int group_size = 0;
    for (int agent_id = 0; agent_id < MAX_AGENTS && group_size < MAX_CBS_GROUP; ++agent_id) {
        const Agent* agent = &manager->agents[agent_id];
        if (!mask.contains(agent_id)) continue;
        if (!default_planner_agent_is_active(agent) || default_planner_agent_is_busy_at_goal(agent)) continue;
        group_ids[group_size++] = agent_id;
    }
    return group_size;
}

bool all_active_agents_waiting(const AgentManager* manager, const AgentNodeSlots& next_positions) {
    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        const Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_is_active(agent)) continue;
        if (next_positions[agent_id] != agent->pos) return false;
    }
    return true;
}

bool is_return_home_escape_move(const Agent* agent, Node* next) {
    return agent &&
        next &&
        agent->state == AgentState::ReturningHomeEmpty &&
        next != agent->pos &&
        next->is_goal &&
        !next->is_parked;
}

int conflict_priority(const Agent* agent, Node* next) {
    if (!agent) return -999999;

    int score = priority_score(agent);
    if (agent->state == AgentState::ReturningHomeEmpty &&
        agent->stuck_steps >= kReturnHomeEscapePriorityStuck) {
        score += kReturnHomeEscapeBoost;
        if (is_return_home_escape_move(agent, next)) {
            score += kReturnHomeBayMoveBoost;
        }
    }
    return score;
}

class WhcaPlanner final {
public:
    WhcaPlanner(
        const PlanningContext& context,
        AgentNodeSlots& next_positions,
        DefaultPlannerScratch& scratch,
        ReservationTable& table)
        : context_(context),
          next_positions_(next_positions),
          scratch_(scratch),
          table_(table) {}

    void plan() {
        seed_next_positions_from_current(context_.agents, next_positions_);
        sort_agents_by_priority(context_.agents, order_);

        for (int order_index = 0; order_index < MAX_AGENTS; ++order_index) {
            Agent* agent = &context_.agents->agents[order_[order_index]];
            if (!default_planner_agent_is_active(agent)) continue;

            if (default_planner_agent_is_busy_at_goal(agent)) {
                reserveWaitingAgentPath(*agent);
                continue;
            }

            planPathForAgent(*agent);
        }
    }

private:
    void reserveWaitingAgentPath(const Agent& agent) {
        for (int step = 1; step <= context_.whcaHorizon(); ++step) {
            table_.setOccupant(step, agent.pos, agent.id, context_.whcaHorizon());
        }
        next_positions_[agent.id] = agent.pos;
    }

    void planPathForAgent(Agent& agent) {
        AgentManager* manager = context_.agents;
        GridMap* map = context_.map;
        toolkit_.preparePathfinder(&agent);
        TemporaryGoalStateScope goal_scope(&agent, agent.pf.get(), map, manager);

        if (agent.pf) {
            agent.pf->updateStart(agent.pos);
            agent.pf->computeShortestPath(map, manager);
            accumulate_whca_dstar_metrics(context_, *agent.pf);
        }

        TimedNodePlan plan{};
        plan.fill(agent.pos);
        Node* current = agent.pos;

        for (int step = 1; step <= context_.whcaHorizon(); ++step) {
            const OrderedMoveCandidates candidates = toolkit_.rankCandidates(agent.pf.get(), map, manager, current, agent.pf->goalNode());

            Node* chosen = current;
            for (int candidate_index = 0; candidate_index < candidates.count; ++candidate_index) {
                Node* candidate = candidates.nodes[candidate_index];
                if (table_.isOccupied(step, candidate, context_.whcaHorizon())) {
                    const int occupant = table_.occupantAt(step, candidate, context_.whcaHorizon());
                    if (occupant != -1) {
                        scratch_.wait_edges.add(agent.id, occupant, step, CauseType::Vertex, candidate->x, candidate->y, 0, 0);
                    }
                    continue;
                }

                const int previous_occupant = table_.occupantAt(step - 1, candidate, context_.whcaHorizon());
                const int occupant_into_current = table_.occupantAt(step, current, context_.whcaHorizon());
                if (previous_occupant != -1 && previous_occupant == occupant_into_current) {
                    scratch_.wait_edges.add(
                        agent.id,
                        previous_occupant,
                        step,
                        CauseType::Swap,
                        current->x,
                        current->y,
                        candidate->x,
                        candidate->y);
                    continue;
                }

                if (chosen == current) {
                    chosen = candidate;
                }
            }

            plan[step] = chosen;
            table_.setOccupant(step, chosen, agent.id, context_.whcaHorizon());
            current = chosen;

            if (current == agent.goal) {
                reserve_goal_tail(context_, table_, step + 1, current, agent.id, plan);
                break;
            }
        }

        next_positions_[agent.id] = plan[1] ? plan[1] : agent.pos;
    }

    const PlanningContext& context_;
    AgentNodeSlots& next_positions_;
    DefaultPlannerScratch& scratch_;
    ReservationTable& table_;
    AgentOrder order_{};
    OrderedPlannerToolkit toolkit_{OrderedPlanningMetric::DStar};
};

class ConflictGraphAnalyzer final {
public:
    ConflictGraphAnalyzer(
        const AgentManager* manager,
        const AgentNodeSlots& next_positions,
        WaitEdgeBuffer& wait_edges)
        : manager_(manager),
          next_positions_(next_positions),
          wait_edges_(wait_edges) {}

    ConflictGraphSummary analyze() {
        recordFirstStepConflicts();
        ConflictGraphSummary summary = analyze_conflict_graph(wait_edges_);
        summary.wait_edge_count = wait_edges_.count;
        return summary;
    }

private:
    void recordFirstStepConflicts() {
        for (int lhs = 0; lhs < MAX_AGENTS; ++lhs) {
            if (!default_planner_agent_is_active(&manager_->agents[lhs]) || !next_positions_[lhs]) continue;
            for (int rhs = lhs + 1; rhs < MAX_AGENTS; ++rhs) {
                if (!default_planner_agent_is_active(&manager_->agents[rhs]) || !next_positions_[rhs]) continue;

                if (next_positions_[lhs] == next_positions_[rhs]) {
                    wait_edges_.add(lhs, rhs, 1, CauseType::Vertex, next_positions_[lhs]->x, next_positions_[lhs]->y, 0, 0);
                    wait_edges_.add(rhs, lhs, 1, CauseType::Vertex, next_positions_[rhs]->x, next_positions_[rhs]->y, 0, 0);
                } else if (next_positions_[lhs] == manager_->agents[rhs].pos && next_positions_[rhs] == manager_->agents[lhs].pos) {
                    wait_edges_.add(
                        lhs,
                        rhs,
                        1,
                        CauseType::Swap,
                        manager_->agents[lhs].pos ? manager_->agents[lhs].pos->x : -1,
                        manager_->agents[lhs].pos ? manager_->agents[lhs].pos->y : -1,
                        next_positions_[lhs]->x,
                        next_positions_[lhs]->y);
                    wait_edges_.add(
                        rhs,
                        lhs,
                        1,
                        CauseType::Swap,
                        manager_->agents[rhs].pos ? manager_->agents[rhs].pos->x : -1,
                        manager_->agents[rhs].pos ? manager_->agents[rhs].pos->y : -1,
                        next_positions_[rhs]->x,
                        next_positions_[rhs]->y);
                }
            }
        }
    }

    const AgentManager* manager_{nullptr};
    const AgentNodeSlots& next_positions_;
    WaitEdgeBuffer& wait_edges_;
};

class CbsFallbackResolver final {
public:
    CbsFallbackResolver(
        const PlanningContext& context,
        AgentNodeSlots& next_positions,
        DefaultPlannerScratch& scratch,
        ReservationTable& table)
        : context_(context),
          next_positions_(next_positions),
          scratch_(scratch),
          table_(table) {}

    FallbackDecision resolve(const ConflictGraphSummary& summary) {
        FallbackDecision decision{};
        if (summary.hasCycle()) {
            handleSccMask(summary.scc_agents, decision);
        } else {
            handleWaitingDeadlock(decision);
        }
        return decision;
    }

private:
    void applyCbsSolution(const std::array<int, MAX_CBS_GROUP>& group_ids, int group_size, const CbsSolveResult& result) {
        for (int group_index = 0; group_index < group_size; ++group_index) {
            const int agent_id = group_ids[group_index];
            if (agent_id < 0) continue;
            if (result.plans[agent_id][1]) {
                next_positions_[agent_id] = result.plans[agent_id][1];
            }
        }
    }

    int applyPullOverFallback(
        const std::array<int, MAX_CBS_GROUP>& group_ids,
        int group_size,
        AgentMask leader_mask,
        FallbackDecision& decision) {
        const int leader = best_in_mask(context_.agents, leader_mask.raw());

        for (int group_index = 0; group_index < group_size; ++group_index) {
            const int agent_id = group_ids[group_index];
            if (agent_id < 0 || agent_id == leader) continue;
            table_.clearAgent(agent_id, context_.whcaHorizon());
        }

        for (int group_index = 0; group_index < group_size; ++group_index) {
            const int agent_id = group_ids[group_index];
            if (agent_id < 0 || agent_id == leader) continue;
            Node* pull_over = try_pull_over(context_, table_, &context_.agents->agents[agent_id], scratch_);
            if (pull_over) {
                next_positions_[agent_id] = pull_over;
                table_.setOccupant(1, pull_over, agent_id, context_.whcaHorizon());
                if (pull_over != context_.agents->agents[agent_id].pos) {
                    decision.pull_over_agents.set(agent_id);
                }
            }
        }

        decision.leader = leader;
        return leader;
    }

    void handleSccMask(AgentMask scc_agents, FallbackDecision& decision) {
        const int group_n = collect_masked_group(context_.agents, scc_agents, scratch_.masked_group_ids);
        if (group_n < 2) return;

        const CbsSolveResult result = run_partial_CBS(context_, scratch_.masked_group_ids, group_n, table_, scratch_);
        if (result.solved) {
            applyCbsSolution(scratch_.masked_group_ids, group_n, result);
            decision.used_cbs = true;
            return;
        }

        const int leader = applyPullOverFallback(scratch_.masked_group_ids, group_n, scc_agents, decision);
        if (leader >= 0) {
            logger_log(context_.logger, "[%sWFG%s] SCC detected: leader=%c commits, others pull over.",
                C_B_YEL, C_NRM, context_.agents->agents[leader].symbol);
        }
    }

    void handleWaitingDeadlock(FallbackDecision& decision) {
        if (!all_active_agents_waiting(context_.agents, next_positions_)) return;

        int active_count = 0;
        for (int index = 0; index < MAX_AGENTS; ++index) {
            if (default_planner_agent_is_active(&context_.agents->agents[index])) {
                active_count++;
            }
        }
        if (active_count < 2) return;

        AgentOrder fallback_order{};
        sort_agents_by_priority(context_.agents, fallback_order);
        const int group_n = collect_fallback_group(context_.agents, fallback_order, scratch_.fallback_group_ids);
        if (group_n < 2) return;

        const CbsSolveResult result = run_partial_CBS(context_, scratch_.fallback_group_ids, group_n, table_, scratch_);
        if (result.solved) {
            applyCbsSolution(scratch_.fallback_group_ids, group_n, result);
            decision.used_cbs = true;
            logger_log(context_.logger, "[%sCBS%s] Deadlock fallback CBS engaged (group=%d).", C_B_CYN, C_NRM, group_n);
            return;
        }

        applyPullOverFallback(scratch_.fallback_group_ids, group_n, AgentMask::all(), decision);
        logger_log(context_.logger, "[%sWFG%s] Deadlock fallback: leader-only move, others pull-over.", C_B_YEL, C_NRM);
    }

    const PlanningContext& context_;
    AgentNodeSlots& next_positions_;
    DefaultPlannerScratch& scratch_;
    ReservationTable& table_;
};

class FirstStepConflictResolver final {
public:
    FirstStepConflictResolver(
        AgentManager* manager,
        Logger* logger,
        AgentNodeSlots& next_positions)
        : manager_(manager),
          logger_(logger),
          next_positions_(next_positions) {}

    void resolve(const FallbackDecision& decision) {
        for (int lhs = 0; lhs < MAX_AGENTS; ++lhs) {
            for (int rhs = lhs + 1; rhs < MAX_AGENTS; ++rhs) {
                if (manager_->agents[lhs].state == AgentState::Idle || manager_->agents[rhs].state == AgentState::Idle ||
                    manager_->agents[lhs].state == AgentState::Charging || manager_->agents[rhs].state == AgentState::Charging) {
                    continue;
                }

                const bool lhs_pull_over = decision.pull_over_agents.contains(lhs);
                const bool rhs_pull_over = decision.pull_over_agents.contains(rhs);
                const bool lhs_is_leader = lhs == decision.leader;
                const bool rhs_is_leader = rhs == decision.leader;
                const bool lhs_escape_move = is_return_home_escape_move(&manager_->agents[lhs], next_positions_[lhs]);
                const bool rhs_escape_move = is_return_home_escape_move(&manager_->agents[rhs], next_positions_[rhs]);
                const int priority_lhs = conflict_priority(&manager_->agents[lhs], next_positions_[lhs]);
                const int priority_rhs = conflict_priority(&manager_->agents[rhs], next_positions_[rhs]);

                if (next_positions_[lhs] == next_positions_[rhs]) {
                    resolveVertexConflict(lhs, rhs, lhs_pull_over, rhs_pull_over, lhs_is_leader, rhs_is_leader,
                        lhs_escape_move, rhs_escape_move, priority_lhs, priority_rhs);
                } else if (next_positions_[lhs] == manager_->agents[rhs].pos && next_positions_[rhs] == manager_->agents[lhs].pos) {
                    resolveSwapConflict(lhs, rhs, lhs_pull_over, rhs_pull_over, lhs_is_leader, rhs_is_leader,
                        lhs_escape_move, rhs_escape_move, priority_lhs, priority_rhs);
                }
            }
        }
    }

private:
    void keepLeaderMove(int winner, int loser, const char* message) {
        logger_log(logger_, message, C_B_RED, C_NRM, manager_->agents[winner].symbol);
        next_positions_[loser] = manager_->agents[loser].pos;
    }

    void yieldAgent(int agent_id, const char* message) {
        logger_log(logger_, message, C_B_RED, C_NRM, manager_->agents[agent_id].symbol);
        next_positions_[agent_id] = manager_->agents[agent_id].pos;
    }

    void resolveParkingVsReturnConflict(int lhs, int rhs, int priority_lhs, int priority_rhs, const char* wait_message, const char* starvation_message) {
        const bool return_lhs = manager_->agents[lhs].state == AgentState::ReturningHomeEmpty;
        const Agent* returning_agent = return_lhs ? &manager_->agents[lhs] : &manager_->agents[rhs];
        if (returning_agent->stuck_steps < kReturnHomeEscapePriorityStuck) {
            yieldAgent(return_lhs ? lhs : rhs, wait_message);
        } else if (priority_lhs >= priority_rhs) {
            yieldAgent(rhs, starvation_message);
        } else {
            yieldAgent(lhs, starvation_message);
        }
    }

    void resolveVertexConflict(
        int lhs,
        int rhs,
        bool lhs_pull_over,
        bool rhs_pull_over,
        bool lhs_is_leader,
        bool rhs_is_leader,
        bool lhs_escape_move,
        bool rhs_escape_move,
        int priority_lhs,
        int priority_rhs) {
        if (lhs_is_leader != rhs_is_leader) {
            keepLeaderMove(lhs_is_leader ? lhs : rhs, lhs_is_leader ? rhs : lhs,
                "[%sAvoid%s] Deadlock leader Agent %c keeps the vertex.");
            return;
        }

        if (lhs_pull_over != rhs_pull_over) {
            yieldAgent(lhs_pull_over ? rhs : lhs, "[%sAvoid%s] Pull-over escape: Agent %c yields.");
            return;
        }

        if (lhs_escape_move != rhs_escape_move) {
            yieldAgent(lhs_escape_move ? rhs : lhs, "[%sAvoid%s] Escape move: Agent %c yields.");
            return;
        }

        if ((manager_->agents[lhs].state == AgentState::GoingToPark && manager_->agents[rhs].state == AgentState::ReturningHomeEmpty) ||
            (manager_->agents[rhs].state == AgentState::GoingToPark && manager_->agents[lhs].state == AgentState::ReturningHomeEmpty)) {
            resolveParkingVsReturnConflict(lhs, rhs, priority_lhs, priority_rhs,
                "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.",
                "[%sAvoid%s] Vertex conflict: starvation escape gives Agent %c priority.");
            return;
        }

        yieldAgent(priority_lhs >= priority_rhs ? rhs : lhs, "[%sAvoid%s] Vertex conflict: Agent %c yields.");
    }

    void resolveSwapConflict(
        int lhs,
        int rhs,
        bool lhs_pull_over,
        bool rhs_pull_over,
        bool lhs_is_leader,
        bool rhs_is_leader,
        bool lhs_escape_move,
        bool rhs_escape_move,
        int priority_lhs,
        int priority_rhs) {
        if (lhs_is_leader != rhs_is_leader) {
            keepLeaderMove(lhs_is_leader ? lhs : rhs, lhs_is_leader ? rhs : lhs,
                "[%sAvoid%s] Deadlock leader Agent %c keeps the swap.");
            return;
        }

        if (lhs_pull_over != rhs_pull_over) {
            yieldAgent(lhs_pull_over ? rhs : lhs, "[%sAvoid%s] Pull-over escape: Agent %c yields the swap.");
            return;
        }

        if (lhs_escape_move != rhs_escape_move) {
            yieldAgent(lhs_escape_move ? rhs : lhs, "[%sAvoid%s] Escape move: Agent %c yields the swap.");
            return;
        }

        if ((manager_->agents[lhs].state == AgentState::GoingToPark && manager_->agents[rhs].state == AgentState::ReturningHomeEmpty) ||
            (manager_->agents[rhs].state == AgentState::GoingToPark && manager_->agents[lhs].state == AgentState::ReturningHomeEmpty)) {
            resolveParkingVsReturnConflict(lhs, rhs, priority_lhs, priority_rhs,
                "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.",
                "[%sAvoid%s] Swap conflict: starvation escape gives Agent %c priority.");
            return;
        }

        yieldAgent(priority_lhs >= priority_rhs ? rhs : lhs, "[%sAvoid%s] Swap conflict: Agent %c yields.");
    }

    AgentManager* manager_{nullptr};
    Logger* logger_{nullptr};
    AgentNodeSlots& next_positions_;
};

}  // namespace

DefaultPlannerSession::DefaultPlannerSession(
    const PlanningContext& context,
    AgentNodeSlots& next_positions,
    DefaultPlannerScratch& scratch)
    : context_(context),
      next_positions_(next_positions),
      scratch_(scratch) {}

void DefaultPlannerSession::execute() {
    scratch_.clear();
    table_.clear();
    table_.seedCurrent(context_.agents);

    WhcaPlanner whca_planner(context_, next_positions_, scratch_, table_);
    whca_planner.plan();

    ConflictGraphAnalyzer graph_analyzer(context_.agents, next_positions_, scratch_.wait_edges);
    summary_ = graph_analyzer.analyze();

    CbsFallbackResolver fallback_resolver(context_, next_positions_, scratch_, table_);
    const FallbackDecision decision = fallback_resolver.resolve(summary_);

    FirstStepConflictResolver conflict_resolver(context_.agents, context_.logger, next_positions_);
    conflict_resolver.resolve(decision);

    WHCA_adjustHorizon(context_, summary_.wait_edge_count, summary_.hasCycle() ? 1 : 0, context_.logger);
}

int DefaultPlannerSession::waitEdgeCount() const {
    return summary_.wait_edge_count;
}

bool DefaultPlannerSession::hasConflictCycle() const {
    return summary_.hasCycle();
}
