#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_default_support.hpp"

#include <algorithm>
#include <numeric>

#include "collision_planner_support.hpp"
#include "collision_planner_whca_support.hpp"

namespace {

constexpr int kReturnHomeEscapePriorityStuck = 10;
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

bool default_planner_agent_participates_in_deadlock(const Agent* agent) {
    return default_planner_agent_is_active(agent) && !default_planner_agent_is_busy_at_goal(agent);
}

void seed_next_positions_from_current(AgentManager* manager, AgentNodeSlots& next_positions) {
    for (int index = 0; index < MAX_AGENTS; ++index) {
        next_positions[index] = manager->agents[index].pos;
    }
}

void apply_first_step_rotation(AgentManager* manager, AgentNodeSlots& next_positions) {
    if (!manager) return;

    const OrderedRotationPolicy rotation_policy{};
    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_is_active(agent) || !agent->pos || !next_positions[agent_id]) {
            continue;
        }
        if (agent->rotation_wait > 0) {
            next_positions[agent_id] = agent->pos;
            continue;
        }

        Node* rotated_next = agent->pos;
        rotation_policy.apply(agent, agent->pos, next_positions[agent_id], &rotated_next);
        next_positions[agent_id] = rotated_next;
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

void capture_planned_path(const PlanningContext& context, int agent_id, const TimedNodePlan& plan) {
    if (!context.sim) {
        return;
    }
    context.sim->render_model.planner_overlay.planned_paths[agent_id] = plan;
}

void sort_deadlock_agents_by_priority(const AgentManager* manager, AgentOrder& order) {
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(),
        [manager](int lhs, int rhs) {
            return priority_score(&manager->agents[lhs]) > priority_score(&manager->agents[rhs]);
        });
}

int collect_masked_group_by_priority(const AgentManager* manager, AgentMask mask, std::array<int, MAX_CBS_GROUP>& group_ids) {
    group_ids.fill(-1);
    AgentOrder order{};
    sort_deadlock_agents_by_priority(manager, order);
    int group_size = 0;
    for (int order_index = 0; order_index < MAX_AGENTS && group_size < MAX_CBS_GROUP; ++order_index) {
        const int agent_id = order[order_index];
        const Agent* agent = &manager->agents[agent_id];
        if (!mask.contains(agent_id)) continue;
        if (!default_planner_agent_participates_in_deadlock(agent)) continue;
        group_ids[group_size++] = agent_id;
    }
    return group_size;
}

AgentMask collect_deadlock_participant_mask(const AgentManager* manager, AgentMask mask = AgentMask::all()) {
    AgentMask participants{};
    if (!manager) return participants;

    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (!mask.contains(agent_id)) continue;
        if (default_planner_agent_participates_in_deadlock(&manager->agents[agent_id])) {
            participants.set(agent_id);
        }
    }
    return participants;
}

int count_agents_in_mask(const AgentManager* manager, AgentMask mask) {
    int count = 0;
    if (!manager) return count;
    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (mask.contains(agent_id) && default_planner_agent_participates_in_deadlock(&manager->agents[agent_id])) {
            count++;
        }
    }
    return count;
}

bool all_agents_in_mask_waiting(const AgentManager* manager, AgentMask mask, const AgentNodeSlots& next_positions) {
    const AgentMask participants = collect_deadlock_participant_mask(manager, mask);
    if (participants.empty()) return false;

    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        if (!participants.contains(agent_id)) continue;
        const Agent* agent = &manager->agents[agent_id];
        if (next_positions[agent_id] != agent->pos) return false;
    }
    return true;
}

bool all_active_agents_waiting(const AgentManager* manager, const AgentNodeSlots& next_positions) {
    return all_agents_in_mask_waiting(manager, AgentMask::all(), next_positions);
}

bool cbs_solution_has_first_step_progress(
    const AgentManager* manager,
    const std::array<int, MAX_CBS_GROUP>& group_ids,
    int group_size,
    const CbsSolveResult& result) {
    if (!manager || !result.solved) return false;

    for (int group_index = 0; group_index < group_size; ++group_index) {
        const int agent_id = group_ids[group_index];
        if (agent_id < 0) continue;

        const Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_participates_in_deadlock(agent)) continue;

        Node* first_step = result.plans[agent_id][1];
        if (first_step && first_step != agent->pos) {
            return true;
        }
    }

    return false;
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

Node* replan_deadlock_leader_first_step(
    const PlanningContext& context,
    ReservationTable& table,
    Agent& agent) {
    if (!agent.pos || !agent.goal) return agent.pos;

    OrderedPlannerToolkit toolkit(OrderedPlanningMetric::DStar);
    toolkit.preparePathfinder(&agent);
    TemporaryGoalStateScope goal_scope(&agent, agent.pf.get(), context.map, context.agents);

    if (!agent.pf) return agent.pos;

    agent.pf->updateStart(agent.pos);
    agent.pf->computeShortestPath(context.map, context.agents);
    accumulate_whca_dstar_metrics(context, *agent.pf);

    TimedNodePlan plan{};
    plan.fill(agent.pos);
    Node* current = agent.pos;

    for (int step = 1; step <= context.whcaHorizon(); ++step) {
        const OrderedMoveCandidates candidates = toolkit.rankCandidates(agent.pf.get(), context.map, context.agents, current, agent.goal);
        Node* chosen = current;

        for (int candidate_index = 0; candidate_index < candidates.count; ++candidate_index) {
            Node* candidate = candidates.nodes[candidate_index];
            if (table.isOccupied(step, candidate, context.whcaHorizon())) continue;

            const int previous_occupant = table.occupantAt(step - 1, candidate, context.whcaHorizon());
            const int occupant_into_current = table.occupantAt(step, current, context.whcaHorizon());
            if (previous_occupant != -1 && previous_occupant == occupant_into_current) continue;

            chosen = candidate;
            break;
        }

        plan[step] = chosen;
        table.setOccupant(step, chosen, agent.id, context.whcaHorizon());
        current = chosen;

        if (current == agent.goal) {
            reserve_goal_tail(context, table, step + 1, current, agent.id, plan);
            break;
        }
    }

    return plan[1] ? plan[1] : agent.pos;
}

void prepare_return_home_deadlock_escape(Agent* agent) {
    if (!agent || !agent->goal) return;
    if (agent->state != AgentState::ReturningHomeEmpty &&
        agent->state != AgentState::ReturningHomeMaintenance) {
        return;
    }
    if (agent->goal != agent->home_base) return;

    agent->goal->reserved_by_agent = -1;
    agent->goal = nullptr;
    agent->pf.reset();
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
            if (!agent->pos) continue;
            if (agent->rotation_wait > 0) {
                reserveWaitingAgentPath(*agent);
                agent->rotation_wait--;
                continue;
            }
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
        TimedNodePlan plan{};
        plan.fill(agent.pos);
        for (int step = 1; step <= context_.whcaHorizon(); ++step) {
            table_.setOccupant(step, agent.pos, agent.id, context_.whcaHorizon());
        }
        next_positions_[agent.id] = agent.pos;
        capture_planned_path(context_, agent.id, plan);
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
        capture_planned_path(context_, agent.id, plan);
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
        if (all_active_agents_waiting(context_.agents, next_positions_)) {
            breakStandstill(AgentMask::all(), decision,
                "[%sWFG%s] Global standstill: %d agents blocked, leader=%c commits.");
            if (decision.hasAction()) {
                return decision;
            }
        }

        if (summary.hasCycle()) {
            handleSccMask(summary.scc_agents, decision);
            if (!decision.hasAction() &&
                all_agents_in_mask_waiting(context_.agents, summary.scc_agents, next_positions_)) {
                breakStandstill(summary.scc_agents, decision,
                    "[%sWFG%s] SCC standstill: %d agents blocked, leader=%c commits.");
            }
        }

        if (!decision.hasAction()) {
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

    bool tryAcceptCbsSolution(
        const std::array<int, MAX_CBS_GROUP>& group_ids,
        int group_size,
        const CbsSolveResult& result,
        FallbackDecision& decision,
        const char* success_log_message = nullptr) {
        if (!result.solved) {
            return false;
        }

        if (!cbs_solution_has_first_step_progress(context_.agents, group_ids, group_size, result)) {
            logger_log(
                context_.logger,
                "[%sCBS%s] Ignoring zero-progress CBS result (group=%d); escalating deadlock escape.",
                C_B_CYN,
                C_NRM,
                group_size);
            return false;
        }

        applyCbsSolution(group_ids, group_size, result);
        decision.used_cbs = true;
        if (context_.sim) {
            context_.sim->render_model.planner_overlay.cbs_paths = result.plans;
        }
        if (success_log_message) {
            logger_log(context_.logger, success_log_message, C_B_CYN, C_NRM, group_size);
        }
        return true;
    }

    int applyPullOverFallbackForMask(AgentMask mask, FallbackDecision& decision) {
        const AgentMask participants = collect_deadlock_participant_mask(context_.agents, mask);
        const int leader = best_in_mask(context_.agents, participants.raw());
        if (leader < 0) return -1;

        AgentOrder order{};
        sort_deadlock_agents_by_priority(context_.agents, order);

        table_.clearAgent(leader, context_.whcaHorizon());

        for (int order_index = 0; order_index < MAX_AGENTS; ++order_index) {
            const int agent_id = order[order_index];
            if (!participants.contains(agent_id) || agent_id == leader) continue;
            table_.clearAgent(agent_id, context_.whcaHorizon());
        }

        for (int order_index = 0; order_index < MAX_AGENTS; ++order_index) {
            const int agent_id = order[order_index];
            if (!participants.contains(agent_id) || agent_id == leader) continue;
            decision.yield_agents.set(agent_id);

            Node* pull_over = try_pull_over(context_, table_, &context_.agents->agents[agent_id], scratch_);
            if (!pull_over) {
                pull_over = context_.agents->agents[agent_id].pos;
            }

            next_positions_[agent_id] = pull_over;
            table_.setOccupant(1, pull_over, agent_id, context_.whcaHorizon());
            if (pull_over != context_.agents->agents[agent_id].pos) {
                decision.pull_over_agents.set(agent_id);
            } else {
                prepare_return_home_deadlock_escape(&context_.agents->agents[agent_id]);
            }
        }

        next_positions_[leader] = replan_deadlock_leader_first_step(context_, table_, context_.agents->agents[leader]);
        decision.leader = leader;
        return leader;
    }

    void handleSccMask(AgentMask scc_agents, FallbackDecision& decision) {
        const AgentMask participants = collect_deadlock_participant_mask(context_.agents, scc_agents);
        const int active_count = count_agents_in_mask(context_.agents, participants);
        if (active_count < 2) return;

        if (active_count > MAX_CBS_GROUP) {
            const int leader = applyPullOverFallbackForMask(participants, decision);
            if (leader >= 0) {
                logger_log(context_.logger, "[%sWFG%s] Large SCC detected (%d agents): leader=%c commits, others pull over.",
                    C_B_YEL, C_NRM, active_count, context_.agents->agents[leader].symbol);
            }
            return;
        }

        const int group_n = collect_masked_group_by_priority(context_.agents, participants, scratch_.masked_group_ids);
        if (group_n < 2) return;

        const CbsSolveResult result = run_partial_CBS(context_, scratch_.masked_group_ids, group_n, table_, scratch_);
        if (tryAcceptCbsSolution(scratch_.masked_group_ids, group_n, result, decision)) {
            return;
        }

        const int leader = applyPullOverFallbackForMask(participants, decision);
        if (leader >= 0) {
            logger_log(context_.logger, "[%sWFG%s] SCC detected: leader=%c commits, others pull over.",
                C_B_YEL, C_NRM, context_.agents->agents[leader].symbol);
        }
    }

    void breakStandstill(AgentMask mask, FallbackDecision& decision, const char* log_message) {
        const AgentMask participants = collect_deadlock_participant_mask(context_.agents, mask);
        const int active_count = count_agents_in_mask(context_.agents, participants);
        if (active_count < 2) return;

        if (active_count > MAX_CBS_GROUP) {
            const int leader = applyPullOverFallbackForMask(participants, decision);
            if (leader >= 0) {
                logger_log(context_.logger, log_message, C_B_YEL, C_NRM, active_count, context_.agents->agents[leader].symbol);
            }
            return;
        }

        const int group_n = collect_masked_group_by_priority(context_.agents, participants, scratch_.fallback_group_ids);
        if (group_n < 2) return;

        const CbsSolveResult result = run_partial_CBS(context_, scratch_.fallback_group_ids, group_n, table_, scratch_);
        if (tryAcceptCbsSolution(
                scratch_.fallback_group_ids,
                group_n,
                result,
                decision,
                "[%sCBS%s] Deadlock fallback CBS engaged (group=%d).")) {
            return;
        }

        const int leader = applyPullOverFallbackForMask(participants, decision);
        if (leader >= 0) {
            logger_log(context_.logger, log_message, C_B_YEL, C_NRM, active_count, context_.agents->agents[leader].symbol);
        }
    }

public:
    void handleWaitingDeadlock(FallbackDecision& decision) {
        if (!all_active_agents_waiting(context_.agents, next_positions_)) return;
        breakStandstill(AgentMask::all(), decision, "[%sWFG%s] Deadlock fallback: large standstill (%d agents), leader=%c commits.");
    }

    void handlePostConflictStandstill(FallbackDecision& decision) {
        if (!all_active_agents_waiting(context_.agents, next_positions_)) return;
        breakStandstill(AgentMask::all(), decision, "[%sWFG%s] Post-conflict standstill: %d agents blocked, leader=%c commits.");
    }

private:
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
                const bool lhs_yield = decision.yield_agents.contains(lhs);
                const bool rhs_yield = decision.yield_agents.contains(rhs);
                const bool lhs_is_leader = lhs == decision.leader;
                const bool rhs_is_leader = rhs == decision.leader;
                const bool lhs_escape_move = is_return_home_escape_move(&manager_->agents[lhs], next_positions_[lhs]);
                const bool rhs_escape_move = is_return_home_escape_move(&manager_->agents[rhs], next_positions_[rhs]);
                const int priority_lhs = conflict_priority(&manager_->agents[lhs], next_positions_[lhs]);
                const int priority_rhs = conflict_priority(&manager_->agents[rhs], next_positions_[rhs]);

                if (next_positions_[lhs] == next_positions_[rhs]) {
                    resolveVertexConflict(lhs, rhs, lhs_pull_over, rhs_pull_over, lhs_yield, rhs_yield, lhs_is_leader, rhs_is_leader,
                        lhs_escape_move, rhs_escape_move, priority_lhs, priority_rhs);
                } else if (next_positions_[lhs] == manager_->agents[rhs].pos && next_positions_[rhs] == manager_->agents[lhs].pos) {
                    resolveSwapConflict(lhs, rhs, lhs_pull_over, rhs_pull_over, lhs_yield, rhs_yield, lhs_is_leader, rhs_is_leader,
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

    void holdSwapPair(int preferred_agent_id, int other_agent_id, const char* message) {
        logger_log(logger_, message, C_B_RED, C_NRM, manager_->agents[preferred_agent_id].symbol);
        next_positions_[preferred_agent_id] = manager_->agents[preferred_agent_id].pos;
        next_positions_[other_agent_id] = manager_->agents[other_agent_id].pos;
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
        bool lhs_yield,
        bool rhs_yield,
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

        if (lhs_yield != rhs_yield) {
            yieldAgent(lhs_yield ? lhs : rhs, "[%sAvoid%s] Deadlock fallback: Agent %c yields.");
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
        bool lhs_yield,
        bool rhs_yield,
        bool lhs_is_leader,
        bool rhs_is_leader,
        bool lhs_escape_move,
        bool rhs_escape_move,
        int priority_lhs,
        int priority_rhs) {
        if (lhs_is_leader != rhs_is_leader) {
            holdSwapPair(lhs_is_leader ? lhs : rhs, lhs_is_leader ? rhs : lhs,
                "[%sAvoid%s] Swap conflict: Agent %c keeps priority, but both wait this step.");
            return;
        }

        if (lhs_pull_over != rhs_pull_over) {
            holdSwapPair(lhs_pull_over ? lhs : rhs, lhs_pull_over ? rhs : lhs,
                "[%sAvoid%s] Swap conflict: pull-over priority assigned to Agent %c, but both wait this step.");
            return;
        }

        if (lhs_yield != rhs_yield) {
            holdSwapPair(lhs_yield ? rhs : lhs, lhs_yield ? lhs : rhs,
                "[%sAvoid%s] Swap conflict: fallback priority assigned to Agent %c, but both wait this step.");
            return;
        }

        if (lhs_escape_move != rhs_escape_move) {
            holdSwapPair(lhs_escape_move ? lhs : rhs, lhs_escape_move ? rhs : lhs,
                "[%sAvoid%s] Swap conflict: escape priority assigned to Agent %c, but both wait this step.");
            return;
        }

        if ((manager_->agents[lhs].state == AgentState::GoingToPark && manager_->agents[rhs].state == AgentState::ReturningHomeEmpty) ||
            (manager_->agents[rhs].state == AgentState::GoingToPark && manager_->agents[lhs].state == AgentState::ReturningHomeEmpty)) {
            holdSwapPair(priority_lhs >= priority_rhs ? lhs : rhs, priority_lhs >= priority_rhs ? rhs : lhs,
                "[%sAvoid%s] Swap conflict: parking flow keeps priority for Agent %c, but both wait this step.");
            return;
        }

        holdSwapPair(priority_lhs >= priority_rhs ? lhs : rhs, priority_lhs >= priority_rhs ? rhs : lhs,
            "[%sAvoid%s] Swap conflict: Agent %c keeps priority, but both wait this step.");
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
    if (context_.sim) {
        context_.sim->render_model.planner_overlay.clear();
        context_.sim->render_model.planner_overlay.valid = true;
        context_.sim->render_model.planner_overlay.algorithm = PathAlgo::Default;
        context_.sim->render_model.planner_overlay.horizon = context_.whcaHorizon();
    }

    WhcaPlanner whca_planner(context_, next_positions_, scratch_, table_);
    whca_planner.plan();
    apply_first_step_rotation(context_.agents, next_positions_);

    ConflictGraphAnalyzer graph_analyzer(context_.agents, next_positions_, scratch_.wait_edges);
    summary_ = graph_analyzer.analyze();

    CbsFallbackResolver fallback_resolver(context_, next_positions_, scratch_, table_);
    const FallbackDecision decision = fallback_resolver.resolve(summary_);
    if (context_.sim) {
        PlannerOverlayCapture& overlay = context_.sim->render_model.planner_overlay;
        overlay.wait_edges = scratch_.wait_edges;
        overlay.wait_edge_count = summary_.wait_edge_count;
        overlay.scc_agents = summary_.scc_agents;
        overlay.leader_agent_id = decision.leader;
        overlay.used_cbs = decision.used_cbs;
        overlay.yield_agents = decision.yield_agents;
        overlay.pull_over_agents = decision.pull_over_agents;
    }

    FirstStepConflictResolver conflict_resolver(context_.agents, context_.logger, next_positions_);
    conflict_resolver.resolve(decision);

    FallbackDecision post_conflict_decision{};
    fallback_resolver.handlePostConflictStandstill(post_conflict_decision);
    if (post_conflict_decision.hasAction()) {
        if (context_.sim) {
            PlannerOverlayCapture& overlay = context_.sim->render_model.planner_overlay;
            overlay.leader_agent_id = post_conflict_decision.leader;
            overlay.used_cbs = post_conflict_decision.used_cbs;
            overlay.yield_agents = post_conflict_decision.yield_agents;
            overlay.pull_over_agents = post_conflict_decision.pull_over_agents;
        }
        conflict_resolver.resolve(post_conflict_decision);
    }

    WHCA_adjustHorizon(context_, summary_.wait_edge_count, summary_.hasCycle() ? 1 : 0, context_.logger);
}

int DefaultPlannerSession::waitEdgeCount() const {
    return summary_.wait_edge_count;
}

bool DefaultPlannerSession::hasConflictCycle() const {
    return summary_.hasCycle();
}
