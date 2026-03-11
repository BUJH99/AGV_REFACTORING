#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_default_support.hpp"

#include "collision_planner_support.hpp"
#include "collision_planner_whca_support.hpp"

namespace {

constexpr int kMaxCbsGroup = 8;
constexpr int kReturnHomeEscapePriorityStuck = 20;
constexpr int kReturnHomeEscapeBoost = 250;
constexpr int kReturnHomeBayMoveBoost = 1000;

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

void reserve_goal_tail_local(
    const PlanningContext& context,
    ReservationTable* table,
    int start_step,
    Node* goal,
    int agent_id,
    Node* plan[MAX_WHCA_HORIZON + 1]) {
    for (int step = start_step; step <= context.whcaHorizon(); ++step) {
        ReservationTable_setOccupant(table, step, goal, agent_id, context.whcaHorizon());
        plan[step] = goal;
    }
}

int collect_fallback_group_local(const AgentManager* manager, const AgentOrder& order, int group_ids[kMaxCbsGroup]) {
    int group_size = 0;
    for (int order_index = 0; order_index < MAX_AGENTS && group_size < kMaxCbsGroup; ++order_index) {
        const int agent_id = order[order_index];
        const Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_is_active(agent) || default_planner_agent_is_busy_at_goal(agent)) continue;
        group_ids[group_size++] = agent_id;
    }
    return group_size;
}

int collect_masked_group_local(const AgentManager* manager, int mask, int group_ids[kMaxCbsGroup]) {
    int group_size = 0;
    for (int agent_id = 0; agent_id < MAX_AGENTS && group_size < kMaxCbsGroup; ++agent_id) {
        const Agent* agent = &manager->agents[agent_id];
        if ((mask & (1 << agent_id)) == 0) continue;
        if (!default_planner_agent_is_active(agent) || default_planner_agent_is_busy_at_goal(agent)) continue;
        group_ids[group_size++] = agent_id;
    }
    return group_size;
}

void apply_cbs_solution_local(const int group_ids[], int group_size, Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], AgentNodeSlots& next_positions) {
    for (int group_index = 0; group_index < group_size; ++group_index) {
        const int agent_id = group_ids[group_index];
        if (cbs_plans[agent_id][1]) {
            next_positions[agent_id] = cbs_plans[agent_id][1];
        }
    }
}

int apply_pull_over_fallback_local(
    const PlanningContext& context,
    AgentManager* manager,
    ReservationTable* table,
    const int group_ids[],
    int group_size,
    int leader_mask,
    AgentNodeSlots& next_positions,
    int* out_pull_over_mask) {
    const int leader = best_in_mask(manager, leader_mask);
    if (out_pull_over_mask) *out_pull_over_mask = 0;

    for (int group_index = 0; group_index < group_size; ++group_index) {
        const int agent_id = group_ids[group_index];
        if (agent_id == leader) continue;
        ReservationTable_clearAgent(table, agent_id, context.whcaHorizon());
    }

    for (int group_index = 0; group_index < group_size; ++group_index) {
        const int agent_id = group_ids[group_index];
        if (agent_id == leader) continue;
        Node* pull_over = try_pull_over(context, table, &manager->agents[agent_id]);
        if (pull_over) {
            next_positions[agent_id] = pull_over;
            ReservationTable_setOccupant(table, 1, pull_over, agent_id, context.whcaHorizon());
            if (out_pull_over_mask && pull_over != manager->agents[agent_id].pos) {
                *out_pull_over_mask |= (1 << agent_id);
            }
        }
    }
    return leader;
}

int all_active_agents_waiting_local(const AgentManager* manager, const AgentNodeSlots& next_positions) {
    for (int agent_id = 0; agent_id < MAX_AGENTS; ++agent_id) {
        const Agent* agent = &manager->agents[agent_id];
        if (!default_planner_agent_is_active(agent)) continue;
        if (next_positions[agent_id] != agent->pos) {
            return 0;
        }
    }
    return 1;
}

int is_return_home_escape_move_local(const Agent* agent, Node* next) {
    return agent &&
        next &&
        agent->state == AgentState::ReturningHomeEmpty &&
        next != agent->pos &&
        next->is_goal &&
        !next->is_parked;
}

int conflict_priority_local(const Agent* agent, Node* next) {
    if (!agent) return -999999;

    int score = priority_score(agent);
    if (agent->state == AgentState::ReturningHomeEmpty &&
        agent->stuck_steps >= kReturnHomeEscapePriorityStuck) {
        score += kReturnHomeEscapeBoost;
        if (is_return_home_escape_move_local(agent, next)) {
            score += kReturnHomeBayMoveBoost;
        }
    }
    return score;
}

class FallbackResolutionPolicyLocal final {
public:
    FallbackResolutionPolicyLocal(
        const PlanningContext& context,
        AgentManager* manager,
        Logger* logger,
        ReservationTable* table,
        AgentNodeSlots& next_positions,
        int* fallback_leader,
        int* pull_over_mask)
        : context_(context),
          manager_(manager),
          logger_(logger),
          table_(table),
          next_positions_(next_positions),
          fallback_leader_(fallback_leader),
          pull_over_mask_(pull_over_mask) {}

    void handleSccMask(int scc_mask) const {
        if (!scc_mask) return;

        int group_ids[kMaxCbsGroup];
        const int group_n = collect_masked_group_local(manager_, scc_mask, group_ids);
        if (group_n < 2) return;

        Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = {{0}};
        const int ok = run_partial_CBS(context_, group_ids, group_n, table_, cbs_plans);
        if (ok) {
            apply_cbs_solution_local(group_ids, group_n, cbs_plans, next_positions_);
            return;
        }

        const int leader = apply_pull_over_fallback_local(context_, manager_, table_, group_ids, group_n, scc_mask, next_positions_, pull_over_mask_);
        if (leader >= 0) {
            if (fallback_leader_) *fallback_leader_ = leader;
            logger_log(logger_, "[%sWFG%s] SCC detected: leader=%c commits, others pull over.",
                C_B_YEL, C_NRM, manager_->agents[leader].symbol);
        }
    }

    void handleWaitingDeadlock() const {
        if (!all_active_agents_waiting_local(manager_, next_positions_)) return;

        int active_count = 0;
        for (int i = 0; i < MAX_AGENTS; ++i) {
            if (default_planner_agent_is_active(&manager_->agents[i])) {
                active_count++;
            }
        }
        if (active_count < 2) return;

        AgentOrder fallback_order{};
        sort_agents_by_priority(manager_, fallback_order);
        int group_ids[kMaxCbsGroup];
        const int group_n = collect_fallback_group_local(manager_, fallback_order, group_ids);
        if (group_n < 2) return;

        Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = {{0}};
        const int ok = run_partial_CBS(context_, group_ids, group_n, table_, cbs_plans);
        if (ok) {
            apply_cbs_solution_local(group_ids, group_n, cbs_plans, next_positions_);
            logger_log(logger_, "[%sCBS%s] Deadlock fallback CBS engaged (group=%d).", C_B_CYN, C_NRM, group_n);
            return;
        }

        const int all_agents_mask = (MAX_AGENTS >= static_cast<int>(sizeof(int) * 8)) ? -1 : ((1 << MAX_AGENTS) - 1);
        const int leader = apply_pull_over_fallback_local(context_, manager_, table_, group_ids, group_n, all_agents_mask, next_positions_, pull_over_mask_);
        if (fallback_leader_) *fallback_leader_ = leader;
        logger_log(logger_, "[%sWFG%s] Deadlock fallback: leader-only move, others pull-over.", C_B_YEL, C_NRM);
    }

private:
    const PlanningContext& context_;
    AgentManager* manager_{nullptr};
    Logger* logger_{nullptr};
    ReservationTable* table_{nullptr};
    AgentNodeSlots& next_positions_;
    int* fallback_leader_{nullptr};
    int* pull_over_mask_{nullptr};
};

}  // namespace

int default_planner_agent_is_active(const Agent* agent) {
    return agent && agent->state != AgentState::Idle && agent->state != AgentState::Charging && agent->goal != nullptr;
}

int default_planner_agent_is_busy_at_goal(const Agent* agent) {
    return agent &&
        agent->action_timer > 0 &&
        agent->pos &&
        agent->goal &&
        agent->pos == agent->goal;
}

void default_planner_reserve_waiting_agent_path(
    const PlanningContext& context,
    ReservationTable* table,
    const Agent* agent,
    AgentNodeSlots& next_positions) {
    for (int step = 1; step <= context.whcaHorizon(); ++step) {
        ReservationTable_setOccupant(table, step, agent->pos, agent->id, context.whcaHorizon());
    }
    next_positions[agent->id] = agent->pos;
}

void default_planner_plan_whca_path_for_agent(
    const PlanningContext& context,
    ReservationTable* table,
    WaitEdge* wait_edges,
    int* wait_edge_count,
    Agent* agent,
    AgentNodeSlots& next_positions) {
    AgentManager* manager = context.agents;
    GridMap* map = context.map;
    ensure_pathfinder_for_agent(agent);

    const int goal_was_parked = (agent->state == AgentState::GoingToCollect && agent->goal->is_parked);
    if (goal_was_parked) {
        agent->goal->is_parked = false;
    }

    if (agent->pf) {
        agent->pf->updateStart(agent->pos);
        agent->pf->computeShortestPath(map, manager);
        accumulate_whca_dstar_metrics(context, *agent->pf);
    }

    Node* plan[MAX_WHCA_HORIZON + 1];
    plan[0] = agent->pos;
    Node* current = agent->pos;

    for (int step = 1; step <= context.whcaHorizon(); ++step) {
        Node* candidates[5];
        int candidate_count = 0;
        best_candidate_order(agent->pf.get(), map, manager, current, agent->pf->goal_node, candidates, &candidate_count);

        Node* chosen = current;
        for (int candidate_index = 0; candidate_index < candidate_count; ++candidate_index) {
            Node* candidate = candidates[candidate_index];
            if (ReservationTable_isOccupied(table, step, candidate, context.whcaHorizon())) {
                const int occupant = ReservationTable_getOccupant(table, step, candidate, context.whcaHorizon());
                if (occupant != -1) {
                    add_wait_edge(wait_edges, wait_edge_count, agent->id, occupant, step, CauseType::Vertex, candidate->x, candidate->y, 0, 0);
                }
                continue;
            }

            const int previous_occupant = ReservationTable_getOccupant(table, step - 1, candidate, context.whcaHorizon());
            const int occupant_into_current = ReservationTable_getOccupant(table, step, current, context.whcaHorizon());
            if (previous_occupant != -1 && previous_occupant == occupant_into_current) {
                add_wait_edge(wait_edges, wait_edge_count, agent->id, previous_occupant, step, CauseType::Swap, current->x, current->y, candidate->x, candidate->y);
                continue;
            }

            if (chosen == current) {
                chosen = candidate;
            }
        }

        plan[step] = chosen;
        ReservationTable_setOccupant(table, step, chosen, agent->id, context.whcaHorizon());
        current = chosen;

        if (current == agent->goal) {
            reserve_goal_tail_local(context, table, step + 1, current, agent->id, plan);
            break;
        }
    }

    next_positions[agent->id] = plan[1];
    if (goal_was_parked) {
        agent->goal->is_parked = true;
    }
}

void default_planner_record_first_step_conflicts(
    const AgentManager* manager,
    const AgentNodeSlots& next_positions,
    WaitEdge* wait_edges,
    int* wait_edge_count) {
    for (int i = 0; i < MAX_AGENTS; ++i) {
            if (!default_planner_agent_is_active(&manager->agents[i]) || !next_positions[i]) continue;
        for (int j = i + 1; j < MAX_AGENTS; ++j) {
            if (!default_planner_agent_is_active(&manager->agents[j]) || !next_positions[j]) continue;

            if (next_positions[i] == next_positions[j]) {
                add_wait_edge(wait_edges, wait_edge_count, i, j, 1, CauseType::Vertex, next_positions[i]->x, next_positions[i]->y, 0, 0);
                add_wait_edge(wait_edges, wait_edge_count, j, i, 1, CauseType::Vertex, next_positions[j]->x, next_positions[j]->y, 0, 0);
            } else if (next_positions[i] == manager->agents[j].pos && next_positions[j] == manager->agents[i].pos) {
                add_wait_edge(wait_edges, wait_edge_count, i, j, 1, CauseType::Swap,
                    manager->agents[i].pos ? manager->agents[i].pos->x : -1,
                    manager->agents[i].pos ? manager->agents[i].pos->y : -1,
                    next_positions[i]->x, next_positions[i]->y);
                add_wait_edge(wait_edges, wait_edge_count, j, i, 1, CauseType::Swap,
                    manager->agents[j].pos ? manager->agents[j].pos->x : -1,
                    manager->agents[j].pos ? manager->agents[j].pos->y : -1,
                    next_positions[j]->x, next_positions[j]->y);
            }
        }
    }
}

void default_planner_apply_fallbacks(
    const PlanningContext& context,
    AgentManager* manager,
    ReservationTable* table,
    int scc_mask,
    AgentNodeSlots& next_positions,
    int* out_fallback_leader,
    int* out_pull_over_mask) {
    if (out_fallback_leader) *out_fallback_leader = -1;
    if (out_pull_over_mask) *out_pull_over_mask = 0;

    const FallbackResolutionPolicyLocal fallback_policy(
        context,
        manager,
        context.logger,
        table,
        next_positions,
        out_fallback_leader,
        out_pull_over_mask);
    if (scc_mask) {
        fallback_policy.handleSccMask(scc_mask);
    } else {
        fallback_policy.handleWaitingDeadlock();
    }
}

void default_planner_resolve_pairwise_first_step_conflicts(
    AgentManager* manager,
    Logger* logger,
    AgentNodeSlots& next_positions,
    int fallback_leader,
    int pull_over_mask) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = i + 1; j < MAX_AGENTS; j++) {
            if (manager->agents[i].state == AgentState::Idle || manager->agents[j].state == AgentState::Idle ||
                manager->agents[i].state == AgentState::Charging || manager->agents[j].state == AgentState::Charging) continue;

            const int i_pull_over = (pull_over_mask & (1 << i)) != 0;
            const int j_pull_over = (pull_over_mask & (1 << j)) != 0;
            const int i_is_leader = (i == fallback_leader);
            const int j_is_leader = (j == fallback_leader);
            const int i_escape_move = is_return_home_escape_move_local(&manager->agents[i], next_positions[i]);
            const int j_escape_move = is_return_home_escape_move_local(&manager->agents[j], next_positions[j]);
            const int priority_i = conflict_priority_local(&manager->agents[i], next_positions[i]);
            const int priority_j = conflict_priority_local(&manager->agents[j], next_positions[j]);

            if (next_positions[i] == next_positions[j]) {
                if (i_is_leader != j_is_leader) {
                    if (i_is_leader) {
                        logger_log(logger, "[%sAvoid%s] Deadlock leader Agent %c keeps the vertex.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Deadlock leader Agent %c keeps the vertex.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if (i_pull_over != j_pull_over) {
                    if (i_pull_over) {
                        logger_log(logger, "[%sAvoid%s] Pull-over escape: Agent %c yields.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Pull-over escape: Agent %c yields.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if (i_escape_move != j_escape_move) {
                    if (i_escape_move) {
                        logger_log(logger, "[%sAvoid%s] Escape move: Agent %c yields.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Escape move: Agent %c yields.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if ((manager->agents[i].state == AgentState::GoingToPark && manager->agents[j].state == AgentState::ReturningHomeEmpty) ||
                    (manager->agents[j].state == AgentState::GoingToPark && manager->agents[i].state == AgentState::ReturningHomeEmpty)) {
                    const int return_i = (manager->agents[i].state == AgentState::ReturningHomeEmpty);
                    const Agent* returning_agent = return_i ? &manager->agents[i] : &manager->agents[j];
                    if (returning_agent->stuck_steps < kReturnHomeEscapePriorityStuck) {
                        if (return_i) {
                            logger_log(logger, "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, manager->agents[i].symbol);
                            next_positions[i] = manager->agents[i].pos;
                        } else {
                            logger_log(logger, "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, manager->agents[j].symbol);
                            next_positions[j] = manager->agents[j].pos;
                        }
                    } else if (priority_i >= priority_j) {
                        logger_log(logger, "[%sAvoid%s] Vertex conflict: starvation escape gives Agent %c priority.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Vertex conflict: starvation escape gives Agent %c priority.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                } else {
                    if (priority_i >= priority_j) {
                        logger_log(logger, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                }
            } else if (next_positions[i] == manager->agents[j].pos && next_positions[j] == manager->agents[i].pos) {
                if (i_is_leader != j_is_leader) {
                    if (i_is_leader) {
                        logger_log(logger, "[%sAvoid%s] Deadlock leader Agent %c keeps the swap.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Deadlock leader Agent %c keeps the swap.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if (i_pull_over != j_pull_over) {
                    if (i_pull_over) {
                        logger_log(logger, "[%sAvoid%s] Pull-over escape: Agent %c yields the swap.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Pull-over escape: Agent %c yields the swap.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if (i_escape_move != j_escape_move) {
                    if (i_escape_move) {
                        logger_log(logger, "[%sAvoid%s] Escape move: Agent %c yields the swap.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Escape move: Agent %c yields the swap.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                    continue;
                }

                if ((manager->agents[i].state == AgentState::GoingToPark && manager->agents[j].state == AgentState::ReturningHomeEmpty) ||
                    (manager->agents[j].state == AgentState::GoingToPark && manager->agents[i].state == AgentState::ReturningHomeEmpty)) {
                    const int return_i = (manager->agents[i].state == AgentState::ReturningHomeEmpty);
                    const Agent* returning_agent = return_i ? &manager->agents[i] : &manager->agents[j];
                    if (returning_agent->stuck_steps < kReturnHomeEscapePriorityStuck) {
                        if (return_i) {
                            logger_log(logger, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, manager->agents[i].symbol);
                            next_positions[i] = manager->agents[i].pos;
                        } else {
                            logger_log(logger, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, manager->agents[j].symbol);
                            next_positions[j] = manager->agents[j].pos;
                        }
                    } else if (priority_i >= priority_j) {
                        logger_log(logger, "[%sAvoid%s] Swap conflict: starvation escape gives Agent %c priority.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Swap conflict: starvation escape gives Agent %c priority.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                } else {
                    if (priority_i >= priority_j) {
                        logger_log(logger, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, manager->agents[j].symbol);
                        next_positions[j] = manager->agents[j].pos;
                    } else {
                        logger_log(logger, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, manager->agents[i].symbol);
                        next_positions[i] = manager->agents[i].pos;
                    }
                }
            }
        }
    }
}
