#pragma once

#include "agv/internal/engine_internal.hpp"

int default_planner_agent_is_active(const Agent* agent);
int default_planner_agent_is_busy_at_goal(const Agent* agent);
void default_planner_reserve_waiting_agent_path(ReservationTable* table, const Agent* agent, Node* next_pos[MAX_AGENTS]);
void default_planner_plan_whca_path_for_agent(
    AgentManager* manager,
    GridMap* map,
    ReservationTable* table,
    WaitEdge* wait_edges,
    int* wait_edge_count,
    Agent* agent,
    Node* next_pos[MAX_AGENTS]);
void default_planner_record_first_step_conflicts(
    const AgentManager* manager,
    Node* next_pos[MAX_AGENTS],
    WaitEdge* wait_edges,
    int* wait_edge_count);
void default_planner_apply_fallbacks(
    AgentManager* manager,
    GridMap* map,
    Logger* logger,
    ReservationTable* table,
    int scc_mask,
    Node* next_pos[MAX_AGENTS],
    int* out_fallback_leader,
    int* out_pull_over_mask);
void default_planner_resolve_pairwise_first_step_conflicts(
    AgentManager* manager,
    Logger* logger,
    Node* next_pos[MAX_AGENTS],
    int fallback_leader,
    int pull_over_mask);
