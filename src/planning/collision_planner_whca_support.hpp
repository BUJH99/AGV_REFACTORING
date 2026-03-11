#pragma once

#include "agv/internal/engine_internal.hpp"

constexpr int kPlannerMaxWaitEdges = 128;

void ReservationTable_clear(ReservationTable* table);
void ReservationTable_clearAgent(ReservationTable* table, int agent_id, int horizon);
void ReservationTable_seedCurrent(ReservationTable* table, AgentManager* manager);
int ReservationTable_isOccupied(const ReservationTable* table, int t, const Node* node, int horizon);
int ReservationTable_getOccupant(const ReservationTable* table, int t, const Node* node, int horizon);
void ReservationTable_setOccupant(ReservationTable* table, int t, const Node* node, int agent_id, int horizon);

void WHCA_adjustHorizon(const PlanningContext& context, int wf_edges, int scc, Logger* logger);
void add_wait_edge(WaitEdge* edges, int* count, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2);
int build_scc_mask_from_edges(const WaitEdge* edges, int count);
Node* try_pull_over(const PlanningContext& context, const ReservationTable* table, Agent* agent);
int run_partial_CBS(
    const PlanningContext& context,
    int group_ids[],
    int group_n,
    const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]);
