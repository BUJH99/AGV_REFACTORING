#pragma once

#include "agv/internal/engine_internal.hpp"

void WHCA_adjustHorizon(const PlanningContext& context, int wf_edges, int scc, Logger* logger);
ConflictGraphSummary analyze_conflict_graph(const WaitEdgeBuffer& wait_edges);
Node* try_pull_over(
    const PlanningContext& context,
    const ReservationTable& table,
    Agent* agent,
    DefaultPlannerScratch& scratch);
CbsSolveResult run_partial_CBS(
    const PlanningContext& context,
    const std::array<int, MAX_CBS_GROUP>& group_ids,
    int group_n,
    const ReservationTable& base_rt,
    DefaultPlannerScratch& scratch);
