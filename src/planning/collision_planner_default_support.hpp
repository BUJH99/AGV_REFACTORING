#pragma once

#include "agv/internal/engine_internal.hpp"

class DefaultPlannerSession final {
public:
    DefaultPlannerSession(const PlanningContext& context, AgentNodeSlots& next_positions, DefaultPlannerScratch& scratch);

    void execute();
    int waitEdgeCount() const;
    bool hasConflictCycle() const;

private:
    const PlanningContext& context_;
    AgentNodeSlots& next_positions_;
    DefaultPlannerScratch& scratch_;
    ReservationTable table_{};
    ConflictGraphSummary summary_{};
};
