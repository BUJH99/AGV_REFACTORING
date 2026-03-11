#pragma once

#include "agv/internal/engine_internal.hpp"

enum class OrderedPlanningMetric {
    AStar,
    DStar,
};

int priority_score(const Agent* agent);
void sort_agents_by_priority(AgentManager* manager, AgentOrder& order);
int best_in_mask(const AgentManager* manager, int mask);

class ConflictResolutionPolicy final {
public:
    ConflictResolutionPolicy() {
        cell_owner_.fill(-1);
    }

    void resolve(AgentManager* manager, const AgentOrder& order, AgentNodeSlots& next_positions);

private:
    std::array<int, GRID_WIDTH * GRID_HEIGHT> cell_owner_{};
    std::array<int, MAX_AGENTS> touched_cell_indices_{};
    int touched_cell_count_{0};

    void clearTouchedCellOwner();
    void setCellOwner(int index, int value);
};

class TemporaryGoalStateScope final {
public:
    TemporaryGoalStateScope(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager);
    ~TemporaryGoalStateScope();

    TemporaryGoalStateScope(const TemporaryGoalStateScope&) = delete;
    TemporaryGoalStateScope& operator=(const TemporaryGoalStateScope&) = delete;

private:
    Agent* agent_{nullptr};
    Pathfinder* pf_{nullptr};
    GridMap* map_{nullptr};
    const AgentManager* manager_{nullptr};
    bool restore_{false};
};

class OrderedMoveRankingPolicy final {
public:
    OrderedMoveCandidates rank(Pathfinder* pf, GridMap* map, const AgentManager* manager, Node* current, Node* goal) const;
};

class OrderedRotationPolicy final {
public:
    void apply(Agent* agent, Node* current, Node* desired, Node** out_next) const;
};

class TempObstacleScope final {
public:
    TempObstacleScope(Pathfinder* pf, GridMap* map, AgentManager* manager, bool auto_notify);
    ~TempObstacleScope();

    TempObstacleScope(const TempObstacleScope&) = delete;
    TempObstacleScope& operator=(const TempObstacleScope&) = delete;

    void mark(Node* node);
    void markOrderBlockers(const AgentManager* manager, const AgentOrder& order, int order_index, const AgentNodeSlots& next_positions);

private:
    TempMarkList marks_{};
    Pathfinder* pf_{nullptr};
    GridMap* map_{nullptr};
    AgentManager* manager_{nullptr};
    bool auto_notify_{false};
};

class OrderedPlannerToolkit final {
public:
    explicit OrderedPlannerToolkit(OrderedPlanningMetric metric_kind)
        : metric_kind_(metric_kind) {}

    void preparePathfinder(Agent* agent) const;
    OrderedMoveCandidates rankCandidates(Pathfinder* pf, GridMap* map, const AgentManager* manager, Node* current, Node* goal) const;
    Node* computeDesiredMove(Agent* agent, GridMap* map, AgentManager* manager) const;
    void applyRotation(Agent* agent, Node* current, Node* desired, Node** out_next) const;

private:
    OrderedPlanningMetric metric_kind_{OrderedPlanningMetric::DStar};
    OrderedMoveRankingPolicy move_ranking_{};
    OrderedRotationPolicy rotation_policy_{};
};
