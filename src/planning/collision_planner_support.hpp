#pragma once

#include "agv/internal/engine_internal.hpp"

enum OrderedPlanningMetric {
    ORDERED_PLANNING_ASTAR,
    ORDERED_PLANNING_DSTAR,
};

int priority_score(const Agent* agent);
void apply_rotation_and_step(Agent* agent, Node* current, Node* desired, Node** out_next);
void ensure_pathfinder_for_agent(Agent* agent);
void sort_agents_by_priority(AgentManager* manager, int order[MAX_AGENTS]);
int best_candidate_order(Pathfinder* pf, GridMap* map, const AgentManager* manager,
    Node* current, Node* goal, Node* out[5], int* out_count);
int best_in_mask(const AgentManager* manager, int mask);
int temporarily_unpark_goal(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager);
void restore_temporarily_unparked_goal(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager, int goal_was_parked);
Node* compute_ordered_pathfinder_move(Agent* agent, GridMap* map, AgentManager* manager, OrderedPlanningMetric metric_kind);
void resolve_conflicts_by_order(AgentManager* manager, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]);

class TempObstacleScope final {
public:
    TempObstacleScope(Pathfinder* pf, GridMap* map, AgentManager* manager, bool auto_notify);
    ~TempObstacleScope();

    TempObstacleScope(const TempObstacleScope&) = delete;
    TempObstacleScope& operator=(const TempObstacleScope&) = delete;

    void mark(Node* node);
    void markOrderBlockers(const AgentManager* manager, const int order[MAX_AGENTS], int order_index, Node* next_pos[MAX_AGENTS]);

private:
    TempMarkList marks_{};
    Pathfinder* pf_{nullptr};
    GridMap* map_{nullptr};
    AgentManager* manager_{nullptr};
    bool auto_notify_{false};
};
