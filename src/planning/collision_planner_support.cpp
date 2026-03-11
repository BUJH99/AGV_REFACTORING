#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const struct Agent_* agent);
Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
void pathfinder_update_start(Pathfinder* pf, Node* new_start);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
Node* pathfinder_get_next_step(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* current_node);

namespace {

constexpr int kPriorityReturningWithCar = 3;
constexpr int kPriorityGoingToCharge = 2;
constexpr int kPriorityMovingTask = 1;
constexpr int kDeadlockThreshold = 5;
constexpr int kStuckBoostMult = 10;
constexpr int kStuckBoostHard = 1000;
constexpr int kTurn90Wait = 2;
constexpr int kDir4X[4] = {0, 0, 1, -1};
constexpr int kDir4Y[4] = {1, -1, 0, 0};

AgentDir dir_from_delta(int dx, int dy) {
    if (dx == 1 && dy == 0) return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == -1) return DIR_UP;
    if (dx == 0 && dy == 1) return DIR_DOWN;
    return DIR_NONE;
}

int dir_turn_steps(AgentDir from, AgentDir to) {
    if (from == DIR_NONE || to == DIR_NONE) return 0;
    const int diff = (static_cast<int>(to) - static_cast<int>(from) + 4) % 4;
    return diff <= 2 ? diff : 4 - diff;
}

double manhattan_xy(int x1, int y1, int x2, int y2) {
    return std::fabs(static_cast<double>(x1) - static_cast<double>(x2)) +
        std::fabs(static_cast<double>(y1) - static_cast<double>(y2));
}

struct CandidateMove final {
    Node* node{nullptr};
    double cost{0.0};
    double distance{0.0};
};

void mark_temp_node(TempMarkList& marks, Node* node) {
    if (!node || node->is_temp) return;
    node->is_temp = true;
    if (marks.count < TEMP_MARK_MAX) {
        marks.nodes[marks.count++] = node;
    }
}

void clear_temp_marks(TempMarkList& marks) {
    for (int i = 0; i < marks.count; ++i) {
        if (marks.nodes[i]) {
            marks.nodes[i]->is_temp = false;
        }
    }
    marks.count = 0;
}

void clear_temp_marks_and_notify(TempMarkList& marks, Pathfinder* pf, GridMap* map, const AgentManager* manager) {
    for (int i = 0; i < marks.count; ++i) {
        Node* node = marks.nodes[i];
        if (!node) continue;
        node->is_temp = false;
        if (pf) {
            pathfinder_notify_cell_change(pf, map, manager, node);
        }
    }
    marks.count = 0;
}

int node_flat_index(const Node* node) {
    return node ? (node->y * GRID_WIDTH + node->x) : -1;
}

}  // namespace

int best_candidate_order(Pathfinder* pf, GridMap* map, const AgentManager* manager,
    Node* current, Node* goal, Node* out[5], int* out_count) {
    constexpr std::array<int, 4> kStepX = {0, 0, 1, -1};
    constexpr std::array<int, 4> kStepY = {1, -1, 0, 0};

    std::array<CandidateMove, 5> candidates{};
    int candidate_count = 0;
    const double current_g = pf->cells[current->y][current->x].g;
    candidates[candidate_count++] = CandidateMove{current, current_g + 1e-6, 1e18};

    for (int i = 0; i < 4; ++i) {
        const int next_x = current->x + kStepX[i];
        const int next_y = current->y + kStepY[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;

        Node* next = &map->grid[next_y][next_x];
        if (grid_is_node_blocked(map, manager, next, pf->agent)) continue;

        const double successor_g = pf->cells[next->y][next->x].g;
        candidates[candidate_count++] = CandidateMove{
            next,
            1.0 + successor_g,
            std::fabs(static_cast<double>(next->x) - static_cast<double>(goal->x)) +
                std::fabs(static_cast<double>(next->y) - static_cast<double>(goal->y)),
        };
    }

    std::sort(candidates.begin(), candidates.begin() + candidate_count,
        [](const CandidateMove& lhs, const CandidateMove& rhs) {
            if (lhs.cost < rhs.cost - 1e-9) return true;
            if (lhs.cost > rhs.cost + 1e-9) return false;
            return lhs.distance < rhs.distance;
        });

    for (int i = 0; i < candidate_count; ++i) {
        out[i] = candidates[i].node;
    }
    *out_count = candidate_count;
    return candidate_count;
}

int priority_score(const Agent* agent) {
    int importance = 0;
    if (agent->state == RETURNING_WITH_CAR) importance = kPriorityReturningWithCar;
    else if (agent->state == GOING_TO_CHARGE) importance = kPriorityGoingToCharge;
    else if (agent->state == GOING_TO_PARK || agent->state == GOING_TO_COLLECT) importance = kPriorityMovingTask;

    const int stuck_boost = (agent->stuck_steps >= kDeadlockThreshold) ? kStuckBoostHard : (agent->stuck_steps * kStuckBoostMult);
    return importance * 100 + stuck_boost - agent->id;
}

void apply_rotation_and_step(Agent* agent, Node* current, Node* desired, Node** out_next) {
    if (!agent || !current || !out_next) return;
    *out_next = current;
    if (!desired || desired == current) return;

    const int dx = desired->x - current->x;
    const int dy = desired->y - current->y;
    const AgentDir new_heading = dir_from_delta(dx, dy);
    if (new_heading == DIR_NONE) return;

    if (agent->heading == DIR_NONE) {
        agent->heading = new_heading;
        *out_next = desired;
        return;
    }

    const int turn_steps = dir_turn_steps(agent->heading, new_heading);
    if (turn_steps == 1) {
        agent->rotation_wait = kTurn90Wait - 1;
        agent->heading = new_heading;
        agent->metrics_turns_current++;
        return;
    }
    agent->heading = new_heading;
    *out_next = desired;
}

void ensure_pathfinder_for_agent(Agent* agent) {
    if (!agent->goal) return;
    if (agent->pf == nullptr) {
        agent->pf.reset(pathfinder_create(agent->pos, agent->goal, agent));
    } else if (agent->pf->goal_node != agent->goal) {
        agent->pf->start_node = agent->pos;
        pathfinder_reset_goal(agent->pf.get(), agent->goal);
    }
}

void sort_agents_by_priority(AgentManager* manager, int order[MAX_AGENTS]) {
    std::array<int, MAX_AGENTS> sorted_order{};
    std::iota(sorted_order.begin(), sorted_order.end(), 0);
    std::sort(sorted_order.begin(), sorted_order.end(),
        [manager](int lhs, int rhs) {
            return priority_score(&manager->agents[lhs]) > priority_score(&manager->agents[rhs]);
        });
    std::copy(sorted_order.begin(), sorted_order.end(), order);
}

int best_in_mask(const AgentManager* manager, int mask) {
    int best = -1;
    int best_score = -999999;
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if ((mask & (1 << i)) == 0) continue;
        const int score = priority_score(&manager->agents[i]);
        if (score > best_score) {
            best_score = score;
            best = i;
        }
    }
    return best;
}

TempObstacleScope::TempObstacleScope(Pathfinder* pf, GridMap* map, AgentManager* manager, bool auto_notify)
    : pf_(pf), map_(map), manager_(manager), auto_notify_(auto_notify) {
    marks_.count = 0;
}

TempObstacleScope::~TempObstacleScope() {
    if (auto_notify_) {
        clear_temp_marks_and_notify(marks_, pf_, map_, manager_);
    } else {
        clear_temp_marks(marks_);
    }
}

void TempObstacleScope::mark(Node* node) {
    mark_temp_node(marks_, node);
    if (auto_notify_ && pf_) {
        pathfinder_notify_cell_change(pf_, map_, manager_, node);
    }
}

void TempObstacleScope::markOrderBlockers(const AgentManager* manager, const int order[MAX_AGENTS], int order_index, Node* next_pos[MAX_AGENTS]) {
    if (!manager || !order) return;
    for (int i = 0; i < order_index; ++i) {
        const int higher_id = order[i];
        if (next_pos[higher_id]) {
            mark(next_pos[higher_id]);
        }
    }
    for (int i = order_index + 1; i < MAX_AGENTS; ++i) {
        const int lower_id = order[i];
        if (manager->agents[lower_id].pos) {
            mark(manager->agents[lower_id].pos);
        }
    }
}

int temporarily_unpark_goal(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager) {
    const int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
    if (goal_was_parked) {
        agent->goal->is_parked = false;
        if (pf) {
            pathfinder_notify_cell_change(pf, map, manager, agent->goal);
        }
    }
    return goal_was_parked;
}

void restore_temporarily_unparked_goal(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager, int goal_was_parked) {
    if (!goal_was_parked) return;
    agent->goal->is_parked = true;
    if (pf) {
        pathfinder_notify_cell_change(pf, map, manager, agent->goal);
    }
}

Node* compute_ordered_pathfinder_move(Agent* agent, GridMap* map, AgentManager* manager, OrderedPlanningMetric metric_kind) {
    if (!agent || !agent->pf) return agent ? agent->pos : nullptr;
    (void)metric_kind;

    pathfinder_update_start(agent->pf.get(), agent->pos);
    pathfinder_compute_shortest_path(agent->pf.get(), map, manager);

    return pathfinder_get_next_step(agent->pf.get(), map, manager, agent->pos);
}

void resolve_conflicts_by_order(AgentManager* manager, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]) {
    int cell_owner[GRID_WIDTH * GRID_HEIGHT];
    for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) cell_owner[i] = -1;

    for (int oi = 0; oi < MAX_AGENTS; ++oi) {
        const int agent_id = order[oi];
        if (!next_pos[agent_id]) continue;
        const int next_idx = node_flat_index(next_pos[agent_id]);
        if (next_idx < 0) continue;
        if (cell_owner[next_idx] != -1) {
            next_pos[agent_id] = manager->agents[agent_id].pos;
            continue;
        }
        cell_owner[next_idx] = agent_id;
    }

    for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; ++i) cell_owner[i] = -1;
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if (!manager->agents[i].pos) continue;
        const int current_idx = node_flat_index(manager->agents[i].pos);
        if (current_idx >= 0) {
            cell_owner[current_idx] = i;
        }
    }

    for (int oi = 0; oi < MAX_AGENTS; ++oi) {
        const int agent_id = order[oi];
        if (!next_pos[agent_id]) continue;
        const int destination_idx = node_flat_index(next_pos[agent_id]);
        const int other = (destination_idx >= 0) ? cell_owner[destination_idx] : -1;
        if (other == -1 || other == agent_id || !next_pos[other]) continue;
        if (next_pos[other] == manager->agents[agent_id].pos) {
            next_pos[other] = manager->agents[other].pos;
        } else if (next_pos[other] == manager->agents[other].pos &&
            next_pos[agent_id] == manager->agents[other].pos) {
            next_pos[agent_id] = manager->agents[agent_id].pos;
        }
    }
}
