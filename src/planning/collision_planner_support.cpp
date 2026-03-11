#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

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
    if (dx == 1 && dy == 0) return AgentDir::Right;
    if (dx == -1 && dy == 0) return AgentDir::Left;
    if (dx == 0 && dy == -1) return AgentDir::Up;
    if (dx == 0 && dy == 1) return AgentDir::Down;
    return AgentDir::None;
}

int dir_turn_steps(AgentDir from, AgentDir to) {
    if (from == AgentDir::None || to == AgentDir::None) return 0;
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
            pf->notifyCellChange(map, manager, node);
        }
    }
    marks.count = 0;
}

int node_flat_index(const Node* node) {
    return node ? (node->y * GRID_WIDTH + node->x) : -1;
}

}  // namespace

OrderedMoveCandidates OrderedMoveRankingPolicy::rank(Pathfinder* pf, GridMap* map, const AgentManager* manager,
    Node* current, Node* goal) const {
    constexpr std::array<int, 4> kStepX = {0, 0, 1, -1};
    constexpr std::array<int, 4> kStepY = {1, -1, 0, 0};

    std::array<CandidateMove, 5> candidates{};
    int candidate_count = 0;
    const double current_g = pf->gCost(current);
    candidates[candidate_count++] = CandidateMove{current, current_g + 1e-6, 1e18};

    for (int i = 0; i < 4; ++i) {
        const int next_x = current->x + kStepX[i];
        const int next_y = current->y + kStepY[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;

        Node* next = &map->grid[next_y][next_x];
        if (grid_is_node_blocked(map, manager, next, pf->ownerAgent())) continue;

        const double successor_g = pf->gCost(next);
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

    OrderedMoveCandidates ordered_candidates{};
    for (int i = 0; i < candidate_count; ++i) {
        ordered_candidates.add(candidates[i].node);
    }
    return ordered_candidates;
}

int priority_score(const Agent* agent) {
    int importance = 0;
    if (agent->state == AgentState::ReturningWithCar) importance = kPriorityReturningWithCar;
    else if (agent->state == AgentState::GoingToCharge) importance = kPriorityGoingToCharge;
    else if (agent->state == AgentState::GoingToPark || agent->state == AgentState::GoingToCollect) importance = kPriorityMovingTask;

    const int stuck_boost = (agent->stuck_steps >= kDeadlockThreshold) ? kStuckBoostHard : (agent->stuck_steps * kStuckBoostMult);
    return importance * 100 + stuck_boost - agent->id;
}

void OrderedRotationPolicy::apply(Agent* agent, Node* current, Node* desired, Node** out_next) const {
    if (!agent || !current || !out_next) return;
    *out_next = current;
    if (!desired || desired == current) return;

    const int dx = desired->x - current->x;
    const int dy = desired->y - current->y;
    const AgentDir new_heading = dir_from_delta(dx, dy);
    if (new_heading == AgentDir::None) return;

    if (agent->heading == AgentDir::None) {
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

void OrderedPlannerToolkit::preparePathfinder(Agent* agent) const {
    if (!agent->goal) return;
    if (agent->pf == nullptr) {
        agent->pf = std::make_unique<Pathfinder>(agent->pos, agent->goal, agent);
    } else if (agent->pf->goalNode() != agent->goal) {
        agent->pf->updateStart(agent->pos);
        agent->pf->reinitializeForGoal(agent->goal);
    }
}

void sort_agents_by_priority(AgentManager* manager, AgentOrder& order) {
    std::array<int, MAX_AGENTS> sorted_order{};
    std::iota(sorted_order.begin(), sorted_order.end(), 0);
    std::sort(sorted_order.begin(), sorted_order.end(),
        [manager](int lhs, int rhs) {
            return priority_score(&manager->agents[lhs]) > priority_score(&manager->agents[rhs]);
        });
    std::copy(sorted_order.begin(), sorted_order.end(), order.begin());
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
        pf_->notifyCellChange(map_, manager_, node);
    }
}

void TempObstacleScope::markOrderBlockers(const AgentManager* manager, const AgentOrder& order, int order_index, const AgentNodeSlots& next_positions) {
    if (!manager) return;
    for (int i = 0; i < order_index; ++i) {
        const int higher_id = order[i];
        if (next_positions[higher_id]) {
            mark(next_positions[higher_id]);
        }
    }
    for (int i = order_index + 1; i < MAX_AGENTS; ++i) {
        const int lower_id = order[i];
        if (manager->agents[lower_id].pos) {
            mark(manager->agents[lower_id].pos);
        }
    }
}

TemporaryGoalStateScope::TemporaryGoalStateScope(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager)
    : agent_(agent),
      pf_(pf),
      map_(map),
      manager_(manager),
      restore_(agent && agent->state == AgentState::GoingToCollect && agent->goal && agent->goal->is_parked) {
    if (restore_) {
        agent_->goal->is_parked = false;
        if (pf_) {
            pf_->notifyCellChange(map_, manager_, agent_->goal);
        }
    }
}

TemporaryGoalStateScope::~TemporaryGoalStateScope() {
    if (!restore_ || !agent_ || !agent_->goal) return;
    agent_->goal->is_parked = true;
    if (pf_) {
        pf_->notifyCellChange(map_, manager_, agent_->goal);
    }
}

OrderedMoveCandidates OrderedPlannerToolkit::rankCandidates(
    Pathfinder* pf,
    GridMap* map,
    const AgentManager* manager,
    Node* current,
    Node* goal) const {
    return move_ranking_.rank(pf, map, manager, current, goal);
}

Node* OrderedPlannerToolkit::computeDesiredMove(Agent* agent, GridMap* map, AgentManager* manager) const {
    if (!agent || !agent->pf) return agent ? agent->pos : nullptr;
    (void)metric_kind_;

    agent->pf->updateStart(agent->pos);
    agent->pf->computeShortestPath(map, manager);

    return agent->pf->getNextStep(map, manager, agent->pos);
}

void OrderedPlannerToolkit::applyRotation(Agent* agent, Node* current, Node* desired, Node** out_next) const {
    rotation_policy_.apply(agent, current, desired, out_next);
}

void ConflictResolutionPolicy::clearTouchedCellOwner() {
    for (int index = 0; index < touched_cell_count_; ++index) {
        const int cell_index = touched_cell_indices_[index];
        if (cell_index >= 0 && cell_index < static_cast<int>(cell_owner_.size())) {
            cell_owner_[cell_index] = -1;
        }
    }
    touched_cell_count_ = 0;
}

void ConflictResolutionPolicy::setCellOwner(int index, int value) {
    if (index < 0 || index >= static_cast<int>(cell_owner_.size())) {
        return;
    }
    if (cell_owner_[index] == -1 && touched_cell_count_ < MAX_AGENTS) {
        touched_cell_indices_[touched_cell_count_++] = index;
    }
    cell_owner_[index] = value;
}

void ConflictResolutionPolicy::resolve(AgentManager* manager, const AgentOrder& order, AgentNodeSlots& next_positions) {
    clearTouchedCellOwner();

    for (int oi = 0; oi < MAX_AGENTS; ++oi) {
        const int agent_id = order[oi];
        if (!next_positions[agent_id]) continue;
        const int next_idx = node_flat_index(next_positions[agent_id]);
        if (next_idx < 0) continue;
        if (cell_owner_[next_idx] != -1) {
            next_positions[agent_id] = manager->agents[agent_id].pos;
            continue;
        }
        setCellOwner(next_idx, agent_id);
    }

    clearTouchedCellOwner();
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if (!manager->agents[i].pos) continue;
        const int current_idx = node_flat_index(manager->agents[i].pos);
        if (current_idx >= 0) {
            setCellOwner(current_idx, i);
        }
    }

    for (int oi = 0; oi < MAX_AGENTS; ++oi) {
        const int agent_id = order[oi];
        if (!next_positions[agent_id]) continue;
        const int destination_idx = node_flat_index(next_positions[agent_id]);
        const int other = (destination_idx >= 0) ? cell_owner_[destination_idx] : -1;
        if (other == -1 || other == agent_id || !next_positions[other]) continue;
        if (next_positions[other] == manager->agents[agent_id].pos) {
            next_positions[other] = manager->agents[other].pos;
        } else if (next_positions[other] == manager->agents[other].pos &&
            next_positions[agent_id] == manager->agents[other].pos) {
            next_positions[agent_id] = manager->agents[agent_id].pos;
        }
    }

    clearTouchedCellOwner();
}
