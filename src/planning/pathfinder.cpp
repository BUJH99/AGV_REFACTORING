#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <utility>

#include "agv/internal/engine_internal.hpp"

namespace {

constexpr double kPathfinderInf = 1e18;
constexpr int kDir4X[4] = {0, 0, 1, -1};
constexpr int kDir4Y[4] = {1, -1, 0, 0};

double manhattan_distance(const Node* lhs, const Node* rhs) {
    return std::fabs(static_cast<double>(lhs->x) - static_cast<double>(rhs->x)) +
        std::fabs(static_cast<double>(lhs->y) - static_cast<double>(rhs->y));
}

}  // namespace

int Pathfinder::compareKeys(Key lhs, Key rhs) {
    if (lhs.k1 < rhs.k1 - 1e-9) return -1;
    if (lhs.k1 > rhs.k1 + 1e-9) return 1;
    if (lhs.k2 < rhs.k2 - 1e-9) return -1;
    if (lhs.k2 > rhs.k2 + 1e-9) return 1;
    return 0;
}

double Pathfinder::heuristic(const Node* lhs, const Node* rhs) {
    return manhattan_distance(lhs, rhs);
}

Pathfinder::Pathfinder(Node* start, Node* goal, const Agent_* agent)
    : agent_(agent) {
    resetCoreState(start, goal);
}

void Pathfinder::reinitializeForGoal(Node* new_goal) {
    resetCoreState(start_node_, new_goal);
}

void Pathfinder::updateStart(Node* new_start) {
    if (!new_start) return;
    if (!start_node_) {
        start_node_ = new_start;
        last_start_ = new_start;
        return;
    }

    km_ += heuristic(last_start_, new_start);
    last_start_ = new_start;
    start_node_ = new_start;
}

void Pathfinder::notifyCellChange(GridMap* map, const AgentManager* am, Node* changed) {
    if (!changed) return;
    updateVertex(map, am, changed);
    for (int index = 0; index < 4; ++index) {
        const int prev_x = changed->x + kDir4X[index];
        const int prev_y = changed->y + kDir4Y[index];
        if (grid_is_valid_coord(prev_x, prev_y)) {
            updateVertex(map, am, &map->grid[prev_y][prev_x]);
        }
    }
}

void Pathfinder::computeShortestPath(GridMap* map, const AgentManager* am) {
    if (!start_node_ || !goal_node_) return;

    resetLastRunMetrics();

    while (true) {
        const Key top = topKey();
        const SearchCell* start_cell = cell(start_node_);
        const Key start_key = calculateKey(start_node_);

        if (heap_.size() == 0 ||
            (compareKeys(top, start_key) >= 0 && std::fabs(start_cell->rhs - start_cell->g) < 1e-9)) {
            break;
        }

        const Key old_key = top;
        Node* node = popNode();
        if (!node) break;

        last_run_metrics_.nodes_expanded++;
        SearchCell* current_cell = cell(node);
        const Key new_key = calculateKey(node);

        if (compareKeys(old_key, new_key) < 0) {
            current_cell->key = new_key;
            pushNode(node);
        } else if (current_cell->g > current_cell->rhs) {
            current_cell->g = current_cell->rhs;
            last_run_metrics_.valid_expansions++;
            for (int index = 0; index < 4; ++index) {
                const int prev_x = node->x + kDir4X[index];
                const int prev_y = node->y + kDir4Y[index];
                if (grid_is_valid_coord(prev_x, prev_y)) {
                    updateVertex(map, am, &map->grid[prev_y][prev_x]);
                }
            }
        } else {
            current_cell->g = kPathfinderInf;
            updateVertex(map, am, node);
            for (int index = 0; index < 4; ++index) {
                const int prev_x = node->x + kDir4X[index];
                const int prev_y = node->y + kDir4Y[index];
                if (grid_is_valid_coord(prev_x, prev_y)) {
                    updateVertex(map, am, &map->grid[prev_y][prev_x]);
                }
            }
        }
    }
}

Node* Pathfinder::getNextStep(GridMap* map, const AgentManager* am, Node* current) {
    if (!goal_node_ || !current) return current;

    const SearchCell* current_cell = cell(current);
    if (!current_cell || current_cell->g >= kPathfinderInf || current == goal_node_) {
        return current;
    }

    double best_cost = kPathfinderInf;
    Node* best_node = current;
    double best_distance = manhattan_distance(current, goal_node_);

    for (int index = 0; index < 4; ++index) {
        const int next_x = current->x + kDir4X[index];
        const int next_y = current->y + kDir4Y[index];
        if (!grid_is_valid_coord(next_x, next_y)) continue;

        Node* neighbor = &map->grid[next_y][next_x];
        if (grid_is_node_blocked(map, am, neighbor, ownerAgent())) continue;

        const double successor_cost = gCost(neighbor);
        const double candidate_cost = 1.0 + successor_cost;
        if (candidate_cost < best_cost) {
            best_cost = candidate_cost;
            best_node = neighbor;
            best_distance = manhattan_distance(neighbor, goal_node_);
        } else if (std::fabs(candidate_cost - best_cost) < 1e-9) {
            const double distance = manhattan_distance(neighbor, goal_node_);
            if (distance < best_distance) {
                best_node = neighbor;
                best_distance = distance;
            }
        }
    }

    return best_node;
}

double Pathfinder::gCost(const Node* node) const {
    const SearchCell* search_cell = cell(node);
    return search_cell ? search_cell->g : kPathfinderInf;
}

void Pathfinder::resetLastRunMetrics() {
    last_run_metrics_ = {};
}

Pathfinder::SearchCell* Pathfinder::cell(const Node* node) {
    return node ? &cells_[node->y][node->x] : nullptr;
}

const Pathfinder::SearchCell* Pathfinder::cell(const Node* node) const {
    return node ? &cells_[node->y][node->x] : nullptr;
}

Key Pathfinder::keyFor(const Node* node) const {
    const SearchCell* search_cell = cell(node);
    return search_cell ? search_cell->key : make_key(kPathfinderInf, kPathfinderInf);
}

bool Pathfinder::heapContains(const Node* node) const {
    const SearchCell* search_cell = cell(node);
    return search_cell ? search_cell->in_pq : false;
}

Key Pathfinder::topKey() const {
    if (heap_.size() == 0) {
        return make_key(kPathfinderInf, kPathfinderInf);
    }
    return keyFor(heap_.at(0));
}

void Pathfinder::pushNode(Node* node) {
    if (!node || heap_.size() >= heap_.capacity()) return;

    SearchCell* search_cell = cell(node);
    if (!search_cell) return;

    last_run_metrics_.generated_nodes++;
    search_cell->in_pq = true;
    search_cell->pq_index = heap_.size_;
    heap_.at(heap_.size_) = node;
    heap_.size_++;
    heapifyUp(heap_.size_ - 1);
}

Node* Pathfinder::popNode() {
    if (heap_.size() == 0) return nullptr;

    Node* top = heap_.at(0);
    SearchCell* top_cell = cell(top);
    top_cell->in_pq = false;
    top_cell->pq_index = -1;
    heap_.size_--;

    if (heap_.size() > 0) {
        heap_.at(0) = heap_.at(heap_.size_);
        cell(heap_.at(0))->pq_index = 0;
        heapifyDown(0);
    }

    return top;
}

void Pathfinder::removeNode(Node* node) {
    SearchCell* search_cell = cell(node);
    if (!search_cell || !search_cell->in_pq) return;

    const int index = search_cell->pq_index;
    heap_.size_--;
    if (index != heap_.size_) {
        heap_.at(index) = heap_.at(heap_.size_);
        cell(heap_.at(index))->pq_index = index;
        const int parent = (index - 1) / 2;
        if (index > 0 && compareKeys(keyFor(heap_.at(index)), keyFor(heap_.at(parent))) < 0) {
            heapifyUp(index);
        } else {
            heapifyDown(index);
        }
    }

    search_cell->in_pq = false;
    search_cell->pq_index = -1;
}

void Pathfinder::swapHeapNodes(int lhs_index, int rhs_index) {
    if (lhs_index == rhs_index) return;

    Node* lhs_node = heap_.at(lhs_index);
    Node* rhs_node = heap_.at(rhs_index);
    std::swap(heap_.at(lhs_index), heap_.at(rhs_index));
    cell(lhs_node)->pq_index = rhs_index;
    cell(rhs_node)->pq_index = lhs_index;
}

void Pathfinder::heapifyUp(int index) {
    while (index > 0) {
        const int parent = (index - 1) / 2;
        if (compareKeys(keyFor(heap_.at(index)), keyFor(heap_.at(parent))) >= 0) {
            break;
        }
        swapHeapNodes(index, parent);
        last_run_metrics_.heap_moves++;
        index = parent;
    }
}

void Pathfinder::heapifyDown(int index) {
    while (true) {
        const int left = 2 * index + 1;
        const int right = 2 * index + 2;
        int smallest = index;

        if (left < heap_.size() && compareKeys(keyFor(heap_.at(left)), keyFor(heap_.at(smallest))) < 0) {
            smallest = left;
        }
        if (right < heap_.size() && compareKeys(keyFor(heap_.at(right)), keyFor(heap_.at(smallest))) < 0) {
            smallest = right;
        }
        if (smallest == index) break;

        swapHeapNodes(index, smallest);
        last_run_metrics_.heap_moves++;
        index = smallest;
    }
}

Key Pathfinder::calculateKey(const Node* node) const {
    const SearchCell* search_cell = cell(node);
    const double minimum = std::fmin(search_cell->g, search_cell->rhs);
    return make_key(minimum + heuristic(start_node_, node) + km_, minimum);
}

void Pathfinder::updateVertex(GridMap* map, const AgentManager* am, Node* node) {
    if (!map || !node) return;

    SearchCell* search_cell = cell(node);
    if (node != goal_node_) {
        double min_rhs = kPathfinderInf;
        for (int index = 0; index < 4; ++index) {
            const int next_x = node->x + kDir4X[index];
            const int next_y = node->y + kDir4Y[index];
            if (!grid_is_valid_coord(next_x, next_y)) continue;

            Node* successor = &map->grid[next_y][next_x];
            if (grid_is_node_blocked(map, am, successor, ownerAgent())) continue;

            const double candidate = 1.0 + gCost(successor);
            if (candidate < min_rhs) {
                min_rhs = candidate;
            }
        }
        search_cell->rhs = min_rhs;
    }

    if (heapContains(node)) {
        removeNode(node);
    }
    if (std::fabs(search_cell->g - search_cell->rhs) > 1e-9) {
        search_cell->key = calculateKey(node);
        pushNode(node);
    }
}

void Pathfinder::resetAllCells() {
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            cells_[y][x].g = kPathfinderInf;
            cells_[y][x].rhs = kPathfinderInf;
            cells_[y][x].in_pq = false;
            cells_[y][x].pq_index = -1;
            cells_[y][x].key = make_key(kPathfinderInf, kPathfinderInf);
        }
    }
}

void Pathfinder::resetCoreState(Node* start, Node* goal) {
    start_node_ = start;
    last_start_ = start;
    goal_node_ = goal;
    km_ = 0.0;
    heap_.clear();
    resetLastRunMetrics();
    resetAllCells();

    if (!goal) return;

    SearchCell* goal_cell = cell(goal);
    goal_cell->rhs = 0.0;
    goal_cell->key = calculateKey(goal);
    pushNode(goal);
    resetLastRunMetrics();
}
