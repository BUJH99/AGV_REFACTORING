#define _CRT_SECURE_NO_WARNINGS

#include <cmath>

#include "agv/internal/engine_internal.hpp"

namespace {

constexpr double kPathfinderInf = 1e18;
constexpr int kDir4X[4] = {0, 0, 1, -1};
constexpr int kDir4Y[4] = {1, -1, 0, 0};

int compare_keys(Key a, Key b) {
    if (a.k1 < b.k1 - 1e-9) return -1;
    if (a.k1 > b.k1 + 1e-9) return 1;
    if (a.k2 < b.k2 - 1e-9) return -1;
    if (a.k2 > b.k2 + 1e-9) return 1;
    return 0;
}

SearchCell* pathfinder_cell(Pathfinder* pf, const Node* node) {
    return &pf->cells[node->y][node->x];
}

Key pathfinder_key(Pathfinder* pf, const Node* node) {
    return pathfinder_cell(pf, node)->key;
}

void pq_init(NodePQ* pq, int cap) {
    (void)cap;
    pq->clear();
}

void pq_free(NodePQ* pq) {
    if (!pq) return;
    pq->clear();
}

void pq_swap(Pathfinder* pf, Node** a, Node** b) {
    Node* temp = *a;
    *a = *b;
    *b = temp;

    int a_index = pathfinder_cell(pf, *a)->pq_index;
    int b_index = pathfinder_cell(pf, *b)->pq_index;
    pathfinder_cell(pf, *a)->pq_index = b_index;
    pathfinder_cell(pf, *b)->pq_index = a_index;
}

void heapify_up(Pathfinder* pf, NodePQ* pq, int index) {
    while (index > 0) {
        const int parent = (index - 1) / 2;
        if (compare_keys(pathfinder_key(pf, pq->nodes[index]), pathfinder_key(pf, pq->nodes[parent])) >= 0) {
            break;
        }
        pq_swap(pf, &pq->nodes[index], &pq->nodes[parent]);
        if (pf) pf->heap_moves_this_call++;
        index = parent;
    }
}

void heapify_down(Pathfinder* pf, NodePQ* pq, int index) {
    while (true) {
        const int left = 2 * index + 1;
        const int right = 2 * index + 2;
        int smallest = index;
        if (left < pq->size && compare_keys(pathfinder_key(pf, pq->nodes[left]), pathfinder_key(pf, pq->nodes[smallest])) < 0) smallest = left;
        if (right < pq->size && compare_keys(pathfinder_key(pf, pq->nodes[right]), pathfinder_key(pf, pq->nodes[smallest])) < 0) smallest = right;
        if (smallest == index) {
            break;
        }
        pq_swap(pf, &pq->nodes[index], &pq->nodes[smallest]);
        if (pf) pf->heap_moves_this_call++;
        index = smallest;
    }
}

int pq_contains(Pathfinder* pf, const Node* node) {
    return pathfinder_cell(pf, node)->in_pq;
}

Key pq_top_key(Pathfinder* pf, const NodePQ* pq) {
    if (pq->size == 0) return make_key(kPathfinderInf, kPathfinderInf);
    return pathfinder_key(pf, pq->nodes[0]);
}

void pq_push(Pathfinder* pf, NodePQ* pq, Node* node) {
    if (pq->size >= pq->capacity()) return;
    if (pf) pf->nodes_generated_this_call++;
    SearchCell* cell = pathfinder_cell(pf, node);
    cell->in_pq = true;
    cell->pq_index = pq->size;
    pq->nodes[pq->size++] = node;
    heapify_up(pf, pq, pq->size - 1);
}

Node* pq_pop(Pathfinder* pf, NodePQ* pq) {
    if (pq->size == 0) return nullptr;
    Node* top = pq->nodes[0];
    SearchCell* top_cell = pathfinder_cell(pf, top);
    top_cell->in_pq = false;
    top_cell->pq_index = -1;
    pq->size--;
    if (pq->size > 0) {
        pq->nodes[0] = pq->nodes[pq->size];
        pathfinder_cell(pf, pq->nodes[0])->pq_index = 0;
        heapify_down(pf, pq, 0);
    }
    return top;
}

void pq_remove(Pathfinder* pf, NodePQ* pq, Node* node) {
    SearchCell* cell = pathfinder_cell(pf, node);
    if (!cell->in_pq) return;
    int index = cell->pq_index;
    pq->size--;
    if (index != pq->size) {
        pq->nodes[index] = pq->nodes[pq->size];
        pathfinder_cell(pf, pq->nodes[index])->pq_index = index;
        int parent = (index - 1) / 2;
        if (index > 0 && compare_keys(pathfinder_key(pf, pq->nodes[index]), pathfinder_key(pf, pq->nodes[parent])) < 0) {
            heapify_up(pf, pq, index);
        } else {
            heapify_down(pf, pq, index);
        }
    }
    cell->in_pq = false;
    cell->pq_index = -1;
}

double pathfinder_manhattan(const Node* a, const Node* b) {
    return std::fabs((double)a->x - (double)b->x) + std::fabs((double)a->y - (double)b->y);
}

double heuristic(const Node* a, const Node* b) {
    return pathfinder_manhattan(a, b);
}

Key calculate_key(Pathfinder* pf, const Node* node) {
    SearchCell* cell = pathfinder_cell(pf, node);
    double minimum = std::fmin(cell->g, cell->rhs);
    return make_key(minimum + heuristic(pf->start_node, node) + pf->km, minimum);
}

void update_vertex(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* node) {
    SearchCell* cell = pathfinder_cell(pf, node);
    if (node != pf->goal_node) {
        double min_rhs = kPathfinderInf;
        for (int i = 0; i < 4; i++) {
            int next_x = node->x + kDir4X[i];
            int next_y = node->y + kDir4Y[i];
            if (!grid_is_valid_coord(next_x, next_y)) continue;
            Node* successor = &map->grid[next_y][next_x];
            if (!grid_is_node_blocked(map, am, successor, pf->agent)) {
                double successor_g = pathfinder_cell(pf, successor)->g;
                double candidate = 1.0 + successor_g;
                if (candidate < min_rhs) min_rhs = candidate;
            }
        }
        cell->rhs = min_rhs;
    }
    if (pq_contains(pf, node)) pq_remove(pf, &pf->pq, node);
    if (std::fabs(cell->g - cell->rhs) > 1e-9) {
        cell->key = calculate_key(pf, node);
        pq_push(pf, &pf->pq, node);
    }
}

void reset_pathfinder_cells(Pathfinder* pf) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            pf->cells[y][x].g = kPathfinderInf;
            pf->cells[y][x].rhs = kPathfinderInf;
            pf->cells[y][x].in_pq = false;
            pf->cells[y][x].pq_index = -1;
            pf->cells[y][x].key = make_key(kPathfinderInf, kPathfinderInf);
        }
    }
}

}  // namespace

Pathfinder_::Pathfinder_(Node* start, Node* goal, const Agent* owning_agent) {
    pq_init(&pq, GRID_WIDTH * GRID_HEIGHT);
    start_node = start;
    last_start = start;
    goal_node = goal;
    km = 0.0;
    agent = owning_agent;
    nodes_expanded_this_call = 0;
    heap_moves_this_call = 0;
    nodes_generated_this_call = 0;
    valid_expansions_this_call = 0;
    reset_pathfinder_cells(this);

    if (goal) {
        SearchCell* goal_cell = &cells[goal->y][goal->x];
        goal_cell->rhs = 0.0;
        goal_cell->key = calculate_key(this, goal);
        pq_push(this, &pq, goal);
    }
}

void Pathfinder_::resetGoal(Node* new_goal) {
    goal_node = new_goal;
    km = 0.0;
    last_start = start_node;
    pq.size = 0;
    reset_pathfinder_cells(this);
    if (new_goal) {
        cells[new_goal->y][new_goal->x].rhs = 0.0;
        cells[new_goal->y][new_goal->x].key = calculate_key(this, new_goal);
        pq_push(this, &pq, new_goal);
    }
}

void Pathfinder_::updateStart(Node* new_start) {
    if (new_start == nullptr) return;
    if (start_node == nullptr) {
        start_node = new_start;
        last_start = new_start;
        return;
    }
    km += heuristic(last_start, new_start);
    last_start = new_start;
    start_node = new_start;
}

void Pathfinder_::notifyCellChange(GridMap* map, const AgentManager* am, Node* changed) {
    update_vertex(this, map, am, changed);
    for (int i = 0; i < 4; i++) {
        int prev_x = changed->x + kDir4X[i];
        int prev_y = changed->y + kDir4Y[i];
        if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(this, map, am, &map->grid[prev_y][prev_x]);
    }
}

void Pathfinder_::computeShortestPath(GridMap* map, const AgentManager* am) {
    if (!start_node || !goal_node) return;

    nodes_expanded_this_call = 0;
    heap_moves_this_call = 0;

    while (true) {
        Key top = pq_top_key(this, &pq);
        SearchCell* start_cell = pathfinder_cell(this, start_node);
        Key start_key = calculate_key(this, start_node);

        if (pq.size == 0 || (compare_keys(top, start_key) >= 0 && std::fabs(start_cell->rhs - start_cell->g) < 1e-9)) break;

        Key old_key = top;
        Node* node = pq_pop(this, &pq);
        if (node) nodes_expanded_this_call++;
        SearchCell* cell = pathfinder_cell(this, node);
        Key new_key = calculate_key(this, node);

        if (compare_keys(old_key, new_key) < 0) {
            cell->key = new_key;
            pq_push(this, &pq, node);
        } else if (cell->g > cell->rhs) {
            cell->g = cell->rhs;
            valid_expansions_this_call++;
            for (int i = 0; i < 4; i++) {
                int prev_x = node->x + kDir4X[i];
                int prev_y = node->y + kDir4Y[i];
                if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(this, map, am, &map->grid[prev_y][prev_x]);
            }
        } else {
            cell->g = kPathfinderInf;
            update_vertex(this, map, am, node);
            for (int i = 0; i < 4; i++) {
                int prev_x = node->x + kDir4X[i];
                int prev_y = node->y + kDir4Y[i];
                if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(this, map, am, &map->grid[prev_y][prev_x]);
            }
        }
    }
}

Node* Pathfinder_::getNextStep(GridMap* map, const AgentManager* am, Node* current) {
    if (!goal_node || !current) return current;
    SearchCell* current_cell = pathfinder_cell(this, current);
    if (current_cell->g >= kPathfinderInf || current == goal_node) return current;

    double best_cost = kPathfinderInf;
    Node* best_node = current;
    double best_distance = std::fabs((double)current->x - (double)goal_node->x) +
        std::fabs((double)current->y - (double)goal_node->y);

    for (int i = 0; i < 4; i++) {
        int next_x = current->x + kDir4X[i];
        int next_y = current->y + kDir4Y[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        Node* neighbor = &map->grid[next_y][next_x];
        if (grid_is_node_blocked(map, am, neighbor, agent)) continue;
        double successor_g = pathfinder_cell(this, neighbor)->g;
        double cost = 1.0 + successor_g;
        if (cost < best_cost) {
            best_cost = cost;
            best_node = neighbor;
            best_distance = pathfinder_manhattan(neighbor, goal_node);
        } else if (std::fabs(cost - best_cost) < 1e-9) {
            double distance = pathfinder_manhattan(neighbor, goal_node);
            if (distance < best_distance) {
                best_node = neighbor;
                best_distance = distance;
            }
        }
    }
    return best_node;
}
