#define _CRT_SECURE_NO_WARNINGS

#include <cmath>

#include "agv/internal/engine_internal.hpp"

#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const struct Agent_* agent);

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
    if (index == 0) return;
    int parent = (index - 1) / 2;
    if (compare_keys(pathfinder_key(pf, pq->nodes[index]), pathfinder_key(pf, pq->nodes[parent])) < 0) {
        pq_swap(pf, &pq->nodes[index], &pq->nodes[parent]);
        if (pf) pf->heap_moves_this_call++;
        heapify_up(pf, pq, parent);
    }
}

void heapify_down(Pathfinder* pf, NodePQ* pq, int index) {
    int left = 2 * index + 1;
    int right = 2 * index + 2;
    int smallest = index;
    if (left < pq->size && compare_keys(pathfinder_key(pf, pq->nodes[left]), pathfinder_key(pf, pq->nodes[smallest])) < 0) smallest = left;
    if (right < pq->size && compare_keys(pathfinder_key(pf, pq->nodes[right]), pathfinder_key(pf, pq->nodes[smallest])) < 0) smallest = right;
    if (smallest != index) {
        pq_swap(pf, &pq->nodes[index], &pq->nodes[smallest]);
        if (pf) pf->heap_moves_this_call++;
        heapify_down(pf, pq, smallest);
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
    cell->in_pq = TRUE;
    cell->pq_index = pq->size;
    pq->nodes[pq->size++] = node;
    heapify_up(pf, pq, pq->size - 1);
}

Node* pq_pop(Pathfinder* pf, NodePQ* pq) {
    if (pq->size == 0) return nullptr;
    Node* top = pq->nodes[0];
    SearchCell* top_cell = pathfinder_cell(pf, top);
    top_cell->in_pq = FALSE;
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
    cell->in_pq = FALSE;
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
            pf->cells[y][x].in_pq = FALSE;
            pf->cells[y][x].pq_index = -1;
            pf->cells[y][x].key = make_key(kPathfinderInf, kPathfinderInf);
        }
    }
}

}  // namespace

Pathfinder* pathfinder_create(Node* start, Node* goal, const Agent* agent) {
    Pathfinder* pf = new Pathfinder();
    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT);
    pf->start_node = start;
    pf->last_start = start;
    pf->goal_node = goal;
    pf->km = 0.0;
    pf->agent = agent;
    pf->nodes_expanded_this_call = 0;
    pf->heap_moves_this_call = 0;
    pf->nodes_generated_this_call = 0;
    pf->valid_expansions_this_call = 0;
    reset_pathfinder_cells(pf);

    if (goal) {
        SearchCell* goal_cell = &pf->cells[goal->y][goal->x];
        goal_cell->rhs = 0.0;
        goal_cell->key = calculate_key(pf, goal);
        pq_push(pf, &pf->pq, goal);
    }
    return pf;
}

void pathfinder_destroy(Pathfinder* pf) {
    if (!pf) return;
    pq_free(&pf->pq);
    delete pf;
}

void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal) {
    pf->goal_node = new_goal;
    pf->km = 0.0;
    pf->last_start = pf->start_node;
    pf->pq.size = 0;
    reset_pathfinder_cells(pf);
    if (new_goal) {
        pf->cells[new_goal->y][new_goal->x].rhs = 0.0;
        pf->cells[new_goal->y][new_goal->x].key = calculate_key(pf, new_goal);
        pq_push(pf, &pf->pq, new_goal);
    }
}

void pathfinder_update_start(Pathfinder* pf, Node* new_start) {
    if (new_start == nullptr) return;
    if (pf->start_node == nullptr) {
        pf->start_node = new_start;
        pf->last_start = new_start;
        return;
    }
    pf->km += heuristic(pf->last_start, new_start);
    pf->last_start = new_start;
    pf->start_node = new_start;
}

void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed) {
    update_vertex(pf, map, am, changed);
    for (int i = 0; i < 4; i++) {
        int prev_x = changed->x + kDir4X[i];
        int prev_y = changed->y + kDir4Y[i];
        if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(pf, map, am, &map->grid[prev_y][prev_x]);
    }
}

void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am) {
    if (!pf->start_node || !pf->goal_node) return;

    pf->nodes_expanded_this_call = 0;
    pf->heap_moves_this_call = 0;

    while (true) {
        Key top = pq_top_key(pf, &pf->pq);
        SearchCell* start_cell = pathfinder_cell(pf, pf->start_node);
        Key start_key = calculate_key(pf, pf->start_node);

        if (pf->pq.size == 0 || (compare_keys(top, start_key) >= 0 && std::fabs(start_cell->rhs - start_cell->g) < 1e-9)) break;

        Key old_key = top;
        Node* node = pq_pop(pf, &pf->pq);
        if (node) pf->nodes_expanded_this_call++;
        SearchCell* cell = pathfinder_cell(pf, node);
        Key new_key = calculate_key(pf, node);

        if (compare_keys(old_key, new_key) < 0) {
            cell->key = new_key;
            pq_push(pf, &pf->pq, node);
        } else if (cell->g > cell->rhs) {
            cell->g = cell->rhs;
            pf->valid_expansions_this_call++;
            for (int i = 0; i < 4; i++) {
                int prev_x = node->x + kDir4X[i];
                int prev_y = node->y + kDir4Y[i];
                if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(pf, map, am, &map->grid[prev_y][prev_x]);
            }
        } else {
            cell->g = kPathfinderInf;
            update_vertex(pf, map, am, node);
            for (int i = 0; i < 4; i++) {
                int prev_x = node->x + kDir4X[i];
                int prev_y = node->y + kDir4Y[i];
                if (grid_is_valid_coord(prev_x, prev_y)) update_vertex(pf, map, am, &map->grid[prev_y][prev_x]);
            }
        }
    }
}

Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current) {
    if (!pf->goal_node || !current) return current;
    SearchCell* current_cell = pathfinder_cell(pf, current);
    if (current_cell->g >= kPathfinderInf || current == pf->goal_node) return current;

    double best_cost = kPathfinderInf;
    Node* best_node = current;
    double best_distance = std::fabs((double)current->x - (double)pf->goal_node->x) +
        std::fabs((double)current->y - (double)pf->goal_node->y);

    for (int i = 0; i < 4; i++) {
        int next_x = current->x + kDir4X[i];
        int next_y = current->y + kDir4Y[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        Node* neighbor = &const_cast<GridMap*>(map)->grid[next_y][next_x];
        if (grid_is_node_blocked(map, am, neighbor, pf->agent)) continue;
        double successor_g = pathfinder_cell(pf, neighbor)->g;
        double cost = 1.0 + successor_g;
        if (cost < best_cost) {
            best_cost = cost;
            best_node = neighbor;
            best_distance = pathfinder_manhattan(neighbor, pf->goal_node);
        } else if (std::fabs(cost - best_cost) < 1e-9) {
            double distance = pathfinder_manhattan(neighbor, pf->goal_node);
            if (distance < best_distance) {
                best_node = neighbor;
                best_distance = distance;
            }
        }
    }
    return best_node;
}
