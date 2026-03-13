#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_whca_support.hpp"

#include <algorithm>
#include <climits>
#include <cmath>

namespace {

constexpr double kCollisionInf = 1e18;
constexpr int kCbsBaseExpansions = 128;
constexpr int kCbsMaxExpansionsCap = 1536;
constexpr int kTurn90Wait = 2;
constexpr int kDir5X[5] = {0, 1, -1, 0, 0};
constexpr int kDir5Y[5] = {0, 0, 0, 1, -1};

PlannerMetricsState& planner_metrics(const PlanningContext& context) {
    return *context.planner_metrics;
}

void accumulate_cbs_metrics(
    const PlanningContext& context,
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.whca_nodes_expanded_this_step += nodes_expanded;
    metrics.whca_heap_moves_this_step += heap_moves;
    metrics.whca_generated_nodes_this_step += generated_nodes;
    metrics.whca_valid_expansions_this_step += valid_expansions;
}

void record_cbs_success(const PlanningContext& context, int expansions) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.cbs_ok_last = 1;
    metrics.cbs_exp_last = expansions;
    metrics.cbs_success_sum++;
}

void record_cbs_failure(const PlanningContext& context, int expansions) {
    PlannerMetricsState& metrics = planner_metrics(context);
    metrics.cbs_ok_last = 0;
    metrics.cbs_exp_last = expansions;
    metrics.cbs_fail_sum++;
}

struct PullOverCandidate final {
    Node* node{nullptr};
    int is_goal{0};
    int degree{99};
    int tie_break{99};
};

struct CBSConflict final {
    int a{0};
    int b{0};
    int t{0};
    int is_edge{0};
    int ax{0};
    int ay{0};
    int bx{0};
    int by{0};
    int apx{0};
    int apy{0};
    int bpx{0};
    int bpy{0};
};

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

int st_index(int t, int y, int x) {
    return (t * GRID_WIDTH * GRID_HEIGHT) + (y * GRID_WIDTH) + x;
}

bool candidate_is_pull_over_better(const PullOverCandidate& lhs, const PullOverCandidate& rhs) {
    if (!lhs.node) return false;
    if (!rhs.node) return true;
    if (lhs.is_goal != rhs.is_goal) return lhs.is_goal > rhs.is_goal;
    if (lhs.degree != rhs.degree) return lhs.degree < rhs.degree;
    return lhs.tie_break < rhs.tie_break;
}

int pull_over_neighbor_degree(const GridMap* map, const Node* node) {
    if (!map || !node) return 99;

    int degree = 0;
    for (int index = 1; index < 5; ++index) {
        const int next_x = node->x + kDir5X[index];
        const int next_y = node->y + kDir5Y[index];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        const Node* candidate = &map->grid[next_y][next_x];
        if (candidate->is_obstacle || candidate->is_parked) continue;
        degree++;
    }
    return degree;
}

int pull_over_target_priority(const GridMap* map, const Node* node) {
    if (!map || !node) return 0;
    if (node->is_goal && !node->is_parked) return 3;

    const int degree = pull_over_neighbor_degree(map, node);
    if (degree <= 1) return 2;
    if (degree == 2) return 1;
    return 0;
}

bool pull_over_can_enter(
    const ReservationTable& table,
    const Agent* agent,
    int current_t,
    const Node* current,
    const Node* next,
    int horizon) {
    if (!agent || !current || !next) return false;
    if (next->is_obstacle || next->is_parked) return false;
    if (next->reserved_by_agent != -1 && next->reserved_by_agent != agent->id) return false;

    const int next_t = current_t + 1;
    if (table.isOccupied(next_t, next, horizon)) return false;

    const int previous_occupant = table.occupantAt(current_t, next, horizon);
    const int occupant_into_current = table.occupantAt(next_t, current, horizon);
    if (previous_occupant != -1 && previous_occupant == occupant_into_current) return false;

    return true;
}

Node* reconstruct_pull_over_first_step(
    GridMap* map,
    int start_idx,
    int target_idx,
    const int* prev) {
    if (!map || !prev || target_idx < 0) return nullptr;

    int cursor = target_idx;
    while (prev[cursor] != -1 && prev[cursor] != start_idx) {
        cursor = prev[cursor];
    }

    const int remainder = cursor % (GRID_WIDTH * GRID_HEIGHT);
    const int y = remainder / GRID_WIDTH;
    const int x = remainder % GRID_WIDTH;
    return &map->grid[y][x];
}

Node* try_pull_over_via_search(
    const PlanningContext& context,
    GridMap* map,
    const ReservationTable& table,
    Agent* agent,
    SpaceTimeSearchBuffers& buffers) {
    if (!map || !agent || !agent->pos) return nullptr;

    const int t_limit = context.whcaHorizon();
    const int total = (t_limit + 1) * GRID_WIDTH * GRID_HEIGHT;
    if (total > MAX_TOT) return nullptr;

    unsigned char* visited = buffers.open.data();
    int* prev = buffers.prev.data();
    int* queue = buffers.heap_nodes.data();

    for (int index = 0; index < total; ++index) {
        visited[index] = 0;
        prev[index] = -1;
    }

    const int start_idx = st_index(0, agent->pos->y, agent->pos->x);
    visited[start_idx] = 1;

    int queue_head = 0;
    int queue_tail = 0;
    queue[queue_tail++] = start_idx;

    int best_idx = -1;
    int best_time = INT_MAX;
    PullOverCandidate best_candidate{};

    while (queue_head < queue_tail) {
        const int current_idx = queue[queue_head++];
        const int current_t = current_idx / (GRID_WIDTH * GRID_HEIGHT);
        if (best_idx != -1 && current_t >= best_time) break;
        if (current_t >= t_limit) continue;

        const int remainder = current_idx % (GRID_WIDTH * GRID_HEIGHT);
        const int current_y = remainder / GRID_WIDTH;
        const int current_x = remainder % GRID_WIDTH;
        const Node* current = &map->grid[current_y][current_x];

        for (int dir_index = 1; dir_index <= 5; ++dir_index) {
            const int move_index = (dir_index < 5) ? dir_index : 0;
            const int next_x = current_x + kDir5X[move_index];
            const int next_y = current_y + kDir5Y[move_index];
            if (!grid_is_valid_coord(next_x, next_y)) continue;

            const Node* next = &map->grid[next_y][next_x];
            if (!pull_over_can_enter(table, agent, current_t, current, next, t_limit)) continue;

            const int next_t = current_t + 1;
            const int next_idx = st_index(next_t, next_y, next_x);
            if (visited[next_idx]) continue;

            visited[next_idx] = 1;
            prev[next_idx] = current_idx;
            queue[queue_tail++] = next_idx;

            if (next == agent->pos) continue;

            const int priority = pull_over_target_priority(map, next);
            if (priority <= 0) continue;

            const PullOverCandidate candidate{
                &map->grid[next_y][next_x],
                next->is_goal ? 1 : 0,
                pull_over_neighbor_degree(map, next),
                (next_t * GRID_WIDTH * GRID_HEIGHT) + (next_y * GRID_WIDTH) + next_x,
            };

            if (best_idx == -1 || next_t < best_time ||
                (next_t == best_time && candidate_is_pull_over_better(candidate, best_candidate))) {
                best_idx = next_idx;
                best_time = next_t;
                best_candidate = candidate;
            }
        }
    }

    if (best_idx == -1) return nullptr;
    return reconstruct_pull_over_first_step(map, start_idx, best_idx, prev);
}

bool heap_prefer(double* fvals, int lhs, int rhs) {
    const double lhs_cost = fvals[lhs];
    const double rhs_cost = fvals[rhs];
    if (lhs_cost < rhs_cost - 1e-9) return true;
    if (lhs_cost > rhs_cost + 1e-9) return false;
    return lhs < rhs;
}

void heap_swap(int* nodes, int* pos, int lhs, int rhs, unsigned long long* swap_counter) {
    if (lhs == rhs) return;
    const int lhs_node = nodes[lhs];
    const int rhs_node = nodes[rhs];
    nodes[lhs] = rhs_node;
    nodes[rhs] = lhs_node;
    pos[rhs_node] = lhs;
    pos[lhs_node] = rhs;
    if (swap_counter) (*swap_counter)++;
}

void heap_sift_up(int* nodes, int* pos, double* fvals, int index, unsigned long long* swap_counter) {
    while (index > 0) {
        const int parent = (index - 1) >> 1;
        if (!heap_prefer(fvals, nodes[index], nodes[parent])) break;
        heap_swap(nodes, pos, index, parent, swap_counter);
        index = parent;
    }
}

void heap_sift_down(int* nodes, int* pos, double* fvals, int size, int index, unsigned long long* swap_counter) {
    while (true) {
        const int left = (index << 1) + 1;
        const int right = left + 1;
        int best = index;
        if (left < size && heap_prefer(fvals, nodes[left], nodes[best])) best = left;
        if (right < size && heap_prefer(fvals, nodes[right], nodes[best])) best = right;
        if (best == index) break;
        heap_swap(nodes, pos, index, best, swap_counter);
        index = best;
    }
}

void heap_push(int* nodes, int* pos, double* fvals, int* size, int node, unsigned long long* swap_counter) {
    nodes[*size] = node;
    pos[node] = *size;
    (*size)++;
    heap_sift_up(nodes, pos, fvals, (*size) - 1, swap_counter);
}

int heap_pop(int* nodes, int* pos, double* fvals, int* size, unsigned long long* swap_counter) {
    if (*size == 0) return -1;
    const int root = nodes[0];
    (*size)--;
    if (*size > 0) {
        const int last = nodes[*size];
        nodes[0] = last;
        pos[last] = 0;
        heap_sift_down(nodes, pos, fvals, *size, 0, swap_counter);
    }
    pos[root] = -1;
    return root;
}

void heap_decrease_key(int* nodes, int* pos, double* fvals, int node, unsigned long long* swap_counter) {
    const int index = pos[node];
    if (index >= 0) {
        heap_sift_up(nodes, pos, fvals, index, swap_counter);
    }
}

bool violates_constraint_for(
    int agent,
    int previous_t,
    int previous_x,
    int previous_y,
    int next_x,
    int next_y,
    const CBSConstraint* constraints,
    int constraint_count) {
    for (int index = 0; index < constraint_count; ++index) {
        if (constraints[index].agent != agent) continue;
        if (constraints[index].is_edge) {
            if (constraints[index].t == previous_t &&
                constraints[index].x == previous_x &&
                constraints[index].y == previous_y &&
                constraints[index].tox == next_x &&
                constraints[index].toy == next_y) {
                return true;
            }
        } else if (constraints[index].t == (previous_t + 1) &&
            constraints[index].x == next_x &&
            constraints[index].y == next_y) {
            return true;
        }
    }
    return false;
}

bool st_astar_plan_single(
    int agent_id,
    GridMap* map,
    Node* start,
    Node* goal,
    int horizon,
    std::int16_t ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    SpaceTimeSearchBuffers& buffers,
    const CBSConstraint* constraints,
    int constraint_count,
    Node* out_plan[MAX_WHCA_HORIZON + 1],
    AgentDir initial_heading,
    unsigned long long* out_nodes_expanded,
    unsigned long long* out_heap_moves,
    unsigned long long* out_generated_nodes,
    unsigned long long* out_valid_expansions) {
    if (!start || !map) return false;

    const int t_limit = horizon;
    const int width = GRID_WIDTH;
    const int height = GRID_HEIGHT;
    const int total = (t_limit + 1) * width * height;

    if (out_nodes_expanded) *out_nodes_expanded = 0;
    if (out_heap_moves) *out_heap_moves = 0;
    if (out_generated_nodes) *out_generated_nodes = 0;
    if (out_valid_expansions) *out_valid_expansions = 0;
    if (total > MAX_TOT) return false;

    double* g = buffers.g.data();
    double* f = buffers.f.data();
    unsigned char* open = buffers.open.data();
    unsigned char* closed = buffers.closed.data();
    int* prev = buffers.prev.data();
    int* heap_nodes = buffers.heap_nodes.data();
    int* heap_pos = buffers.heap_pos.data();
    int heap_size = 0;

    for (int index = 0; index < total; ++index) {
        g[index] = kCollisionInf;
        f[index] = kCollisionInf;
        open[index] = 0;
        closed[index] = 0;
        prev[index] = -1;
        heap_pos[index] = -1;
    }

    const int start_x = start->x;
    const int start_y = start->y;
    const int goal_x = goal ? goal->x : start_x;
    const int goal_y = goal ? goal->y : start_y;
    const int start_idx = st_index(0, start_y, start_x);

    g[start_idx] = 0.0;
    f[start_idx] = goal ? manhattan_xy(start_x, start_y, goal_x, goal_y) : 0.0;
    open[start_idx] = 1;

    int best_idx = start_idx;
    double best_val = f[start_idx];
    unsigned long long nodes_expanded = 0;
    unsigned long long heap_moves = 0;
    unsigned long long generated_nodes = 0;
    unsigned long long valid_expansions = 0;

    heap_push(heap_nodes, heap_pos, f, &heap_size, start_idx, &heap_moves);

    while (heap_size > 0) {
        const int current = heap_pop(heap_nodes, heap_pos, f, &heap_size, &heap_moves);
        open[current] = 0;
        closed[current] = 1;
        nodes_expanded++;

        const int current_t = current / (width * height);
        const int remainder = current % (width * height);
        const int current_y = remainder / width;
        const int current_x = remainder % width;

        if (goal && current_x == goal_x && current_y == goal_y) {
            best_idx = current;
            break;
        }
        if (f[current] < best_val) {
            best_val = f[current];
            best_idx = current;
        }
        if (current_t == t_limit) continue;

        for (int move_index = 0; move_index < 5; ++move_index) {
            const int next_x = current_x + kDir5X[move_index];
            const int next_y = current_y + kDir5Y[move_index];
            const int next_t = current_t + 1;
            if (!grid_is_valid_coord(next_x, next_y)) continue;
            if (ext_occ[next_t][next_y][next_x] != -1) continue;
            if (ext_occ[current_t][next_y][next_x] != -1 &&
                ext_occ[next_t][current_y][current_x] == ext_occ[current_t][next_y][next_x]) {
                continue;
            }

            Node* next_cell = &map->grid[next_y][next_x];
            if (next_cell->is_obstacle) continue;
            if (violates_constraint_for(agent_id, current_t, current_x, current_y, next_x, next_y, constraints, constraint_count)) continue;

            const int next_idx = st_index(next_t, next_y, next_x);
            if (closed[next_idx]) continue;
            generated_nodes++;

            double next_g = g[current] + 1.0;
            if (current_t == 0 && !(next_x == current_x && next_y == current_y)) {
                const AgentDir move_heading = dir_from_delta(next_x - current_x, next_y - current_y);
                if (initial_heading != AgentDir::None) {
                    const int turn_steps = dir_turn_steps(initial_heading, move_heading);
                    if (turn_steps == 1) {
                        next_g += static_cast<double>(kTurn90Wait);
                    }
                }
            }

            if (next_g + 1e-9 < g[next_idx]) {
                g[next_idx] = next_g;
                const double heuristic_cost = goal ? manhattan_xy(next_x, next_y, goal_x, goal_y) : 0.0;
                f[next_idx] = next_g + heuristic_cost;
                prev[next_idx] = current;
                if (!open[next_idx]) {
                    open[next_idx] = 1;
                    heap_push(heap_nodes, heap_pos, f, &heap_size, next_idx, &heap_moves);
                } else {
                    heap_decrease_key(heap_nodes, heap_pos, f, next_idx, &heap_moves);
                }
                valid_expansions++;
            }
        }
    }

    int path_idx[MAX_WHCA_HORIZON + 1];
    int plan_length = 0;
    int current = best_idx;
    while (current != -1 && plan_length < (MAX_WHCA_HORIZON + 1)) {
        path_idx[plan_length++] = current;
        current = prev[current];
    }

    if (out_nodes_expanded) *out_nodes_expanded = nodes_expanded;
    if (out_heap_moves) *out_heap_moves = heap_moves;
    if (out_generated_nodes) *out_generated_nodes = generated_nodes;
    if (out_valid_expansions) *out_valid_expansions = valid_expansions;

    if (plan_length == 0) {
        for (int t = 0; t <= t_limit; ++t) out_plan[t] = start;
        return true;
    }

    for (int t = 0; t < plan_length; ++t) {
        const int idx = path_idx[plan_length - 1 - t];
        const int tt = idx / (width * height);
        const int remainder = idx % (width * height);
        const int y = remainder / width;
        const int x = remainder % width;
        if (tt <= t_limit) out_plan[tt] = &map->grid[y][x];
    }

    Node* last = out_plan[plan_length - 1 <= t_limit ? plan_length - 1 : t_limit];
    for (int t = plan_length; t <= t_limit; ++t) out_plan[t] = last;
    return true;
}

double cbs_cost_sum_adv(const int ids[], int count, const CBSNode& node, Node* goals[MAX_AGENTS], int horizon) {
    constexpr double alpha = 1.0;
    constexpr double beta = 0.5;
    constexpr double gamma = 0.1;
    double sum = 0.0;
    for (int index = 0; index < count; ++index) {
        const int id = ids[index];
        int moves = 0;
        int waits = 0;
        for (int t = 1; t <= horizon; ++t) {
            if (!node.plans[index][t] || !node.plans[index][t - 1]) {
                waits += horizon;
                break;
            }
            if (node.plans[index][t] != node.plans[index][t - 1]) {
                moves++;
            } else {
                waits++;
            }
        }
        double heuristic_residual = 0.0;
        if (goals[id]) {
            Node* last = node.plans[index][horizon];
            if (last) {
                heuristic_residual = manhattan_xy(last->x, last->y, goals[id]->x, goals[id]->y);
            }
        }
        sum += alpha * moves + beta * waits + gamma * heuristic_residual;
    }
    return sum;
}

bool detect_first_conflict(const CBSNode& node, const int ids[], int count, CBSConflict* out, int horizon) {
    for (int t = 1; t <= horizon; ++t) {
        for (int lhs_index = 0; lhs_index < count; ++lhs_index) {
            for (int rhs_index = lhs_index + 1; rhs_index < count; ++rhs_index) {
                const int lhs = ids[lhs_index];
                const int rhs = ids[rhs_index];
                Node* lhs_t = node.plans[lhs_index][t];
                Node* rhs_t = node.plans[rhs_index][t];
                Node* lhs_prev = node.plans[lhs_index][t - 1];
                Node* rhs_prev = node.plans[rhs_index][t - 1];
                if (!lhs_t || !rhs_t || !lhs_prev || !rhs_prev) {
                    continue;
                }

                if (lhs_t == rhs_t) {
                    out->a = lhs;
                    out->b = rhs;
                    out->t = t;
                    out->is_edge = 0;
                    out->ax = lhs_t->x;
                    out->ay = lhs_t->y;
                    out->bx = rhs_t->x;
                    out->by = rhs_t->y;
                    out->apx = lhs_prev->x;
                    out->apy = lhs_prev->y;
                    out->bpx = rhs_prev->x;
                    out->bpy = rhs_prev->y;
                    return true;
                }

                if (lhs_t == rhs_prev && rhs_t == lhs_prev) {
                    out->a = lhs;
                    out->b = rhs;
                    out->t = t;
                    out->is_edge = 1;
                    out->ax = lhs_prev->x;
                    out->ay = lhs_prev->y;
                    out->bx = rhs_prev->x;
                    out->by = rhs_prev->y;
                    out->apx = lhs_prev->x;
                    out->apy = lhs_prev->y;
                    out->bpx = rhs_prev->x;
                    out->bpy = rhs_prev->y;
                    return true;
                }
            }
        }
    }
    return false;
}

void copy_cbs_node_for_group(const CBSNode& source, int group_n, int horizon, CBSNode* destination) {
    if (!destination) return;

    destination->ncons = source.ncons;
    destination->cost = source.cost;
    for (int index = 0; index < source.ncons; ++index) {
        destination->cons[index] = source.cons[index];
    }
    for (int group_index = 0; group_index < group_n; ++group_index) {
        for (int t = 0; t <= horizon; ++t) {
            destination->plans[group_index][t] = source.plans[group_index][t];
        }
    }
}

int find_group_index(const std::array<int, MAX_CBS_GROUP>& group_ids, int group_n, int agent_id) {
    for (int group_index = 0; group_index < group_n; ++group_index) {
        if (group_ids[group_index] == agent_id) {
            return group_index;
        }
    }
    return -1;
}

void copy_ext_occ_without_group(
    const PlanningContext& context,
    const ReservationTable& base,
    AgentMask group_mask,
    std::int16_t out_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]) {
    for (int t = 0; t <= context.whcaHorizon(); ++t) {
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                const int occupant = base.occupantAt(t, y, x, context.whcaHorizon());
                out_occ[t][y][x] = static_cast<std::int16_t>((occupant != -1 && group_mask.contains(occupant)) ? -1 : occupant);
            }
        }
    }
}

bool cbs_heap_push(
    std::array<CBSNode, MAX_CBS_NODES>& nodes,
    int* next_unused_index,
    int* free_indices,
    int* free_count,
    int* heap,
    int* size,
    const CBSNode& node) {
    if (*size >= MAX_CBS_NODES) return false;

    int node_index = -1;
    if (*free_count > 0) {
        node_index = free_indices[--(*free_count)];
    } else if (*next_unused_index < MAX_CBS_NODES) {
        node_index = (*next_unused_index)++;
    }
    if (node_index < 0) return false;

    nodes[node_index] = node;
    heap[*size] = node_index;
    int index = *size;
    (*size)++;
    while (index > 0) {
        const int parent = (index - 1) / 2;
        if (nodes[heap[parent]].cost <= nodes[heap[index]].cost) break;
        std::swap(heap[parent], heap[index]);
        index = parent;
    }
    return true;
}

int cbs_heap_pop(std::array<CBSNode, MAX_CBS_NODES>& nodes, int* heap, int* size) {
    if (*size <= 0) return -1;
    const int root_index = heap[0];
    *size = *size - 1;
    if (*size > 0) {
        heap[0] = heap[*size];
        int index = 0;
        while (true) {
            const int left = 2 * index + 1;
            const int right = 2 * index + 2;
            int smallest = index;
            if (left < *size && nodes[heap[left]].cost < nodes[heap[smallest]].cost) smallest = left;
            if (right < *size && nodes[heap[right]].cost < nodes[heap[smallest]].cost) smallest = right;
            if (smallest == index) break;
            std::swap(heap[smallest], heap[index]);
            index = smallest;
        }
    }
    return root_index;
}

void cbs_release_node_index(int node_index, int* free_indices, int* free_count) {
    if (node_index < 0 || *free_count >= MAX_CBS_NODES) {
        return;
    }
    free_indices[(*free_count)++] = node_index;
}

bool cbs_plan_agent_with_metrics(
    const PlanningContext& context,
    AgentManager* manager,
    GridMap* map,
    int agent_id,
    std::int16_t ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    SpaceTimeSearchBuffers& buffers,
    const CBSConstraint* constraints,
    int constraint_count,
    Node* out_plan[MAX_WHCA_HORIZON + 1]) {
    Agent* agent = &manager->agents[agent_id];
    unsigned long long nodes_expanded = 0;
    unsigned long long heap_moves = 0;
    unsigned long long generated_nodes = 0;
    unsigned long long valid_expansions = 0;

    if (!st_astar_plan_single(
            agent_id,
            map,
            agent->pos,
            agent->goal,
            context.whcaHorizon(),
            ext_occ,
            buffers,
            constraints,
            constraint_count,
            out_plan,
            agent->heading,
            &nodes_expanded,
            &heap_moves,
            &generated_nodes,
            &valid_expansions)) {
        return false;
    }

    accumulate_cbs_metrics(context, nodes_expanded, heap_moves, generated_nodes, valid_expansions);
    return true;
}

int cbs_expansion_budget(int group_n) {
    const int scaled = kCbsBaseExpansions + std::max(0, group_n - 2) * 96;
    return std::min(kCbsMaxExpansionsCap, scaled);
}

}  // namespace

ReservationTable::ReservationTable() {
    occ_.fill(-1);
    touched_bits_.fill(0);
}

int ReservationTable::flatIndex(int t, int y, int x) {
    return (t * GRID_WIDTH * GRID_HEIGHT) + (y * GRID_WIDTH) + x;
}

void ReservationTable::clearTouchedEntries() {
    for (int i = 0; i < touched_count_; ++i) {
        const int index = touched_indices_[i];
        occ_[index] = -1;
        touched_bits_[index / 64] &= ~(std::uint64_t{1} << (index % 64));
    }
    touched_count_ = 0;
}

void ReservationTable::rememberTouchedIndex(int index) {
    if (index < 0 || index >= static_cast<int>(occ_.size())) {
        return;
    }

    const std::uint64_t bit = std::uint64_t{1} << (index % 64);
    std::uint64_t& word = touched_bits_[index / 64];
    if ((word & bit) != 0) {
        return;
    }

    word |= bit;
    if (touched_count_ < static_cast<int>(touched_indices_.size())) {
        touched_indices_[touched_count_++] = static_cast<std::uint16_t>(index);
    }
}

void ReservationTable::clear() {
    clearTouchedEntries();
}

void ReservationTable::clearAgent(int agent_id, int horizon) {
    const int t_limit = std::min(horizon, MAX_WHCA_HORIZON);
    for (int i = 0; i < touched_count_; ++i) {
        const int index = touched_indices_[i];
        const int t = index / (GRID_WIDTH * GRID_HEIGHT);
        if (t < 1 || t > t_limit) continue;
        if (occ_[index] == agent_id) {
            occ_[index] = -1;
        }
    }
}

void ReservationTable::seedCurrent(AgentManager* manager) {
    if (!manager) return;
    for (int index = 0; index < MAX_AGENTS; ++index) {
        Agent* agent = &manager->agents[index];
        if (agent->pos && agent->state != AgentState::Charging) {
            setOccupant(0, agent->pos, agent->id, 0);
        }
    }
}

bool ReservationTable::isOccupied(int t, const Node* node, int horizon) const {
    if (!node || t < 0 || t > horizon) return true;
    return occ_[flatIndex(t, node->y, node->x)] != -1;
}

int ReservationTable::occupantAt(int t, const Node* node, int horizon) const {
    if (!node) return -1;
    return occupantAt(t, node->y, node->x, horizon);
}

int ReservationTable::occupantAt(int t, int y, int x, int horizon) const {
    if (t < 0 || t > horizon || !grid_is_valid_coord(x, y)) return -1;
    return occ_[flatIndex(t, y, x)];
}

void ReservationTable::setOccupant(int t, const Node* node, int agent_id, int horizon) {
    if (!node || t < 0 || t > horizon) return;
    const int index = flatIndex(t, node->y, node->x);
    rememberTouchedIndex(index);
    occ_[index] = static_cast<std::int16_t>(agent_id);
}

void WHCA_adjustHorizon(const PlanningContext& context, int wf_edges, int scc, Logger* logger) {
    RuntimeTuningState& tuning = *context.runtime_tuning;
    PlannerMetricsState& metrics = *context.planner_metrics;
    int conflict_score = static_cast<int>(tuning.conflict_score * 0.6) + wf_edges + (scc ? 5 : 0);
    int horizon = tuning.whca_horizon;
    const int old_horizon = horizon;
    constexpr int kHighConflict = 24;
    constexpr int kLowConflict = 10;

    if (conflict_score > kHighConflict && horizon < MAX_WHCA_HORIZON) horizon += 2;
    else if (conflict_score < kLowConflict && horizon > MIN_WHCA_HORIZON) horizon -= 2;

    if (horizon < MIN_WHCA_HORIZON) horizon = MIN_WHCA_HORIZON;
    if (horizon > MAX_WHCA_HORIZON) horizon = MAX_WHCA_HORIZON;

    if (old_horizon != horizon) {
        logger_log_event(
            logger,
            "Planner",
            "Info",
            std::nullopt,
            std::nullopt,
            "[WHCA] Horizon adjusted %d -> %d (score=%d)",
            old_horizon,
            horizon,
            conflict_score);
    }

    tuning.conflict_score = conflict_score;
    tuning.whca_horizon = horizon;
    metrics.whca_h = horizon;
}

ConflictGraphSummary analyze_conflict_graph(const WaitEdgeBuffer& wait_edges) {
    int adjacency[MAX_AGENTS][MAX_AGENTS] = {{0}};
    for (int index = 0; index < wait_edges.count; ++index) {
        const int from = wait_edges.edges[index].from_id;
        const int to = wait_edges.edges[index].to_id;
        if (from >= 0 && to >= 0 && from != to) {
            adjacency[from][to] = 1;
        }
    }

    int reach[MAX_AGENTS][MAX_AGENTS] = {{0}};
    for (int row = 0; row < MAX_AGENTS; ++row) {
        for (int col = 0; col < MAX_AGENTS; ++col) {
            reach[row][col] = adjacency[row][col];
        }
    }

    for (int mid = 0; mid < MAX_AGENTS; ++mid) {
        for (int row = 0; row < MAX_AGENTS; ++row) {
            for (int col = 0; col < MAX_AGENTS; ++col) {
                reach[row][col] = reach[row][col] || (reach[row][mid] && reach[mid][col]);
            }
        }
    }

    AgentMask mask{};
    for (int row = 0; row < MAX_AGENTS; ++row) {
        for (int col = 0; col < MAX_AGENTS; ++col) {
            if (row != col && reach[row][col] && reach[col][row]) {
                mask.set(row);
                mask.set(col);
            }
        }
    }
    return ConflictGraphSummary{mask, wait_edges.count};
}

Node* try_pull_over(
    const PlanningContext& context,
    const ReservationTable& table,
    Agent* agent,
    DefaultPlannerScratch& scratch) {
    GridMap* map = context.map;
    if (Node* searched = try_pull_over_via_search(context, map, table, agent, scratch.pull_over_search)) {
        return searched;
    }

    PullOverCandidate best{};
    for (int index = 1; index < 5; ++index) {
        const int next_x = agent->pos->x + kDir5X[index];
        const int next_y = agent->pos->y + kDir5Y[index];
        if (!grid_is_valid_coord(next_x, next_y)) continue;

        Node* candidate = &map->grid[next_y][next_x];
        if (candidate->is_obstacle || candidate->is_parked) continue;
        if (candidate->reserved_by_agent != -1 && candidate->reserved_by_agent != agent->id) continue;
        if (table.isOccupied(1, candidate, context.whcaHorizon())) continue;

        const PullOverCandidate current{
            candidate,
            candidate->is_goal ? 1 : 0,
            pull_over_neighbor_degree(map, candidate),
            index,
        };
        if (candidate_is_pull_over_better(current, best)) {
            best = current;
        }
    }

    if (best.node) return best.node;

    Node* current = agent->pos;
    if (current && !table.isOccupied(1, current, context.whcaHorizon())) {
        return current;
    }
    return agent->pos;
}

CbsSolveResult run_partial_CBS(
    const PlanningContext& context,
    const std::array<int, MAX_CBS_GROUP>& group_ids,
    int group_n,
    const ReservationTable& base_rt,
    DefaultPlannerScratch& scratch) {
    CbsSolveResult result{};
    if (group_n <= 1) return result;

    AgentManager* manager = context.agents;
    GridMap* map = context.map;
    Logger* logger = context.logger;
    const int horizon = context.whcaHorizon();

    AgentMask group_mask{};
    for (int index = 0; index < group_n; ++index) {
        group_mask.set(group_ids[index]);
    }

    copy_ext_occ_without_group(context, base_rt, group_mask, scratch.ext_occ);

    int heap_size = 0;
    int next_node_index = 0;
    std::array<int, MAX_CBS_NODES> free_node_indices{};
    int free_node_count = 0;
    int expansions = 0;
    const int expansion_budget = cbs_expansion_budget(group_n);

    CBSNode root{};
    for (int group_index = 0; group_index < group_n; ++group_index) {
        const int agent_id = group_ids[group_index];
        Node* plan[MAX_WHCA_HORIZON + 1];
        if (!cbs_plan_agent_with_metrics(
                context,
                manager,
                map,
                agent_id,
                scratch.ext_occ,
                scratch.cbs_search,
                root.cons,
                root.ncons,
                plan)) {
            record_cbs_failure(context, expansions);
            return result;
        }
        for (int t = 0; t <= horizon; ++t) {
            root.plans[group_index][t] = plan[t];
        }
    }

    Node* goals[MAX_AGENTS] = {0};
    for (int index = 0; index < group_n; ++index) {
        goals[group_ids[index]] = manager->agents[group_ids[index]].goal;
    }
    root.cost = cbs_cost_sum_adv(group_ids.data(), group_n, root, goals, horizon);
    if (!cbs_heap_push(
            scratch.cbs_nodes,
            &next_node_index,
            free_node_indices.data(),
            &free_node_count,
            scratch.cbs_heap_indices.data(),
            &heap_size,
            root)) {
        record_cbs_failure(context, expansions);
        return result;
    }

    while (heap_size > 0 && expansions < expansion_budget) {
        const int current_index = cbs_heap_pop(scratch.cbs_nodes, scratch.cbs_heap_indices.data(), &heap_size);
        if (current_index < 0) break;
        CBSNode* current = &scratch.cbs_nodes[current_index];
        expansions++;
        if (expansions > expansion_budget) break;

        CBSConflict conflict{};
        if (!detect_first_conflict(*current, group_ids.data(), group_n, &conflict, horizon)) {
            for (int group_index = 0; group_index < group_n; ++group_index) {
                const int agent_id = group_ids[group_index];
                for (int t = 0; t <= horizon; ++t) {
                    result.plans[agent_id][t] = current->plans[group_index][t];
                }
            }
            logger_log_event(
                logger,
                "Planner",
                "Info",
                std::nullopt,
                std::nullopt,
                "[CBS] Partial CBS succeeded (group=%d agents, expansions=%d).",
                group_n,
                expansions);
            record_cbs_success(context, expansions);
            result.solved = true;
            result.expansions = expansions;
            return result;
        }

        std::array<CBSNode, 2> pending_children{};
        int pending_children_count = 0;
        for (int branch = 0; branch < 2; ++branch) {
            CBSNode child{};
            copy_cbs_node_for_group(*current, group_n, horizon, &child);
            if (child.ncons >= MAX_CBS_CONS) continue;

            CBSConstraint constraint{};
            constraint.agent = (branch == 0) ? conflict.a : conflict.b;
            if (conflict.is_edge) {
                constraint.is_edge = 1;
                constraint.t = conflict.t - 1;
                if (branch == 0) {
                    constraint.x = conflict.apx;
                    constraint.y = conflict.apy;
                    constraint.tox = conflict.bpx;
                    constraint.toy = conflict.bpy;
                } else {
                    constraint.x = conflict.bpx;
                    constraint.y = conflict.bpy;
                    constraint.tox = conflict.apx;
                    constraint.toy = conflict.apy;
                }
            } else {
                constraint.is_edge = 0;
                constraint.t = conflict.t;
                constraint.x = conflict.ax;
                constraint.y = conflict.ay;
            }
            child.cons[child.ncons++] = constraint;

            const int constrained_group_index = find_group_index(group_ids, group_n, constraint.agent);
            if (constrained_group_index < 0) {
                continue;
            }

            Node* plan[MAX_WHCA_HORIZON + 1];
            if (!cbs_plan_agent_with_metrics(
                    context,
                    manager,
                    map,
                    constraint.agent,
                    scratch.ext_occ,
                    scratch.cbs_search,
                    child.cons,
                    child.ncons,
                    plan)) {
                continue;
            }
            for (int t = 0; t <= horizon; ++t) {
                child.plans[constrained_group_index][t] = plan[t];
            }

            child.cost = cbs_cost_sum_adv(group_ids.data(), group_n, child, goals, horizon);
            if (pending_children_count < static_cast<int>(pending_children.size())) {
                pending_children[pending_children_count++] = child;
            }
        }

        cbs_release_node_index(current_index, free_node_indices.data(), &free_node_count);
        for (int child_index = 0; child_index < pending_children_count; ++child_index) {
            if (!cbs_heap_push(
                    scratch.cbs_nodes,
                    &next_node_index,
                    free_node_indices.data(),
                    &free_node_count,
                    scratch.cbs_heap_indices.data(),
                    &heap_size,
                    pending_children[child_index])) {
                break;
            }
        }
    }

    logger_log_event(
        logger,
        "Planner",
        "Warn",
        std::nullopt,
        std::nullopt,
        "[CBS] Partial CBS failed within the search budget (%d). Falling back to pull-over.",
        expansion_budget);
    record_cbs_failure(context, expansions);
    result.expansions = expansions;
    return result;
}
