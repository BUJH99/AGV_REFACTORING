#define _CRT_SECURE_NO_WARNINGS

#include "collision_planner_whca_support.hpp"

#include <array>
#include <climits>
#include <cmath>
#include <cstring>

#define grid_is_valid_coord Grid_isValidCoord

int grid_is_valid_coord(int x, int y);
void logger_log(Logger* logger, const char* format, ...);

namespace {

constexpr double kCollisionInf = 1e18;
constexpr int kCbsBaseExpansions = 128;
constexpr int kCbsMaxExpansionsCap = 768;
constexpr int kMaxCbsNodes = 1024;
constexpr int kMaxTot = ((MAX_WHCA_HORIZON) + 1) * GRID_WIDTH * GRID_HEIGHT;
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

struct SpaceTimeSearchBuffers final {
    std::array<double, kMaxTot> g{};
    std::array<double, kMaxTot> f{};
    std::array<unsigned char, kMaxTot> open{};
    std::array<unsigned char, kMaxTot> closed{};
    std::array<int, kMaxTot> prev{};
    std::array<int, kMaxTot> heap_nodes{};
    std::array<int, kMaxTot> heap_pos{};
};

SpaceTimeSearchBuffers& search_buffers_local() {
    static SpaceTimeSearchBuffers buffers{};
    return buffers;
}

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

int st_index_local(int t, int y, int x) {
    return (t * GRID_WIDTH * GRID_HEIGHT) + (y * GRID_WIDTH) + x;
}

int candidate_is_pull_over_better(const PullOverCandidate& lhs, const PullOverCandidate& rhs) {
    if (!lhs.node) return 0;
    if (!rhs.node) return 1;
    if (lhs.is_goal != rhs.is_goal) return lhs.is_goal > rhs.is_goal;
    if (lhs.degree != rhs.degree) return lhs.degree < rhs.degree;
    return lhs.tie_break < rhs.tie_break;
}

int pull_over_neighbor_degree(const GridMap* map, const Node* node) {
    if (!map || !node) return 99;

    int degree = 0;
    for (int i = 1; i < 5; ++i) {
        const int next_x = node->x + kDir5X[i];
        const int next_y = node->y + kDir5Y[i];
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

int pull_over_can_enter(
    const ReservationTable* table,
    const Agent* agent,
    int current_t,
    const Node* current,
    const Node* next,
    int horizon) {
    if (!table || !agent || !current || !next) return 0;
    if (next->is_obstacle || next->is_parked) return 0;
    if (next->reserved_by_agent != -1 && next->reserved_by_agent != agent->id) return 0;

    const int next_t = current_t + 1;
    if (ReservationTable_isOccupied(table, next_t, next, horizon)) return 0;

    const int previous_occupant = ReservationTable_getOccupant(table, current_t, next, horizon);
    const int occupant_into_current = ReservationTable_getOccupant(table, next_t, current, horizon);
    if (previous_occupant != -1 && previous_occupant == occupant_into_current) return 0;

    return 1;
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

Node* try_pull_over_via_search(const PlanningContext& context, GridMap* map, const ReservationTable* table, Agent* agent) {
    if (!map || !table || !agent || !agent->pos) return nullptr;

    const int t_limit = context.whcaHorizon();
    const int total = (t_limit + 1) * GRID_WIDTH * GRID_HEIGHT;
    if (total > kMaxTot) return nullptr;

    SpaceTimeSearchBuffers& buffers = search_buffers_local();
    unsigned char* visited = buffers.open.data();
    int* prev = buffers.prev.data();
    int* queue = buffers.heap_nodes.data();

    for (int i = 0; i < total; ++i) {
        visited[i] = 0;
        prev[i] = -1;
    }

    const int start_idx = st_index_local(0, agent->pos->y, agent->pos->x);
    visited[start_idx] = 1;

    int queue_head = 0;
    int queue_tail = 0;
    queue[queue_tail++] = start_idx;

    int best_idx = -1;
    int best_time = INT_MAX;
    PullOverCandidate best_candidate{};

    while (queue_head < queue_tail) {
        const int cur = queue[queue_head++];
        const int cur_t = cur / (GRID_WIDTH * GRID_HEIGHT);
        if (best_idx != -1 && cur_t >= best_time) break;
        if (cur_t >= t_limit) continue;

        const int remainder = cur % (GRID_WIDTH * GRID_HEIGHT);
        const int cur_y = remainder / GRID_WIDTH;
        const int cur_x = remainder % GRID_WIDTH;
        const Node* current = &map->grid[cur_y][cur_x];

        for (int dir_index = 1; dir_index <= 5; ++dir_index) {
            const int move_index = (dir_index < 5) ? dir_index : 0;
            const int next_x = cur_x + kDir5X[move_index];
            const int next_y = cur_y + kDir5Y[move_index];
            if (!grid_is_valid_coord(next_x, next_y)) continue;

            const Node* next = &map->grid[next_y][next_x];
            if (!pull_over_can_enter(table, agent, cur_t, current, next, t_limit)) continue;

            const int next_t = cur_t + 1;
            const int next_idx = st_index_local(next_t, next_y, next_x);
            if (visited[next_idx]) continue;

            visited[next_idx] = 1;
            prev[next_idx] = cur;
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

int heap_prefer(double* fvals, int a, int b) {
    const double fa = fvals[a];
    const double fb = fvals[b];
    if (fa < fb - 1e-9) return 1;
    if (fa > fb + 1e-9) return 0;
    return a < b;
}

void heap_swap(int* nodes, int* pos, int i, int j, unsigned long long* swap_counter) {
    if (i == j) return;
    const int node_i = nodes[i];
    const int node_j = nodes[j];
    nodes[i] = node_j;
    nodes[j] = node_i;
    pos[node_j] = i;
    pos[node_i] = j;
    if (swap_counter) (*swap_counter)++;
}

void heap_sift_up(int* nodes, int* pos, double* fvals, int idx, unsigned long long* swap_counter) {
    while (idx > 0) {
        const int parent = (idx - 1) >> 1;
        if (!heap_prefer(fvals, nodes[idx], nodes[parent])) break;
        heap_swap(nodes, pos, idx, parent, swap_counter);
        idx = parent;
    }
}

void heap_sift_down(int* nodes, int* pos, double* fvals, int size, int idx, unsigned long long* swap_counter) {
    while (true) {
        const int left = (idx << 1) + 1;
        const int right = left + 1;
        int best = idx;
        if (left < size && heap_prefer(fvals, nodes[left], nodes[best])) best = left;
        if (right < size && heap_prefer(fvals, nodes[right], nodes[best])) best = right;
        if (best == idx) break;
        heap_swap(nodes, pos, idx, best, swap_counter);
        idx = best;
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
    const int idx = pos[node];
    if (idx >= 0) heap_sift_up(nodes, pos, fvals, idx, swap_counter);
}

int violates_constraint_for(int agent, int t_prev, int x_prev, int y_prev, int x_new, int y_new,
    const CBSConstraint* constraints, int constraint_count) {
    for (int i = 0; i < constraint_count; ++i) {
        if (constraints[i].agent != agent) continue;
        if (constraints[i].is_edge) {
            if (constraints[i].t == t_prev &&
                constraints[i].x == x_prev &&
                constraints[i].y == y_prev &&
                constraints[i].tox == x_new &&
                constraints[i].toy == y_new) {
                return 1;
            }
        } else if (constraints[i].t == (t_prev + 1) &&
            constraints[i].x == x_new &&
            constraints[i].y == y_new) {
            return 1;
        }
    }
    return 0;
}

int st_astar_plan_single(
    int agent_id,
    GridMap* map,
    Node* start,
    Node* goal,
    int horizon,
    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    const CBSConstraint* constraints,
    int constraint_count,
    Node* out_plan[MAX_WHCA_HORIZON + 1],
    AgentDir initial_heading,
    unsigned long long* out_nodes_expanded,
    unsigned long long* out_heap_moves,
    unsigned long long* out_generated_nodes,
    unsigned long long* out_valid_expansions) {
    if (!start) return 0;

    const int t_limit = horizon;
    const int width = GRID_WIDTH;
    const int height = GRID_HEIGHT;
    const int total = (t_limit + 1) * width * height;

    if (out_nodes_expanded) *out_nodes_expanded = 0;
    if (out_heap_moves) *out_heap_moves = 0;
    if (out_generated_nodes) *out_generated_nodes = 0;
    if (out_valid_expansions) *out_valid_expansions = 0;
    if (total > kMaxTot) return 0;

    SpaceTimeSearchBuffers& buffers = search_buffers_local();
    double* g = buffers.g.data();
    double* f = buffers.f.data();
    unsigned char* open = buffers.open.data();
    unsigned char* closed = buffers.closed.data();
    int* prev = buffers.prev.data();
    int* heap_nodes = buffers.heap_nodes.data();
    int* heap_pos = buffers.heap_pos.data();
    int heap_size = 0;

    for (int i = 0; i < total; ++i) {
        g[i] = kCollisionInf;
        f[i] = kCollisionInf;
        open[i] = 0;
        closed[i] = 0;
        prev[i] = -1;
        heap_pos[i] = -1;
    }

#ifndef ST_INDEX
#define ST_INDEX(t,y,x,width,height) ((t) * (width) * (height) + (y) * (width) + (x))
#endif

    const int start_x = start->x;
    const int start_y = start->y;
    const int goal_x = goal ? goal->x : start_x;
    const int goal_y = goal ? goal->y : start_y;
    const int start_idx = ST_INDEX(0, start_y, start_x, width, height);

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
        const int cur = heap_pop(heap_nodes, heap_pos, f, &heap_size, &heap_moves);
        open[cur] = 0;
        closed[cur] = 1;
        nodes_expanded++;

        const int cur_t = cur / (width * height);
        const int remainder = cur % (width * height);
        const int cur_y = remainder / width;
        const int cur_x = remainder % width;

        if (goal && cur_x == goal_x && cur_y == goal_y) {
            best_idx = cur;
            break;
        }
        if (f[cur] < best_val) {
            best_val = f[cur];
            best_idx = cur;
        }
        if (cur_t == t_limit) continue;

        for (int k = 0; k < 5; ++k) {
            const int next_x = cur_x + kDir5X[k];
            const int next_y = cur_y + kDir5Y[k];
            const int next_t = cur_t + 1;
            if (!grid_is_valid_coord(next_x, next_y)) continue;
            if (ext_occ[next_t][next_y][next_x] != -1) continue;
            if (ext_occ[cur_t][next_y][next_x] != -1 &&
                ext_occ[next_t][cur_y][cur_x] == ext_occ[cur_t][next_y][next_x]) {
                continue;
            }

            Node* next_cell = &map->grid[next_y][next_x];
            if (next_cell->is_obstacle) continue;
            if (violates_constraint_for(agent_id, cur_t, cur_x, cur_y, next_x, next_y, constraints, constraint_count)) continue;

            const int next_idx = ST_INDEX(next_t, next_y, next_x, width, height);
            if (closed[next_idx]) continue;
            generated_nodes++;

            double next_g = g[cur] + 1.0;
            if (cur_t == 0 && !(next_x == cur_x && next_y == cur_y)) {
                const AgentDir move_heading = dir_from_delta(next_x - cur_x, next_y - cur_y);
                if (initial_heading != DIR_NONE) {
                    const int turn_steps = dir_turn_steps(initial_heading, move_heading);
                    if (turn_steps == 1) next_g += static_cast<double>(kTurn90Wait);
                }
            }

            if (next_g + 1e-9 < g[next_idx]) {
                g[next_idx] = next_g;
                const double heuristic = goal ? manhattan_xy(next_x, next_y, goal_x, goal_y) : 0.0;
                f[next_idx] = next_g + heuristic;
                prev[next_idx] = cur;
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
        return 1;
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
    return 1;
}

struct CBSConflict {
    int a;
    int b;
    int t;
    int is_edge;
    int ax;
    int ay;
    int bx;
    int by;
    int apx;
    int apy;
    int bpx;
    int bpy;
};

double cbs_cost_sum_adv(int ids[], int count, Node* const plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], Node* goals[MAX_AGENTS], int horizon) {
    constexpr double alpha = 1.0;
    constexpr double beta = 0.5;
    constexpr double gamma = 0.1;
    double sum = 0.0;
    for (int i = 0; i < count; ++i) {
        const int id = ids[i];
        int moves = 0;
        int waits = 0;
        for (int t = 1; t <= horizon; ++t) {
            if (plans[id][t] != plans[id][t - 1]) moves++;
            else waits++;
        }
        double heuristic_residual = 0.0;
        if (goals[id]) {
            Node* last = plans[id][horizon];
            heuristic_residual = manhattan_xy(last->x, last->y, goals[id]->x, goals[id]->y);
        }
        sum += alpha * moves + beta * waits + gamma * heuristic_residual;
    }
    return sum;
}

int detect_first_conflict(Node* const plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], int ids[], int count, CBSConflict* out, int horizon) {
    for (int t = 1; t <= horizon; ++t) {
        for (int i = 0; i < count; ++i) {
            for (int j = i + 1; j < count; ++j) {
                const int a = ids[i];
                const int b = ids[j];
                Node* a_t = plans[a][t];
                Node* b_t = plans[b][t];
                Node* a_tm1 = plans[a][t - 1];
                Node* b_tm1 = plans[b][t - 1];
                if (a_t == b_t) {
                    out->a = a;
                    out->b = b;
                    out->t = t;
                    out->is_edge = 0;
                    out->ax = a_t->x;
                    out->ay = a_t->y;
                    out->bx = b_t->x;
                    out->by = b_t->y;
                    out->apx = a_tm1->x;
                    out->apy = a_tm1->y;
                    out->bpx = b_tm1->x;
                    out->bpy = b_tm1->y;
                    return 1;
                }
                if (a_t == b_tm1 && b_t == a_tm1) {
                    out->a = a;
                    out->b = b;
                    out->t = t;
                    out->is_edge = 1;
                    out->ax = a_tm1->x;
                    out->ay = a_tm1->y;
                    out->bx = b_tm1->x;
                    out->by = b_tm1->y;
                    out->apx = a_tm1->x;
                    out->apy = a_tm1->y;
                    out->bpx = b_tm1->x;
                    out->bpy = b_tm1->y;
                    return 1;
                }
            }
        }
    }
    return 0;
}

void copy_ext_occ_without_group(const PlanningContext& context, const ReservationTable* base, int group_mask,
    int out_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]) {
    for (int t = 0; t <= context.whcaHorizon(); ++t) {
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                const int who = base->occ[t][y][x];
                out_occ[t][y][x] = (who != -1 && (group_mask & (1 << who))) ? -1 : who;
            }
        }
    }
}

void cbs_heap_push(CBSNode* heap, int* size, const CBSNode* node) {
    if (*size >= kMaxCbsNodes) return;
    heap[*size] = *node;
    int index = *size;
    (*size)++;
    while (index > 0) {
        const int parent = (index - 1) / 2;
        if (heap[parent].cost <= heap[index].cost) break;
        const CBSNode temp = heap[parent];
        heap[parent] = heap[index];
        heap[index] = temp;
        index = parent;
    }
}

CBSNode cbs_heap_pop(CBSNode* heap, int* size) {
    const CBSNode result = heap[0];
    *size = *size - 1;
    heap[0] = heap[*size];
    int index = 0;
    while (true) {
        const int left = 2 * index + 1;
        const int right = 2 * index + 2;
        int smallest = index;
        if (left < *size && heap[left].cost < heap[smallest].cost) smallest = left;
        if (right < *size && heap[right].cost < heap[smallest].cost) smallest = right;
        if (smallest == index) break;
        const CBSNode temp = heap[smallest];
        heap[smallest] = heap[index];
        heap[index] = temp;
        index = smallest;
    }
    return result;
}

int cbs_plan_agent_with_metrics(
    const PlanningContext& context,
    AgentManager* manager,
    GridMap* map,
    int agent_id,
    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
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
            constraints,
            constraint_count,
            out_plan,
            agent->heading,
            &nodes_expanded,
            &heap_moves,
            &generated_nodes,
            &valid_expansions)) {
        return 0;
    }

    accumulate_cbs_metrics(context, nodes_expanded, heap_moves, generated_nodes, valid_expansions);
    return 1;
}

int cbs_expansion_budget(int group_n) {
    const int scaled = kCbsBaseExpansions + std::max(0, group_n - 2) * 96;
    return std::min(kCbsMaxExpansionsCap, scaled);
}

}  // namespace

void ReservationTable_clear(ReservationTable* table) {
    for (int t = 0; t <= MAX_WHCA_HORIZON; ++t) {
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                table->occ[t][y][x] = -1;
            }
        }
    }
}

void ReservationTable_clearAgent(ReservationTable* table, int agent_id, int horizon) {
    if (!table) return;
    for (int t = 1; t <= horizon; ++t) {
        for (int y = 0; y < GRID_HEIGHT; ++y) {
            for (int x = 0; x < GRID_WIDTH; ++x) {
                if (table->occ[t][y][x] == agent_id) {
                    table->occ[t][y][x] = -1;
                }
            }
        }
    }
}

void ReservationTable_seedCurrent(ReservationTable* table, AgentManager* manager) {
    for (int i = 0; i < MAX_AGENTS; ++i) {
        Agent* agent = &manager->agents[i];
        if (agent->pos && agent->state != CHARGING) {
            table->occ[0][agent->pos->y][agent->pos->x] = agent->id;
        }
    }
}

int ReservationTable_isOccupied(const ReservationTable* table, int t, const Node* node, int horizon) {
    if (t < 0 || t > horizon) return true;
    return table->occ[t][node->y][node->x] != -1;
}

int ReservationTable_getOccupant(const ReservationTable* table, int t, const Node* node, int horizon) {
    if (t < 0 || t > horizon) return -1;
    return table->occ[t][node->y][node->x];
}

void ReservationTable_setOccupant(ReservationTable* table, int t, const Node* node, int agent_id, int horizon) {
    if (t < 0 || t > horizon) return;
    table->occ[t][node->y][node->x] = agent_id;
}

void WHCA_adjustHorizon(const PlanningContext& context, int wf_edges, int scc, Logger* logger) {
    RuntimeTuningState& tuning = *context.runtime_tuning;
    PlannerMetricsState& metrics = *context.planner_metrics;
    int conflict_score = static_cast<int>(tuning.conflict_score * 0.6) + wf_edges + (scc ? 5 : 0);
    int horizon = tuning.whca_horizon;
    const int old_horizon = horizon;
    constexpr int high_conflict = 24;
    constexpr int low_conflict = 10;

    if (conflict_score > high_conflict && horizon < MAX_WHCA_HORIZON) horizon += 2;
    else if (conflict_score < low_conflict && horizon > MIN_WHCA_HORIZON) horizon -= 2;

    if (horizon < MIN_WHCA_HORIZON) horizon = MIN_WHCA_HORIZON;
    if (horizon > MAX_WHCA_HORIZON) horizon = MAX_WHCA_HORIZON;

    if (old_horizon != horizon) {
        logger_log(logger, "[%sWHCA*%s] Horizon adjusted %d -> %d (score=%d)", "\x1b[1;36m", "\x1b[0m", old_horizon, horizon, conflict_score);
    }

    tuning.conflict_score = conflict_score;
    tuning.whca_horizon = horizon;
    metrics.whca_h = horizon;
}

void add_wait_edge(WaitEdge* edges, int* count, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2) {
    if (*count >= kPlannerMaxWaitEdges) return;
    edges[*count].from_id = from;
    edges[*count].to_id = to;
    edges[*count].t = t;
    edges[*count].cause = cause;
    edges[*count].x1 = x1;
    edges[*count].y1 = y1;
    edges[*count].x2 = x2;
    edges[*count].y2 = y2;
    (*count)++;
}

int build_scc_mask_from_edges(const WaitEdge* edges, int count) {
    int adjacency[MAX_AGENTS][MAX_AGENTS] = {{0}};
    for (int i = 0; i < count; ++i) {
        const int from = edges[i].from_id;
        const int to = edges[i].to_id;
        if (from >= 0 && to >= 0 && from != to) adjacency[from][to] = 1;
    }

    int reach[MAX_AGENTS][MAX_AGENTS] = {{0}};
    for (int i = 0; i < MAX_AGENTS; ++i) {
        for (int j = 0; j < MAX_AGENTS; ++j) {
            reach[i][j] = adjacency[i][j];
        }
    }

    for (int k = 0; k < MAX_AGENTS; ++k) {
        for (int i = 0; i < MAX_AGENTS; ++i) {
            for (int j = 0; j < MAX_AGENTS; ++j) {
                reach[i][j] = reach[i][j] || (reach[i][k] && reach[k][j]);
            }
        }
    }

    int mask = 0;
    for (int i = 0; i < MAX_AGENTS; ++i) {
        for (int j = 0; j < MAX_AGENTS; ++j) {
            if (i != j && reach[i][j] && reach[j][i]) {
                mask |= (1 << i);
                mask |= (1 << j);
            }
        }
    }
    return mask;
}

Node* try_pull_over(const PlanningContext& context, const ReservationTable* table, Agent* agent) {
    GridMap* map = context.map;
    if (Node* searched = try_pull_over_via_search(context, map, table, agent)) {
        return searched;
    }

    PullOverCandidate best{};
    for (int i = 1; i < 5; ++i) {
        const int next_x = agent->pos->x + kDir5X[i];
        const int next_y = agent->pos->y + kDir5Y[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        Node* candidate = &map->grid[next_y][next_x];
        if (candidate->is_obstacle || candidate->is_parked) continue;
        if (candidate->reserved_by_agent != -1 && candidate->reserved_by_agent != agent->id) continue;
        if (ReservationTable_isOccupied(table, 1, candidate, context.whcaHorizon())) continue;

        const PullOverCandidate current{
            candidate,
            candidate->is_goal ? 1 : 0,
            pull_over_neighbor_degree(map, candidate),
            i,
        };
        if (candidate_is_pull_over_better(current, best)) {
            best = current;
        }
    }

    if (best.node) return best.node;

    Node* current = agent->pos;
    if (current && !ReservationTable_isOccupied(table, 1, current, context.whcaHorizon())) return current;
    return agent->pos;
}

int run_partial_CBS(
    const PlanningContext& context,
    int group_ids[],
    int group_n,
    const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]) {
    if (group_n <= 1) return 0;

    AgentManager* manager = context.agents;
    GridMap* map = context.map;
    Logger* logger = context.logger;
    const int horizon = context.whcaHorizon();

    int group_mask = 0;
    for (int i = 0; i < group_n; ++i) group_mask |= (1 << group_ids[i]);

    static int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH];
    copy_ext_occ_without_group(context, base_rt, group_mask, ext_occ);

    static CBSNode heap[kMaxCbsNodes];
    int heap_size = 0;
    int expansions = 0;
    const int expansion_budget = cbs_expansion_budget(group_n);

    CBSNode root;
    std::memset(&root, 0, sizeof(root));
    for (int i = 0; i < group_n; ++i) {
        const int id = group_ids[i];
        Node* plan[MAX_WHCA_HORIZON + 1];
        if (!cbs_plan_agent_with_metrics(context, manager, map, id, ext_occ, root.cons, root.ncons, plan)) {
            record_cbs_failure(context, expansions);
            return 0;
        }
        for (int t = 0; t <= horizon; ++t) root.plans[id][t] = plan[t];
    }

    Node* goals[MAX_AGENTS] = {0};
    for (int i = 0; i < group_n; ++i) goals[group_ids[i]] = manager->agents[group_ids[i]].goal;
    root.cost = cbs_cost_sum_adv(group_ids, group_n, root.plans, goals, horizon);
    cbs_heap_push(heap, &heap_size, &root);

    while (heap_size > 0 && expansions < expansion_budget) {
        const CBSNode current = cbs_heap_pop(heap, &heap_size);
        expansions++;
        if (expansions > expansion_budget) break;

        CBSConflict conflict;
        if (!detect_first_conflict(current.plans, group_ids, group_n, &conflict, horizon)) {
            for (int i = 0; i < group_n; ++i) {
                const int id = group_ids[i];
                for (int t = 0; t <= horizon; ++t) out_plans[id][t] = current.plans[id][t];
            }
            logger_log(logger, "[%sCBS%s] Partial CBS succeeded (group=%d agents, expansions=%d).", "\x1b[1;32m", "\x1b[0m", group_n, expansions);
            record_cbs_success(context, expansions);
            return 1;
        }

        for (int branch = 0; branch < 2; ++branch) {
            if (heap_size >= kMaxCbsNodes) break;
            CBSNode child = current;
            if (child.ncons >= MAX_CBS_CONS) continue;

            CBSConstraint constraint;
            std::memset(&constraint, 0, sizeof(constraint));
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

            int ok = 1;
            for (int i = 0; i < group_n; ++i) {
                const int id = group_ids[i];
                Node* plan[MAX_WHCA_HORIZON + 1];
                if (!cbs_plan_agent_with_metrics(context, manager, map, id, ext_occ, child.cons, child.ncons, plan)) {
                    ok = 0;
                    break;
                }
                for (int t = 0; t <= horizon; ++t) child.plans[id][t] = plan[t];
            }
            if (!ok) continue;

            for (int i = 0; i < group_n; ++i) goals[group_ids[i]] = manager->agents[group_ids[i]].goal;
            child.cost = cbs_cost_sum_adv(group_ids, group_n, child.plans, goals, horizon);
            cbs_heap_push(heap, &heap_size, &child);
        }
    }

    logger_log(logger, "[%sCBS%s] Partial CBS failed within the search budget (%d). Falling back to pull-over.", "\x1b[1;31m", "\x1b[0m", expansion_budget);
    record_cbs_failure(context, expansions);
    return 0;
}
