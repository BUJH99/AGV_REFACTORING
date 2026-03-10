#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <cstring>

#include "agv/internal/engine_internal.hpp"

#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#define C_B_CYN "\x1b[1;36m"
#endif

int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const struct Agent_* agent);
Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
void pathfinder_update_start(Pathfinder* pf, Node* new_start);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current_node);
void logger_log(Logger* logger, const char* format, ...);
void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg);

namespace {

constexpr int kMaxWaitEdges = 128;
constexpr int kMaxCbsGroup = 8;
constexpr int kMaxCbsNodes = 256;
constexpr int kCbsMaxExpansions = 128;
constexpr int kMaxTot = ((MAX_WHCA_HORIZON) + 1) * GRID_WIDTH * GRID_HEIGHT;
constexpr double kCollisionInf = 1e18;
constexpr int kPriorityReturningWithCar = 3;
constexpr int kPriorityGoingToCharge = 2;
constexpr int kPriorityMovingTask = 1;
constexpr int kDeadlockThreshold = 5;
constexpr int kStuckBoostMult = 10;
constexpr int kStuckBoostHard = 1000;
constexpr int kTurn90Wait = 2;
constexpr int kDir4X[4] = {0, 0, 1, -1};
constexpr int kDir4Y[4] = {1, -1, 0, 0};
constexpr int kDir5X[5] = {0, 1, -1, 0, 0};
constexpr int kDir5Y[5] = {0, 0, 0, 1, -1};

double g_buffer[kMaxTot];
double f_buffer[kMaxTot];
unsigned char open_buffer[kMaxTot];
unsigned char closed_buffer[kMaxTot];
int prev_buffer[kMaxTot];
int heap_node_buffer[kMaxTot];
int heap_pos_buffer[kMaxTot];

AgentDir dir_from_delta_local(int dx, int dy) {
    if (dx == 1 && dy == 0) return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == -1) return DIR_UP;
    if (dx == 0 && dy == 1) return DIR_DOWN;
    return DIR_NONE;
}

int dir_turn_steps_local(AgentDir from, AgentDir to) {
    if (from == DIR_NONE || to == DIR_NONE) return 0;
    int diff = ((int)to - (int)from + 4) % 4;
    return diff <= 2 ? diff : 4 - diff;
}

double manhattan_xy_local(int x1, int y1, int x2, int y2) {
    return std::fabs((double)x1 - (double)x2) + std::fabs((double)y1 - (double)y2);
}

int heap_prefer(double* fvals, int a, int b) {
    double fa = fvals[a];
    double fb = fvals[b];
    if (fa < fb - 1e-9) return 1;
    if (fa > fb + 1e-9) return 0;
    return a < b;
}

void heap_swap(int* nodes, int* pos, int i, int j, unsigned long long* swap_counter) {
    if (i == j) return;
    int node_i = nodes[i];
    int node_j = nodes[j];
    nodes[i] = node_j;
    nodes[j] = node_i;
    pos[node_j] = i;
    pos[node_i] = j;
    if (swap_counter) (*swap_counter)++;
}

void heap_sift_up(int* nodes, int* pos, double* fvals, int idx, unsigned long long* swap_counter) {
    while (idx > 0) {
        int parent = (idx - 1) >> 1;
        if (!heap_prefer(fvals, nodes[idx], nodes[parent])) break;
        heap_swap(nodes, pos, idx, parent, swap_counter);
        idx = parent;
    }
}

void heap_sift_down(int* nodes, int* pos, double* fvals, int size, int idx, unsigned long long* swap_counter) {
    while (1) {
        int left = (idx << 1) + 1;
        int right = left + 1;
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
    int root = nodes[0];
    (*size)--;
    if (*size > 0) {
        int last = nodes[*size];
        nodes[0] = last;
        pos[last] = 0;
        heap_sift_down(nodes, pos, fvals, *size, 0, swap_counter);
    }
    pos[root] = -1;
    return root;
}

void heap_decrease_key(int* nodes, int* pos, double* fvals, int node, unsigned long long* swap_counter) {
    int idx = pos[node];
    if (idx >= 0) heap_sift_up(nodes, pos, fvals, idx, swap_counter);
}

int violates_constraint_for(int agent, int t_prev, int x_prev, int y_prev, int x_new, int y_new,
    const CBSConstraint* cons, int ncons) {
    for (int i = 0; i < ncons; i++) {
        if (cons[i].agent != agent) continue;
        if (cons[i].is_edge) {
            if (cons[i].t == t_prev && cons[i].x == x_prev && cons[i].y == y_prev &&
                cons[i].tox == x_new && cons[i].toy == y_new) return 1;
        } else {
            if (cons[i].t == (t_prev + 1) && cons[i].x == x_new && cons[i].y == y_new) return 1;
        }
    }
    return 0;
}

}  // namespace

void ReservationTable_clear(ReservationTable* table) {
    for (int t = 0; t <= MAX_WHCA_HORIZON; t++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            for (int x = 0; x < GRID_WIDTH; x++) {
                table->occ[t][y][x] = -1;
            }
        }
    }
}

void ReservationTable_seedCurrent(ReservationTable* table, AgentManager* manager) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        if (agent->pos && agent->state != CHARGING) table->occ[0][agent->pos->y][agent->pos->x] = agent->id;
    }
}

int ReservationTable_isOccupied(const ReservationTable* table, int t, const Node* node) {
    if (t < 0 || t > agv_current_whca_horizon()) return TRUE;
    return table->occ[t][node->y][node->x] != -1;
}

int ReservationTable_getOccupant(const ReservationTable* table, int t, const Node* node) {
    if (t < 0 || t > agv_current_whca_horizon()) return -1;
    return table->occ[t][node->y][node->x];
}

void ReservationTable_setOccupant(ReservationTable* table, int t, const Node* node, int agent_id) {
    if (t < 0 || t > agv_current_whca_horizon()) return;
    table->occ[t][node->y][node->x] = agent_id;
}

void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg) {
    int conflict_score = (int)(agv_current_conflict_score() * 0.6) + wf_edges + (scc ? 5 : 0);
    int horizon = agv_current_whca_horizon();
    const int old_horizon = horizon;
    const int hi = 24;
    const int lo = 10;

    if (conflict_score > hi && horizon < MAX_WHCA_HORIZON) horizon += 2;
    else if (conflict_score < lo && horizon > MIN_WHCA_HORIZON) horizon -= 2;

    if (horizon < MIN_WHCA_HORIZON) horizon = MIN_WHCA_HORIZON;
    if (horizon > MAX_WHCA_HORIZON) horizon = MAX_WHCA_HORIZON;

    if (old_horizon != horizon) {
        logger_log(lg, "[%sWHCA*%s] Horizon adjusted %d -> %d (score=%d)", C_B_CYN, C_NRM, old_horizon, horizon, conflict_score);
    }

    agv_set_whca_runtime_state(conflict_score, horizon);
}

void add_wait_edge(WaitEdge* edges, int* count, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2) {
    if (*count >= kMaxWaitEdges) return;
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
    int adjacency[MAX_AGENTS][MAX_AGENTS] = {0};
    for (int i = 0; i < count; i++) {
        int from = edges[i].from_id;
        int to = edges[i].to_id;
        if (from >= 0 && to >= 0 && from != to) adjacency[from][to] = 1;
    }

    int reach[MAX_AGENTS][MAX_AGENTS] = {0};
    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = 0; j < MAX_AGENTS; j++) reach[i][j] = adjacency[i][j];
    }

    for (int k = 0; k < MAX_AGENTS; k++) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            for (int j = 0; j < MAX_AGENTS; j++) {
                reach[i][j] = reach[i][j] || (reach[i][k] && reach[k][j]);
            }
        }
    }

    int mask = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = 0; j < MAX_AGENTS; j++) {
            if (i != j && reach[i][j] && reach[j][i]) {
                mask |= (1 << i);
                mask |= (1 << j);
            }
        }
    }
    return mask;
}

Node* try_pull_over(const GridMap* map, const ReservationTable* table, Agent* agent) {
    for (int i = 0; i < 5; i++) {
        int next_x = agent->pos->x + kDir5X[i];
        int next_y = agent->pos->y + kDir5Y[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        Node* candidate = &const_cast<GridMap*>(map)->grid[next_y][next_x];
        if (candidate == agent->pos) {
            if (!ReservationTable_isOccupied(table, 1, candidate)) return candidate;
            continue;
        }
        if (candidate->is_obstacle || candidate->is_parked) continue;
        if (candidate->reserved_by_agent != -1 && candidate->reserved_by_agent != agent->id) continue;
        if (!ReservationTable_isOccupied(table, 1, candidate)) return candidate;
    }
    return agent->pos;
}

int st_astar_plan_single(int agent_id, GridMap* map, Node* start, Node* goal, int horizon,
    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    const CBSConstraint* cons, int ncons,
    Node* out_plan[MAX_WHCA_HORIZON + 1],
    AgentDir initial_heading,
    unsigned long long* out_nodes_expanded,
    unsigned long long* out_heap_moves,
    unsigned long long* out_generated_nodes,
    unsigned long long* out_valid_expansions) {
    if (!start) return 0;
    int t_limit = horizon;
    int width = GRID_WIDTH;
    int height = GRID_HEIGHT;
    int total = (t_limit + 1) * width * height;
    if (out_nodes_expanded) *out_nodes_expanded = 0;
    if (out_heap_moves) *out_heap_moves = 0;
    if (out_generated_nodes) *out_generated_nodes = 0;
    if (out_valid_expansions) *out_valid_expansions = 0;
    if (total > kMaxTot) return 0;

    double* g = g_buffer;
    double* f = f_buffer;
    unsigned char* open = open_buffer;
    unsigned char* closed = closed_buffer;
    int* prev = prev_buffer;
    int* heap_nodes = heap_node_buffer;
    int* heap_pos = heap_pos_buffer;
    int heap_size = 0;

    for (int i = 0; i < total; i++) {
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

    int start_x = start->x;
    int start_y = start->y;
    int goal_x = goal ? goal->x : start_x;
    int goal_y = goal ? goal->y : start_y;

    int start_idx = ST_INDEX(0, start_y, start_x, width, height);
    g[start_idx] = 0.0;
    f[start_idx] = goal ? manhattan_xy_local(start_x, start_y, goal_x, goal_y) : 0.0;
    open[start_idx] = 1;

    int best_idx = start_idx;
    double best_val = f[start_idx];
    unsigned long long nodes_expanded = 0;
    unsigned long long heap_moves = 0;
    unsigned long long generated_nodes = 0;
    unsigned long long valid_expansions = 0;

    heap_push(heap_nodes, heap_pos, f, &heap_size, start_idx, &heap_moves);

    while (heap_size > 0) {
        int cur = heap_pop(heap_nodes, heap_pos, f, &heap_size, &heap_moves);
        open[cur] = 0;
        closed[cur] = 1;
        nodes_expanded++;

        int cur_t = cur / (width * height);
        int rem = cur % (width * height);
        int cur_y = rem / width;
        int cur_x = rem % width;

        if (goal && cur_x == goal_x && cur_y == goal_y) {
            best_idx = cur;
            break;
        }
        if (f[cur] < best_val) {
            best_val = f[cur];
            best_idx = cur;
        }

        if (cur_t == t_limit) continue;

        for (int k = 0; k < 5; k++) {
            int next_x = cur_x + kDir5X[k];
            int next_y = cur_y + kDir5Y[k];
            int next_t = cur_t + 1;
            if (!grid_is_valid_coord(next_x, next_y)) continue;
            if (ext_occ[next_t][next_y][next_x] != -1) continue;
            if (ext_occ[cur_t][next_y][next_x] != -1 && ext_occ[next_t][cur_y][cur_x] == ext_occ[cur_t][next_y][next_x]) continue;

            Node* next_cell = &map->grid[next_y][next_x];
            if (next_cell->is_obstacle) continue;
            if (violates_constraint_for(agent_id, cur_t, cur_x, cur_y, next_x, next_y, cons, ncons)) continue;

            int next_idx = ST_INDEX(next_t, next_y, next_x, width, height);
            if (closed[next_idx]) continue;
            generated_nodes++;

            double next_g = g[cur] + 1.0;
            if (cur_t == 0 && !(next_x == cur_x && next_y == cur_y)) {
                AgentDir move_heading = dir_from_delta_local(next_x - cur_x, next_y - cur_y);
                if (initial_heading != DIR_NONE) {
                    int turn_steps = dir_turn_steps_local(initial_heading, move_heading);
                    if (turn_steps == 1) next_g += (double)kTurn90Wait;
                }
            }

            if (next_g + 1e-9 < g[next_idx]) {
                g[next_idx] = next_g;
                double h = goal ? manhattan_xy_local(next_x, next_y, goal_x, goal_y) : 0.0;
                f[next_idx] = next_g + h;
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
    int cur = best_idx;
    while (cur != -1 && plan_length < (MAX_WHCA_HORIZON + 1)) {
        path_idx[plan_length++] = cur;
        cur = prev[cur];
    }

    if (out_nodes_expanded) *out_nodes_expanded = nodes_expanded;
    if (out_heap_moves) *out_heap_moves = heap_moves;
    if (out_generated_nodes) *out_generated_nodes = generated_nodes;
    if (out_valid_expansions) *out_valid_expansions = valid_expansions;

    if (plan_length == 0) {
        for (int t = 0; t <= t_limit; t++) out_plan[t] = start;
        return 1;
    }

    for (int t = 0; t < plan_length; t++) {
        int idx = path_idx[plan_length - 1 - t];
        int tt = idx / (width * height);
        int rem = idx % (width * height);
        int y = rem / width;
        int x = rem % width;
        if (tt <= t_limit) out_plan[tt] = &map->grid[y][x];
    }

    Node* last = out_plan[plan_length - 1 <= t_limit ? plan_length - 1 : t_limit];
    for (int t = plan_length; t <= t_limit; t++) out_plan[t] = last;
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

static double cbs_cost_sum_adv(int ids[], int n,
    Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1],
    Node* goals[MAX_AGENTS], int horizon) {
    const double alpha = 1.0;
    const double beta = 0.5;
    const double gamma = 0.1;
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        int id = ids[i];
        int moves = 0;
        int waits = 0;
        for (int t = 1; t <= horizon; t++) {
            if (plans[id][t] != plans[id][t - 1]) moves++;
            else waits++;
        }
        double heuristic_residual = 0.0;
        if (goals[id]) {
            Node* last = plans[id][horizon];
            heuristic_residual = manhattan_xy_local(last->x, last->y, goals[id]->x, goals[id]->y);
        }
        sum += alpha * moves + beta * waits + gamma * heuristic_residual;
    }
    return sum;
}

static int detect_first_conflict(Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], int ids[], int n, CBSConflict* out, int horizon) {
    for (int t = 1; t <= horizon; t++) {
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                int a = ids[i];
                int b = ids[j];
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

static void copy_ext_occ_without_group(const ReservationTable* base, int group_mask,
    int out_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]) {
    for (int t = 0; t <= agv_current_whca_horizon(); t++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            for (int x = 0; x < GRID_WIDTH; x++) {
                int who = base->occ[t][y][x];
                if (who != -1 && (group_mask & (1 << who))) out_occ[t][y][x] = -1;
                else out_occ[t][y][x] = who;
            }
        }
    }
}

static void cbs_heap_push(CBSNode* heap, int* hsize, const CBSNode* node) {
    if (*hsize >= kMaxCbsNodes) return;
    heap[*hsize] = *node;
    int i = *hsize;
    (*hsize)++;
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap[parent].cost <= heap[i].cost) break;
        CBSNode temp = heap[parent];
        heap[parent] = heap[i];
        heap[i] = temp;
        i = parent;
    }
}

static CBSNode cbs_heap_pop(CBSNode* heap, int* hsize) {
    CBSNode ret = heap[0];
    *hsize = *hsize - 1;
    heap[0] = heap[*hsize];
    int i = 0;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = i;
        if (left < *hsize && heap[left].cost < heap[smallest].cost) smallest = left;
        if (right < *hsize && heap[right].cost < heap[smallest].cost) smallest = right;
        if (smallest == i) break;
        CBSNode temp = heap[smallest];
        heap[smallest] = heap[i];
        heap[i] = temp;
        i = smallest;
    }
    return ret;
}

static int cbs_plan_agent_with_metrics(
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
        agv_current_whca_horizon(),
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

    agv_accumulate_cbs_step_metrics(nodes_expanded, heap_moves, generated_nodes, valid_expansions);
    return 1;
}

int run_partial_CBS(AgentManager* manager, GridMap* map, Logger* logger,
    int group_ids[], int group_n, const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]) {
    if (group_n <= 1) return 0;

    int group_mask = 0;
    for (int i = 0; i < group_n; i++) group_mask |= (1 << group_ids[i]);

    static int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH];
    copy_ext_occ_without_group(base_rt, group_mask, ext_occ);

    static CBSNode heap[kMaxCbsNodes];
    int hsize = 0;
    int expansions = 0;

    CBSNode root;
    memset(&root, 0, sizeof(root));
    for (int i = 0; i < group_n; i++) {
        int id = group_ids[i];
        Node* plan[MAX_WHCA_HORIZON + 1];
        if (!cbs_plan_agent_with_metrics(manager, map, id, ext_occ, root.cons, root.ncons, plan)) {
            agv_record_cbs_failure(expansions);
            return 0;
        }
        for (int t = 0; t <= agv_current_whca_horizon(); t++) root.plans[id][t] = plan[t];
    }

    Node* goals[MAX_AGENTS] = {0};
    for (int i = 0; i < group_n; i++) goals[group_ids[i]] = manager->agents[group_ids[i]].goal;
    root.cost = cbs_cost_sum_adv(group_ids, group_n, root.plans, goals, agv_current_whca_horizon());
    cbs_heap_push(heap, &hsize, &root);

    while (hsize > 0 && expansions < kCbsMaxExpansions) {
        CBSNode cur = cbs_heap_pop(heap, &hsize);
        expansions++;
        if (expansions > kCbsMaxExpansions) break;

        CBSConflict conflict;
        if (!detect_first_conflict(cur.plans, group_ids, group_n, &conflict, agv_current_whca_horizon())) {
            for (int i = 0; i < group_n; i++) {
                int id = group_ids[i];
                for (int t = 0; t <= agv_current_whca_horizon(); t++) out_plans[id][t] = cur.plans[id][t];
            }
            logger_log(logger, "[%sCBS%s] Partial CBS succeeded (group=%d agents, expansions=%d).", C_B_GRN, C_NRM, group_n, expansions);
            agv_record_cbs_success(expansions);
            return 1;
        }

        for (int branch = 0; branch < 2; branch++) {
            if (hsize >= kMaxCbsNodes) break;
            CBSNode child = cur;
            if (child.ncons >= MAX_CBS_CONS) continue;

            CBSConstraint c;
            memset(&c, 0, sizeof(c));
            if (branch == 0) c.agent = conflict.a;
            else c.agent = conflict.b;
            if (conflict.is_edge) {
                c.is_edge = 1;
                c.t = conflict.t - 1;
                if (branch == 0) {
                    c.x = conflict.apx;
                    c.y = conflict.apy;
                    c.tox = conflict.bpx;
                    c.toy = conflict.bpy;
                } else {
                    c.x = conflict.bpx;
                    c.y = conflict.bpy;
                    c.tox = conflict.apx;
                    c.toy = conflict.apy;
                }
            } else {
                c.is_edge = 0;
                c.t = conflict.t;
                c.x = conflict.ax;
                c.y = conflict.ay;
            }
            child.cons[child.ncons++] = c;

            int ok = 1;
            for (int i = 0; i < group_n; i++) {
                int id = group_ids[i];
                Node* plan[MAX_WHCA_HORIZON + 1];
                if (!cbs_plan_agent_with_metrics(manager, map, id, ext_occ, child.cons, child.ncons, plan)) {
                    ok = 0;
                    break;
                }
                for (int t = 0; t <= agv_current_whca_horizon(); t++) child.plans[id][t] = plan[t];
            }
            if (!ok) continue;

            for (int i = 0; i < group_n; i++) goals[group_ids[i]] = manager->agents[group_ids[i]].goal;
            child.cost = cbs_cost_sum_adv(group_ids, group_n, child.plans, goals, agv_current_whca_horizon());
            cbs_heap_push(heap, &hsize, &child);
        }
    }

    logger_log(logger, "[%sCBS%s] Partial CBS failed within the search budget. Falling back to pull-over.", C_B_RED, C_NRM);
    agv_record_cbs_failure(expansions);
    return 0;
}

int priority_score(const Agent* agent) {
    int importance = 0;
    if (agent->state == RETURNING_WITH_CAR) importance = kPriorityReturningWithCar;
    else if (agent->state == GOING_TO_CHARGE) importance = kPriorityGoingToCharge;
    else if (agent->state == GOING_TO_PARK || agent->state == GOING_TO_COLLECT) importance = kPriorityMovingTask;

    int stuck_boost = (agent->stuck_steps >= kDeadlockThreshold) ? kStuckBoostHard : (agent->stuck_steps * kStuckBoostMult);
    return importance * 100 + stuck_boost - agent->id;
}

void agent_apply_rotation_and_step_local(Agent* ag, Node* current, Node* desired, Node** out_next) {
    if (!ag || !current || !out_next) return;
    *out_next = current;
    if (!desired || desired == current) return;

    int dx = desired->x - current->x;
    int dy = desired->y - current->y;
    AgentDir new_heading = dir_from_delta_local(dx, dy);
    if (new_heading == DIR_NONE) return;

    if (ag->heading == DIR_NONE) {
        ag->heading = new_heading;
        *out_next = desired;
        return;
    }

    int turn_steps = dir_turn_steps_local(ag->heading, new_heading);
    if (turn_steps == 1) {
        ag->rotation_wait = kTurn90Wait - 1;
        ag->heading = new_heading;
        ag->metrics_turns_current++;
        return;
    }
    ag->heading = new_heading;
    *out_next = desired;
}

void ensure_pathfinder_for_agent(Agent* ag) {
    if (!ag->goal) return;
    if (ag->pf == nullptr) {
        ag->pf = pathfinder_create(ag->pos, ag->goal, ag);
    } else if (ag->pf->goal_node != ag->goal) {
        ag->pf->start_node = ag->pos;
        pathfinder_reset_goal(ag->pf, ag->goal);
    }
}

void sort_agents_by_priority(AgentManager* manager, int order[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) order[i] = i;
    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = i + 1; j < MAX_AGENTS; j++) {
            if (priority_score(&manager->agents[order[j]]) > priority_score(&manager->agents[order[i]])) {
                int temp = order[i];
                order[i] = order[j];
                order[j] = temp;
            }
        }
    }
}

int best_candidate_order(Pathfinder* pf, const GridMap* map, const AgentManager* am,
    Node* cur, Node* goal, Node* out[5], int* outN) {
    struct Candidate {
        Node* node;
        double cost;
        double distance;
    };

    Candidate candidates[5];
    int candidate_count = 0;
    double current_g = pf->cells[cur->y][cur->x].g;
    candidates[candidate_count++] = Candidate{cur, current_g + 1e-6, 1e18};

    for (int i = 0; i < 4; i++) {
        int next_x = cur->x + kDir4X[i];
        int next_y = cur->y + kDir4Y[i];
        if (!grid_is_valid_coord(next_x, next_y)) continue;
        Node* next = &const_cast<GridMap*>(map)->grid[next_y][next_x];
        if (grid_is_node_blocked(map, am, next, pf->agent)) continue;
        double successor_g = pf->cells[next->y][next->x].g;
        double cost = 1.0 + successor_g;
        double distance = manhattan_xy_local(next->x, next->y, goal->x, goal->y);
        candidates[candidate_count++] = Candidate{next, cost, distance};
    }

    for (int a = 0; a < candidate_count; a++) {
        for (int b = a + 1; b < candidate_count; b++) {
            if (candidates[b].cost < candidates[a].cost ||
                (std::fabs(candidates[b].cost - candidates[a].cost) < 1e-9 && candidates[b].distance < candidates[a].distance)) {
                Candidate temp = candidates[a];
                candidates[a] = candidates[b];
                candidates[b] = temp;
            }
        }
    }

    for (int i = 0; i < candidate_count; i++) out[i] = candidates[i].node;
    *outN = candidate_count;
    return candidate_count;
}

int best_in_mask(const AgentManager* manager, int mask) {
    int best = -1;
    int best_score = -999999;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (mask & (1 << i)) {
            int score = priority_score(&manager->agents[i]);
            if (score > best_score) {
                best_score = score;
                best = i;
            }
        }
    }
    return best;
}

void temp_mark_init_local(TempMarkList* list) {
    if (list) list->count = 0;
}

void temp_mark_node_local(TempMarkList* list, Node* node) {
    if (!list || !node) return;
    if (!node->is_temp) {
        node->is_temp = TRUE;
        if (list->count < TEMP_MARK_MAX) list->nodes[list->count++] = node;
    }
}

void temp_unmark_all_local(TempMarkList* list) {
    if (!list) return;
    for (int i = 0; i < list->count; i++) {
        if (list->nodes[i]) list->nodes[i]->is_temp = FALSE;
    }
    list->count = 0;
}

void temp_unmark_all_and_notify_local(TempMarkList* list, Pathfinder* pf, GridMap* map, const AgentManager* am) {
    if (!list) return;
    for (int i = 0; i < list->count; i++) {
        Node* node = list->nodes[i];
        if (!node) continue;
        node->is_temp = FALSE;
        if (pf) pathfinder_notify_cell_change(pf, map, am, node);
    }
    list->count = 0;
}

struct TempMarkContextLocal final {
    TempMarkList marks{};
    Pathfinder* pf{nullptr};
    GridMap* map{nullptr};
    AgentManager* am{nullptr};
    int auto_notify{FALSE};
};

void temp_context_init_local(TempMarkContextLocal* ctx, Pathfinder* pf, GridMap* map, AgentManager* am, int auto_notify) {
    if (!ctx) return;
    temp_mark_init_local(&ctx->marks);
    ctx->pf = pf;
    ctx->map = map;
    ctx->am = am;
    ctx->auto_notify = auto_notify;
}

void temp_context_mark_local(TempMarkContextLocal* ctx, Node* node) {
    if (!ctx || !node) return;
    temp_mark_node_local(&ctx->marks, node);
    if (ctx->auto_notify && ctx->pf) pathfinder_notify_cell_change(ctx->pf, ctx->map, ctx->am, node);
}

void temp_context_cleanup_local(TempMarkContextLocal* ctx) {
    if (!ctx) return;
    if (ctx->auto_notify) temp_unmark_all_and_notify_local(&ctx->marks, ctx->pf, ctx->map, ctx->am);
    else temp_unmark_all_local(&ctx->marks);
}

void temp_context_mark_order_blockers_local(
    TempMarkContextLocal* ctx,
    const AgentManager* manager,
    const int order[MAX_AGENTS],
    int order_index,
    Node* next_pos[MAX_AGENTS]) {
    if (!ctx || !manager || !order) return;
    for (int h = 0; h < order_index; h++) {
        int higher_id = order[h];
        if (next_pos[higher_id]) temp_context_mark_local(ctx, next_pos[higher_id]);
    }
    for (int l = order_index + 1; l < MAX_AGENTS; l++) {
        int lower_id = order[l];
        if (manager->agents[lower_id].pos) temp_context_mark_local(ctx, manager->agents[lower_id].pos);
    }
}

int temporarily_unpark_goal_local(Agent* agent, Pathfinder* pf, GridMap* map, const AgentManager* manager) {
    int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
    if (goal_was_parked) {
        agent->goal->is_parked = FALSE;
        if (pf) pathfinder_notify_cell_change(pf, map, manager, agent->goal);
    }
    return goal_was_parked;
}

void restore_temporarily_unparked_goal_local(
    Agent* agent,
    Pathfinder* pf,
    GridMap* map,
    const AgentManager* manager,
    int goal_was_parked) {
    if (!goal_was_parked) return;
    agent->goal->is_parked = TRUE;
    if (pf) pathfinder_notify_cell_change(pf, map, manager, agent->goal);
}

enum OrderedPlanningMetricLocal {
    ORDERED_PLANNING_ASTAR,
    ORDERED_PLANNING_DSTAR,
};

Node* compute_ordered_pathfinder_move_local(
    Agent* agent,
    GridMap* map,
    AgentManager* manager,
    OrderedPlanningMetricLocal metric_kind) {
    if (!agent || !agent->pf) return agent ? agent->pos : nullptr;

    pathfinder_update_start(agent->pf, agent->pos);
    pathfinder_compute_shortest_path(agent->pf, map, manager);

    if (metric_kind == ORDERED_PLANNING_ASTAR) {
        agv_accumulate_astar_step_metrics(
            agent->pf->nodes_expanded_this_call,
            agent->pf->heap_moves_this_call,
            agent->pf->nodes_generated_this_call,
            agent->pf->valid_expansions_this_call);
    } else {
        agv_accumulate_dstar_step_metrics(
            agent->pf->nodes_expanded_this_call,
            agent->pf->heap_moves_this_call,
            agent->pf->nodes_generated_this_call,
            agent->pf->valid_expansions_this_call);
    }

    return pathfinder_get_next_step(agent->pf, map, manager, agent->pos);
}

int node_flat_index_local(const Node* node) {
    return node ? (node->y * GRID_WIDTH + node->x) : -1;
}

void resolve_conflicts_by_order_local(const AgentManager* manager, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]) {
    int cell_owner[GRID_WIDTH * GRID_HEIGHT];
    for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; i++) cell_owner[i] = -1;

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        if (!next_pos[i]) continue;
        int next_idx = node_flat_index_local(next_pos[i]);
        if (next_idx < 0) continue;
        if (cell_owner[next_idx] != -1) {
            next_pos[i] = const_cast<AgentManager*>(manager)->agents[i].pos;
            continue;
        }
        cell_owner[next_idx] = i;
    }

    for (int i = 0; i < GRID_WIDTH * GRID_HEIGHT; i++) cell_owner[i] = -1;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!manager->agents[i].pos) continue;
        int current_idx = node_flat_index_local(manager->agents[i].pos);
        if (current_idx >= 0) cell_owner[current_idx] = i;
    }

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        if (!next_pos[i]) continue;
        int dest_idx = node_flat_index_local(next_pos[i]);
        int other = (dest_idx >= 0) ? cell_owner[dest_idx] : -1;
        if (other == -1 || other == i || !next_pos[other]) continue;
        if (next_pos[other] == manager->agents[i].pos) {
            next_pos[other] = const_cast<AgentManager*>(manager)->agents[other].pos;
        } else if (next_pos[other] == manager->agents[other].pos && next_pos[i] == manager->agents[other].pos) {
            next_pos[i] = const_cast<AgentManager*>(manager)->agents[i].pos;
        }
    }
}

void agent_manager_plan_and_resolve_collisions_core(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) next_pos[i] = m->agents[i].pos;

    int order[MAX_AGENTS];
    sort_agents_by_priority(m, order);

    ReservationTable rt;
    ReservationTable_clear(&rt);
    ReservationTable_seedCurrent(&rt, m);
    WaitEdge wf_edges[128];
    int wf_cnt = 0;

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        Agent* ag = &m->agents[i];
        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == nullptr) continue;

        if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) {
            for (int kk = 1; kk <= agv_current_whca_horizon(); kk++) {
                ReservationTable_setOccupant(&rt, kk, ag->pos, ag->id);
            }
            next_pos[ag->id] = ag->pos;
            continue;
        }

        ensure_pathfinder_for_agent(ag);

        int goal_was_parked = (ag->state == GOING_TO_COLLECT && ag->goal->is_parked);
        if (goal_was_parked) ag->goal->is_parked = FALSE;

        if (ag->pf) {
            pathfinder_update_start(ag->pf, ag->pos);
            pathfinder_compute_shortest_path(ag->pf, map, m);

            agv_accumulate_whca_dstar_step_metrics(
                ag->pf->nodes_expanded_this_call,
                ag->pf->heap_moves_this_call,
                ag->pf->nodes_generated_this_call,
                ag->pf->valid_expansions_this_call);
        }

        Node* plan[MAX_WHCA_HORIZON + 1];
        plan[0] = ag->pos;
        Node* cur = ag->pos;

        for (int k = 1; k <= agv_current_whca_horizon(); k++) {
            Node* cand[5];
            int cn = 0;
            best_candidate_order(ag->pf, map, m, cur, ag->pf->goal_node, cand, &cn);

            Node* chosen = cur;
            for (int ci = 0; ci < cn; ci++) {
                Node* nb = cand[ci];
                if (ReservationTable_isOccupied(&rt, k, nb)) {
                    int who = ReservationTable_getOccupant(&rt, k, nb);
                    if (who != -1) add_wait_edge(wf_edges, &wf_cnt, ag->id, who, k, CAUSE_VERTEX, nb->x, nb->y, 0, 0);
                    continue;
                }

                int who_prev = ReservationTable_getOccupant(&rt, k - 1, nb);
                int who_into_cur = ReservationTable_getOccupant(&rt, k, cur);
                if (who_prev != -1 && who_prev == who_into_cur) {
                    add_wait_edge(wf_edges, &wf_cnt, ag->id, who_prev, k, CAUSE_SWAP, cur->x, cur->y, nb->x, nb->y);
                    continue;
                }

                if (chosen == cur) chosen = nb;
            }

            plan[k] = chosen;
            ReservationTable_setOccupant(&rt, k, chosen, ag->id);
            cur = chosen;

            if (cur == ag->goal) {
                for (int kk = k + 1; kk <= agv_current_whca_horizon(); kk++) {
                    ReservationTable_setOccupant(&rt, kk, cur, ag->id);
                    plan[kk] = cur;
                }
                break;
            }
        }
        next_pos[ag->id] = plan[1];
        if (goal_was_parked) ag->goal->is_parked = TRUE;
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == nullptr) continue;
        for (int j = i + 1; j < MAX_AGENTS; j++) {
            if (m->agents[j].state == IDLE || m->agents[j].state == CHARGING || m->agents[j].goal == nullptr) continue;
            if (!next_pos[i] || !next_pos[j]) continue;

            if (next_pos[i] == next_pos[j]) {
                add_wait_edge(wf_edges, &wf_cnt, i, j, 1, CAUSE_VERTEX, next_pos[i]->x, next_pos[i]->y, 0, 0);
                add_wait_edge(wf_edges, &wf_cnt, j, i, 1, CAUSE_VERTEX, next_pos[j]->x, next_pos[j]->y, 0, 0);
            } else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {
                add_wait_edge(wf_edges, &wf_cnt, i, j, 1, CAUSE_SWAP,
                    m->agents[i].pos ? m->agents[i].pos->x : -1,
                    m->agents[i].pos ? m->agents[i].pos->y : -1,
                    next_pos[i]->x, next_pos[i]->y);
                add_wait_edge(wf_edges, &wf_cnt, j, i, 1, CAUSE_SWAP,
                    m->agents[j].pos ? m->agents[j].pos->x : -1,
                    m->agents[j].pos ? m->agents[j].pos->y : -1,
                    next_pos[j]->x, next_pos[j]->y);
            }
        }
    }

    int scc_mask = build_scc_mask_from_edges(wf_edges, wf_cnt);
    agv_record_wf_scc_metrics(wf_cnt, scc_mask ? 1 : 0);

    if (scc_mask) {
        int group_ids[kMaxCbsGroup];
        int group_n = 0;
        for (int i = 0; i < MAX_AGENTS && group_n < kMaxCbsGroup; i++) {
            if ((scc_mask & (1 << i)) == 0) continue;
            if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == nullptr) continue;
            if (m->agents[i].action_timer > 0 && m->agents[i].pos && m->agents[i].goal && m->agents[i].pos == m->agents[i].goal) continue;
            group_ids[group_n++] = i;
        }
        if (group_n >= 2) {
            Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = {{0}};
            int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
            if (ok) {
                for (int gi = 0; gi < group_n; gi++) {
                    int id = group_ids[gi];
                    if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
                }
            } else {
                int leader = best_in_mask(m, scc_mask);
                for (int gi = 0; gi < group_n; gi++) {
                    int id = group_ids[gi];
                    if (id == leader) continue;
                    Node* po = try_pull_over(map, &rt, &m->agents[id]);
                    if (po) next_pos[id] = po;
                }
                logger_log(lg, "[%sWFG%s] SCC detected: leader=%c commits, others pull over.", C_B_YEL, C_NRM, m->agents[leader].symbol);
            }
        }
    } else {
        int active_ids[MAX_AGENTS];
        int active_n = 0;
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* ag = &m->agents[i];
            if (ag->state == IDLE || ag->state == CHARGING || ag->goal == nullptr) continue;
            active_ids[active_n++] = i;
        }
        int all_wait = 1;
        for (int ai = 0; ai < active_n; ai++) {
            int id = active_ids[ai];
            if (next_pos[id] != m->agents[id].pos) {
                all_wait = 0;
                break;
            }
        }
        if (all_wait && active_n >= 2) {
            int fallback_order[MAX_AGENTS];
            sort_agents_by_priority(m, fallback_order);
            int group_ids[kMaxCbsGroup];
            int group_n = 0;
            for (int oi = 0; oi < MAX_AGENTS && group_n < kMaxCbsGroup; oi++) {
                int id = fallback_order[oi];
                Agent* ag = &m->agents[id];
                if (ag->state == IDLE || ag->state == CHARGING || ag->goal == nullptr) continue;
                if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) continue;
                group_ids[group_n++] = id;
            }
            if (group_n >= 2) {
                Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = {{0}};
                int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
                if (ok) {
                    for (int gi = 0; gi < group_n; gi++) {
                        int id = group_ids[gi];
                        if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
                    }
                    logger_log(lg, "[%sCBS%s] Deadlock fallback CBS engaged (group=%d).", C_B_CYN, C_NRM, group_n);
                } else {
                    int leader = best_in_mask(m, scc_mask ? scc_mask : 0x3FF);
                    for (int gi = 0; gi < group_n; gi++) {
                        int id = group_ids[gi];
                        if (id == leader) continue;
                        Node* po = try_pull_over(map, &rt, &m->agents[id]);
                        if (po) next_pos[id] = po;
                    }
                    logger_log(lg, "[%sWFG%s] Deadlock fallback: leader-only move, others pull-over.", C_B_YEL, C_NRM);
                }
            }
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        for (int j = i + 1; j < MAX_AGENTS; j++) {
            if (m->agents[i].state == IDLE || m->agents[j].state == IDLE ||
                m->agents[i].state == CHARGING || m->agents[j].state == CHARGING) continue;

            if (next_pos[i] == next_pos[j]) {
                if ((m->agents[i].state == GOING_TO_PARK && m->agents[j].state == RETURNING_HOME_EMPTY) ||
                    (m->agents[j].state == GOING_TO_PARK && m->agents[i].state == RETURNING_HOME_EMPTY)) {
                    if (m->agents[i].state == RETURNING_HOME_EMPTY) {
                        logger_log(lg, "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[i].symbol);
                        next_pos[i] = m->agents[i].pos;
                    } else {
                        logger_log(lg, "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    }
                } else {
                    int pi = priority_score(&m->agents[i]);
                    int pj = priority_score(&m->agents[j]);
                    if (pi >= pj) {
                        logger_log(lg, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    } else {
                        logger_log(lg, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[i].symbol);
                        next_pos[i] = m->agents[i].pos;
                    }
                }
            } else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {
                if ((m->agents[i].state == GOING_TO_PARK && m->agents[j].state == RETURNING_HOME_EMPTY) ||
                    (m->agents[j].state == GOING_TO_PARK && m->agents[i].state == RETURNING_HOME_EMPTY)) {
                    if (m->agents[i].state == RETURNING_HOME_EMPTY) {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[i].symbol);
                        next_pos[i] = m->agents[i].pos;
                    } else {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    }
                } else {
                    int pi = priority_score(&m->agents[i]);
                    int pj = priority_score(&m->agents[j]);
                    if (pi >= pj) {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    } else {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[i].symbol);
                        next_pos[i] = m->agents[i].pos;
                    }
                }
            }
        }
    }

    WHCA_adjustHorizon(wf_cnt, scc_mask ? 1 : 0, lg);
}

void agent_manager_plan_and_resolve_collisions_astar_core(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    int order[MAX_AGENTS];
    sort_agents_by_priority(manager, order);
    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        Agent* agent = &manager->agents[i];
        Node* current_pos = agent->pos;

        if (agent->rotation_wait > 0) {
            agent->rotation_wait--;
            continue;
        }
        if (agent->action_timer > 0) continue;
        if (agent->state == IDLE || agent->state == CHARGING || agent->goal == nullptr || !current_pos) continue;

        Node* desired_move = current_pos;
        ensure_pathfinder_for_agent(agent);
        TempMarkContextLocal ctx;
        temp_context_init_local(&ctx, agent->pf, map, manager, 1);
        temp_context_mark_order_blockers_local(&ctx, manager, order, oi, next_pos);
        int goal_was_parked = temporarily_unpark_goal_local(agent, agent->pf, map, manager);

        if (agent->pf) {
            desired_move = compute_ordered_pathfinder_move_local(
                agent, map, manager, ORDERED_PLANNING_ASTAR);
        }
        restore_temporarily_unparked_goal_local(agent, agent->pf, map, manager, goal_was_parked);
        temp_context_cleanup_local(&ctx);
        agent_apply_rotation_and_step_local(agent, current_pos, desired_move, &next_pos[i]);
    }

    resolve_conflicts_by_order_local(manager, order, next_pos);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic_core(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
    int order[MAX_AGENTS];
    sort_agents_by_priority(m, order);
    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        Agent* ag = &m->agents[i];
        Node* current_pos = ag->pos;

        if (ag->rotation_wait > 0) {
            ag->rotation_wait--;
            continue;
        }
        if (ag->action_timer > 0) continue;
        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == nullptr || !current_pos) continue;

        Node* desired_move = current_pos;
        ensure_pathfinder_for_agent(ag);
        TempMarkContextLocal ctx;
        temp_context_init_local(&ctx, ag->pf, map, m, 1);
        temp_context_mark_order_blockers_local(&ctx, m, order, oi, next_pos);
        int goal_was_parked = temporarily_unpark_goal_local(ag, ag->pf, map, m);

        if (ag->pf) {
            desired_move = compute_ordered_pathfinder_move_local(
                ag, map, m, ORDERED_PLANNING_DSTAR);
        }
        restore_temporarily_unparked_goal_local(ag, ag->pf, map, m, goal_was_parked);
        temp_context_cleanup_local(&ctx);
        agent_apply_rotation_and_step_local(ag, current_pos, desired_move, &next_pos[i]);
    }

    resolve_conflicts_by_order_local(m, order, next_pos);
}

void assign_goals_for_active_agents_local(AgentManager* manager, GridMap* map, Logger* logger) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        agv_set_goal_if_needed(agent, map, manager, logger);
    }
}

void seed_next_positions_from_current_local(AgentManager* manager, Node* next_pos[MAX_AGENTS]) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        next_pos[i] = manager->agents[i].pos;
    }
}

void agent_manager_plan_and_resolve_collisions(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    assign_goals_for_active_agents_local(manager, map, logger);
    agent_manager_plan_and_resolve_collisions_core(manager, map, logger, next_pos);
}

void agent_manager_plan_and_resolve_collisions_astar(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    assign_goals_for_active_agents_local(manager, map, logger);
    seed_next_positions_from_current_local(manager, next_pos);
    agent_manager_plan_and_resolve_collisions_astar_core(manager, map, logger, next_pos);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    assign_goals_for_active_agents_local(manager, map, logger);
    seed_next_positions_from_current_local(manager, next_pos);
    agent_manager_plan_and_resolve_collisions_dstar_basic_core(manager, map, logger, next_pos);
}

namespace {

class DefaultPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions(agents, map, logger, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DefaultPlannerStrategy>(*this);
    }
};

class AStarPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions_astar(agents, map, logger, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<AStarPlannerStrategy>(*this);
    }
};

class DStarPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions_dstar_basic(agents, map, logger, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DStarPlannerStrategy>(*this);
    }
};

Planner planner_make_default_local() {
    return Planner(std::make_unique<DefaultPlannerStrategy>());
}

Planner planner_make_astar_local() {
    return Planner(std::make_unique<AStarPlannerStrategy>());
}

Planner planner_make_dstar_local() {
    return Planner(std::make_unique<DStarPlannerStrategy>());
}

}  // namespace

Planner planner_from_pathalgo(PathAlgo algo) {
    switch (algo) {
    case PATHALGO_ASTAR_SIMPLE:
        return planner_make_astar_local();
    case PATHALGO_DSTAR_BASIC:
        return planner_make_dstar_local();
    case PATHALGO_DEFAULT:
    default:
        return planner_make_default_local();
    }
}
