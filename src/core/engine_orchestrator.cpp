// Legacy engine translation unit preserved during C++ migration.

#define _CRT_SECURE_NO_WARNINGS
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <windows.h>
#include <psapi.h>
#include <conio.h>
#ifdef _WIN32
#pragma comment(lib, "psapi.lib")
#endif

#define sleep_ms(ms) Sleep(ms)

#ifndef APPEND_FMT
#define APPEND_FMT(P, REM, ...)                            \
    do {                                                                \
        int __w = snprintf((P), (REM), __VA_ARGS__);                    \
        if (__w < 0) __w = 0;                                           \
        if ((size_t)__w > (REM)) __w = (int)(REM);                      \
        (P) += __w;                                                     \
        (REM) -= __w;                                                   \
    } while (0)
#endif

#define INPUT_BUFFER_SIZE 500

#define C_NRM "\x1b[0m"
#define C_RED "\x1b[31m"
#define C_GRN "\x1b[32m"
#define C_YEL "\x1b[33m"
#define C_BLU "\x1b[34m"
#define C_MAG "\x1b[35m"
#define C_CYN "\x1b[36m"
#define C_WHT "\x1b[37m"
#define C_GRY "\x1b[90m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#define C_B_MAG "\x1b[1;35m"
#define C_B_CYN "\x1b[1;36m"
#define C_B_WHT "\x1b[1;37m"

#define NUM_DIRECTIONS 4
#ifndef DIR4_COUNT
#define DIR4_COUNT 4
#endif

#define DISTANCE_BEFORE_CHARGE 300.0
#define CHARGE_TIME 20
#define REALTIME_MODE_TIMELIMIT 1000000
#define DASHBOARD_INTERVAL_STEPS 2500
#define MAX_TASKS 50
#define MAX_SPEED_MULTIPLIER 10000.0f
#define EVENT_GENERATION_INTERVAL 10
#ifndef CLEANUP_FORCE_IDLE_AFTER_STEPS
#define CLEANUP_FORCE_IDLE_AFTER_STEPS 11
#endif

#define STATUS_STRING_WIDTH 25

#ifndef PAUSE_POLL_INTERVAL_MS
#define PAUSE_POLL_INTERVAL_MS 50
#endif
#ifndef RENDER_STRIDE_MAX
#define RENDER_STRIDE_MAX 8
#endif
#ifndef RENDER_STRIDE_MIN
#define RENDER_STRIDE_MIN 1
#endif

#ifndef TASK_ACTION_TICKS
#define TASK_ACTION_TICKS 10
#endif
#ifndef PRIORITY_RETURNING_WITH_CAR
#define PRIORITY_RETURNING_WITH_CAR 3
#endif
#ifndef PRIORITY_GOING_TO_CHARGE
#define PRIORITY_GOING_TO_CHARGE 2
#endif
#ifndef PRIORITY_MOVING_TASK
#define PRIORITY_MOVING_TASK 1
#endif
#ifndef STUCK_BOOST_MULT
#define STUCK_BOOST_MULT 10
#endif
#ifndef STUCK_BOOST_HARD
#define STUCK_BOOST_HARD 1000
#endif

#define DEADLOCK_THRESHOLD 5
#define MAX_WAIT_EDGES 128
#define MAX_CBS_GROUP  8
#define MAX_CBS_NODES  256
#define CBS_MAX_EXPANSIONS 128
#define MAX_TOT (((MAX_WHCA_HORIZON)+1) * GRID_WIDTH * GRID_HEIGHT)

#include "agv/internal/engine_internal.hpp"

static thread_local Simulation* g_active_simulation = nullptr;

static Simulation* simulation_active();
static void simulation_set_active(Simulation* sim);
static Simulation& require_active_simulation();
static PlannerMetricsState& planner_metrics_state();
static RuntimeTuningState& runtime_tuning_state();
static RendererState& renderer_state();
static StepScratch& step_scratch_state();
static std::array<char, DISPLAY_BUFFER_SIZE>& display_buffer_state();
static int& whca_horizon_state();
static int& conflict_score_state();

static Simulation* simulation_active() { return g_active_simulation; }
static void simulation_set_active(Simulation* sim) { g_active_simulation = sim; }
static Simulation& require_active_simulation() { return *g_active_simulation; }
static PlannerMetricsState& planner_metrics_state() { return require_active_simulation().planner_metrics; }
static RuntimeTuningState& runtime_tuning_state() { return require_active_simulation().runtime_tuning; }
static RendererState& renderer_state() { return require_active_simulation().render_state; }
static StepScratch& step_scratch_state() { return require_active_simulation().step_scratch; }
static std::array<char, DISPLAY_BUFFER_SIZE>& display_buffer_state() { return require_active_simulation().display_buffer; }
static int& whca_horizon_state() { return runtime_tuning_state().whca_horizon; }
static int& conflict_score_state() { return runtime_tuning_state().conflict_score; }

int agv_current_whca_horizon() { return whca_horizon_state(); }
int agv_current_conflict_score() { return conflict_score_state(); }
void agv_set_whca_runtime_state(int conflict_score, int horizon) {
    conflict_score_state() = conflict_score;
    whca_horizon_state() = horizon;
    planner_metrics_state().whca_h = whca_horizon_state();
}
void agv_accumulate_cbs_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions) {
    planner_metrics_state().whca_nodes_expanded_this_step += nodes_expanded;
    planner_metrics_state().whca_heap_moves_this_step += heap_moves;
    planner_metrics_state().whca_generated_nodes_this_step += generated_nodes;
    planner_metrics_state().whca_valid_expansions_this_step += valid_expansions;
}
void agv_accumulate_whca_dstar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions) {
    planner_metrics_state().whca_dstar_nodes_expanded_this_step += nodes_expanded;
    planner_metrics_state().whca_dstar_heap_moves_this_step += heap_moves;
    planner_metrics_state().whca_dstar_generated_nodes_this_step += generated_nodes;
    planner_metrics_state().whca_dstar_valid_expansions_this_step += valid_expansions;
}
void agv_accumulate_astar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions) {
    planner_metrics_state().astar_nodes_expanded_this_step += nodes_expanded;
    planner_metrics_state().astar_heap_moves_this_step += heap_moves;
    planner_metrics_state().astar_generated_nodes_this_step += generated_nodes;
    planner_metrics_state().astar_valid_expansions_this_step += valid_expansions;
}
void agv_accumulate_dstar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions) {
    planner_metrics_state().dstar_nodes_expanded_this_step += nodes_expanded;
    planner_metrics_state().dstar_heap_moves_this_step += heap_moves;
    planner_metrics_state().dstar_generated_nodes_this_step += generated_nodes;
    planner_metrics_state().dstar_valid_expansions_this_step += valid_expansions;
}
void agv_record_wf_scc_metrics(int wf_edges, int scc) {
    planner_metrics_state().wf_edges_last = wf_edges;
    planner_metrics_state().wf_edges_sum += wf_edges;
    planner_metrics_state().scc_last = scc;
    planner_metrics_state().scc_sum += scc;
}
void agv_record_cbs_success(int expansions) {
    planner_metrics_state().cbs_ok_last = 1;
    planner_metrics_state().cbs_exp_last = expansions;
    planner_metrics_state().cbs_success_sum++;
}
void agv_record_cbs_failure(int expansions) {
    planner_metrics_state().cbs_ok_last = 0;
    planner_metrics_state().cbs_exp_last = expansions;
    planner_metrics_state().cbs_fail_sum++;
}

class SimulationContextGuard final {
public:
    explicit SimulationContextGuard(Simulation* sim)
        : previous_(simulation_active()) {
        simulation_set_active(sim);
    }

    ~SimulationContextGuard() {
        simulation_set_active(previous_);
    }

private:
    Simulation* previous_{ nullptr };
};

static const int DIR4_X[4] = { 0, 0, 1, -1 };
static const int DIR4_Y[4] = { 1, -1, 0, 0 };
static const int DIR5_X[5] = { 0, 1, -1, 0, 0 };
static const int DIR5_Y[5] = { 0, 0, 0, 1, -1 };
#ifndef DIR5_COUNT
#define DIR5_COUNT 5
#endif

static inline double manhattan_nodes(const Node* a, const Node* b) {
    return fabs((double)a->x - (double)b->x) + fabs((double)a->y - (double)b->y);
}

static inline double manhattan_xy(int x1, int y1, int x2, int y2) {
    return fabs((double)x1 - (double)x2) + fabs((double)y1 - (double)y2);
}

typedef void (*MetricsObserverFn)(void* ctx, const MetricsSnapshot* snap);
struct MetricsObserver {
    MetricsObserverFn fn{ nullptr };
    void* ctx{ nullptr };
};

#ifndef MAX_METRICS_OBSERVERS
#define MAX_METRICS_OBSERVERS 8
#endif

static inline MetricsSnapshot metrics_build_snapshot(void) {
    MetricsSnapshot s;
    PlannerMetricsState& metrics = planner_metrics_state();
    RuntimeTuningState& tuning = runtime_tuning_state();
    s.whca_h = metrics.whca_h;
    s.wf_edges_last = metrics.wf_edges_last;
    s.wf_edges_sum = metrics.wf_edges_sum;
    s.scc_last = metrics.scc_last;
    s.scc_sum = metrics.scc_sum;
    s.cbs_ok_last = metrics.cbs_ok_last;
    s.cbs_exp_last = metrics.cbs_exp_last;
    s.cbs_success_sum = metrics.cbs_success_sum;
    s.cbs_fail_sum = metrics.cbs_fail_sum;
    s.whca_horizon = tuning.whca_horizon;
    return s;
}

class MetricsObserverRegistry final {
public:
    void subscribe(MetricsObserverFn fn, void* ctx) {
        if (!fn) return;
        if (observer_count_ >= MAX_METRICS_OBSERVERS) return;
        observers_[observer_count_++] = MetricsObserver{ fn, ctx };
    }

    void unsubscribe(void* ctx) {
        if (!ctx) return;
        int write_index = 0;
        for (int read_index = 0; read_index < observer_count_; ++read_index) {
            if (observers_[read_index].ctx == ctx) {
                continue;
            }
            observers_[write_index++] = observers_[read_index];
        }
        for (int i = write_index; i < observer_count_; ++i) {
            observers_[i] = MetricsObserver{};
        }
        observer_count_ = write_index;
    }

    void notifyAll() const {
        MetricsSnapshot snap = metrics_build_snapshot();
        for (int i = 0; i < observer_count_; i++) {
            if (observers_[i].fn) observers_[i].fn(observers_[i].ctx, &snap);
        }
    }

private:
    MetricsObserver observers_[MAX_METRICS_OBSERVERS]{};
    int observer_count_{ 0 };
};
static MetricsObserverRegistry g_metrics_registry;

static void metrics_subscribe(MetricsObserverFn fn, void* ctx) {
    g_metrics_registry.subscribe(fn, ctx);
}

static void metrics_unsubscribe(void* ctx) {
    g_metrics_registry.unsubscribe(ctx);
}

static void metrics_notify_all(void) {
    g_metrics_registry.notifyAll();
}

static void simulation_metrics_observer(void* ctx, const MetricsSnapshot* s) {
    Simulation* sim = (Simulation*)ctx;
    if (!sim || !s) return;
    sim->whca_horizon_shadow = s->whca_horizon;
    sim->algo_rt_metrics_shadow.whca_h = s->whca_h;
    sim->algo_rt_metrics_shadow.wf_edges_last = s->wf_edges_last;
    sim->algo_rt_metrics_shadow.wf_edges_sum = s->wf_edges_sum;
    sim->algo_rt_metrics_shadow.scc_last = s->scc_last;
    sim->algo_rt_metrics_shadow.scc_sum = s->scc_sum;
    sim->algo_rt_metrics_shadow.cbs_ok_last = s->cbs_ok_last;
    sim->algo_rt_metrics_shadow.cbs_exp_last = s->cbs_exp_last;
    sim->algo_rt_metrics_shadow.cbs_success_sum = s->cbs_success_sum;
    sim->algo_rt_metrics_shadow.cbs_fail_sum = s->cbs_fail_sum;
}

#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

void simulation_run(Simulation* sim);
void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag);
void simulation_display_status(Simulation* sim, int is_paused);
static void maybe_report_realtime_dashboard(Simulation* sim);
Planner planner_from_pathalgo(PathAlgo algo);
RendererFacade renderer_create_facade(void);

void logger_log(Logger* logger, const char* format, ...);
int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap*, const AgentManager*, const Node*, const struct Agent_*);
void agent_manager_plan_and_resolve_collisions(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_plan_and_resolve_collisions_core(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_update_state_after_move(AgentManager*, ScenarioManager*, GridMap*, Logger*, Simulation*);
void agent_manager_update_charge_state(AgentManager*, GridMap*, Logger*);
void agent_manager_plan_and_resolve_collisions_astar(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_plan_and_resolve_collisions_astar_core(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_plan_and_resolve_collisions_dstar_basic_core(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);

Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
void pathfinder_update_start(Pathfinder* pf, Node* new_start);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
Node* pathfinder_get_next_step(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* current_node);

void ReservationTable_clear(ReservationTable* r);
void ReservationTable_seedCurrent(ReservationTable* r, AgentManager* m);
int ReservationTable_isOccupied(const ReservationTable* r, int t, const Node* n);
int ReservationTable_getOccupant(const ReservationTable* r, int t, const Node* n);
void ReservationTable_setOccupant(ReservationTable* r, int t, const Node* n, int agent_id);
void add_wait_edge(WaitEdge* edges, int* cnt, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2);
int build_scc_mask_from_edges(const WaitEdge* edges, int cnt);
int run_partial_CBS(AgentManager* m, GridMap* map, Logger* lg,
    int group_ids[], int group_n, const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]);
Node* try_pull_over(GridMap* map, const ReservationTable* rt, Agent* ag);
int priority_score(const Agent* ag);
void sort_agents_by_priority(AgentManager* m, int order[MAX_AGENTS]);
int best_candidate_order(Pathfinder* pf, GridMap* map, const AgentManager* am,
    Node* cur, Node* goal, Node* out[5], int* outN);
void ensure_pathfinder_for_agent(Agent* ag);
int st_astar_plan_single(int agent_id, GridMap* map, Node* start, Node* goal, int horizon,
    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    const CBSConstraint* cons, int ncons,
    Node* out_plan[MAX_WHCA_HORIZON + 1],
    AgentDir initial_heading,
    unsigned long long* out_nodes_expanded,
    unsigned long long* out_heap_moves,
    unsigned long long* out_generated_nodes,
    unsigned long long* out_valid_expansions);

Planner::Planner(std::unique_ptr<PlannerStrategy> strategy)
    : strategy_(std::move(strategy)) {}

Planner::Planner(const Planner& other) {
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
}

Planner& Planner::operator=(const Planner& other) {
    if (this == &other) return *this;
    strategy_.reset();
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
    return *this;
}

void Planner::reset(std::unique_ptr<PlannerStrategy> strategy) {
    strategy_ = std::move(strategy);
}

void Planner::planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const {
    if (strategy_) {
        strategy_->planStep(agents, map, logger, next_pos);
    }
}

RendererFacade::RendererFacade(std::unique_ptr<RendererStrategy> strategy)
    : strategy_(std::move(strategy)) {}

RendererFacade::RendererFacade(const RendererFacade& other) {
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
}

RendererFacade& RendererFacade::operator=(const RendererFacade& other) {
    if (this == &other) return *this;
    strategy_.reset();
    if (other.strategy_) {
        strategy_ = other.strategy_->clone();
    }
    return *this;
}

void RendererFacade::drawFrame(Simulation_* sim, int is_paused) const {
    if (strategy_) {
        strategy_->drawFrame(sim, is_paused);
    }
}

void Logger::logV(const char* fmt, va_list args) {
    int idx = (log_head + log_count) % LOG_BUFFER_LINES;
    vsnprintf(logs[idx], LOG_BUFFER_WIDTH, fmt, args);
    if (log_count < LOG_BUFFER_LINES) {
        log_count++;
    }
    else {
        log_head = (log_head + 1) % LOG_BUFFER_LINES;
    }
}

void Logger::log(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logV(fmt, args);
    va_end(args);
}

Simulation_::Simulation_() {
    simulation_set_active(this);
    map_id = 1;
    path_algo = PATHALGO_DEFAULT;
    planner = planner_from_pathalgo(path_algo);
    renderer = renderer_create_facade();
    render_state.configureForAlgorithm(path_algo);
    planner_metrics.whca_h = runtime_tuning.whca_horizon;
    grid_map_load_scenario(map, agent_manager, map_id);
    metrics_subscribe(simulation_metrics_observer, this);
}

Simulation_::~Simulation_() {
    metrics_unsubscribe(this);
    if (simulation_active() == this) {
        simulation_set_active(nullptr);
    }
}


// =============================================================================

// =============================================================================


void Simulation_::collectMemorySample() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
        memory_usage_sum_kb += working_set_kb;
        if (working_set_kb > memory_usage_peak_kb) memory_usage_peak_kb = working_set_kb;
        memory_samples++;
    }
#endif
}

void Simulation_::collectMemorySampleAlgo() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
        algo_mem_sum_kb += working_set_kb;
        if (working_set_kb > algo_mem_peak_kb) algo_mem_peak_kb = working_set_kb;
        algo_mem_samples++;
    }
#endif
}


void Simulation_::resetRuntimeStats() {
    total_cpu_time_ms = 0.0;
    last_step_cpu_time_ms = 0.0;
    max_step_cpu_time_ms = 0.0;
    memset(phase_cpu_time_ms, 0, sizeof(phase_cpu_time_ms));
    memset(phase_step_counts, 0, sizeof(phase_step_counts));
    memset(phase_completed_tasks, 0, sizeof(phase_completed_tasks));
    for (int i = 0; i < MAX_PHASES; i++) {
        phase_first_step[i] = -1;
        phase_last_step[i] = -1;
    }
    post_phase_cpu_time_ms = 0.0;
    post_phase_step_count = 0;
    post_phase_first_step = -1;
    post_phase_last_step = -1;
    total_planning_time_ms = 0.0;
    last_planning_time_ms = 0.0;
    max_planning_time_ms = 0.0;
    tasks_completed_total = 0;
    algorithm_operation_count = 0;
    total_movement_cost = 0.0;
    deadlock_count = 0;
    memory_usage_sum_kb = 0.0;
    memory_usage_peak_kb = 0.0;
    memory_samples = 0;
    algo_mem_sum_kb = 0.0;
    algo_mem_peak_kb = 0.0;
    algo_mem_samples = 0;
    last_task_completion_step = 0;
    total_executed_steps = 0;
    last_report_completed_tasks = 0;
    last_report_step = 0;
    metrics_task_count = 0;
    metrics_sum_dmove = 0.0;
    metrics_sum_turns = 0;
    metrics_sum_ttask = 0.0;
    requests_created_total = 0;
    request_wait_ticks_sum = 0;
    algo_generated_nodes_total = 0;
    algo_valid_expansions_total = 0;
    algo_generated_nodes_last_step = 0;
    algo_valid_expansions_last_step = 0;
}

void Simulation_::reportRealtimeDashboard() {
    ScenarioManager* sc = scenario_manager;
    int steps = (total_executed_steps > 0) ? total_executed_steps : (sc ? sc->time_step : 0);
    if (steps <= 0) steps = 1;

    unsigned long long total_completed = tasks_completed_total;
    int interval_steps = steps - last_report_step;
    if (interval_steps <= 0) interval_steps = 1;
    unsigned long long delta_completed = total_completed - last_report_completed_tasks;

    double throughput_avg = (double)total_completed / (double)steps;
    double throughput_interval = (double)delta_completed / (double)interval_steps;
    double avg_planning_ms = (steps > 0) ? total_planning_time_ms / (double)steps : 0.0;
    double avg_memory_kb = (memory_samples > 0) ? memory_usage_sum_kb / (double)memory_samples : 0.0;

    int active_agents = 0;
    if (agent_manager) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            if (agent_manager->agents[i].pos) active_agents++;
        }
    }

    if (suppress_stdout) {
        last_report_completed_tasks = total_completed;
        last_report_step = steps;
        return;
    }

    printf("\n========== Real-Time Dashboard @ step %d ==========\n", steps);
    printf(" Total Physical Time Steps      : %d\n", steps);
    printf(" Operating AGVs                 : %d\n", active_agents);
    printf(" Tasks Completed (total)        : %llu\n", total_completed);
    printf(" Throughput (total avg)         : %.4f tasks/step\n", throughput_avg);
    printf(" Throughput (last interval)     : %.4f tasks/step over %d steps\n", throughput_interval, interval_steps);
    printf(" Total Computation CPU Time     : %.2f ms\n", total_cpu_time_ms);
    printf(" Average Planning Time / Step   : %.4f ms\n", avg_planning_ms);
    printf(" Total Task Completion Step     : %d\n", last_task_completion_step);
    printf(" Total Movement Cost            : %.2f cells\n", total_movement_cost);
    printf(" Requests Created (total)       : %llu\n", requests_created_total);
    printf(" Request Wait Ticks (sum)       : %llu\n", request_wait_ticks_sum);
    printf(" Process Memory Usage Sum      : %.2f KB (avg %.2f KB / sample, peak %.2f KB)\n",
        memory_usage_sum_kb, avg_memory_kb, memory_usage_peak_kb);
    printf(" Heap Moves (total/last)          : %llu / %llu\n", algo_heap_moves_total, algo_heap_moves_last_step);
    printf(" Generated Nodes (total/last)     : %llu / %llu\n", algo_generated_nodes_total, algo_generated_nodes_last_step);
    printf(" Valid Expansions (total/last)    : %llu / %llu\n", algo_valid_expansions_total, algo_valid_expansions_last_step);
    double dash_ratio_total = (algo_generated_nodes_total > 0) ? (double)algo_valid_expansions_total / (double)algo_generated_nodes_total : 0.0;
    double dash_ratio_last = (algo_generated_nodes_last_step > 0) ? (double)algo_valid_expansions_last_step / (double)algo_generated_nodes_last_step : 0.0;
    printf(" Valid Expansion Ratio (total/last): %.4f / %.4f\n", dash_ratio_total, dash_ratio_last);
    printf("===================================================\n");

    last_report_completed_tasks = total_completed;
    last_report_step = steps;
}

struct PlannerStepCounters final {
    unsigned long long nodes_expanded{0};
    unsigned long long heap_moves{0};
    unsigned long long generated_nodes{0};
    unsigned long long valid_expansions{0};
};

static void reset_planner_step_metrics(Simulation* sim) {
    sim->algo_nodes_expanded_last_step = 0;
    sim->algo_heap_moves_last_step = 0;
    sim->algo_generated_nodes_last_step = 0;
    sim->algo_valid_expansions_last_step = 0;
    planner_metrics_state().resetStepCounters();

    AgentManager* agent_manager = sim->agent_manager;
    if (!agent_manager) return;

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &agent_manager->agents[i];
        if (!ag->pf) continue;
        ag->pf->nodes_expanded_this_call = 0;
        ag->pf->heap_moves_this_call = 0;
        ag->pf->nodes_generated_this_call = 0;
        ag->pf->valid_expansions_this_call = 0;
    }
}

static PlannerStepCounters collect_planner_step_counters(PathAlgo path_algo, const PlannerMetricsState& metrics) {
    if (path_algo == PATHALGO_ASTAR_SIMPLE) {
        return {
            metrics.astar_nodes_expanded_this_step,
            metrics.astar_heap_moves_this_step,
            metrics.astar_generated_nodes_this_step,
            metrics.astar_valid_expansions_this_step,
        };
    }

    if (path_algo == PATHALGO_DSTAR_BASIC) {
        return {
            metrics.dstar_nodes_expanded_this_step,
            metrics.dstar_heap_moves_this_step,
            metrics.dstar_generated_nodes_this_step,
            metrics.dstar_valid_expansions_this_step,
        };
    }

    return {
        metrics.whca_dstar_nodes_expanded_this_step + metrics.whca_nodes_expanded_this_step,
        metrics.whca_dstar_heap_moves_this_step + metrics.whca_heap_moves_this_step,
        metrics.whca_dstar_generated_nodes_this_step + metrics.whca_generated_nodes_this_step,
        metrics.whca_dstar_valid_expansions_this_step + metrics.whca_valid_expansions_this_step,
    };
}

static void record_planner_step_results(
    Simulation* sim,
    const PlannerStepCounters& step_counters,
    const PlannerMetricsState& metrics,
    clock_t plan_start_cpu,
    clock_t plan_end_cpu) {
    sim->algo_nodes_expanded_last_step = step_counters.nodes_expanded;
    sim->algo_heap_moves_last_step = step_counters.heap_moves;
    sim->algo_generated_nodes_last_step = step_counters.generated_nodes;
    sim->algo_valid_expansions_last_step = step_counters.valid_expansions;
    sim->algo_nodes_expanded_total += step_counters.nodes_expanded;
    sim->algo_heap_moves_total += step_counters.heap_moves;
    sim->algo_generated_nodes_total += step_counters.generated_nodes;
    sim->algo_valid_expansions_total += step_counters.valid_expansions;

    double planning_time_ms = ((double)(plan_end_cpu - plan_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
    sim->last_planning_time_ms = planning_time_ms;
    sim->total_planning_time_ms += planning_time_ms;
    if (planning_time_ms > sim->max_planning_time_ms) sim->max_planning_time_ms = planning_time_ms;
    sim->algorithm_operation_count += (unsigned long long)((metrics.wf_edges_last > 0 ? metrics.wf_edges_last : 0) +
        (metrics.scc_last > 0 ? metrics.scc_last : 0) +
        (metrics.cbs_exp_last > 0 ? metrics.cbs_exp_last : 0));

    metrics_notify_all();
}


void Simulation_::planStep(Node* next_pos[MAX_AGENTS]) {
    SimulationContextGuard guard(this);
    Simulation* sim = this;
    clock_t plan_start_cpu = clock();

    // Reset per-step metrics
    reset_planner_step_metrics(sim);

    sim->planner.planStep(sim->agent_manager, sim->map, sim->logger, next_pos);

    const PlannerMetricsState& metrics = planner_metrics_state();
    const PlannerStepCounters step_counters = collect_planner_step_counters(sim->path_algo, metrics);

    clock_t plan_end_cpu = clock();
    record_planner_step_results(sim, step_counters, metrics, plan_start_cpu, plan_end_cpu);
}

static void maybe_report_realtime_dashboard(Simulation* sim) {
    ScenarioManager* sc = sim->scenario_manager;
    sc->time_step++;
    if (sc->mode == MODE_REALTIME && (sc->time_step % DASHBOARD_INTERVAL_STEPS) == 0) {
        sim->reportRealtimeDashboard();
    }
}

static int are_all_agents_idle(const AgentManager* agent_manager) {
    if (!agent_manager) return true;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].state != IDLE) return false;
    }
    return true;
}

static void print_completion_message_if_needed(const Simulation* sim, const char* message) {
    if (!sim || sim->suppress_stdout) return;
    printf(C_B_GRN "\n%s\n" C_NRM, message);
}

static int is_custom_scenario_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    if (!scenario || scenario->mode != MODE_CUSTOM) return false;
    if (scenario->current_phase_index < scenario->num_phases) return false;
    return are_all_agents_idle(sim->agent_manager);
}

static int is_realtime_scenario_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    return scenario && scenario->mode == MODE_REALTIME &&
        scenario->time_step >= REALTIME_MODE_TIMELIMIT;
}

static int read_control_key() {
    return _kbhit() ? _getch() : 0;
}

static void handle_control_input(Simulation* sim, int last_key, int* is_paused, int* quit_flag) {
    if (!last_key) return;
    ui_handle_control_key(sim, last_key, is_paused, quit_flag);
    sim->renderer.drawFrame(sim, *is_paused);
}

static int should_wait_while_paused(int is_paused, int last_key) {
    return is_paused && std::tolower(last_key) != 's';
}

static void maybe_sleep_for_simulation_speed(const ScenarioManager* scenario) {
    if (scenario && scenario->simulation_speed > 0) {
        sleep_ms(scenario->simulation_speed);
    }
}

// =============================================================================
// 4.5) Renderer Facade Implementation (delegates to simulation_display_status)
// =============================================================================
// Logger
void logger_log(Logger* l, const char* fmt, ...) {
    if (!l) return;
    va_list a;
    va_start(a, fmt);
    l->logV(fmt, a);
    va_end(a);
}

// =============================================================================

// =============================================================================









int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* n, const struct Agent_* agent) {
    constexpr int kReturnHomeBayEscapeStuckSteps = 6;
    if (n->is_obstacle || n->is_parked || n->is_temp) return true;


    if (agent && agent->state == RETURNING_HOME_EMPTY && n->is_goal && !n->is_parked) {
        const bool allow_temporary_bay_escape =
            agent->stuck_steps >= kReturnHomeBayEscapeStuckSteps &&
            (n->reserved_by_agent == -1 || n->reserved_by_agent == agent->id);
        if (!allow_temporary_bay_escape) {
            return true;
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        if (am->agents[i].pos == n && am->agents[i].state == CHARGING) return true;
    }
    return false;
}

// =============================================================================

// =============================================================================

void Simulation_::updateState() {
    SimulationContextGuard guard(this);
    agv_update_task_dispatch(this);
}

void Simulation_::executeOneStep(int is_paused) {
    SimulationContextGuard guard(this);
    agv_execute_step_service(this, is_paused);
}
bool Simulation_::isComplete() const {
    const Simulation* sim = this;
    if (is_custom_scenario_complete(sim)) {
        print_completion_message_if_needed(sim, "Custom scenario completed.");
        return true;
    }
    if (is_realtime_scenario_complete(sim)) {
        print_completion_message_if_needed(sim, "Real-time simulation completed.");
        return true;
    }
    return false;
}

void Simulation_::run() {
    SimulationContextGuard guard(this);
    Simulation* sim = this;
    int is_paused = false;
    int quit_flag = false;

    sim->resetRuntimeStats();

    sim->renderer.drawFrame(sim, is_paused);

    while (!quit_flag) {
        const int last_key = read_control_key();
        handle_control_input(sim, last_key, &is_paused, &quit_flag);
        if (quit_flag) continue;
        if (should_wait_while_paused(is_paused, last_key)) {
            sleep_ms(PAUSE_POLL_INTERVAL_MS);
            continue;
        }
        sim->executeOneStep(is_paused);
        if (sim->isComplete()) {
            break;
        }
        maybe_report_realtime_dashboard(sim);
        maybe_sleep_for_simulation_speed(sim->scenario_manager);
    }
}

bool execute_headless_step(Simulation* sim) {
    if (!sim) return true;
    if (sim->isComplete()) return true;
    sim->executeOneStep(false);
    if (sim->isComplete()) return true;
    maybe_report_realtime_dashboard(sim);
    if (sim->scenario_manager->simulation_speed > 0) sleep_ms(sim->scenario_manager->simulation_speed);
    return false;
}

bool run_simulation_to_completion(Simulation* sim) {
    if (!sim) return false;
    while (!execute_headless_step(sim)) {
    }
    return true;
}

int copy_render_frame(Simulation* sim, bool is_paused, char* buffer, size_t buffer_size) {
    if (!sim || !buffer || buffer_size == 0) return 0;
    SimulationContextGuard guard(sim);
    {
        const bool prev_suppress = renderer_state().suppress_flush;
        renderer_state().suppress_flush = true;
        simulation_display_status(sim, is_paused);
        renderer_state().suppress_flush = prev_suppress;
    }
    snprintf(buffer, buffer_size, "%s", sim->display_buffer.data());
    return (int)strlen(buffer);
}

#ifndef AGV_NO_MAIN
int main() {
    srand((unsigned int)time(nullptr));
    agv_prepare_console();
    Simulation sim;

    if (simulation_setup(&sim)) {
        ui_enter_alt_screen();
        sim.run();
        ui_leave_alt_screen();
        sim.printPerformanceSummary();
        printf("\nPress any key to exit...\n");
        (void)_getch();
    }
    else {
        printf("\nSimulation setup was cancelled. Exiting.\n");
    }


    return 0;
}
#endif


