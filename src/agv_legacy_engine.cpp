// Legacy engine translation unit preserved during C++ migration.

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <time.h>
#include <stdarg.h>
#include <ctype.h>
#include <locale.h>

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

#define TRUE 1
#define FALSE 0
#define INPUT_BUFFER_SIZE 500
#define DISPLAY_BUFFER_SIZE 512000

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

static const char* AGENT_COLORS[10] = {
    C_B_CYN, C_B_YEL, C_B_MAG, C_B_GRN, C_B_RED,
    C_B_WHT, C_CYN,   C_YEL,   C_MAG,   C_GRN
};

#define GRID_WIDTH  82
#define GRID_HEIGHT 42
#define MAX_AGENTS  16
#define MAX_GOALS   (GRID_WIDTH * GRID_HEIGHT)
#define INF 1e18
#define NUM_DIRECTIONS 4
#ifndef DIR4_COUNT
#define DIR4_COUNT 4
#endif

typedef enum {
    DIR_NONE = -1,
    DIR_UP = 0,
    DIR_RIGHT = 1,
    DIR_DOWN = 2,
    DIR_LEFT = 3
} AgentDir;

#define DISTANCE_BEFORE_CHARGE 300.0
#define CHARGE_TIME 20
#define MAX_CHARGE_STATIONS 10
#define MAX_PHASES 20
#define REALTIME_MODE_TIMELIMIT 1000000
#define DASHBOARD_INTERVAL_STEPS 2500
#define MAX_TASKS 50
#define MAX_SPEED_MULTIPLIER 10000.0f
#define EVENT_GENERATION_INTERVAL 10
#ifndef CLEANUP_FORCE_IDLE_AFTER_STEPS
#define CLEANUP_FORCE_IDLE_AFTER_STEPS 11
#endif

#define LOG_BUFFER_LINES 5
#define LOG_BUFFER_WIDTH 256
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
#define MIN_WHCA_HORIZON 5
#define MAX_WHCA_HORIZON 11
#define MAX_WAIT_EDGES 128
#define MAX_CBS_GROUP  8
#define MAX_CBS_CONS   128
#define MAX_CBS_NODES  256
#define CBS_MAX_EXPANSIONS 128
#define MAX_TOT (((MAX_WHCA_HORIZON)+1) * GRID_WIDTH * GRID_HEIGHT)

#ifndef TEMP_MARK_MAX
#define TEMP_MARK_MAX 128
#endif

#include "agv/legacy_engine_model.hpp"

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
static inline int node_flat_index(const Node* node);

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
static inline int node_flat_index(const Node* node) { return node ? (node->y * GRID_WIDTH + node->x) : -1; }

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

static inline AgentDir dir_from_delta(int dx, int dy) {
    if (dx == 1 && dy == 0) return DIR_RIGHT;
    if (dx == -1 && dy == 0) return DIR_LEFT;
    if (dx == 0 && dy == -1) return DIR_UP;
    if (dx == 0 && dy == 1) return DIR_DOWN;
    return DIR_NONE;
}

static inline int dir_turn_steps(AgentDir from, AgentDir to) {
    if (from == DIR_NONE || to == DIR_NONE) return 0;
    int diff = ((int)to - (int)from + NUM_DIRECTIONS) % NUM_DIRECTIONS;
    return diff <= 2 ? diff : 4 - diff;
}

#ifndef TURN_90_WAIT
#define TURN_90_WAIT 2
#endif

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

static inline void temp_mark_init(TempMarkList* l) {
    if (l) l->count = 0;
}

static inline void temp_mark_node(TempMarkList* l, Node* n) {
    if (!l || !n) return;
    if (!n->is_temp) {
        n->is_temp = TRUE;
        if (l->count < TEMP_MARK_MAX) l->nodes[l->count++] = n;
    }
}

static inline void temp_unmark_all(TempMarkList* l) {
    if (!l) return;
    for (int i = 0; i < l->count; i++) {
        if (l->nodes[i]) l->nodes[i]->is_temp = FALSE;
    }
    l->count = 0;
}

static inline void agent_apply_rotation_and_step(Agent* ag, Node* current, Node* desired, Node** out_next) {
    if (!ag || !current || !out_next) return;
    *out_next = current;
    if (!desired || desired == current) return;

    int dx = desired->x - current->x;
    int dy = desired->y - current->y;
    AgentDir new_heading = dir_from_delta(dx, dy);
    if (new_heading == DIR_NONE) return;

    if (ag->heading == DIR_NONE) {
        ag->heading = new_heading;
        *out_next = desired;
        return;
    }

    int turn_steps = dir_turn_steps(ag->heading, new_heading);
    if (turn_steps == 1) {
        ag->rotation_wait = TURN_90_WAIT - 1;
        ag->heading = new_heading;
        ag->metrics_turns_current++;
        return;
    }
    ag->heading = new_heading;
    *out_next = desired;
}

static inline void metrics_finalize_task_if_active(Simulation* sim, Agent* ag) {
    if (!sim || !ag || !ag->metrics_task_active) return;
    int steps_now = (sim->scenario_manager ? sim->scenario_manager->time_step : 0);
    double d_move = ag->total_distance_traveled - ag->metrics_distance_at_start;
    int turns = ag->metrics_turns_current;
    double t_task = (double)(steps_now - ag->metrics_task_start_step);
    if (t_task < 0) t_task = 0;
    sim->metrics_task_count++;
    sim->metrics_sum_dmove += d_move;
    sim->metrics_sum_turns += turns;
    sim->metrics_sum_ttask += t_task;
    ag->metrics_task_active = 0;
    ag->metrics_turns_current = 0;
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

typedef enum {
    AGV_PHASECFG_PARK = 0,
    AGV_PHASECFG_EXIT = 1
} AgvPhaseType;

typedef enum {
    AGV_MODECFG_CUSTOM = 0,
    AGV_MODECFG_REALTIME = 1
} AgvSimulationMode;

struct AgvPhaseConfig {
    int type;
    int task_count;
};

struct AgvSimulationConfig {
    unsigned int seed;
    int map_id;
    int path_algo;
    int mode;
    float speed_multiplier;
    int realtime_park_chance;
    int realtime_exit_chance;
    int num_phases;
    AgvPhaseConfig phases[MAX_PHASES];
    int suppress_stdout;
    const char* step_metrics_path;
};

struct AgvRunSummary {
    unsigned int seed;
    int map_id;
    int path_algo;
    int mode;
    int active_agents;
    int recorded_steps;
    unsigned long long tasks_completed_total;
    double throughput;
    double total_movement_cost;
    unsigned long long deadlock_count;
    double total_cpu_time_ms;
    double avg_cpu_time_ms;
    double total_planning_time_ms;
    double avg_planning_time_ms;
    double memory_usage_sum_kb;
    double avg_memory_usage_kb;
    double memory_usage_peak_kb;
    unsigned long long algo_nodes_expanded_total;
    unsigned long long algo_heap_moves_total;
    unsigned long long algo_generated_nodes_total;
    unsigned long long algo_valid_expansions_total;
    double valid_expansion_ratio;
    unsigned long long requests_created_total;
    unsigned long long request_wait_ticks_sum;
    int remaining_parked_vehicles;
};

#define grid_map_create Grid_create
#define grid_map_destroy Grid_destroy
#define grid_is_valid_coord Grid_isValidCoord
#define grid_is_node_blocked Grid_isNodeBlocked

void ensure_console_width(int minCols);

Simulation* simulation_create();
void simulation_destroy(Simulation* sim);
void simulation_run(Simulation* sim);
void simulation_print_performance_summary(const Simulation* sim);
static int simulation_is_complete(const Simulation* sim);
static void simulation_report_realtime_dashboard(Simulation* sim);
static void simulation_collect_memory_sample(Simulation* sim);
static void simulation_collect_memory_sample_algo(Simulation* sim);
static void simulation_reset_runtime_stats(Simulation* sim);
static int simulation_open_step_metrics_file(Simulation* sim, const char* path);
static void simulation_close_step_metrics_file(Simulation* sim);
static void simulation_append_step_metrics(Simulation* sim, int step_label);
static void ui_append_controls_help(char** p, size_t* rem);
static void ui_flush_display_buffer(void);
static void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag);
static void simulation_plan_step(Simulation* sim, Node* next_pos[MAX_AGENTS]);
static int apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]);
static void update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode);
static void accumulate_wait_ticks_if_realtime(Simulation* sim);
static void maybe_report_realtime_dashboard(Simulation* sim);
static Planner planner_from_pathalgo(PathAlgo algo);
static RendererFacade renderer_create_facade(void);
static void simulation_execute_one_step(Simulation* sim, int is_paused);
static void force_idle_cleanup(AgentManager* am, Simulation* sim, Logger* lg);
void agv_default_config(AgvSimulationConfig* cfg);
void agv_prepare_console(void);
int agv_apply_config(Simulation* sim, const AgvSimulationConfig* cfg);
int agv_execute_headless_step(Simulation* sim);
int agv_run_to_completion(Simulation* sim);
int agv_is_complete(const Simulation* sim);
void agv_collect_run_summary(const Simulation* sim, AgvRunSummary* out);
int agv_copy_render_frame(Simulation* sim, int is_paused, char* buffer, size_t buffer_size);
int agv_open_step_metrics(Simulation* sim, const char* path);
void agv_close_step_metrics(Simulation* sim);
int agv_write_run_summary_json(const Simulation* sim, const char* path);
void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg);
void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg);

void system_enable_virtual_terminal();
void ui_clear_screen_optimized();

int simulation_setup(Simulation* sim);
static int simulation_setup_map(Simulation* sim);
void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
static void map_build_hypermart(GridMap* m, AgentManager* am);
static void map_build_10agents_200slots(GridMap* m, AgentManager* am);
static void map_build_biggrid_onegoal(GridMap* m, AgentManager* am);
static void map_build_cross_4agents(GridMap* m, AgentManager* am);
static void grid_map_clear(GridMap* map);
static void map_all_free(GridMap* m);
static void map_add_border_walls(GridMap* m);
static void map_place_goal(GridMap* m, int x, int y);
static void map_place_charge(GridMap* m, int x, int y);
static void map_place_agent_at(AgentManager* am, GridMap* m, int idx, int x, int y);
static void map_reserve_area_as_start(GridMap* m, int x0, int y0, int w, int h);

void logger_log(Logger* logger, const char* format, ...);
Logger* logger_create();
void logger_destroy(Logger*);

GridMap* grid_map_create(AgentManager*);
void grid_map_destroy(GridMap*);
int grid_is_valid_coord(int x, int y);
int grid_is_node_blocked(const GridMap*, const AgentManager*, const Node*, const struct Agent_*);

ScenarioManager* scenario_manager_create();
void scenario_manager_destroy(ScenarioManager*);

AgentManager* agent_manager_create();
void agent_manager_destroy(AgentManager*);
void agent_manager_plan_and_resolve_collisions(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_update_state_after_move(AgentManager*, ScenarioManager*, GridMap*, Logger*, Simulation*);
void agent_manager_update_charge_state(AgentManager*, GridMap*, Logger*);
void agent_manager_plan_and_resolve_collisions_astar(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);
void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager*, GridMap*, Logger*, Node* next_pos[MAX_AGENTS]);

Pathfinder* pathfinder_create(Node* start, Node* goal, const struct Agent_* agent);
void pathfinder_destroy(Pathfinder* pf);
void pathfinder_reset_goal(Pathfinder* pf, Node* new_goal);
void pathfinder_update_start(Pathfinder* pf, Node* new_start);
void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed);
void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am);
Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current_node);

static void add_wait_edge(WaitEdge* edges, int* cnt, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2);
static int  build_scc_mask_from_edges(const WaitEdge* edges, int cnt);
static int  run_partial_CBS(AgentManager* m, GridMap* map, Logger* lg,
    int group_ids[], int group_n, const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]);
static void broadcast_cell_change(AgentManager* am, GridMap* map, Node* changed);
static void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg);
static char get_single_char();
static char get_char_input(const char* prompt, const char* valid);
static int  get_integer_input(const char* prompt, int min, int max);
static float get_float_input(const char* prompt, float min, float max);
static int check_for_input();

class PathfinderFactory final {
public:
    Pathfinder* create(Node* start, Node* goal) const { return pathfinder_create(start, goal, NULL); }
    void destroy(Pathfinder* pf) const { pathfinder_destroy(pf); }
};
static const PathfinderFactory g_pf_factory{};

static void resolve_conflicts_by_order(const AgentManager* m, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]);
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

AgentManager::AgentManager() {
    for (int i = 0; i < MAX_AGENTS; i++) {
        agents[i].id = i;
        agents[i].symbol = 'A' + i;
        agents[i].state = IDLE;
        agents[i].heading = DIR_NONE;
        agents[i].rotation_wait = 0;
        agents[i].action_timer = 0;
        agents[i].pf.reset();
        agents[i].stuck_steps = 0;
        agents[i].metrics_task_active = 0;
        agents[i].metrics_task_start_step = 0;
        agents[i].metrics_distance_at_start = 0.0;
        agents[i].metrics_turns_current = 0;
    }
}

void AgentManager::releasePathfinders() {
    for (int i = 0; i < MAX_AGENTS; i++) {
        agents[i].pf.reset();
    }
}

AgentManager::~AgentManager() {
    releasePathfinders();
}

ScenarioManager::ScenarioManager() = default;

void ScenarioManager::clearTaskQueue() {
    task_queue.clear();
    task_count = 0;
}

void ScenarioManager::enqueueTask(TaskType type) {
    if (task_count >= MAX_TASKS) return;
    task_queue.push_back(TaskNode{ type, time_step });
    task_count++;
}

bool ScenarioManager::tryDequeueAssignableTask(int lot_full, int parked_count, TaskType* out_type) {
    for (auto it = task_queue.begin(); it != task_queue.end(); ++it) {
        int can = FALSE;
        if (lot_full) {
            if (it->type == TASK_EXIT && parked_count > 0) can = TRUE;
        }
        else {
            if (it->type == TASK_PARK) can = TRUE;
            else if (it->type == TASK_EXIT && parked_count > 0) can = TRUE;
        }
        if (!can) continue;
        if (out_type) *out_type = it->type;
        task_queue.erase(it);
        task_count = static_cast<int>(task_queue.size());
        return TRUE;
    }
    return FALSE;
}

void ScenarioManager::applySpeedMultiplier(float speedMultiplier) {
    speed_multiplier = speedMultiplier;
    if (speedMultiplier <= 0.0f) {
        simulation_speed = 0;
        return;
    }
    simulation_speed = (int)(100.0f / speedMultiplier);
    if (simulation_speed < 0) simulation_speed = 0;
}

ScenarioManager::~ScenarioManager() {
    clearTaskQueue();
}

void Simulation_::destroyOwnedResources() {
    if (step_metrics_stream) {
        fclose(step_metrics_stream);
        step_metrics_stream = NULL;
    }
    step_metrics_header_written = FALSE;
    map.reset();
    agent_manager.reset();
    scenario_manager.reset();
    logger.reset();
}

Simulation_::~Simulation_() {
    destroyOwnedResources();
}


// =============================================================================

// =============================================================================


static inline void temp_unmark_all_and_notify(TempMarkList* l, Pathfinder* pf, GridMap* map, const AgentManager* am) {
    if (!l) return;
    for (int i = 0; i < l->count; i++) {
        Node* n = l->nodes[i];
        if (!n) continue;
        n->is_temp = FALSE;
        if (pf) pathfinder_notify_cell_change(pf, map, am, n);
    }
    l->count = 0;
}

struct TempMarkContext final {
    TempMarkList marks{};
    Pathfinder* pf{ nullptr };
    GridMap* map{ nullptr };
    AgentManager* am{ nullptr };
    int auto_notify{ FALSE };
};


static inline void temp_context_init(TempMarkContext* ctx, Pathfinder* pf, GridMap* map, AgentManager* am, int auto_notify) {
    if (!ctx) return;
    temp_mark_init(&ctx->marks);
    ctx->pf = pf;
    ctx->map = map;
    ctx->am = am;
    ctx->auto_notify = auto_notify;
}


static inline void temp_context_mark(TempMarkContext* ctx, Node* n) {
    if (!ctx || !n) return;
    temp_mark_node(&ctx->marks, n);
    if (ctx->auto_notify && ctx->pf) {
        pathfinder_notify_cell_change(ctx->pf, ctx->map, ctx->am, n);
    }
}


static inline void temp_context_cleanup(TempMarkContext* ctx) {
    if (!ctx) return;
    if (ctx->auto_notify) temp_unmark_all_and_notify(&ctx->marks, ctx->pf, ctx->map, ctx->am);
    else temp_unmark_all(&ctx->marks);
}

void system_enable_virtual_terminal() {
    // [Triple Defense for UTF-8 Korean Support]
    // 1. Set Windows API Output/Input Code Page to UTF-8
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
    // 2. Set C Library locale to UTF-8
    setlocale(LC_ALL, ".UTF8");
    // 3. Fallback: Shell command to ensure terminal is 65001
    (void)system("chcp 65001 > nul");

    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hOut == INVALID_HANDLE_VALUE) return;
    DWORD dwMode = 0;
    if (!GetConsoleMode(hOut, &dwMode)) return;
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
}


void ui_clear_screen_optimized() {
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    if (hConsole == INVALID_HANDLE_VALUE) return;

    DWORD mode = 0;
    if (GetConsoleMode(hConsole, &mode) && (mode & ENABLE_VIRTUAL_TERMINAL_PROCESSING)) {

        fputs("\x1b[H\x1b[2J\x1b[3J", stdout);
        fflush(stdout);
        return;
    }


    CONSOLE_SCREEN_BUFFER_INFO csbi;
    if (!GetConsoleScreenBufferInfo(hConsole, &csbi)) return;

    DWORD cellCount = (DWORD)csbi.dwSize.X * (DWORD)csbi.dwSize.Y;
    DWORD count;
    COORD home = { 0, 0 };

    FillConsoleOutputCharacterA(hConsole, ' ', cellCount, home, &count);
    FillConsoleOutputAttribute(hConsole, csbi.wAttributes, cellCount, home, &count);
    SetConsoleCursorPosition(hConsole, home);
}


void ensure_console_width(int minCols) {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h == INVALID_HANDLE_VALUE) return;

    CONSOLE_SCREEN_BUFFER_INFO csbi;
    if (!GetConsoleScreenBufferInfo(h, &csbi)) return;

    COORD size = csbi.dwSize;
    if (size.X < minCols) {
        size.X = (SHORT)minCols;

        SetConsoleScreenBufferSize(h, size);
    }
}


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

static void simulation_collect_memory_sample(Simulation* sim) {
    if (sim) sim->collectMemorySample();
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

static void simulation_collect_memory_sample_algo(Simulation* sim) {
    if (sim) sim->collectMemorySampleAlgo();
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

static void simulation_reset_runtime_stats(Simulation* sim) {
    if (sim) sim->resetRuntimeStats();
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

static void simulation_report_realtime_dashboard(Simulation* sim) {
    if (sim) sim->reportRealtimeDashboard();
}



static int check_for_input() {
    if (_kbhit()) {
        return _getch();
    }
    return 0;
}


struct ControlState final {
    int is_paused{ FALSE };
    int quit_flag{ FALSE };
    int last_key{ 0 };

    void reset() {
        is_paused = FALSE;
        quit_flag = FALSE;
        last_key = 0;
    }
};

static inline void ControlState_init(ControlState* cs) {
    if (!cs) return;
    cs->reset();
}


static char get_single_char() {
    return _getch();
}


static char get_char_input(const char* prompt, const char* valid) {
    char c;
    while (TRUE) {
        printf("%s", prompt);
        c = (char)tolower(get_single_char());
        printf("%c\n", c);
        if (strchr(valid, c)) return c;
        printf(C_B_RED "\nInvalid input. Allowed values: (%s)\n" C_NRM, valid);
    }
}


static int get_integer_input(const char* prompt, int min, int max) {
    char buf[INPUT_BUFFER_SIZE];
    int v;
    while (TRUE) {
        printf("%s", prompt);
        if (fgets(buf, sizeof(buf), stdin) && sscanf(buf, "%d", &v) == 1 && v >= min && v <= max) return v;
        printf(C_B_RED "Invalid input. Enter an integer in the range %d~%d.\n" C_NRM, min, max);
    }
}


static float get_float_input(const char* prompt, float min, float max) {
    char buf[INPUT_BUFFER_SIZE];
    float v;
    while (TRUE) {
        printf("%s", prompt);
        if (fgets(buf, sizeof(buf), stdin) && sscanf(buf, "%f", &v) == 1 && v >= min && v <= max) return v;
        printf(C_B_RED "Invalid input. Enter a value in the range %.1f~%.1f.\n" C_NRM, min, max);
    }
}

// ---- UI / Render ----
void ui_enter_alt_screen(void) {

    fputs("\x1b[?1049h\x1b[H\x1b[?25l", stdout);
    fflush(stdout);
}
void ui_leave_alt_screen(void) {

    fputs("\x1b[?1049l\x1b[?25h", stdout);
    fflush(stdout);
}

static void ui_append_controls_help(char** p, size_t* rem) {
    APPEND_FMT(*p, *rem, "%s--- Controls ---%s\n", C_B_WHT, C_NRM);
    APPEND_FMT(*p, *rem, "[%sP%s]ause/Resume | [%sS%s]tep | [%s+%s]/[%s-%s] Speed | ",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
    APPEND_FMT(*p, *rem, "[%s[%s]/[%s]%s Render stride | ", C_YEL, C_NRM, C_YEL, C_NRM);
    APPEND_FMT(*p, *rem, "[%sF%s]ast render | [%sC%s]olor simple | [%sQ%s]uit\n",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
}

static void ui_flush_display_buffer(void) {
    const char* display_buffer = display_buffer_state().data();
    size_t cur_len = strlen(display_buffer);
    if (!renderer_state().fast_render) ui_clear_screen_optimized(); else fputs("\x1b[H", stdout);
    fwrite(display_buffer, 1, cur_len, stdout);
    fflush(stdout);
}


static void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag) {
    switch (tolower(ch)) {
    case 'p':
        *is_paused = !*is_paused;
        logger_log(sim->logger, *is_paused ? "[CTRL] Simulation Paused." : "[CTRL] Simulation Resumed.");
        break;
    case 's':
        if (*is_paused) {
            logger_log(sim->logger, "[CTRL] Advancing one step.");
        }
        break;
    case '+':
    case '=':
        sim->scenario_manager->speed_multiplier += 0.5f;
        if (sim->scenario_manager->speed_multiplier > MAX_SPEED_MULTIPLIER)
            sim->scenario_manager->speed_multiplier = MAX_SPEED_MULTIPLIER;
        logger_log(sim->logger, "[CTRL] Speed increased to %.1fx", sim->scenario_manager->speed_multiplier);
        break;
    case '-':
        sim->scenario_manager->speed_multiplier -= 0.5f;
        if (sim->scenario_manager->speed_multiplier < 0.1f)
            sim->scenario_manager->speed_multiplier = 0.1f;
        logger_log(sim->logger, "[CTRL] Speed decreased to %.1fx", sim->scenario_manager->speed_multiplier);
        break;
    case 'q':
        *quit_flag = TRUE;
        logger_log(sim->logger, "[CTRL] Quit simulation.");
        break;
    case ']':
        if (renderer_state().render_stride < RENDER_STRIDE_MAX) renderer_state().render_stride <<= 1;
        if (renderer_state().render_stride < RENDER_STRIDE_MIN) renderer_state().render_stride = RENDER_STRIDE_MIN;
        logger_log(sim->logger, "[CTRL] Render stride = %d", renderer_state().render_stride);
        break;
    case '[':
        if (renderer_state().render_stride > RENDER_STRIDE_MIN) renderer_state().render_stride >>= 1;
        logger_log(sim->logger, "[CTRL] Render stride = %d", renderer_state().render_stride);
        break;
    case 'f':
        renderer_state().fast_render = !renderer_state().fast_render;
        logger_log(sim->logger, renderer_state().fast_render ? "[CTRL] Fast render ON" : "[CTRL] Fast render OFF");
        break;
    case 'c':
        renderer_state().simple_colors = !renderer_state().simple_colors;
        logger_log(sim->logger, renderer_state().simple_colors ? "[CTRL] Simple colors ON" : "[CTRL] Simple colors OFF");
        break;
    }


    if (ch == '+' || ch == '=' || ch == '-') {
        sim->scenario_manager->simulation_speed = (int)(100.0f / sim->scenario_manager->speed_multiplier);
        if (sim->scenario_manager->simulation_speed < 0) sim->scenario_manager->simulation_speed = 0;
    }
}


void Simulation_::planStep(Node* next_pos[MAX_AGENTS]) {
    SimulationContextGuard guard(this);
    Simulation* sim = this;
    clock_t plan_start_cpu = clock();

    // Reset per-step metrics
    sim->algo_nodes_expanded_last_step = 0;
    sim->algo_heap_moves_last_step = 0;
    sim->algo_generated_nodes_last_step = 0;
    sim->algo_valid_expansions_last_step = 0;
    planner_metrics_state().resetStepCounters();


    if (sim->agent_manager) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* ag = &sim->agent_manager->agents[i];
            if (ag->pf) {
                ag->pf->nodes_expanded_this_call = 0;
                ag->pf->heap_moves_this_call = 0;
                ag->pf->nodes_generated_this_call = 0;
                ag->pf->valid_expansions_this_call = 0;
            }
        }
    }

    sim->planner.planStep(sim->agent_manager, sim->map, sim->logger, next_pos);


    unsigned long long step_nodes = 0;
    unsigned long long step_heap_moves = 0;
    unsigned long long step_generated_nodes = 0;
    unsigned long long step_valid_expansions = 0;

    if (sim->path_algo == PATHALGO_ASTAR_SIMPLE) {
        // 2(A*)
        step_nodes = planner_metrics_state().astar_nodes_expanded_this_step;
        step_heap_moves = planner_metrics_state().astar_heap_moves_this_step;
        step_generated_nodes = planner_metrics_state().astar_generated_nodes_this_step;
        step_valid_expansions = planner_metrics_state().astar_valid_expansions_this_step;
    }
    else if (sim->path_algo == PATHALGO_DSTAR_BASIC) {
        // 3 (D* Lite)
        step_nodes = planner_metrics_state().dstar_nodes_expanded_this_step;
        step_heap_moves = planner_metrics_state().dstar_heap_moves_this_step;
        step_generated_nodes = planner_metrics_state().dstar_generated_nodes_this_step;
        step_valid_expansions = planner_metrics_state().dstar_valid_expansions_this_step;
    }
    else {
        // 1(WHCA*)
        // WHCA* D* Lite  + Partial CBS (WHCA*) 
        step_nodes = planner_metrics_state().whca_dstar_nodes_expanded_this_step + planner_metrics_state().whca_nodes_expanded_this_step;
        step_heap_moves = planner_metrics_state().whca_dstar_heap_moves_this_step + planner_metrics_state().whca_heap_moves_this_step;
        step_generated_nodes = planner_metrics_state().whca_dstar_generated_nodes_this_step + planner_metrics_state().whca_generated_nodes_this_step;
        step_valid_expansions = planner_metrics_state().whca_dstar_valid_expansions_this_step + planner_metrics_state().whca_valid_expansions_this_step;
    }

    sim->algo_nodes_expanded_last_step = step_nodes;
    sim->algo_heap_moves_last_step = step_heap_moves;
    sim->algo_generated_nodes_last_step = step_generated_nodes;
    sim->algo_valid_expansions_last_step = step_valid_expansions;
    sim->algo_nodes_expanded_total += step_nodes;
    sim->algo_heap_moves_total += step_heap_moves;
    sim->algo_generated_nodes_total += step_generated_nodes;
    sim->algo_valid_expansions_total += step_valid_expansions;

    clock_t plan_end_cpu = clock();
    double planning_time_ms = ((double)(plan_end_cpu - plan_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
    sim->last_planning_time_ms = planning_time_ms;
    sim->total_planning_time_ms += planning_time_ms;
    if (planning_time_ms > sim->max_planning_time_ms) sim->max_planning_time_ms = planning_time_ms;
    sim->algorithm_operation_count += (unsigned long long)((planner_metrics_state().wf_edges_last > 0 ? planner_metrics_state().wf_edges_last : 0) +
        (planner_metrics_state().scc_last > 0 ? planner_metrics_state().scc_last : 0) +
        (planner_metrics_state().cbs_exp_last > 0 ? planner_metrics_state().cbs_exp_last : 0));

    metrics_notify_all();
}

static void simulation_plan_step(Simulation* sim, Node* next_pos[MAX_AGENTS]) {
    if (sim) sim->planStep(next_pos);
}


static int apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]) {
    int moved_this_step = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &sim->agent_manager->agents[i];
        if (ag->state != CHARGING && next_pos[i]) {
            if (ag->pos != next_pos[i]) {
                ag->total_distance_traveled += 1.0;
                sim->total_movement_cost += 1.0;
                moved_this_step = 1;
            }
            ag->pos = next_pos[i];
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &sim->agent_manager->agents[i];
        if (ag->state == CHARGING || ag->state == IDLE || ag->action_timer > 0) { ag->stuck_steps = 0; continue; }
        if (ag->pos == prev_pos[i]) ag->stuck_steps++;
        else ag->stuck_steps = 0;
    }

    return moved_this_step;
}


static void update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode) {
    ScenarioManager* sc = sim->scenario_manager;
    if (moved_this_step) return;
    int unresolved = 0;
    if (is_custom_mode) {
        if (sc->current_phase_index < sc->num_phases) {
            const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
            if (sc->tasks_completed_in_phase < ph->task_count) unresolved = 1;
        }
    }
    else if (sc->mode == MODE_REALTIME) {
        if (sc->task_count > 0) unresolved = 1;
    }
    if (unresolved) sim->deadlock_count++;
}


static void accumulate_wait_ticks_if_realtime(Simulation* sim) {
    ScenarioManager* sc = sim->scenario_manager;
    if (sc->mode == MODE_REALTIME && sc->task_count > 0) {
        sim->request_wait_ticks_sum += static_cast<unsigned long long>(sc->task_queue.size());
    }
}


static void maybe_report_realtime_dashboard(Simulation* sim) {
    ScenarioManager* sc = sim->scenario_manager;
    sc->time_step++;
    if (sc->mode == MODE_REALTIME && (sc->time_step % DASHBOARD_INTERVAL_STEPS) == 0) {
        simulation_report_realtime_dashboard(sim);
    }
}


static int GridMap_renderToBuffer(char* buffer, size_t buffer_size,
    const GridMap* map, const AgentManager* am)
{
    static char view[GRID_HEIGHT][GRID_WIDTH];
    static const char* colors[GRID_HEIGHT][GRID_WIDTH];
    char* p = buffer;
    size_t rem = buffer_size;


    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            const Node* n = &map->grid[y][x];
            if (n->is_obstacle) { view[y][x] = '+'; colors[y][x] = C_WHT; }
            else { view[y][x] = '.'; colors[y][x] = C_GRY; }
        }
    }


    {
        int ncs = map->num_charge_stations;
        for (int i = 0; i < ncs; i++) {
            Node* cs = map->charge_stations[i];
            view[cs->y][cs->x] = 'e';
            if (!renderer_state().simple_colors) {
                int charging = FALSE;
                for (int j = 0; j < MAX_AGENTS; j++) {
                    if (am->agents[j].state == CHARGING && am->agents[j].pos == cs) { charging = TRUE; break; }
                }
                colors[cs->y][cs->x] = charging ? C_B_RED : C_B_YEL;
            }
        }
    }


    {
        int ng = map->num_goals;
        for (int i = 0; i < ng; i++) {
            Node* g = map->goals[i];
            if (g->is_parked) { view[g->y][g->x] = 'P'; if (!renderer_state().simple_colors) colors[g->y][g->x] = C_RED; }
            else if (g->is_goal) { view[g->y][g->x] = 'G'; if (!renderer_state().simple_colors) colors[g->y][g->x] = C_GRN; }
        }
    }


    for (int i = 0; i < MAX_AGENTS; i++) {
        if (am->agents[i].pos) {
            Node* n = am->agents[i].pos;
            view[n->y][n->x] = am->agents[i].symbol;
            colors[n->y][n->x] = AGENT_COLORS[i % 10];
        }
    }


    APPEND_FMT(p, rem, "%s\n--- D* Lite + WHCA* + WFG(SCC) + partial CBS ---%s\n", C_B_WHT, C_NRM);

    if (renderer_state().simple_colors) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            for (int x = 0; x < GRID_WIDTH; x++) {
                if (rem <= 1) break;
                *p++ = view[y][x]; rem--;
            }
            if (rem <= 1) break;
            *p++ = '\n'; rem--;
        }
    }
    else {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            for (int x = 0; x < GRID_WIDTH; x++) {
                APPEND_FMT(p, rem, "%s%c%s", colors[y][x], view[y][x], C_NRM);
            }
            APPEND_FMT(p, rem, "\n");
        }
    }
    APPEND_FMT(p, rem, "\n");

    return (int)(p - buffer);
}



static void simulation_display_status(Simulation* sim, int is_paused) {
    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    const GridMap* map = sim->map;
    const Logger* lg = sim->logger;
    const int display_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : sc->time_step;
    const double avg_cpu_ms = (display_steps > 0) ? (sim->total_cpu_time_ms / (double)display_steps) : 0.0;

    char* p = sim->display_buffer.data();
    size_t rem = sim->display_buffer.size();


    APPEND_FMT(p, rem, "%s", C_B_WHT);
    if (sc->mode == MODE_CUSTOM) {
        if (sc->current_phase_index < sc->num_phases) {
            const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
            APPEND_FMT(p, rem, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---  (Map #%d)",
                sc->current_phase_index + 1, sc->num_phases, sc->speed_multiplier, sim->map_id);
            if (is_paused) APPEND_FMT(p, rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
            APPEND_FMT(p, rem, "\n");

            APPEND_FMT(p, rem, "Time: %d, Current Task: %s (%d/%d)\n",
                sc->time_step, ph->type_name, sc->tasks_completed_in_phase, ph->task_count);
        }
        else {
            APPEND_FMT(p, rem, "--- Custom Scenario: All phases complete ---  (Map #%d)\n", sim->map_id);
        }
    }
    else if (sc->mode == MODE_REALTIME) {
        APPEND_FMT(p, rem, "--- Real-Time Simulation [Speed: %.1fx] ---  (Map #%d)",
            sc->speed_multiplier, sim->map_id);
        if (is_paused) APPEND_FMT(p, rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
        APPEND_FMT(p, rem, "\n");

        int park = 0, exitc = 0;
        for (const TaskNode& task : sc->task_queue) {
            if (task.type == TASK_PARK) park++;
            else if (task.type == TASK_EXIT) exitc++;
        }
        APPEND_FMT(p, rem, "Time: %d / %d | Pending Tasks: %d (%sPark: %d%s, %sExit: %d%s)\n",
            sc->time_step, REALTIME_MODE_TIMELIMIT, sc->task_count,
            C_B_GRN, park, C_NRM, C_B_YEL, exitc, C_NRM);
    }
    APPEND_FMT(p, rem, "Parked Cars: %d/%d\n%s", am->total_cars_parked, map->num_goals, C_NRM);
    APPEND_FMT(p, rem, "CPU Time (ms) - Last: %.3f | Avg: %.3f | Total: %.2f\n",
        sim->last_step_cpu_time_ms, avg_cpu_ms, sim->total_cpu_time_ms);


    {
        const char* algo = "Default (WHCA*+D*Lite+WFG+CBS)";
        if (sim->path_algo == PATHALGO_ASTAR_SIMPLE) algo = "A* (Single-Agent)";
        else if (sim->path_algo == PATHALGO_DSTAR_BASIC) algo = "D* Lite (Incremental)";
        APPEND_FMT(p, rem, "%sPath Algo:%s %s\n", C_B_WHT, C_NRM, algo);
    }

    APPEND_FMT(p, rem, "%sWHCA horizon:%s %d  | wf_edges(last): %d  | SCC(last): %d  | CBS(last): %s (exp:%d)\n",
        C_B_WHT, C_NRM, whca_horizon_state(), planner_metrics_state().wf_edges_last, planner_metrics_state().scc_last,
        planner_metrics_state().cbs_ok_last ? "OK" : "FAIL", planner_metrics_state().cbs_exp_last);


    {
        int w = GridMap_renderToBuffer(p, rem, map, am);
        if (w < 0) w = 0;
        if ((size_t)w >= rem) { p += rem - 1; rem = 1; }
        else { p += w; rem -= w; }
    }


    {
        static const char* stS[] = {
            "IDLE", "GOING_TO_PARK", "RETURN_HOME_EMPTY", "GOING_TO_COLLECT",
            "RETURN_WITH_CAR", "GO_TO_CHARGE", "CHARGING", "RETURN_HOME_MAINT"
        };
        static const char* stC[] = { C_GRY,C_YEL,C_CYN,C_YEL,C_GRN,C_B_RED,C_RED,C_CYN };

        for (int i = 0; i < MAX_AGENTS; i++) {
            const Agent* ag = &am->agents[i];
            const char* c = AGENT_COLORS[i % 10];

            char sbuf[100];
            if (ag->state == CHARGING)
                snprintf(sbuf, sizeof(sbuf), "CHARGING... (%d)", ag->charge_timer);
            else
                snprintf(sbuf, sizeof(sbuf), "%s", stS[ag->state]);

            APPEND_FMT(p, rem, "%sAgent %c%s: (%2d,%d) ",
                c, ag->symbol, C_NRM,
                ag->pos ? ag->pos->x : -1, ag->pos ? ag->pos->y : -1);

            if (ag->goal)
                APPEND_FMT(p, rem, "-> (%2d,%d) ", ag->goal->x, ag->goal->y);
            else
                APPEND_FMT(p, rem, "-> (none)          ");

            APPEND_FMT(p, rem, "[Mileage: %6.1f/%d] [%s%-*s%s]  [stuck:%d]\n",
                ag->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
                stC[ag->state], STATUS_STRING_WIDTH, sbuf, C_NRM, ag->stuck_steps);
        }
        APPEND_FMT(p, rem, "\n");
    }


    APPEND_FMT(p, rem, "%s--- Simulation Log ---%s\n", C_B_WHT, C_NRM);
    for (int i = 0; i < lg->log_count; i++) {
        int idx = (lg->log_head + i) % LOG_BUFFER_LINES;
        APPEND_FMT(p, rem, "%s%s%s\n", C_GRY, lg->logs[idx], C_NRM);
        if (rem < 512) break;
    }


    ui_append_controls_help(&p, &rem);


    static int s_frame_counter = 0;
    s_frame_counter++;
    if (!renderer_state().suppress_flush &&
        (s_frame_counter % (renderer_state().render_stride > 0 ? renderer_state().render_stride : 1)) == 0) {
        ui_flush_display_buffer();
    }
}

// =============================================================================
// 4.5) Renderer Facade Implementation (delegates to simulation_display_status)
// =============================================================================
class LegacyRendererStrategy final : public RendererStrategy {
public:
    void drawFrame(Simulation_* sim, int is_paused) const override {
        simulation_display_status(sim, is_paused);
    }

    std::unique_ptr<RendererStrategy> clone() const override {
        return std::make_unique<LegacyRendererStrategy>(*this);
    }
};

static RendererFacade renderer_create_facade(void) {
    return RendererFacade(std::make_unique<LegacyRendererStrategy>());
}


// Logger
Logger* logger_create() { return new Logger(); }
void logger_destroy(Logger* l) { delete l; }
void logger_log(Logger* l, const char* fmt, ...) {
    if (!l) return;
    va_list a;
    va_start(a, fmt);
    l->logV(fmt, a);
    va_end(a);
}

// OO-style aliases (kept as inline wrappers for readability; behavior unchanged)
#define Logger_log logger_log
#define Logger_create logger_create
#define Logger_destroy logger_destroy

// =============================================================================

// =============================================================================

static void map_all_free(GridMap* m) {
    grid_map_clear(m); // Initialize all cells to free space
}


static void map_add_border_walls(GridMap* m) {
    for (int x = 0; x < GRID_WIDTH; ++x) {
        m->grid[0][x].is_obstacle = TRUE;
        m->grid[GRID_HEIGHT - 1][x].is_obstacle = TRUE;
    }
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        m->grid[y][0].is_obstacle = TRUE;
        m->grid[y][GRID_WIDTH - 1].is_obstacle = TRUE;
    }
}


static void map_place_goal(GridMap* m, int x, int y) {
    if (!grid_is_valid_coord(x, y)) return;
    Node* n = &m->grid[y][x];
    if (!n->is_obstacle && !n->is_goal) {
        n->is_goal = TRUE;
        if (m->num_goals < MAX_GOALS) m->goals[m->num_goals++] = n;
    }
}

static void map_place_charge(GridMap* m, int x, int y) {
    if (!grid_is_valid_coord(x, y)) return;
    Node* n = &m->grid[y][x];
    if (!n->is_obstacle) {
        if (m->num_charge_stations < MAX_CHARGE_STATIONS)
            m->charge_stations[m->num_charge_stations++] = n;
    }
}

static void map_place_agent_at(AgentManager* am, GridMap* m, int idx, int x, int y) {
    if (idx < 0 || idx >= MAX_AGENTS) return;
    Node* n = &m->grid[y][x];
    am->agents[idx].pos = n;
    am->agents[idx].home_base = n;
    am->agents[idx].symbol = 'A' + idx; // A..J
    am->agents[idx].heading = DIR_NONE;
    am->agents[idx].rotation_wait = 0;
}

static void map_reserve_area_as_start(GridMap* m, int x0, int y0, int w, int h) {
    for (int y = y0; y < y0 + h && y < GRID_HEIGHT; ++y)
        for (int x = x0; x < x0 + w && x < GRID_WIDTH; ++x) {
            Node* n = &m->grid[y][x];
            n->is_obstacle = FALSE;
            n->is_goal = FALSE;
            n->is_temp = FALSE;
        }
}

static void agent_manager_reset_for_new_map(AgentManager* am) {
    if (!am) return;
    for (int i = 0; i < MAX_AGENTS; i++) {
        am->agents[i].pf.reset();
        am->agents[i].id = i;
        am->agents[i].symbol = 'A' + i;
        am->agents[i].pos = NULL;
        am->agents[i].home_base = NULL;
        am->agents[i].goal = NULL;
        am->agents[i].state = IDLE;
        am->agents[i].total_distance_traveled = 0.0;
        am->agents[i].charge_timer = 0;
        am->agents[i].action_timer = 0;
        am->agents[i].heading = DIR_NONE;
        am->agents[i].rotation_wait = 0;
        am->agents[i].stuck_steps = 0;
        am->agents[i].metrics_task_active = 0;
        am->agents[i].metrics_task_start_step = 0;
        am->agents[i].metrics_distance_at_start = 0.0;
        am->agents[i].metrics_turns_current = 0;
    }
    am->total_cars_parked = 0;
}

static void grid_map_clear(GridMap* map) {
    memset(map, 0, sizeof(*map));
    for (int y = 0; y < GRID_HEIGHT; ++y)
        for (int x = 0; x < GRID_WIDTH; ++x) {
            map->grid[y][x].x = x;
            map->grid[y][x].y = y;
            map->grid[y][x].is_obstacle = FALSE;
            map->grid[y][x].is_goal = FALSE;
            map->grid[y][x].is_temp = FALSE;
            map->grid[y][x].is_parked = FALSE;
            map->grid[y][x].reserved_by_agent = -1;
        }
    map->num_goals = 0;
    map->num_charge_stations = 0;
}

static void grid_map_fill_from_string(GridMap* map, AgentManager* am, const char* m) {
    grid_map_clear(map);

    int x = 0, y = 0;
    int last_was_cr = 0; 

    for (const char* p = m; *p && y < GRID_HEIGHT; ++p) {
        char ch = *p;

        
        if (ch == '\r') {
            x = 0; y++; last_was_cr = 1;
            continue;
        }
        if (ch == '\n') {
            if (!last_was_cr) { x = 0; y++; }
            last_was_cr = 0;
            continue;
        }
        last_was_cr = 0;

        
        if (x >= GRID_WIDTH) continue;
        if (y >= GRID_HEIGHT) break;

        Node* n = &map->grid[y][x];

        
        n->is_obstacle = FALSE;
        n->is_goal = FALSE;
        n->is_temp = FALSE;
        n->is_parked = FALSE;
        n->reserved_by_agent = -1;

        switch (ch) {
        case '1':  
            n->is_obstacle = TRUE;
            break;

        case 'A':  
            am->agents[0].pos = n;
            am->agents[0].home_base = n;
            break;

        case 'B':  
            am->agents[1].pos = n;
            am->agents[1].home_base = n;
            break;

        case 'C':  
            am->agents[2].pos = n;
            am->agents[2].home_base = n;
            break;

        case 'D':  
            am->agents[3].pos = n;
            am->agents[3].home_base = n;
            break;

        case 'G':  
            n->is_goal = TRUE;
            if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n;
            break;

        case 'e':  
            if (map->num_charge_stations < MAX_CHARGE_STATIONS)
                map->charge_stations[map->num_charge_stations++] = n;
            break;

        case '0':  
        default:
            
            break;
        }

        x++;
    }

    
}


void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id) {
    agent_manager_reset_for_new_map(am);
    grid_map_clear(map);

    switch (scenario_id) {
    case 1: {
        static const char* MAP1 =
            "1111111111111111111111111111111111111\n"
            "001GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n"
            "A000000000000000000000000000000000001\n"
            "B000000000000000000000000000000000001\n"
            "C001GG1GG1GGG10001GGG1GGG1GGG1100e111\n"
            "111111111111110001GGG1GGG1GGG11001111\n"
            "100000000000000000000000000000000e111\n"
            "100000000000000000000000000000000e111\n"
            "11111111111111GGG1GGG1GGG1GGG1GG11111\n"
            "1111111111111111111111111111111111111\n"
            "1111111111111111111111111111111111111\n"
            "1111111111111111111111111111111111111\n";
        grid_map_fill_from_string(map, am, MAP1);
        break;
    }
    case 2: map_build_hypermart(map, am);              break;
    case 3: map_build_10agents_200slots(map, am);      break;
    case 4: map_build_biggrid_onegoal(map, am);        break;
    case 5: map_build_cross_4agents(map, am);          break;
    default:
        map_build_hypermart(map, am); // fallback
        break;
    }
}

static void map_build_hypermart(GridMap* m, AgentManager* am) {
    int x, y;


    map_all_free(m);
    map_add_border_walls(m);


    for (y = 1; y < GRID_HEIGHT - 1; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            m->grid[y][x].is_obstacle = TRUE;
            m->grid[y][x].is_goal = FALSE;
        }


    map_reserve_area_as_start(m, 2, 2, 8, 5);
    map_place_agent_at(am, m, 0, 2, 2);
    map_place_agent_at(am, m, 1, 3, 2);
    map_place_agent_at(am, m, 2, 4, 2);
    map_place_agent_at(am, m, 3, 5, 2);


    for (x = 1; x < GRID_WIDTH - 1; ++x) m->grid[6][x].is_obstacle = FALSE;


    const int vCols[] = { 12, 22, 32, 42, 52, 62, 72 };
    const int nV = (int)(sizeof(vCols) / sizeof(vCols[0]));
    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][cx].is_obstacle = FALSE;
    }


    for (x = 1; x < GRID_WIDTH - 1; ++x) {
        m->grid[10][x].is_obstacle = FALSE;
        m->grid[30][x].is_obstacle = FALSE;
    }
    for (y = 19; y <= 21; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x)
            m->grid[y][x].is_obstacle = TRUE;


    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        for (y = 19; y <= 21; ++y) m->grid[y][cx].is_obstacle = FALSE;
    }

    m->grid[20][34].is_obstacle = FALSE;
    m->grid[20][50].is_obstacle = FALSE;


    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][4].is_obstacle = FALSE;
    for (x = 4; x <= 10; ++x) m->grid[6][x].is_obstacle = FALSE;


    const int pocketY[] = { 14, 16, 26, 28, 34 };
    const int nP = (int)(sizeof(pocketY) / sizeof(pocketY[0]));
    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        for (int k = 0; k < nP; ++k) {
            int py = pocketY[k];
            if (py >= 19 && py <= 21) continue;
            if (py == 10 || py == 30) continue;
            if (grid_is_valid_coord(cx - 1, py)) m->grid[py][cx - 1].is_obstacle = FALSE;
            if (grid_is_valid_coord(cx + 1, py)) m->grid[py][cx + 1].is_obstacle = FALSE;
        }
    }


    map_place_charge(m, 12, 8);
    map_place_charge(m, 42, 8);
    map_place_charge(m, 42, 32);
    map_place_charge(m, 72, 8);


    int markRow[GRID_HEIGHT] = { 0 };
    markRow[6] = 1;
    markRow[10] = 1;
    markRow[19] = 1; markRow[20] = 1; markRow[21] = 1;
    markRow[30] = 1;


    const int y_min = 8, y_max = GRID_HEIGHT - 4;
    for (int i = 0; i < nV; ++i) {
        int roadX = vCols[i];
        int leftCol = roadX - 1;
        int rightCol = roadX + 1;
        for (y = y_min; y <= y_max; ++y) {
            if (markRow[y]) continue;
            if (grid_is_valid_coord(leftCol, y)) { m->grid[y][leftCol].is_obstacle = FALSE; map_place_goal(m, leftCol, y); }
            if (grid_is_valid_coord(rightCol, y)) { m->grid[y][rightCol].is_obstacle = FALSE; map_place_goal(m, rightCol, y); }
        }
    }


    const int side_right_col = GRID_WIDTH - 4;
    const int side_right_road = GRID_WIDTH - 5;

    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][side_right_road].is_obstacle = FALSE;
    for (y = 19; y <= 21; ++y)            m->grid[y][side_right_road].is_obstacle = FALSE;

    for (y = y_min; y <= y_max; ++y) {
        if (markRow[y]) continue;
        if (grid_is_valid_coord(side_right_col, y)) {
            m->grid[y][side_right_col].is_obstacle = FALSE;
            map_place_goal(m, side_right_col, y);
        }
    }


    for (y = 2; y <= 8; ++y)
        for (x = 2; x <= 8; ++x)
            m->grid[y][x].is_goal = FALSE;
    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][4].is_goal = FALSE;


    m->num_goals = 0;
    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            Node* n = &m->grid[y][x];
            if (!n->is_goal) continue;
            int ok = 0;
            const int dx[4] = { 1,-1,0,0 }, dy[4] = { 0,0,1,-1 };
            for (int k = 0; k < 4; k++) {
                int nx = x + dx[k], ny = y + dy[k];
                if (!grid_is_valid_coord(nx, ny)) continue;
                if (!m->grid[ny][nx].is_obstacle) { ok = 1; break; }
            }
            if (!ok) n->is_goal = FALSE;
            if (n->is_goal && m->num_goals < MAX_GOALS) m->goals[m->num_goals++] = n;
        }
    }
}

// #3: 8 agents + 900 parking slots




static void map_build_10agents_200slots(GridMap* m, AgentManager* am) {
    int x, y;

    
    map_all_free(m);
    map_add_border_walls(m);

    
    for (y = 1; y < GRID_HEIGHT - 1; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            m->grid[y][x].is_obstacle = TRUE;
            m->grid[y][x].is_goal = FALSE;
        }

    
    const int sx0 = 2, sy0 = 2;
    const int sW = 16, sH = 6;
    map_reserve_area_as_start(m, sx0, sy0, sW, sH);

    
    const int staged_agents = (MAX_AGENTS < 16) ? MAX_AGENTS : 16;
    const int slots_per_row = 8;
    for (int i = 0; i < staged_agents; ++i) {
        int row = i / slots_per_row;
        int col = i % slots_per_row;
        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
    }

    
    const int lane_w = 1;
    const int y_min = 8;
    const int y_max = GRID_HEIGHT - 5;


    const int ax_start = 16;
    const int ax_end = GRID_WIDTH - 6;
    const int ax_step = 4;


    const int cross_start = 10;
    const int cross_end = GRID_HEIGHT - 6;
    const int cross_step = 6;

    
    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
        if (grid_is_valid_coord(2, y)) m->grid[y][2].is_obstacle = FALSE;
        if (grid_is_valid_coord(3, y)) m->grid[y][3].is_obstacle = FALSE;
    }

    
    for (x = 1; x < GRID_WIDTH - 1; ++x) {
        if (grid_is_valid_coord(x, 6)) m->grid[6][x].is_obstacle = FALSE;
        if (grid_is_valid_coord(x, 7)) m->grid[7][x].is_obstacle = FALSE;
    }

    
    for (x = ax_start; x <= ax_end; x += ax_step)
        for (y = 1; y < GRID_HEIGHT - 1; ++y)
            m->grid[y][x].is_obstacle = FALSE;

    
    for (y = cross_start; y <= cross_end; y += cross_step)
        for (x = 1; x < GRID_WIDTH - 1; ++x)
            m->grid[y][x].is_obstacle = FALSE;

    

    {
        int cxL = sx0;
        int cyT = sy0 + 1;
        map_place_charge(m, cxL, cyT);
        map_place_charge(m, cxL + 1, cyT);
        map_place_charge(m, cxL, cyT + 2);
        map_place_charge(m, cxL + 1, cyT + 2);
    }

    
    int markRow[GRID_HEIGHT] = { 0 };
    markRow[6] = 1; markRow[7] = 1;
    for (y = cross_start; y <= cross_end; y += cross_step)
        if (y >= 0 && y < GRID_HEIGHT) markRow[y] = 1;

    int markCol[GRID_WIDTH] = { 0 };
    markCol[2] = 1; markCol[3] = 1;
    for (x = ax_start; x <= ax_end; x += ax_step)
        if (x >= 0 && x < GRID_WIDTH) markCol[x] = 1;

    
    const int target = 900;
    int placed = 0;
    for (x = ax_start; x <= ax_end && placed < target; x += ax_step) {
        int leftCol = x - 1;
        int rightCol = x + lane_w; // x+1
        for (y = y_min; y <= y_max && placed < target; ++y) {
            if (markRow[y]) continue;

            if (grid_is_valid_coord(leftCol, y) && placed < target) {
                m->grid[y][leftCol].is_obstacle = FALSE;
                map_place_goal(m, leftCol, y);
                placed++;
            }
            if (grid_is_valid_coord(rightCol, y) && placed < target) {
                m->grid[y][rightCol].is_obstacle = FALSE;
                map_place_goal(m, rightCol, y);
                placed++;
            }
        }
    }

    
    if (placed < target) {
        for (y = cross_start; y <= cross_end && placed < target; y += cross_step) {
            int row = y;
            for (x = 2; x < GRID_WIDTH - 2 && placed < target; ++x) {
                if (markCol[x]) continue;
                if (m->grid[row][x].is_obstacle != FALSE) continue;

                if (grid_is_valid_coord(x, row - 1) &&
                    !m->grid[row - 1][x].is_goal && placed < target) {
                    m->grid[row - 1][x].is_obstacle = FALSE;
                    map_place_goal(m, x, row - 1);
                    placed++;
                }
                if (grid_is_valid_coord(x, row + 1) &&
                    !m->grid[row + 1][x].is_goal && placed < target) {
                    m->grid[row + 1][x].is_obstacle = FALSE;
                    map_place_goal(m, x, row + 1);
                    placed++;
                }
            }
        }
    }

    
    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
        m->grid[y][2].is_goal = FALSE;
        m->grid[y][3].is_goal = FALSE;
    }
}







static void carve_block_1lane(GridMap* m,
    int cx, int cy, int Wg, int Hg,
    int vstep, int hstep, int vx0, int hy0,
    int CX, int CY)
{
    int x, y;


    int gx0 = cx - (Wg / 2 - 1);
    int gx1 = gx0 + Wg - 1;
    int gy0 = cy - (Hg / 2);
    int gy1 = gy0 + Hg - 1;

    for (y = gy0; y <= gy1; ++y)
        for (x = gx0; x <= gx1; ++x) {
            if (!grid_is_valid_coord(x, y)) continue;
            m->grid[y][x].is_obstacle = FALSE;
            map_place_goal(m, x, y);
        }


    int rxL = gx0 - 1, rxR = gx1 + 1;
    int ryT = gy0 - 1, ryB = gy1 + 1;

    for (x = gx0 - 1; x <= gx1 + 1; ++x) {
        if (grid_is_valid_coord(x, ryT)) m->grid[ryT][x].is_obstacle = FALSE;
        if (grid_is_valid_coord(x, ryB)) m->grid[ryB][x].is_obstacle = FALSE;
    }
    for (y = gy0 - 1; y <= gy1 + 1; ++y) {
        if (grid_is_valid_coord(rxL, y)) m->grid[y][rxL].is_obstacle = FALSE;
        if (grid_is_valid_coord(rxR, y)) m->grid[y][rxR].is_obstacle = FALSE;
    }



    int kx = (cx - vx0 + vstep / 2) / vstep;
    int vx = vx0 + kx * vstep;
    if (vx < 1) vx = 1;
    if (vx > GRID_WIDTH - 2) vx = GRID_WIDTH - 2;

    int ky = (cy - hy0 + hstep / 2) / hstep;
    int hy = hy0 + ky * hstep;
    if (hy < 1) hy = 1;
    if (hy > GRID_HEIGHT - 2) hy = GRID_HEIGHT - 2;


    {
        int linkY = (abs(ryT - CY) <= abs(ryB - CY)) ? ryT : ryB;
        if (vx <= rxL) { for (x = vx; x <= rxL; ++x) m->grid[linkY][x].is_obstacle = FALSE; }
        else if (vx >= rxR) { for (x = rxR; x <= vx; ++x) m->grid[linkY][x].is_obstacle = FALSE; }
        else { m->grid[linkY][vx].is_obstacle = FALSE; }
    }

    {
        int linkX = (abs(rxL - CX) <= abs(rxR - CX)) ? rxL : rxR;
        if (hy <= ryT) { for (y = hy; y <= ryT; ++y) m->grid[y][linkX].is_obstacle = FALSE; }
        else if (hy >= ryB) { for (y = ryB; y <= hy; ++y) m->grid[y][linkX].is_obstacle = FALSE; }
        else { m->grid[hy][linkX].is_obstacle = FALSE; }
    }
}



static void map_build_biggrid_onegoal(GridMap* m, AgentManager* am) {
    map_all_free(m);
    map_add_border_walls(m);

    int x, y;


    for (y = 1; y < GRID_HEIGHT - 1; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            m->grid[y][x].is_obstacle = TRUE;
            m->grid[y][x].is_goal = FALSE;
        }


    const int CX = GRID_WIDTH / 2;
    const int CY = GRID_HEIGHT / 2;    // 21
    const int sW = 10, sH = 4;
    const int sx0 = CX - sW / 2;
    const int sy0 = CY - sH / 2;
    map_reserve_area_as_start(m, sx0, sy0, sW, sH);
    for (int i = 0; i < 10; ++i) {
        int row = i / 5, col = i % 5;
        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
    }


    const int vstep = 5, hstep = 5;
    const int vx0 = 6;
    const int hy0 = 9;


    for (x = vx0; x < GRID_WIDTH - 1; x += vstep)
        for (y = 1; y < GRID_HEIGHT - 1; ++y)
            m->grid[y][x].is_obstacle = FALSE;


    for (y = hy0; y < GRID_HEIGHT - 1; y += hstep)
        for (x = 1; x < GRID_WIDTH - 1; ++x)
            m->grid[y][x].is_obstacle = FALSE;


    for (x = 1; x < GRID_WIDTH - 1; ++x)
        m->grid[6][x].is_obstacle = FALSE;


    const int Wg = 6, Hg = 2;
    const int bxL = 8;
    const int bxR = GRID_WIDTH - 8;
    const int byT = 8;
    const int byB = GRID_HEIGHT - 8;


    carve_block_1lane(m, bxL, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);

    carve_block_1lane(m, bxR, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);

    carve_block_1lane(m, bxL, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);

    carve_block_1lane(m, bxR, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);


    if (grid_is_valid_coord(CX, CY - 6)) map_place_charge(m, CX, CY - 6);
    if (grid_is_valid_coord(CX, CY + 6)) map_place_charge(m, CX, CY + 6);
    if (grid_is_valid_coord(CX - 6, CY)) map_place_charge(m, CX - 6, CY);
    if (grid_is_valid_coord(CX + 6, CY)) map_place_charge(m, CX + 6, CY);


    m->num_goals = 0;
    for (y = 1; y < GRID_HEIGHT - 1; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x)
            if (m->grid[y][x].is_goal && m->num_goals < MAX_GOALS)
                m->goals[m->num_goals++] = &m->grid[y][x];
}


static void map_build_cross_4agents(GridMap* m, AgentManager* am) {
    int x, y;


    map_all_free(m);
    map_add_border_walls(m);


    for (y = 1; y < GRID_HEIGHT - 1; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            m->grid[y][x].is_obstacle = TRUE;
            m->grid[y][x].is_goal = FALSE;
        }

    const int CX = GRID_WIDTH / 2;
    const int CY = GRID_HEIGHT / 2;

    for (y = 1; y < GRID_HEIGHT - 1; ++y) m->grid[y][CX].is_obstacle = FALSE;
    for (x = 1; x < GRID_WIDTH - 1; ++x) m->grid[CY][x].is_obstacle = FALSE;


    map_place_agent_at(am, m, 0, 1, CY);
    map_place_agent_at(am, m, 1, GRID_WIDTH - 2, CY);
    map_place_agent_at(am, m, 2, CX, 1);
    map_place_agent_at(am, m, 3, CX, GRID_HEIGHT - 2);


    map_place_goal(m, 1 + 4, CY);
    map_place_goal(m, GRID_WIDTH - 2 - 4, CY);
    map_place_goal(m, CX, 1 + 4);
    map_place_goal(m, CX, GRID_HEIGHT - 2 - 4);


    map_place_charge(m, CX, CY);
}
GridMap* grid_map_create(AgentManager* am) {
    GridMap* m = new GridMap();
    grid_map_load_scenario(m, am, 1);
    return m;
}
void grid_map_destroy(GridMap* m) { delete m; }

int grid_is_valid_coord(int x, int y) { return (x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT); }

int grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* n, const struct Agent_* agent) {
    if (n->is_obstacle || n->is_parked || n->is_temp) return TRUE;


    if (agent && agent->state == RETURNING_HOME_EMPTY && n->is_goal && !n->is_parked) {
        return TRUE;
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        if (am->agents[i].pos == n && am->agents[i].state == CHARGING) return TRUE;
    }
    return FALSE;
}

// =============================================================================

// =============================================================================

static void scenario_manager_clear_task_queue(ScenarioManager* s) {
    if (!s) return;
    s->clearTaskQueue();
}

ScenarioManager* scenario_manager_create() {
    return new ScenarioManager();
}

void scenario_manager_destroy(ScenarioManager* s) { delete s; }


class AgentOps final {
public:
    void beginTaskPark(Agent* ag, ScenarioManager* sc, Logger* lg) const;
    void beginTaskExit(Agent* ag, ScenarioManager* sc, Logger* lg) const;
    void setGoalIfNeeded(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) const;
};

static void agent_set_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);

void AgentOps::beginTaskPark(Agent* ag, ScenarioManager* sc, Logger* lg) const {
    agent_begin_task_park(ag, sc, lg);
}

void AgentOps::beginTaskExit(Agent* ag, ScenarioManager* sc, Logger* lg) const {
    agent_begin_task_exit(ag, sc, lg);
}

void AgentOps::setGoalIfNeeded(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) const {
    if (ag->goal == NULL && ag->state != IDLE && ag->state != CHARGING) {
        agent_set_goal(ag, map, am, lg);
    }
}

static const AgentOps g_agent_ops{};

static void add_task_to_queue(ScenarioManager* s, TaskType t) {
    if (!s) return;
    s->enqueueTask(t);
}

// =============================================================================

// =============================================================================
struct _cmpKey { double k1; double k2; }; // same as Key


static int compare_keys(Key a, Key b) {
    if (a.k1 < b.k1 - 1e-9) return -1;
    if (a.k1 > b.k1 + 1e-9) return  1;
    if (a.k2 < b.k2 - 1e-9) return -1;
    if (a.k2 > b.k2 + 1e-9) return  1;
    return 0;
}

static inline SearchCell* cell_of(Pathfinder* pf, const Node* n) { return &pf->cells[n->y][n->x]; }

static inline Key key_of(Pathfinder* pf, const Node* n) { return cell_of(pf, n)->key; }


static void pq_init(NodePQ* pq, int cap) {
    (void)cap;
    pq->clear();
}

static void pq_free(NodePQ* pq) {
    if (!pq) return;
    pq->clear();
}


static void pq_swap(Pathfinder* pf, Node** a, Node** b) {
    Node* t = *a; *a = *b; *b = t;
    {
        int ia = cell_of(pf, *a)->pq_index;
        int ib = cell_of(pf, *b)->pq_index;
        cell_of(pf, *a)->pq_index = ib;
        cell_of(pf, *b)->pq_index = ia;
    }
}

static void heapify_up(Pathfinder* pf, NodePQ* pq, int i) {
    if (i == 0) return;
    int p = (i - 1) / 2;
    if (compare_keys(key_of(pf, pq->nodes[i]), key_of(pf, pq->nodes[p])) < 0) {
        pq_swap(pf, &pq->nodes[i], &pq->nodes[p]);
        if (pf) pf->heap_moves_this_call++;
        heapify_up(pf, pq, p);
    }
}

static void heapify_down(Pathfinder* pf, NodePQ* pq, int i) {
    int l = 2 * i + 1, r = 2 * i + 2, s = i;
    if (l < pq->size && compare_keys(key_of(pf, pq->nodes[l]), key_of(pf, pq->nodes[s])) < 0) s = l;
    if (r < pq->size && compare_keys(key_of(pf, pq->nodes[r]), key_of(pf, pq->nodes[s])) < 0) s = r;
    if (s != i) {
        pq_swap(pf, &pq->nodes[i], &pq->nodes[s]);
        if (pf) pf->heap_moves_this_call++;
        heapify_down(pf, pq, s);
    }
}

static int pq_contains(Pathfinder* pf, const Node* n) { return cell_of(pf, n)->in_pq; }

static Key pq_top_key(Pathfinder* pf, const NodePQ* pq) {
    if (pq->size == 0) return make_key(INF, INF);
    return key_of(pf, pq->nodes[0]);
}

static void pq_push(Pathfinder* pf, NodePQ* pq, Node* n) {
    if (pq->size >= pq->capacity()) return;
    if (pf) pf->nodes_generated_this_call++;
    SearchCell* c = cell_of(pf, n);
    c->in_pq = TRUE; c->pq_index = pq->size; pq->nodes[pq->size++] = n;
    heapify_up(pf, pq, pq->size - 1);
}

static Node* pq_pop(Pathfinder* pf, NodePQ* pq) {
    if (pq->size == 0) return NULL;
    Node* top = pq->nodes[0];
    SearchCell* ct = cell_of(pf, top); ct->in_pq = FALSE; ct->pq_index = -1;
    pq->size--;
    if (pq->size > 0) {
        pq->nodes[0] = pq->nodes[pq->size];
        cell_of(pf, pq->nodes[0])->pq_index = 0;
        heapify_down(pf, pq, 0);
    }
    return top;
}

static void pq_remove(Pathfinder* pf, NodePQ* pq, Node* n) {
    SearchCell* c = cell_of(pf, n); if (!c->in_pq) return;
    int idx = c->pq_index; pq->size--;
    if (idx != pq->size) {
        pq->nodes[idx] = pq->nodes[pq->size];
        cell_of(pf, pq->nodes[idx])->pq_index = idx;
        int parent = (idx - 1) / 2;
        if (idx > 0 && compare_keys(key_of(pf, pq->nodes[idx]), key_of(pf, pq->nodes[parent])) < 0)
            heapify_up(pf, pq, idx);
        else heapify_down(pf, pq, idx);
    }
    c->in_pq = FALSE; c->pq_index = -1;
}

// =============================================================================

// =============================================================================

static double heuristic(const Node* a, const Node* b) {
    return manhattan_nodes(a, b);
}

static Key calculate_key(Pathfinder* pf, const Node* n) {
    SearchCell* c = cell_of(pf, n);
    double m = fmin(c->g, c->rhs);
    return make_key(m + heuristic(pf->start_node, n) + pf->km, m);
}

static void updateVertex(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* u) {
    SearchCell* cu = cell_of(pf, u);
    if (u != pf->goal_node) {
        double min_rhs = INF;
        for (int i = 0; i < DIR4_COUNT; i++) {
            int nx = u->x + DIR4_X[i], ny = u->y + DIR4_Y[i];
            if (!grid_is_valid_coord(nx, ny)) continue;
            Node* s = &map->grid[ny][nx];
            if (!grid_is_node_blocked(map, am, s, pf->agent)) {
                double gsucc = cell_of(pf, s)->g;
                double cand = 1.0 + gsucc;
                if (cand < min_rhs) min_rhs = cand;
            }
        }
        cu->rhs = min_rhs;
    }
    if (pq_contains(pf, u)) pq_remove(pf, &pf->pq, u);
    if (fabs(cu->g - cu->rhs) > 1e-9) {
        cu->key = calculate_key(pf, u);
        pq_push(pf, &pf->pq, u);
    }
}

Pathfinder* pathfinder_create(Node* start, Node* goal, const Agent* agent) {
    Pathfinder* pf = new Pathfinder();
    pq_init(&pf->pq, GRID_WIDTH * GRID_HEIGHT);
    pf->start_node = start; pf->last_start = start; pf->goal_node = goal; pf->km = 0.0;
    pf->agent = agent;
    pf->nodes_expanded_this_call = 0;
    pf->heap_moves_this_call = 0;
    pf->nodes_generated_this_call = 0;
    pf->valid_expansions_this_call = 0;

    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++) {
            pf->cells[y][x].g = INF; pf->cells[y][x].rhs = INF;
            pf->cells[y][x].in_pq = FALSE; pf->cells[y][x].pq_index = -1;
            pf->cells[y][x].key = make_key(INF, INF);
        }

    if (goal) {
        SearchCell* cg = &pf->cells[goal->y][goal->x];
        cg->rhs = 0.0; cg->key = calculate_key(pf, goal);
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
    pf->goal_node = new_goal; pf->km = 0.0; pf->last_start = pf->start_node;
    pf->pq.size = 0;
    for (int y = 0; y < GRID_HEIGHT; y++)
        for (int x = 0; x < GRID_WIDTH; x++) {
            pf->cells[y][x].g = INF; pf->cells[y][x].rhs = INF;
            pf->cells[y][x].in_pq = FALSE; pf->cells[y][x].pq_index = -1;
            pf->cells[y][x].key = make_key(INF, INF);
        }
    if (new_goal) {
        pf->cells[new_goal->y][new_goal->x].rhs = 0.0;
        pf->cells[new_goal->y][new_goal->x].key = calculate_key(pf, new_goal);
        pq_push(pf, &pf->pq, new_goal);
    }
}

void pathfinder_update_start(Pathfinder* pf, Node* new_start) {
    if (new_start == NULL) return;
    if (pf->start_node == NULL) { pf->start_node = new_start; pf->last_start = new_start; return; }
    pf->km += heuristic(pf->last_start, new_start);
    pf->last_start = new_start;
    pf->start_node = new_start;
}

void pathfinder_notify_cell_change(Pathfinder* pf, GridMap* map, const AgentManager* am, Node* changed) {
    updateVertex(pf, map, am, changed);
    for (int i = 0; i < DIR4_COUNT; i++) {
        int px = changed->x + DIR4_X[i], py = changed->y + DIR4_Y[i];
        if (grid_is_valid_coord(px, py)) updateVertex(pf, map, am, &map->grid[py][px]);
    }
}

void pathfinder_compute_shortest_path(Pathfinder* pf, GridMap* map, const AgentManager* am) {
    if (!pf->start_node || !pf->goal_node) return;


    pf->nodes_expanded_this_call = 0;
    pf->heap_moves_this_call = 0;

    while (TRUE) {
        Key top = pq_top_key(pf, &pf->pq);
        SearchCell* cs = cell_of(pf, pf->start_node);
        Key kstart = calculate_key(pf, pf->start_node);

        if (pf->pq.size == 0 || (compare_keys(top, kstart) >= 0 && fabs(cs->rhs - cs->g) < 1e-9)) break;

        Key k_old = top;
        Node* u = pq_pop(pf, &pf->pq);
        if (u) pf->nodes_expanded_this_call++;
        SearchCell* cu = cell_of(pf, u);
        Key k_new = calculate_key(pf, u);

        if (compare_keys(k_old, k_new) < 0) {
            cu->key = k_new;
            pq_push(pf, &pf->pq, u);
        }
        else if (cu->g > cu->rhs) {
            cu->g = cu->rhs;
            pf->valid_expansions_this_call++;
            for (int i = 0; i < DIR4_COUNT; i++) {
                int px = u->x + DIR4_X[i], py = u->y + DIR4_Y[i];
                if (grid_is_valid_coord(px, py))
                    updateVertex(pf, map, am, &map->grid[py][px]);
            }
        }
        else {
            cu->g = INF;
            updateVertex(pf, map, am, u);
            for (int i = 0; i < DIR4_COUNT; i++) {
                int px = u->x + DIR4_X[i], py = u->y + DIR4_Y[i];
                if (grid_is_valid_coord(px, py))
                    updateVertex(pf, map, am, &map->grid[py][px]);
            }
        }
    }
}

Node* pathfinder_get_next_step(Pathfinder* pf, const GridMap* map, const AgentManager* am, Node* current) {
    if (!pf->goal_node || !current) return current;
    SearchCell* cc = cell_of(pf, current);
    if (cc->g >= INF || current == pf->goal_node) return current;

    double best = INF; Node* bestn = current;
    double tie_euclid = fabs((double)current->x - (double)pf->goal_node->x) + fabs((double)current->y - (double)pf->goal_node->y);

    for (int i = 0; i < DIR4_COUNT; i++) {
        int nx = current->x + DIR4_X[i], ny = current->y + DIR4_Y[i];
        if (!grid_is_valid_coord(nx, ny)) continue;
        Node* nb = &((GridMap*)map)->grid[ny][nx];
        if (grid_is_node_blocked(map, am, nb, pf->agent)) continue;
        double gsucc = cell_of(pf, nb)->g;
        double cost = 1.0 + gsucc;
        if (cost < best) {
            best = cost; bestn = nb;
            tie_euclid = manhattan_nodes(nb, pf->goal_node);
        }
        else if (fabs(cost - best) < 1e-9) {
            double d = manhattan_nodes(nb, pf->goal_node);
            if (d < tie_euclid) { bestn = nb; tie_euclid = d; }
        }
    }
    return bestn;
}

// =============================================================================

// =============================================================================
static void ReservationTable_clear(ReservationTable* r) {
    for (int t = 0; t <= MAX_WHCA_HORIZON; t++)
        for (int y = 0; y < GRID_HEIGHT; y++)
            for (int x = 0; x < GRID_WIDTH; x++)
                r->occ[t][y][x] = -1;
}

static void ReservationTable_seedCurrent(ReservationTable* r, AgentManager* m) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &m->agents[i];
        if (ag->pos && ag->state != CHARGING) r->occ[0][ag->pos->y][ag->pos->x] = ag->id;
    }
}

static int ReservationTable_isOccupied(const ReservationTable* r, int t, const Node* n) {
    if (t < 0 || t > whca_horizon_state()) return TRUE;
    return r->occ[t][n->y][n->x] != -1;
}

static int ReservationTable_getOccupant(const ReservationTable* r, int t, const Node* n) {
    if (t < 0 || t > whca_horizon_state()) return -1;
    return r->occ[t][n->y][n->x];
}

static void ReservationTable_setOccupant(ReservationTable* r, int t, const Node* n, int agent_id) {
    if (t < 0 || t > whca_horizon_state()) return;
    r->occ[t][n->y][n->x] = agent_id;
}

AgentManager* agent_manager_create() {
    
    return new AgentManager();
}

void agent_manager_destroy(AgentManager* m) {
    delete m;
}

// =============================================================================
// Agent OO-like wrappers (semantics-preserving; use existing logging/metrics)
// =============================================================================
void agent_begin_task_park(Agent* ag, ScenarioManager* sc, Logger* lg) {
    
    if (!ag || !sc) return;
    ag->state = GOING_TO_PARK;
    ag->metrics_task_active = 1;
    ag->metrics_task_start_step = sc->time_step;
    ag->metrics_distance_at_start = ag->total_distance_traveled;
    ag->metrics_turns_current = 0;
    if (lg) logger_log(lg, "[%sTask%s] Agent %c assigned to a parking task.", C_CYN, C_NRM, ag->symbol);
}
void agent_begin_task_exit(Agent* ag, ScenarioManager* sc, Logger* lg) {
    
    if (!ag || !sc) return;
    ag->state = GOING_TO_COLLECT;
    ag->metrics_task_active = 1;
    ag->metrics_task_start_step = sc->time_step;
    ag->metrics_distance_at_start = ag->total_distance_traveled;
    ag->metrics_turns_current = 0;
    if (lg) logger_log(lg, "[%sTask%s] Agent %c assigned to a retrieval task.", C_CYN, C_NRM, ag->symbol);
}

static double calculate_path_cost_tempPF(Agent* ag, Node* goal, GridMap* map, AgentManager* am)
{
    

    if (!ag || !ag->pos || !goal || !map || !am) return INF;


    if (ag->pos == goal) return 0.0;


    if (goal->is_obstacle) return INF;

    Pathfinder* pf = pathfinder_create(ag->pos, goal, ag);
    if (!pf) return INF;

    pathfinder_compute_shortest_path(pf, map, am);
    double cost = cell_of(pf, ag->pos)->g;

    pathfinder_destroy(pf);


    if (cost >= INF * 0.5) return INF;
    return cost;
}










static Node* select_best_from_list(Agent* ag, GridMap* map, AgentManager* am,
    Node** list, int count, int require_parked, int check_reserved, int toggle_parked_during_eval, double* out_best_cost)
{
    
    double best = INF; Node* bestn = NULL;
    for (int j = 0; j < count; j++) {
        Node* n = list[j]; if (!n) continue;
        if (require_parked == 1 && !n->is_parked) continue;
        if (require_parked == 0 && n->is_parked) continue;
        if (check_reserved && (n->reserved_by_agent != -1 && n->reserved_by_agent != ag->id)) continue;

        int restored = 0;
        if (toggle_parked_during_eval && n->is_parked) { n->is_parked = FALSE; restored = 1; }
        double c = calculate_path_cost_tempPF(ag, n, map, am);
        if (restored) n->is_parked = TRUE;

        if (c < best) { best = c; bestn = n; }
    }
    if (out_best_cost) *out_best_cost = best;
    return bestn;
}


typedef enum { GOAL_PARKING, GOAL_PARKED_CAR, GOAL_CHARGE } GoalType;

static Node* select_best_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg, GoalType type, double* out_cost) {
    
    Node** list = NULL; int count = 0; int require_parked = -1; int check_reserved = 1; int toggle_parked = 0;
    switch (type) {
    case GOAL_PARKING:
        list = map->goals; count = map->num_goals; require_parked = 0; toggle_parked = 0; break;
    case GOAL_PARKED_CAR:
        list = map->goals; count = map->num_goals; require_parked = 1; toggle_parked = 1; break;
    case GOAL_CHARGE:
        list = map->charge_stations; count = map->num_charge_stations; require_parked = -1; toggle_parked = 0; break;
    }
    return select_best_from_list(ag, map, am, list, count, require_parked, check_reserved, toggle_parked, out_cost);
}

static Node* select_best_parking_spot(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
    
    double best_cost = INF;
    Node* bestg = select_best_goal(ag, map, am, lg, GOAL_PARKING, &best_cost);
    if (bestg) logger_log(lg, "[%sPlan%s] Agent %c selected parking goal (%d,%d) (cost %.1f)",
        C_CYN, C_NRM, ag->symbol, bestg->x, bestg->y, best_cost);
    return bestg;
}
static Node* select_best_parked_car(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
    
    double best_cost = INF;
    Node* bests = select_best_goal(ag, map, am, lg, GOAL_PARKED_CAR, &best_cost);
    if (bests) logger_log(lg, "[%sPlan%s] Agent %c selected retrieval target (%d,%d) (cost %.1f)",
        C_CYN, C_NRM, ag->symbol, bests->x, bests->y, best_cost);
    return bests;
}
static Node* select_best_charge_station(Agent* ag, GridMap* map, AgentManager* am, Logger* lg) {
    
    double best_cost = INF;
    Node* bests = select_best_goal(ag, map, am, lg, GOAL_CHARGE, &best_cost);
    if (bests) logger_log(lg, "[%sPlan%s] Agent %c selected charge station (%d,%d) (cost: %.1f)",
        C_CYN, C_NRM, ag->symbol, bests->x, bests->y, best_cost);
    return bests;
}
static void ensure_pathfinder_for_agent(Agent* ag) {
    
    if (!ag->goal) return;
    if (ag->pf == NULL) {
        ag->pf = pathfinder_create(ag->pos, ag->goal, ag);
    }
    else if (ag->pf->goal_node != ag->goal) {
        ag->pf->start_node = ag->pos;
        pathfinder_reset_goal(ag->pf, ag->goal);
    }
}

// priority score
static int priority_score(const Agent* ag) {
    
    int imp = 0;
    if (ag->state == RETURNING_WITH_CAR) imp = PRIORITY_RETURNING_WITH_CAR;
    else if (ag->state == GOING_TO_CHARGE) imp = PRIORITY_GOING_TO_CHARGE;
    else if (ag->state == GOING_TO_PARK || ag->state == GOING_TO_COLLECT) imp = PRIORITY_MOVING_TASK;

    {
        int stuck_boost = (ag->stuck_steps >= DEADLOCK_THRESHOLD) ? STUCK_BOOST_HARD : (ag->stuck_steps * STUCK_BOOST_MULT);
        return imp * 100 + stuck_boost - ag->id;
    }
}
static void sort_agents_by_priority(AgentManager* m, int order[MAX_AGENTS]) {
    
    for (int i = 0; i < MAX_AGENTS; i++) order[i] = i;
    for (int i = 0; i < MAX_AGENTS; i++)
        for (int j = i + 1; j < MAX_AGENTS; j++)
            if (priority_score(&m->agents[order[j]]) > priority_score(&m->agents[order[i]])) {
                int t = order[i]; order[i] = order[j]; order[j] = t;
            }
}

static AgentWorkloadSnapshot collect_agent_workload(const AgentManager* am) {
    AgentWorkloadSnapshot snapshot{};
    if (!am) return snapshot;

    for (int i = 0; i < MAX_AGENTS; ++i) {
        const Agent* ag = &am->agents[i];
        switch (ag->state) {
        case GOING_TO_PARK:
        case RETURNING_HOME_EMPTY:
            snapshot.active_park_agents++;
            break;
        case GOING_TO_COLLECT:
        case RETURNING_WITH_CAR:
            snapshot.active_exit_agents++;
            break;
        default:
            break;
        }
    }

    return snapshot;
}
static void agent_set_goal(Agent* ag, GridMap* map, AgentManager* am, Logger* lg)
{
    

    if (!ag || !ag->pos) {
        ag->goal = NULL;
        ag->state = IDLE;
        return;
    }


    if (ag->state == RETURNING_HOME_EMPTY &&
        ag->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
        if (ag->goal) { ag->goal->reserved_by_agent = -1; ag->goal = NULL; }
        logger_log(lg, "[%sCharge%s] Agent %c exceeded the mileage threshold while returning home. Switching to charge mode.", C_B_YEL, C_NRM, ag->symbol);
        ag->state = GOING_TO_CHARGE;
    }


    if (ag->state == IDLE || ag->state == CHARGING || ag->goal) return;


    Node* new_goal = NULL;
    switch (ag->state) {
    case GOING_TO_PARK:
        new_goal = select_best_parking_spot(ag, map, am, lg);
        break;
    case RETURNING_HOME_EMPTY:
    case RETURNING_WITH_CAR:
    case RETURNING_HOME_MAINTENANCE:
        new_goal = ag->home_base;
        break;
    case GOING_TO_COLLECT:
        new_goal = select_best_parked_car(ag, map, am, lg);
        break;
    case GOING_TO_CHARGE:
        new_goal = select_best_charge_station(ag, map, am, lg);
        break;
    default:
        break;
    }


    if (new_goal) {

        if (ag->goal && ag->goal != new_goal) ag->goal->reserved_by_agent = -1;

        ag->goal = new_goal;
        ag->goal->reserved_by_agent = ag->id;
    }
    else {

        if (ag->state == RETURNING_HOME_EMPTY ||
            ag->state == RETURNING_WITH_CAR ||
            ag->state == RETURNING_HOME_MAINTENANCE) {
            if (!ag->home_base) {
                ag->state = IDLE;
                logger_log(lg, "[%sWarn%s] Agent %c: no home position is configured. Switching to IDLE.", C_B_RED, C_NRM, ag->symbol);
            }
        }
        else {
            ag->state = IDLE;
            logger_log(lg, "[%sInfo%s] Agent %c: no valid goal found. Waiting.", C_YEL, C_NRM, ag->symbol);
        }
    }
}

static int best_candidate_order(Pathfinder* pf, const GridMap* map, const AgentManager* am,
    Node* cur, Node* goal, Node* out[5], int* outN) {
    
    struct Cand { Node* n; double cost; double d; };
    Cand cands[5]; int cn = 0;

    double gcur = cell_of(pf, cur)->g;
    cands[cn++] = (Cand){ cur, gcur + 1e-6, 1e18 };

    for (int k = 0; k < DIR4_COUNT; k++) {
        int nx = cur->x + DIR4_X[k], ny = cur->y + DIR4_Y[k];
        if (!grid_is_valid_coord(nx, ny)) continue;
        Node* nb = &((GridMap*)map)->grid[ny][nx];
        if (grid_is_node_blocked(map, am, nb, pf->agent)) continue;
        double gsucc = cell_of(pf, nb)->g;
        double cost = 1.0 + gsucc;
        double d = manhattan_nodes(nb, goal);
        cands[cn++] = (Cand){ nb, cost, d };
    }
    for (int a = 0; a < cn; a++) for (int b = a + 1; b < cn; b++) {
        if (cands[b].cost < cands[a].cost || (fabs(cands[b].cost - cands[a].cost) < 1e-9 && cands[b].d < cands[a].d)) {
            Cand t = cands[a]; cands[a] = cands[b]; cands[b] = t;
        }
    }
    for (int i = 0; i < cn; i++) out[i] = cands[i].n;
    *outN = cn;
    return cn;
}

// ---- WFG helpers ----
static void add_wait_edge(WaitEdge* edges, int* cnt, int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2) {
    
    if (*cnt >= MAX_WAIT_EDGES) return;
    edges[*cnt].from_id = from; edges[*cnt].to_id = to; edges[*cnt].t = t;
    edges[*cnt].cause = cause; edges[*cnt].x1 = x1; edges[*cnt].y1 = y1; edges[*cnt].x2 = x2; edges[*cnt].y2 = y2;
    (*cnt)++;
}
static int build_scc_mask_from_edges(const WaitEdge* edges, int cnt) {
    
    int adj[MAX_AGENTS][MAX_AGENTS] = { 0 };
    for (int i = 0; i < cnt; i++) {
        int u = edges[i].from_id, v = edges[i].to_id;
        if (u >= 0 && v >= 0 && u != v) adj[u][v] = 1;
    }
    int reach[MAX_AGENTS][MAX_AGENTS] = { 0 };
    for (int i = 0; i < MAX_AGENTS; i++)
        for (int j = 0; j < MAX_AGENTS; j++) reach[i][j] = adj[i][j];
    for (int k = 0; k < MAX_AGENTS; k++)
        for (int i = 0; i < MAX_AGENTS; i++)
            for (int j = 0; j < MAX_AGENTS; j++)
                reach[i][j] = reach[i][j] || (reach[i][k] && reach[k][j]);

    {
        int mask = 0;
        for (int i = 0; i < MAX_AGENTS; i++)
            for (int j = 0; j < MAX_AGENTS; j++)
                if (i != j && reach[i][j] && reach[j][i]) { mask |= (1 << i); mask |= (1 << j); }
        return mask;
    }
}


static Node* try_pull_over(const GridMap* map, const ReservationTable* rt, Agent* ag) {
    
    for (int k = 0; k < DIR5_COUNT; k++) {
        int nx = ag->pos->x + DIR5_X[k], ny = ag->pos->y + DIR5_Y[k];
        if (!grid_is_valid_coord(nx, ny)) continue;
        Node* nb = (Node*)&map->grid[ny][nx];
        if (nb == ag->pos) { if (!ReservationTable_isOccupied(rt, 1, nb)) return nb; else continue; }
        if (nb->is_obstacle || nb->is_parked) continue;

        if (nb->reserved_by_agent != -1 && nb->reserved_by_agent != ag->id) continue;
        if (!ReservationTable_isOccupied(rt, 1, nb)) return nb;
    }
    return ag->pos;
}

// priority helper
static int best_in_mask(const AgentManager* m, int mask) {
    
    int best = -1, bestScore = -999999;
    for (int i = 0; i < MAX_AGENTS; i++) if (mask & (1 << i)) {
        int sc = priority_score(&m->agents[i]);
        if (sc > bestScore) { bestScore = sc; best = i; }
    }
    return best;
}

// ---- Partial-team CBS: low-level ST-A* ----
static double G_buf[MAX_TOT], F_buf[MAX_TOT];
static unsigned char OPEN_buf[MAX_TOT], CLOSED_buf[MAX_TOT];
static int PREV_buf[MAX_TOT];
static int HEAP_NODE_buf[MAX_TOT];
static int HEAP_POS_buf[MAX_TOT];

static int heap_prefer(double* fvals, int a, int b) {
    double fa = fvals[a];
    double fb = fvals[b];
    if (fa < fb - 1e-9) return 1;
    if (fa > fb + 1e-9) return 0;
    return a < b;
}

static void heap_swap(int* nodes, int* pos, int i, int j, unsigned long long* swap_counter) {
    if (i == j) return;
    int ni = nodes[i];
    int nj = nodes[j];
    nodes[i] = nj;
    nodes[j] = ni;
    pos[nj] = i;
    pos[ni] = j;
    if (swap_counter) (*swap_counter)++;
}

static void heap_sift_up(int* nodes, int* pos, double* fvals, int idx, unsigned long long* swap_counter) {
    while (idx > 0) {
        int parent = (idx - 1) >> 1;
        if (!heap_prefer(fvals, nodes[idx], nodes[parent])) break;
        heap_swap(nodes, pos, idx, parent, swap_counter);
        idx = parent;
    }
}

static void heap_sift_down(int* nodes, int* pos, double* fvals, int size, int idx, unsigned long long* swap_counter) {
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

static void heap_push(int* nodes, int* pos, double* fvals, int* size, int node, unsigned long long* swap_counter) {
    nodes[*size] = node;
    pos[node] = *size;
    (*size)++;
    heap_sift_up(nodes, pos, fvals, (*size) - 1, swap_counter);
}

static int heap_pop(int* nodes, int* pos, double* fvals, int* size, unsigned long long* swap_counter) {
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

static void heap_decrease_key(int* nodes, int* pos, double* fvals, int node, unsigned long long* swap_counter) {
    int idx = pos[node];
    if (idx >= 0) {
        heap_sift_up(nodes, pos, fvals, idx, swap_counter);
    }
}

static int violates_constraint_for(int agent, int t_prev, int x_prev, int y_prev, int x_new, int y_new,
    const CBSConstraint* cons, int ncons) {
    
    for (int i = 0; i < ncons; i++) {
        if (cons[i].agent != agent) continue;
        if (cons[i].is_edge) {
            if (cons[i].t == t_prev && cons[i].x == x_prev && cons[i].y == y_prev &&
                cons[i].tox == x_new && cons[i].toy == y_new) return 1;
        }
        else {
            if (cons[i].t == (t_prev + 1) && cons[i].x == x_new && cons[i].y == y_new) return 1;
        }
    }
    return 0;
}
static int st_astar_plan_single(int agent_id, GridMap* map, Node* start, Node* goal, int horizon,
    int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH],
    const CBSConstraint* cons, int ncons,
    Node* out_plan[MAX_WHCA_HORIZON + 1],
    AgentDir initial_heading,
    unsigned long long* out_nodes_expanded,
    unsigned long long* out_heap_moves,
    unsigned long long* out_generated_nodes,
    unsigned long long* out_valid_expansions) {
    
    if (!start) return 0;
    int T = horizon;
    int W = GRID_WIDTH, H = GRID_HEIGHT;
    int TOT = (T + 1) * W * H;
    if (out_nodes_expanded) *out_nodes_expanded = 0;
    if (out_heap_moves) *out_heap_moves = 0;
    if (out_generated_nodes) *out_generated_nodes = 0;
    if (out_valid_expansions) *out_valid_expansions = 0;
    if (TOT > MAX_TOT) return 0;

    double* g = G_buf;
    double* f = F_buf;
    unsigned char* open = OPEN_buf;
    unsigned char* closed = CLOSED_buf;
    int* prev = PREV_buf;
    int* heap_nodes = HEAP_NODE_buf;
    int* heap_pos = HEAP_POS_buf;
    int heap_size = 0;

    for (int i = 0; i < TOT; i++) {
        g[i] = INF;
        f[i] = INF;
        open[i] = 0;
        closed[i] = 0;
        prev[i] = -1;
        heap_pos[i] = -1;
    }

#ifndef ST_INDEX
#define ST_INDEX(t,y,x,width,height) ((t)*(width)*(height) + (y)*(width) + (x))
#endif

    int sx = start->x, sy = start->y;
    int gx = goal ? goal->x : sx, gy = goal ? goal->y : sy;

    int start_idx = ST_INDEX(0, sy, sx, W, H);
    g[start_idx] = 0.0;
    f[start_idx] = goal ? manhattan_xy(sx, sy, gx, gy) : 0.0;
    open[start_idx] = 1;

    int best_idx = start_idx; double best_val = f[start_idx];
    unsigned long long nodes_expanded = 0;
    unsigned long long heap_moves = 0;
    unsigned long long generated_nodes = 0;
    unsigned long long valid_expansions = 0;

    heap_push(heap_nodes, heap_pos, f, &heap_size, start_idx, &heap_moves);

    while (heap_size > 0) {
        int cur = heap_pop(heap_nodes, heap_pos, f, &heap_size, &heap_moves);
        double curF = f[cur];
        open[cur] = 0;
        closed[cur] = 1;
        nodes_expanded++;

        int ct = cur / (W * H);
        int rem = cur % (W * H);
        int cy = rem / W, cx = rem % W;

        if (goal && cx == gx && cy == gy) {
            best_idx = cur; break;
        }
        if (f[cur] < best_val) { best_val = f[cur]; best_idx = cur; }

        if (ct == T) continue;

        for (int k = 0; k < DIR5_COUNT; k++) {
            int nx = cx + DIR5_X[k], ny = cy + DIR5_Y[k];
            int nt = ct + 1;
            if (!grid_is_valid_coord(nx, ny)) continue;

            if (ext_occ[nt][ny][nx] != -1) continue;

            if (ext_occ[ct][ny][nx] != -1 && ext_occ[nt][cy][cx] == ext_occ[ct][ny][nx]) continue;

            Node* ncell = &map->grid[ny][nx];
            if (ncell->is_obstacle) continue;

            if (violates_constraint_for(agent_id, ct, cx, cy, nx, ny, cons, ncons)) continue;

            int nid = ST_INDEX(nt, ny, nx, W, H);
            if (closed[nid]) continue;
            generated_nodes++;
            {
                double ng = g[cur] + 1.0;

                if (ct == 0 && !(nx == cx && ny == cy)) {
                    AgentDir move_heading = dir_from_delta(nx - cx, ny - cy);
                    if (initial_heading != DIR_NONE) {
                        int tsteps = dir_turn_steps(initial_heading, move_heading);
                        if (tsteps == 1) {
                            ng += (double)TURN_90_WAIT;
                        }
                    }
                }
                if (ng + 1e-9 < g[nid]) {
                    g[nid] = ng;
                    double h = goal ? manhattan_xy(nx, ny, gx, gy) : 0.0;
                    f[nid] = ng + h;
                    prev[nid] = cur;
                    if (!open[nid]) {
                        open[nid] = 1;
                        heap_push(heap_nodes, heap_pos, f, &heap_size, nid, &heap_moves);
                    }
                    else {
                        heap_decrease_key(heap_nodes, heap_pos, f, nid, &heap_moves);
                    }
                    valid_expansions++;
                }
            }
        }
    }

    int path_idx[MAX_WHCA_HORIZON + 1]; int plen = 0;
    {
        int cur = best_idx;
        while (cur != -1 && plen < (MAX_WHCA_HORIZON + 1)) { path_idx[plen++] = cur; cur = prev[cur]; }
    }
    if (out_nodes_expanded) *out_nodes_expanded = nodes_expanded;
    if (out_heap_moves) *out_heap_moves = heap_moves;
    if (out_generated_nodes) *out_generated_nodes = generated_nodes;
    if (out_valid_expansions) *out_valid_expansions = valid_expansions;

    if (plen == 0) {
        for (int t = 0; t <= T; t++) out_plan[t] = start;
        return 1;
    }
    for (int t = 0; t < plen; t++) {
        int idx = path_idx[plen - 1 - t];
        int tt = idx / (W * H);
        int rem = idx % (W * H);
        int y = rem / W, x = rem % W;
        if (tt <= T) out_plan[tt] = &map->grid[y][x];
    }
    {
        Node* last = out_plan[plen - 1 <= T ? plen - 1 : T];
        for (int t = plen; t <= T; t++) out_plan[t] = last;
    }

    return 1;
}

// ---- Partial-team CBS high-level ----
struct CBSConflict {
    int a, b;
    int t;
    int is_edge;
    int ax, ay, bx, by;
    int apx, apy, bpx, bpy;
};

static double cbs_cost_sum_adv(int ids[], int n,
    Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1],
    Node* goals[MAX_AGENTS], int horizon) {
    const double ALPHA = 1.0, BETA = 0.5, GAMMA = 0.1;
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        int id = ids[i];
        int moves = 0, waits = 0;
        for (int t = 1; t <= horizon; t++) {
            if (plans[id][t] != plans[id][t - 1]) moves++;
            else waits++;
        }
        {
            double hres = 0.0;
            if (goals[id]) {
                Node* last = plans[id][horizon];
                hres = manhattan_nodes(last, goals[id]);
            }
            s += ALPHA * moves + BETA * waits + GAMMA * hres;
        }
    }
    return s;
}
static int detect_first_conflict(Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1], int ids[], int n, CBSConflict* out, int horizon) {
    for (int t = 1; t <= horizon; t++) {
        for (int i = 0; i < n; i++) for (int j = i + 1; j < n; j++) {
            int a = ids[i], b = ids[j];
            Node* a_t = plans[a][t];
            Node* b_t = plans[b][t];
            Node* a_tm1 = plans[a][t - 1];
            Node* b_tm1 = plans[b][t - 1];
            if (a_t == b_t) {
                out->a = a; out->b = b; out->t = t; out->is_edge = 0;
                out->ax = a_t->x; out->ay = a_t->y; out->bx = b_t->x; out->by = b_t->y;
                out->apx = a_tm1->x; out->apy = a_tm1->y; out->bpx = b_tm1->x; out->bpy = b_tm1->y;
                return 1;
            }
            if (a_t == b_tm1 && b_t == a_tm1) {
                out->a = a; out->b = b; out->t = t; out->is_edge = 1;
                out->ax = a_tm1->x; out->ay = a_tm1->y; out->bx = b_tm1->x; out->by = b_tm1->y;
                out->apx = a_tm1->x; out->apy = a_tm1->y; out->bpx = b_tm1->x; out->bpy = b_tm1->y;
                return 1;
            }
        }
    }
    return 0;
}
static void copy_ext_occ_without_group(const ReservationTable* base, int group_mask,
    int out_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]) {
    for (int t = 0; t <= whca_horizon_state(); t++)
        for (int y = 0; y < GRID_HEIGHT; y++)
            for (int x = 0; x < GRID_WIDTH; x++) {
                int who = base->occ[t][y][x];
                if (who != -1 && (group_mask & (1 << who))) out_occ[t][y][x] = -1;
                else out_occ[t][y][x] = who;
            }
}


static void cbs_heap_push(CBSNode* heap, int* hsize, const CBSNode* node) {
    
    if (*hsize >= MAX_CBS_NODES) return;
    heap[*hsize] = *node;
    int i = *hsize;
    (*hsize)++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].cost <= heap[i].cost) break;
        CBSNode tmp = heap[p]; heap[p] = heap[i]; heap[i] = tmp;
        i = p;
    }
}
static CBSNode cbs_heap_pop(CBSNode* heap, int* hsize) {
    
    CBSNode ret = heap[0];
    *hsize = *hsize - 1;
    heap[0] = heap[*hsize];
    int i = 0;
    while (1) {
        int l = 2 * i + 1, r = 2 * i + 2, s = i;
        if (l < *hsize && heap[l].cost < heap[s].cost) s = l;
        if (r < *hsize && heap[r].cost < heap[s].cost) s = r;
        if (s == i) break;
        CBSNode tmp = heap[s]; heap[s] = heap[i]; heap[i] = tmp;
        i = s;
    }
    return ret;
}

// Partial CBS
static int run_partial_CBS(AgentManager* m, GridMap* map, Logger* lg,
    int group_ids[], int group_n, const ReservationTable* base_rt,
    Node* out_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]) {
    
    if (group_n <= 1) return 0;

    int group_mask = 0; for (int i = 0; i < group_n; i++) group_mask |= (1 << group_ids[i]);

    // Move large temporaries out of the stack to avoid stack overflow (0xC00000FD)
    static int ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH];
    copy_ext_occ_without_group(base_rt, group_mask, ext_occ);

    static CBSNode heap[MAX_CBS_NODES]; int hsize = 0; int expansions = 0;

    CBSNode root; memset(&root, 0, sizeof(root));
    for (int i = 0; i < group_n; i++) {
        int id = group_ids[i];
        Node* plan[MAX_WHCA_HORIZON + 1];
        unsigned long long nodes_exp = 0;
        unsigned long long heap_moves = 0;
        unsigned long long generated_nodes = 0;
        unsigned long long valid_expansions = 0;
        if (!st_astar_plan_single(id, map, m->agents[id].pos, m->agents[id].goal, whca_horizon_state(), ext_occ,
            root.cons, root.ncons, plan, m->agents[id].heading, &nodes_exp, &heap_moves, &generated_nodes, &valid_expansions)) {
            planner_metrics_state().cbs_ok_last = 0; planner_metrics_state().cbs_exp_last = expansions; planner_metrics_state().cbs_fail_sum++;
            return 0;
        }
        planner_metrics_state().whca_nodes_expanded_this_step += nodes_exp;
        planner_metrics_state().whca_heap_moves_this_step += heap_moves;
        planner_metrics_state().whca_generated_nodes_this_step += generated_nodes;
        planner_metrics_state().whca_valid_expansions_this_step += valid_expansions;
        for (int t = 0; t <= whca_horizon_state(); t++) root.plans[id][t] = plan[t];
    }
    {
        Node* goals[MAX_AGENTS] = { 0 };
        for (int i = 0; i < group_n; i++) goals[group_ids[i]] = m->agents[group_ids[i]].goal;
        root.cost = cbs_cost_sum_adv(group_ids, group_n, root.plans, goals, whca_horizon_state());
    }
    cbs_heap_push(heap, &hsize, &root);

    while (hsize > 0 && expansions < CBS_MAX_EXPANSIONS) {
        CBSNode cur = cbs_heap_pop(heap, &hsize); expansions++;
        if (expansions > CBS_MAX_EXPANSIONS) break; // safety guard

        CBSConflict conf;
        if (!detect_first_conflict(cur.plans, group_ids, group_n, &conf, whca_horizon_state())) {
            for (int i = 0; i < group_n; i++) {
                int id = group_ids[i];
                for (int t = 0; t <= whca_horizon_state(); t++) out_plans[id][t] = cur.plans[id][t];
            }
            logger_log(lg, "[%sCBS%s] Partial CBS succeeded (group=%d agents, expansions=%d).", C_B_GRN, C_NRM, group_n, expansions);
            planner_metrics_state().cbs_ok_last = 1; planner_metrics_state().cbs_exp_last = expansions; planner_metrics_state().cbs_success_sum++;
            return 1;
        }

        for (int branch = 0; branch < 2; branch++) {
            if (hsize >= MAX_CBS_NODES) break;
            CBSNode child = cur; // copy by value (heap local); keep heap size bounded
            if (child.ncons >= MAX_CBS_CONS) continue;

            CBSConstraint c; memset(&c, 0, sizeof(c));
            if (branch == 0) c.agent = conf.a; else c.agent = conf.b;
            if (conf.is_edge) {
                c.is_edge = 1; c.t = conf.t - 1;
                if (branch == 0) { c.x = conf.apx; c.y = conf.apy; c.tox = conf.bpx; c.toy = conf.bpy; }
                else { c.x = conf.bpx; c.y = conf.bpy; c.tox = conf.apx; c.toy = conf.apy; }
            }
            else {
                c.is_edge = 0; c.t = conf.t;
                c.x = conf.ax; c.y = conf.ay;
            }
            child.cons[child.ncons++] = c;

            {
                int ok = 1;
                for (int i = 0; i < group_n; i++) {
                    int id = group_ids[i];
                    Node* plan[MAX_WHCA_HORIZON + 1];
                    unsigned long long nodes_exp = 0;
                    unsigned long long heap_moves = 0;
                    unsigned long long generated_nodes = 0;
                    unsigned long long valid_expansions = 0;
                    if (!st_astar_plan_single(id, map, m->agents[id].pos, m->agents[id].goal, whca_horizon_state(), ext_occ,
                        child.cons, child.ncons, plan, m->agents[id].heading, &nodes_exp, &heap_moves, &generated_nodes, &valid_expansions)) {
                        ok = 0; break;
                    }
                    planner_metrics_state().whca_nodes_expanded_this_step += nodes_exp;
                    planner_metrics_state().whca_heap_moves_this_step += heap_moves;
                    planner_metrics_state().whca_generated_nodes_this_step += generated_nodes;
                    planner_metrics_state().whca_valid_expansions_this_step += valid_expansions;
                    for (int t = 0; t <= whca_horizon_state(); t++) child.plans[id][t] = plan[t];
                }
                if (!ok) continue;
            }

            {
                Node* goals[MAX_AGENTS] = { 0 };
                for (int i = 0; i < group_n; i++) goals[group_ids[i]] = m->agents[group_ids[i]].goal;
                child.cost = cbs_cost_sum_adv(group_ids, group_n, child.plans, goals, whca_horizon_state());
            }

            cbs_heap_push(heap, &hsize, &child);
        }
    }
    logger_log(lg, "[%sCBS%s] Partial CBS failed within the search budget. Falling back to pull-over.", C_B_RED, C_NRM);
    planner_metrics_state().cbs_ok_last = 0; planner_metrics_state().cbs_exp_last = expansions; planner_metrics_state().cbs_fail_sum++;
    return 0;
}

void agent_manager_plan_and_resolve_collisions(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
    
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &m->agents[i];
        g_agent_ops.setGoalIfNeeded(ag, map, m, lg);
    }
    for (int i = 0; i < MAX_AGENTS; i++) next_pos[i] = m->agents[i].pos;

    int order[MAX_AGENTS]; sort_agents_by_priority(m, order);

    ReservationTable rt; ReservationTable_clear(&rt); ReservationTable_seedCurrent(&rt, m);
    WaitEdge wf_edges[MAX_WAIT_EDGES]; int wf_cnt = 0;

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        Agent* ag = &m->agents[i];
        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;


        if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) {
            for (int kk = 1; kk <= whca_horizon_state(); kk++) {
                ReservationTable_setOccupant(&rt, kk, ag->pos, ag->id);
            }
            next_pos[ag->id] = ag->pos;
            continue;
        }

        ensure_pathfinder_for_agent(ag);

        {
            int goal_was_parked = (ag->state == GOING_TO_COLLECT && ag->goal->is_parked);
            if (goal_was_parked) ag->goal->is_parked = FALSE;

            if (ag->pf) {
                pathfinder_update_start(ag->pf, ag->pos);
                pathfinder_compute_shortest_path(ag->pf, map, m);

                planner_metrics_state().whca_dstar_nodes_expanded_this_step += ag->pf->nodes_expanded_this_call;
                planner_metrics_state().whca_dstar_heap_moves_this_step += ag->pf->heap_moves_this_call;
                planner_metrics_state().whca_dstar_generated_nodes_this_step += ag->pf->nodes_generated_this_call;
                planner_metrics_state().whca_dstar_valid_expansions_this_step += ag->pf->valid_expansions_this_call;
            }

            Node* plan[MAX_WHCA_HORIZON + 1]; plan[0] = ag->pos;
            Node* cur = ag->pos;

            for (int k = 1; k <= whca_horizon_state(); k++) {
                Node* cand[5]; int cn = 0;
                best_candidate_order(ag->pf, map, m, cur, ag->pf->goal_node, cand, &cn);

                Node* chosen = cur;

                for (int ci = 0; ci < cn; ci++) {
                    Node* nb = cand[ci];


                    if (ReservationTable_isOccupied(&rt, k, nb)) {
                        int who = ReservationTable_getOccupant(&rt, k, nb);
                        if (who != -1) add_wait_edge(wf_edges, &wf_cnt, ag->id, who, k, CAUSE_VERTEX, nb->x, nb->y, 0, 0);
                        continue;
                    }

                    {
                        int who_prev = ReservationTable_getOccupant(&rt, k - 1, nb);
                        int who_into_cur = ReservationTable_getOccupant(&rt, k, cur);
                        if (who_prev != -1 && who_prev == who_into_cur) {
                            add_wait_edge(wf_edges, &wf_cnt, ag->id, who_prev, k, CAUSE_SWAP, cur->x, cur->y, nb->x, nb->y);
                            continue;
                        }
                    }


                    if (chosen == cur) {
                        chosen = nb;
                    }
                }

                plan[k] = chosen;
                ReservationTable_setOccupant(&rt, k, chosen, ag->id);
                cur = chosen;

                if (cur == ag->goal) {
                    for (int kk = k + 1; kk <= whca_horizon_state(); kk++) ReservationTable_setOccupant(&rt, kk, cur, ag->id), plan[kk] = cur;
                    break;
                }
            }
            next_pos[ag->id] = plan[1];

            if (goal_was_parked) ag->goal->is_parked = TRUE;
        }
    }

    {

        for (int i = 0; i < MAX_AGENTS; i++) {
            if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == NULL) continue;
            for (int j = i + 1; j < MAX_AGENTS; j++) {
                if (m->agents[j].state == IDLE || m->agents[j].state == CHARGING || m->agents[j].goal == NULL) continue;
                if (!next_pos[i] || !next_pos[j]) continue;

                if (next_pos[i] == next_pos[j]) {
                    add_wait_edge(wf_edges, &wf_cnt, i, j, 1, CAUSE_VERTEX, next_pos[i]->x, next_pos[i]->y, 0, 0);
                    add_wait_edge(wf_edges, &wf_cnt, j, i, 1, CAUSE_VERTEX, next_pos[j]->x, next_pos[j]->y, 0, 0);
                }

                else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {
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

        int sccMask = build_scc_mask_from_edges(wf_edges, wf_cnt);
        planner_metrics_state().wf_edges_last = wf_cnt; planner_metrics_state().wf_edges_sum += wf_cnt;
        planner_metrics_state().scc_last = (sccMask ? 1 : 0); planner_metrics_state().scc_sum += (sccMask ? 1 : 0);

        if (sccMask) {
            int group_ids[MAX_CBS_GROUP]; int group_n = 0;
            for (int i = 0; i < MAX_AGENTS && group_n < MAX_CBS_GROUP; i++) {
                if ((sccMask & (1 << i)) == 0) continue;
                if (m->agents[i].state == IDLE || m->agents[i].state == CHARGING || m->agents[i].goal == NULL) continue;
                if (m->agents[i].action_timer > 0 && m->agents[i].pos && m->agents[i].goal && m->agents[i].pos == m->agents[i].goal) continue;
                group_ids[group_n++] = i;
            }
            if (group_n >= 2) {
                Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = { {0} };
                int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
                if (ok) {
                    for (int gi = 0; gi < group_n; gi++) {
                        int id = group_ids[gi];
                        if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
                    }
                }
                else {
                    int leader = best_in_mask(m, sccMask);
                    for (int gi = 0; gi < group_n; gi++) {
                        int id = group_ids[gi];
                        if (id == leader) continue;
                        Node* po = try_pull_over(map, &rt, &m->agents[id]);
                        if (po) next_pos[id] = po;
                    }
                    logger_log(lg, "[%sWFG%s] SCC detected: leader=%c commits, others pull over.", C_B_YEL, C_NRM, m->agents[leader].symbol);
                }
            }
        }
        else {

            int active_ids[MAX_AGENTS]; int active_n = 0;
            for (int i = 0; i < MAX_AGENTS; i++) {
                Agent* ag = &m->agents[i];
                if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;
                active_ids[active_n++] = i;
            }
            int all_wait = 1;
            for (int ai = 0; ai < active_n; ai++) {
                int id = active_ids[ai];
                if (next_pos[id] != m->agents[id].pos) { all_wait = 0; break; }
            }
            if (all_wait && active_n >= 2) {
                int order[MAX_AGENTS]; sort_agents_by_priority(m, order);
                int group_ids[MAX_CBS_GROUP]; int group_n = 0;
                for (int oi = 0; oi < MAX_AGENTS && group_n < MAX_CBS_GROUP; oi++) {
                    int id = order[oi];
                    Agent* ag = &m->agents[id];
                    if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL) continue;
                    if (ag->action_timer > 0 && ag->pos && ag->goal && ag->pos == ag->goal) continue;
                    group_ids[group_n++] = id;
                }
                if (group_n >= 2) {
                    Node* cbs_plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1] = { {0} };
                    int ok = run_partial_CBS(m, map, lg, group_ids, group_n, &rt, cbs_plans);
                    if (ok) {
                        for (int gi = 0; gi < group_n; gi++) {
                            int id = group_ids[gi];
                            if (cbs_plans[id][1]) next_pos[id] = cbs_plans[id][1];
                        }
                        logger_log(lg, "[%sCBS%s] Deadlock fallback CBS engaged (group=%d).", C_B_CYN, C_NRM, group_n);
                    }
                    else {

                        int leader = best_in_mask(m, sccMask ? sccMask : 0x3FF);
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
                    }
                    else {
                        logger_log(lg, "[%sAvoid%s] Vertex conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    }
                }
                else {
                    int pi = priority_score(&m->agents[i]);
                    int pj = priority_score(&m->agents[j]);
                    if (pi >= pj) { logger_log(lg, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[j].symbol); next_pos[j] = m->agents[j].pos; }
                    else { logger_log(lg, "[%sAvoid%s] Vertex conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[i].symbol); next_pos[i] = m->agents[i].pos; }
                }
            }
            else if (next_pos[i] == m->agents[j].pos && next_pos[j] == m->agents[i].pos) {

                if ((m->agents[i].state == GOING_TO_PARK && m->agents[j].state == RETURNING_HOME_EMPTY) ||
                    (m->agents[j].state == GOING_TO_PARK && m->agents[i].state == RETURNING_HOME_EMPTY)) {
                    if (m->agents[i].state == RETURNING_HOME_EMPTY) {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[i].symbol);
                        next_pos[i] = m->agents[i].pos;
                    }
                    else {
                        logger_log(lg, "[%sAvoid%s] Swap conflict: parking flow has priority, Agent %c waits.", C_B_RED, C_NRM, m->agents[j].symbol);
                        next_pos[j] = m->agents[j].pos;
                    }
                }
                else {
                    int pi = priority_score(&m->agents[i]);
                    int pj = priority_score(&m->agents[j]);
                    if (pi >= pj) { logger_log(lg, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[j].symbol); next_pos[j] = m->agents[j].pos; }
                    else { logger_log(lg, "[%sAvoid%s] Swap conflict: Agent %c yields.", C_B_RED, C_NRM, m->agents[i].symbol); next_pos[i] = m->agents[i].pos; }
                }
            }
        }
    }


    WHCA_adjustHorizon(wf_cnt, planner_metrics_state().scc_last, lg);
}


static void broadcast_cell_change(AgentManager* am, GridMap* map, Node* changed) {
    
    if (!map || !changed) return;
    for (int a = 0; a < MAX_AGENTS; a++) {
        if (am->agents[a].pf) pathfinder_notify_cell_change(am->agents[a].pf, map, am, changed);
    }
}

void AgentManager::updateStateAfterMove(ScenarioManager* sc, GridMap* map, Logger* lg, Simulation_* sim) {
    AgentManager* m = this;
    
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &m->agents[i];
        if (ag->state == IDLE || ag->state == CHARGING || !ag->goal || ag->pos != ag->goal) continue;


        if (ag->state == GOING_TO_PARK || ag->state == GOING_TO_COLLECT) {
            if (ag->action_timer <= 0) {
                ag->action_timer = TASK_ACTION_TICKS;
                logger_log(lg, "[%sTask%s] Agent %c, %s task started (%d ticks).", C_YEL, C_NRM, ag->symbol,
                    ag->state == GOING_TO_PARK ? "parking" : "exiting", ag->action_timer);
                continue;
            }
            else {
                ag->action_timer--;
                if (ag->action_timer > 0) {
                    continue;
                }

            }
        }

        Node* reached = ag->goal;
        if (ag->state != GOING_TO_CHARGE) reached->reserved_by_agent = -1;
        ag->goal = NULL;

        switch (ag->state) {
        case GOING_TO_PARK:
            reached->is_parked = TRUE; m->total_cars_parked++;
            broadcast_cell_change(m, map, reached);
            logger_log(lg, "[%sPark%s] Agent %c parked a vehicle at (%d,%d).", C_GRN, C_NRM, ag->symbol, reached->x, reached->y);
            if (sc->mode == MODE_CUSTOM && sc->current_phase_index < sc->num_phases &&
                sc->phases[sc->current_phase_index].type == PARK_PHASE) {
                sc->tasks_completed_in_phase++;
                if (sim) {
                    int phase_idx = sc->current_phase_index;
                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
                        sim->phase_completed_tasks[phase_idx]++;
                    }
                    sim->tasks_completed_total++;
                }
            }
            else if (sim) {
                sim->tasks_completed_total++;
            }
            ag->state = RETURNING_HOME_EMPTY;
            ag->pf.reset();
            break;
        case RETURNING_HOME_EMPTY:
            logger_log(lg, "[%sInfo%s] Agent %c returned home after parking.", C_CYN, C_NRM, ag->symbol);
            ag->state = IDLE;
            ag->pf.reset();
            // --- metrics aggregate for PARK cycle ---
            metrics_finalize_task_if_active(sim, ag);
            break;
        case GOING_TO_COLLECT:
            logger_log(lg, "[%sExit%s] Agent %c picked up a parked vehicle at (%d,%d).", C_YEL, C_NRM, ag->symbol, reached->x, reached->y);
            reached->is_parked = FALSE; m->total_cars_parked--;
            broadcast_cell_change(m, map, reached);
            ag->state = RETURNING_WITH_CAR;
            ag->pf.reset();
            break;
        case RETURNING_WITH_CAR:
            logger_log(lg, "[%sExit%s] Agent %c completed the retrieval task.", C_GRN, C_NRM, ag->symbol);
            if (sc->mode == MODE_CUSTOM && sc->current_phase_index < sc->num_phases &&
                sc->phases[sc->current_phase_index].type == EXIT_PHASE) {
                sc->tasks_completed_in_phase++;
                if (sim) {
                    int phase_idx = sc->current_phase_index;
                    if (phase_idx >= 0 && phase_idx < MAX_PHASES) {
                        sim->phase_completed_tasks[phase_idx]++;
                    }
                    sim->tasks_completed_total++;
                }
            }
            else if (sim) {
                sim->tasks_completed_total++;
            }
            ag->state = IDLE;
            ag->pf.reset();
            // --- metrics aggregate for EXIT cycle ---
            metrics_finalize_task_if_active(sim, ag);
            break;
        case GOING_TO_CHARGE:
            logger_log(lg, "[%sCharge%s] Agent %c started charging (%d steps).", C_B_YEL, C_NRM, ag->symbol, CHARGE_TIME);
            ag->state = CHARGING; ag->charge_timer = CHARGE_TIME;
            if (ag->pos) broadcast_cell_change(m, map, ag->pos);
            break;
        case RETURNING_HOME_MAINTENANCE:
            logger_log(lg, "[%sInfo%s] Agent %c returned home after charging.", C_CYN, C_NRM, ag->symbol);
            ag->state = IDLE;
            ag->pf.reset();
            break;
        default: break;
        }
        ag->stuck_steps = 0;
    }
}

void agent_manager_update_state_after_move(AgentManager* m, ScenarioManager* sc, GridMap* map, Logger* lg, Simulation* sim) {
    if (m) m->updateStateAfterMove(sc, map, lg, sim);
}

void AgentManager::updateChargeState(GridMap* map, Logger* lg) {
    AgentManager* m = this;
    
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &m->agents[i];
        if (ag->state == CHARGING) {
            ag->charge_timer--;
            if (ag->charge_timer <= 0) {
                logger_log(lg, "[%sCharge%s] Agent %c finished charging.", C_B_GRN, C_NRM, ag->symbol);
                ag->total_distance_traveled = 0.0;
                ag->state = RETURNING_HOME_MAINTENANCE;
                if (ag->pos) ag->pos->reserved_by_agent = -1;
                if (ag->pos) broadcast_cell_change(m, map, ag->pos);
                ag->goal = NULL;
                ag->pf.reset();
                ag->stuck_steps = 0;
            }
        }
    }
}



static void WHCA_adjustHorizon(int wf_edges, int scc, Logger* lg) {
    
    conflict_score_state() = (int)(conflict_score_state() * 0.6) + wf_edges + (scc ? 5 : 0);
    {
        int oldH = whca_horizon_state();
        const int HI = 24;
        const int LO = 10;

        if (conflict_score_state() > HI && whca_horizon_state() < MAX_WHCA_HORIZON) whca_horizon_state() += 2;
        else if (conflict_score_state() < LO && whca_horizon_state() > MIN_WHCA_HORIZON) whca_horizon_state() -= 2;

        if (whca_horizon_state() < MIN_WHCA_HORIZON) whca_horizon_state() = MIN_WHCA_HORIZON;
        if (whca_horizon_state() > MAX_WHCA_HORIZON) whca_horizon_state() = MAX_WHCA_HORIZON;

        if (oldH != whca_horizon_state()) {
            logger_log(lg, "[%sWHCA*%s] Horizon adjusted %d -> %d (score=%d)", C_B_CYN, C_NRM, oldH, whca_horizon_state(), conflict_score_state());
        }
        planner_metrics_state().whca_h = whca_horizon_state();
    }
}

// =============================================================================

// =============================================================================


void agent_manager_plan_and_resolve_collisions_astar(AgentManager* manager, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) {
    

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &manager->agents[i];
        g_agent_ops.setGoalIfNeeded(agent, map, manager, logger);
        next_pos[i] = agent->pos;
    }


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
        if (agent->action_timer > 0) {
            continue;
        }


        if (agent->state == IDLE || agent->state == CHARGING || agent->goal == NULL || !current_pos) {
            continue;
        }

        Node* desired_move = current_pos;
        {
            ensure_pathfinder_for_agent(agent);
            TempMarkContext ctx; temp_context_init(&ctx, agent->pf, map, manager, 1);
            for (int h = 0; h < oi; h++) {
                int hid = order[h];
                if (next_pos[hid]) temp_context_mark(&ctx, next_pos[hid]);
            }
            for (int l = oi + 1; l < MAX_AGENTS; l++) {
                int lid = order[l];
                if (manager->agents[lid].pos) temp_context_mark(&ctx, manager->agents[lid].pos);
            }

            int goal_was_parked = (agent->state == GOING_TO_COLLECT && agent->goal->is_parked);
            if (goal_was_parked) {
                agent->goal->is_parked = FALSE;
                if (agent->pf) pathfinder_notify_cell_change(agent->pf, map, manager, agent->goal);
            }

            if (agent->pf && agent->pf->goal_node != agent->goal) {
                agent->pf->start_node = agent->pos;
                pathfinder_reset_goal(agent->pf, agent->goal);
            }

            if (agent->pf) {
                pathfinder_update_start(agent->pf, agent->pos);
                pathfinder_compute_shortest_path(agent->pf, map, manager);
                planner_metrics_state().astar_nodes_expanded_this_step += agent->pf->nodes_expanded_this_call;
                planner_metrics_state().astar_heap_moves_this_step += agent->pf->heap_moves_this_call;
                planner_metrics_state().astar_generated_nodes_this_step += agent->pf->nodes_generated_this_call;
                planner_metrics_state().astar_valid_expansions_this_step += agent->pf->valid_expansions_this_call;
                desired_move = pathfinder_get_next_step(agent->pf, map, manager, agent->pos);
            }

            if (goal_was_parked) {
                agent->goal->is_parked = TRUE;
                if (agent->pf) pathfinder_notify_cell_change(agent->pf, map, manager, agent->goal);
            }
            temp_context_cleanup(&ctx);
        }



        agent_apply_rotation_and_step(agent, current_pos, desired_move, &next_pos[i]);
    }


    resolve_conflicts_by_order(manager, order, next_pos);
}

void agent_manager_plan_and_resolve_collisions_dstar_basic(AgentManager* m, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) {
    

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &m->agents[i];
        g_agent_ops.setGoalIfNeeded(ag, map, m, lg);
        next_pos[i] = ag->pos;
    }


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
        if (ag->action_timer > 0) {
            continue;
        }


        if (ag->state == IDLE || ag->state == CHARGING || ag->goal == NULL || !current_pos) {
            continue;
        }


        Node* desired_move = current_pos;
        {
            if (!ag->pf) {
                ag->pf = pathfinder_create(ag->pos, ag->goal, ag);
            }
            else if (ag->pf->goal_node != ag->goal) {
                ag->pf->start_node = ag->pos;
                pathfinder_reset_goal(ag->pf, ag->goal);
            }

            TempMarkContext ctx; temp_context_init(&ctx, ag->pf, map, m, 1);
            for (int h = 0; h < oi; h++) {
                int hid = order[h];
                if (next_pos[hid]) temp_context_mark(&ctx, next_pos[hid]);
            }
            for (int l = oi + 1; l < MAX_AGENTS; l++) {
                int lid = order[l];
                if (m->agents[lid].pos) temp_context_mark(&ctx, m->agents[lid].pos);
            }

            int goal_was_parked = (ag->state == GOING_TO_COLLECT && ag->goal->is_parked);
            if (goal_was_parked) { ag->goal->is_parked = FALSE; if (ag->pf) pathfinder_notify_cell_change(ag->pf, map, m, ag->goal); }

            if (ag->pf) {
                pathfinder_update_start(ag->pf, ag->pos);
                pathfinder_compute_shortest_path(ag->pf, map, m);

                planner_metrics_state().dstar_nodes_expanded_this_step += ag->pf->nodes_expanded_this_call;
                planner_metrics_state().dstar_heap_moves_this_step += ag->pf->heap_moves_this_call;
                planner_metrics_state().dstar_generated_nodes_this_step += ag->pf->nodes_generated_this_call;
                planner_metrics_state().dstar_valid_expansions_this_step += ag->pf->valid_expansions_this_call;
                desired_move = pathfinder_get_next_step(ag->pf, map, m, ag->pos);
            }

            if (goal_was_parked) { ag->goal->is_parked = TRUE; if (ag->pf) pathfinder_notify_cell_change(ag->pf, map, m, ag->goal); }
            temp_context_cleanup(&ctx);
        }


        agent_apply_rotation_and_step(ag, current_pos, desired_move, &next_pos[i]);
    }


    resolve_conflicts_by_order(m, order, next_pos);
}
// =============================================================================

// =============================================================================
class DefaultPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions(am, map, lg, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DefaultPlannerStrategy>(*this);
    }
};

class AStarPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions_astar(am, map, lg, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<AStarPlannerStrategy>(*this);
    }
};

class DStarPlannerStrategy final : public PlannerStrategy {
public:
    void planStep(AgentManager* am, GridMap* map, Logger* lg, Node* next_pos[MAX_AGENTS]) const override {
        agent_manager_plan_and_resolve_collisions_dstar_basic(am, map, lg, next_pos);
    }

    std::unique_ptr<PlannerStrategy> clone() const override {
        return std::make_unique<DStarPlannerStrategy>(*this);
    }
};

static Planner planner_make_default(void) {
    return Planner(std::make_unique<DefaultPlannerStrategy>());
}

void agent_manager_update_charge_state(AgentManager* m, GridMap* map, Logger* lg) {
    if (m) m->updateChargeState(map, lg);
}
static Planner planner_make_astar(void) {
    return Planner(std::make_unique<AStarPlannerStrategy>());
}

static Planner planner_make_dstar(void) {
    return Planner(std::make_unique<DStarPlannerStrategy>());
}

static Planner planner_from_pathalgo(PathAlgo algo) {
    switch (algo) {
    case PATHALGO_ASTAR_SIMPLE: return planner_make_astar();
    case PATHALGO_DSTAR_BASIC:  return planner_make_dstar();
    case PATHALGO_DEFAULT:
    default:                    return planner_make_default();
    }
}


static void resolve_conflicts_by_order(const AgentManager* m, const int order[MAX_AGENTS], Node* next_pos[MAX_AGENTS]) {
    
    StepScratch& scratch = step_scratch_state();
    scratch.resetCellOwner(-1);

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        if (!next_pos[i]) continue;
        int next_idx = node_flat_index(next_pos[i]);
        if (next_idx < 0) continue;
        if (scratch.cell_owner[next_idx] != -1) {
            next_pos[i] = ((AgentManager*)m)->agents[i].pos;
            continue;
        }
        scratch.cell_owner[next_idx] = i;
    }

    scratch.resetCellOwner(-1);
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!m->agents[i].pos) continue;
        int current_idx = node_flat_index(m->agents[i].pos);
        if (current_idx >= 0) scratch.cell_owner[current_idx] = i;
    }

    for (int oi = 0; oi < MAX_AGENTS; oi++) {
        int i = order[oi];
        if (!next_pos[i]) continue;
        int dest_idx = node_flat_index(next_pos[i]);
        int other = (dest_idx >= 0) ? scratch.cell_owner[dest_idx] : -1;
        if (other == -1 || other == i || !next_pos[other]) continue;
        if (next_pos[other] == m->agents[i].pos) {
            next_pos[other] = ((AgentManager*)m)->agents[other].pos;
        }
        else if (next_pos[other] == m->agents[other].pos && next_pos[i] == m->agents[other].pos) {
            next_pos[i] = ((AgentManager*)m)->agents[i].pos;
        }
    }
}
// =============================================================================

// =============================================================================

static void do_ms_pause(int ms) { sleep_ms(ms); }


static int simulation_setup_custom_scenario(Simulation* sim) {
    ScenarioManager* s = sim->scenario_manager;

    printf(C_B_WHT "--- Custom Scenario Setup ---\n" C_NRM);
    s->num_phases = get_integer_input(C_YEL "Enter phase count (1-20, 0=cancel): " C_NRM, 0, MAX_PHASES);
    if (s->num_phases == 0) return 0;

    int max_per_phase = (sim->map && sim->map->num_goals > 0) ? sim->map->num_goals : 100000;

    for (int i = 0; i < s->num_phases; i++) {
        printf(C_B_CYN "\n--- Phase %d/%d ---\n" C_NRM, i + 1, s->num_phases);
        printf("a. %sParking%s\n", C_YEL, C_NRM);
        printf("b. %sRetrieval%s\n", C_CYN, C_NRM);
        char c = get_char_input("Select phase type: ", "ab");

        char prompt[64];
        snprintf(prompt, sizeof(prompt), "Phase task count (1~%d): ", max_per_phase);
        s->phases[i].task_count = get_integer_input(prompt, 1, max_per_phase);

        if (c == 'a') { s->phases[i].type = PARK_PHASE;  snprintf(s->phases[i].type_name, sizeof(s->phases[i].type_name), "park"); }
        else { s->phases[i].type = EXIT_PHASE;  snprintf(s->phases[i].type_name, sizeof(s->phases[i].type_name), "exit"); }

        printf(C_GRN "Phase %d configured: %s x %d.\n" C_NRM, i + 1, s->phases[i].type_name, s->phases[i].task_count);
    }
    printf(C_B_GRN "\n--- Custom scenario configuration complete. ---\n" C_NRM);
    do_ms_pause(1500);
    return 1;
}


static int simulation_setup_realtime(ScenarioManager* s) {
    printf(C_B_WHT "--- Real-Time Scenario Setup ---\n" C_NRM);
    while (TRUE) {
        s->park_chance = get_integer_input("\nParking request probability (0~100): ", 0, 100);
        s->exit_chance = get_integer_input("Retrieval request probability (0~100): ", 0, 100);
        if (s->park_chance + s->exit_chance <= 100) break;
        printf(C_B_RED "The total probability must not exceed 100.\n" C_NRM);
    }
    printf(C_B_GRN "\nReal-time configuration complete: parking=%d%%, retrieval=%d%%\n" C_NRM, s->park_chance, s->exit_chance);
    do_ms_pause(1500); return 1;
}

static int simulation_setup_speed(ScenarioManager* s) {
    printf(C_B_WHT "\n--- Simulation Speed Setup ---\n" C_NRM);


    s->speed_multiplier = get_float_input("Enter speed multiplier (0.0=as fast as possible, up to 10000.0): ", 0.0f, MAX_SPEED_MULTIPLIER);
    if (s->speed_multiplier <= 0.0f) {
        s->simulation_speed = 0;          // 0ms sleep
    }
    else {
        s->simulation_speed = (int)(100.0f / s->speed_multiplier);
        if (s->simulation_speed < 0) s->simulation_speed = 0;
    }

    printf(C_B_GRN "\n--- %.1fx simulation speed configured. ---\n" C_NRM, s->speed_multiplier);
    do_ms_pause(1500);
    return 1;
}




static int simulation_setup_map(Simulation* sim) {
    printf(C_B_WHT "--- Select Map (1~5) ---\n" C_NRM);
    printf("1. %sCompact parking lot%s (baseline)\n", C_B_GRN, C_NRM);
    printf("2. %sMid-size lot with one retrieval target%s\n", C_B_YEL, C_NRM);
    printf("3. %s8 AGVs + 900 requests%s (up to 16 AGVs, A~H)\n", C_B_YEL, C_NRM);
    printf("4. %sDense lot with one retrieval target and four parking waves%s (up to 10 AGVs, A~J)\n", C_B_YEL, C_NRM);
    printf("5. %sCharging-stress map%s (extra chargers, longer aisles, and heavy parking load)\n\n", C_B_YEL, C_NRM);
    int mid = get_integer_input("Select map id (1~5): ", 1, 5);
    sim->map_id = mid;
    grid_map_load_scenario(sim->map, sim->agent_manager, mid);
    logger_log(sim->logger, "[%sMap%s] Map #%d loaded.", C_B_CYN, C_NRM, mid);
    do_ms_pause(800);
    return 1;
}


int simulation_setup(Simulation* sim) {
    ui_clear_screen_optimized();

    if (!simulation_setup_map(sim)) return 0;


    printf(C_B_WHT "\n--- Select Path Planning Algorithm ---\n" C_NRM);
    printf("1. %sDefault (WHCA* + D* Lite + WFG + CBS)%s\n", C_B_GRN, C_NRM);
    printf("2. %sA* (single-agent)%s - recomputes the path from scratch each step\n", C_B_YEL, C_NRM);
    printf("3. %sD* Lite (incremental)%s - reuses previous search when the map changes\n\n", C_B_YEL, C_NRM);
    {
        int a = get_integer_input("Select algorithm (1~3): ", 1, 3);
        sim->path_algo = (a == 2) ? PATHALGO_ASTAR_SIMPLE : (a == 3) ? PATHALGO_DSTAR_BASIC : PATHALGO_DEFAULT;
        logger_log(sim->logger, "[%sAlgo%s] Algorithm selected: %d", C_B_CYN, C_NRM, a);
        sim->render_state.configureForAlgorithm(sim->path_algo);

        sim->planner = planner_from_pathalgo(sim->path_algo);
    }

    printf(C_B_WHT "\n--- Select Simulation Mode ---\n" C_NRM);
    printf("a. %sCustom phased scenario%s\n", C_YEL, C_NRM);
    printf("b. %sReal-time random scenario%s\n", C_CYN, C_NRM);
    printf("q. %sQuit%s\n\n", C_RED, C_NRM);
    char c = get_char_input("Select mode: ", "abq");
    int ok = 0;
    switch (c) {
    case 'a': sim->scenario_manager->mode = MODE_CUSTOM;
        if (simulation_setup_custom_scenario(sim))
            ok = simulation_setup_speed(sim->scenario_manager);
        break;
    case 'b': sim->scenario_manager->mode = MODE_REALTIME;
        if (simulation_setup_realtime(sim->scenario_manager))
            ok = simulation_setup_speed(sim->scenario_manager);
        break;
    case 'q': return 0;
    }
    if (ok) ui_clear_screen_optimized();
    return ok;
}

class TaskDispatchService final {
public:
    void update(Simulation* sim) const {
        if (!sim) return;
        ScenarioManager* sc = sim->scenario_manager;
        AgentManager* am = sim->agent_manager;
        GridMap* map = sim->map;
        Logger* lg = sim->logger;

        if (!advanceCustomPhaseIfNeeded(sc, lg)) return;
        generateRealtimeRequests(sim, sc, am, map, lg);
        sim->workload_snapshot = collect_agent_workload(am);
        assignIdleAgents(sim, sc, am, map, lg);
    }

private:
    bool advanceCustomPhaseIfNeeded(ScenarioManager* sc, Logger* lg) const {
        if (sc->mode != MODE_CUSTOM) return true;
        if (sc->current_phase_index >= sc->num_phases) return false;

        DynamicPhase* ph = &sc->phases[sc->current_phase_index];
        if (sc->tasks_completed_in_phase < ph->task_count) return true;

        logger_log(lg, "[%sPhase%s] %d completed (%s %d).", C_B_YEL, C_NRM,
            sc->current_phase_index + 1, ph->type_name, ph->task_count);
        sc->current_phase_index++;
        sc->tasks_completed_in_phase = 0;
        if (sc->current_phase_index < sc->num_phases) {
            DynamicPhase* next_phase = &sc->phases[sc->current_phase_index];
            logger_log(lg, "[%sPhase%s] %d start: %s %d.",
                C_B_YEL, C_NRM, sc->current_phase_index + 1, next_phase->type_name, next_phase->task_count);
            do_ms_pause(1500);
        }
        return false;
    }

    void generateRealtimeRequests(Simulation* sim, ScenarioManager* sc, AgentManager* am, GridMap* map, Logger* lg) const {
        if (sc->mode != MODE_REALTIME || sc->time_step <= 0) return;

        int roll_park = rand() % 500;
        int roll_exit = rand() % 500;

        if (sc->park_chance > 0 && roll_park < sc->park_chance) {
            int before = sc->task_count;
            if (am->total_cars_parked < map->num_goals) {
                logger_log(lg, "[%sEvent%s] New parking request.", C_B_GRN, C_NRM);
                add_task_to_queue(sc, TASK_PARK);
            }
            if (sc->task_count > before) sim->requests_created_total++;
        }

        if (sc->exit_chance > 0 && roll_exit < sc->exit_chance) {
            int before = sc->task_count;
            if (am->total_cars_parked > 0) {
                logger_log(lg, "[%sEvent%s] New exit request.", C_B_YEL, C_NRM);
                add_task_to_queue(sc, TASK_EXIT);
            }
            if (sc->task_count > before) sim->requests_created_total++;
        }
    }

    void assignIdleAgents(Simulation* sim, ScenarioManager* sc, AgentManager* am, GridMap* map, Logger* lg) const {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* ag = &am->agents[i];
            if (ag->state != IDLE || !ag->pos) continue;

            if (ag->total_distance_traveled >= DISTANCE_BEFORE_CHARGE) {
                if (select_best_charge_station(ag, map, am, lg)) {
                    ag->state = GOING_TO_CHARGE;
                }
                else {
                    logger_log(lg, "[%sWarn%s] Agent %c charge required but no station is available.", C_YEL, C_NRM, ag->symbol);
                }
                continue;
            }

            if (sc->mode == MODE_CUSTOM) {
                if (sc->current_phase_index >= sc->num_phases) continue;
                DynamicPhase* ph = &sc->phases[sc->current_phase_index];
                if (ph->type == PARK_PHASE) {
                    if ((sc->tasks_completed_in_phase + sim->workload_snapshot.active_park_agents) < ph->task_count &&
                        am->total_cars_parked < map->num_goals) {
                        g_agent_ops.beginTaskPark(ag, sc, lg);
                        sim->workload_snapshot.active_park_agents++;
                    }
                }
                else if ((sc->tasks_completed_in_phase + sim->workload_snapshot.active_exit_agents) < ph->task_count &&
                    am->total_cars_parked > 0) {
                    g_agent_ops.beginTaskExit(ag, sc, lg);
                    sim->workload_snapshot.active_exit_agents++;
                }
                continue;
            }

            if (sc->mode == MODE_REALTIME && sc->task_count > 0) {
                TaskType dequeued_type = TASK_NONE;
                int lot_full = (am->total_cars_parked >= map->num_goals);
                if (!sc->tryDequeueAssignableTask(lot_full, am->total_cars_parked, &dequeued_type)) continue;
                if (dequeued_type == TASK_PARK) g_agent_ops.beginTaskPark(ag, sc, lg);
                else g_agent_ops.beginTaskExit(ag, sc, lg);
            }
        }
    }
};

static const TaskDispatchService g_task_dispatch_service{};

void Simulation_::updateState() {
    SimulationContextGuard guard(this);
    g_task_dispatch_service.update(this);
}

struct StepExecutionFrame {
    int phase_idx_for_step{ 0 };
    int step_label{ 0 };
    int is_custom_mode{ FALSE };
    int phase_active{ FALSE };
    int cleanup_region{ FALSE };
    clock_t step_start_cpu{ 0 };
};

class StepExecutorService final {
public:
    void execute(Simulation* sim, int is_paused) const {
        if (!sim) return;

        StepExecutionFrame frame = beginFrame(sim);
        agent_manager_update_charge_state(sim->agent_manager, sim->map, sim->logger);
        g_task_dispatch_service.update(sim);

        StepScratch& scratch = sim->step_scratch;
        Node** next_pos = scratch.next_positions.data();
        Node** prev_pos = scratch.previous_positions.data();
        prepareMovementPlan(sim, next_pos, prev_pos, scratch);

        int moved_this_step = apply_moves_and_update_stuck(sim, next_pos, prev_pos);
        finalizeMoveState(sim, frame.step_label);
        finalizeFrame(sim, frame, moved_this_step, is_paused);
    }

private:
    StepExecutionFrame beginFrame(Simulation* sim) const {
        StepExecutionFrame frame;
        frame.phase_idx_for_step = sim->scenario_manager->current_phase_index;
        frame.step_label = sim->scenario_manager->time_step + 1;
        frame.is_custom_mode = (sim->scenario_manager->mode == MODE_CUSTOM);
        frame.phase_active = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= 0 &&
            frame.phase_idx_for_step < sim->scenario_manager->num_phases);
        frame.cleanup_region = (frame.is_custom_mode &&
            frame.phase_idx_for_step >= sim->scenario_manager->num_phases);
        frame.step_start_cpu = clock();
        return frame;
    }

    void prepareMovementPlan(Simulation* sim, Node** next_pos, Node** prev_pos, StepScratch& scratch) const {
        AgentManager* am = sim->agent_manager;
        int* order = scratch.priority_order.data();

        for (int i = 0; i < MAX_AGENTS; i++) {
            prev_pos[i] = am->agents[i].pos;
        }

        simulation_plan_step(sim, next_pos);
        applyRotationStage(am, next_pos);
        resolveStationaryBlockers(am, next_pos, scratch);
        sort_agents_by_priority(am, order);
        resolve_conflicts_by_order(am, order, next_pos);
    }

    void applyRotationStage(AgentManager* am, Node** next_pos) const {
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* ag = &am->agents[i];
            if (ag->state == CHARGING) continue;
            Node* current = ag->pos;
            if (!current || !next_pos[i]) continue;
            if (ag->rotation_wait > 0) {
                next_pos[i] = current;
                ag->rotation_wait--;
                continue;
            }
            Node* adjusted = current;
            agent_apply_rotation_and_step(ag, current, next_pos[i], &adjusted);
            next_pos[i] = adjusted;
        }
    }

    void resolveStationaryBlockers(AgentManager* am, Node** next_pos, StepScratch& scratch) const {
        scratch.resetCellOwner(-1);
        for (int i = 0; i < MAX_AGENTS; i++) {
            Agent* blocker = &am->agents[i];
            if (!blocker->pos || !next_pos[i]) continue;
            if (blocker->rotation_wait > 0 || next_pos[i] == blocker->pos) {
                int blocked_idx = node_flat_index(blocker->pos);
                if (blocked_idx >= 0) scratch.cell_owner[blocked_idx] = i;
            }
        }

        for (int j = 0; j < MAX_AGENTS; j++) {
            Agent* mover = &am->agents[j];
            if (!mover->pos || !next_pos[j] || next_pos[j] == mover->pos) continue;
            int blocked_idx = node_flat_index(next_pos[j]);
            if (blocked_idx < 0) continue;
            int blocker = scratch.cell_owner[blocked_idx];
            if (blocker != -1 && blocker != j) {
                next_pos[j] = mover->pos;
            }
        }
    }

    void finalizeMoveState(Simulation* sim, int step_label) const {
        unsigned long long prev_completed_tasks = sim->tasks_completed_total;
        agent_manager_update_state_after_move(sim->agent_manager, sim->scenario_manager, sim->map, sim->logger, sim);
        if (sim->tasks_completed_total != prev_completed_tasks) {
            sim->last_task_completion_step = step_label;
        }
    }

    void finalizeFrame(Simulation* sim, const StepExecutionFrame& frame, int moved_this_step, int is_paused) const {
        clock_t step_end_cpu = clock();
        double step_time_ms = ((double)(step_end_cpu - frame.step_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
        sim->last_step_cpu_time_ms = step_time_ms;
        sim->total_cpu_time_ms += step_time_ms;
        if (step_time_ms > sim->max_step_cpu_time_ms) {
            sim->max_step_cpu_time_ms = step_time_ms;
        }

        if (frame.is_custom_mode) {
            if (frame.phase_active) {
                int idx = frame.phase_idx_for_step;
                if (idx >= 0 && idx < MAX_PHASES) {
                    if (sim->phase_step_counts[idx] == 0) {
                        sim->phase_first_step[idx] = frame.step_label;
                    }
                    sim->phase_last_step[idx] = frame.step_label;
                    sim->phase_step_counts[idx]++;
                    sim->phase_cpu_time_ms[idx] += step_time_ms;
                }
            }
            else if (frame.cleanup_region) {
                if (sim->post_phase_step_count == 0) {
                    sim->post_phase_first_step = frame.step_label;
                }
                sim->post_phase_last_step = frame.step_label;
                sim->post_phase_step_count++;
                sim->post_phase_cpu_time_ms += step_time_ms;
                if (sim->post_phase_step_count >= CLEANUP_FORCE_IDLE_AFTER_STEPS) {
                    force_idle_cleanup(sim->agent_manager, sim, sim->logger);
                }
            }
        }

        update_deadlock_counter(sim, moved_this_step, frame.is_custom_mode);
        accumulate_wait_ticks_if_realtime(sim);
        simulation_collect_memory_sample_algo(sim);
        simulation_collect_memory_sample(sim);
        sim->total_executed_steps = frame.step_label;
        simulation_append_step_metrics(sim, frame.step_label);
        if (!renderer_state().suppress_flush) {
            sim->renderer.drawFrame(sim, is_paused);
        }
    }
};

static const StepExecutorService g_step_executor_service{};

void Simulation_::executeOneStep(int is_paused) {
    SimulationContextGuard guard(this);
    g_step_executor_service.execute(this, is_paused);
}
static void simulation_execute_one_step(Simulation* sim, int is_paused) {
    if (sim) sim->executeOneStep(is_paused);
}


static void force_idle_cleanup(AgentManager* am, Simulation* sim, Logger* lg) {
    if (!am) return;
    int changed = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* ag = &am->agents[i];
        if (!ag->pos) continue;
        if (ag->state == IDLE) continue;
        if (ag->goal) { ag->goal->reserved_by_agent = -1; ag->goal = NULL; }
        ag->pf.reset();
        ag->rotation_wait = 0;
        ag->stuck_steps = 0;
        ag->action_timer = 0;
        ag->state = IDLE;
        changed = 1;
    }
    if (changed && lg) {
        logger_log(lg, "[%sCleanup%s] Forced unfinished agents back to IDLE before shutdown.", C_B_CYN, C_NRM);
    }
}

bool Simulation_::isComplete() const {
    const Simulation* sim = this;
    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    if (sc->mode == MODE_CUSTOM && sc->current_phase_index >= sc->num_phases) {
        for (int i = 0; i < MAX_AGENTS; i++) if (am->agents[i].state != IDLE) return FALSE;
        if (!sim->suppress_stdout) printf(C_B_GRN "\nCustom scenario completed.\n" C_NRM);
        return TRUE;
    }
    if (sc->mode == MODE_REALTIME && sc->time_step >= REALTIME_MODE_TIMELIMIT) {
        if (!sim->suppress_stdout) printf(C_B_GRN "\nReal-time simulation completed.\n" C_NRM);
        return TRUE;
    }
    return FALSE;
}

static int simulation_is_complete(const Simulation* sim) {
    return sim ? sim->isComplete() : FALSE;
}


void Simulation_::run() {
    SimulationContextGuard guard(this);
    Simulation* sim = this;
    ControlState cs; 
    ControlState_init(&cs);

    simulation_reset_runtime_stats(sim);


    sim->renderer.drawFrame(sim, cs.is_paused);

    while (!cs.quit_flag) {

        cs.last_key = check_for_input();
        if (cs.last_key) {
            ui_handle_control_key(sim, cs.last_key, &cs.is_paused, &cs.quit_flag);
            sim->renderer.drawFrame(sim, cs.is_paused);
            if (cs.quit_flag) continue;
        }



        if (cs.is_paused && tolower(cs.last_key) != 's') {
            sleep_ms(PAUSE_POLL_INTERVAL_MS);
            continue;
        }


        simulation_execute_one_step(sim, cs.is_paused);

        if (simulation_is_complete(sim)) {
            break;
        }

        maybe_report_realtime_dashboard(sim);

        if (sim->scenario_manager->simulation_speed > 0) sleep_ms(sim->scenario_manager->simulation_speed);
    }
}

void simulation_run(Simulation* sim) {
    if (sim) sim->run();
}


void Simulation_::printPerformanceSummary() const {
    const Simulation* sim = this;
    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    const int recorded_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : (sc ? sc->time_step : 0);
    const double avg_cpu_ms = (recorded_steps > 0) ? (sim->total_cpu_time_ms / (double)recorded_steps) : 0.0;
    const double avg_plan_ms = (recorded_steps > 0) ? (sim->total_planning_time_ms / (double)recorded_steps) : 0.0;
    const double throughput = (recorded_steps > 0) ? ((double)sim->tasks_completed_total / (double)recorded_steps) : 0.0;
    const double avg_memory_kb = (sim->memory_samples > 0) ? (sim->memory_usage_sum_kb / (double)sim->memory_samples) : 0.0;

    const char* mode_label = "Uninitialized";
    if (sc) {
        switch (sc->mode) {
        case MODE_CUSTOM: mode_label = "Custom"; break;
        case MODE_REALTIME: mode_label = "Real-Time"; break;
        default: mode_label = "Uninitialized"; break;
        }
    }

    printf("\n============================================\n");
    printf("          Simulation Result Report\n");
    printf("============================================\n");
    printf(" Mode                                : %s\n", mode_label);
    printf(" Map ID                              : %d\n", sim->map_id);
    {
        const char* algo = "Default (WHCA* + D* Lite + WFG + CBS)";
        if (sim->path_algo == PATHALGO_ASTAR_SIMPLE) algo = "A* (Single-Agent)";
        else if (sim->path_algo == PATHALGO_DSTAR_BASIC) algo = "D* Lite (Incremental)";
        printf(" Path Planning Algorithm             : %s\n", algo);
    }
    printf(" Total Physical Time Steps           : %d\n", recorded_steps);
    {
        int active_agents = 0;
        if (am) {
            for (int i = 0; i < MAX_AGENTS; i++) if (am->agents[i].pos) active_agents++;
        }
        printf(" Operating AGVs                     : %d\n", active_agents);
    }

    printf(" Tasks Completed (total)             : %llu\n", sim->tasks_completed_total);
    printf(" Throughput [task / total physical time] : %.4f\n", throughput);
    printf(" Total Movement Cost (cells)         : %.2f\n", sim->total_movement_cost);

    printf(" Requests Created (total)            : %llu\n", sim->requests_created_total);
    printf(" Request Wait Ticks (sum)            : %llu\n", sim->request_wait_ticks_sum);
    printf(" Process Memory Usage Sum            : %.2f KB\n", sim->memory_usage_sum_kb);
    printf(" Process Memory Usage Average        : %.2f KB\n", avg_memory_kb);
    printf(" Process Memory Usage Peak           : %.2f KB\n", sim->memory_usage_peak_kb);
    printf(" Remaining Parked Vehicles           : %d\n", am ? am->total_cars_parked : 0);
    printf("\n -- Algorithm and Planner Statistics --\n");
    printf(" Nodes Expanded (total)             : %llu\n", sim->algo_nodes_expanded_total);
    printf(" Heap Moves (total)                  : %llu\n", sim->algo_heap_moves_total);
    printf(" Generated Nodes (total)            : %llu\n", sim->algo_generated_nodes_total);
    printf(" Valid Expansions (total)           : %llu\n", sim->algo_valid_expansions_total);
    double valid_ratio_total = (sim->algo_generated_nodes_total > 0) ? (double)sim->algo_valid_expansions_total / (double)sim->algo_generated_nodes_total : 0.0;
    printf(" Valid Expansion Ratio (valid/gen) : %.4f\n", valid_ratio_total);
    if (recorded_steps > 0) {
        const double avg_nodes_per_step = (double)sim->algo_nodes_expanded_total / (double)recorded_steps;
        const double avg_heap_moves_per_step = (double)sim->algo_heap_moves_total / (double)recorded_steps;
        const double avg_generated_per_step = (double)sim->algo_generated_nodes_total / (double)recorded_steps;
        const double avg_valid_per_step = (double)sim->algo_valid_expansions_total / (double)recorded_steps;
        printf(" Nodes Expanded (avg per step)      : %.2f\n", avg_nodes_per_step);
        printf(" Heap Moves (avg per step)          : %.2f\n", avg_heap_moves_per_step);
        printf(" Generated Nodes (avg per step)     : %.2f\n", avg_generated_per_step);
        printf(" Valid Expansions (avg per step)    : %.2f\n", avg_valid_per_step);
    }

    if (sc && sc->mode == MODE_CUSTOM) {
        printf("\n -- Custom Scenario Breakdown --\n");
        for (int i = 0; i < sc->num_phases; i++) {
            const DynamicPhase* ph = &sc->phases[i];
            const int planned = ph->task_count;
            const int completed = sim->phase_completed_tasks[i];
            const int step_count = sim->phase_step_counts[i];
            printf(" Phase %d (%s)\n", i + 1, ph->type_name);
            printf("   Planned Tasks           : %d\n", planned);
            printf("   Completed Tasks         : %d\n", completed);
            if (step_count > 0) {
                printf("   Step Span               : %d step(s)", step_count);
                if (sim->phase_first_step[i] >= 0 && sim->phase_last_step[i] >= 0) {
                    printf(" [#%d -> #%d]\n", sim->phase_first_step[i], sim->phase_last_step[i]);
                }
                else {
                    printf("\n");
                }
                const double phase_avg_cpu = sim->phase_cpu_time_ms[i] / (double)step_count;
                printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n",
                    sim->phase_cpu_time_ms[i], phase_avg_cpu);
            }
            else {
                printf("   Step Span               : N/A\n");
            }
            if (completed < planned) {
                printf("   Remaining Tasks         : %d\n", planned - completed);
            }
        }
    }
    else if (sc && sc->mode == MODE_REALTIME) {
        printf("\n -- Custom Scenario Breakdown --\n");

        printf(" Phase 1 (%s)\n", "Real-Time");
        printf("   Planned Tasks           : %d\n", (int)sim->tasks_completed_total);
        printf("   Completed Tasks         : %d\n", (int)sim->tasks_completed_total);
        if (recorded_steps > 0) {
            printf("   Step Span               : %d step(s) [#%d -> #%d]\n", recorded_steps, 1, recorded_steps);
            const double phase_avg_cpu = sim->total_cpu_time_ms / (double)recorded_steps;
            printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n", sim->total_cpu_time_ms, phase_avg_cpu);
        }
        else {
            printf("   Step Span               : N/A\n");
            printf("   CPU Time                : 0.00 ms (avg 0.0000 ms/step)\n");
        }
    }

    printf("============================================\n");
}

void simulation_print_performance_summary(const Simulation* sim) {
    if (sim) sim->printPerformanceSummary();
}

// =============================================================================

// =============================================================================
Simulation* simulation_create() {
    Simulation* s = new Simulation();
    simulation_set_active(s);
    s->agent_manager = agent_manager_create();
    s->map = grid_map_create(s->agent_manager); // default map #1
    s->scenario_manager = scenario_manager_create();
    s->logger = logger_create();
    s->map_id = 1; // current default map
    s->path_algo = PATHALGO_DEFAULT;
    s->planner = planner_from_pathalgo(s->path_algo);
    s->renderer = renderer_create_facade();
    s->render_state.configureForAlgorithm(s->path_algo);
    planner_metrics_state().whca_h = whca_horizon_state();
    
    metrics_subscribe(simulation_metrics_observer, s);
    return s;
}
void simulation_destroy(Simulation* s) {
    if (simulation_active() == s) simulation_set_active(nullptr);
    delete s;
}
bool Simulation_::openStepMetricsFile(const char* path) {
    closeStepMetricsFile();
    if (!path || !path[0]) return TRUE;

    step_metrics_stream = fopen(path, "w");
    if (!step_metrics_stream) return FALSE;

    fprintf(step_metrics_stream,
        "step,tasks_completed_total,total_movement_cost,deadlock_count,last_step_cpu_ms,total_cpu_ms,last_planning_ms,total_planning_ms,algo_nodes_expanded_last,algo_heap_moves_last,algo_generated_nodes_last,algo_valid_expansions_last,requests_created_total,request_wait_ticks_sum\n");
    fflush(step_metrics_stream);
    step_metrics_header_written = TRUE;
    return TRUE;
}

static int simulation_open_step_metrics_file(Simulation* sim, const char* path) {
    return sim ? sim->openStepMetricsFile(path) : FALSE;
}

void Simulation_::closeStepMetricsFile() {
    if (step_metrics_stream) {
        fclose(step_metrics_stream);
        step_metrics_stream = NULL;
    }
    step_metrics_header_written = FALSE;
}

static void simulation_close_step_metrics_file(Simulation* sim) {
    if (sim) sim->closeStepMetricsFile();
}

void Simulation_::appendStepMetrics(int step_label) {
    if (!step_metrics_stream || !step_metrics_header_written) return;
    fprintf(step_metrics_stream,
        "%d,%llu,%.6f,%llu,%.6f,%.6f,%.6f,%.6f,%llu,%llu,%llu,%llu,%llu,%llu\n",
        step_label,
        tasks_completed_total,
        total_movement_cost,
        deadlock_count,
        last_step_cpu_time_ms,
        total_cpu_time_ms,
        last_planning_time_ms,
        total_planning_time_ms,
        algo_nodes_expanded_last_step,
        algo_heap_moves_last_step,
        algo_generated_nodes_last_step,
        algo_valid_expansions_last_step,
        requests_created_total,
        request_wait_ticks_sum);
    fflush(step_metrics_stream);
}

static void simulation_append_step_metrics(Simulation* sim, int step_label) {
    if (sim) sim->appendStepMetrics(step_label);
}

void agv_default_config(AgvSimulationConfig* cfg) {
    if (!cfg) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->seed = (unsigned int)time(NULL);
    cfg->map_id = 1;
    cfg->path_algo = PATHALGO_DEFAULT;
    cfg->mode = AGV_MODECFG_CUSTOM;
    cfg->speed_multiplier = 0.0f;
    cfg->num_phases = 1;
    cfg->phases[0].type = AGV_PHASECFG_PARK;
    cfg->phases[0].task_count = 1;
    cfg->suppress_stdout = FALSE;
}

void agv_prepare_console(void) {
    system_enable_virtual_terminal();
    ensure_console_width(180);
}

static int agv_clamp_map_id(int map_id) {
    if (map_id < 1) return 1;
    if (map_id > 5) return 5;
    return map_id;
}

static PathAlgo agv_path_algo_from_config(int path_algo) {
    if (path_algo == PATHALGO_ASTAR_SIMPLE || path_algo == 1) return PATHALGO_ASTAR_SIMPLE;
    if (path_algo == PATHALGO_DSTAR_BASIC || path_algo == 2 || path_algo == 3) return PATHALGO_DSTAR_BASIC;
    return PATHALGO_DEFAULT;
}

static void agv_apply_speed_to_scenario(ScenarioManager* sc, float speed_multiplier) {
    if (!sc) return;
    sc->applySpeedMultiplier(speed_multiplier);
}

int agv_apply_config(Simulation* sim, const AgvSimulationConfig* cfg) {
    if (!sim || !cfg) return FALSE;

    simulation_set_active(sim);
    sim->configured_seed = cfg->seed;
    srand(cfg->seed);

    sim->suppress_stdout = cfg->suppress_stdout ? TRUE : FALSE;
    renderer_state().suppress_flush = cfg->suppress_stdout ? TRUE : FALSE;

    sim->map_id = agv_clamp_map_id(cfg->map_id);
    grid_map_load_scenario(sim->map, sim->agent_manager, sim->map_id);

    sim->path_algo = agv_path_algo_from_config(cfg->path_algo);
    sim->planner = planner_from_pathalgo(sim->path_algo);
    sim->render_state.configureForAlgorithm(sim->path_algo);

    ScenarioManager* sc = sim->scenario_manager;
    scenario_manager_clear_task_queue(sc);
    sc->mode = (cfg->mode == AGV_MODECFG_REALTIME) ? MODE_REALTIME : MODE_CUSTOM;
    sc->time_step = 0;
    sc->num_phases = 0;
    sc->current_phase_index = 0;
    sc->tasks_completed_in_phase = 0;
    sc->task_queue.clear();
    sc->task_count = 0;
    sc->park_chance = 0;
    sc->exit_chance = 0;

    if (sc->mode == MODE_REALTIME) {
        sc->park_chance = cfg->realtime_park_chance;
        sc->exit_chance = cfg->realtime_exit_chance;
    }
    else {
        sc->num_phases = cfg->num_phases;
        if (sc->num_phases < 0) sc->num_phases = 0;
        if (sc->num_phases > MAX_PHASES) sc->num_phases = MAX_PHASES;
        for (int i = 0; i < sc->num_phases; i++) {
            sc->phases[i].type = (cfg->phases[i].type == AGV_PHASECFG_EXIT) ? EXIT_PHASE : PARK_PHASE;
            sc->phases[i].task_count = cfg->phases[i].task_count;
            if (sc->phases[i].task_count < 1) sc->phases[i].task_count = 1;
            snprintf(sc->phases[i].type_name, sizeof(sc->phases[i].type_name), "%s",
                (sc->phases[i].type == PARK_PHASE) ? "park" : "exit");
        }
    }

    agv_apply_speed_to_scenario(sc, cfg->speed_multiplier);
    simulation_reset_runtime_stats(sim);
    return simulation_open_step_metrics_file(sim, cfg->step_metrics_path);
}

int agv_execute_headless_step(Simulation* sim) {
    if (!sim) return TRUE;
    if (simulation_is_complete(sim)) return TRUE;
    simulation_execute_one_step(sim, FALSE);
    if (simulation_is_complete(sim)) return TRUE;
    maybe_report_realtime_dashboard(sim);
    if (sim->scenario_manager->simulation_speed > 0) sleep_ms(sim->scenario_manager->simulation_speed);
    return FALSE;
}

int agv_run_to_completion(Simulation* sim) {
    if (!sim) return FALSE;
    while (!agv_execute_headless_step(sim)) {
    }
    return TRUE;
}

int agv_is_complete(const Simulation* sim) {
    if (!sim) return TRUE;
    return simulation_is_complete(sim);
}

void agv_collect_run_summary(const Simulation* sim, AgvRunSummary* out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!sim) return;

    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    int recorded_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : (sc ? sc->time_step : 0);
    if (recorded_steps < 0) recorded_steps = 0;

    out->seed = sim->configured_seed;
    out->map_id = sim->map_id;
    out->path_algo = sim->path_algo;
    if (!sc) out->mode = AGV_MODECFG_CUSTOM;
    else out->mode = (sc->mode == MODE_REALTIME) ? AGV_MODECFG_REALTIME : AGV_MODECFG_CUSTOM;
    out->recorded_steps = recorded_steps;
    out->tasks_completed_total = sim->tasks_completed_total;
    out->throughput = (recorded_steps > 0) ? ((double)sim->tasks_completed_total / (double)recorded_steps) : 0.0;
    out->total_movement_cost = sim->total_movement_cost;
    out->deadlock_count = sim->deadlock_count;
    out->total_cpu_time_ms = sim->total_cpu_time_ms;
    out->avg_cpu_time_ms = (recorded_steps > 0) ? (sim->total_cpu_time_ms / (double)recorded_steps) : 0.0;
    out->total_planning_time_ms = sim->total_planning_time_ms;
    out->avg_planning_time_ms = (recorded_steps > 0) ? (sim->total_planning_time_ms / (double)recorded_steps) : 0.0;
    out->memory_usage_sum_kb = sim->memory_usage_sum_kb;
    out->avg_memory_usage_kb = (sim->memory_samples > 0) ? (sim->memory_usage_sum_kb / (double)sim->memory_samples) : 0.0;
    out->memory_usage_peak_kb = sim->memory_usage_peak_kb;
    out->algo_nodes_expanded_total = sim->algo_nodes_expanded_total;
    out->algo_heap_moves_total = sim->algo_heap_moves_total;
    out->algo_generated_nodes_total = sim->algo_generated_nodes_total;
    out->algo_valid_expansions_total = sim->algo_valid_expansions_total;
    out->valid_expansion_ratio = (sim->algo_generated_nodes_total > 0)
        ? ((double)sim->algo_valid_expansions_total / (double)sim->algo_generated_nodes_total)
        : 0.0;
    out->requests_created_total = sim->requests_created_total;
    out->request_wait_ticks_sum = sim->request_wait_ticks_sum;
    out->remaining_parked_vehicles = am ? am->total_cars_parked : 0;

    if (am) {
        for (int i = 0; i < MAX_AGENTS; i++) {
            if (am->agents[i].pos) out->active_agents++;
        }
    }
}

static void simulation_update_state(Simulation* sim) {
    if (sim) sim->updateState();
}

int agv_copy_render_frame(Simulation* sim, int is_paused, char* buffer, size_t buffer_size) {
    if (!sim || !buffer || buffer_size == 0) return 0;
    SimulationContextGuard guard(sim);
    {
        int prev_suppress = renderer_state().suppress_flush;
        renderer_state().suppress_flush = TRUE;
        simulation_display_status(sim, is_paused);
        renderer_state().suppress_flush = prev_suppress;
    }
    snprintf(buffer, buffer_size, "%s", sim->display_buffer.data());
    return (int)strlen(buffer);
}

int agv_open_step_metrics(Simulation* sim, const char* path) {
    return simulation_open_step_metrics_file(sim, path);
}

void agv_close_step_metrics(Simulation* sim) {
    simulation_close_step_metrics_file(sim);
}

int agv_write_run_summary_json(const Simulation* sim, const char* path) {
    if (!sim || !path || !path[0]) return FALSE;

    FILE* fp = fopen(path, "w");
    if (!fp) return FALSE;

    AgvRunSummary summary;
    agv_collect_run_summary(sim, &summary);

    {
        const char* mode_label = "uninitialized";
        const char* algo_label = "default";
        if (summary.mode == AGV_MODECFG_CUSTOM) mode_label = "custom";
        else if (summary.mode == AGV_MODECFG_REALTIME) mode_label = "realtime";
        if (summary.path_algo == PATHALGO_ASTAR_SIMPLE) algo_label = "astar";
        else if (summary.path_algo == PATHALGO_DSTAR_BASIC) algo_label = "dstar";

        fprintf(fp, "{\n");
        fprintf(fp, "  \"seed\": %u,\n", summary.seed);
        fprintf(fp, "  \"map_id\": %d,\n", summary.map_id);
        fprintf(fp, "  \"path_algo\": %d,\n", summary.path_algo);
        fprintf(fp, "  \"path_algo_label\": \"%s\",\n", algo_label);
        fprintf(fp, "  \"mode\": %d,\n", summary.mode);
        fprintf(fp, "  \"mode_label\": \"%s\",\n", mode_label);
        fprintf(fp, "  \"active_agents\": %d,\n", summary.active_agents);
        fprintf(fp, "  \"recorded_steps\": %d,\n", summary.recorded_steps);
        fprintf(fp, "  \"tasks_completed_total\": %llu,\n", summary.tasks_completed_total);
        fprintf(fp, "  \"throughput\": %.10f,\n", summary.throughput);
        fprintf(fp, "  \"total_movement_cost\": %.10f,\n", summary.total_movement_cost);
        fprintf(fp, "  \"deadlock_count\": %llu,\n", summary.deadlock_count);
        fprintf(fp, "  \"total_cpu_time_ms\": %.10f,\n", summary.total_cpu_time_ms);
        fprintf(fp, "  \"avg_cpu_time_ms\": %.10f,\n", summary.avg_cpu_time_ms);
        fprintf(fp, "  \"total_planning_time_ms\": %.10f,\n", summary.total_planning_time_ms);
        fprintf(fp, "  \"avg_planning_time_ms\": %.10f,\n", summary.avg_planning_time_ms);
        fprintf(fp, "  \"memory_usage_sum_kb\": %.10f,\n", summary.memory_usage_sum_kb);
        fprintf(fp, "  \"avg_memory_usage_kb\": %.10f,\n", summary.avg_memory_usage_kb);
        fprintf(fp, "  \"memory_usage_peak_kb\": %.10f,\n", summary.memory_usage_peak_kb);
        fprintf(fp, "  \"algo_nodes_expanded_total\": %llu,\n", summary.algo_nodes_expanded_total);
        fprintf(fp, "  \"algo_heap_moves_total\": %llu,\n", summary.algo_heap_moves_total);
        fprintf(fp, "  \"algo_generated_nodes_total\": %llu,\n", summary.algo_generated_nodes_total);
        fprintf(fp, "  \"algo_valid_expansions_total\": %llu,\n", summary.algo_valid_expansions_total);
        fprintf(fp, "  \"valid_expansion_ratio\": %.10f,\n", summary.valid_expansion_ratio);
        fprintf(fp, "  \"requests_created_total\": %llu,\n", summary.requests_created_total);
        fprintf(fp, "  \"request_wait_ticks_sum\": %llu,\n", summary.request_wait_ticks_sum);
        fprintf(fp, "  \"remaining_parked_vehicles\": %d\n", summary.remaining_parked_vehicles);
        fprintf(fp, "}\n");
    }

    fclose(fp);
    return TRUE;
}

#ifndef AGV_NO_MAIN
int main() {
    srand((unsigned int)time(NULL));
    system_enable_virtual_terminal();
    ensure_console_width(180);


    Simulation* sim = simulation_create();
    if (!sim) return 1;

    if (simulation_setup(sim)) {
        ui_enter_alt_screen();
        simulation_run(sim);
        ui_leave_alt_screen();
        simulation_print_performance_summary(sim);
        printf("\nPress any key to exit...\n");
        (void)_getch();
    }
    else {
        printf("\nSimulation setup was cancelled. Exiting.\n");
    }


    simulation_destroy(sim);
    return 0;
}
#endif
