#pragma once

#include <array>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <memory>

// Private legacy-engine type model.
// This header is intentionally included from agv_legacy_engine.cpp
// after the engine-wide compile-time constants/macros are defined.

template <typename T>
class OwnedPtr final {
public:
    OwnedPtr() = default;
    OwnedPtr(std::nullptr_t) {}
    explicit OwnedPtr(std::unique_ptr<T> value) : value_(std::move(value)) {}
    explicit OwnedPtr(T* value) : value_(value) {}

    OwnedPtr(const OwnedPtr&) = delete;
    OwnedPtr& operator=(const OwnedPtr&) = delete;
    OwnedPtr(OwnedPtr&&) noexcept = default;
    OwnedPtr& operator=(OwnedPtr&&) noexcept = default;

    OwnedPtr& operator=(std::unique_ptr<T> value) {
        value_ = std::move(value);
        return *this;
    }

    OwnedPtr& operator=(T* value) {
        value_.reset(value);
        return *this;
    }

    OwnedPtr& operator=(std::nullptr_t) {
        value_.reset();
        return *this;
    }

    T* get() const { return value_.get(); }
    T* release() { return value_.release(); }
    void reset(T* value = nullptr) { value_.reset(value); }
    explicit operator bool() const { return static_cast<bool>(value_); }
    operator T* () const { return value_.get(); }
    T* operator->() const { return value_.get(); }
    T& operator*() const { return *value_; }

private:
    std::unique_ptr<T> value_{};
};

struct Key {
    double k1{0.0};
    double k2{0.0};
};

static inline Key make_key(double a, double b) {
    return Key{a, b};
}

struct Node {
    int x{0};
    int y{0};
    int is_obstacle{FALSE};
    int is_goal{FALSE};
    int is_temp{FALSE};
    int is_parked{FALSE};
    int reserved_by_agent{-1};
};

enum AgentState {
    IDLE,
    GOING_TO_PARK,
    RETURNING_HOME_EMPTY,
    GOING_TO_COLLECT,
    RETURNING_WITH_CAR,
    GOING_TO_CHARGE,
    CHARGING,
    RETURNING_HOME_MAINTENANCE
};

enum PhaseType { PARK_PHASE, EXIT_PHASE };

struct DynamicPhase {
    PhaseType type{PARK_PHASE};
    int task_count{0};
    char type_name[10]{};
};

enum TaskType { TASK_NONE, TASK_PARK, TASK_EXIT };

struct TaskNode {
    TaskType type{TASK_NONE};
    int created_at_step{0};
};

enum SimulationMode { MODE_UNINITIALIZED, MODE_CUSTOM, MODE_REALTIME };

enum PathAlgo {
    PATHALGO_DEFAULT = 0,
    PATHALGO_ASTAR_SIMPLE = 1,
    PATHALGO_DSTAR_BASIC = 2
};

class GridMap final {
public:
    GridMap() = default;

    Node grid[GRID_HEIGHT][GRID_WIDTH]{};
    Node* goals[MAX_GOALS]{};
    int num_goals{0};
    Node* charge_stations[MAX_CHARGE_STATIONS]{};
    int num_charge_stations{0};
};

struct SearchCell {
    double g{0.0};
    double rhs{0.0};
    Key key{};
    int in_pq{FALSE};
    int pq_index{-1};
};

struct NodePQ {
    std::array<Node*, GRID_WIDTH * GRID_HEIGHT> nodes{};
    int size{0};

    void clear() { size = 0; }
    int capacity() const { return static_cast<int>(nodes.size()); }
};

struct Pathfinder_;
struct Agent_;
class Logger;
class ScenarioManager;
class Simulation_;

using Agent = Agent_;
using Pathfinder = Pathfinder_;

struct Pathfinder_ {
    NodePQ pq{};
    SearchCell cells[GRID_HEIGHT][GRID_WIDTH]{};
    Node* start_node{nullptr};
    Node* goal_node{nullptr};
    Node* last_start{nullptr};
    double km{0.0};
    const Agent_* agent{nullptr};
    unsigned long long nodes_expanded_this_call{0};
    unsigned long long heap_moves_this_call{0};
    unsigned long long nodes_generated_this_call{0};
    unsigned long long valid_expansions_this_call{0};
};

struct ReservationTable {
    int occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]{};
};

enum CauseType { CAUSE_VERTEX = 0, CAUSE_SWAP = 1 };

struct WaitEdge {
    int from_id{0};
    int to_id{0};
    int t{0};
    CauseType cause{CAUSE_VERTEX};
    int x1{0};
    int y1{0};
    int x2{0};
    int y2{0};
};

struct CBSConstraint {
    int agent{0};
    int t{0};
    int is_edge{0};
    int x{0};
    int y{0};
    int tox{0};
    int toy{0};
};

struct CBSNode {
    CBSConstraint cons[MAX_CBS_CONS]{};
    int ncons{0};
    Node* plans[MAX_AGENTS][MAX_WHCA_HORIZON + 1]{};
    double cost{0.0};
};

struct TempMarkList {
    Node* nodes[TEMP_MARK_MAX]{};
    int count{0};
};

struct Agent_ {
    int id{0};
    char symbol{0};
    Node* pos{nullptr};
    Node* home_base{nullptr};
    Node* goal{nullptr};
    AgentState state{IDLE};
    double total_distance_traveled{0.0};
    int charge_timer{0};
    int action_timer{0};
    AgentDir heading{DIR_NONE};
    int rotation_wait{0};
    OwnedPtr<Pathfinder> pf{};
    int stuck_steps{0};
    int metrics_task_active{0};
    int metrics_task_start_step{0};
    double metrics_distance_at_start{0.0};
    int metrics_turns_current{0};
};

class AgentManager final {
public:
    AgentManager();
    ~AgentManager();

    void releasePathfinders();
    void updateStateAfterMove(ScenarioManager* scenario, GridMap* map, Logger* logger, Simulation_* sim);
    void updateChargeState(GridMap* map, Logger* logger);

    Agent agents[MAX_AGENTS]{};
    int total_cars_parked{0};
};

class ScenarioManager final {
public:
    ScenarioManager();
    ~ScenarioManager();

    void clearTaskQueue();
    void enqueueTask(TaskType type);
    bool tryDequeueAssignableTask(int lot_full, int parked_count, TaskType* out_type);
    void applySpeedMultiplier(float speedMultiplier);

    SimulationMode mode{MODE_UNINITIALIZED};
    int time_step{0};
    int simulation_speed{100};
    float speed_multiplier{1.0f};
    DynamicPhase phases[MAX_PHASES]{};
    int num_phases{0};
    int current_phase_index{0};
    int tasks_completed_in_phase{0};
    std::deque<TaskNode> task_queue{};
    int task_count{0};
    int park_chance{40};
    int exit_chance{30};
};

class Logger final {
public:
    Logger() = default;

    void log(const char* fmt, ...);
    void logV(const char* fmt, va_list args);

    char logs[LOG_BUFFER_LINES][LOG_BUFFER_WIDTH]{};
    int log_head{0};
    int log_count{0};
};

using Simulation = Simulation_;

struct AlgoRTMetrics {
    int whca_h{0};
    int wf_edges_last{0};
    long long wf_edges_sum{0};
    int scc_last{0};
    long long scc_sum{0};
    int cbs_ok_last{0};
    int cbs_exp_last{0};
    long long cbs_success_sum{0};
    long long cbs_fail_sum{0};
};

struct RuntimeTuningState {
    int whca_horizon{MIN_WHCA_HORIZON};
    int conflict_score{0};
};

struct RendererState {
    int render_stride{1};
    int fast_render{0};
    int simple_colors{0};
    int suppress_flush{0};

    void configureForAlgorithm(PathAlgo algo) {
        render_stride = (algo == PATHALGO_DEFAULT) ? 1 : 2;
    }
};

struct PlannerMetricsState {
    int whca_h{0};
    int wf_edges_last{0};
    long long wf_edges_sum{0};
    int scc_last{0};
    long long scc_sum{0};
    int cbs_ok_last{0};
    int cbs_exp_last{0};
    long long cbs_success_sum{0};
    long long cbs_fail_sum{0};
    unsigned long long whca_nodes_expanded_this_step{0};
    unsigned long long whca_heap_moves_this_step{0};
    unsigned long long astar_nodes_expanded_this_step{0};
    unsigned long long astar_heap_moves_this_step{0};
    unsigned long long whca_dstar_nodes_expanded_this_step{0};
    unsigned long long whca_dstar_heap_moves_this_step{0};
    unsigned long long dstar_nodes_expanded_this_step{0};
    unsigned long long dstar_heap_moves_this_step{0};
    unsigned long long whca_generated_nodes_this_step{0};
    unsigned long long whca_valid_expansions_this_step{0};
    unsigned long long whca_dstar_generated_nodes_this_step{0};
    unsigned long long whca_dstar_valid_expansions_this_step{0};
    unsigned long long astar_generated_nodes_this_step{0};
    unsigned long long astar_valid_expansions_this_step{0};
    unsigned long long dstar_generated_nodes_this_step{0};
    unsigned long long dstar_valid_expansions_this_step{0};

    void resetStepCounters() {
        whca_nodes_expanded_this_step = 0;
        whca_heap_moves_this_step = 0;
        astar_nodes_expanded_this_step = 0;
        astar_heap_moves_this_step = 0;
        whca_dstar_nodes_expanded_this_step = 0;
        whca_dstar_heap_moves_this_step = 0;
        dstar_nodes_expanded_this_step = 0;
        dstar_heap_moves_this_step = 0;
        whca_generated_nodes_this_step = 0;
        whca_valid_expansions_this_step = 0;
        whca_dstar_generated_nodes_this_step = 0;
        whca_dstar_valid_expansions_this_step = 0;
        astar_generated_nodes_this_step = 0;
        astar_valid_expansions_this_step = 0;
        dstar_generated_nodes_this_step = 0;
        dstar_valid_expansions_this_step = 0;
    }
};

struct AgentWorkloadSnapshot {
    int active_park_agents{0};
    int active_exit_agents{0};
};

struct StepScratch {
    std::array<Node*, MAX_AGENTS> next_positions{};
    std::array<Node*, MAX_AGENTS> previous_positions{};
    std::array<int, MAX_AGENTS> priority_order{};
    std::array<int, GRID_WIDTH * GRID_HEIGHT> cell_owner{};

    void resetCellOwner(int value = -1) {
        cell_owner.fill(value);
    }
};

class PlannerStrategy {
public:
    virtual ~PlannerStrategy() = default;
    virtual void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const = 0;
    virtual std::unique_ptr<PlannerStrategy> clone() const = 0;
};

class Planner final {
public:
    Planner() = default;
    explicit Planner(std::unique_ptr<PlannerStrategy> strategy);
    Planner(const Planner& other);
    Planner& operator=(const Planner& other);
    Planner(Planner&&) noexcept = default;
    Planner& operator=(Planner&&) noexcept = default;
    ~Planner() = default;

    void planStep(AgentManager* agents, GridMap* map, Logger* logger, Node* next_pos[MAX_AGENTS]) const;
    void reset(std::unique_ptr<PlannerStrategy> strategy);

private:
    std::unique_ptr<PlannerStrategy> strategy_{};
};

class RendererStrategy {
public:
    virtual ~RendererStrategy() = default;
    virtual void drawFrame(Simulation_* sim, int is_paused) const = 0;
    virtual std::unique_ptr<RendererStrategy> clone() const = 0;
};

class RendererFacade final {
public:
    RendererFacade() = default;
    explicit RendererFacade(std::unique_ptr<RendererStrategy> strategy);
    RendererFacade(const RendererFacade& other);
    RendererFacade& operator=(const RendererFacade& other);
    RendererFacade(RendererFacade&&) noexcept = default;
    RendererFacade& operator=(RendererFacade&&) noexcept = default;
    ~RendererFacade() = default;

    void drawFrame(Simulation_* sim, int is_paused) const;

private:
    std::unique_ptr<RendererStrategy> strategy_{};
};

class Simulation_ final {
public:
    Simulation_() = default;
    ~Simulation_();

    void destroyOwnedResources();
    void collectMemorySample();
    void collectMemorySampleAlgo();
    void resetRuntimeStats();
    void reportRealtimeDashboard();
    void planStep(Node* next_pos[MAX_AGENTS]);
    void updateState();
    void executeOneStep(int is_paused);
    bool isComplete() const;
    void run();
    void printPerformanceSummary() const;
    bool openStepMetricsFile(const char* path);
    void closeStepMetricsFile();
    void appendStepMetrics(int step_label);

    OwnedPtr<GridMap> map{};
    OwnedPtr<AgentManager> agent_manager{};
    OwnedPtr<ScenarioManager> scenario_manager{};
    OwnedPtr<Logger> logger{};
    int map_id{0};
    PathAlgo path_algo{PATHALGO_DEFAULT};
    Planner planner{};
    RendererFacade renderer{};
    RuntimeTuningState runtime_tuning{};
    RendererState render_state{};
    PlannerMetricsState planner_metrics{};
    AgentWorkloadSnapshot workload_snapshot{};
    StepScratch step_scratch{};
    std::array<char, DISPLAY_BUFFER_SIZE> display_buffer{};
    int whca_horizon_shadow{0};
    AlgoRTMetrics algo_rt_metrics_shadow{};
    double total_cpu_time_ms{0.0};
    double last_step_cpu_time_ms{0.0};
    double max_step_cpu_time_ms{0.0};
    double phase_cpu_time_ms[MAX_PHASES]{};
    int phase_step_counts[MAX_PHASES]{};
    int phase_first_step[MAX_PHASES]{};
    int phase_last_step[MAX_PHASES]{};
    int phase_completed_tasks[MAX_PHASES]{};
    double post_phase_cpu_time_ms{0.0};
    int post_phase_step_count{0};
    int post_phase_first_step{0};
    int post_phase_last_step{0};
    double total_planning_time_ms{0.0};
    double last_planning_time_ms{0.0};
    double max_planning_time_ms{0.0};
    unsigned long long tasks_completed_total{0};
    unsigned long long algorithm_operation_count{0};
    double total_movement_cost{0.0};
    unsigned long long deadlock_count{0};
    double memory_usage_sum_kb{0.0};
    double memory_usage_peak_kb{0.0};
    int memory_samples{0};
    double algo_mem_sum_kb{0.0};
    double algo_mem_peak_kb{0.0};
    int algo_mem_samples{0};
    int last_task_completion_step{0};
    int total_executed_steps{0};
    unsigned long long last_report_completed_tasks{0};
    int last_report_step{0};
    unsigned long long metrics_task_count{0};
    double metrics_sum_dmove{0.0};
    long long metrics_sum_turns{0};
    double metrics_sum_ttask{0.0};
    unsigned long long requests_created_total{0};
    unsigned long long request_wait_ticks_sum{0};
    unsigned long long algo_nodes_expanded_total{0};
    unsigned long long algo_heap_moves_total{0};
    unsigned long long algo_nodes_expanded_last_step{0};
    unsigned long long algo_heap_moves_last_step{0};
    unsigned long long algo_generated_nodes_total{0};
    unsigned long long algo_valid_expansions_total{0};
    unsigned long long algo_generated_nodes_last_step{0};
    unsigned long long algo_valid_expansions_last_step{0};
    FILE* step_metrics_stream{nullptr};
    int step_metrics_header_written{FALSE};
    int suppress_stdout{FALSE};
    unsigned int configured_seed{0};
};

struct MetricsSnapshot_ {
    int whca_h{0};
    int wf_edges_last{0};
    long long wf_edges_sum{0};
    int scc_last{0};
    long long scc_sum{0};
    int cbs_ok_last{0};
    int cbs_exp_last{0};
    long long cbs_success_sum{0};
    long long cbs_fail_sum{0};
    int whca_horizon{0};
};

using MetricsSnapshot = MetricsSnapshot_;
