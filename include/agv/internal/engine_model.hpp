#pragma once

#include "agv/simulation_engine.hpp"
#include "agv/internal/text_format.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <vector>

// Private engine type model.
// This header is intentionally included from src/core/engine_orchestrator.cpp
// after the engine-wide compile-time constants/macros are defined.

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
    bool is_obstacle{false};
    bool is_goal{false};
    bool is_temp{false};
    bool is_parked{false};
    int reserved_by_agent{-1};
};

enum class AgentState {
    Idle,
    GoingToPark,
    ReturningHomeEmpty,
    GoingToCollect,
    ReturningWithCar,
    GoingToCharge,
    Charging,
    ReturningHomeMaintenance
};

enum class PhaseType { Park, Exit };

struct DynamicPhase {
    PhaseType type{PhaseType::Park};
    int task_count{0};
    std::string type_name{};
};

enum class TaskType { None, Park, Exit };

struct TaskNode {
    TaskType type{TaskType::None};
    int created_at_step{0};
};

enum class SimulationMode { Uninitialized, Custom, Realtime };

enum class PathAlgo {
    Default = 0,
    AStarSimple = 1,
    DStarBasic = 2
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

struct Agent_;
class AgentManager;
class Logger;
class ScenarioManager;
class Simulation_;

using Agent = Agent_;

struct PathfinderRunMetrics final {
    unsigned long long nodes_expanded{0};
    unsigned long long heap_moves{0};
    unsigned long long generated_nodes{0};
    unsigned long long valid_expansions{0};
};

class Pathfinder final {
public:
    Pathfinder() = default;
    Pathfinder(Node* start, Node* goal, const Agent_* agent);

    void reinitializeForGoal(Node* new_goal);
    void updateStart(Node* new_start);
    void notifyCellChange(GridMap* map, const AgentManager* am, Node* changed);
    void computeShortestPath(GridMap* map, const AgentManager* am);
    Node* getNextStep(GridMap* map, const AgentManager* am, Node* current);

    Node* goalNode() const { return goal_node_; }
    const Agent_* ownerAgent() const { return agent_; }
    double gCost(const Node* node) const;
    const PathfinderRunMetrics& lastRunMetrics() const { return last_run_metrics_; }
    void resetLastRunMetrics();

private:
    struct SearchCell final {
        double g{0.0};
        double rhs{0.0};
        Key key{};
        bool in_pq{false};
        int pq_index{-1};
        std::uint16_t generation{0};
    };

    class NodeHeap final {
    public:
        void clear() { size_ = 0; }
        int size() const { return size_; }
        int capacity() const { return static_cast<int>(nodes_.size()); }
        Node*& at(int index) { return nodes_[index]; }
        Node* at(int index) const { return nodes_[index]; }

    private:
        std::array<Node*, GRID_WIDTH * GRID_HEIGHT> nodes_{};
        int size_{0};

        friend class Pathfinder;
    };

    static int compareKeys(Key lhs, Key rhs);
    static double heuristic(const Node* lhs, const Node* rhs);
    static SearchCell makeInactiveSearchCell();

    SearchCell* cell(const Node* node);
    const SearchCell* cell(const Node* node) const;
    Key keyFor(const Node* node) const;
    bool heapContains(const Node* node) const;
    Key topKey() const;
    void pushNode(Node* node);
    Node* popNode();
    void removeNode(Node* node);
    void swapHeapNodes(int lhs_index, int rhs_index);
    void heapifyUp(int index);
    void heapifyDown(int index);
    Key calculateKey(const Node* node) const;
    void updateVertex(GridMap* map, const AgentManager* am, Node* node);
    void resetAllCells();
    void resetCoreState(Node* start, Node* goal);

    NodeHeap heap_{};
    SearchCell cells_[GRID_HEIGHT][GRID_WIDTH]{};
    Node* start_node_{nullptr};
    Node* goal_node_{nullptr};
    Node* last_start_{nullptr};
    double km_{0.0};
    const Agent_* agent_{nullptr};
    PathfinderRunMetrics last_run_metrics_{};
    std::uint16_t generation_{1};
};

class ReservationTable final {
public:
    ReservationTable();

    void clear();
    void clearAgent(int agent_id, int horizon);
    void seedCurrent(AgentManager* manager);
    bool isOccupied(int t, const Node* node, int horizon) const;
    int occupantAt(int t, const Node* node, int horizon) const;
    int occupantAt(int t, int y, int x, int horizon) const;
    void setOccupant(int t, const Node* node, int agent_id, int horizon);

private:
    static int flatIndex(int t, int y, int x);
    void clearTouchedEntries();
    void rememberTouchedIndex(int index);

    std::array<std::int16_t, MAX_TOT> occ_{};
    std::array<std::uint16_t, MAX_TOT> touched_indices_{};
    std::array<std::uint64_t, (MAX_TOT + 63) / 64> touched_bits_{};
    int touched_count_{0};
};

enum class CauseType { Vertex = 0, Swap = 1 };

struct WaitEdge {
    int from_id{0};
    int to_id{0};
    int t{0};
    CauseType cause{CauseType::Vertex};
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
    Node* plans[MAX_CBS_GROUP][MAX_WHCA_HORIZON + 1]{};
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
    Node* last_pos{nullptr};
    Node* home_base{nullptr};
    Node* goal{nullptr};
    AgentState state{AgentState::Idle};
    double total_distance_traveled{0.0};
    int charge_timer{0};
    int action_timer{0};
    AgentDir heading{AgentDir::None};
    int rotation_wait{0};
    std::unique_ptr<Pathfinder> pf{};
    int stuck_steps{0};
    int oscillation_steps{0};
    bool metrics_task_active{false};
    int metrics_task_start_step{0};
    double metrics_distance_at_start{0.0};
    int metrics_turns_current{0};
    unsigned long long metrics_completed_tasks_total{0};
    double metrics_total_distance_all_time{0.0};
    unsigned long long metrics_idle_steps_total{0};
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

    SimulationMode mode{SimulationMode::Uninitialized};
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

class Simulation_;

struct LoggerMessageMeta final {
    std::string category{"General"};
    std::string level{"Info"};
    std::optional<int> agentId{};
    std::optional<int> phaseIndex{};
};

class Logger final {
public:
    static constexpr std::size_t kStructuredLogHistoryLimit = 512;

    Logger() = default;

    void bindSimulation(Simulation_* simulation);
    void setContext(int step, std::uint64_t frame_id, int phase_index);
    void appendLine(std::string_view message);
    void appendStructuredLine(const LoggerMessageMeta& meta, std::string_view message);

    std::array<std::string, LOG_BUFFER_LINES> logs{};
    int log_head{0};
    int log_count{0};
    std::deque<agv::core::StructuredLogEntry> structured_logs{};
    std::uint64_t next_structured_seq{1};
    Simulation_* owner{nullptr};
    int context_step{0};
    std::uint64_t context_frame_id{0};
    int context_phase_index{-1};
};

template <typename... Args>
inline void logger_log(Logger* logger, std::string_view format, Args&&... args) {
    if (!logger) {
        return;
    }
    logger->appendLine(agv::internal::text::printf_like(format, std::forward<Args>(args)...));
}

template <typename... Args>
inline void logger_log_event(
    Logger* logger,
    std::string_view category,
    std::string_view level,
    std::optional<int> agent_id,
    std::optional<int> phase_index,
    std::string_view format,
    Args&&... args) {
    if (!logger) {
        return;
    }

    LoggerMessageMeta meta;
    meta.category = std::string(category);
    meta.level = std::string(level);
    meta.agentId = agent_id;
    meta.phaseIndex = phase_index;
    logger->appendStructuredLine(meta, agv::internal::text::printf_like(format, std::forward<Args>(args)...));
}

using Simulation = Simulation_;

struct RuntimeTuningState {
    int whca_horizon{MIN_WHCA_HORIZON};
    int conflict_score{0};
};

struct RendererState {
    int render_stride{1};
    bool fast_render{false};
    bool simple_colors{false};
    bool suppress_flush{false};
    bool force_next_flush{false};

    void configureForAlgorithm(PathAlgo algo) {
        render_stride = (algo == PathAlgo::Default) ? 1 : 2;
    }
};

struct PlannerMetricsState {
    int whca_h{0};
    int wf_edges_last{0};
    long long wf_edges_sum{0};
    long long wf_step_count{0};
    int scc_last{0};
    long long scc_sum{0};
    long long scc_step_count{0};
    int cbs_ok_last{0};
    int cbs_exp_last{0};
    long long cbs_attempt_sum{0};
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
        wf_edges_last = 0;
        scc_last = 0;
        cbs_ok_last = 0;
        cbs_exp_last = 0;
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

struct DeadlockEventRecord final {
    bool valid{false};
    int step{0};
    unsigned long long deadlock_count{0};
    int phase_index{-1};
    int phase_task_target{0};
    int phase_tasks_completed{0};
    int pending_task_count{0};
    int active_agent_count{0};
    int waiting_agent_count{0};
    int stuck_agent_count{0};
    int planner_wait_edges{0};
    int planner_scc_count{0};
    int planner_cbs_succeeded{0};
    int planner_cbs_expansions{0};
    int whca_horizon{0};
    int planned_move_count{0};
    int post_rotation_move_count{0};
    int post_blocker_move_count{0};
    int final_move_count{0};
    std::array<int, MAX_AGENTS> rotation_canceled_agent_ids{};
    int rotation_canceled_count{0};
    std::array<int, MAX_AGENTS> blocker_canceled_agent_ids{};
    int blocker_canceled_count{0};
    std::array<int, MAX_AGENTS> order_canceled_agent_ids{};
    int order_canceled_count{0};
    std::array<int, MAX_AGENTS> participant_agent_ids{};
    int participant_count{0};
    std::string reason{};
};

struct AgentWorkloadSnapshot {
    int active_park_agents{0};
    int active_exit_agents{0};
};

using AgentNodeSlots = std::array<Node*, MAX_AGENTS>;
using AgentOrder = std::array<int, MAX_AGENTS>;
using TimedNodePlan = std::array<Node*, MAX_WHCA_HORIZON + 1>;

class AgentMask final {
public:
    AgentMask() = default;
    explicit AgentMask(int raw_bits)
        : bits_(raw_bits) {}

    static AgentMask all() {
        if (MAX_AGENTS >= static_cast<int>(sizeof(int) * 8)) {
            return AgentMask(-1);
        }
        return AgentMask((1 << MAX_AGENTS) - 1);
    }

    bool contains(int agent_id) const {
        return agent_id >= 0 && agent_id < MAX_AGENTS && (bits_ & (1 << agent_id)) != 0;
    }

    void set(int agent_id) {
        if (agent_id >= 0 && agent_id < MAX_AGENTS) {
            bits_ |= (1 << agent_id);
        }
    }

    void clear(int agent_id) {
        if (agent_id >= 0 && agent_id < MAX_AGENTS) {
            bits_ &= ~(1 << agent_id);
        }
    }

    bool empty() const { return bits_ == 0; }
    int raw() const { return bits_; }

private:
    int bits_{0};
};

struct OrderedMoveCandidates final {
    std::array<Node*, 5> nodes{};
    int count{0};

    void add(Node* node) {
        if (count < static_cast<int>(nodes.size())) {
            nodes[count++] = node;
        }
    }

    void clear() {
        nodes.fill(nullptr);
        count = 0;
    }
};

struct WaitEdgeBuffer final {
    std::array<WaitEdge, MAX_WAIT_EDGES> edges{};
    int count{0};

    void clear() { count = 0; }

    void add(int from, int to, int t, CauseType cause, int x1, int y1, int x2, int y2) {
        if (count >= static_cast<int>(edges.size())) return;
        WaitEdge& edge = edges[count++];
        edge.from_id = from;
        edge.to_id = to;
        edge.t = t;
        edge.cause = cause;
        edge.x1 = x1;
        edge.y1 = y1;
        edge.x2 = x2;
        edge.y2 = y2;
    }
};

struct ConflictGraphSummary final {
    AgentMask scc_agents{};
    int wait_edge_count{0};

    bool hasCycle() const { return !scc_agents.empty(); }
};

struct CbsPlanBuffer final {
    std::array<TimedNodePlan, MAX_AGENTS> plans{};

    void clear() {
        for (TimedNodePlan& plan : plans) {
            plan.fill(nullptr);
        }
    }

    TimedNodePlan& operator[](int agent_id) { return plans[agent_id]; }
    const TimedNodePlan& operator[](int agent_id) const { return plans[agent_id]; }
};

struct SpaceTimeSearchBuffers final {
    std::array<double, MAX_TOT> g{};
    std::array<double, MAX_TOT> f{};
    std::array<unsigned char, MAX_TOT> open{};
    std::array<unsigned char, MAX_TOT> closed{};
    std::array<int, MAX_TOT> prev{};
    std::array<int, MAX_TOT> heap_nodes{};
    std::array<int, MAX_TOT> heap_pos{};
};

struct CbsSolveResult final {
    bool solved{false};
    int expansions{0};
    CbsPlanBuffer plans{};
};

struct FallbackDecision final {
    int leader{-1};
    AgentMask yield_agents{};
    AgentMask pull_over_agents{};
    bool used_cbs{false};

    bool hasAction() const {
        return used_cbs || leader >= 0 || !yield_agents.empty() || !pull_over_agents.empty();
    }
};

struct DefaultPlannerScratch final {
    WaitEdgeBuffer wait_edges{};
    std::array<int, MAX_CBS_GROUP> group_ids{};
    std::array<int, MAX_CBS_GROUP> masked_group_ids{};
    std::array<int, MAX_CBS_GROUP> fallback_group_ids{};
    SpaceTimeSearchBuffers pull_over_search{};
    SpaceTimeSearchBuffers cbs_search{};
    std::int16_t ext_occ[MAX_WHCA_HORIZON + 1][GRID_HEIGHT][GRID_WIDTH]{};
    std::array<CBSNode, MAX_CBS_NODES> cbs_nodes{};
    std::array<int, MAX_CBS_NODES> cbs_heap_indices{};

    void clear() {
        wait_edges.clear();
        group_ids.fill(-1);
        masked_group_ids.fill(-1);
        fallback_group_ids.fill(-1);
    }
};

struct StepScratch {
    StepScratch() {
        cell_owner.fill(-1);
    }

    AgentNodeSlots next_positions{};
    AgentNodeSlots previous_positions{};
    AgentNodeSlots planner_positions{};
    AgentNodeSlots post_rotation_positions{};
    AgentNodeSlots post_blocker_positions{};
    AgentOrder priority_order{};
    std::array<int, GRID_WIDTH * GRID_HEIGHT> cell_owner{};
    std::array<int, MAX_AGENTS> touched_cell_indices{};
    int touched_cell_count{0};
    int planned_move_count{0};
    int post_rotation_move_count{0};
    int post_blocker_move_count{0};
    int final_move_count{0};
    std::array<int, MAX_AGENTS> rotation_canceled_agent_ids{};
    int rotation_canceled_count{0};
    std::array<int, MAX_AGENTS> blocker_canceled_agent_ids{};
    int blocker_canceled_count{0};
    std::array<int, MAX_AGENTS> order_canceled_agent_ids{};
    int order_canceled_count{0};

    void clearTouchedCellOwner() {
        for (int i = 0; i < touched_cell_count; ++i) {
            const int index = touched_cell_indices[i];
            if (index >= 0 && index < static_cast<int>(cell_owner.size())) {
                cell_owner[index] = -1;
            }
        }
        touched_cell_count = 0;
    }

    void setCellOwner(int index, int value) {
        if (index < 0 || index >= static_cast<int>(cell_owner.size())) {
            return;
        }
        if (cell_owner[index] == -1 && touched_cell_count < MAX_AGENTS) {
            touched_cell_indices[touched_cell_count++] = index;
        }
        cell_owner[index] = value;
    }

    void resetPlanDebug() {
        planner_positions.fill(nullptr);
        post_rotation_positions.fill(nullptr);
        post_blocker_positions.fill(nullptr);
        planned_move_count = 0;
        post_rotation_move_count = 0;
        post_blocker_move_count = 0;
        final_move_count = 0;
        rotation_canceled_agent_ids.fill(-1);
        rotation_canceled_count = 0;
        blocker_canceled_agent_ids.fill(-1);
        blocker_canceled_count = 0;
        order_canceled_agent_ids.fill(-1);
        order_canceled_count = 0;
    }
};

struct PlannerOverlayCapture final {
    bool valid{false};
    PathAlgo algorithm{PathAlgo::Default};
    int horizon{0};
    int wait_edge_count{0};
    AgentMask scc_agents{};
    int leader_agent_id{-1};
    bool used_cbs{false};
    AgentMask yield_agents{};
    AgentMask pull_over_agents{};
    WaitEdgeBuffer wait_edges{};
    CbsPlanBuffer planned_paths{};
    CbsPlanBuffer cbs_paths{};

    void clear() {
        valid = false;
        algorithm = PathAlgo::Default;
        horizon = 0;
        wait_edge_count = 0;
        scc_agents = {};
        leader_agent_id = -1;
        used_cbs = false;
        yield_agents = {};
        pull_over_agents = {};
        wait_edges.clear();
        planned_paths.clear();
        cbs_paths.clear();
    }
};

struct RenderModelCache final {
    static constexpr std::size_t kDeltaHistoryLimit = 256;

    std::uint64_t session_id{0};
    std::uint64_t scene_version{0};
    std::uint64_t frame_id{0};
    agv::core::RenderFrameSnapshot last_advanced_frame{};
    bool has_last_advanced_frame{false};
    std::deque<agv::core::RenderFrameDelta> recent_deltas{};
    PlannerOverlayCapture planner_overlay{};

    void reset(std::uint64_t new_session_id) {
        session_id = new_session_id;
        scene_version = 1;
        frame_id = 0;
        last_advanced_frame = {};
        has_last_advanced_frame = false;
        recent_deltas.clear();
        planner_overlay.clear();
    }
};

struct PlanningContext final {
    Simulation_* sim{nullptr};
    AgentManager* agents{nullptr};
    GridMap* map{nullptr};
    Logger* logger{nullptr};
    RuntimeTuningState* runtime_tuning{nullptr};
    PlannerMetricsState* planner_metrics{nullptr};

    int whcaHorizon() const {
        return runtime_tuning ? runtime_tuning->whca_horizon : MIN_WHCA_HORIZON;
    }

    int conflictScore() const {
        return runtime_tuning ? runtime_tuning->conflict_score : 0;
    }
};

class PlannerStrategy {
public:
    virtual ~PlannerStrategy() = default;
    virtual void planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const = 0;
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

    void planStep(const PlanningContext& context, AgentNodeSlots& next_positions) const;
    void reset(std::unique_ptr<PlannerStrategy> strategy);

private:
    std::unique_ptr<PlannerStrategy> strategy_{};
};

class RendererStrategy {
public:
    virtual ~RendererStrategy() = default;
    virtual void drawFrame(Simulation_* sim, bool is_paused) const = 0;
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

    void drawFrame(Simulation_* sim, bool is_paused) const;

private:
    std::unique_ptr<RendererStrategy> strategy_{};
};

class Simulation_ final {
public:
    Simulation_();
    ~Simulation_();

    void collectMemorySample();
    void collectMemorySampleAlgo();
    void reseedRandom(unsigned int seed);
    int nextRandomInt(int exclusive_upper_bound);
    void resetRuntimeStats();
    void reportRealtimeDashboard();
    void planStep(AgentNodeSlots& next_positions);
    void updateState();
    void executeOneStep(bool is_paused);
    bool isComplete() const;
    bool run();
    void printPerformanceSummary() const;

    GridMap map_storage{};
    AgentManager agent_manager_storage{};
    ScenarioManager scenario_manager_storage{};
    Logger logger_storage{};
    GridMap* map{&map_storage};
    AgentManager* agent_manager{&agent_manager_storage};
    ScenarioManager* scenario_manager{&scenario_manager_storage};
    Logger* logger{&logger_storage};
    int map_id{0};
    PathAlgo path_algo{PathAlgo::Default};
    Planner planner{};
    RendererFacade renderer{};
    RuntimeTuningState runtime_tuning{};
    RendererState render_state{};
    RenderModelCache render_model{};
    PlannerMetricsState planner_metrics{};
    DeadlockEventRecord last_deadlock_event{};
    AgentWorkloadSnapshot workload_snapshot{};
    StepScratch step_scratch{};
    std::string display_buffer{};
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
    int no_movement_streak{0};
    int max_no_movement_streak{0};
    unsigned long long steps_with_movement{0};
    unsigned long long stall_step_count{0};
    int last_active_agent_count{0};
    int last_waiting_agent_count{0};
    int last_stuck_agent_count{0};
    int last_oscillating_agent_count{0};
    int last_action_agent_count{0};
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
    unsigned long long outstanding_task_sum{0};
    unsigned long long outstanding_task_samples{0};
    int outstanding_task_peak{0};
    int oldest_request_age_last{0};
    unsigned long long oldest_request_age_sum{0};
    int oldest_request_age_peak{0};
    unsigned long long algo_nodes_expanded_total{0};
    unsigned long long algo_heap_moves_total{0};
    unsigned long long algo_nodes_expanded_last_step{0};
    unsigned long long algo_heap_moves_last_step{0};
    unsigned long long algo_generated_nodes_total{0};
    unsigned long long algo_valid_expansions_total{0};
    unsigned long long algo_generated_nodes_last_step{0};
    unsigned long long algo_valid_expansions_last_step{0};
    bool suppress_stdout{false};
    unsigned int configured_seed{0};
    std::mt19937 random_engine{};
};
