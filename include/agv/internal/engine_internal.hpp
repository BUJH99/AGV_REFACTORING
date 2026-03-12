#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

inline constexpr int DISPLAY_BUFFER_SIZE = 512000;
inline constexpr int GRID_WIDTH = 82;
inline constexpr int GRID_HEIGHT = 42;
inline constexpr int MAX_AGENTS = 16;
inline constexpr int MAX_GOALS = GRID_WIDTH * GRID_HEIGHT;
inline constexpr int MAX_CHARGE_STATIONS = 10;
inline constexpr int MAX_PHASES = 20;
inline constexpr int LOG_BUFFER_LINES = 5;
inline constexpr int LOG_BUFFER_WIDTH = 256;
inline constexpr std::size_t RENDER_LOG_TAIL_LINES = 8;
inline constexpr std::size_t RENDER_LOG_BATCH_LIMIT = 256;
inline constexpr int MAX_WHCA_HORIZON = 11;
inline constexpr int MIN_WHCA_HORIZON = 5;
inline constexpr int MAX_CBS_CONS = 128;
inline constexpr int TEMP_MARK_MAX = 128;
inline constexpr int INPUT_BUFFER_SIZE = 500;
inline constexpr int NUM_DIRECTIONS = 4;
inline constexpr int DIR4_COUNT = 4;
inline constexpr int DIR5_COUNT = 5;
inline constexpr double DISTANCE_BEFORE_CHARGE = 300.0;
inline constexpr int CHARGE_TIME = 20;
inline constexpr int REALTIME_MODE_TIMELIMIT = 1000000;
inline constexpr int DASHBOARD_INTERVAL_STEPS = 2500;
inline constexpr int MAX_TASKS = 50;
inline constexpr float MAX_SPEED_MULTIPLIER = 10000.0f;
inline constexpr int EVENT_GENERATION_INTERVAL = 10;
inline constexpr int CLEANUP_FORCE_IDLE_AFTER_STEPS = 11;
inline constexpr int TURN_90_WAIT = 2;
inline constexpr int STATUS_STRING_WIDTH = 25;
inline constexpr int PAUSE_POLL_INTERVAL_MS = 50;
inline constexpr int RENDER_STRIDE_MAX = 8;
inline constexpr int RENDER_STRIDE_MIN = 1;
inline constexpr int TASK_ACTION_TICKS = 10;
inline constexpr int PRIORITY_RETURNING_WITH_CAR = 3;
inline constexpr int PRIORITY_GOING_TO_CHARGE = 2;
inline constexpr int PRIORITY_MOVING_TASK = 1;
inline constexpr int STUCK_BOOST_MULT = 10;
inline constexpr int STUCK_BOOST_HARD = 1000;
inline constexpr int DEADLOCK_THRESHOLD = 5;
inline constexpr int MAX_WAIT_EDGES = 512;
inline constexpr int MAX_CBS_GROUP = MAX_AGENTS;
inline constexpr int MAX_CBS_NODES = 1024;
inline constexpr int CBS_MAX_EXPANSIONS = 128;
inline constexpr int MAX_TOT = (MAX_WHCA_HORIZON + 1) * GRID_WIDTH * GRID_HEIGHT;
inline constexpr double INF = 1e18;

inline constexpr const char* C_NRM = "\x1b[0m";
inline constexpr const char* C_RED = "\x1b[31m";
inline constexpr const char* C_GRN = "\x1b[32m";
inline constexpr const char* C_YEL = "\x1b[33m";
inline constexpr const char* C_BLU = "\x1b[34m";
inline constexpr const char* C_MAG = "\x1b[35m";
inline constexpr const char* C_CYN = "\x1b[36m";
inline constexpr const char* C_WHT = "\x1b[37m";
inline constexpr const char* C_GRY = "\x1b[90m";
inline constexpr const char* C_B_RED = "\x1b[1;31m";
inline constexpr const char* C_B_GRN = "\x1b[1;32m";
inline constexpr const char* C_B_YEL = "\x1b[1;33m";
inline constexpr const char* C_B_MAG = "\x1b[1;35m";
inline constexpr const char* C_B_CYN = "\x1b[1;36m";
inline constexpr const char* C_B_WHT = "\x1b[1;37m";

enum class AgentDir {
    None = -1,
    Up = 0,
    Right = 1,
    Down = 2,
    Left = 3
};

#include "agv/internal/engine_model.hpp"

inline bool grid_is_valid_coord(int x, int y) {
    return x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT;
}

bool grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const Agent* agent);

struct ConfigPhase {
    PhaseType type{PhaseType::Park};
    int task_count{1};
};

struct SimulationConfig {
    unsigned int seed{0};
    int map_id{1};
    PathAlgo path_algo{PathAlgo::Default};
    SimulationMode mode{SimulationMode::Custom};
    float speed_multiplier{0.0f};
    int realtime_park_chance{0};
    int realtime_exit_chance{0};
    int num_phases{1};
    std::array<ConfigPhase, MAX_PHASES> phases{};
    bool suppress_stdout{false};
};

struct DistributionSummary {
    double min{0.0};
    double avg{0.0};
    double max{0.0};
    double stddev{0.0};
    double coefficient_of_variation{0.0};
    double min_max_ratio{0.0};
};

struct AgentFairnessRecord {
    int id{0};
    char symbol{0};
    unsigned long long tasks_completed{0};
    double distance_cells{0.0};
    unsigned long long idle_steps{0};
};

struct RunSummary {
    unsigned int seed{0};
    int map_id{0};
    PathAlgo path_algo{PathAlgo::Default};
    SimulationMode mode{SimulationMode::Custom};
    int active_agents{0};
    int recorded_steps{0};
    unsigned long long tasks_completed_total{0};
    double throughput{0.0};
    double tasks_per_agent{0.0};
    double total_movement_cost{0.0};
    unsigned long long deadlock_count{0};
    double total_cpu_time_ms{0.0};
    double avg_cpu_time_ms{0.0};
    double max_step_cpu_time_ms{0.0};
    double avg_cpu_time_per_task_ms{0.0};
    double tasks_per_cpu_second{0.0};
    double total_planning_time_ms{0.0};
    double avg_planning_time_ms{0.0};
    double max_planning_time_ms{0.0};
    double planning_cpu_share{0.0};
    double avg_planning_time_per_task_ms{0.0};
    double tasks_per_planning_second{0.0};
    double memory_usage_sum_kb{0.0};
    double avg_memory_usage_kb{0.0};
    double memory_usage_peak_kb{0.0};
    double avg_memory_usage_per_agent_kb{0.0};
    unsigned long long algo_nodes_expanded_total{0};
    unsigned long long algo_heap_moves_total{0};
    unsigned long long algo_generated_nodes_total{0};
    unsigned long long algo_valid_expansions_total{0};
    double valid_expansion_ratio{0.0};
    double avg_nodes_expanded_per_step{0.0};
    double avg_nodes_expanded_per_task{0.0};
    double nodes_expanded_per_planning_ms{0.0};
    double heap_moves_per_node_expanded{0.0};
    unsigned long long requests_created_total{0};
    unsigned long long request_wait_ticks_sum{0};
    double avg_request_wait_ticks{0.0};
    double avg_movement_per_task{0.0};
    double non_task_movement_cost{0.0};
    double task_movement_coverage_ratio{0.0};
    double non_task_movement_ratio{0.0};
    double avg_task_distance{0.0};
    double avg_task_turns{0.0};
    double avg_task_steps{0.0};
    double avg_task_steps_per_cell{0.0};
    double avg_task_turns_per_100_cells{0.0};
    unsigned long long steps_with_movement{0};
    unsigned long long stall_step_count{0};
    double movement_step_ratio{0.0};
    double stall_step_ratio{0.0};
    unsigned long long max_no_movement_streak{0};
    int last_task_completion_step{0};
    int steps_since_last_task_completion{0};
    int queued_task_count{0};
    int in_flight_task_count{0};
    int outstanding_task_count{0};
    double avg_outstanding_task_count{0.0};
    int peak_outstanding_task_count{0};
    double avg_outstanding_tasks_per_agent{0.0};
    double peak_outstanding_tasks_per_agent{0.0};
    int oldest_queued_request_age{0};
    double avg_oldest_queued_request_age{0.0};
    int peak_oldest_queued_request_age{0};
    long long planner_wait_edges_sum{0};
    long long planner_conflict_cycle_total{0};
    long long planner_wait_edge_step_count{0};
    long long planner_cycle_step_count{0};
    double planner_wait_edges_per_step{0.0};
    double planner_wait_edges_per_conflict_step{0.0};
    double planner_cycle_step_ratio{0.0};
    long long planner_cbs_attempt_count{0};
    long long planner_cbs_success_count{0};
    long long planner_cbs_failure_count{0};
    double planner_cbs_attempt_rate{0.0};
    double planner_cbs_success_rate{0.0};
    double planner_cbs_failure_rate{0.0};
    DistributionSummary tasks_per_agent_spread{};
    DistributionSummary distance_per_agent_spread{};
    DistributionSummary idle_steps_per_agent_spread{};
    std::vector<AgentFairnessRecord> agent_fairness_breakdown{};
    int remaining_parked_vehicles{0};
};

AgentWorkloadSnapshot agv_collect_agent_workload(const AgentManager* am);
Node* agv_select_best_charge_station(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_set_goal_if_needed(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_update_task_dispatch(Simulation* sim);
void agent_manager_update_state_after_move(
    AgentManager* manager,
    ScenarioManager* scenario,
    GridMap* map,
    Logger* logger,
    Simulation* sim);
void agent_manager_update_charge_state(AgentManager* manager, GridMap* map, Logger* logger);
bool agv_apply_moves_and_update_stuck(Simulation* sim, AgentNodeSlots& next_positions, AgentNodeSlots& previous_positions);
void agv_update_deadlock_counter(Simulation* sim, const AgentNodeSlots& next_positions, bool moved_this_step, bool is_custom_mode);
void agv_accumulate_wait_ticks_if_realtime(Simulation* sim);
void agv_execute_step_service(Simulation* sim, bool is_paused);

SimulationConfig default_simulation_config();
bool apply_simulation_config(Simulation* sim, const SimulationConfig& config);
void simulation_set_speed_multiplier(Simulation* sim, double speed_multiplier);
bool execute_headless_step(Simulation* sim, bool allow_sleep = true);
bool run_simulation_to_completion(Simulation* sim);
RunSummary collect_run_summary(const Simulation* sim);
std::string build_render_frame_text(Simulation* sim, bool is_paused);
std::vector<std::string> collect_recent_logs(const Logger* logger);
std::vector<agv::core::StructuredLogEntry> collect_structured_logs(
    const Logger* logger,
    std::uint64_t since_seq = 0,
    std::size_t max_entries = RENDER_LOG_BATCH_LIMIT);
void render_model_reset(Simulation* sim, std::uint64_t session_id);
void render_model_capture_advanced_frame(Simulation* sim);
agv::core::StaticSceneSnapshot snapshot_static_scene(const Simulation* sim);
agv::core::RenderFrameSnapshot snapshot_render_frame(Simulation* sim, const agv::core::RenderQueryOptions& options);
agv::core::RenderFrameDelta snapshot_render_delta(
    Simulation* sim,
    std::uint64_t since_frame_id,
    const agv::core::RenderQueryOptions& options);
std::vector<agv::core::StructuredLogEntry> snapshot_structured_logs(
    Simulation* sim,
    std::uint64_t since_seq,
    std::size_t max_entries);

struct ConsoleSize {
    int columns{0};
    int rows{0};

    bool operator==(const ConsoleSize& other) const {
        return columns == other.columns && rows == other.rows;
    }

    bool operator!=(const ConsoleSize& other) const {
        return !(*this == other);
    }
};

void agv_prepare_console();
void ui_handle_control_key(
    Simulation* sim,
    int ch,
    bool& is_paused,
    bool& quit_flag,
    bool& return_to_menu);
void ui_enter_alt_screen();
void ui_leave_alt_screen();
void platform_sleep_for_ms(int ms);
int console_read_key_blocking();
std::optional<int> console_read_key_nonblocking();
std::optional<ConsoleSize> console_current_size();
int simulation_setup(Simulation* sim);
void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
