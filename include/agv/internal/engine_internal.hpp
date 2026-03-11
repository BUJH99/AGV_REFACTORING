#pragma once

#include <array>
#include <cstddef>
#include <optional>

inline constexpr int DISPLAY_BUFFER_SIZE = 512000;
inline constexpr int GRID_WIDTH = 82;
inline constexpr int GRID_HEIGHT = 42;
inline constexpr int MAX_AGENTS = 16;
inline constexpr int MAX_GOALS = GRID_WIDTH * GRID_HEIGHT;
inline constexpr int MAX_CHARGE_STATIONS = 10;
inline constexpr int MAX_PHASES = 20;
inline constexpr int LOG_BUFFER_LINES = 5;
inline constexpr int LOG_BUFFER_WIDTH = 256;
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
inline constexpr int MAX_WAIT_EDGES = 128;
inline constexpr int MAX_CBS_GROUP = 8;
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

struct RunSummary {
    unsigned int seed{0};
    int map_id{0};
    PathAlgo path_algo{PathAlgo::Default};
    SimulationMode mode{SimulationMode::Custom};
    int active_agents{0};
    int recorded_steps{0};
    unsigned long long tasks_completed_total{0};
    double throughput{0.0};
    double total_movement_cost{0.0};
    unsigned long long deadlock_count{0};
    double total_cpu_time_ms{0.0};
    double avg_cpu_time_ms{0.0};
    double total_planning_time_ms{0.0};
    double avg_planning_time_ms{0.0};
    double memory_usage_sum_kb{0.0};
    double avg_memory_usage_kb{0.0};
    double memory_usage_peak_kb{0.0};
    unsigned long long algo_nodes_expanded_total{0};
    unsigned long long algo_heap_moves_total{0};
    unsigned long long algo_generated_nodes_total{0};
    unsigned long long algo_valid_expansions_total{0};
    double valid_expansion_ratio{0.0};
    unsigned long long requests_created_total{0};
    unsigned long long request_wait_ticks_sum{0};
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
void agv_update_deadlock_counter(Simulation* sim, bool moved_this_step, bool is_custom_mode);
void agv_accumulate_wait_ticks_if_realtime(Simulation* sim);
void agv_execute_step_service(Simulation* sim, bool is_paused);

SimulationConfig default_simulation_config();
bool apply_simulation_config(Simulation* sim, const SimulationConfig& config);
bool execute_headless_step(Simulation* sim);
bool run_simulation_to_completion(Simulation* sim);
RunSummary collect_run_summary(const Simulation* sim);
std::string build_render_frame_text(Simulation* sim, bool is_paused);

void agv_prepare_console();
void ui_enter_alt_screen();
void ui_leave_alt_screen();
void platform_sleep_for_ms(int ms);
int console_read_key_blocking();
std::optional<int> console_read_key_nonblocking();
int simulation_setup(Simulation* sim);
void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
