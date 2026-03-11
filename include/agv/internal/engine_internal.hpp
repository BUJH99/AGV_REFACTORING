#pragma once

#include <array>
#include <cstddef>

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

enum AgentDir {
    DIR_NONE = -1,
    DIR_UP = 0,
    DIR_RIGHT = 1,
    DIR_DOWN = 2,
    DIR_LEFT = 3
};

#include "agv/internal/engine_model.hpp"

struct ConfigPhase {
    PhaseType type{PARK_PHASE};
    int task_count{1};
};

struct SimulationConfig {
    unsigned int seed{0};
    int map_id{1};
    PathAlgo path_algo{PATHALGO_DEFAULT};
    SimulationMode mode{MODE_CUSTOM};
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
    PathAlgo path_algo{PATHALGO_DEFAULT};
    SimulationMode mode{MODE_CUSTOM};
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

int agv_current_whca_horizon();
int agv_current_conflict_score();
void agv_set_whca_runtime_state(int conflict_score, int horizon);
AgentWorkloadSnapshot agv_collect_agent_workload(const AgentManager* am);
Node* agv_select_best_charge_station(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_set_goal_if_needed(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_update_task_dispatch(Simulation* sim);
int agv_apply_moves_and_update_stuck(Simulation* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]);
void agv_update_deadlock_counter(Simulation* sim, int moved_this_step, int is_custom_mode);
void agv_accumulate_wait_ticks_if_realtime(Simulation* sim);
void agv_execute_step_service(Simulation* sim, int is_paused);
void agv_accumulate_cbs_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions);
void agv_accumulate_whca_dstar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions);
void agv_accumulate_astar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions);
void agv_accumulate_dstar_step_metrics(
    unsigned long long nodes_expanded,
    unsigned long long heap_moves,
    unsigned long long generated_nodes,
    unsigned long long valid_expansions);
void agv_record_wf_scc_metrics(int wf_edges, int scc);
void agv_record_cbs_success(int expansions);
void agv_record_cbs_failure(int expansions);

SimulationConfig default_simulation_config();
bool apply_simulation_config(Simulation* sim, const SimulationConfig& config);
bool execute_headless_step(Simulation* sim);
bool run_simulation_to_completion(Simulation* sim);
RunSummary collect_run_summary(const Simulation* sim);
int copy_render_frame(Simulation* sim, bool is_paused, char* buffer, std::size_t buffer_size);

void agv_prepare_console();
void ui_enter_alt_screen();
void ui_leave_alt_screen();
int simulation_setup(Simulation* sim);
void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id);
