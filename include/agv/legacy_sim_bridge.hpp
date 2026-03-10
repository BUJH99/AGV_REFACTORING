#pragma once

#include <cstddef>

struct Simulation_;
typedef struct Simulation_ Simulation;

constexpr int kAgvMaxPhases = 20;
constexpr int kAgvRenderBufferSize = 1024 * 1024;

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
    AgvPhaseConfig phases[kAgvMaxPhases];
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

Simulation* simulation_create();
void simulation_destroy(Simulation* sim);
int simulation_setup(Simulation* sim);
void simulation_run(Simulation* sim);
void simulation_print_performance_summary(const Simulation* sim);
void ui_enter_alt_screen(void);
void ui_leave_alt_screen(void);

void agv_default_config(AgvSimulationConfig* cfg);
void agv_prepare_console(void);
int agv_apply_config(Simulation* sim, const AgvSimulationConfig* cfg);
int agv_execute_headless_step(Simulation* sim);
int agv_run_to_completion(Simulation* sim);
int agv_is_complete(const Simulation* sim);
void agv_collect_run_summary(const Simulation* sim, AgvRunSummary* out);
int agv_copy_render_frame(Simulation* sim, int is_paused, char* buffer, std::size_t buffer_size);
int agv_open_step_metrics(Simulation* sim, const char* path);
void agv_close_step_metrics(Simulation* sim);
int agv_write_run_summary_json(const Simulation* sim, const char* path);
