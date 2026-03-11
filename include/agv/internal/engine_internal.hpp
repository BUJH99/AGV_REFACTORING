#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef DISPLAY_BUFFER_SIZE
#define DISPLAY_BUFFER_SIZE 512000
#endif

#ifndef GRID_WIDTH
#define GRID_WIDTH 82
#endif

#ifndef GRID_HEIGHT
#define GRID_HEIGHT 42
#endif

#ifndef MAX_AGENTS
#define MAX_AGENTS 16
#endif

#ifndef MAX_GOALS
#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT)
#endif

#ifndef MAX_CHARGE_STATIONS
#define MAX_CHARGE_STATIONS 10
#endif

#ifndef MAX_PHASES
#define MAX_PHASES 20
#endif

#ifndef LOG_BUFFER_LINES
#define LOG_BUFFER_LINES 5
#endif

#ifndef LOG_BUFFER_WIDTH
#define LOG_BUFFER_WIDTH 256
#endif

#ifndef MAX_WHCA_HORIZON
#define MAX_WHCA_HORIZON 11
#endif

#ifndef MIN_WHCA_HORIZON
#define MIN_WHCA_HORIZON 5
#endif

#ifndef MAX_CBS_CONS
#define MAX_CBS_CONS 128
#endif

#ifndef TEMP_MARK_MAX
#define TEMP_MARK_MAX 128
#endif

typedef enum {
    DIR_NONE = -1,
    DIR_UP = 0,
    DIR_RIGHT = 1,
    DIR_DOWN = 2,
    DIR_LEFT = 3
} AgentDir;

typedef enum {
    AGV_PHASECFG_PARK = 0,
    AGV_PHASECFG_EXIT = 1
} AgvPhaseType;

typedef enum {
    AGV_MODECFG_CUSTOM = 0,
    AGV_MODECFG_REALTIME = 1
} AgvSimulationMode;

#include "agv/sim_bridge.hpp"
#include "agv/internal/engine_model.hpp"

int agv_current_whca_horizon();
int agv_current_conflict_score();
void agv_set_whca_runtime_state(int conflict_score, int horizon);
void agv_begin_apply_config(Simulation_* sim, int suppress_stdout);
int agv_finalize_apply_config(Simulation_* sim);
AgentWorkloadSnapshot agv_collect_agent_workload(const AgentManager* am);
Node* agv_select_best_charge_station(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_set_goal_if_needed(Agent* ag, GridMap* map, AgentManager* am, Logger* lg);
void agv_update_task_dispatch(Simulation_* sim);
int agv_apply_moves_and_update_stuck(Simulation_* sim, Node* next_pos[MAX_AGENTS], Node* prev_pos[MAX_AGENTS]);
void agv_update_deadlock_counter(Simulation_* sim, int moved_this_step, int is_custom_mode);
void agv_accumulate_wait_ticks_if_realtime(Simulation_* sim);
void agv_execute_step_service(Simulation_* sim, int is_paused);
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
