#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

static int simulation_recorded_steps(const Simulation* sim, const ScenarioManager* sc) {
    int recorded_steps = (sim && sim->total_executed_steps > 0) ? sim->total_executed_steps : (sc ? sc->time_step : 0);
    return (recorded_steps < 0) ? 0 : recorded_steps;
}

static int simulation_active_agent_count(const AgentManager* am) {
    if (!am) return 0;

    int active_agents = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (am->agents[i].pos) active_agents++;
    }
    return active_agents;
}

static const char* simulation_mode_report_label(const ScenarioManager* sc) {
    if (!sc) return "Uninitialized";

    switch (sc->mode) {
    case MODE_CUSTOM: return "Custom";
    case MODE_REALTIME: return "Real-Time";
    default: return "Uninitialized";
    }
}

static const char* simulation_path_algo_report_label(PathAlgo path_algo) {
    if (path_algo == PATHALGO_ASTAR_SIMPLE) return "A* (Single-Agent)";
    if (path_algo == PATHALGO_DSTAR_BASIC) return "D* Lite (Incremental)";
    return "Default (WHCA* + D* Lite + WFG + CBS)";
}

static SimulationMode summary_mode_value(const ScenarioManager* sc) {
    if (!sc) return MODE_CUSTOM;
    return (sc->mode == MODE_REALTIME) ? MODE_REALTIME : MODE_CUSTOM;
}

RunSummary collect_run_summary(const Simulation* sim) {
    RunSummary summary{};
    if (!sim) return summary;

    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    const int recorded_steps = simulation_recorded_steps(sim, sc);

    summary.seed = sim->configured_seed;
    summary.map_id = sim->map_id;
    summary.path_algo = sim->path_algo;
    summary.mode = summary_mode_value(sc);
    summary.active_agents = simulation_active_agent_count(am);
    summary.recorded_steps = recorded_steps;
    summary.tasks_completed_total = sim->tasks_completed_total;
    summary.throughput = (recorded_steps > 0) ? ((double)sim->tasks_completed_total / (double)recorded_steps) : 0.0;
    summary.total_movement_cost = sim->total_movement_cost;
    summary.deadlock_count = sim->deadlock_count;
    summary.total_cpu_time_ms = sim->total_cpu_time_ms;
    summary.avg_cpu_time_ms = (recorded_steps > 0) ? (sim->total_cpu_time_ms / (double)recorded_steps) : 0.0;
    summary.total_planning_time_ms = sim->total_planning_time_ms;
    summary.avg_planning_time_ms = (recorded_steps > 0) ? (sim->total_planning_time_ms / (double)recorded_steps) : 0.0;
    summary.memory_usage_sum_kb = sim->memory_usage_sum_kb;
    summary.avg_memory_usage_kb = (sim->memory_samples > 0) ? (sim->memory_usage_sum_kb / (double)sim->memory_samples) : 0.0;
    summary.memory_usage_peak_kb = sim->memory_usage_peak_kb;
    summary.algo_nodes_expanded_total = sim->algo_nodes_expanded_total;
    summary.algo_heap_moves_total = sim->algo_heap_moves_total;
    summary.algo_generated_nodes_total = sim->algo_generated_nodes_total;
    summary.algo_valid_expansions_total = sim->algo_valid_expansions_total;
    summary.valid_expansion_ratio = (sim->algo_generated_nodes_total > 0)
        ? ((double)sim->algo_valid_expansions_total / (double)sim->algo_generated_nodes_total)
        : 0.0;
    summary.requests_created_total = sim->requests_created_total;
    summary.request_wait_ticks_sum = sim->request_wait_ticks_sum;
    summary.remaining_parked_vehicles = am ? am->total_cars_parked : 0;
    return summary;
}

void Simulation_::printPerformanceSummary() const {
    const Simulation* sim = this;
    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    const int recorded_steps = simulation_recorded_steps(sim, sc);
    const double avg_cpu_ms = (recorded_steps > 0) ? (sim->total_cpu_time_ms / (double)recorded_steps) : 0.0;
    const double avg_plan_ms = (recorded_steps > 0) ? (sim->total_planning_time_ms / (double)recorded_steps) : 0.0;
    const double throughput = (recorded_steps > 0) ? ((double)sim->tasks_completed_total / (double)recorded_steps) : 0.0;
    const double avg_memory_kb = (sim->memory_samples > 0) ? (sim->memory_usage_sum_kb / (double)sim->memory_samples) : 0.0;

    printf("\n============================================\n");
    printf("          Simulation Result Report\n");
    printf("============================================\n");
    printf(" Mode                                : %s\n", simulation_mode_report_label(sc));
    printf(" Map ID                              : %d\n", sim->map_id);
    printf(" Path Planning Algorithm             : %s\n", simulation_path_algo_report_label(sim->path_algo));
    printf(" Total Physical Time Steps           : %d\n", recorded_steps);
    printf(" Operating AGVs                     : %d\n", simulation_active_agent_count(am));

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
            printf(" Phase %d (%s)\n", i + 1, ph->type_name.c_str());
            printf("   Planned Tasks           : %d\n", planned);
            printf("   Completed Tasks         : %d\n", completed);
            if (step_count > 0) {
                printf("   Step Span               : %d step(s)", step_count);
                if (sim->phase_first_step[i] >= 0 && sim->phase_last_step[i] >= 0) {
                    printf(" [#%d -> #%d]\n", sim->phase_first_step[i], sim->phase_last_step[i]);
                } else {
                    printf("\n");
                }
                const double phase_avg_cpu = sim->phase_cpu_time_ms[i] / (double)step_count;
                printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n",
                    sim->phase_cpu_time_ms[i], phase_avg_cpu);
            } else {
                printf("   Step Span               : N/A\n");
            }
            if (completed < planned) {
                printf("   Remaining Tasks         : %d\n", planned - completed);
            }
        }
    } else if (sc && sc->mode == MODE_REALTIME) {
        printf("\n -- Custom Scenario Breakdown --\n");

        printf(" Phase 1 (%s)\n", "Real-Time");
        printf("   Planned Tasks           : %d\n", (int)sim->tasks_completed_total);
        printf("   Completed Tasks         : %d\n", (int)sim->tasks_completed_total);
        if (recorded_steps > 0) {
            printf("   Step Span               : %d step(s) [#%d -> #%d]\n", recorded_steps, 1, recorded_steps);
            const double phase_avg_cpu = sim->total_cpu_time_ms / (double)recorded_steps;
            printf("   CPU Time                : %.2f ms (avg %.4f ms/step)\n", sim->total_cpu_time_ms, phase_avg_cpu);
        } else {
            printf("   Step Span               : N/A\n");
            printf("   CPU Time                : 0.00 ms (avg 0.0000 ms/step)\n");
        }
    }

    printf("============================================\n");
}

void simulation_print_performance_summary(const Simulation* sim) {
    if (sim) sim->printPerformanceSummary();
}
