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

static std::string_view simulation_mode_report_label(const ScenarioManager* sc) {
    if (!sc) return "Uninitialized";

    switch (sc->mode) {
    case SimulationMode::Custom: return "Custom";
    case SimulationMode::Realtime: return "Real-Time";
    default: return "Uninitialized";
    }
}

static std::string_view simulation_path_algo_report_label(PathAlgo path_algo) {
    if (path_algo == PathAlgo::AStarSimple) return "A* (Single-Agent)";
    if (path_algo == PathAlgo::DStarBasic) return "D* Lite (Incremental)";
    return "Default (WHCA* + D* Lite + WFG + CBS)";
}

static SimulationMode summary_mode_value(const ScenarioManager* sc) {
    if (!sc) return SimulationMode::Custom;
    return (sc->mode == SimulationMode::Realtime) ? SimulationMode::Realtime : SimulationMode::Custom;
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

    agv::internal::text::console_print("\n============================================\n");
    agv::internal::text::console_print("          Simulation Result Report\n");
    agv::internal::text::console_print("============================================\n");
    agv::internal::text::console_print(" Mode                                : %s\n", simulation_mode_report_label(sc));
    agv::internal::text::console_print(" Map ID                              : %d\n", sim->map_id);
    agv::internal::text::console_print(" Path Planning Algorithm             : %s\n", simulation_path_algo_report_label(sim->path_algo));
    agv::internal::text::console_print(" Total Physical Time Steps           : %d\n", recorded_steps);
    agv::internal::text::console_print(" Operating AGVs                     : %d\n", simulation_active_agent_count(am));

    agv::internal::text::console_print(" Tasks Completed (total)             : %llu\n", sim->tasks_completed_total);
    agv::internal::text::console_print(" Throughput [task / total physical time] : %.4f\n", throughput);
    agv::internal::text::console_print(" Total Movement Cost (cells)         : %.2f\n", sim->total_movement_cost);

    agv::internal::text::console_print(" Requests Created (total)            : %llu\n", sim->requests_created_total);
    agv::internal::text::console_print(" Request Wait Ticks (sum)            : %llu\n", sim->request_wait_ticks_sum);
    agv::internal::text::console_print(" Process Memory Usage Sum            : %.2f KB\n", sim->memory_usage_sum_kb);
    agv::internal::text::console_print(" Process Memory Usage Average        : %.2f KB\n", avg_memory_kb);
    agv::internal::text::console_print(" Process Memory Usage Peak           : %.2f KB\n", sim->memory_usage_peak_kb);
    agv::internal::text::console_print(" Remaining Parked Vehicles           : %d\n", am ? am->total_cars_parked : 0);
    agv::internal::text::console_print("\n -- Algorithm and Planner Statistics --\n");
    agv::internal::text::console_print(" Nodes Expanded (total)             : %llu\n", sim->algo_nodes_expanded_total);
    agv::internal::text::console_print(" Heap Moves (total)                  : %llu\n", sim->algo_heap_moves_total);
    agv::internal::text::console_print(" Generated Nodes (total)            : %llu\n", sim->algo_generated_nodes_total);
    agv::internal::text::console_print(" Valid Expansions (total)           : %llu\n", sim->algo_valid_expansions_total);
    double valid_ratio_total = (sim->algo_generated_nodes_total > 0) ? (double)sim->algo_valid_expansions_total / (double)sim->algo_generated_nodes_total : 0.0;
    agv::internal::text::console_print(" Valid Expansion Ratio (valid/gen) : %.4f\n", valid_ratio_total);
    if (recorded_steps > 0) {
        const double avg_nodes_per_step = (double)sim->algo_nodes_expanded_total / (double)recorded_steps;
        const double avg_heap_moves_per_step = (double)sim->algo_heap_moves_total / (double)recorded_steps;
        const double avg_generated_per_step = (double)sim->algo_generated_nodes_total / (double)recorded_steps;
        const double avg_valid_per_step = (double)sim->algo_valid_expansions_total / (double)recorded_steps;
        agv::internal::text::console_print(" Nodes Expanded (avg per step)      : %.2f\n", avg_nodes_per_step);
        agv::internal::text::console_print(" Heap Moves (avg per step)          : %.2f\n", avg_heap_moves_per_step);
        agv::internal::text::console_print(" Generated Nodes (avg per step)     : %.2f\n", avg_generated_per_step);
        agv::internal::text::console_print(" Valid Expansions (avg per step)    : %.2f\n", avg_valid_per_step);
    }

    if (sc && sc->mode == SimulationMode::Custom) {
        agv::internal::text::console_print("\n -- Custom Scenario Breakdown --\n");
        for (int i = 0; i < sc->num_phases; i++) {
            const DynamicPhase* ph = &sc->phases[i];
            const int planned = ph->task_count;
            const int completed = sim->phase_completed_tasks[i];
            const int step_count = sim->phase_step_counts[i];
            agv::internal::text::console_print(" Phase %d (%s)\n", i + 1, ph->type_name.c_str());
            agv::internal::text::console_print("   Planned Tasks           : %d\n", planned);
            agv::internal::text::console_print("   Completed Tasks         : %d\n", completed);
            if (step_count > 0) {
                agv::internal::text::console_print("   Step Span               : %d step(s)", step_count);
                if (sim->phase_first_step[i] >= 0 && sim->phase_last_step[i] >= 0) {
                    agv::internal::text::console_print(" [#%d -> #%d]\n", sim->phase_first_step[i], sim->phase_last_step[i]);
                } else {
                    agv::internal::text::console_print("\n");
                }
                const double phase_avg_cpu = sim->phase_cpu_time_ms[i] / (double)step_count;
                agv::internal::text::console_print("   CPU Time                : %.2f ms (avg %.4f ms/step)\n",
                    sim->phase_cpu_time_ms[i], phase_avg_cpu);
            } else {
                agv::internal::text::console_print("   Step Span               : N/A\n");
            }
            if (completed < planned) {
                agv::internal::text::console_print("   Remaining Tasks         : %d\n", planned - completed);
            }
        }
    } else if (sc && sc->mode == SimulationMode::Realtime) {
        agv::internal::text::console_print("\n -- Custom Scenario Breakdown --\n");

        agv::internal::text::console_print(" Phase 1 (%s)\n", "Real-Time");
        agv::internal::text::console_print("   Planned Tasks           : %d\n", (int)sim->tasks_completed_total);
        agv::internal::text::console_print("   Completed Tasks         : %d\n", (int)sim->tasks_completed_total);
        if (recorded_steps > 0) {
            agv::internal::text::console_print("   Step Span               : %d step(s) [#%d -> #%d]\n", recorded_steps, 1, recorded_steps);
            const double phase_avg_cpu = sim->total_cpu_time_ms / (double)recorded_steps;
            agv::internal::text::console_print("   CPU Time                : %.2f ms (avg %.4f ms/step)\n", sim->total_cpu_time_ms, phase_avg_cpu);
        } else {
            agv::internal::text::console_print("   Step Span               : N/A\n");
            agv::internal::text::console_print("   CPU Time                : 0.00 ms (avg 0.0000 ms/step)\n");
        }
    }

    agv::internal::text::console_print("============================================\n");
}

void simulation_print_performance_summary(const Simulation* sim) {
    if (sim) sim->printPerformanceSummary();
}
