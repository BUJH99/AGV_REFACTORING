#define _CRT_SECURE_NO_WARNINGS

#include <cctype>
#include <ctime>

#include <windows.h>
#include <psapi.h>
#ifdef _WIN32
#pragma comment(lib, "psapi.lib")
#endif

#include "agv/internal/engine_internal.hpp"

void ui_handle_control_key(Simulation* sim, int ch, bool& is_paused, bool& quit_flag);
void simulation_display_status(Simulation* sim, bool is_paused);
Planner planner_from_pathalgo(PathAlgo algo);
RendererFacade renderer_create_facade(void);

namespace {

PlanningContext make_planning_context(Simulation* sim) {
    return PlanningContext{
        sim,
        sim ? sim->agent_manager : nullptr,
        sim ? sim->map : nullptr,
        sim ? sim->logger : nullptr,
        sim ? &sim->runtime_tuning : nullptr,
        sim ? &sim->planner_metrics : nullptr,
    };
}

struct PlannerStepCounters final {
    unsigned long long nodes_expanded{0};
    unsigned long long heap_moves{0};
    unsigned long long generated_nodes{0};
    unsigned long long valid_expansions{0};
};

void reset_planner_step_metrics(Simulation* sim) {
    sim->algo_nodes_expanded_last_step = 0;
    sim->algo_heap_moves_last_step = 0;
    sim->algo_generated_nodes_last_step = 0;
    sim->algo_valid_expansions_last_step = 0;
    sim->planner_metrics.resetStepCounters();
    sim->planner_metrics.whca_h = sim->runtime_tuning.whca_horizon;

    AgentManager* agent_manager = sim->agent_manager;
    if (!agent_manager) return;

    for (int i = 0; i < MAX_AGENTS; i++) {
        Agent* agent = &agent_manager->agents[i];
        if (!agent->pf) continue;
        agent->pf->nodes_expanded_this_call = 0;
        agent->pf->heap_moves_this_call = 0;
        agent->pf->nodes_generated_this_call = 0;
        agent->pf->valid_expansions_this_call = 0;
    }
}

PlannerStepCounters collect_planner_step_counters(PathAlgo path_algo, const PlannerMetricsState& metrics) {
    if (path_algo == PathAlgo::AStarSimple) {
        return {
            metrics.astar_nodes_expanded_this_step,
            metrics.astar_heap_moves_this_step,
            metrics.astar_generated_nodes_this_step,
            metrics.astar_valid_expansions_this_step,
        };
    }

    if (path_algo == PathAlgo::DStarBasic) {
        return {
            metrics.dstar_nodes_expanded_this_step,
            metrics.dstar_heap_moves_this_step,
            metrics.dstar_generated_nodes_this_step,
            metrics.dstar_valid_expansions_this_step,
        };
    }

    return {
        metrics.whca_dstar_nodes_expanded_this_step + metrics.whca_nodes_expanded_this_step,
        metrics.whca_dstar_heap_moves_this_step + metrics.whca_heap_moves_this_step,
        metrics.whca_dstar_generated_nodes_this_step + metrics.whca_generated_nodes_this_step,
        metrics.whca_dstar_valid_expansions_this_step + metrics.whca_valid_expansions_this_step,
    };
}

void record_planner_step_results(
    Simulation* sim,
    const PlannerStepCounters& step_counters,
    const PlannerMetricsState& metrics,
    clock_t plan_start_cpu,
    clock_t plan_end_cpu) {
    sim->algo_nodes_expanded_last_step = step_counters.nodes_expanded;
    sim->algo_heap_moves_last_step = step_counters.heap_moves;
    sim->algo_generated_nodes_last_step = step_counters.generated_nodes;
    sim->algo_valid_expansions_last_step = step_counters.valid_expansions;
    sim->algo_nodes_expanded_total += step_counters.nodes_expanded;
    sim->algo_heap_moves_total += step_counters.heap_moves;
    sim->algo_generated_nodes_total += step_counters.generated_nodes;
    sim->algo_valid_expansions_total += step_counters.valid_expansions;

    const double planning_time_ms = ((double)(plan_end_cpu - plan_start_cpu) * 1000.0) / CLOCKS_PER_SEC;
    sim->last_planning_time_ms = planning_time_ms;
    sim->total_planning_time_ms += planning_time_ms;
    if (planning_time_ms > sim->max_planning_time_ms) sim->max_planning_time_ms = planning_time_ms;
    sim->algorithm_operation_count += (unsigned long long)((metrics.wf_edges_last > 0 ? metrics.wf_edges_last : 0) +
        (metrics.scc_last > 0 ? metrics.scc_last : 0) +
        (metrics.cbs_exp_last > 0 ? metrics.cbs_exp_last : 0));
}

void maybe_report_realtime_dashboard(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;
    scenario->time_step++;
    if (scenario->mode == SimulationMode::Realtime && (scenario->time_step % DASHBOARD_INTERVAL_STEPS) == 0) {
        sim->reportRealtimeDashboard();
    }
}

bool are_all_agents_idle(const AgentManager* agent_manager) {
    if (!agent_manager) return true;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].state != AgentState::Idle) return false;
    }
    return true;
}

void print_completion_message_if_needed(const Simulation* sim, std::string_view message) {
    if (!sim || sim->suppress_stdout) return;
    agv::internal::text::console_print(std::string(C_B_GRN) + "\n%s\n" + C_NRM, message);
}

bool is_custom_scenario_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    if (!scenario || scenario->mode != SimulationMode::Custom) return false;
    if (scenario->current_phase_index < scenario->num_phases) return false;
    return are_all_agents_idle(sim->agent_manager);
}

bool is_realtime_scenario_complete(const Simulation* sim) {
    const ScenarioManager* scenario = sim->scenario_manager;
    return scenario && scenario->mode == SimulationMode::Realtime &&
        scenario->time_step >= REALTIME_MODE_TIMELIMIT;
}

int read_control_key() {
    return console_read_key_nonblocking().value_or(0);
}

void handle_control_input(Simulation* sim, int last_key, bool& is_paused, bool& quit_flag) {
    if (!last_key) return;
    ui_handle_control_key(sim, last_key, is_paused, quit_flag);
    sim->renderer.drawFrame(sim, is_paused);
}

bool should_wait_while_paused(bool is_paused, int last_key) {
    return is_paused && std::tolower(last_key) != 's';
}

void maybe_sleep_for_simulation_speed(const ScenarioManager* scenario) {
    if (scenario && scenario->simulation_speed > 0) {
        platform_sleep_for_ms(scenario->simulation_speed);
    }
}

}  // namespace

Simulation_::Simulation_() {
    reseedRandom(static_cast<unsigned int>(time(nullptr)));
    map_id = 1;
    path_algo = PathAlgo::Default;
    display_buffer.reserve(DISPLAY_BUFFER_SIZE);
    planner = planner_from_pathalgo(path_algo);
    renderer = renderer_create_facade();
    render_state.configureForAlgorithm(path_algo);
    planner_metrics.whca_h = runtime_tuning.whca_horizon;
    grid_map_load_scenario(map, agent_manager, map_id);
}

Simulation_::~Simulation_() = default;

void Simulation_::reseedRandom(unsigned int seed) {
    configured_seed = seed;
    random_engine.seed(seed);
}

int Simulation_::nextRandomInt(int exclusive_upper_bound) {
    if (exclusive_upper_bound <= 0) {
        return 0;
    }
    std::uniform_int_distribution<int> distribution(0, exclusive_upper_bound - 1);
    return distribution(random_engine);
}

void Simulation_::collectMemorySample() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        const double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
        memory_usage_sum_kb += working_set_kb;
        if (working_set_kb > memory_usage_peak_kb) memory_usage_peak_kb = working_set_kb;
        memory_samples++;
    }
#endif
}

void Simulation_::collectMemorySampleAlgo() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
        const double working_set_kb = (double)pmc.WorkingSetSize / 1024.0;
        algo_mem_sum_kb += working_set_kb;
        if (working_set_kb > algo_mem_peak_kb) algo_mem_peak_kb = working_set_kb;
        algo_mem_samples++;
    }
#endif
}

void Simulation_::resetRuntimeStats() {
    total_cpu_time_ms = 0.0;
    last_step_cpu_time_ms = 0.0;
    max_step_cpu_time_ms = 0.0;
    std::fill(std::begin(phase_cpu_time_ms), std::end(phase_cpu_time_ms), 0.0);
    std::fill(std::begin(phase_step_counts), std::end(phase_step_counts), 0);
    std::fill(std::begin(phase_completed_tasks), std::end(phase_completed_tasks), 0);
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

void Simulation_::reportRealtimeDashboard() {
    ScenarioManager* scenario = scenario_manager;
    int steps = (total_executed_steps > 0) ? total_executed_steps : (scenario ? scenario->time_step : 0);
    if (steps <= 0) steps = 1;

    const unsigned long long total_completed = tasks_completed_total;
    int interval_steps = steps - last_report_step;
    if (interval_steps <= 0) interval_steps = 1;
    const unsigned long long delta_completed = total_completed - last_report_completed_tasks;

    const double throughput_avg = (double)total_completed / (double)steps;
    const double throughput_interval = (double)delta_completed / (double)interval_steps;
    const double avg_planning_ms = (steps > 0) ? total_planning_time_ms / (double)steps : 0.0;
    const double avg_memory_kb = (memory_samples > 0) ? memory_usage_sum_kb / (double)memory_samples : 0.0;

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

    agv::internal::text::console_print("\n========== Real-Time Dashboard @ step %d ==========\n", steps);
    agv::internal::text::console_print(" Total Physical Time Steps      : %d\n", steps);
    agv::internal::text::console_print(" Operating AGVs                 : %d\n", active_agents);
    agv::internal::text::console_print(" Tasks Completed (total)        : %llu\n", total_completed);
    agv::internal::text::console_print(" Throughput (total avg)         : %.4f tasks/step\n", throughput_avg);
    agv::internal::text::console_print(" Throughput (last interval)     : %.4f tasks/step over %d steps\n", throughput_interval, interval_steps);
    agv::internal::text::console_print(" Total Computation CPU Time     : %.2f ms\n", total_cpu_time_ms);
    agv::internal::text::console_print(" Average Planning Time / Step   : %.4f ms\n", avg_planning_ms);
    agv::internal::text::console_print(" Total Task Completion Step     : %d\n", last_task_completion_step);
    agv::internal::text::console_print(" Total Movement Cost            : %.2f cells\n", total_movement_cost);
    agv::internal::text::console_print(" Requests Created (total)       : %llu\n", requests_created_total);
    agv::internal::text::console_print(" Request Wait Ticks (sum)       : %llu\n", request_wait_ticks_sum);
    agv::internal::text::console_print(" Process Memory Usage Sum      : %.2f KB (avg %.2f KB / sample, peak %.2f KB)\n",
        memory_usage_sum_kb, avg_memory_kb, memory_usage_peak_kb);
    agv::internal::text::console_print(" Heap Moves (total/last)          : %llu / %llu\n", algo_heap_moves_total, algo_heap_moves_last_step);
    agv::internal::text::console_print(" Generated Nodes (total/last)     : %llu / %llu\n", algo_generated_nodes_total, algo_generated_nodes_last_step);
    agv::internal::text::console_print(" Valid Expansions (total/last)    : %llu / %llu\n", algo_valid_expansions_total, algo_valid_expansions_last_step);
    const double dash_ratio_total = (algo_generated_nodes_total > 0) ? (double)algo_valid_expansions_total / (double)algo_generated_nodes_total : 0.0;
    const double dash_ratio_last = (algo_generated_nodes_last_step > 0) ? (double)algo_valid_expansions_last_step / (double)algo_generated_nodes_last_step : 0.0;
    agv::internal::text::console_print(" Valid Expansion Ratio (total/last): %.4f / %.4f\n", dash_ratio_total, dash_ratio_last);
    agv::internal::text::console_print("===================================================\n");

    last_report_completed_tasks = total_completed;
    last_report_step = steps;
}

void Simulation_::planStep(AgentNodeSlots& next_positions) {
    Simulation* sim = this;
    const clock_t plan_start_cpu = clock();

    reset_planner_step_metrics(sim);

    const PlanningContext context = make_planning_context(sim);
    sim->planner.planStep(context, next_positions);

    const PlannerMetricsState& metrics = sim->planner_metrics;
    const PlannerStepCounters step_counters = collect_planner_step_counters(sim->path_algo, metrics);
    const clock_t plan_end_cpu = clock();
    record_planner_step_results(sim, step_counters, metrics, plan_start_cpu, plan_end_cpu);
}

void Simulation_::updateState() {
    agv_update_task_dispatch(this);
}

void Simulation_::executeOneStep(bool is_paused) {
    agv_execute_step_service(this, is_paused);
}

bool Simulation_::isComplete() const {
    const Simulation* sim = this;
    if (is_custom_scenario_complete(sim)) {
        print_completion_message_if_needed(sim, "Custom scenario completed.");
        return true;
    }
    if (is_realtime_scenario_complete(sim)) {
        print_completion_message_if_needed(sim, "Real-time simulation completed.");
        return true;
    }
    return false;
}

void Simulation_::run() {
    Simulation* sim = this;
    bool is_paused = false;
    bool quit_flag = false;

    sim->resetRuntimeStats();
    sim->renderer.drawFrame(sim, is_paused);

    while (!quit_flag) {
        const int last_key = read_control_key();
        handle_control_input(sim, last_key, is_paused, quit_flag);
        if (quit_flag) continue;
        if (should_wait_while_paused(is_paused, last_key)) {
            platform_sleep_for_ms(PAUSE_POLL_INTERVAL_MS);
            continue;
        }
        sim->executeOneStep(is_paused);
        if (sim->isComplete()) {
            break;
        }
        maybe_report_realtime_dashboard(sim);
        maybe_sleep_for_simulation_speed(sim->scenario_manager);
    }
}

bool execute_headless_step(Simulation* sim) {
    if (!sim) return true;
    if (sim->isComplete()) return true;
    sim->executeOneStep(false);
    if (sim->isComplete()) return true;
    maybe_report_realtime_dashboard(sim);
    if (sim->scenario_manager->simulation_speed > 0) {
        platform_sleep_for_ms(sim->scenario_manager->simulation_speed);
    }
    return false;
}

bool run_simulation_to_completion(Simulation* sim) {
    if (!sim) return false;
    while (!execute_headless_step(sim)) {
    }
    return true;
}

std::string build_render_frame_text(Simulation* sim, bool is_paused) {
    if (!sim) {
        return {};
    }

    const bool previous_suppress = sim->render_state.suppress_flush;
    sim->render_state.suppress_flush = true;
    simulation_display_status(sim, is_paused);
    sim->render_state.suppress_flush = previous_suppress;
    return sim->display_buffer;
}
