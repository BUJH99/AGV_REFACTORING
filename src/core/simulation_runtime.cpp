#define _CRT_SECURE_NO_WARNINGS

#include <cctype>
#include <ctime>

#include <windows.h>
#include <psapi.h>
#ifdef _WIN32
#pragma comment(lib, "psapi.lib")
#endif

#include "agv/internal/engine_internal.hpp"

Planner planner_from_pathalgo(PathAlgo algo);

namespace {

class SimulationObservationSink final : public ObservationSink {
public:
    explicit SimulationObservationSink(Simulation* simulation)
        : simulation_(simulation) {}

    PlannerOverlayCapture* plannerOverlay() override {
        return simulation_ ? &simulation_->planner_capture : nullptr;
    }

private:
    Simulation* simulation_{nullptr};
};

PlanningContext make_planning_context(Simulation* sim, ObservationSink* observation) {
    return PlanningContext{
        sim,
        sim ? sim->agent_manager : nullptr,
        sim ? sim->map : nullptr,
        sim ? sim->logger : nullptr,
        sim ? &sim->runtime_tuning : nullptr,
        sim ? &sim->planner_metrics : nullptr,
        observation,
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
        agent->pf->resetLastRunMetrics();
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
    if (metrics.wf_edges_last > 0) {
        sim->planner_metrics.wf_step_count++;
    }
    if (metrics.scc_last > 0) {
        sim->planner_metrics.scc_step_count++;
    }
    if (metrics.cbs_exp_last > 0 || metrics.cbs_ok_last > 0) {
        sim->planner_metrics.cbs_attempt_sum++;
    }
    sim->algorithm_operation_count += (unsigned long long)((metrics.wf_edges_last > 0 ? metrics.wf_edges_last : 0) +
        (metrics.scc_last > 0 ? metrics.scc_last : 0) +
        (metrics.cbs_exp_last > 0 ? metrics.cbs_exp_last : 0));
}

void maybe_report_realtime_dashboard(Simulation* sim) {
    ScenarioManager* scenario = sim->scenario_manager;
    scenario->time_step++;
}

void print_completion_message_if_needed(const Simulation* sim, std::string_view message) {
    if (!sim || sim->suppress_stdout) return;
    agv::internal::text::console_print(std::string(C_B_GRN) + "\n%s\n" + C_NRM, message);
}

bool are_all_agents_idle(const AgentManager* agent_manager) {
    if (!agent_manager) return true;
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (agent_manager->agents[i].state != AgentState::Idle) return false;
    }
    return true;
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

}  // namespace

Simulation_::Simulation_() {
    reseedRandom(static_cast<unsigned int>(time(nullptr)));
    map_id = 1;
    path_algo = PathAlgo::Default;
    display_buffer.reserve(DISPLAY_BUFFER_SIZE);
    logger->bindSimulation(this);
    planner = planner_from_pathalgo(path_algo);
    render_state.configureForAlgorithm(path_algo);
    planner_metrics.whca_h = runtime_tuning.whca_horizon;
    grid_map_load_scenario(map, agent_manager, map_id);
    logger->setContext(0, 0, scenario_manager ? scenario_manager->current_phase_index : -1);
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
    no_movement_streak = 0;
    max_no_movement_streak = 0;
    steps_with_movement = 0;
    stall_step_count = 0;
    last_active_agent_count = 0;
    last_waiting_agent_count = 0;
    last_stuck_agent_count = 0;
    last_oscillating_agent_count = 0;
    last_action_agent_count = 0;
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
    outstanding_task_sum = 0;
    outstanding_task_samples = 0;
    outstanding_task_peak = 0;
    oldest_request_age_last = 0;
    oldest_request_age_sum = 0;
    oldest_request_age_peak = 0;
    algo_nodes_expanded_total = 0;
    algo_heap_moves_total = 0;
    algo_nodes_expanded_last_step = 0;
    algo_heap_moves_last_step = 0;
    algo_generated_nodes_total = 0;
    algo_valid_expansions_total = 0;
    algo_generated_nodes_last_step = 0;
    algo_valid_expansions_last_step = 0;
    planner_metrics = {};
    planner_metrics.whca_h = runtime_tuning.whca_horizon;
    last_deadlock_event = {};
    workload_snapshot = {};
    step_scratch = {};
    logger->setContext(0, render_model.frame_id, scenario_manager ? scenario_manager->current_phase_index : -1);
}

void Simulation_::planStep(AgentNodeSlots& next_positions) {
    Simulation* sim = this;
    const clock_t plan_start_cpu = clock();

    reset_planner_step_metrics(sim);

    SimulationObservationSink observation_sink(sim);
    const PlanningContext context = make_planning_context(sim, &observation_sink);
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

bool execute_headless_step(Simulation* sim, bool allow_sleep) {
    (void)allow_sleep;
    if (!sim) return true;
    if (sim->isComplete()) return true;
    sim->executeOneStep(false);
    if (sim->isComplete()) return true;
    maybe_report_realtime_dashboard(sim);
    return false;
}

bool run_simulation_to_completion(Simulation* sim) {
    if (!sim) return false;
    while (!execute_headless_step(sim, false)) {
    }
    return true;
}
