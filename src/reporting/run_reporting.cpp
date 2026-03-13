#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <cmath>

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

static int simulation_in_flight_task_count(const AgentManager* am) {
    if (!am) return 0;

    int count = 0;
    for (int i = 0; i < MAX_AGENTS; i++) {
        const AgentState state = am->agents[i].state;
        if (state == AgentState::GoingToPark ||
            state == AgentState::ReturningHomeEmpty ||
            state == AgentState::GoingToCollect ||
            state == AgentState::ReturningWithCar) {
            count++;
        }
    }
    return count;
}

static int simulation_phase_remaining_tasks(const Simulation* sim, const ScenarioManager* sc) {
    if (!sim || !sc ||
        sc->mode != SimulationMode::Custom ||
        sc->current_phase_index < 0 ||
        sc->current_phase_index >= sc->num_phases) {
        return 0;
    }

    const DynamicPhase& phase = sc->phases[sc->current_phase_index];
    return std::max(phase.task_count - sc->tasks_completed_in_phase, 0);
}

static int simulation_oldest_queued_request_age(const ScenarioManager* sc) {
    if (!sc || sc->task_queue.empty()) {
        return 0;
    }

    int oldest_age = 0;
    for (const TaskNode& task : sc->task_queue) {
        const int age = std::max(sc->time_step - task.created_at_step, 0);
        if (age > oldest_age) {
            oldest_age = age;
        }
    }
    return oldest_age;
}

static int simulation_outstanding_task_count(const Simulation* sim, const ScenarioManager* sc, const AgentManager* am) {
    if (!sc) {
        return 0;
    }

    if (sc->mode == SimulationMode::Custom) {
        return simulation_phase_remaining_tasks(sim, sc);
    }

    return sc->task_count + simulation_in_flight_task_count(am);
}

static double safe_divide(double numerator, double denominator) {
    return (denominator > 0.0) ? (numerator / denominator) : 0.0;
}

static double summarize_min_max_ratio(double min_value, double max_value) {
    if (std::fabs(max_value - min_value) <= 1e-9) {
        return 1.0;
    }
    return safe_divide(min_value, max_value);
}

template <typename MetricReader>
static DistributionSummary summarize_active_agent_metric(const AgentManager* am, MetricReader reader) {
    DistributionSummary summary{};
    if (!am) {
        return summary;
    }

    double sum = 0.0;
    double sum_sq = 0.0;
    int count = 0;
    double min_value = 0.0;
    double max_value = 0.0;

    for (int i = 0; i < MAX_AGENTS; ++i) {
        const Agent& agent = am->agents[i];
        if (!agent.pos) {
            continue;
        }

        const double value = reader(agent);
        if (count == 0) {
            min_value = value;
            max_value = value;
        } else {
            min_value = std::min(min_value, value);
            max_value = std::max(max_value, value);
        }

        sum += value;
        sum_sq += value * value;
        count++;
    }

    if (count == 0) {
        return summary;
    }

    summary.min = min_value;
    summary.avg = sum / (double)count;
    summary.max = max_value;
    const double variance = std::max((sum_sq / (double)count) - (summary.avg * summary.avg), 0.0);
    summary.stddev = std::sqrt(variance);
    summary.coefficient_of_variation = safe_divide(summary.stddev, std::fabs(summary.avg));
    summary.min_max_ratio = summarize_min_max_ratio(summary.min, summary.max);
    return summary;
}

static std::vector<AgentFairnessRecord> collect_active_agent_fairness_breakdown(const AgentManager* am) {
    std::vector<AgentFairnessRecord> breakdown;
    if (!am) {
        return breakdown;
    }

    breakdown.reserve(MAX_AGENTS);
    for (int index = 0; index < MAX_AGENTS; ++index) {
        const Agent& agent = am->agents[index];
        if (!agent.pos) {
            continue;
        }

        AgentFairnessRecord record;
        record.id = agent.id;
        record.symbol = agent.symbol;
        record.tasks_completed = agent.metrics_completed_tasks_total;
        record.distance_cells = agent.metrics_total_distance_all_time;
        record.idle_steps = agent.metrics_idle_steps_total;
        breakdown.push_back(record);
    }

    return breakdown;
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
    summary.throughput = safe_divide((double)sim->tasks_completed_total, (double)recorded_steps);
    summary.tasks_per_agent = safe_divide((double)sim->tasks_completed_total, (double)summary.active_agents);
    summary.total_movement_cost = sim->total_movement_cost;
    summary.deadlock_count = sim->deadlock_count;
    summary.total_cpu_time_ms = sim->total_cpu_time_ms;
    summary.avg_cpu_time_ms = safe_divide(sim->total_cpu_time_ms, (double)recorded_steps);
    summary.max_step_cpu_time_ms = sim->max_step_cpu_time_ms;
    summary.avg_cpu_time_per_task_ms = safe_divide(sim->total_cpu_time_ms, (double)sim->tasks_completed_total);
    summary.tasks_per_cpu_second = safe_divide((double)sim->tasks_completed_total * 1000.0, sim->total_cpu_time_ms);
    summary.total_planning_time_ms = sim->total_planning_time_ms;
    summary.avg_planning_time_ms = safe_divide(sim->total_planning_time_ms, (double)recorded_steps);
    summary.max_planning_time_ms = sim->max_planning_time_ms;
    summary.planning_cpu_share = safe_divide(sim->total_planning_time_ms, sim->total_cpu_time_ms);
    summary.avg_planning_time_per_task_ms = safe_divide(sim->total_planning_time_ms, (double)sim->tasks_completed_total);
    summary.tasks_per_planning_second = safe_divide((double)sim->tasks_completed_total * 1000.0, sim->total_planning_time_ms);
    summary.memory_usage_sum_kb = sim->memory_usage_sum_kb;
    summary.avg_memory_usage_kb = safe_divide(sim->memory_usage_sum_kb, (double)sim->memory_samples);
    summary.memory_usage_peak_kb = sim->memory_usage_peak_kb;
    summary.avg_memory_usage_per_agent_kb = safe_divide(summary.avg_memory_usage_kb, (double)summary.active_agents);
    summary.algo_nodes_expanded_total = sim->algo_nodes_expanded_total;
    summary.algo_heap_moves_total = sim->algo_heap_moves_total;
    summary.algo_generated_nodes_total = sim->algo_generated_nodes_total;
    summary.algo_valid_expansions_total = sim->algo_valid_expansions_total;
    summary.valid_expansion_ratio = safe_divide((double)sim->algo_valid_expansions_total, (double)sim->algo_generated_nodes_total);
    summary.avg_nodes_expanded_per_step = safe_divide((double)sim->algo_nodes_expanded_total, (double)recorded_steps);
    summary.avg_nodes_expanded_per_task = safe_divide((double)sim->algo_nodes_expanded_total, (double)sim->tasks_completed_total);
    summary.nodes_expanded_per_planning_ms = safe_divide((double)sim->algo_nodes_expanded_total, sim->total_planning_time_ms);
    summary.heap_moves_per_node_expanded = safe_divide((double)sim->algo_heap_moves_total, (double)sim->algo_nodes_expanded_total);
    summary.requests_created_total = sim->requests_created_total;
    summary.request_wait_ticks_sum = sim->request_wait_ticks_sum;
    summary.avg_request_wait_ticks = safe_divide((double)sim->request_wait_ticks_sum, (double)sim->requests_created_total);
    summary.avg_movement_per_task = safe_divide(sim->total_movement_cost, (double)sim->tasks_completed_total);
    summary.non_task_movement_cost = std::max(sim->total_movement_cost - sim->metrics_sum_dmove, 0.0);
    summary.task_movement_coverage_ratio = safe_divide(sim->metrics_sum_dmove, sim->total_movement_cost);
    summary.non_task_movement_ratio = safe_divide(summary.non_task_movement_cost, sim->total_movement_cost);
    summary.avg_task_distance = safe_divide(sim->metrics_sum_dmove, (double)sim->metrics_task_count);
    summary.avg_task_turns = safe_divide((double)sim->metrics_sum_turns, (double)sim->metrics_task_count);
    summary.avg_task_steps = safe_divide(sim->metrics_sum_ttask, (double)sim->metrics_task_count);
    summary.avg_task_steps_per_cell = safe_divide(sim->metrics_sum_ttask, sim->metrics_sum_dmove);
    summary.avg_task_turns_per_100_cells = safe_divide((double)sim->metrics_sum_turns * 100.0, sim->metrics_sum_dmove);
    summary.steps_with_movement = sim->steps_with_movement;
    summary.stall_step_count = sim->stall_step_count;
    summary.movement_step_ratio = safe_divide((double)sim->steps_with_movement, (double)recorded_steps);
    summary.stall_step_ratio = safe_divide((double)sim->stall_step_count, (double)recorded_steps);
    summary.max_no_movement_streak = sim->max_no_movement_streak;
    summary.last_task_completion_step = sim->last_task_completion_step;
    summary.steps_since_last_task_completion = (sim->last_task_completion_step > 0)
        ? std::max(recorded_steps - sim->last_task_completion_step, 0)
        : recorded_steps;
    summary.queued_task_count = sc ? sc->task_count : 0;
    summary.in_flight_task_count = simulation_in_flight_task_count(am);
    summary.outstanding_task_count = simulation_outstanding_task_count(sim, sc, am);
    summary.avg_outstanding_task_count = safe_divide((double)sim->outstanding_task_sum, (double)sim->outstanding_task_samples);
    summary.peak_outstanding_task_count = sim->outstanding_task_peak;
    summary.avg_outstanding_tasks_per_agent = safe_divide(summary.avg_outstanding_task_count, (double)summary.active_agents);
    summary.peak_outstanding_tasks_per_agent = safe_divide((double)summary.peak_outstanding_task_count, (double)summary.active_agents);
    summary.oldest_queued_request_age = simulation_oldest_queued_request_age(sc);
    summary.avg_oldest_queued_request_age = safe_divide((double)sim->oldest_request_age_sum, (double)sim->outstanding_task_samples);
    summary.peak_oldest_queued_request_age = sim->oldest_request_age_peak;
    summary.planner_wait_edges_sum = sim->planner_metrics.wf_edges_sum;
    summary.planner_conflict_cycle_total = sim->planner_metrics.scc_sum;
    summary.planner_wait_edge_step_count = sim->planner_metrics.wf_step_count;
    summary.planner_cycle_step_count = sim->planner_metrics.scc_step_count;
    summary.planner_wait_edges_per_step = safe_divide((double)sim->planner_metrics.wf_edges_sum, (double)recorded_steps);
    summary.planner_wait_edges_per_conflict_step = safe_divide((double)sim->planner_metrics.wf_edges_sum, (double)sim->planner_metrics.wf_step_count);
    summary.planner_cycle_step_ratio = safe_divide((double)sim->planner_metrics.scc_step_count, (double)recorded_steps);
    summary.planner_cbs_attempt_count = sim->planner_metrics.cbs_attempt_sum;
    summary.planner_cbs_success_count = sim->planner_metrics.cbs_success_sum;
    summary.planner_cbs_failure_count = sim->planner_metrics.cbs_fail_sum;
    summary.planner_cbs_attempt_rate = safe_divide((double)sim->planner_metrics.cbs_attempt_sum, (double)recorded_steps);
    summary.planner_cbs_success_rate = safe_divide((double)sim->planner_metrics.cbs_success_sum, (double)sim->planner_metrics.cbs_attempt_sum);
    summary.planner_cbs_failure_rate = safe_divide((double)sim->planner_metrics.cbs_fail_sum, (double)sim->planner_metrics.cbs_attempt_sum);
    summary.tasks_per_agent_spread = summarize_active_agent_metric(am, [](const Agent& agent) {
        return (double)agent.metrics_completed_tasks_total;
    });
    summary.distance_per_agent_spread = summarize_active_agent_metric(am, [](const Agent& agent) {
        return agent.metrics_total_distance_all_time;
    });
    summary.idle_steps_per_agent_spread = summarize_active_agent_metric(am, [](const Agent& agent) {
        return (double)agent.metrics_idle_steps_total;
    });
    summary.agent_fairness_breakdown = collect_active_agent_fairness_breakdown(am);
    summary.remaining_parked_vehicles = am ? am->total_cars_parked : 0;
    return summary;
}
