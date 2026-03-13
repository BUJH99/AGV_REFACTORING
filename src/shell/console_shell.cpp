#include "agv/console_shell.hpp"

#include "agv/internal/console_launch_wizard.hpp"
#include "agv/internal/engine_internal.hpp"
#include "agv/internal/simulation_engine_access.hpp"

#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

void simulation_display_status(Simulation* sim, bool is_paused);
RendererFacade renderer_create_facade(void);

namespace {

using agv::core::AgentDebugSnapshot;
using agv::core::AgentState;
using agv::core::DebugSnapshot;
using agv::core::MetricsSnapshot;
using agv::core::PhaseType;
using agv::core::SimulationEngine;
using agv::core::SimulationMode;
using agv::internal::SimulationEngineAccess;

template <typename... Args>
std::string linef(std::string_view format, Args&&... args) {
    return agv::internal::text::printf_like(format, std::forward<Args>(args)...);
}

std::string_view mode_label(SimulationMode mode) {
    return (mode == SimulationMode::Realtime) ? "Real-Time" : "Custom";
}

std::string_view algorithm_label(agv::core::PathAlgo algorithm) {
    switch (algorithm) {
        case agv::core::PathAlgo::AStarSimple:
            return "A* (Single-Agent)";
        case agv::core::PathAlgo::DStarBasic:
            return "D* Lite (Incremental)";
        case agv::core::PathAlgo::Default:
        default:
            return "Default (WHCA* + D* Lite + WFG + CBS)";
    }
}

std::string_view algorithm_label(::PathAlgo algorithm) {
    switch (algorithm) {
        case ::PathAlgo::AStarSimple:
            return "A* (Single-Agent)";
        case ::PathAlgo::DStarBasic:
            return "D* Lite (Incremental)";
        case ::PathAlgo::Default:
        default:
            return "Default (WHCA* + D* Lite + WFG + CBS)";
    }
}

std::string_view mode_label(::SimulationMode mode) {
    return (mode == ::SimulationMode::Realtime) ? "Real-Time" : "Custom";
}

std::string_view phase_label(PhaseType phase) {
    return (phase == PhaseType::Exit) ? "exit" : "park";
}

std::string_view agent_state_label(AgentState state) {
    switch (state) {
        case AgentState::GoingToPark:
            return "GoingToPark";
        case AgentState::ReturningHomeEmpty:
            return "ReturningHomeEmpty";
        case AgentState::GoingToCollect:
            return "GoingToCollect";
        case AgentState::ReturningWithCar:
            return "ReturningWithCar";
        case AgentState::GoingToCharge:
            return "GoingToCharge";
        case AgentState::Charging:
            return "Charging";
        case AgentState::ReturningHomeMaintenance:
            return "ReturningHomeMaintenance";
        case AgentState::Idle:
        default:
            return "Idle";
    }
}

void attach_console_renderer(Simulation& simulation) {
    simulation.renderer = renderer_create_facade();
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

std::string build_agent_block(const AgentDebugSnapshot& agent) {
    return linef(
        " AGV %c (#%d) state=%s active=%s pos=(%d,%d) last=(%d,%d) home=(%d,%d) goal=(%d,%d) "
        "dist=%.2f charge=%d action=%d rot=%d task=%s age=%d task_dist=%.2f turns=%d stuck=%d osc=%d\n",
        agent.symbol,
        agent.id,
        agent_state_label(agent.state).data(),
        agent.isActive ? "true" : "false",
        agent.posX,
        agent.posY,
        agent.lastPosX,
        agent.lastPosY,
        agent.homeX,
        agent.homeY,
        agent.goalX,
        agent.goalY,
        agent.totalDistanceTraveled,
        agent.chargeTimer,
        agent.actionTimer,
        agent.rotationWait,
        agent.taskActive ? "true" : "false",
        agent.taskAgeSteps,
        agent.taskDistance,
        agent.taskTurns,
        agent.stuckSteps,
        agent.oscillationSteps);
}

std::string build_debug_report_text(SimulationEngine& engine, bool paused) {
    Simulation& simulation = SimulationEngineAccess::requireInitialized(engine, "buildDebugReport");
    const DebugSnapshot snapshot = engine.snapshotDebugState(paused);
    const MetricsSnapshot& metrics = snapshot.metrics;

    std::string report;
    report.reserve(16384);

    report += "Simulation Debug Report\n";
    report += "=======================\n\n";

    report += "Session Summary\n";
    report += "---------------\n";
    report += linef("Seed              : %u\n", metrics.seed);
    report += linef("Map               : %d\n", metrics.mapId);
    report += linef("Algorithm         : %s\n", algorithm_label(metrics.algorithm).data());
    report += linef("Mode              : %s\n", mode_label(metrics.mode).data());
    report += linef("Recorded Steps    : %d\n", metrics.recordedSteps);
    report += linef("Tasks Completed   : %llu\n", metrics.tasksCompletedTotal);
    report += linef("Outstanding Tasks : %d\n", metrics.outstandingTaskCount);
    report += linef("Deadlocks         : %llu\n\n", metrics.deadlockCount);

    report += "Runtime Snapshot\n";
    report += "----------------\n";
    report += linef("Current Step      : %d\n", snapshot.runtime.currentStep);
    report += linef("Current Phase     : %d / %d\n", snapshot.runtime.currentPhaseIndex, snapshot.runtime.totalPhases);
    report += linef("Phase Type        : %s\n", phase_label(snapshot.runtime.currentPhaseType).data());
    report += linef("Phase Progress    : %d / %d (remaining %d)\n",
        snapshot.runtime.phaseTasksCompleted,
        snapshot.runtime.phaseTaskTarget,
        snapshot.runtime.phaseRemainingTasks);
    report += linef("Queued/InFlight   : %d / %d\n",
        snapshot.runtime.queuedTaskCount,
        snapshot.runtime.inFlightTaskCount);
    report += linef("Ready/Waiting     : %d / %d\n",
        snapshot.runtime.readyIdleAgentCount,
        snapshot.runtime.lastWaitingAgentCount);
    report += linef("Stuck/Oscillating : %d / %d\n",
        snapshot.runtime.lastStuckAgentCount,
        snapshot.runtime.lastOscillatingAgentCount);
    report += linef("Planner Wait/SCC  : %d / %d\n",
        snapshot.runtime.plannerWaitEdges,
        snapshot.runtime.plannerSccCount);
    report += linef("Planner CBS       : %d expansions=%d\n",
        snapshot.runtime.plannerCbsSucceeded,
        snapshot.runtime.plannerCbsExpansions);
    report += linef("Step CPU / Plan   : %.4f / %.4f ms\n\n",
        snapshot.runtime.lastStepCpuTimeMs,
        snapshot.runtime.lastPlanningTimeMs);

    report += "Planner Derived KPIs\n";
    report += "--------------------\n";
    report += linef("Throughput         : %.4f task/step\n", metrics.throughput);
    report += linef("Planning CPU Share : %.4f\n", metrics.planningCpuShare);
    report += linef("Tasks/AGV          : %.4f\n", metrics.tasksPerAgent);
    report += linef("Avg Task Distance  : %.4f\n", metrics.avgTaskDistance);
    report += linef("Avg Task Duration  : %.4f\n", metrics.avgTaskSteps);
    report += linef("Wait Edge Density  : %.4f / %.4f\n",
        metrics.plannerWaitEdgesPerStep,
        metrics.plannerWaitEdgesPerConflictStep);
    report += linef("CBS Success/Fail   : %lld / %lld\n\n",
        metrics.plannerCbsSuccessCount,
        metrics.plannerCbsFailureCount);

    report += "AGV Fairness\n";
    report += "------------\n";
    report += linef("Tasks/AGV cv/min-max    : %.4f / %.4f\n",
        metrics.tasksPerAgentSpread.coefficientOfVariation,
        metrics.tasksPerAgentSpread.minMaxRatio);
    report += linef("Distance cv/min-max     : %.4f / %.4f\n",
        metrics.distancePerAgentSpread.coefficientOfVariation,
        metrics.distancePerAgentSpread.minMaxRatio);
    report += linef("Idle Steps cv/min-max   : %.4f / %.4f\n",
        metrics.idleStepsPerAgentSpread.coefficientOfVariation,
        metrics.idleStepsPerAgentSpread.minMaxRatio);
    for (const auto& agent : metrics.agentFairnessBreakdown) {
        report += linef(" AGV %c : tasks=%llu distance=%.2f idle=%llu\n",
            agent.symbol,
            agent.tasksCompleted,
            agent.distanceCells,
            agent.idleSteps);
    }
    report += '\n';

    report += "Recent Logs\n";
    report += "-----------\n";
    if (snapshot.recentLogs.empty()) {
        report += "(none)\n";
    } else {
        for (const auto& entry : snapshot.recentLogs) {
            report += entry;
            report += '\n';
        }
    }
    report += '\n';

    report += "Agent Snapshots\n";
    report += "---------------\n";
    for (const auto& agent : snapshot.agents) {
        report += build_agent_block(agent);
    }
    report += '\n';

    report += "Deadlock Snapshot\n";
    report += "-----------------\n";
    report += linef("Has Event         : %s\n", snapshot.deadlock.hasEvent ? "true" : "false");
    report += linef("Reason            : %s\n", snapshot.deadlock.reason.c_str());
    report += linef("Participants      : %zu\n\n", snapshot.deadlock.participantAgentIds.size());

    report += "Render Frame\n";
    report += "------------\n";
    report += build_render_frame_text(&simulation, paused);

    return report;
}

int read_control_key() {
    return console_read_key_nonblocking().value_or(0);
}

bool should_wait_while_paused(bool is_paused, int last_key) {
    return is_paused && std::tolower(last_key) != 's';
}

void handle_control_input(
    Simulation* sim,
    int last_key,
    bool& is_paused,
    bool& quit_flag,
    bool& return_to_menu) {
    if (!last_key) {
        return;
    }

    ui_handle_control_key(sim, last_key, is_paused, quit_flag, return_to_menu);
    if (quit_flag || return_to_menu) {
        return;
    }

    if (is_paused && std::tolower(last_key) == 's') {
        sim->render_state.force_next_flush = true;
        return;
    }

    sim->render_state.force_next_flush = true;
    sim->renderer.drawFrame(sim, is_paused);
}

void maybe_sleep_for_console_speed(const ScenarioManager* scenario) {
    if (scenario && scenario->simulation_speed > 0) {
        platform_sleep_for_ms(scenario->simulation_speed);
    }
}

}  // namespace

bool Simulation_::run() {
    bool is_paused = false;
    bool quit_flag = false;
    bool return_to_menu = false;

    resetRuntimeStats();
    render_state.force_next_flush = true;
    renderer.drawFrame(this, is_paused);

    while (!quit_flag && !return_to_menu) {
        const int last_key = read_control_key();
        handle_control_input(this, last_key, is_paused, quit_flag, return_to_menu);
        if (quit_flag || return_to_menu) {
            continue;
        }
        if (should_wait_while_paused(is_paused, last_key)) {
            platform_sleep_for_ms(PAUSE_POLL_INTERVAL_MS);
            continue;
        }

        execute_headless_step(this, false);
        renderer.drawFrame(this, is_paused);
        if (isComplete()) {
            break;
        }
        maybe_sleep_for_console_speed(scenario_manager);
    }

    return return_to_menu;
}

namespace agv::console {

void prepareConsole() {
    agv_prepare_console();
}

bool interactiveSetup(core::SimulationEngine& engine) {
    const std::filesystem::path last_launch_path = agv::internal::console::default_last_launch_path();
    core::LaunchConfig chosen;
    if (!agv::internal::console::run_console_launch_wizard(std::cin, std::cout, last_launch_path, chosen)) {
        return false;
    }

    (void)agv::internal::console::save_last_launch_config(last_launch_path, chosen);
    engine.setTerminalOutputEnabled(true);
    engine.setCaptureLevel(core::CaptureLevel::Frame);
    engine.configureLaunch(chosen);
    engine.startConfiguredSession();

    Simulation& simulation = SimulationEngineAccess::requireInitialized(engine, "interactiveSetup");
    attach_console_renderer(simulation);
    return true;
}

bool runInteractiveConsole(core::SimulationEngine& engine) {
    Simulation& simulation = SimulationEngineAccess::requireInitialized(engine, "runInteractiveConsole");
    attach_console_renderer(simulation);

    ui_enter_alt_screen();
    const bool return_to_menu = simulation.run();
    ui_leave_alt_screen();
    return return_to_menu;
}

void printPerformanceSummary(core::SimulationEngine& engine) {
    Simulation& simulation = SimulationEngineAccess::requireInitialized(engine, "printPerformanceSummary");
    const RunSummary summary = collect_run_summary(&simulation);

    agv::internal::text::console_print("\n============================================\n");
    agv::internal::text::console_print("          Simulation Result Report\n");
    agv::internal::text::console_print("============================================\n");
    agv::internal::text::console_print(" Mode                                : %s\n", mode_label(summary.mode).data());
    agv::internal::text::console_print(" Map ID                              : %d\n", summary.map_id);
    agv::internal::text::console_print(" Path Planning Algorithm             : %s\n", algorithm_label(summary.path_algo).data());
    agv::internal::text::console_print(" Total Physical Time Steps           : %d\n", summary.recorded_steps);
    agv::internal::text::console_print(" Operating AGVs                      : %d\n", summary.active_agents);
    agv::internal::text::console_print(" Tasks Completed (total)             : %llu\n", summary.tasks_completed_total);
    agv::internal::text::console_print(" Throughput [task / step]            : %.4f\n", summary.throughput);
    agv::internal::text::console_print(" Tasks Completed / AGV               : %.2f\n", summary.tasks_per_agent);
    agv::internal::text::console_print(" Total Movement Cost (cells)         : %.2f\n", summary.total_movement_cost);
    agv::internal::text::console_print(" Deadlocks                           : %llu\n", summary.deadlock_count);
    agv::internal::text::console_print(" Outstanding Tasks (current/avg/peak): %d / %.2f / %d\n",
        summary.outstanding_task_count,
        summary.avg_outstanding_task_count,
        summary.peak_outstanding_task_count);
    agv::internal::text::console_print(" Planning Time (total/avg/max)       : %.2f / %.4f / %.4f ms\n",
        summary.total_planning_time_ms,
        summary.avg_planning_time_ms,
        summary.max_planning_time_ms);
    agv::internal::text::console_print(" Planner Wait Edge Density           : %.4f / %.4f\n",
        summary.planner_wait_edges_per_step,
        summary.planner_wait_edges_per_conflict_step);
    agv::internal::text::console_print(" CBS Attempts / Success / Fail       : %lld / %lld / %lld\n",
        summary.planner_cbs_attempt_count,
        summary.planner_cbs_success_count,
        summary.planner_cbs_failure_count);
    agv::internal::text::console_print(" Tasks / AGV Balance (cv/min-max)    : %.4f / %.4f\n",
        summary.tasks_per_agent_spread.coefficient_of_variation,
        summary.tasks_per_agent_spread.min_max_ratio);
    agv::internal::text::console_print(" Distance / AGV Balance (cv/min-max) : %.4f / %.4f\n",
        summary.distance_per_agent_spread.coefficient_of_variation,
        summary.distance_per_agent_spread.min_max_ratio);
    agv::internal::text::console_print("============================================\n");
}

std::string buildDebugReport(core::SimulationEngine& engine, bool paused) {
    return build_debug_report_text(engine, paused);
}

bool writeDebugReport(core::SimulationEngine& engine, const std::string& path, bool paused) {
    const std::string report = build_debug_report_text(engine, paused);
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }
    output << report;
    return output.good();
}

}  // namespace agv::console
