#include "agv/simulation_engine.hpp"

#include "agv/internal/engine_internal.hpp"

#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace agv::core {

namespace {

::PathAlgo toInternalAlgo(PathAlgo algorithm) {
    switch (algorithm) {
        case PathAlgo::AStarSimple:
            return ::PathAlgo::AStarSimple;
        case PathAlgo::DStarBasic:
            return ::PathAlgo::DStarBasic;
        case PathAlgo::Default:
        default:
            return ::PathAlgo::Default;
    }
}

PathAlgo fromInternalAlgo(::PathAlgo algorithm) {
    switch (algorithm) {
        case ::PathAlgo::AStarSimple:
            return PathAlgo::AStarSimple;
        case ::PathAlgo::DStarBasic:
            return PathAlgo::DStarBasic;
        case ::PathAlgo::Default:
        default:
            return PathAlgo::Default;
    }
}

::SimulationMode toInternalMode(SimulationMode mode) {
    return mode == SimulationMode::Realtime ? ::SimulationMode::Realtime : ::SimulationMode::Custom;
}

SimulationMode fromInternalMode(::SimulationMode mode) {
    return mode == ::SimulationMode::Realtime ? SimulationMode::Realtime : SimulationMode::Custom;
}

::PhaseType toInternalPhase(PhaseType phase) {
    return phase == PhaseType::Exit ? ::PhaseType::Exit : ::PhaseType::Park;
}

MetricsSnapshot toMetricsSnapshot(const RunSummary& legacy) {
    MetricsSnapshot snapshot;
    snapshot.seed = legacy.seed;
    snapshot.mapId = legacy.map_id;
    snapshot.algorithm = fromInternalAlgo(legacy.path_algo);
    snapshot.mode = fromInternalMode(legacy.mode);
    snapshot.activeAgents = legacy.active_agents;
    snapshot.recordedSteps = legacy.recorded_steps;
    snapshot.tasksCompletedTotal = legacy.tasks_completed_total;
    snapshot.throughput = legacy.throughput;
    snapshot.totalMovementCost = legacy.total_movement_cost;
    snapshot.deadlockCount = legacy.deadlock_count;
    snapshot.totalCpuTimeMs = legacy.total_cpu_time_ms;
    snapshot.avgCpuTimeMs = legacy.avg_cpu_time_ms;
    snapshot.totalPlanningTimeMs = legacy.total_planning_time_ms;
    snapshot.avgPlanningTimeMs = legacy.avg_planning_time_ms;
    snapshot.memoryUsageSumKb = legacy.memory_usage_sum_kb;
    snapshot.avgMemoryUsageKb = legacy.avg_memory_usage_kb;
    snapshot.memoryUsagePeakKb = legacy.memory_usage_peak_kb;
    snapshot.algoNodesExpandedTotal = legacy.algo_nodes_expanded_total;
    snapshot.algoHeapMovesTotal = legacy.algo_heap_moves_total;
    snapshot.algoGeneratedNodesTotal = legacy.algo_generated_nodes_total;
    snapshot.algoValidExpansionsTotal = legacy.algo_valid_expansions_total;
    snapshot.validExpansionRatio = legacy.valid_expansion_ratio;
    snapshot.requestsCreatedTotal = legacy.requests_created_total;
    snapshot.requestWaitTicksSum = legacy.request_wait_ticks_sum;
    snapshot.remainingParkedVehicles = legacy.remaining_parked_vehicles;
    return snapshot;
}

}  // namespace

struct SimulationEngine::Impl {
    std::unique_ptr<Simulation> simulation;
    SimulationConfig config{};
    bool dirty{true};
 
    Impl()
        : config(default_simulation_config()) {
    }

    Simulation& createFreshSimulation() {
        simulation = std::make_unique<Simulation>();
        return *simulation;
    }

    Simulation& requireInitialized(std::string_view operation) const {
        if (simulation == nullptr) {
            throw std::runtime_error(std::string(operation));
        }
        return *simulation;
    }

    Simulation& rebuildSimulationIfNeeded() {
        if (!dirty && simulation != nullptr) {
            return *simulation;
        }

        createFreshSimulation();
        if (!apply_simulation_config(simulation.get(), config)) {
            throw std::runtime_error("apply_simulation_config failed");
        }

        dirty = false;
        return *simulation;
    }
};

SimulationEngine::SimulationEngine()
    : impl_(std::make_unique<Impl>()) {}

SimulationEngine::~SimulationEngine() = default;

SimulationEngine::SimulationEngine(SimulationEngine&&) noexcept = default;

SimulationEngine& SimulationEngine::operator=(SimulationEngine&&) noexcept = default;

void SimulationEngine::setSeed(std::uint32_t seed) {
    impl_->config.seed = seed;
    impl_->dirty = true;
}

void SimulationEngine::loadMap(int mapId) {
    impl_->config.map_id = mapId;
    impl_->dirty = true;
}

void SimulationEngine::setAlgorithm(PathAlgo algorithm) {
    impl_->config.path_algo = toInternalAlgo(algorithm);
    impl_->dirty = true;
}

void SimulationEngine::configureScenario(const ScenarioConfig& config) {
    impl_->config.mode = toInternalMode(config.mode);
    impl_->config.speed_multiplier = static_cast<float>(config.speedMultiplier);
    impl_->config.realtime_park_chance = config.realtimeParkChance;
    impl_->config.realtime_exit_chance = config.realtimeExitChance;
    impl_->config.num_phases = std::min<int>(static_cast<int>(config.phases.size()), MAX_PHASES);

    std::fill(impl_->config.phases.begin(), impl_->config.phases.end(), ConfigPhase{::PhaseType::Park, 1});
    for (int i = 0; i < impl_->config.num_phases; ++i) {
        const auto& phase = config.phases[static_cast<std::size_t>(i)];
        impl_->config.phases[i].type = toInternalPhase(phase.type);
        impl_->config.phases[i].task_count = phase.taskCount;
    }

    impl_->dirty = true;
}

void SimulationEngine::setSuppressOutput(bool suppress) {
    impl_->config.suppress_stdout = suppress;
    impl_->dirty = true;
}

void SimulationEngine::prepareConsole() {
    agv_prepare_console();
}

bool SimulationEngine::interactiveSetup() {
    Simulation& simulation = impl_->createFreshSimulation();
    impl_->dirty = false;
    return simulation_setup(&simulation) != 0;
}

void SimulationEngine::runInteractiveConsole() {
    Simulation& simulation = impl_->requireInitialized("interactiveSetup must be called first");

    ui_enter_alt_screen();
    try {
        simulation.run();
    } catch (...) {
        ui_leave_alt_screen();
        throw;
    }
    ui_leave_alt_screen();
}

void SimulationEngine::printPerformanceSummary() const {
    impl_->requireInitialized("simulation is not initialized").printPerformanceSummary();
}

void SimulationEngine::step() {
    execute_headless_step(&impl_->rebuildSimulationIfNeeded());
}

void SimulationEngine::runUntilComplete() {
    Simulation& simulation = impl_->rebuildSimulationIfNeeded();
    if (!run_simulation_to_completion(&simulation)) {
        throw std::runtime_error("run_simulation_to_completion failed");
    }
}

bool SimulationEngine::isComplete() {
    return impl_->rebuildSimulationIfNeeded().isComplete();
}

MetricsSnapshot SimulationEngine::snapshotMetrics() {
    return toMetricsSnapshot(collect_run_summary(&impl_->rebuildSimulationIfNeeded()));
}

RenderFrame SimulationEngine::snapshotFrame(bool paused) {
    RenderFrame frame;
    frame.text = build_render_frame_text(&impl_->rebuildSimulationIfNeeded(), paused);
    return frame;
}

}  // namespace agv::core
