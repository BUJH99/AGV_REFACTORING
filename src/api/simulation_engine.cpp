#include "agv/simulation_engine.hpp"

#include "agv/sim_bridge.hpp"

#include <algorithm>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace agv::core {

namespace {

constexpr int kPathAlgoDefault = 0;
constexpr int kPathAlgoAStar = 1;
constexpr int kPathAlgoDStar = 2;
constexpr int kConfigModeCustom = 0;
constexpr int kConfigModeRealtime = 1;
constexpr int kSummaryModeCustom = 1;
constexpr int kSummaryModeRealtime = 2;
constexpr int kPhasePark = 0;
constexpr int kPhaseExit = 1;

struct SimulationDeleter {
    void operator()(Simulation* sim) const noexcept {
        if (sim != nullptr) {
            simulation_destroy(sim);
        }
    }
};

int toLegacyAlgo(PathAlgo algorithm) {
    switch (algorithm) {
        case PathAlgo::AStarSimple:
            return kPathAlgoAStar;
        case PathAlgo::DStarBasic:
            return kPathAlgoDStar;
        case PathAlgo::Default:
        default:
            return kPathAlgoDefault;
    }
}

PathAlgo fromLegacyAlgo(int algorithm) {
    switch (algorithm) {
        case kPathAlgoAStar:
            return PathAlgo::AStarSimple;
        case kPathAlgoDStar:
            return PathAlgo::DStarBasic;
        case kPathAlgoDefault:
        default:
            return PathAlgo::Default;
    }
}

int toLegacyMode(SimulationMode mode) {
    return mode == SimulationMode::Realtime ? kConfigModeRealtime : kConfigModeCustom;
}

SimulationMode fromLegacyMode(int mode) {
    return mode == kSummaryModeRealtime ? SimulationMode::Realtime : SimulationMode::Custom;
}

int toLegacyPhase(PhaseType phase) {
    return phase == PhaseType::Exit ? kPhaseExit : kPhasePark;
}

MetricsSnapshot toMetricsSnapshot(const AgvRunSummary& legacy) {
    MetricsSnapshot snapshot;
    snapshot.seed = legacy.seed;
    snapshot.mapId = legacy.map_id;
    snapshot.algorithm = fromLegacyAlgo(legacy.path_algo);
    snapshot.mode = fromLegacyMode(legacy.mode);
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
    std::unique_ptr<Simulation, SimulationDeleter> simulation;
    AgvSimulationConfig config{};
    bool dirty{true};
    std::vector<char> renderBuffer;

    Impl()
        : renderBuffer(static_cast<std::size_t>(kAgvRenderBufferSize), '\0') {
        agv_default_config(&config);
    }

    Simulation& createFreshSimulation() {
        simulation.reset(simulation_create());
        if (simulation == nullptr) {
            throw std::runtime_error("simulation_create failed");
        }
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
        if (!agv_apply_config(simulation.get(), &config)) {
            throw std::runtime_error("agv_apply_config failed");
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
    impl_->config.path_algo = toLegacyAlgo(algorithm);
    impl_->dirty = true;
}

void SimulationEngine::configureScenario(const ScenarioConfig& config) {
    impl_->config.mode = toLegacyMode(config.mode);
    impl_->config.speed_multiplier = static_cast<float>(config.speedMultiplier);
    impl_->config.realtime_park_chance = config.realtimeParkChance;
    impl_->config.realtime_exit_chance = config.realtimeExitChance;
    impl_->config.num_phases = std::min<int>(static_cast<int>(config.phases.size()), kAgvMaxPhases);

    std::fill_n(impl_->config.phases, kAgvMaxPhases, AgvPhaseConfig{kPhasePark, 1});
    for (int i = 0; i < impl_->config.num_phases; ++i) {
        const auto& phase = config.phases[static_cast<std::size_t>(i)];
        impl_->config.phases[i].type = toLegacyPhase(phase.type);
        impl_->config.phases[i].task_count = phase.taskCount;
    }

    impl_->dirty = true;
}

void SimulationEngine::setSuppressOutput(bool suppress) {
    impl_->config.suppress_stdout = suppress ? 1 : 0;
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
        simulation_run(&simulation);
    } catch (...) {
        ui_leave_alt_screen();
        throw;
    }
    ui_leave_alt_screen();
}

void SimulationEngine::printPerformanceSummary() const {
    simulation_print_performance_summary(&impl_->requireInitialized("simulation is not initialized"));
}

void SimulationEngine::step() {
    agv_execute_headless_step(&impl_->rebuildSimulationIfNeeded());
}

void SimulationEngine::runUntilComplete() {
    Simulation& simulation = impl_->rebuildSimulationIfNeeded();
    if (!agv_run_to_completion(&simulation)) {
        throw std::runtime_error("agv_run_to_completion failed");
    }
}

bool SimulationEngine::isComplete() {
    return agv_is_complete(&impl_->rebuildSimulationIfNeeded()) != 0;
}

MetricsSnapshot SimulationEngine::snapshotMetrics() {
    AgvRunSummary legacy{};
    agv_collect_run_summary(&impl_->rebuildSimulationIfNeeded(), &legacy);
    return toMetricsSnapshot(legacy);
}

RenderFrame SimulationEngine::snapshotFrame(bool paused) {
    const int written = agv_copy_render_frame(
        &impl_->rebuildSimulationIfNeeded(),
        paused ? 1 : 0,
        impl_->renderBuffer.data(),
        impl_->renderBuffer.size());

    RenderFrame frame;
    if (written > 0) {
        frame.text.assign(impl_->renderBuffer.data(), static_cast<std::size_t>(written));
    }
    return frame;
}

}  // namespace agv::core
