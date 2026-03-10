#include "agv/simulation_engine.hpp"

#include "agv/legacy_sim_bridge.hpp"

#include <stdexcept>
#include <string>
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

std::string pathToUtf8(const std::filesystem::path& path) {
    return path.string();
}

}  // namespace

struct SimulationEngine::Impl {
    std::unique_ptr<Simulation, SimulationDeleter> simulation;
    AgvSimulationConfig config{};
    bool dirty{true};
    std::string stepMetricsPath;

    Impl() {
        agv_default_config(&config);
    }

    void refreshPathPointer() {
        config.step_metrics_path = stepMetricsPath.empty() ? nullptr : stepMetricsPath.c_str();
    }

    void rebuildSimulationIfNeeded() {
        if (!dirty && simulation != nullptr) {
            return;
        }

        simulation.reset(simulation_create());
        if (simulation == nullptr) {
            throw std::runtime_error("simulation_create failed");
        }

        refreshPathPointer();
        if (!agv_apply_config(simulation.get(), &config)) {
            throw std::runtime_error("agv_apply_config failed");
        }

        dirty = false;
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
    impl_->config.num_phases = static_cast<int>(config.phases.size());
    if (impl_->config.num_phases > kAgvMaxPhases) {
        impl_->config.num_phases = kAgvMaxPhases;
    }

    for (int i = 0; i < kAgvMaxPhases; ++i) {
        impl_->config.phases[i].type = kPhasePark;
        impl_->config.phases[i].task_count = 1;
    }

    for (int i = 0; i < impl_->config.num_phases; ++i) {
        impl_->config.phases[i].type = toLegacyPhase(config.phases[static_cast<std::size_t>(i)].type);
        impl_->config.phases[i].task_count = config.phases[static_cast<std::size_t>(i)].taskCount;
    }

    impl_->dirty = true;
}

void SimulationEngine::setSuppressOutput(bool suppress) {
    impl_->config.suppress_stdout = suppress ? 1 : 0;
    impl_->dirty = true;
}

void SimulationEngine::setStepMetricsOutput(const std::optional<std::filesystem::path>& outputPath) {
    impl_->stepMetricsPath.clear();
    if (outputPath.has_value()) {
        impl_->stepMetricsPath = pathToUtf8(outputPath.value());
    }
    impl_->dirty = true;
}

void SimulationEngine::prepareConsole() {
    agv_prepare_console();
}

bool SimulationEngine::interactiveSetup() {
    impl_->simulation.reset(simulation_create());
    if (impl_->simulation == nullptr) {
        throw std::runtime_error("simulation_create failed");
    }
    impl_->dirty = false;
    return simulation_setup(impl_->simulation.get()) != 0;
}

void SimulationEngine::runInteractiveConsole() {
    if (impl_->simulation == nullptr) {
        throw std::runtime_error("interactiveSetup must be called first");
    }

    ui_enter_alt_screen();
    try {
        simulation_run(impl_->simulation.get());
    } catch (...) {
        ui_leave_alt_screen();
        throw;
    }
    ui_leave_alt_screen();
}

void SimulationEngine::printPerformanceSummary() const {
    if (impl_->simulation == nullptr) {
        throw std::runtime_error("simulation is not initialized");
    }
    simulation_print_performance_summary(impl_->simulation.get());
}

void SimulationEngine::step() {
    impl_->rebuildSimulationIfNeeded();
    agv_execute_headless_step(impl_->simulation.get());
}

void SimulationEngine::runUntilComplete() {
    impl_->rebuildSimulationIfNeeded();
    if (!agv_run_to_completion(impl_->simulation.get())) {
        throw std::runtime_error("agv_run_to_completion failed");
    }
}

bool SimulationEngine::isComplete() {
    impl_->rebuildSimulationIfNeeded();
    return agv_is_complete(impl_->simulation.get()) != 0;
}

MetricsSnapshot SimulationEngine::snapshotMetrics() {
    impl_->rebuildSimulationIfNeeded();

    AgvRunSummary legacy{};
    agv_collect_run_summary(impl_->simulation.get(), &legacy);

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

RenderFrame SimulationEngine::snapshotFrame(bool paused) {
    impl_->rebuildSimulationIfNeeded();

    std::vector<char> buffer(static_cast<std::size_t>(kAgvRenderBufferSize), '\0');
    const int written = agv_copy_render_frame(
        impl_->simulation.get(), paused ? 1 : 0, buffer.data(), buffer.size());

    RenderFrame frame;
    if (written > 0) {
        frame.text.assign(buffer.data(), static_cast<std::size_t>(written));
    }
    return frame;
}

void SimulationEngine::writeRunSummary(const std::filesystem::path& outputPath) {
    impl_->rebuildSimulationIfNeeded();
    const std::string utf8Path = pathToUtf8(outputPath);
    if (!agv_write_run_summary_json(impl_->simulation.get(), utf8Path.c_str())) {
        throw std::runtime_error("agv_write_run_summary_json failed");
    }
}

}  // namespace agv::core
