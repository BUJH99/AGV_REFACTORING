#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace agv::core {

enum class PathAlgo {
    Default = 0,
    AStarSimple = 1,
    DStarBasic = 2,
};

enum class AgentState {
    Idle = 0,
    GoingToPark = 1,
    ReturningHomeEmpty = 2,
    GoingToCollect = 3,
    ReturningWithCar = 4,
    GoingToCharge = 5,
    Charging = 6,
    ReturningHomeMaintenance = 7,
};

enum class SimulationMode {
    Custom = 0,
    Realtime = 1,
};

enum class PhaseType {
    Park = 0,
    Exit = 1,
};

struct PhaseConfig {
    PhaseType type{PhaseType::Park};
    int taskCount{1};
};

struct ScenarioConfig {
    SimulationMode mode{SimulationMode::Custom};
    double speedMultiplier{0.0};
    int realtimeParkChance{0};
    int realtimeExitChance{0};
    std::vector<PhaseConfig> phases{{PhaseType::Park, 1}};
};

struct MetricsSnapshot {
    std::uint32_t seed{0};
    int mapId{1};
    PathAlgo algorithm{PathAlgo::Default};
    SimulationMode mode{SimulationMode::Custom};
    int activeAgents{0};
    int recordedSteps{0};
    std::uint64_t tasksCompletedTotal{0};
    double throughput{0.0};
    double totalMovementCost{0.0};
    std::uint64_t deadlockCount{0};
    double totalCpuTimeMs{0.0};
    double avgCpuTimeMs{0.0};
    double totalPlanningTimeMs{0.0};
    double avgPlanningTimeMs{0.0};
    double memoryUsageSumKb{0.0};
    double avgMemoryUsageKb{0.0};
    double memoryUsagePeakKb{0.0};
    std::uint64_t algoNodesExpandedTotal{0};
    std::uint64_t algoHeapMovesTotal{0};
    std::uint64_t algoGeneratedNodesTotal{0};
    std::uint64_t algoValidExpansionsTotal{0};
    double validExpansionRatio{0.0};
    std::uint64_t requestsCreatedTotal{0};
    std::uint64_t requestWaitTicksSum{0};
    int remainingParkedVehicles{0};
};

struct RenderFrame {
    std::string text;
};

class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine();

    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;
    SimulationEngine(SimulationEngine&&) noexcept;
    SimulationEngine& operator=(SimulationEngine&&) noexcept;

    void setSeed(std::uint32_t seed);
    void loadMap(int mapId);
    void setAlgorithm(PathAlgo algorithm);
    void configureScenario(const ScenarioConfig& config);
    void setSuppressOutput(bool suppress);

    void prepareConsole();
    bool interactiveSetup();
    void runInteractiveConsole();
    void printPerformanceSummary() const;

    void step();
    void runUntilComplete();
    bool isComplete();
    MetricsSnapshot snapshotMetrics();
    RenderFrame snapshotFrame(bool paused = false);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace agv::core
