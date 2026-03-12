#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
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

struct LaunchConfig {
    std::uint32_t seed{0};
    int mapId{1};
    PathAlgo algorithm{PathAlgo::Default};
    ScenarioConfig scenario{};
};

struct HeadlessRunOptions {
    std::optional<int> maxSteps{};
    bool renderOutputEnabled{false};
    std::optional<std::string> debugReportPath{};
    std::optional<std::string> deadlockReportPath{};
    bool stopOnDeadlock{false};
};

struct ValidationIssue {
    std::string field{};
    std::string code{};
    std::string message{};
};

struct ValidationResult {
    LaunchConfig normalizedConfig{};
    std::vector<ValidationIssue> errors{};
    std::vector<ValidationIssue> warnings{};

    [[nodiscard]] bool ok() const { return errors.empty(); }
};

struct SessionDescriptor {
    std::uint64_t sessionId{0};
    std::uint64_t sceneVersion{0};
    std::uint64_t frameId{0};
    LaunchConfig launchConfig{};
};

struct BurstRunResult {
    int executedSteps{0};
    bool complete{false};
    std::uint64_t frameId{0};
    std::uint64_t lastLogSeq{0};
};

struct DistributionSummary {
    double min{0.0};
    double avg{0.0};
    double max{0.0};
    double stddev{0.0};
    double coefficientOfVariation{0.0};
    double minMaxRatio{0.0};
};

struct AgentFairnessSnapshot {
    int id{0};
    char symbol{0};
    std::uint64_t tasksCompleted{0};
    double distanceCells{0.0};
    std::uint64_t idleSteps{0};
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
    double tasksPerAgent{0.0};
    double totalMovementCost{0.0};
    std::uint64_t deadlockCount{0};
    double totalCpuTimeMs{0.0};
    double avgCpuTimeMs{0.0};
    double maxStepCpuTimeMs{0.0};
    double avgCpuTimePerTaskMs{0.0};
    double tasksPerCpuSecond{0.0};
    double totalPlanningTimeMs{0.0};
    double avgPlanningTimeMs{0.0};
    double maxPlanningTimeMs{0.0};
    double planningCpuShare{0.0};
    double avgPlanningTimePerTaskMs{0.0};
    double tasksPerPlanningSecond{0.0};
    double memoryUsageSumKb{0.0};
    double avgMemoryUsageKb{0.0};
    double memoryUsagePeakKb{0.0};
    double avgMemoryUsagePerAgentKb{0.0};
    std::uint64_t algoNodesExpandedTotal{0};
    std::uint64_t algoHeapMovesTotal{0};
    std::uint64_t algoGeneratedNodesTotal{0};
    std::uint64_t algoValidExpansionsTotal{0};
    double validExpansionRatio{0.0};
    double avgNodesExpandedPerStep{0.0};
    double avgNodesExpandedPerTask{0.0};
    double nodesExpandedPerPlanningMs{0.0};
    double heapMovesPerNodeExpanded{0.0};
    std::uint64_t requestsCreatedTotal{0};
    std::uint64_t requestWaitTicksSum{0};
    double avgRequestWaitTicks{0.0};
    double avgMovementPerTask{0.0};
    double nonTaskMovementCost{0.0};
    double taskMovementCoverageRatio{0.0};
    double nonTaskMovementRatio{0.0};
    double avgTaskDistance{0.0};
    double avgTaskTurns{0.0};
    double avgTaskSteps{0.0};
    double avgTaskStepsPerCell{0.0};
    double avgTaskTurnsPer100Cells{0.0};
    std::uint64_t stepsWithMovement{0};
    std::uint64_t stallStepCount{0};
    double movementStepRatio{0.0};
    double stallStepRatio{0.0};
    std::uint64_t maxNoMovementStreak{0};
    int lastTaskCompletionStep{0};
    int stepsSinceLastTaskCompletion{0};
    int queuedTaskCount{0};
    int inFlightTaskCount{0};
    int outstandingTaskCount{0};
    double avgOutstandingTaskCount{0.0};
    int peakOutstandingTaskCount{0};
    double avgOutstandingTasksPerAgent{0.0};
    double peakOutstandingTasksPerAgent{0.0};
    int oldestQueuedRequestAge{0};
    double avgOldestQueuedRequestAge{0.0};
    int peakOldestQueuedRequestAge{0};
    long long plannerWaitEdgesSum{0};
    long long plannerConflictCycleTotal{0};
    long long plannerWaitEdgeStepCount{0};
    long long plannerCycleStepCount{0};
    double plannerWaitEdgesPerStep{0.0};
    double plannerWaitEdgesPerConflictStep{0.0};
    double plannerCycleStepRatio{0.0};
    long long plannerCbsAttemptCount{0};
    long long plannerCbsSuccessCount{0};
    long long plannerCbsFailureCount{0};
    double plannerCbsAttemptRate{0.0};
    double plannerCbsSuccessRate{0.0};
    double plannerCbsFailureRate{0.0};
    DistributionSummary tasksPerAgentSpread{};
    DistributionSummary distancePerAgentSpread{};
    DistributionSummary idleStepsPerAgentSpread{};
    std::vector<AgentFairnessSnapshot> agentFairnessBreakdown{};
    int remainingParkedVehicles{0};
};

struct GridCoord {
    int x{-1};
    int y{-1};
};

struct HomeCellSnapshot {
    int agentId{0};
    char symbol{0};
    GridCoord position{};
};

struct GoalRenderState {
    GridCoord position{};
    bool isGoal{false};
    bool isParked{false};
    int reservedByAgent{-1};
};

enum class AgentWaitReason {
    None = 0,
    Idle = 1,
    Charging = 2,
    GoalAction = 3,
    Rotation = 4,
    BlockedByStationary = 5,
    PriorityYield = 6,
    Stuck = 7,
    Oscillating = 8,
};

struct StructuredLogEntry {
    std::uint64_t seq{0};
    int step{0};
    std::uint64_t frameId{0};
    std::string category{};
    std::string level{};
    std::string text{};
    std::optional<int> agentId{};
    std::optional<int> phaseIndex{};
};

struct OverlayWaitEdge {
    int fromAgentId{0};
    int toAgentId{0};
    int timeOffset{0};
    std::string cause{};
    GridCoord from{};
    GridCoord to{};
};

struct OverlayPlannedPath {
    int agentId{0};
    std::vector<GridCoord> cells{};
};

struct PlannerOverlaySnapshot {
    bool available{false};
    PathAlgo algorithm{PathAlgo::Default};
    int horizon{0};
    int waitEdgeCount{0};
    int leaderAgentId{-1};
    bool usedCbs{false};
    std::vector<int> sccParticipantAgentIds{};
    std::vector<int> yieldAgentIds{};
    std::vector<int> pullOverAgentIds{};
    std::vector<OverlayWaitEdge> waitEdges{};
    std::vector<OverlayPlannedPath> plannedPaths{};
    std::vector<OverlayPlannedPath> cbsPaths{};
};

struct HudSnapshot {
    int mapId{1};
    PathAlgo algorithm{PathAlgo::Default};
    SimulationMode mode{SimulationMode::Custom};
    int step{0};
    bool paused{false};
    double speedMultiplier{0.0};
    int currentPhaseIndex{-1};
    int totalPhases{0};
    PhaseType currentPhaseType{PhaseType::Park};
    int phaseTaskTarget{0};
    int phaseTasksCompleted{0};
    int phaseRemainingTasks{0};
    int queuedTaskCount{0};
    int inFlightTaskCount{0};
    int outstandingTaskCount{0};
    int readyIdleAgentCount{0};
    int activeGoalActionCount{0};
    int waitingAgentCount{0};
    int stuckAgentCount{0};
    int oscillatingAgentCount{0};
    int noMovementStreak{0};
    int maxNoMovementStreak{0};
    int lastTaskCompletionStep{0};
    int stepsSinceLastTaskCompletion{0};
    int oldestQueuedRequestAge{0};
    int parkedCars{0};
    int totalGoalCount{0};
    double lastStepCpuTimeMs{0.0};
    double lastPlanningTimeMs{0.0};
    double avgCpuTimeMs{0.0};
    double totalCpuTimeMs{0.0};
    int plannedMoveCount{0};
    int postRotationMoveCount{0};
    int postBlockerMoveCount{0};
    int finalMoveCount{0};
    int rotationCanceledCount{0};
    int blockerCanceledCount{0};
    int orderCanceledCount{0};
    int plannerWaitEdges{0};
    int plannerSccCount{0};
    bool plannerCbsSucceeded{false};
    int plannerCbsExpansions{0};
    int whcaHorizon{0};
};

struct AgentRenderState {
    int id{0};
    char symbol{0};
    AgentState state{AgentState::Idle};
    bool isActive{false};
    GridCoord position{};
    GridCoord lastPosition{};
    GridCoord home{};
    GridCoord goal{};
    double totalDistanceTraveled{0.0};
    int chargeTimer{0};
    int actionTimer{0};
    int rotationWait{0};
    bool taskActive{false};
    int taskAgeSteps{0};
    double taskDistance{0.0};
    int taskTurns{0};
    bool goalActionInProgress{false};
    bool movedLastStep{false};
    AgentWaitReason waitReason{AgentWaitReason::None};
    int stuckSteps{0};
    int oscillationSteps{0};
};

struct StaticSceneSnapshot {
    std::uint64_t sessionId{0};
    std::uint64_t sceneVersion{0};
    int mapId{1};
    int width{0};
    int height{0};
    std::string baseTiles{};
    std::vector<GridCoord> goalCells{};
    std::vector<GridCoord> chargerCells{};
    std::vector<HomeCellSnapshot> homeCells{};
};

struct RenderQueryOptions {
    bool paused{false};
    bool logsTail{true};
    std::size_t maxLogEntries{8};
    bool plannerOverlay{false};
};

struct RenderFrameSnapshot {
    std::uint64_t sessionId{0};
    std::uint64_t sceneVersion{0};
    std::uint64_t frameId{0};
    std::uint64_t lastLogSeq{0};
    HudSnapshot hud{};
    std::vector<AgentRenderState> agents{};
    std::vector<GoalRenderState> goalStates{};
    std::vector<StructuredLogEntry> logsTail{};
    PlannerOverlaySnapshot plannerOverlay{};
};

struct RenderFrameDelta {
    std::uint64_t sessionId{0};
    std::uint64_t sceneVersion{0};
    std::uint64_t fromFrameId{0};
    std::uint64_t toFrameId{0};
    std::uint64_t lastLogSeq{0};
    bool requiresFullResync{false};
    bool hudChanged{false};
    bool overlayChanged{false};
    HudSnapshot hud{};
    std::vector<AgentRenderState> agentUpdates{};
    std::vector<GoalRenderState> goalStateChanges{};
    std::vector<StructuredLogEntry> newLogs{};
    PlannerOverlaySnapshot plannerOverlay{};
};

struct RenderFrame {
    std::string text;
};

struct AgentDebugSnapshot {
    int id{0};
    char symbol{0};
    AgentState state{AgentState::Idle};
    bool isActive{false};
    int posX{-1};
    int posY{-1};
    int lastPosX{-1};
    int lastPosY{-1};
    int homeX{-1};
    int homeY{-1};
    int goalX{-1};
    int goalY{-1};
    double totalDistanceTraveled{0.0};
    int chargeTimer{0};
    int actionTimer{0};
    int rotationWait{0};
    bool taskActive{false};
    int taskAgeSteps{0};
    double taskDistance{0.0};
    int taskTurns{0};
    int stuckSteps{0};
    int oscillationSteps{0};
};

struct DeadlockSnapshot {
    bool hasEvent{false};
    int step{0};
    std::uint64_t deadlockCount{0};
    int phaseIndex{-1};
    int phaseTaskTarget{0};
    int phaseTasksCompleted{0};
    int pendingTaskCount{0};
    int activeAgentCount{0};
    int waitingAgentCount{0};
    int stuckAgentCount{0};
    int plannerWaitEdges{0};
    int plannerSccCount{0};
    int plannerCbsSucceeded{0};
    int plannerCbsExpansions{0};
    int whcaHorizon{0};
    int plannedMoveCount{0};
    int postRotationMoveCount{0};
    int postBlockerMoveCount{0};
    int finalMoveCount{0};
    std::vector<int> rotationCanceledAgentIds;
    std::vector<int> blockerCanceledAgentIds;
    std::vector<int> orderCanceledAgentIds;
    std::vector<int> participantAgentIds;
    std::string reason;
};

struct RuntimeDebugSnapshot {
    int currentStep{0};
    bool hasActivePhase{false};
    bool inCleanupRegion{false};
    int currentPhaseIndex{-1};
    int totalPhases{0};
    PhaseType currentPhaseType{PhaseType::Park};
    int phaseTaskTarget{0};
    int phaseTasksCompleted{0};
    int phaseRemainingTasks{0};
    int queuedTaskCount{0};
    int inFlightTaskCount{0};
    int outstandingTaskCount{0};
    int readyIdleAgentCount{0};
    int activeGoalActionCount{0};
    int lastActiveAgentCount{0};
    int lastWaitingAgentCount{0};
    int lastStuckAgentCount{0};
    int lastOscillatingAgentCount{0};
    int noMovementStreak{0};
    int maxNoMovementStreak{0};
    int lastTaskCompletionStep{0};
    int stepsSinceLastTaskCompletion{0};
    int oldestQueuedRequestAge{0};
    double lastStepCpuTimeMs{0.0};
    double lastPlanningTimeMs{0.0};
    int plannedMoveCount{0};
    int postRotationMoveCount{0};
    int postBlockerMoveCount{0};
    int finalMoveCount{0};
    std::vector<int> rotationCanceledAgentIds;
    std::vector<int> blockerCanceledAgentIds;
    std::vector<int> orderCanceledAgentIds;
    int plannerWaitEdges{0};
    int plannerSccCount{0};
    int plannerCbsSucceeded{0};
    int plannerCbsExpansions{0};
    int whcaHorizon{0};
};

struct DebugSnapshot {
    MetricsSnapshot metrics;
    RuntimeDebugSnapshot runtime;
    RenderFrame frame;
    std::vector<std::string> recentLogs;
    std::vector<AgentDebugSnapshot> agents;
    DeadlockSnapshot deadlock;
};

ValidationResult validateLaunchConfig(const LaunchConfig& config);

class SimulationEngine {
public:
    SimulationEngine();
    ~SimulationEngine();

    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;
    SimulationEngine(SimulationEngine&&) noexcept;
    SimulationEngine& operator=(SimulationEngine&&) noexcept;

    void configureLaunch(const LaunchConfig& config);
    SessionDescriptor startConfiguredSession();
    void setTerminalOutputEnabled(bool enabled);
    void setSpeedMultiplier(double multiplier);

    void setSeed(std::uint32_t seed);
    void loadMap(int mapId);
    void setAlgorithm(PathAlgo algorithm);
    void configureScenario(const ScenarioConfig& config);
    void setSuppressOutput(bool suppress);

    void prepareConsole();
    bool interactiveSetup();
    bool runInteractiveConsole();
    void printPerformanceSummary() const;

    void step();
    BurstRunResult runBurst(int maxSteps, int maxDurationMs);
    void runUntilComplete();
    bool isComplete();
    MetricsSnapshot snapshotMetrics();
    StaticSceneSnapshot snapshotStaticScene();
    RenderFrameSnapshot snapshotRenderFrame(const RenderQueryOptions& options = {});
    RenderFrameDelta snapshotRenderDelta(std::uint64_t sinceFrameId, const RenderQueryOptions& options = {});
    std::vector<StructuredLogEntry> snapshotStructuredLogs(
        std::uint64_t sinceSeq = 0,
        std::size_t maxEntries = 256);
    RenderFrame snapshotFrame(bool paused = false);
    std::vector<std::string> snapshotRecentLogs();
    DebugSnapshot snapshotDebugState(bool paused = false);
    std::string buildDebugReport(bool paused = false);
    bool writeDebugReport(const std::string& path, bool paused = false);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace agv::core
