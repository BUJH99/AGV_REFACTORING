#include "agv/simulation_engine.hpp"
#include "agv/internal/engine_internal.hpp"
#include "../src/planning/collision_planner_support.hpp"
#include "../src/planning/collision_planner_whca_support.hpp"

#include <array>
#include <initializer_list>

#include <gtest/gtest.h>

namespace {

agv::core::ScenarioConfig make_single_phase_custom_scenario() {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};
    return scenario;
}

agv::core::ScenarioConfig make_realtime_scenario() {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Realtime;
    scenario.speedMultiplier = 0.0;
    scenario.realtimeParkChance = 30;
    scenario.realtimeExitChance = 0;
    return scenario;
}

void configure_engine(
    agv::core::SimulationEngine& engine,
    agv::core::PathAlgo algorithm,
    const agv::core::ScenarioConfig& scenario,
    int map_id = 1) {
    engine.setSeed(7);
    engine.loadMap(map_id);
    engine.setAlgorithm(algorithm);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
}

agv::core::MetricsSnapshot run_headless_custom_once(
    agv::core::SimulationEngine& engine,
    agv::core::PathAlgo algorithm,
    int map_id = 1) {
    const auto scenario = make_single_phase_custom_scenario();

    configure_engine(engine, algorithm, scenario, map_id);
    engine.runUntilComplete();
    return engine.snapshotMetrics();
}

agv::core::MetricsSnapshot run_bounded_steps(
    agv::core::SimulationEngine& engine,
    int max_steps) {
    agv::core::MetricsSnapshot metrics = engine.snapshotMetrics();
    for (int step = 0; step < max_steps && !engine.isComplete(); ++step) {
        engine.step();
        if ((step % 64) == 63 || engine.isComplete()) {
            metrics = engine.snapshotMetrics();
            if (metrics.deadlockCount > 0u) {
                break;
            }
        }
    }

    return engine.snapshotMetrics();
}

TEST(SimulationEngineTest, HeadlessCustomScenarioProducesMetricsAndFrame) {
    agv::core::SimulationEngine engine;
    const auto metrics = run_headless_custom_once(engine, agv::core::PathAlgo::Default);
    EXPECT_GT(metrics.recordedSteps, 0);
    EXPECT_GE(metrics.tasksCompletedTotal, 1u);
    EXPECT_GE(metrics.maxStepCpuTimeMs, metrics.avgCpuTimeMs);
    EXPECT_GE(metrics.maxPlanningTimeMs, metrics.avgPlanningTimeMs);
    EXPECT_GE(metrics.stepsWithMovement, 1u);
    EXPECT_LE(metrics.stepsWithMovement, static_cast<std::uint64_t>(metrics.recordedSteps));
    EXPECT_EQ(metrics.outstandingTaskCount, 0);
    EXPECT_GT(metrics.avgMovementPerTask, 0.0);
    EXPECT_GT(metrics.avgTaskSteps, 0.0);
    EXPECT_GT(metrics.tasksPerAgent, 0.0);
    EXPECT_GT(metrics.tasksPerCpuSecond, 0.0);
    EXPECT_GT(metrics.tasksPerPlanningSecond, 0.0);
    EXPECT_GE(metrics.avgCpuTimePerTaskMs, 0.0);
    EXPECT_GE(metrics.avgPlanningTimePerTaskMs, 0.0);
    EXPECT_GE(metrics.nonTaskMovementCost, 0.0);
    EXPECT_GE(metrics.taskMovementCoverageRatio, 0.0);
    EXPECT_LE(metrics.taskMovementCoverageRatio, 1.0);
    EXPECT_GE(metrics.nonTaskMovementRatio, 0.0);
    EXPECT_LE(metrics.nonTaskMovementRatio, 1.0);
    EXPECT_GT(metrics.avgTaskStepsPerCell, 0.0);
    EXPECT_GE(metrics.avgTaskTurnsPer100Cells, 0.0);
    EXPECT_GE(metrics.nodesExpandedPerPlanningMs, 0.0);
    EXPECT_GE(metrics.heapMovesPerNodeExpanded, 0.0);
    EXPECT_GE(metrics.avgOutstandingTasksPerAgent, 0.0);
    EXPECT_GE(metrics.peakOutstandingTasksPerAgent, 0.0);
    EXPECT_GE(metrics.plannerWaitEdgesPerStep, 0.0);
    EXPECT_GE(metrics.plannerWaitEdgesPerConflictStep, 0.0);
    EXPECT_GE(metrics.plannerCycleStepRatio, 0.0);
    EXPECT_LE(metrics.plannerCycleStepRatio, 1.0);
    EXPECT_GE(metrics.plannerCbsAttemptRate, 0.0);
    EXPECT_GE(metrics.plannerCbsFailureRate, 0.0);
    EXPECT_LE(metrics.plannerCbsFailureRate, 1.0);
    EXPECT_EQ(metrics.agentFairnessBreakdown.size(), static_cast<std::size_t>(metrics.activeAgents));
    EXPECT_DOUBLE_EQ(metrics.tasksPerAgentSpread.avg, metrics.tasksPerAgent);
    EXPECT_DOUBLE_EQ(metrics.distancePerAgentSpread.avg, metrics.totalMovementCost / (double)metrics.activeAgents);
    double tasks_sum = 0.0;
    double distance_sum = 0.0;
    double idle_sum = 0.0;
    for (const auto& agent : metrics.agentFairnessBreakdown) {
        tasks_sum += static_cast<double>(agent.tasksCompleted);
        distance_sum += agent.distanceCells;
        idle_sum += static_cast<double>(agent.idleSteps);
    }
    EXPECT_DOUBLE_EQ(tasks_sum, static_cast<double>(metrics.tasksCompletedTotal));
    EXPECT_DOUBLE_EQ(distance_sum, metrics.totalMovementCost);
    EXPECT_DOUBLE_EQ(idle_sum / (double)metrics.activeAgents, metrics.idleStepsPerAgentSpread.avg);
    EXPECT_GE(metrics.tasksPerAgentSpread.min, 0.0);
    EXPECT_GE(metrics.distancePerAgentSpread.min, 0.0);
    EXPECT_GE(metrics.idleStepsPerAgentSpread.min, 0.0);
    EXPECT_GE(metrics.tasksPerAgentSpread.max, metrics.tasksPerAgentSpread.min);
    EXPECT_GE(metrics.distancePerAgentSpread.max, metrics.distancePerAgentSpread.min);
    EXPECT_GE(metrics.idleStepsPerAgentSpread.max, metrics.idleStepsPerAgentSpread.min);
    EXPECT_GE(metrics.tasksPerAgentSpread.stddev, 0.0);
    EXPECT_GE(metrics.distancePerAgentSpread.stddev, 0.0);
    EXPECT_GE(metrics.idleStepsPerAgentSpread.stddev, 0.0);
    EXPECT_GE(metrics.tasksPerAgentSpread.coefficientOfVariation, 0.0);
    EXPECT_GE(metrics.distancePerAgentSpread.coefficientOfVariation, 0.0);
    EXPECT_GE(metrics.idleStepsPerAgentSpread.coefficientOfVariation, 0.0);
    EXPECT_GE(metrics.tasksPerAgentSpread.minMaxRatio, 0.0);
    EXPECT_GE(metrics.distancePerAgentSpread.minMaxRatio, 0.0);
    EXPECT_GE(metrics.idleStepsPerAgentSpread.minMaxRatio, 0.0);
    EXPECT_LE(metrics.tasksPerAgentSpread.minMaxRatio, 1.0);
    EXPECT_LE(metrics.distancePerAgentSpread.minMaxRatio, 1.0);
    EXPECT_LE(metrics.idleStepsPerAgentSpread.minMaxRatio, 1.0);
    EXPECT_GE(metrics.planningCpuShare, 0.0);

    const auto frame = engine.snapshotFrame();
    EXPECT_FALSE(frame.text.empty());
}

TEST(SimulationEngineTest, DifferentAlgorithmsRemainExecutableThroughPublicApi) {
    const std::array<agv::core::PathAlgo, 3> algorithms = {
        agv::core::PathAlgo::Default,
        agv::core::PathAlgo::AStarSimple,
        agv::core::PathAlgo::DStarBasic,
    };

    for (const auto algorithm : algorithms) {
        agv::core::SimulationEngine engine;
        const auto metrics = run_headless_custom_once(engine, algorithm);
        EXPECT_GT(metrics.recordedSteps, 0) << static_cast<int>(algorithm);
        EXPECT_GE(metrics.tasksCompletedTotal, 1u) << static_cast<int>(algorithm);

        const auto frame = engine.snapshotFrame();
        EXPECT_FALSE(frame.text.empty()) << static_cast<int>(algorithm);
    }
}

TEST(SimulationEngineTest, PublicMap6CornerCaseGauntletSupportsAllAlgorithmsOnShortScenario) {
    const std::array<agv::core::PathAlgo, 3> algorithms = {
        agv::core::PathAlgo::Default,
        agv::core::PathAlgo::AStarSimple,
        agv::core::PathAlgo::DStarBasic,
    };

    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {
        {agv::core::PhaseType::Park, 6},
        {agv::core::PhaseType::Exit, 6},
    };

    for (const auto algorithm : algorithms) {
        agv::core::SimulationEngine engine;
        configure_engine(engine, algorithm, scenario, 6);
        engine.runUntilComplete();

        const auto metrics = engine.snapshotMetrics();
        EXPECT_TRUE(engine.isComplete()) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.mapId, 6) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.tasksCompletedTotal, 12u) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.deadlockCount, 0u) << static_cast<int>(algorithm);
    }
}

TEST(SimulationEngineTest, PublicMap7ReferenceSplitRoomSupportsAllAlgorithmsOnShortScenario) {
    const std::array<agv::core::PathAlgo, 3> algorithms = {
        agv::core::PathAlgo::Default,
        agv::core::PathAlgo::AStarSimple,
        agv::core::PathAlgo::DStarBasic,
    };

    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {
        {agv::core::PhaseType::Park, 12},
        {agv::core::PhaseType::Exit, 12},
    };

    for (const auto algorithm : algorithms) {
        agv::core::SimulationEngine engine;
        configure_engine(engine, algorithm, scenario, 7);
        engine.runUntilComplete();

        const auto metrics = engine.snapshotMetrics();
        EXPECT_TRUE(engine.isComplete()) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.mapId, 7) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.tasksCompletedTotal, 24u) << static_cast<int>(algorithm);
        EXPECT_EQ(metrics.deadlockCount, 0u) << static_cast<int>(algorithm);
    }
}

TEST(SimulationEngineTest, PublicMap7DefaultAlgorithmCompletesMaximumParkingThenExitWithoutDeadlock) {
    agv::core::SimulationEngine engine;

    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {
        {agv::core::PhaseType::Park, 132},
        {agv::core::PhaseType::Exit, 132},
    };

    engine.setSeed(1);
    engine.loadMap(7);
    engine.setAlgorithm(agv::core::PathAlgo::Default);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);

    constexpr int kMaxSteps = 12000;
    const auto metrics = run_bounded_steps(engine, kMaxSteps);
    const std::string debug_report = engine.buildDebugReport(true);

    EXPECT_TRUE(engine.isComplete()) << debug_report;
    EXPECT_EQ(metrics.mapId, 7);
    EXPECT_EQ(metrics.algorithm, agv::core::PathAlgo::Default);
    EXPECT_EQ(metrics.tasksCompletedTotal, 264u) << debug_report;
    EXPECT_EQ(metrics.deadlockCount, 0u) << debug_report;
}

TEST(SimulationEngineTest, StepAdvancesSimulationIncrementally) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_single_phase_custom_scenario());

    engine.step();
    const auto first_metrics = engine.snapshotMetrics();
    const auto first_frame = engine.snapshotFrame();

    EXPECT_GE(first_metrics.recordedSteps, 1);
    EXPECT_FALSE(first_frame.text.empty());
    EXPECT_FALSE(engine.isComplete());

    engine.step();
    const auto second_metrics = engine.snapshotMetrics();
    EXPECT_GT(second_metrics.recordedSteps, first_metrics.recordedSteps);
}

TEST(SimulationEngineTest, RealtimeScenarioCanBeSteppedThroughPublicApi) {
    agv::core::SimulationEngine engine;
    configure_engine(engine, agv::core::PathAlgo::Default, make_realtime_scenario());

    for (int i = 0; i < 5; ++i) {
        engine.step();
    }

    const auto metrics = engine.snapshotMetrics();
    EXPECT_EQ(metrics.mode, agv::core::SimulationMode::Realtime);
    EXPECT_GE(metrics.recordedSteps, 5);
    EXPECT_FALSE(engine.snapshotFrame().text.empty());
}

TEST(SimulationEngineTest, RealtimeScenarioRemainsDeterministicForSameSeed) {
    agv::core::SimulationEngine first;
    configure_engine(first, agv::core::PathAlgo::Default, make_realtime_scenario());
    for (int i = 0; i < 5; ++i) {
        first.step();
    }
    const auto first_metrics = first.snapshotMetrics();

    agv::core::SimulationEngine second;
    configure_engine(second, agv::core::PathAlgo::Default, make_realtime_scenario());
    for (int i = 0; i < 5; ++i) {
        second.step();
    }
    const auto second_metrics = second.snapshotMetrics();

    EXPECT_EQ(first_metrics.seed, second_metrics.seed);
    EXPECT_EQ(first_metrics.recordedSteps, second_metrics.recordedSteps);
    EXPECT_EQ(first_metrics.requestsCreatedTotal, second_metrics.requestsCreatedTotal);
    EXPECT_EQ(first_metrics.tasksCompletedTotal, second_metrics.tasksCompletedTotal);
    EXPECT_EQ(first_metrics.deadlockCount, second_metrics.deadlockCount);
    EXPECT_DOUBLE_EQ(first_metrics.totalMovementCost, second_metrics.totalMovementCost);
}

::SimulationConfig make_internal_custom_config(std::initializer_list<ConfigPhase> phases) {
    ::SimulationConfig config = default_simulation_config();
    config.seed = 7;
    config.map_id = 1;
    config.path_algo = PathAlgo::Default;
    config.mode = SimulationMode::Custom;
    config.suppress_stdout = true;
    config.num_phases = 0;
    for (const ConfigPhase& phase : phases) {
        if (config.num_phases >= MAX_PHASES) {
            break;
        }
        config.phases[config.num_phases++] = phase;
    }
    return config;
}

Agent* find_first_agent_with_home(Simulation& sim) {
    for (int i = 0; i < MAX_AGENTS; ++i) {
        Agent* agent = &sim.agent_manager->agents[i];
        if (agent->pos && agent->home_base) {
            return agent;
        }
    }
    return nullptr;
}

bool has_agent_in_state(const Simulation& sim, AgentState state) {
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if (sim.agent_manager->agents[i].state == state) {
            return true;
        }
    }
    return false;
}

int count_agents_in_state(const Simulation& sim, AgentState state) {
    int count = 0;
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if (sim.agent_manager->agents[i].state == state) {
            count++;
        }
    }
    return count;
}

int count_agents_with_position(const Simulation& sim) {
    int count = 0;
    for (int i = 0; i < MAX_AGENTS; ++i) {
        if (sim.agent_manager->agents[i].pos != nullptr) {
            count++;
        }
    }
    return count;
}

int count_walkable_neighbors(const GridMap& map, int x, int y) {
    int count = 0;
    constexpr std::array<int, 4> kStepX = {1, -1, 0, 0};
    constexpr std::array<int, 4> kStepY = {0, 0, 1, -1};
    for (int index = 0; index < 4; ++index) {
        const int next_x = x + kStepX[index];
        const int next_y = y + kStepY[index];
        if (!grid_is_valid_coord(next_x, next_y)) {
            continue;
        }
        if (!map.grid[next_y][next_x].is_obstacle) {
            count++;
        }
    }
    return count;
}

bool is_charge_station_goal(const Simulation& sim, const Node* node) {
    if (!node) {
        return false;
    }
    for (int i = 0; i < sim.map->num_charge_stations; ++i) {
        if (sim.map->charge_stations[i] == node) {
            return true;
        }
    }
    return false;
}

TEST(SimulationEngineTest, InternalMap6BuildsCornerCaseGauntletLayout) {
    Simulation sim;
    SimulationConfig config = make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}});
    config.map_id = 6;
    ASSERT_TRUE(apply_simulation_config(&sim, config));

    EXPECT_EQ(count_agents_with_position(sim), 16);
    EXPECT_GE(sim.map->num_goals, 50);
    EXPECT_GE(sim.map->num_charge_stations, 3);

    const Node& upper_dead_end_goal = sim.map->grid[6][28];
    EXPECT_TRUE(upper_dead_end_goal.is_goal);
    EXPECT_EQ(count_walkable_neighbors(*sim.map, upper_dead_end_goal.x, upper_dead_end_goal.y), 1);

    const Node& lower_dead_end_goal = sim.map->grid[36][70];
    EXPECT_TRUE(lower_dead_end_goal.is_goal);
    EXPECT_EQ(count_walkable_neighbors(*sim.map, lower_dead_end_goal.x, lower_dead_end_goal.y), 1);

    const Node& right_side_goal = sim.map->grid[31][77];
    EXPECT_TRUE(right_side_goal.is_goal);
    EXPECT_EQ(count_walkable_neighbors(*sim.map, right_side_goal.x, right_side_goal.y), 1);

    EXPECT_FALSE(sim.map->grid[21][54].is_obstacle);
    EXPECT_FALSE(sim.map->grid[21][53].is_obstacle);
    EXPECT_FALSE(sim.map->grid[21][55].is_obstacle);

    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[21][32]));
    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[13][64]));
    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[29][64]));
}

TEST(SimulationEngineTest, InternalMap7BuildsReferenceSplitRoomLayout) {
    Simulation sim;
    SimulationConfig config = make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}});
    config.map_id = 7;
    ASSERT_TRUE(apply_simulation_config(&sim, config));

    EXPECT_EQ(count_agents_with_position(sim), 16);
    EXPECT_GE(sim.map->num_goals, 120);
    EXPECT_EQ(sim.map->num_charge_stations, 4);

    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[12][6]));
    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[12][8]));
    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[26][6]));
    EXPECT_TRUE(is_charge_station_goal(sim, &sim.map->grid[26][8]));

    EXPECT_FALSE(sim.map->grid[19][25].is_obstacle);
    EXPECT_FALSE(sim.map->grid[19][34].is_obstacle);
    EXPECT_FALSE(sim.map->grid[11][36].is_obstacle);
    EXPECT_FALSE(sim.map->grid[26][36].is_obstacle);
    EXPECT_TRUE(sim.map->grid[10][40].is_obstacle);
    EXPECT_FALSE(sim.map->grid[11][40].is_obstacle);
    EXPECT_TRUE(sim.map->grid[12][40].is_obstacle);
    EXPECT_TRUE(sim.map->grid[25][40].is_obstacle);
    EXPECT_FALSE(sim.map->grid[26][40].is_obstacle);
    EXPECT_TRUE(sim.map->grid[27][40].is_obstacle);

    EXPECT_TRUE(sim.map->grid[17][25].is_obstacle);
    EXPECT_TRUE(sim.map->grid[21][25].is_obstacle);

    EXPECT_TRUE(sim.map->grid[5][45].is_goal);
    EXPECT_TRUE(sim.map->grid[3][45].is_goal);
    EXPECT_TRUE(sim.map->grid[13][65].is_goal);
    EXPECT_TRUE(sim.map->grid[15][65].is_goal);
    EXPECT_TRUE(sim.map->grid[24][46].is_goal);
    EXPECT_TRUE(sim.map->grid[28][46].is_goal);
    EXPECT_TRUE(sim.map->grid[36][66].is_goal);
}

bool plans_have_spacetime_conflict(const TimedNodePlan& lhs, const TimedNodePlan& rhs, int horizon) {
    for (int t = 1; t <= horizon; ++t) {
        if (!lhs[t] || !rhs[t] || !lhs[t - 1] || !rhs[t - 1]) {
            return true;
        }
        if (lhs[t] == rhs[t]) {
            return true;
        }
        if (lhs[t] == rhs[t - 1] && rhs[t] == lhs[t - 1]) {
            return true;
        }
    }
    return false;
}

void initialize_empty_map(GridMap* map) {
    ASSERT_NE(map, nullptr);
    map->num_goals = 0;
    map->num_charge_stations = 0;
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            Node& node = map->grid[y][x];
            node.x = x;
            node.y = y;
            node.is_obstacle = false;
            node.is_goal = false;
            node.is_temp = false;
            node.is_parked = false;
            node.reserved_by_agent = -1;
        }
    }
}

Node* find_adjacent_walkable_node(GridMap* map, Node* origin) {
    if (!map || !origin) {
        return nullptr;
    }

    constexpr std::array<int, 4> kStepX = {1, -1, 0, 0};
    constexpr std::array<int, 4> kStepY = {0, 0, 1, -1};
    for (int index = 0; index < 4; ++index) {
        const int next_x = origin->x + kStepX[index];
        const int next_y = origin->y + kStepY[index];
        if (!grid_is_valid_coord(next_x, next_y)) {
            continue;
        }
        Node* next = &map->grid[next_y][next_x];
        if (!next->is_obstacle && !next->is_parked) {
            return next;
        }
    }

    return nullptr;
}

Agent* initialize_pathfinder_agent(AgentManager* manager, Node* start) {
    if (!manager || !start) {
        return nullptr;
    }
    Agent* agent = &manager->agents[0];
    agent->id = 0;
    agent->symbol = 'A';
    agent->pos = start;
    agent->state = AgentState::GoingToPark;
    agent->goal = nullptr;
    agent->heading = AgentDir::None;
    agent->rotation_wait = 0;
    return agent;
}

TEST(SimulationEngineTest, SequentialEnginesStayIndependentAcrossAlgorithmsAndModes) {
    agv::core::SimulationEngine first;
    const auto first_metrics = run_headless_custom_once(first, agv::core::PathAlgo::Default);
    EXPECT_GE(first_metrics.tasksCompletedTotal, 1u);

    agv::core::SimulationEngine second;
    configure_engine(second, agv::core::PathAlgo::DStarBasic, make_realtime_scenario());
    for (int i = 0; i < 3; ++i) {
        second.step();
    }
    const auto second_metrics = second.snapshotMetrics();
    EXPECT_EQ(second_metrics.mode, agv::core::SimulationMode::Realtime);
    EXPECT_EQ(second_metrics.recordedSteps, 3);

    agv::core::SimulationEngine third;
    configure_engine(third, agv::core::PathAlgo::AStarSimple, make_single_phase_custom_scenario());
    third.step();
    third.runUntilComplete();
    const auto third_metrics = third.snapshotMetrics();
    EXPECT_GE(third_metrics.tasksCompletedTotal, 1u);
    EXPECT_FALSE(third.snapshotFrame().text.empty());
}

TEST(SimulationEngineTest, InternalPathfinderReinitializeAndUpdateStartRemainStable) {
    GridMap map;
    initialize_empty_map(&map);

    AgentManager manager;
    Agent* agent = initialize_pathfinder_agent(&manager, &map.grid[1][1]);
    ASSERT_NE(agent, nullptr);
    Node* first_goal = &map.grid[1][4];
    Node* second_goal = &map.grid[4][4];
    first_goal->is_goal = true;
    second_goal->is_goal = true;

    Pathfinder pathfinder(agent->pos, first_goal, agent);
    pathfinder.computeShortestPath(&map, &manager);
    const PathfinderRunMetrics initial_metrics = pathfinder.lastRunMetrics();
    EXPECT_EQ(pathfinder.goalNode(), first_goal);
    EXPECT_LT(pathfinder.gCost(agent->pos), INF * 0.5);
    EXPECT_GT(initial_metrics.nodes_expanded, 0u);

    Node* first_step = pathfinder.getNextStep(&map, &manager, agent->pos);
    ASSERT_NE(first_step, nullptr);
    ASSERT_NE(first_step, agent->pos);

    pathfinder.updateStart(first_step);
    pathfinder.computeShortestPath(&map, &manager);
    Node* second_step = pathfinder.getNextStep(&map, &manager, first_step);
    EXPECT_NE(second_step, nullptr);
    EXPECT_LT(pathfinder.gCost(first_step), INF * 0.5);

    agent->pos = first_step;
    pathfinder.reinitializeForGoal(second_goal);
    pathfinder.computeShortestPath(&map, &manager);
    EXPECT_EQ(pathfinder.goalNode(), second_goal);
    EXPECT_LT(pathfinder.gCost(first_step), INF * 0.5);
}

TEST(SimulationEngineTest, InternalGoalAssignmentKeepsExactBestCandidateSelection) {
    GridMap map;
    initialize_empty_map(&map);

    AgentManager manager;
    Logger logger;
    Agent* agent = initialize_pathfinder_agent(&manager, &map.grid[1][1]);
    ASSERT_NE(agent, nullptr);

    agent->goal = nullptr;
    agent->home_base = agent->pos;
    agent->state = AgentState::GoingToPark;

    Node* direct_goal = &map.grid[5][1];
    direct_goal->is_goal = true;
    Node* detour_goal = &map.grid[1][5];
    detour_goal->is_goal = true;

    map.goals[0] = direct_goal;
    map.goals[1] = detour_goal;
    map.num_goals = 2;

    map.grid[1][2].is_obstacle = true;
    map.grid[1][3].is_obstacle = true;
    map.grid[1][4].is_obstacle = true;

    agv_set_goal_if_needed(agent, &map, &manager, &logger);
    EXPECT_EQ(agent->goal, direct_goal);
    ASSERT_NE(agent->goal, nullptr);
    EXPECT_EQ(agent->goal->reserved_by_agent, agent->id);
}

TEST(SimulationEngineTest, InternalStepScratchTouchedCellOwnerResetRemainsStable) {
    StepScratch scratch;
    EXPECT_EQ(scratch.cell_owner[11], -1);
    EXPECT_EQ(scratch.cell_owner[29], -1);

    scratch.setCellOwner(11, 2);
    scratch.setCellOwner(29, 5);
    EXPECT_EQ(scratch.cell_owner[11], 2);
    EXPECT_EQ(scratch.cell_owner[29], 5);
    EXPECT_EQ(scratch.touched_cell_count, 2);

    scratch.clearTouchedCellOwner();
    EXPECT_EQ(scratch.cell_owner[11], -1);
    EXPECT_EQ(scratch.cell_owner[29], -1);
    EXPECT_EQ(scratch.touched_cell_count, 0);
}

TEST(SimulationEngineTest, InternalReservationTableTouchedClearRemainsStable) {
    GridMap map;
    initialize_empty_map(&map);

    ReservationTable table;
    Node* stay = &map.grid[1][1];
    Node* future = &map.grid[2][2];

    table.setOccupant(0, stay, 3, MAX_WHCA_HORIZON);
    table.setOccupant(2, future, 5, MAX_WHCA_HORIZON);

    EXPECT_EQ(table.occupantAt(0, stay, MAX_WHCA_HORIZON), 3);
    EXPECT_EQ(table.occupantAt(2, future, MAX_WHCA_HORIZON), 5);

    table.clearAgent(5, MAX_WHCA_HORIZON);
    EXPECT_EQ(table.occupantAt(0, stay, MAX_WHCA_HORIZON), 3);
    EXPECT_EQ(table.occupantAt(2, future, MAX_WHCA_HORIZON), -1);

    table.clear();
    EXPECT_EQ(table.occupantAt(0, stay, MAX_WHCA_HORIZON), -1);
    EXPECT_EQ(table.occupantAt(2, future, MAX_WHCA_HORIZON), -1);
}

TEST(SimulationEngineTest, InternalPartialCbsResolvesCrossingConflictWithGroupLocalPlans) {
    GridMap map;
    initialize_empty_map(&map);

    AgentManager manager;
    Logger logger;
    RuntimeTuningState tuning;
    PlannerMetricsState metrics;
    DefaultPlannerScratch scratch;
    ReservationTable table;

    tuning.whca_horizon = 4;

    Agent* first = &manager.agents[0];
    first->id = 0;
    first->symbol = 'A';
    first->pos = &map.grid[1][1];
    first->goal = &map.grid[1][3];
    first->heading = AgentDir::None;

    Agent* second = &manager.agents[1];
    second->id = 1;
    second->symbol = 'B';
    second->pos = &map.grid[1][3];
    second->goal = &map.grid[1][1];
    second->heading = AgentDir::None;

    PlanningContext context;
    context.agents = &manager;
    context.map = &map;
    context.logger = &logger;
    context.runtime_tuning = &tuning;
    context.planner_metrics = &metrics;

    std::array<int, MAX_CBS_GROUP> group_ids{};
    group_ids.fill(-1);
    group_ids[0] = first->id;
    group_ids[1] = second->id;

    const CbsSolveResult result = run_partial_CBS(context, group_ids, 2, table, scratch);
    ASSERT_TRUE(result.solved);
    EXPECT_GT(result.expansions, 0);
    EXPECT_FALSE(plans_have_spacetime_conflict(result.plans[first->id], result.plans[second->id], tuning.whca_horizon));
}

TEST(SimulationEngineTest, InternalConflictResolutionPolicyKeepsSwapTargetsDistinct) {
    GridMap map;
    initialize_empty_map(&map);

    AgentManager manager;
    Agent* first = &manager.agents[0];
    first->id = 0;
    first->pos = &map.grid[1][1];
    first->goal = &map.grid[1][2];
    first->state = AgentState::GoingToCollect;

    Agent* second = &manager.agents[1];
    second->id = 1;
    second->pos = &map.grid[1][2];
    second->goal = &map.grid[1][1];
    second->state = AgentState::GoingToCollect;

    AgentOrder order{};
    order[0] = first->id;
    order[1] = second->id;
    for (int index = 2; index < MAX_AGENTS; ++index) {
        order[index] = index;
    }

    AgentNodeSlots next_positions{};
    next_positions[first->id] = second->pos;
    next_positions[second->id] = first->pos;

    ConflictResolutionPolicy policy;
    policy.resolve(&manager, order, next_positions);

    EXPECT_EQ(next_positions[first->id], first->pos);
    EXPECT_EQ(next_positions[second->id], second->pos);
}

TEST(SimulationEngineTest, InternalOscillationCountsAsStuckProgress) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);

    Node* origin = agent->pos;
    Node* neighbor = find_adjacent_walkable_node(sim.map, origin);
    ASSERT_NE(neighbor, nullptr);

    agent->state = AgentState::ReturningHomeEmpty;
    agent->goal = agent->home_base;

    AgentNodeSlots next_positions{};
    AgentNodeSlots previous_positions{};

    previous_positions[agent->id] = origin;
    next_positions[agent->id] = neighbor;
    ASSERT_TRUE(agv_apply_moves_and_update_stuck(&sim, next_positions, previous_positions));
    EXPECT_EQ(agent->last_pos, origin);
    EXPECT_EQ(agent->stuck_steps, 0);

    previous_positions[agent->id] = neighbor;
    next_positions[agent->id] = origin;
    ASSERT_TRUE(agv_apply_moves_and_update_stuck(&sim, next_positions, previous_positions));
    EXPECT_EQ(agent->oscillation_steps, 1);
    EXPECT_GT(agent->stuck_steps, 0);
}

TEST(SimulationEngineTest, InternalFallbackDecisionTracksYieldingAgents) {
    FallbackDecision decision;
    EXPECT_FALSE(decision.hasAction());

    decision.yield_agents.set(2);
    EXPECT_TRUE(decision.hasAction());
    EXPECT_TRUE(decision.yield_agents.contains(2));
}

TEST(SimulationEngineTest, InternalPlannerStepResetClearsLastConflictFlags) {
    PlannerMetricsState metrics;
    metrics.wf_edges_last = 5;
    metrics.scc_last = 2;
    metrics.cbs_ok_last = 1;
    metrics.cbs_exp_last = 9;
    metrics.whca_nodes_expanded_this_step = 7;

    metrics.resetStepCounters();

    EXPECT_EQ(metrics.wf_edges_last, 0);
    EXPECT_EQ(metrics.scc_last, 0);
    EXPECT_EQ(metrics.cbs_ok_last, 0);
    EXPECT_EQ(metrics.cbs_exp_last, 0);
    EXPECT_EQ(metrics.whca_nodes_expanded_this_step, 0u);
}

TEST(SimulationEngineTest, InternalCbsGroupLimitMatchesAgentCapacity) {
    EXPECT_EQ(MAX_CBS_GROUP, MAX_AGENTS);
}

TEST(SimulationEngineTest, InterleavedDefaultEnginesRemainIndependent) {
    agv::core::SimulationEngine first;
    agv::core::SimulationEngine second;
    const auto scenario = make_realtime_scenario();

    configure_engine(first, agv::core::PathAlgo::Default, scenario);
    configure_engine(second, agv::core::PathAlgo::Default, scenario);

    for (int step = 0; step < 4; ++step) {
        first.step();
        second.step();
    }

    const auto first_metrics = first.snapshotMetrics();
    const auto second_metrics = second.snapshotMetrics();
    EXPECT_EQ(first_metrics.recordedSteps, second_metrics.recordedSteps);
    EXPECT_EQ(first_metrics.requestsCreatedTotal, second_metrics.requestsCreatedTotal);
    EXPECT_EQ(first_metrics.tasksCompletedTotal, second_metrics.tasksCompletedTotal);
    EXPECT_EQ(first_metrics.deadlockCount, second_metrics.deadlockCount);
    EXPECT_DOUBLE_EQ(first_metrics.totalMovementCost, second_metrics.totalMovementCost);
}

TEST(SimulationEngineTest, InternalTaskDispatchAndPhaseAdvancementRemainStable) {
    Simulation park_sim;
    ASSERT_TRUE(apply_simulation_config(&park_sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    park_sim.workload_snapshot = agv_collect_agent_workload(park_sim.agent_manager);
    agv_update_task_dispatch(&park_sim);
    EXPECT_TRUE(has_agent_in_state(park_sim, AgentState::GoingToPark));

    Simulation exit_sim;
    ASSERT_TRUE(apply_simulation_config(&exit_sim, make_internal_custom_config({ConfigPhase{PhaseType::Exit, 1}})));
    ASSERT_GT(exit_sim.map->num_goals, 0);
    exit_sim.map->goals[0]->is_parked = true;
    exit_sim.agent_manager->total_cars_parked = 1;
    exit_sim.workload_snapshot = agv_collect_agent_workload(exit_sim.agent_manager);
    agv_update_task_dispatch(&exit_sim);
    EXPECT_TRUE(has_agent_in_state(exit_sim, AgentState::GoingToCollect));

    Simulation phase_sim;
    ASSERT_TRUE(apply_simulation_config(
        &phase_sim,
        make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}, ConfigPhase{PhaseType::Exit, 1}})));
    ASSERT_GT(phase_sim.map->num_goals, 0);
    phase_sim.map->goals[0]->is_parked = true;
    phase_sim.agent_manager->total_cars_parked = 1;
    phase_sim.scenario_manager->tasks_completed_in_phase = 1;
    phase_sim.workload_snapshot = agv_collect_agent_workload(phase_sim.agent_manager);
    agv_update_task_dispatch(&phase_sim);
    EXPECT_EQ(phase_sim.scenario_manager->current_phase_index, 1);
    EXPECT_EQ(phase_sim.scenario_manager->phases[1].type, PhaseType::Exit);
    phase_sim.workload_snapshot = agv_collect_agent_workload(phase_sim.agent_manager);
    agv_update_task_dispatch(&phase_sim);
    EXPECT_TRUE(has_agent_in_state(phase_sim, AgentState::GoingToCollect));
}

TEST(SimulationEngineTest, InternalMap3ParkingDispatchUsesAllStagedAgents) {
    Simulation sim;
    SimulationConfig config = make_internal_custom_config({ConfigPhase{PhaseType::Park, 900}});
    config.map_id = 3;
    ASSERT_TRUE(apply_simulation_config(&sim, config));
    EXPECT_EQ(count_agents_with_position(sim), 16);

    sim.workload_snapshot = agv_collect_agent_workload(sim.agent_manager);
    agv_update_task_dispatch(&sim);

    EXPECT_EQ(count_agents_in_state(sim, AgentState::GoingToPark), 16);
}

TEST(SimulationEngineTest, PublicMap3DefaultAlgorithmCompletes900ParkingRequestsWithoutDeadlock) {
    agv::core::SimulationEngine engine;

    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 900}};

    engine.setSeed(1);
    engine.loadMap(3);
    engine.setAlgorithm(agv::core::PathAlgo::Default);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);

    engine.runUntilComplete();

    const auto metrics = engine.snapshotMetrics();
    EXPECT_TRUE(engine.isComplete());
    EXPECT_EQ(metrics.mapId, 3);
    EXPECT_EQ(metrics.algorithm, agv::core::PathAlgo::Default);
    EXPECT_EQ(metrics.tasksCompletedTotal, 900u);
    EXPECT_EQ(metrics.deadlockCount, 0u);
}

TEST(SimulationEngineTest, InternalDeadlockCounterIgnoresGoalActionTimerStalls) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    agent->state = AgentState::GoingToPark;
    agent->goal = sim.map->goals[0];
    agent->action_timer = 3;

    AgentNodeSlots next_positions{};
    next_positions[agent->id] = agent->pos;

    for (int step = 0; step < (DEADLOCK_THRESHOLD + 2); ++step) {
        agv_update_deadlock_counter(&sim, next_positions, false, true);
    }

    EXPECT_EQ(sim.deadlock_count, 0u);
}

TEST(SimulationEngineTest, InternalDeadlockCounterRequiresSustainedStallBeforeCounting) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    agent->state = AgentState::GoingToPark;
    agent->goal = sim.map->goals[0];

    AgentNodeSlots next_positions{};
    next_positions[agent->id] = agent->pos;

    for (int step = 0; step < (DEADLOCK_THRESHOLD - 1); ++step) {
        agv_update_deadlock_counter(&sim, next_positions, false, true);
    }
    EXPECT_EQ(sim.deadlock_count, 0u);

    agv_update_deadlock_counter(&sim, next_positions, false, true);
    EXPECT_EQ(sim.deadlock_count, 1u);

    agv_update_deadlock_counter(&sim, next_positions, false, true);
    EXPECT_EQ(sim.deadlock_count, 1u);

    agv_update_deadlock_counter(&sim, next_positions, true, true);
    for (int step = 0; step < DEADLOCK_THRESHOLD; ++step) {
        agv_update_deadlock_counter(&sim, next_positions, false, true);
    }
    EXPECT_EQ(sim.deadlock_count, 2u);
}

TEST(SimulationEngineTest, InternalDeadlockEventTracksOutstandingCustomPhaseWork) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 2}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    agent->state = AgentState::GoingToPark;
    agent->goal = sim.map->goals[0];

    AgentNodeSlots next_positions{};
    next_positions[agent->id] = agent->pos;

    for (int step = 0; step < DEADLOCK_THRESHOLD; ++step) {
        agv_update_deadlock_counter(&sim, next_positions, false, true);
    }

    ASSERT_TRUE(sim.last_deadlock_event.valid);
    EXPECT_EQ(sim.last_deadlock_event.pending_task_count, 2);
    EXPECT_EQ(sim.last_deadlock_event.phase_task_target, 2);
}

TEST(SimulationEngineTest, PublicMap3AStarAlgorithmCompletes900ParkingThen900ExitWithoutDeadlock) {
    agv::core::SimulationEngine engine;

    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {
        {agv::core::PhaseType::Park, 900},
        {agv::core::PhaseType::Exit, 900},
    };

    engine.setSeed(1);
    engine.loadMap(3);
    engine.setAlgorithm(agv::core::PathAlgo::AStarSimple);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);

    engine.runUntilComplete();

    const auto metrics = engine.snapshotMetrics();
    EXPECT_TRUE(engine.isComplete());
    EXPECT_EQ(metrics.mapId, 3);
    EXPECT_EQ(metrics.algorithm, agv::core::PathAlgo::AStarSimple);
    EXPECT_EQ(metrics.tasksCompletedTotal, 1800u);
    EXPECT_EQ(metrics.deadlockCount, 0u);
}

TEST(SimulationEngineTest, InternalGoalCompletionTransitionsRemainStable) {
    Simulation park_sim;
    ASSERT_TRUE(apply_simulation_config(&park_sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* park_agent = find_first_agent_with_home(park_sim);
    ASSERT_NE(park_agent, nullptr);
    ASSERT_GT(park_sim.map->num_goals, 0);

    Node* park_goal = park_sim.map->goals[0];
    park_agent->goal = park_goal;
    park_goal->reserved_by_agent = park_agent->id;
    park_agent->pos = park_goal;
    park_agent->state = AgentState::GoingToPark;
    park_agent->action_timer = 1;

    agent_manager_update_state_after_move(
        park_sim.agent_manager,
        park_sim.scenario_manager,
        park_sim.map,
        park_sim.logger,
        &park_sim);
    EXPECT_TRUE(park_goal->is_parked);
    EXPECT_EQ(park_sim.agent_manager->total_cars_parked, 1);
    EXPECT_EQ(park_agent->state, AgentState::ReturningHomeEmpty);
    EXPECT_EQ(park_sim.tasks_completed_total, 1u);

    park_agent->goal = park_agent->home_base;
    park_agent->pos = park_agent->home_base;
    agent_manager_update_state_after_move(
        park_sim.agent_manager,
        park_sim.scenario_manager,
        park_sim.map,
        park_sim.logger,
        &park_sim);
    EXPECT_EQ(park_agent->state, AgentState::Idle);

    Simulation exit_sim;
    ASSERT_TRUE(apply_simulation_config(&exit_sim, make_internal_custom_config({ConfigPhase{PhaseType::Exit, 1}})));
    Agent* exit_agent = find_first_agent_with_home(exit_sim);
    ASSERT_NE(exit_agent, nullptr);
    ASSERT_GT(exit_sim.map->num_goals, 0);

    Node* exit_goal = exit_sim.map->goals[0];
    exit_goal->is_parked = true;
    exit_sim.agent_manager->total_cars_parked = 1;
    exit_agent->goal = exit_goal;
    exit_goal->reserved_by_agent = exit_agent->id;
    exit_agent->pos = exit_goal;
    exit_agent->state = AgentState::GoingToCollect;
    exit_agent->action_timer = 1;

    agent_manager_update_state_after_move(
        exit_sim.agent_manager,
        exit_sim.scenario_manager,
        exit_sim.map,
        exit_sim.logger,
        &exit_sim);
    EXPECT_FALSE(exit_goal->is_parked);
    EXPECT_EQ(exit_sim.agent_manager->total_cars_parked, 0);
    EXPECT_EQ(exit_agent->state, AgentState::ReturningWithCar);

    exit_agent->goal = exit_agent->home_base;
    exit_agent->pos = exit_agent->home_base;
    agent_manager_update_state_after_move(
        exit_sim.agent_manager,
        exit_sim.scenario_manager,
        exit_sim.map,
        exit_sim.logger,
        &exit_sim);
    EXPECT_EQ(exit_agent->state, AgentState::Idle);
    EXPECT_EQ(exit_sim.tasks_completed_total, 1u);
}

TEST(SimulationEngineTest, InternalChargeLifecycleRemainsStable) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_charge_stations, 0);

    agent->state = AgentState::ReturningHomeEmpty;
    agent->goal = agent->home_base;
    agent->total_distance_traveled = 1'000.0;
    if (agent->goal) {
        agent->goal->reserved_by_agent = agent->id;
    }

    agv_set_goal_if_needed(agent, sim.map, sim.agent_manager, sim.logger);
    EXPECT_EQ(agent->state, AgentState::GoingToCharge);
    ASSERT_TRUE(is_charge_station_goal(sim, agent->goal));

    agent->pos = agent->goal;
    agent_manager_update_state_after_move(
        sim.agent_manager,
        sim.scenario_manager,
        sim.map,
        sim.logger,
        &sim);
    EXPECT_EQ(agent->state, AgentState::Charging);
    EXPECT_GT(agent->charge_timer, 0);

    agent->charge_timer = 1;
    agent_manager_update_charge_state(sim.agent_manager, sim.map, sim.logger);
    EXPECT_EQ(agent->state, AgentState::ReturningHomeMaintenance);
    EXPECT_EQ(agent->goal, nullptr);
    EXPECT_DOUBLE_EQ(agent->total_distance_traveled, 0.0);

    agent->goal = agent->home_base;
    agent->pos = agent->home_base;
    agent_manager_update_state_after_move(
        sim.agent_manager,
        sim.scenario_manager,
        sim.map,
        sim.logger,
        &sim);
    EXPECT_EQ(agent->state, AgentState::Idle);
}

TEST(SimulationEngineTest, InternalChargeStationReservationPersistsUntilAgentLeaves) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_charge_stations, 0);

    Node* charge_station = sim.map->charge_stations[0];
    ASSERT_NE(charge_station, nullptr);
    Node* departure = find_adjacent_walkable_node(sim.map, charge_station);
    ASSERT_NE(departure, nullptr);

    agent->state = AgentState::Charging;
    agent->pos = charge_station;
    agent->goal = nullptr;
    agent->charge_timer = 1;
    charge_station->reserved_by_agent = agent->id;

    agent_manager_update_charge_state(sim.agent_manager, sim.map, sim.logger);
    EXPECT_EQ(agent->state, AgentState::ReturningHomeMaintenance);
    EXPECT_EQ(charge_station->reserved_by_agent, agent->id);

    AgentNodeSlots next_positions{};
    AgentNodeSlots previous_positions{};
    previous_positions[agent->id] = agent->pos;
    next_positions[agent->id] = departure;

    EXPECT_TRUE(agv_apply_moves_and_update_stuck(&sim, next_positions, previous_positions));
    EXPECT_EQ(agent->pos, departure);
    EXPECT_EQ(charge_station->reserved_by_agent, -1);
}

TEST(SimulationEngineTest, InternalReturningHomeMaintenanceUsesHoldingGoalWithoutIdling) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    Node* holding_goal = sim.map->goals[0];
    ASSERT_NE(holding_goal, nullptr);
    ASSERT_NE(holding_goal, agent->home_base);

    agent->state = AgentState::ReturningHomeMaintenance;
    agent->goal = holding_goal;
    agent->pos = holding_goal;
    holding_goal->reserved_by_agent = agent->id;

    agent_manager_update_state_after_move(
        sim.agent_manager,
        sim.scenario_manager,
        sim.map,
        sim.logger,
        &sim);

    EXPECT_EQ(agent->state, AgentState::ReturningHomeMaintenance);
    EXPECT_EQ(agent->goal, nullptr);
    EXPECT_EQ(agent->stuck_steps, 0);
}

TEST(SimulationEngineTest, InternalReturningHomeEmptyRestoresHomeGoalInsteadOfKeepingTemporaryParkingGoal) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    Node* holding_goal = sim.map->goals[0];
    ASSERT_NE(holding_goal, nullptr);
    ASSERT_NE(holding_goal, agent->home_base);

    agent->state = AgentState::ReturningHomeEmpty;
    agent->goal = holding_goal;
    agent->stuck_steps = 12;
    holding_goal->reserved_by_agent = agent->id;

    agv_set_goal_if_needed(agent, sim.map, sim.agent_manager, sim.logger);

    EXPECT_EQ(holding_goal->reserved_by_agent, -1);
    ASSERT_EQ(agent->goal, agent->home_base);
    EXPECT_EQ(agent->goal->reserved_by_agent, agent->id);
}

TEST(SimulationEngineTest, InternalReturningHomeEmptyKeepsHomeGoalWhenStuck) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PhaseType::Park, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_goals, 0);

    Node* holding_goal = sim.map->goals[0];
    ASSERT_NE(holding_goal, nullptr);
    ASSERT_NE(holding_goal, agent->home_base);

    agent->state = AgentState::ReturningHomeEmpty;
    agent->goal = agent->home_base;
    agent->stuck_steps = 12;
    agent->home_base->reserved_by_agent = agent->id;

    agv_set_goal_if_needed(agent, sim.map, sim.agent_manager, sim.logger);

    EXPECT_EQ(agent->goal, agent->home_base);
    EXPECT_EQ(agent->goal->reserved_by_agent, agent->id);
    EXPECT_EQ(holding_goal->reserved_by_agent, -1);
}

}  // namespace
