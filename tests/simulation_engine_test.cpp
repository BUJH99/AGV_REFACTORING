#include "agv/simulation_engine.hpp"
#include "agv/internal/engine_internal.hpp"

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
    const agv::core::ScenarioConfig& scenario) {
    engine.setSeed(7);
    engine.loadMap(1);
    engine.setAlgorithm(algorithm);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
}

agv::core::MetricsSnapshot run_headless_custom_once(
    agv::core::SimulationEngine& engine,
    agv::core::PathAlgo algorithm) {
    const auto scenario = make_single_phase_custom_scenario();

    configure_engine(engine, algorithm, scenario);
    engine.runUntilComplete();
    return engine.snapshotMetrics();
}

TEST(SimulationEngineTest, HeadlessCustomScenarioProducesMetricsAndFrame) {
    agv::core::SimulationEngine engine;
    const auto metrics = run_headless_custom_once(engine, agv::core::PathAlgo::Default);
    EXPECT_GT(metrics.recordedSteps, 0);
    EXPECT_GE(metrics.tasksCompletedTotal, 1u);

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

::SimulationConfig make_internal_custom_config(std::initializer_list<ConfigPhase> phases) {
    ::SimulationConfig config = default_simulation_config();
    config.seed = 7;
    config.map_id = 1;
    config.path_algo = PATHALGO_DEFAULT;
    config.mode = MODE_CUSTOM;
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

TEST(SimulationEngineTest, InternalTaskDispatchAndPhaseAdvancementRemainStable) {
    Simulation park_sim;
    ASSERT_TRUE(apply_simulation_config(&park_sim, make_internal_custom_config({ConfigPhase{PARK_PHASE, 1}})));
    park_sim.workload_snapshot = agv_collect_agent_workload(park_sim.agent_manager);
    agv_update_task_dispatch(&park_sim);
    EXPECT_TRUE(has_agent_in_state(park_sim, GOING_TO_PARK));

    Simulation exit_sim;
    ASSERT_TRUE(apply_simulation_config(&exit_sim, make_internal_custom_config({ConfigPhase{EXIT_PHASE, 1}})));
    ASSERT_GT(exit_sim.map->num_goals, 0);
    exit_sim.map->goals[0]->is_parked = true;
    exit_sim.agent_manager->total_cars_parked = 1;
    exit_sim.workload_snapshot = agv_collect_agent_workload(exit_sim.agent_manager);
    agv_update_task_dispatch(&exit_sim);
    EXPECT_TRUE(has_agent_in_state(exit_sim, GOING_TO_COLLECT));

    Simulation phase_sim;
    ASSERT_TRUE(apply_simulation_config(
        &phase_sim,
        make_internal_custom_config({ConfigPhase{PARK_PHASE, 1}, ConfigPhase{EXIT_PHASE, 1}})));
    ASSERT_GT(phase_sim.map->num_goals, 0);
    phase_sim.map->goals[0]->is_parked = true;
    phase_sim.agent_manager->total_cars_parked = 1;
    phase_sim.scenario_manager->tasks_completed_in_phase = 1;
    phase_sim.workload_snapshot = agv_collect_agent_workload(phase_sim.agent_manager);
    agv_update_task_dispatch(&phase_sim);
    EXPECT_EQ(phase_sim.scenario_manager->current_phase_index, 1);
    EXPECT_EQ(phase_sim.scenario_manager->phases[1].type, EXIT_PHASE);
    phase_sim.workload_snapshot = agv_collect_agent_workload(phase_sim.agent_manager);
    agv_update_task_dispatch(&phase_sim);
    EXPECT_TRUE(has_agent_in_state(phase_sim, GOING_TO_COLLECT));
}

TEST(SimulationEngineTest, InternalGoalCompletionTransitionsRemainStable) {
    Simulation park_sim;
    ASSERT_TRUE(apply_simulation_config(&park_sim, make_internal_custom_config({ConfigPhase{PARK_PHASE, 1}})));
    Agent* park_agent = find_first_agent_with_home(park_sim);
    ASSERT_NE(park_agent, nullptr);
    ASSERT_GT(park_sim.map->num_goals, 0);

    Node* park_goal = park_sim.map->goals[0];
    park_agent->goal = park_goal;
    park_goal->reserved_by_agent = park_agent->id;
    park_agent->pos = park_goal;
    park_agent->state = GOING_TO_PARK;
    park_agent->action_timer = 1;

    agent_manager_update_state_after_move(
        park_sim.agent_manager,
        park_sim.scenario_manager,
        park_sim.map,
        park_sim.logger,
        &park_sim);
    EXPECT_TRUE(park_goal->is_parked);
    EXPECT_EQ(park_sim.agent_manager->total_cars_parked, 1);
    EXPECT_EQ(park_agent->state, RETURNING_HOME_EMPTY);
    EXPECT_EQ(park_sim.tasks_completed_total, 1u);

    park_agent->goal = park_agent->home_base;
    park_agent->pos = park_agent->home_base;
    agent_manager_update_state_after_move(
        park_sim.agent_manager,
        park_sim.scenario_manager,
        park_sim.map,
        park_sim.logger,
        &park_sim);
    EXPECT_EQ(park_agent->state, IDLE);

    Simulation exit_sim;
    ASSERT_TRUE(apply_simulation_config(&exit_sim, make_internal_custom_config({ConfigPhase{EXIT_PHASE, 1}})));
    Agent* exit_agent = find_first_agent_with_home(exit_sim);
    ASSERT_NE(exit_agent, nullptr);
    ASSERT_GT(exit_sim.map->num_goals, 0);

    Node* exit_goal = exit_sim.map->goals[0];
    exit_goal->is_parked = true;
    exit_sim.agent_manager->total_cars_parked = 1;
    exit_agent->goal = exit_goal;
    exit_goal->reserved_by_agent = exit_agent->id;
    exit_agent->pos = exit_goal;
    exit_agent->state = GOING_TO_COLLECT;
    exit_agent->action_timer = 1;

    agent_manager_update_state_after_move(
        exit_sim.agent_manager,
        exit_sim.scenario_manager,
        exit_sim.map,
        exit_sim.logger,
        &exit_sim);
    EXPECT_FALSE(exit_goal->is_parked);
    EXPECT_EQ(exit_sim.agent_manager->total_cars_parked, 0);
    EXPECT_EQ(exit_agent->state, RETURNING_WITH_CAR);

    exit_agent->goal = exit_agent->home_base;
    exit_agent->pos = exit_agent->home_base;
    agent_manager_update_state_after_move(
        exit_sim.agent_manager,
        exit_sim.scenario_manager,
        exit_sim.map,
        exit_sim.logger,
        &exit_sim);
    EXPECT_EQ(exit_agent->state, IDLE);
    EXPECT_EQ(exit_sim.tasks_completed_total, 1u);
}

TEST(SimulationEngineTest, InternalChargeLifecycleRemainsStable) {
    Simulation sim;
    ASSERT_TRUE(apply_simulation_config(&sim, make_internal_custom_config({ConfigPhase{PARK_PHASE, 1}})));
    Agent* agent = find_first_agent_with_home(sim);
    ASSERT_NE(agent, nullptr);
    ASSERT_GT(sim.map->num_charge_stations, 0);

    agent->state = RETURNING_HOME_EMPTY;
    agent->goal = agent->home_base;
    agent->total_distance_traveled = 1'000.0;
    if (agent->goal) {
        agent->goal->reserved_by_agent = agent->id;
    }

    agv_set_goal_if_needed(agent, sim.map, sim.agent_manager, sim.logger);
    EXPECT_EQ(agent->state, GOING_TO_CHARGE);
    ASSERT_TRUE(is_charge_station_goal(sim, agent->goal));

    agent->pos = agent->goal;
    agent_manager_update_state_after_move(
        sim.agent_manager,
        sim.scenario_manager,
        sim.map,
        sim.logger,
        &sim);
    EXPECT_EQ(agent->state, CHARGING);
    EXPECT_GT(agent->charge_timer, 0);

    agent->charge_timer = 1;
    agent_manager_update_charge_state(sim.agent_manager, sim.map, sim.logger);
    EXPECT_EQ(agent->state, RETURNING_HOME_MAINTENANCE);
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
    EXPECT_EQ(agent->state, IDLE);
}

}  // namespace
