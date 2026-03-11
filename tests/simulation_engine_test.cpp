#include "agv/simulation_engine.hpp"

#include <array>

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

}  // namespace
