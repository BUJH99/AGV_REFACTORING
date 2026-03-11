#include "agv/simulation_engine.hpp"

#include <gtest/gtest.h>

namespace {

agv::core::MetricsSnapshot run_headless_custom_once(
    agv::core::SimulationEngine& engine,
    agv::core::PathAlgo algorithm) {
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};

    engine.setSeed(7);
    engine.loadMap(1);
    engine.setAlgorithm(algorithm);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
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
    agv::core::SimulationEngine default_engine;
    const auto default_metrics = run_headless_custom_once(default_engine, agv::core::PathAlgo::Default);
    EXPECT_GT(default_metrics.recordedSteps, 0);
    EXPECT_GE(default_metrics.tasksCompletedTotal, 1u);

    agv::core::SimulationEngine dstar_engine;
    const auto dstar_metrics = run_headless_custom_once(dstar_engine, agv::core::PathAlgo::DStarBasic);
    EXPECT_GT(dstar_metrics.recordedSteps, 0);
    EXPECT_GE(dstar_metrics.tasksCompletedTotal, 1u);

    const auto dstar_frame = dstar_engine.snapshotFrame();
    EXPECT_FALSE(dstar_frame.text.empty());
}

}  // namespace
