#include "agv/simulation_engine.hpp"

#include <filesystem>

#include <gtest/gtest.h>

namespace {

TEST(SimulationEngineTest, HeadlessCustomScenarioProducesMetricsAndFrame) {
    agv::core::SimulationEngine engine;
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};

    engine.setSeed(7);
    engine.loadMap(1);
    engine.setAlgorithm(agv::core::PathAlgo::Default);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
    engine.runUntilComplete();

    const auto metrics = engine.snapshotMetrics();
    EXPECT_GT(metrics.recordedSteps, 0);
    EXPECT_GE(metrics.tasksCompletedTotal, 1u);

    const auto frame = engine.snapshotFrame();
    EXPECT_FALSE(frame.text.empty());
}

TEST(SimulationEngineTest, SummaryJsonIsWritten) {
    agv::core::SimulationEngine engine;
    agv::core::ScenarioConfig scenario;
    scenario.mode = agv::core::SimulationMode::Custom;
    scenario.speedMultiplier = 0.0;
    scenario.phases = {{agv::core::PhaseType::Park, 1}};

    const auto output = std::filesystem::temp_directory_path() / "agv-summary-test.json";

    engine.setSeed(11);
    engine.loadMap(3);
    engine.setAlgorithm(agv::core::PathAlgo::AStarSimple);
    engine.configureScenario(scenario);
    engine.setSuppressOutput(true);
    engine.runUntilComplete();
    engine.writeRunSummary(output);

    EXPECT_TRUE(std::filesystem::exists(output));
    std::filesystem::remove(output);
}

}  // namespace
