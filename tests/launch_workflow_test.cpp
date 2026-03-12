#include "agv/internal/console_launch_wizard.hpp"
#include "agv/render_ipc_server.hpp"
#include "agv/simulation_engine.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace {

agv::core::LaunchConfig make_launch_config() {
    agv::core::LaunchConfig config;
    config.seed = 7;
    config.mapId = 1;
    config.algorithm = agv::core::PathAlgo::Default;
    config.scenario.mode = agv::core::SimulationMode::Custom;
    config.scenario.speedMultiplier = 0.0;
    config.scenario.phases = {{agv::core::PhaseType::Park, 1}};
    return config;
}

std::filesystem::path unique_temp_path(std::string_view stem) {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    return std::filesystem::temp_directory_path() /
        (std::string(stem) + "_" + std::to_string(now) + ".json");
}

TEST(LaunchWorkflowTest, ValidateLaunchConfigNormalizesAndRejectsInvalidFields) {
    agv::core::LaunchConfig config = make_launch_config();
    config.mapId = 8;
    config.scenario.mode = agv::core::SimulationMode::Custom;
    config.scenario.phases.clear();
    config.scenario.speedMultiplier = 10001.0;
    config.scenario.realtimeParkChance = 60;
    config.scenario.realtimeExitChance = 50;

    const agv::core::ValidationResult validation = agv::core::validateLaunchConfig(config);

    EXPECT_FALSE(validation.ok());
    EXPECT_FALSE(validation.warnings.empty());
    EXPECT_FALSE(validation.normalizedConfig.scenario.phases.empty());
    EXPECT_EQ(validation.normalizedConfig.scenario.phases.front().type, agv::core::PhaseType::Park);
    EXPECT_EQ(validation.normalizedConfig.scenario.phases.front().taskCount, 1);
}

TEST(LaunchWorkflowTest, StartConfiguredSessionAlwaysCreatesNewSession) {
    agv::core::SimulationEngine engine;
    engine.setTerminalOutputEnabled(false);
    engine.configureLaunch(make_launch_config());

    const auto first = engine.startConfiguredSession();
    const auto second = engine.startConfiguredSession();

    EXPECT_GT(second.sessionId, first.sessionId);
    EXPECT_EQ(first.sceneVersion, 1u);
    EXPECT_EQ(second.sceneVersion, 1u);
    EXPECT_EQ(first.frameId, 0u);
    EXPECT_EQ(second.frameId, 0u);
    EXPECT_EQ(second.launchConfig.mapId, 1);
}

TEST(LaunchWorkflowTest, RunBurstSkipsLegacySleepPath) {
    agv::core::SimulationEngine engine;
    auto launch = make_launch_config();
    launch.scenario.speedMultiplier = 0.1;

    engine.setTerminalOutputEnabled(false);
    engine.configureLaunch(launch);
    engine.startConfiguredSession();

    const auto start = std::chrono::steady_clock::now();
    const auto burst = engine.runBurst(1, 5000);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start);

    EXPECT_EQ(burst.executedSteps, 1);
    EXPECT_LT(elapsed.count(), 500);
}

TEST(LaunchWorkflowTest, ConsoleLaunchWizardSupportsGuidedBackAndLastUsedFallback) {
    const std::filesystem::path last_used_path = unique_temp_path("agv_last_launch");
    std::error_code error;
    std::filesystem::remove(last_used_path, error);

    {
        std::stringstream input;
        input
            << "3\n"   // Guided
            << "1\n"   // Map
            << "1\n"   // Algorithm
            << "1\n"   // Mode custom
            << "1\n"   // Phase count
            << "park\n"
            << "999999\n" // Invalid against map capacity
            << "1\n"   // Phase task count
            << "2.5\n" // Speed
            << "123\n" // Seed
            << "2\n"   // Summary -> Back
            << "456\n" // Seed again
            << "1\n";  // Summary -> Start
        std::stringstream output;
        agv::core::LaunchConfig chosen;
        ASSERT_TRUE(agv::internal::console::run_console_launch_wizard(input, output, last_used_path, chosen));
        EXPECT_EQ(chosen.seed, 456u);
        EXPECT_NE(output.str().find("Current draft:"), std::string::npos);
        EXPECT_NE(output.str().find("[1/7] Map Setup"), std::string::npos);
        EXPECT_NE(output.str().find("Map #1 capacity"), std::string::npos);
        EXPECT_NE(output.str().find("range 1~"), std::string::npos);
        EXPECT_TRUE(agv::internal::console::save_last_launch_config(last_used_path, chosen));
    }

    {
        std::stringstream input;
        input << "2\n"; // Last Used
        std::stringstream output;
        agv::core::LaunchConfig chosen;
        ASSERT_TRUE(agv::internal::console::run_console_launch_wizard(input, output, last_used_path, chosen));
        EXPECT_EQ(chosen.seed, 456u);
    }

    {
        std::ofstream corrupt(last_used_path, std::ios::out | std::ios::trunc);
        corrupt << "{broken";
        corrupt.close();

        std::stringstream input;
        input << "2\n"; // Last Used -> fallback to Recommended
        std::stringstream output;
        agv::core::LaunchConfig chosen;
        ASSERT_TRUE(agv::internal::console::run_console_launch_wizard(input, output, last_used_path, chosen));
        EXPECT_EQ(chosen.mapId, 1);
        EXPECT_EQ(chosen.algorithm, agv::core::PathAlgo::Default);
    }

    std::filesystem::remove(last_used_path, error);
}

TEST(LaunchWorkflowTest, ConsoleLaunchWizardCancelReturnsToStartMenuBeforeQuit) {
    const std::filesystem::path last_used_path = unique_temp_path("agv_last_launch_cancel");
    std::error_code error;
    std::filesystem::remove(last_used_path, error);

    std::stringstream input;
    input
        << "3\n"  // Guided
        << "q\n"  // Cancel guided setup
        << "4\n"; // Quit from start menu
    std::stringstream output;
    agv::core::LaunchConfig chosen;
    EXPECT_FALSE(agv::internal::console::run_console_launch_wizard(input, output, last_used_path, chosen));
    EXPECT_NE(output.str().find("Setup canceled. Returning to launch menu."), std::string::npos);
    EXPECT_NE(output.str().find("AGV Launch"), std::string::npos);

    std::filesystem::remove(last_used_path, error);
}

TEST(LaunchWorkflowTest, RenderIpcProtocolV1SupportsSessionFlowAndSessionErrors) {
    agv::ipc::RenderIpcServer server;

    const auto capabilities = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "cap"},
        {"command", "getCapabilities"},
    });
    ASSERT_EQ(capabilities.size(), 1u);
    EXPECT_TRUE(capabilities.front()["ok"].get<bool>());
    EXPECT_EQ(capabilities.front()["protocolVersion"], 1);
    EXPECT_EQ(capabilities.front()["requestId"], "cap");
    ASSERT_TRUE(capabilities.front().contains("capabilities"));
    const auto& capability_payload = capabilities.front()["capabilities"];
    EXPECT_EQ(capability_payload["platform"], "windows");
    EXPECT_TRUE(capability_payload["recommendedLaunchConfig"].is_object());
    EXPECT_EQ(capability_payload["recommendedPreset"]["seedStrategy"], "timestamp_seconds");
    EXPECT_TRUE(capability_payload["maps"].is_array());
    EXPECT_TRUE(capability_payload["algorithms"].is_array());
    EXPECT_TRUE(capability_payload["modes"].is_array());
    EXPECT_TRUE(capability_payload["wizardSteps"].is_array());
    EXPECT_EQ(capability_payload["wizardSteps"].front()["id"], "map");
    EXPECT_EQ(capability_payload["launchSchema"]["customPhases"]["taskCountCapacityField"], "maps[].capacity");

    const auto start = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "start-1"},
        {"command", "startSession"},
        {"launchConfig", {
            {"seed", 7},
            {"mapId", 1},
            {"algorithm", "default"},
            {"scenario", {
                {"mode", "custom"},
                {"speedMultiplier", 0.0},
                {"phases", nlohmann::json::array({{{"type", "park"}, {"taskCount", 1}}})},
            }},
        }},
    });
    ASSERT_EQ(start.size(), 1u);
    ASSERT_TRUE(start.front()["ok"].get<bool>());
    const auto first_session_id = start.front()["session"]["sessionId"].get<std::uint64_t>();
    EXPECT_GT(first_session_id, 0u);

    const auto scene = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "scene"},
        {"command", "getStaticScene"},
        {"sessionId", first_session_id},
    });
    EXPECT_EQ(scene.front()["scene"]["width"], 82);

    const auto frame = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "frame"},
        {"command", "getFrame"},
        {"sessionId", first_session_id},
    });
    const auto initial_frame_id = frame.front()["frame"]["frameId"].get<std::uint64_t>();

    const auto burst = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "burst"},
        {"command", "runBurst"},
        {"sessionId", first_session_id},
        {"maxSteps", 4},
        {"maxDurationMs", 10},
    });
    ASSERT_EQ(burst.size(), 1u);
    EXPECT_TRUE(burst.front()["ok"].get<bool>());
    EXPECT_GE(burst.front()["executedSteps"].get<int>(), 0);
    EXPECT_GE(burst.front()["frameId"].get<std::uint64_t>(), initial_frame_id);

    const auto delta = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "delta"},
        {"command", "getDelta"},
        {"sessionId", first_session_id},
        {"sinceFrameId", initial_frame_id},
    });
    EXPECT_FALSE(delta.front()["delta"]["requiresFullResync"].get<bool>());

    const auto set_paused = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "pause"},
        {"command", "setPaused"},
        {"sessionId", first_session_id},
        {"paused", true},
    });
    EXPECT_TRUE(set_paused.front()["paused"].get<bool>());

    const auto paused_burst = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "paused-burst"},
        {"command", "runBurst"},
        {"sessionId", first_session_id},
        {"maxSteps", 4},
        {"maxDurationMs", 10},
    });
    EXPECT_EQ(paused_burst.front()["executedSteps"].get<int>(), 0);
    EXPECT_TRUE(paused_burst.front()["paused"].get<bool>());

    const auto speed = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "speed"},
        {"command", "setSpeedMultiplier"},
        {"sessionId", first_session_id},
        {"speedMultiplier", 2.0},
    });
    EXPECT_DOUBLE_EQ(speed.front()["speedMultiplier"].get<double>(), 2.0);

    const auto resume = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "resume"},
        {"command", "setPaused"},
        {"sessionId", first_session_id},
        {"paused", false},
    });
    EXPECT_FALSE(resume.front()["paused"].get<bool>());

    const auto metrics = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "metrics"},
        {"command", "getMetrics"},
        {"sessionId", first_session_id},
    });
    EXPECT_TRUE(metrics.front().contains("metrics"));

    const auto debug = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "debug"},
        {"command", "getDebugSnapshot"},
        {"sessionId", first_session_id},
    });
    EXPECT_TRUE(debug.front().contains("snapshot"));

    const auto logs = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "logs"},
        {"command", "getLogs"},
        {"sessionId", first_session_id},
        {"sinceSeq", 0},
    });
    EXPECT_TRUE(logs.front()["logs"].is_array());

    agv::ipc::RenderIpcServer history_server;

    const auto history_start = history_server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "history-start"},
        {"command", "startSession"},
        {"launchConfig", {
            {"seed", 7},
            {"mapId", 1},
            {"algorithm", "default"},
            {"scenario", {
                {"mode", "realtime"},
                {"speedMultiplier", 0.0},
                {"realtimeParkChance", 0},
                {"realtimeExitChance", 0},
            }},
        }},
    });
    const auto history_session_id = history_start.front()["session"]["sessionId"].get<std::uint64_t>();
    const auto history_frame = history_server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "history-frame"},
        {"command", "getFrame"},
        {"sessionId", history_session_id},
    });
    const auto history_initial_frame_id = history_frame.front()["frame"]["frameId"].get<std::uint64_t>();

    for (int i = 0; i < 300; ++i) {
        history_server.processRequest(nlohmann::json{
            {"protocolVersion", 1},
            {"requestId", std::string("burst-") + std::to_string(i)},
            {"command", "runBurst"},
            {"sessionId", history_session_id},
            {"maxSteps", 1},
            {"maxDurationMs", 10},
        });
    }

    const auto stale_delta = history_server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "stale-delta"},
        {"command", "getDelta"},
        {"sessionId", history_session_id},
        {"sinceFrameId", history_initial_frame_id},
    });
    EXPECT_TRUE(stale_delta.front()["delta"]["requiresFullResync"].get<bool>());

    const auto second_start = history_server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "start-2"},
        {"command", "startSession"},
        {"launchConfig", {
            {"seed", 7},
            {"mapId", 1},
            {"algorithm", "default"},
            {"scenario", {
                {"mode", "custom"},
                {"speedMultiplier", 0.0},
                {"phases", nlohmann::json::array({{{"type", "park"}, {"taskCount", 1}}})},
            }},
        }},
    });
    const auto second_session_id = second_start.front()["session"]["sessionId"].get<std::uint64_t>();
    EXPECT_GT(second_session_id, first_session_id);

    std::stringstream input;
    input << nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "stale-session"},
        {"command", "getFrame"},
        {"sessionId", history_session_id},
    }.dump() << '\n';
    std::stringstream output;
    history_server.run(input, output);

    std::string line;
    ASSERT_TRUE(std::getline(output, line));
    const auto stale_response = nlohmann::json::parse(line);
    EXPECT_FALSE(stale_response["ok"].get<bool>());
    EXPECT_EQ(stale_response["errorCode"], "session_stale");

    const auto shutdown = server.processRequest(nlohmann::json{
        {"protocolVersion", 1},
        {"requestId", "shutdown"},
        {"command", "shutdown"},
    });
    EXPECT_TRUE(shutdown.front()["shuttingDown"].get<bool>());
}

}  // namespace
