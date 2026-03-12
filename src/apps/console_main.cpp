#include "agv/simulation_engine.hpp"
#include "agv/internal/engine_internal.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using agv::core::PathAlgo;
using agv::core::PhaseConfig;
using agv::core::PhaseType;
using agv::core::ScenarioConfig;
using agv::core::SimulationEngine;
using agv::core::SimulationMode;

struct CliOptions {
    bool interactive{true};
    agv::core::LaunchConfig launch{};
    agv::core::HeadlessRunOptions run{};
};

PathAlgo parseAlgorithm(const std::string& value);
SimulationMode parseMode(const std::string& value);
PhaseConfig parsePhase(const std::string& value);

void mark_headless_mode(CliOptions& options) {
    options.interactive = false;
}

std::string next_argument_value(int& index, int argc, char* argv[], std::string_view name) {
    if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + std::string(name));
    }
    return argv[++index];
}

bool parse_headless_option(CliOptions& options, std::string_view argument, int& index, int argc, char* argv[]) {
    if (argument == "--headless") {
        mark_headless_mode(options);
    } else if (argument == "--seed") {
        options.launch.seed = static_cast<std::uint32_t>(std::stoul(next_argument_value(index, argc, argv, "--seed")));
        mark_headless_mode(options);
    } else if (argument == "--map") {
        options.launch.mapId = std::stoi(next_argument_value(index, argc, argv, "--map"));
        mark_headless_mode(options);
    } else if (argument == "--algo") {
        options.launch.algorithm = parseAlgorithm(next_argument_value(index, argc, argv, "--algo"));
        mark_headless_mode(options);
    } else if (argument == "--mode") {
        options.launch.scenario.mode = parseMode(next_argument_value(index, argc, argv, "--mode"));
        mark_headless_mode(options);
    } else if (argument == "--speed") {
        options.launch.scenario.speedMultiplier = std::stod(next_argument_value(index, argc, argv, "--speed"));
        mark_headless_mode(options);
    } else if (argument == "--phase") {
        if (options.launch.scenario.phases.size() == 1 &&
            options.launch.scenario.phases.front().type == PhaseType::Park &&
            options.launch.scenario.phases.front().taskCount == 1) {
            options.launch.scenario.phases.clear();
        }
        options.launch.scenario.phases.push_back(parsePhase(next_argument_value(index, argc, argv, "--phase")));
        mark_headless_mode(options);
    } else if (argument == "--park-chance") {
        options.launch.scenario.realtimeParkChance = std::stoi(next_argument_value(index, argc, argv, "--park-chance"));
        mark_headless_mode(options);
    } else if (argument == "--exit-chance") {
        options.launch.scenario.realtimeExitChance = std::stoi(next_argument_value(index, argc, argv, "--exit-chance"));
        mark_headless_mode(options);
    } else if (argument == "--max-steps") {
        options.run.maxSteps = std::stoi(next_argument_value(index, argc, argv, "--max-steps"));
        mark_headless_mode(options);
    } else if (argument == "--debug-report") {
        options.run.debugReportPath = next_argument_value(index, argc, argv, "--debug-report");
        mark_headless_mode(options);
    } else if (argument == "--deadlock-report") {
        options.run.deadlockReportPath = next_argument_value(index, argc, argv, "--deadlock-report");
        mark_headless_mode(options);
    } else if (argument == "--stop-on-deadlock") {
        options.run.stopOnDeadlock = true;
        mark_headless_mode(options);
    } else if (argument == "--render") {
        options.run.renderOutputEnabled = true;
        mark_headless_mode(options);
    } else {
        return false;
    }

    return true;
}

void printUsage() {
    std::cout
        << "Usage:\n"
        << "  agv_console                     # interactive console mode\n"
        << "  agv_console --headless [options]\n\n"
        << "Headless options:\n"
        << "  --seed <n>\n"
        << "  --map <1-7>\n"
        << "  --algo <default|astar|dstar|1|2|3>\n"
        << "  --mode <custom|realtime>\n"
        << "  --speed <multiplier>\n"
        << "  --phase <park:N|exit:N>\n"
        << "  --park-chance <0-100>\n"
        << "  --exit-chance <0-100>\n"
        << "  --max-steps <n>\n"
        << "  --debug-report <path>\n"
        << "  --deadlock-report <path>\n"
        << "  --stop-on-deadlock\n"
        << "  --render\n";
}

PathAlgo parseAlgorithm(const std::string& value) {
    if (value == "default" || value == "1") {
        return PathAlgo::Default;
    }
    if (value == "astar" || value == "2") {
        return PathAlgo::AStarSimple;
    }
    if (value == "dstar" || value == "3") {
        return PathAlgo::DStarBasic;
    }
    throw std::runtime_error("invalid --algo value: " + value);
}

SimulationMode parseMode(const std::string& value) {
    if (value == "custom") {
        return SimulationMode::Custom;
    }
    if (value == "realtime") {
        return SimulationMode::Realtime;
    }
    throw std::runtime_error("invalid --mode value: " + value);
}

PhaseConfig parsePhase(const std::string& value) {
    const auto delimiter = value.find(':');
    if (delimiter == std::string::npos) {
        throw std::runtime_error("phase must be <park:N|exit:N>");
    }

    const std::string kind = value.substr(0, delimiter);
    const int taskCount = std::stoi(value.substr(delimiter + 1));
    if (kind != "park" && kind != "exit") {
        throw std::runtime_error("invalid phase type: " + kind);
    }

    PhaseConfig phase;
    phase.type = (kind == "exit") ? PhaseType::Exit : PhaseType::Park;
    phase.taskCount = taskCount;
    return phase;
}

CliOptions parseArgs(int argc, char* argv[]) {
    CliOptions options;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else if (!parse_headless_option(options, arg, i, argc, argv)) {
            throw std::runtime_error("unknown argument: " + std::string(arg));
        }
    }

    if (options.launch.seed == 0) {
        options.launch.seed = 1;
    }

    return options;
}

int runInteractive() {
    SimulationEngine engine;
    engine.prepareConsole();
    while (true) {
        if (!engine.interactiveSetup()) {
            std::cout << "\nSimulation cancelled.\n";
            return 0;
        }

        const bool return_to_menu = engine.runInteractiveConsole();
        if (return_to_menu) {
            std::cout << "\nReturned to launch menu.\n";
            continue;
        }

        engine.printPerformanceSummary();
        std::cout << "\nPress any key to exit...\n";
        (void)console_read_key_blocking();
        return 0;
    }
}

int runHeadless(const CliOptions& options) {
    SimulationEngine engine;
    const agv::core::ValidationResult validation = agv::core::validateLaunchConfig(options.launch);
    if (!validation.ok()) {
        for (const auto& issue : validation.errors) {
            std::cerr << issue.field << ": " << issue.message << '\n';
        }
        return 1;
    }

    engine.setTerminalOutputEnabled(options.run.renderOutputEnabled);
    engine.configureLaunch(validation.normalizedConfig);
    engine.startConfiguredSession();

    auto build_deadlock_report_path = [](const std::string& base_path, const agv::core::MetricsSnapshot& metrics) {
        std::filesystem::path path(base_path);
        const std::string stem = path.stem().string();
        const std::string suffix =
            "_deadlock_" + std::to_string(metrics.deadlockCount) +
            "_step_" + std::to_string(metrics.recordedSteps);
        if (path.has_extension()) {
            path.replace_filename(stem + suffix + path.extension().string());
        } else {
            path += suffix;
        }
        return path.string();
    };

    const bool needs_step_loop =
        options.run.maxSteps.has_value() ||
        options.run.debugReportPath.has_value() ||
        options.run.deadlockReportPath.has_value() ||
        options.run.stopOnDeadlock;

    if (needs_step_loop) {
        std::uint64_t previous_deadlock_count = 0;
        const int max_steps = options.run.maxSteps.value_or(std::numeric_limits<int>::max());
        for (int step = 0; step < max_steps && !engine.isComplete(); ++step) {
            engine.step();
            const auto metrics = engine.snapshotMetrics();
            if (metrics.deadlockCount > previous_deadlock_count) {
                previous_deadlock_count = metrics.deadlockCount;
                if (options.run.deadlockReportPath.has_value()) {
                    const std::string report_path = build_deadlock_report_path(*options.run.deadlockReportPath, metrics);
                    engine.writeDebugReport(report_path, true);
                }
                if (options.run.stopOnDeadlock) {
                    break;
                }
            }
        }
    } else {
        engine.runUntilComplete();
    }

    if (options.run.debugReportPath.has_value()) {
        engine.writeDebugReport(*options.run.debugReportPath, true);
    }

    const auto metrics = engine.snapshotMetrics();
    std::cout
        << "steps=" << metrics.recordedSteps
        << " tasks=" << metrics.tasksCompletedTotal
        << " throughput=" << metrics.throughput
        << " movement=" << metrics.totalMovementCost
        << " stall_steps=" << metrics.stallStepCount
        << " deadlocks=" << metrics.deadlockCount
        << " outstanding=" << metrics.outstandingTaskCount
        << " cpu_ms=" << metrics.totalCpuTimeMs
        << " planning_ms=" << metrics.totalPlanningTimeMs
        << '\n';
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const CliOptions options = parseArgs(argc, argv);
        if (options.interactive) {
            return runInteractive();
        }
        return runHeadless(options);
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << '\n';
        printUsage();
        return 1;
    }
}
