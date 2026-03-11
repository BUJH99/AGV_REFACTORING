#include "agv/simulation_engine.hpp"

#include <conio.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
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
    std::uint32_t seed{1};
    int mapId{1};
    PathAlgo algorithm{PathAlgo::Default};
    ScenarioConfig scenario{};
    bool suppressOutput{true};
    std::optional<int> maxSteps;
    std::optional<std::filesystem::path> summaryPath;
    std::optional<std::filesystem::path> stepMetricsPath;
};

void printUsage() {
    std::cout
        << "Usage:\n"
        << "  agv_console                     # interactive console mode\n"
        << "  agv_console --headless [options]\n\n"
        << "Headless options:\n"
        << "  --seed <n>\n"
        << "  --map <1-5>\n"
        << "  --algo <default|astar|dstar|1|2|3>\n"
        << "  --mode <custom|realtime>\n"
        << "  --speed <multiplier>\n"
        << "  --phase <park:N|exit:N>\n"
        << "  --park-chance <0-100>\n"
        << "  --exit-chance <0-100>\n"
        << "  --summary <path.json>\n"
        << "  --steps <path.csv>\n"
        << "  --max-steps <n>\n"
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
        const std::string arg = argv[i];

        auto nextValue = [&](std::string_view name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + std::string(name));
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else if (arg == "--headless") {
            options.interactive = false;
        } else if (arg == "--seed") {
            options.seed = static_cast<std::uint32_t>(std::stoul(nextValue("--seed")));
        } else if (arg == "--map") {
            options.mapId = std::stoi(nextValue("--map"));
        } else if (arg == "--algo") {
            options.algorithm = parseAlgorithm(nextValue("--algo"));
        } else if (arg == "--mode") {
            options.scenario.mode = parseMode(nextValue("--mode"));
            options.interactive = false;
        } else if (arg == "--speed") {
            options.scenario.speedMultiplier = std::stod(nextValue("--speed"));
            options.interactive = false;
        } else if (arg == "--phase") {
            if (options.scenario.phases.size() == 1 &&
                options.scenario.phases.front().type == PhaseType::Park &&
                options.scenario.phases.front().taskCount == 1) {
                options.scenario.phases.clear();
            }
            options.scenario.phases.push_back(parsePhase(nextValue("--phase")));
            options.interactive = false;
        } else if (arg == "--park-chance") {
            options.scenario.realtimeParkChance = std::stoi(nextValue("--park-chance"));
            options.interactive = false;
        } else if (arg == "--exit-chance") {
            options.scenario.realtimeExitChance = std::stoi(nextValue("--exit-chance"));
            options.interactive = false;
        } else if (arg == "--summary") {
            options.summaryPath = std::filesystem::path(nextValue("--summary"));
            options.interactive = false;
        } else if (arg == "--steps") {
            options.stepMetricsPath = std::filesystem::path(nextValue("--steps"));
            options.interactive = false;
        } else if (arg == "--max-steps") {
            options.maxSteps = std::stoi(nextValue("--max-steps"));
            options.interactive = false;
        } else if (arg == "--render") {
            options.suppressOutput = false;
            options.interactive = false;
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (options.scenario.mode == SimulationMode::Custom && options.scenario.phases.empty()) {
        options.scenario.phases.push_back({PhaseType::Park, 1});
    }

    return options;
}

int runInteractive() {
    SimulationEngine engine;
    engine.prepareConsole();
    if (!engine.interactiveSetup()) {
        std::cout << "\nSimulation cancelled.\n";
        return 0;
    }

    engine.runInteractiveConsole();
    engine.printPerformanceSummary();
    std::cout << "\nPress any key to exit...\n";
    (void)_getch();
    return 0;
}

int runHeadless(const CliOptions& options) {
    SimulationEngine engine;
    if (options.stepMetricsPath.has_value()) {
        const auto parent = options.stepMetricsPath->parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
    }
    engine.setSeed(options.seed);
    engine.loadMap(options.mapId);
    engine.setAlgorithm(options.algorithm);
    engine.configureScenario(options.scenario);
    engine.setSuppressOutput(options.suppressOutput);
    engine.setStepMetricsOutput(options.stepMetricsPath);
    if (options.maxSteps.has_value()) {
        for (int step = 0; step < *options.maxSteps && !engine.isComplete(); ++step) {
            engine.step();
        }
    } else {
        engine.runUntilComplete();
    }

    if (options.summaryPath.has_value()) {
        const auto parent = options.summaryPath->parent_path();
        if (!parent.empty()) {
            std::filesystem::create_directories(parent);
        }
        engine.writeRunSummary(*options.summaryPath);
    }

    const auto metrics = engine.snapshotMetrics();
    if (!options.summaryPath.has_value()) {
        std::cout
            << "steps=" << metrics.recordedSteps
            << " tasks=" << metrics.tasksCompletedTotal
            << " movement=" << metrics.totalMovementCost
            << " deadlocks=" << metrics.deadlockCount
            << '\n';
    }
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
