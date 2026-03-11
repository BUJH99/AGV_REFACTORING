#include "agv/simulation_engine.hpp"
#include "agv/internal/engine_internal.hpp"

#include <cstdlib>
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
        options.seed = static_cast<std::uint32_t>(std::stoul(next_argument_value(index, argc, argv, "--seed")));
        mark_headless_mode(options);
    } else if (argument == "--map") {
        options.mapId = std::stoi(next_argument_value(index, argc, argv, "--map"));
        mark_headless_mode(options);
    } else if (argument == "--algo") {
        options.algorithm = parseAlgorithm(next_argument_value(index, argc, argv, "--algo"));
        mark_headless_mode(options);
    } else if (argument == "--mode") {
        options.scenario.mode = parseMode(next_argument_value(index, argc, argv, "--mode"));
        mark_headless_mode(options);
    } else if (argument == "--speed") {
        options.scenario.speedMultiplier = std::stod(next_argument_value(index, argc, argv, "--speed"));
        mark_headless_mode(options);
    } else if (argument == "--phase") {
        if (options.scenario.phases.size() == 1 &&
            options.scenario.phases.front().type == PhaseType::Park &&
            options.scenario.phases.front().taskCount == 1) {
            options.scenario.phases.clear();
        }
        options.scenario.phases.push_back(parsePhase(next_argument_value(index, argc, argv, "--phase")));
        mark_headless_mode(options);
    } else if (argument == "--park-chance") {
        options.scenario.realtimeParkChance = std::stoi(next_argument_value(index, argc, argv, "--park-chance"));
        mark_headless_mode(options);
    } else if (argument == "--exit-chance") {
        options.scenario.realtimeExitChance = std::stoi(next_argument_value(index, argc, argv, "--exit-chance"));
        mark_headless_mode(options);
    } else if (argument == "--max-steps") {
        options.maxSteps = std::stoi(next_argument_value(index, argc, argv, "--max-steps"));
        mark_headless_mode(options);
    } else if (argument == "--render") {
        options.suppressOutput = false;
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
        << "  --map <1-5>\n"
        << "  --algo <default|astar|dstar|1|2|3>\n"
        << "  --mode <custom|realtime>\n"
        << "  --speed <multiplier>\n"
        << "  --phase <park:N|exit:N>\n"
        << "  --park-chance <0-100>\n"
        << "  --exit-chance <0-100>\n"
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
        const std::string_view arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage();
            std::exit(0);
        } else if (!parse_headless_option(options, arg, i, argc, argv)) {
            throw std::runtime_error("unknown argument: " + std::string(arg));
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
    (void)console_read_key_blocking();
    return 0;
}

int runHeadless(const CliOptions& options) {
    SimulationEngine engine;
    engine.setSeed(options.seed);
    engine.loadMap(options.mapId);
    engine.setAlgorithm(options.algorithm);
    engine.configureScenario(options.scenario);
    engine.setSuppressOutput(options.suppressOutput);
    if (options.maxSteps.has_value()) {
        for (int step = 0; step < *options.maxSteps && !engine.isComplete(); ++step) {
            engine.step();
        }
    } else {
        engine.runUntilComplete();
    }

    const auto metrics = engine.snapshotMetrics();
    std::cout
        << "steps=" << metrics.recordedSteps
        << " tasks=" << metrics.tasksCompletedTotal
        << " movement=" << metrics.totalMovementCost
        << " deadlocks=" << metrics.deadlockCount
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
