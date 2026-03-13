#include "agv/internal/launch_ui_metadata.hpp"
#include "agv/internal/engine_internal.hpp"

#include <algorithm>
#include <sstream>

namespace agv::internal::launch_ui {

namespace {

int compute_map_capacity(int map_id) {
    GridMap map{};
    AgentManager agents{};
    grid_map_load_scenario(&map, &agents, map_id);
    return std::max(map.num_goals, 1);
}

}  // namespace

const std::array<MapOption, 7>& map_options() {
    static const std::array<MapOption, 7> options{{
        {1, "map1", "Compact parking lot", "Baseline map for quick validation runs.", compute_map_capacity(1)},
        {2, "map2", "Mid-size lot", "Mid-size lot with one retrieval target.", compute_map_capacity(2)},
        {3, "map3", "16 AGVs + 900 requests", "Stress map with A~P agents and heavy traffic.", compute_map_capacity(3)},
        {4, "map4", "Dense lot", "Dense lot with one retrieval target and four parking waves.", compute_map_capacity(4)},
        {5, "map5", "Cross intersection micro-map", "Four-way conflict and swap resolution micro-map.", compute_map_capacity(5)},
        {6, "map6", "Corner-case gauntlet", "Single-lane loops, bridge bottleneck, bays, and charger branches.", compute_map_capacity(6)},
        {7, "map7", "Reference split-room map", "Split-room reference map with shared trunk.", compute_map_capacity(7)},
    }};
    return options;
}

const std::array<AlgorithmOption, 3>& algorithm_options() {
    static const std::array<AlgorithmOption, 3> options{{
        {core::PathAlgo::Default, "default", "Default", "WHCA* + D* Lite + WFG + CBS hybrid planner."},
        {core::PathAlgo::AStarSimple, "astar", "AStar", "Single-agent replanning from scratch each step."},
        {core::PathAlgo::DStarBasic, "dstar", "DStar", "Incremental D* Lite replanning when the map changes."},
    }};
    return options;
}

const std::array<ModeOption, 2>& mode_options() {
    static const std::array<ModeOption, 2> options{{
        {core::SimulationMode::Custom, "custom", "Custom", "Phased scenario with explicit park and exit tasks."},
        {core::SimulationMode::Realtime, "realtime", "Realtime", "Random request generation with park/exit probabilities."},
    }};
    return options;
}

const std::array<WizardStepOption, 7>& wizard_steps() {
    static const std::array<WizardStepOption, 7> options{{
        {"map", "Map Setup", "Choose the parking layout and overall capacity envelope."},
        {"algorithm", "Algorithm Setup", "Pick the planner behavior used during each simulation step."},
        {"mode", "Mode Setup", "Choose between explicit phase replay and probabilistic live traffic."},
        {"scenario", "Scenario Setup", "Edit the mode-specific traffic inputs for this run."},
        {"speed", "Speed Setup", "Set deliberate slowdown; 0.0 keeps the simulation at full speed."},
        {"seed", "Seed Setup", "Fix the random seed when you need a run to be reproducible."},
        {"summary", "Summary", "Review the final normalized config before the session starts."},
    }};
    return options;
}

const MapOption& map_option(int map_id) {
    const auto& options = map_options();
    const auto it = std::find_if(options.begin(), options.end(), [map_id](const MapOption& option) {
        return option.id == map_id;
    });
    return (it != options.end()) ? *it : options.front();
}

const AlgorithmOption& algorithm_option(core::PathAlgo algorithm) {
    const auto& options = algorithm_options();
    const auto it = std::find_if(options.begin(), options.end(), [algorithm](const AlgorithmOption& option) {
        return option.value == algorithm;
    });
    return (it != options.end()) ? *it : options.front();
}

const ModeOption& mode_option(core::SimulationMode mode) {
    const auto& options = mode_options();
    const auto it = std::find_if(options.begin(), options.end(), [mode](const ModeOption& option) {
        return option.value == mode;
    });
    return (it != options.end()) ? *it : options.front();
}

core::LaunchConfig recommended_launch_config(std::uint32_t seed) {
    core::LaunchConfig config;
    config.seed = seed;
    config.mapId = 1;
    config.algorithm = core::PathAlgo::Default;
    config.scenario.mode = core::SimulationMode::Custom;
    config.scenario.speedMultiplier = 0.0;
    config.scenario.realtimeParkChance = 0;
    config.scenario.realtimeExitChance = 0;
    config.scenario.phases = {{core::PhaseType::Park, 1}};
    return config;
}

std::string summarize_launch_config(const core::LaunchConfig& config) {
    std::ostringstream out;
    const MapOption& map = map_option(config.mapId);
    const AlgorithmOption& algorithm = algorithm_option(config.algorithm);
    const ModeOption& mode = mode_option(config.scenario.mode);

    out << "Map " << map.id << " (" << map.label << "), "
        << algorithm.label << ", "
        << mode.label;
    if (config.scenario.mode == core::SimulationMode::Realtime) {
        out << ", park=" << config.scenario.realtimeParkChance
            << "% exit=" << config.scenario.realtimeExitChance << "%";
    } else if (!config.scenario.phases.empty()) {
        out << ", " << ((config.scenario.phases.front().type == core::PhaseType::Exit) ? "Exit" : "Park")
            << " x" << config.scenario.phases.front().taskCount;
        if (config.scenario.phases.size() > 1) {
            out << " +" << (config.scenario.phases.size() - 1) << " phase(s)";
        }
    }
    out << ", speed=" << config.scenario.speedMultiplier
        << ", seed=" << config.seed;
    return out.str();
}

}  // namespace agv::internal::launch_ui
