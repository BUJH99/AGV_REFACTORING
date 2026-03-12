#pragma once

#include "agv/simulation_engine.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace agv::internal::launch_ui {

struct MapOption {
    int id{1};
    std::string_view key{};
    std::string_view label{};
    std::string_view description{};
    int capacity{1};
};

struct AlgorithmOption {
    core::PathAlgo value{core::PathAlgo::Default};
    std::string_view key{};
    std::string_view label{};
    std::string_view description{};
};

struct ModeOption {
    core::SimulationMode value{core::SimulationMode::Custom};
    std::string_view key{};
    std::string_view label{};
    std::string_view description{};
};

struct WizardStepOption {
    std::string_view key{};
    std::string_view title{};
    std::string_view description{};
};

const std::array<MapOption, 7>& map_options();
const std::array<AlgorithmOption, 3>& algorithm_options();
const std::array<ModeOption, 2>& mode_options();
const std::array<WizardStepOption, 7>& wizard_steps();
const MapOption& map_option(int map_id);
const AlgorithmOption& algorithm_option(core::PathAlgo algorithm);
const ModeOption& mode_option(core::SimulationMode mode);
core::LaunchConfig recommended_launch_config(std::uint32_t seed);
std::string summarize_launch_config(const core::LaunchConfig& config);

}  // namespace agv::internal::launch_ui
