#pragma once

#include "agv/simulation_engine.hpp"

#include <filesystem>
#include <iosfwd>
#include <optional>

namespace agv::internal::console {

std::filesystem::path default_last_launch_path();
std::optional<core::LaunchConfig> load_last_launch_config(const std::filesystem::path& path);
bool save_last_launch_config(const std::filesystem::path& path, const core::LaunchConfig& config);
bool run_console_launch_wizard(
    std::istream& input,
    std::ostream& output,
    const std::filesystem::path& last_launch_path,
    core::LaunchConfig& out_config);

}  // namespace agv::internal::console
