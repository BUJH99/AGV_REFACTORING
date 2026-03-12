#pragma once

#include "agv/simulation_engine.hpp"

#include <filesystem>
#include <string>

namespace agv::console {

void prepareConsole();
bool interactiveSetup(core::SimulationEngine& engine);
bool runInteractiveConsole(core::SimulationEngine& engine);
void printPerformanceSummary(core::SimulationEngine& engine);
std::string buildDebugReport(core::SimulationEngine& engine, bool paused = false);
bool writeDebugReport(core::SimulationEngine& engine, const std::string& path, bool paused = false);

}  // namespace agv::console
