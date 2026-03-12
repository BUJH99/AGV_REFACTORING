#pragma once

#include "agv/internal/engine_internal.hpp"
#include "agv/simulation_engine.hpp"

#include <string_view>

namespace agv::internal {

struct SimulationEngineAccess {
    static Simulation& rebuild(core::SimulationEngine& engine);
    static Simulation& requireInitialized(core::SimulationEngine& engine, std::string_view operation);
};

}  // namespace agv::internal
