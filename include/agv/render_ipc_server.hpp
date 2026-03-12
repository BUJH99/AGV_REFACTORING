#pragma once

#include "agv/simulation_engine.hpp"

#include <iosfwd>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace agv::ipc {

class RenderIpcServer {
public:
    RenderIpcServer();

    std::vector<nlohmann::json> processRequest(const nlohmann::json& request);
    void run(std::istream& input, std::ostream& output);

private:
    core::SimulationEngine engine_{};
    bool subscribed_{false};
    bool paused_{false};

    nlohmann::json buildFrameEvent();
};

}  // namespace agv::ipc
