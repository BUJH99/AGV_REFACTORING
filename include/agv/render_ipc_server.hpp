#pragma once

#include "agv/simulation_engine.hpp"

#include <cstdint>
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
    bool shutting_down_{false};
    std::uint64_t current_session_id_{0};
    std::uint64_t next_event_id_{1};

    nlohmann::json buildFrameEvent();
};

}  // namespace agv::ipc
