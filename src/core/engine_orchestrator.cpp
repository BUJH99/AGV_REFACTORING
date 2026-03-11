#define _CRT_SECURE_NO_WARNINGS

#include "agv/internal/engine_internal.hpp"

bool grid_is_node_blocked(const GridMap* map, const AgentManager* am, const Node* node, const Agent* agent) {
    constexpr int kReturnHomeBayEscapeStuckSteps = 6;
    if (node->is_obstacle || node->is_parked || node->is_temp) return true;

    if (agent && agent->state == AgentState::ReturningHomeEmpty && node->is_goal && !node->is_parked) {
        const bool allow_temporary_bay_escape =
            agent->stuck_steps >= kReturnHomeBayEscapeStuckSteps &&
            (node->reserved_by_agent == -1 || node->reserved_by_agent == agent->id);
        if (!allow_temporary_bay_escape) {
            return true;
        }
    }

    for (int i = 0; i < MAX_AGENTS; i++) {
        if (am->agents[i].pos == node && am->agents[i].state == AgentState::Charging) return true;
    }
    return false;
}

#ifndef AGV_NO_MAIN
int main() {
    agv_prepare_console();
    Simulation sim;

    if (simulation_setup(&sim)) {
        ui_enter_alt_screen();
        sim.run();
        ui_leave_alt_screen();
        sim.printPerformanceSummary();
        agv::internal::text::console_print("\nPress any key to exit...\n");
        (void)console_read_key_blocking();
    } else {
        agv::internal::text::console_print("\nSimulation setup was cancelled. Exiting.\n");
    }

    return 0;
}
#endif
