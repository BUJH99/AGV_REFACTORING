#define _CRT_SECURE_NO_WARNINGS

#include <array>
#include <cctype>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "agv/internal/engine_internal.hpp"

using agv::core::AgentRenderState;
using agv::core::GoalRenderState;
using agv::core::HudSnapshot;
using agv::core::RenderFrameSnapshot;
using agv::core::RenderQueryOptions;
using agv::core::StaticSceneSnapshot;
using agv::core::StructuredLogEntry;

void ui_clear_screen_optimized();

namespace {

using GridCharBuffer = std::array<std::array<char, GRID_WIDTH>, GRID_HEIGHT>;
using GridColorBuffer = std::array<std::array<const char*, GRID_WIDTH>, GRID_HEIGHT>;

static constexpr std::array<const char*, 10> kAgentColors = {
    C_B_CYN, C_B_YEL, C_B_MAG, C_B_GRN, C_B_RED,
    C_B_WHT, C_CYN,   C_YEL,   C_B_MAG, C_GRN
};

template <typename... Args>
std::string linef(std::string_view format, Args&&... args) {
    return agv::internal::text::printf_like(format, std::forward<Args>(args)...);
}

std::string_view path_algo_label(agv::core::PathAlgo algorithm) {
    switch (algorithm) {
        case agv::core::PathAlgo::AStarSimple:
            return "A* (Single-Agent)";
        case agv::core::PathAlgo::DStarBasic:
            return "D* Lite (Incremental)";
        case agv::core::PathAlgo::Default:
        default:
            return "Default (WHCA*+D*Lite+WFG+CBS)";
    }
}

std::string_view agent_state_label(agv::core::AgentState state) {
    switch (state) {
        case agv::core::AgentState::GoingToPark:
            return "GOING_TO_PARK";
        case agv::core::AgentState::ReturningHomeEmpty:
            return "RETURN_HOME_EMPTY";
        case agv::core::AgentState::GoingToCollect:
            return "GOING_TO_COLLECT";
        case agv::core::AgentState::ReturningWithCar:
            return "RETURN_WITH_CAR";
        case agv::core::AgentState::GoingToCharge:
            return "GO_TO_CHARGE";
        case agv::core::AgentState::Charging:
            return "CHARGING";
        case agv::core::AgentState::ReturningHomeMaintenance:
            return "RETURN_HOME_MAINT";
        case agv::core::AgentState::Idle:
        default:
            return "IDLE";
    }
}

const char* agent_state_color(agv::core::AgentState state) {
    switch (state) {
        case agv::core::AgentState::GoingToPark:
        case agv::core::AgentState::GoingToCollect:
            return C_YEL;
        case agv::core::AgentState::ReturningHomeEmpty:
        case agv::core::AgentState::ReturningHomeMaintenance:
            return C_CYN;
        case agv::core::AgentState::ReturningWithCar:
            return C_GRN;
        case agv::core::AgentState::GoingToCharge:
            return C_B_RED;
        case agv::core::AgentState::Charging:
            return C_RED;
        case agv::core::AgentState::Idle:
        default:
            return C_GRY;
    }
}

const char* log_level_color(const StructuredLogEntry& entry) {
    if (entry.level == "warning") {
        return C_B_YEL;
    }
    if (entry.level == "error") {
        return C_B_RED;
    }
    if (entry.category == "CTRL") {
        return C_B_CYN;
    }
    return C_GRY;
}

std::string join_lines(const std::vector<std::string>& lines) {
    std::string buffer;
    for (const std::string& line : lines) {
        buffer += line;
        buffer.push_back('\n');
    }
    return buffer;
}

void append_blank_line(std::vector<std::string>& lines) {
    lines.emplace_back();
}

void append_hud_lines(std::vector<std::string>& lines, const HudSnapshot& hud) {
    std::string paused_suffix;
    if (hud.paused) {
        paused_suffix = linef(" %s[ PAUSED ]%s", C_B_YEL, C_NRM);
    }

    if (hud.mode == agv::core::SimulationMode::Custom) {
        if (hud.currentPhaseIndex >= 0 && hud.currentPhaseIndex < hud.totalPhases) {
            lines.push_back(linef(
                "%s--- Custom Scenario: %d/%d [Speed: %.1fx] ---  (Map #%d)%s%s",
                C_B_WHT,
                hud.currentPhaseIndex + 1,
                hud.totalPhases,
                hud.speedMultiplier,
                hud.mapId,
                paused_suffix.c_str(),
                C_NRM));
            lines.push_back(linef(
                "Time: %d, Current Task: %s (%d/%d)",
                hud.step,
                hud.currentPhaseType == agv::core::PhaseType::Exit ? "Exit" : "Park",
                hud.phaseTasksCompleted,
                hud.phaseTaskTarget));
        } else {
            lines.push_back(linef(
                "%s--- Custom Scenario: All phases complete ---  (Map #%d)%s%s",
                C_B_WHT,
                hud.mapId,
                paused_suffix.c_str(),
                C_NRM));
            lines.push_back(linef("Time: %d", hud.step));
        }
    } else {
        lines.push_back(linef(
            "%s--- Real-Time Simulation [Speed: %.1fx] ---  (Map #%d)%s%s",
            C_B_WHT,
            hud.speedMultiplier,
            hud.mapId,
            paused_suffix.c_str(),
            C_NRM));
        lines.push_back(linef(
            "Time: %d | Pending Tasks: %d | In-Flight: %d | Outstanding: %d",
            hud.step,
            hud.queuedTaskCount,
            hud.inFlightTaskCount,
            hud.outstandingTaskCount));
    }

    lines.push_back(linef("Parked Cars: %d/%d", hud.parkedCars, hud.totalGoalCount));
    lines.push_back(linef(
        "CPU Time (ms) - Last: %.3f | Avg: %.3f | Total: %.2f",
        hud.lastStepCpuTimeMs,
        hud.avgCpuTimeMs,
        hud.totalCpuTimeMs));
    lines.push_back(linef("%sPath Algo:%s %s", C_B_WHT, C_NRM, path_algo_label(hud.algorithm).data()));
    lines.push_back(linef(
        "%sWHCA horizon:%s %d  | wf_edges(last): %d  | SCC(last): %d  | CBS(last): %s (exp:%d)",
        C_B_WHT,
        C_NRM,
        hud.whcaHorizon,
        hud.plannerWaitEdges,
        hud.plannerSccCount,
        hud.plannerCbsSucceeded ? "OK" : "FAIL",
        hud.plannerCbsExpansions));
}

void initialize_grid_buffers(
    GridCharBuffer& chars,
    GridColorBuffer& colors,
    const StaticSceneSnapshot& scene) {
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            const std::size_t index = static_cast<std::size_t>(y * GRID_WIDTH + x);
            const char tile = (index < scene.baseTiles.size()) ? scene.baseTiles[index] : '.';
            chars[y][x] = tile;
            colors[y][x] = (tile == '+') ? C_WHT : C_GRY;
        }
    }
}

void apply_static_grid_markers(
    GridCharBuffer& chars,
    GridColorBuffer& colors,
    const StaticSceneSnapshot& scene) {
    for (const auto& charger : scene.chargerCells) {
        if (!grid_is_valid_coord(charger.x, charger.y)) {
            continue;
        }
        chars[charger.y][charger.x] = 'e';
        colors[charger.y][charger.x] = C_B_YEL;
    }
}

void apply_goal_markers(
    GridCharBuffer& chars,
    GridColorBuffer& colors,
    const std::vector<GoalRenderState>& goals) {
    for (const GoalRenderState& goal : goals) {
        if (!grid_is_valid_coord(goal.position.x, goal.position.y)) {
            continue;
        }
        chars[goal.position.y][goal.position.x] = goal.isParked ? 'P' : 'G';
        colors[goal.position.y][goal.position.x] = goal.isParked ? C_RED : C_GRN;
    }
}

void apply_agent_markers(
    GridCharBuffer& chars,
    GridColorBuffer& colors,
    const std::vector<AgentRenderState>& agents) {
    for (const AgentRenderState& agent : agents) {
        if (!agent.isActive || !grid_is_valid_coord(agent.position.x, agent.position.y)) {
            continue;
        }
        chars[agent.position.y][agent.position.x] = agent.symbol;
        colors[agent.position.y][agent.position.x] = kAgentColors[static_cast<std::size_t>(agent.id % kAgentColors.size())];
    }

    for (const AgentRenderState& agent : agents) {
        if (agent.state == agv::core::AgentState::Charging &&
            grid_is_valid_coord(agent.position.x, agent.position.y)) {
            colors[agent.position.y][agent.position.x] = C_B_RED;
        }
    }
}

std::string build_grid_row(
    const GridCharBuffer& chars,
    const GridColorBuffer& colors,
    int row,
    bool simple_colors) {
    std::string line;
    if (simple_colors) {
        line.reserve(GRID_WIDTH);
        for (int x = 0; x < GRID_WIDTH; ++x) {
            line.push_back(chars[row][x]);
        }
        return line;
    }

    line.reserve(GRID_WIDTH * 8);
    for (int x = 0; x < GRID_WIDTH; ++x) {
        line += colors[row][x];
        line.push_back(chars[row][x]);
        line += C_NRM;
    }
    return line;
}

void append_grid_lines(
    std::vector<std::string>& lines,
    const StaticSceneSnapshot& scene,
    const RenderFrameSnapshot& frame,
    const RendererState& render_state) {
    GridCharBuffer chars{};
    GridColorBuffer colors{};
    initialize_grid_buffers(chars, colors, scene);
    apply_static_grid_markers(chars, colors, scene);
    apply_goal_markers(chars, colors, frame.goalStates);
    apply_agent_markers(chars, colors, frame.agents);

    lines.push_back(linef("%s--- D* Lite + WHCA* + WFG(SCC) + partial CBS ---%s", C_B_WHT, C_NRM));
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        lines.push_back(build_grid_row(chars, colors, y, render_state.simple_colors));
    }
}

void append_agent_lines(std::vector<std::string>& lines, const std::vector<AgentRenderState>& agents) {
    for (const AgentRenderState& agent : agents) {
        const char* agent_color = kAgentColors[static_cast<std::size_t>(agent.id % kAgentColors.size())];
        const char* state_color = agent_state_color(agent.state);
        const std::string status = (agent.state == agv::core::AgentState::Charging)
            ? linef("CHARGING... (%d)", agent.chargeTimer)
            : std::string(agent_state_label(agent.state));

        std::string line = linef(
            "%sAgent %c%s: (%2d,%d) -> (%2d,%d) [Mileage: %.1f/%d] ",
            agent_color,
            agent.symbol,
            C_NRM,
            agent.position.x,
            agent.position.y,
            agent.goal.x,
            agent.goal.y,
            agent.totalDistanceTraveled,
            static_cast<int>(DISTANCE_BEFORE_CHARGE));
        line += linef("[%s%s%s] [stuck:%d]",
            state_color,
            status.c_str(),
            C_NRM,
            agent.stuckSteps);
        lines.push_back(std::move(line));
    }
}

void append_log_lines(std::vector<std::string>& lines, const std::vector<StructuredLogEntry>& logs) {
    lines.push_back(linef("%s--- Simulation Log ---%s", C_B_WHT, C_NRM));
    if (logs.empty()) {
        lines.push_back(linef("%s(no logs captured)%s", C_GRY, C_NRM));
        return;
    }

    for (const StructuredLogEntry& entry : logs) {
        lines.push_back(linef("%s%s%s", log_level_color(entry), entry.text.c_str(), C_NRM));
    }
}

void append_overlay_lines(std::vector<std::string>& lines, const RenderFrameSnapshot& frame) {
    if (!frame.plannerOverlay.available) {
        return;
    }

    lines.push_back(linef(
        "%s--- Planner Overlay ---%s algo=%s horizon=%d wait_edges=%d leader=%d cbs=%s",
        C_B_WHT,
        C_NRM,
        path_algo_label(frame.plannerOverlay.algorithm).data(),
        frame.plannerOverlay.horizon,
        frame.plannerOverlay.waitEdgeCount,
        frame.plannerOverlay.leaderAgentId,
        frame.plannerOverlay.usedCbs ? "yes" : "no"));
    if (!frame.plannerOverlay.sccParticipantAgentIds.empty()) {
        std::string scc = "SCC participants:";
        for (int agent_id : frame.plannerOverlay.sccParticipantAgentIds) {
            scc += linef(" %d", agent_id);
        }
        lines.push_back(std::move(scc));
    }
}

void append_control_lines(std::vector<std::string>& lines) {
    lines.push_back(linef("%s--- Controls ---%s", C_B_WHT, C_NRM));
    lines.push_back(
        std::string("[") + C_YEL + "P" + C_NRM + "]ause/Resume | [" + C_YEL + "S" + C_NRM +
        "]tep | [" + C_YEL + "+" + C_NRM + "]/[" + C_YEL + "-" + C_NRM +
        "] Speed | [" + C_YEL + "[" + C_NRM + "]/[" + C_YEL + "]" + C_NRM +
        "] Render stride | [" + C_YEL + "F" + C_NRM + "]ast render | [" +
        C_YEL + "C" + C_NRM + "]olor simple | [" + C_YEL + "Q" + C_NRM + "]uit");
}

std::vector<std::string> compose_terminal_lines(Simulation* sim, bool is_paused) {
    RenderQueryOptions options;
    options.paused = is_paused;
    options.logsTail = true;
    options.maxLogEntries = LOG_BUFFER_LINES;
    options.plannerOverlay = false;

    const StaticSceneSnapshot scene = snapshot_static_scene(sim);
    const RenderFrameSnapshot frame = snapshot_render_frame(sim, options);

    std::vector<std::string> lines;
    lines.reserve(static_cast<std::size_t>(GRID_HEIGHT + MAX_AGENTS + LOG_BUFFER_LINES + 24));

    append_hud_lines(lines, frame.hud);
    append_blank_line(lines);
    append_grid_lines(lines, scene, frame, sim->render_state);
    append_blank_line(lines);
    append_agent_lines(lines, frame.agents);
    append_blank_line(lines);
    append_log_lines(lines, frame.logsTail);
    if (frame.plannerOverlay.available) {
        append_blank_line(lines);
        append_overlay_lines(lines, frame);
    }
    append_blank_line(lines);
    append_control_lines(lines);
    return lines;
}

void ui_update_simulation_speed_from_multiplier(Simulation* sim) {
    sim->scenario_manager->simulation_speed = static_cast<int>(100.0f / sim->scenario_manager->speed_multiplier);
    if (sim->scenario_manager->simulation_speed < 0) {
        sim->scenario_manager->simulation_speed = 0;
    }
}

void ui_apply_speed_multiplier_delta(Simulation* sim, float delta) {
    sim->scenario_manager->speed_multiplier += delta;
    if (sim->scenario_manager->speed_multiplier > MAX_SPEED_MULTIPLIER) {
        sim->scenario_manager->speed_multiplier = MAX_SPEED_MULTIPLIER;
    }
    if (sim->scenario_manager->speed_multiplier < 0.1f) {
        sim->scenario_manager->speed_multiplier = 0.1f;
    }
    ui_update_simulation_speed_from_multiplier(sim);
}

std::size_t render_stride_or_one(const RendererState& render_state) {
    return (render_state.render_stride > 0) ? static_cast<std::size_t>(render_state.render_stride) : 1u;
}

class TerminalDiffRenderer final {
public:
    void render(Simulation* sim, bool is_paused) {
        const StaticSceneSnapshot scene = snapshot_static_scene(sim);
        const std::vector<std::string> lines = compose_terminal_lines(sim, is_paused);
        sim->display_buffer = join_lines(lines);

        if (sim->render_state.suppress_flush) {
            previous_lines_ = lines;
            last_session_id_ = scene.sessionId;
            last_scene_version_ = scene.sceneVersion;
            last_simple_colors_ = sim->render_state.simple_colors;
            last_console_size_ = console_current_size();
            return;
        }

        const bool needs_full_redraw =
            previous_lines_.empty() ||
            last_session_id_ != scene.sessionId ||
            last_scene_version_ != scene.sceneVersion ||
            last_simple_colors_ != sim->render_state.simple_colors ||
            console_current_size() != last_console_size_;

        if (needs_full_redraw) {
            ui_clear_screen_optimized();
            agv::internal::text::console_write(sim->display_buffer);
        } else {
            flush_line_diffs(lines);
        }

        previous_lines_ = lines;
        last_session_id_ = scene.sessionId;
        last_scene_version_ = scene.sceneVersion;
        last_simple_colors_ = sim->render_state.simple_colors;
        last_console_size_ = console_current_size();
    }

private:
    void flush_line_diffs(const std::vector<std::string>& lines) const {
        const std::size_t max_rows = std::max(previous_lines_.size(), lines.size());
        for (std::size_t row = 0; row < max_rows; ++row) {
            const std::string current = (row < lines.size()) ? lines[row] : std::string{};
            const std::string previous = (row < previous_lines_.size()) ? previous_lines_[row] : std::string{};
            if (current == previous) {
                continue;
            }

            agv::internal::text::console_write(linef("\x1b[%d;1H", static_cast<int>(row + 1)));
            agv::internal::text::console_write(current);
            agv::internal::text::console_write("\x1b[K");
        }
        agv::internal::text::console_write(linef("\x1b[%d;1H", static_cast<int>(lines.size() + 1)));
    }

    std::vector<std::string> previous_lines_{};
    std::uint64_t last_session_id_{0};
    std::uint64_t last_scene_version_{0};
    bool last_simple_colors_{false};
    std::optional<ConsoleSize> last_console_size_{};
};

TerminalDiffRenderer& terminal_diff_renderer() {
    static TerminalDiffRenderer renderer;
    return renderer;
}

bool simulation_should_flush_display_buffer(Simulation* sim) {
    if (!sim || sim->render_state.suppress_flush) {
        return false;
    }

    if (sim->render_state.force_next_flush) {
        sim->render_state.force_next_flush = false;
        return true;
    }

    static int s_frame_counter = 0;
    return (++s_frame_counter % render_stride_or_one(sim->render_state)) == 0;
}

}  // namespace

void ui_handle_control_key(Simulation* sim, int ch, bool& is_paused, bool& quit_flag) {
    switch (tolower(ch)) {
        case 'p':
            is_paused = !is_paused;
            logger_log(sim->logger, is_paused ? "[CTRL] Simulation Paused." : "[CTRL] Simulation Resumed.");
            break;
        case 's':
            if (is_paused) {
                logger_log(sim->logger, "[CTRL] Advancing one step.");
            }
            break;
        case '+':
        case '=':
            ui_apply_speed_multiplier_delta(sim, 0.5f);
            logger_log(sim->logger, "[CTRL] Speed increased to %.1fx", sim->scenario_manager->speed_multiplier);
            break;
        case '-':
            ui_apply_speed_multiplier_delta(sim, -0.5f);
            logger_log(sim->logger, "[CTRL] Speed decreased to %.1fx", sim->scenario_manager->speed_multiplier);
            break;
        case 'q':
            quit_flag = true;
            logger_log(sim->logger, "[CTRL] Quit simulation.");
            break;
        case ']':
            if (sim->render_state.render_stride < RENDER_STRIDE_MAX) {
                sim->render_state.render_stride <<= 1;
            }
            if (sim->render_state.render_stride < RENDER_STRIDE_MIN) {
                sim->render_state.render_stride = RENDER_STRIDE_MIN;
            }
            logger_log(sim->logger, "[CTRL] Render stride = %d", sim->render_state.render_stride);
            break;
        case '[':
            if (sim->render_state.render_stride > RENDER_STRIDE_MIN) {
                sim->render_state.render_stride >>= 1;
            }
            logger_log(sim->logger, "[CTRL] Render stride = %d", sim->render_state.render_stride);
            break;
        case 'f':
            sim->render_state.fast_render = !sim->render_state.fast_render;
            logger_log(sim->logger, sim->render_state.fast_render ? "[CTRL] Fast render ON" : "[CTRL] Fast render OFF");
            break;
        case 'c':
            sim->render_state.simple_colors = !sim->render_state.simple_colors;
            logger_log(sim->logger, sim->render_state.simple_colors ? "[CTRL] Simple colors ON" : "[CTRL] Simple colors OFF");
            break;
    }
}

void simulation_display_status(Simulation* sim, bool is_paused) {
    if (!sim) {
        return;
    }

    const bool should_flush = simulation_should_flush_display_buffer(sim);
    if (!sim->render_state.suppress_flush && !should_flush) {
        return;
    }

    terminal_diff_renderer().render(sim, is_paused);
}

using DrawFrameFnLocal = void (*)(Simulation_* sim, bool is_paused);

class FunctionRendererStrategy final : public RendererStrategy {
public:
    explicit FunctionRendererStrategy(DrawFrameFnLocal draw_frame)
        : draw_frame_(draw_frame) {}

    void drawFrame(Simulation_* sim, bool is_paused) const override {
        draw_frame_(sim, is_paused);
    }

    std::unique_ptr<RendererStrategy> clone() const override {
        return std::make_unique<FunctionRendererStrategy>(draw_frame_);
    }

private:
    DrawFrameFnLocal draw_frame_{nullptr};
};

RendererFacade renderer_create_facade(void) {
    return RendererFacade(std::make_unique<FunctionRendererStrategy>(simulation_display_status));
}
