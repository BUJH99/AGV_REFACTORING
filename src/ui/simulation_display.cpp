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

std::string_view agent_state_short_label(agv::core::AgentState state) {
    switch (state) {
        case agv::core::AgentState::GoingToPark:
            return "TO_PARK";
        case agv::core::AgentState::ReturningHomeEmpty:
            return "HOME_EMPTY";
        case agv::core::AgentState::GoingToCollect:
            return "TO_PICK";
        case agv::core::AgentState::ReturningWithCar:
            return "RETURN_CAR";
        case agv::core::AgentState::GoingToCharge:
            return "TO_CHARGE";
        case agv::core::AgentState::Charging:
            return "CHARGING";
        case agv::core::AgentState::ReturningHomeMaintenance:
            return "HOME_MAINT";
        case agv::core::AgentState::Idle:
        default:
            return "IDLE";
    }
}

const char* log_level_color(const StructuredLogEntry& entry) {
    if (entry.level == "Warn") {
        return C_B_YEL;
    }
    if (entry.level == "Error") {
        return C_B_RED;
    }
    if (entry.category == "Control") {
        return C_B_CYN;
    }
    if (entry.category == "Planner") {
        return C_B_MAG;
    }
    if (entry.category == "Charge") {
        return C_B_GRN;
    }
    if (entry.category == "Wait") {
        return C_CYN;
    }
    if (entry.category == "Dispatch") {
        return C_GRN;
    }
    if (entry.category == "Scenario") {
        return C_YEL;
    }
    if (entry.category == "Deadlock") {
        return C_B_RED;
    }
    return C_GRY;
}

std::string_view wait_reason_label(agv::core::AgentWaitReason reason) {
    switch (reason) {
        case agv::core::AgentWaitReason::Idle:
            return "IDLE";
        case agv::core::AgentWaitReason::Charging:
            return "CHG";
        case agv::core::AgentWaitReason::GoalAction:
            return "ACT";
        case agv::core::AgentWaitReason::Rotation:
            return "ROT";
        case agv::core::AgentWaitReason::BlockedByStationary:
            return "BLK";
        case agv::core::AgentWaitReason::PriorityYield:
            return "YLD";
        case agv::core::AgentWaitReason::Stuck:
            return "STK";
        case agv::core::AgentWaitReason::Oscillating:
            return "OSC";
        case agv::core::AgentWaitReason::None:
        default:
            return "-";
    }
}

std::string format_task_summary(const AgentRenderState& agent) {
    if (!agent.taskActive) {
        return "-";
    }
    return linef("%3d/%4.1f/%2d", agent.taskAgeSteps, agent.taskDistance, agent.taskTurns);
}

std::string format_timer_summary(const AgentRenderState& agent) {
    return linef("%2d/%2d/%2d", agent.chargeTimer, agent.actionTimer, agent.rotationWait);
}

std::string format_log_prefix(const StructuredLogEntry& entry) {
    std::string prefix = linef("[s%06d][%s]", entry.step, entry.category.c_str());
    if (entry.agentId.has_value()) {
        prefix += linef("[A%d]", *entry.agentId);
    }
    if (entry.phaseIndex.has_value()) {
        prefix += linef("[P%d]", *entry.phaseIndex);
    }
    return prefix;
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

    const char* mode_label = (hud.mode == agv::core::SimulationMode::Realtime) ? "Real-Time" : "Custom";
    lines.push_back(linef(
        "%s--- %s Simulation --- Map #%d | Step %d | Speed %.1fx%s%s",
        C_B_WHT,
        mode_label,
        hud.mapId,
        hud.step,
        hud.speedMultiplier,
        paused_suffix.c_str(),
        C_NRM));
    lines.push_back(linef(
        "Backlog/Completion | queued:%d inflight:%d outstanding:%d oldest:%d last_done:%d ago:%d parked:%d/%d",
        hud.queuedTaskCount,
        hud.inFlightTaskCount,
        hud.outstandingTaskCount,
        hud.oldestQueuedRequestAge,
        hud.lastTaskCompletionStep,
        hud.stepsSinceLastTaskCompletion,
        hud.parkedCars,
        hud.totalGoalCount));
    lines.push_back(linef(
        "Fleet Activity     | ready_idle:%d action:%d waiting:%d stuck:%d oscillating:%d no_move:%d/%d",
        hud.readyIdleAgentCount,
        hud.activeGoalActionCount,
        hud.waitingAgentCount,
        hud.stuckAgentCount,
        hud.oscillatingAgentCount,
        hud.noMovementStreak,
        hud.maxNoMovementStreak));
    lines.push_back(linef(
        "Planner Pipeline   | moves p/r/b/f=%d/%d/%d/%d cancel r/b/o=%d/%d/%d wf:%d scc:%d cbs:%s exp:%d whca:%d cpu:%.3f/%.3f/%.2f",
        hud.plannedMoveCount,
        hud.postRotationMoveCount,
        hud.postBlockerMoveCount,
        hud.finalMoveCount,
        hud.rotationCanceledCount,
        hud.blockerCanceledCount,
        hud.orderCanceledCount,
        hud.plannerWaitEdges,
        hud.plannerSccCount,
        hud.plannerCbsSucceeded ? "OK" : "FAIL",
        hud.plannerCbsExpansions,
        hud.whcaHorizon,
        hud.lastPlanningTimeMs,
        hud.lastStepCpuTimeMs,
        hud.totalCpuTimeMs));
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
    lines.push_back(linef(
        "%-5s %-11s %-9s %-9s %-14s %-5s %-8s %-7s %-6s",
        "AGV",
        "State",
        "Pos",
        "Goal",
        "Task(age/d/t)",
        "Wait",
        "Timers",
        "Stk/Osc",
        "Dist"));
    for (const AgentRenderState& agent : agents) {
        const char* agent_color = kAgentColors[static_cast<std::size_t>(agent.id % kAgentColors.size())];
        const std::string task = format_task_summary(agent);
        const std::string timers = format_timer_summary(agent);
        const std::string wait = std::string(wait_reason_label(agent.waitReason));
        lines.push_back(linef(
            "%s%-5c%s %-11.11s (%2d,%2d)   (%2d,%2d)   %-14.14s %-5.5s %-8.8s %2d/%-4d %.1f",
            agent_color,
            agent.symbol,
            C_NRM,
            agent_state_short_label(agent.state).data(),
            agent.position.x,
            agent.position.y,
            agent.goal.x,
            agent.goal.y,
            task.c_str(),
            wait.c_str(),
            timers.c_str(),
            agent.stuckSteps,
            agent.oscillationSteps,
            agent.totalDistanceTraveled));
    }
}

void append_log_lines(std::vector<std::string>& lines, const std::vector<StructuredLogEntry>& logs) {
    lines.push_back(linef("%s--- Simulation Log ---%s", C_B_WHT, C_NRM));
    if (logs.empty()) {
        lines.push_back(linef("%s(no logs captured)%s", C_GRY, C_NRM));
        return;
    }

    for (const StructuredLogEntry& entry : logs) {
        const std::string prefix = format_log_prefix(entry);
        lines.push_back(linef("%s%s %s%s", log_level_color(entry), prefix.c_str(), entry.text.c_str(), C_NRM));
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
        C_YEL + "C" + C_NRM + "]olor simple | [" + C_YEL + "M" + C_NRM +
        "]enu (paused) | [" + C_YEL + "Q" + C_NRM + "]uit");
}

std::vector<std::string> compose_terminal_lines(Simulation* sim, bool is_paused) {
    RenderQueryOptions options;
    options.paused = is_paused;
    options.logsTail = true;
    options.maxLogEntries = RENDER_LOG_TAIL_LINES;
    options.plannerOverlay = false;

    const StaticSceneSnapshot scene = snapshot_static_scene(sim);
    const RenderFrameSnapshot frame = snapshot_render_frame(sim, options);

    std::vector<std::string> lines;
    lines.reserve(static_cast<std::size_t>(GRID_HEIGHT + MAX_AGENTS + RENDER_LOG_TAIL_LINES + 24));

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

void ui_apply_speed_multiplier_delta(Simulation* sim, float delta) {
    if (!sim || !sim->scenario_manager) {
        return;
    }

    double next = static_cast<double>(sim->scenario_manager->speed_multiplier) + delta;
    if (next < 0.1) {
        next = 0.1;
    }
    if (next > static_cast<double>(MAX_SPEED_MULTIPLIER)) {
        next = static_cast<double>(MAX_SPEED_MULTIPLIER);
    }
    simulation_set_speed_multiplier(sim, next);
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

void ui_handle_control_key(
    Simulation* sim,
    int ch,
    bool& is_paused,
    bool& quit_flag,
    bool& return_to_menu) {
    switch (tolower(ch)) {
        case 'p':
            is_paused = !is_paused;
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                is_paused ? "Simulation paused." : "Simulation resumed.");
            break;
        case 's':
            if (is_paused) {
                logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                    "Advancing one step.");
            }
            break;
        case '+':
        case '=':
            ui_apply_speed_multiplier_delta(sim, 0.5f);
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                "Speed increased to %.1fx", sim->scenario_manager->speed_multiplier);
            break;
        case '-':
            ui_apply_speed_multiplier_delta(sim, -0.5f);
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                "Speed decreased to %.1fx", sim->scenario_manager->speed_multiplier);
            break;
        case 'q':
            quit_flag = true;
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                "Quit simulation.");
            break;
        case 'm':
            if (is_paused) {
                return_to_menu = true;
                logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                    "Returning to launch menu.");
            }
            break;
        case ']':
            if (sim->render_state.render_stride < RENDER_STRIDE_MAX) {
                sim->render_state.render_stride <<= 1;
            }
            if (sim->render_state.render_stride < RENDER_STRIDE_MIN) {
                sim->render_state.render_stride = RENDER_STRIDE_MIN;
            }
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                "Render stride = %d", sim->render_state.render_stride);
            break;
        case '[':
            if (sim->render_state.render_stride > RENDER_STRIDE_MIN) {
                sim->render_state.render_stride >>= 1;
            }
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                "Render stride = %d", sim->render_state.render_stride);
            break;
        case 'f':
            sim->render_state.fast_render = !sim->render_state.fast_render;
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                sim->render_state.fast_render ? "Fast render ON" : "Fast render OFF");
            break;
        case 'c':
            sim->render_state.simple_colors = !sim->render_state.simple_colors;
            logger_log_event(sim->logger, "Control", "Info", std::nullopt, std::nullopt,
                sim->render_state.simple_colors ? "Simple colors ON" : "Simple colors OFF");
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
