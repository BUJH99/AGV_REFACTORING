#define _CRT_SECURE_NO_WARNINGS

#include <array>
#include <cctype>
#include <string>
#include <string_view>

#include "agv/internal/engine_internal.hpp"

using GridCharBuffer = std::array<std::array<char, GRID_WIDTH>, GRID_HEIGHT>;
using GridColorBuffer = std::array<std::array<const char*, GRID_WIDTH>, GRID_HEIGHT>;

static constexpr std::array<const char*, 10> AGENT_COLORS = {
    C_B_CYN, C_B_YEL, C_B_MAG, C_B_GRN, C_B_RED,
    C_B_WHT, C_CYN,   C_YEL,   C_B_MAG, C_GRN
};

struct DisplayFrameContext final {
    const Simulation* sim{nullptr};
    const ScenarioManager* scenario{nullptr};
    const AgentManager* agents{nullptr};
    const GridMap* map{nullptr};
    const Logger* logger{nullptr};
    double avg_cpu_ms{0.0};
    bool is_paused{false};
};

class DisplayBufferWriter final {
public:
    explicit DisplayBufferWriter(std::string& buffer, size_t max_size)
        : buffer_(buffer), max_size_(max_size) {
        buffer_.clear();
    }

    template <typename... Args>
    void appendf(std::string_view format, Args&&... args) {
        if (remaining() == 0) return;
        append(agv::internal::text::printf_like(format, std::forward<Args>(args)...));
    }

    void appendChar(char ch) {
        if (remaining() == 0) return;
        buffer_.push_back(ch);
    }

    void appendText(std::string_view text) {
        if (remaining() == 0) return;
        append(text);
    }

    void appendColoredChar(const char* color, char ch, const char* reset = C_NRM) {
        appendText(color ? std::string_view(color) : std::string_view{});
        appendChar(ch);
        appendText(reset ? std::string_view(reset) : std::string_view{});
    }

    size_t remaining() const {
        if (buffer_.size() >= max_size_) {
            return 0;
        }
        return max_size_ - buffer_.size();
    }

    void finish() {
        if (buffer_.size() > max_size_) {
            buffer_.resize(max_size_);
        }
    }

private:
    void append(std::string_view text) {
        const size_t advance = std::min(remaining(), text.size());
        if (advance == 0) return;
        buffer_.append(text.data(), advance);
    }

    std::string& buffer_;
    size_t max_size_{0};
};

void ui_clear_screen_optimized();

static void ui_update_simulation_speed_from_multiplier(Simulation* sim) {
    sim->scenario_manager->simulation_speed = (int)(100.0f / sim->scenario_manager->speed_multiplier);
    if (sim->scenario_manager->simulation_speed < 0) sim->scenario_manager->simulation_speed = 0;
}

static void ui_apply_speed_multiplier_delta(Simulation* sim, float delta) {
    sim->scenario_manager->speed_multiplier += delta;
    if (sim->scenario_manager->speed_multiplier > MAX_SPEED_MULTIPLIER)
        sim->scenario_manager->speed_multiplier = MAX_SPEED_MULTIPLIER;
    if (sim->scenario_manager->speed_multiplier < 0.1f)
        sim->scenario_manager->speed_multiplier = 0.1f;
    ui_update_simulation_speed_from_multiplier(sim);
}

static void ui_toggle_fast_render(Simulation* sim) {
    sim->render_state.fast_render = !sim->render_state.fast_render;
    logger_log(sim->logger, sim->render_state.fast_render ? "[CTRL] Fast render ON" : "[CTRL] Fast render OFF");
}

static void ui_toggle_simple_colors(Simulation* sim) {
    sim->render_state.simple_colors = !sim->render_state.simple_colors;
    logger_log(sim->logger, sim->render_state.simple_colors ? "[CTRL] Simple colors ON" : "[CTRL] Simple colors OFF");
}

static void ui_adjust_render_stride(Simulation* sim, int delta) {
    if (delta > 0) {
        if (sim->render_state.render_stride < RENDER_STRIDE_MAX) sim->render_state.render_stride <<= 1;
        if (sim->render_state.render_stride < RENDER_STRIDE_MIN) sim->render_state.render_stride = RENDER_STRIDE_MIN;
    } else if (delta < 0) {
        if (sim->render_state.render_stride > RENDER_STRIDE_MIN) sim->render_state.render_stride >>= 1;
    }
    logger_log(sim->logger, "[CTRL] Render stride = %d", sim->render_state.render_stride);
}

void ui_handle_control_key(Simulation* sim, int ch, bool& is_paused, bool& quit_flag) {
    switch (tolower(ch)) {
    case 'p':
        is_paused = !is_paused;
        logger_log(sim->logger, is_paused ? "[CTRL] Simulation Paused." : "[CTRL] Simulation Resumed.");
        break;
    case 's':
        if (is_paused) logger_log(sim->logger, "[CTRL] Advancing one step.");
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
        ui_adjust_render_stride(sim, 1);
        break;
    case '[':
        ui_adjust_render_stride(sim, -1);
        break;
    case 'f':
        ui_toggle_fast_render(sim);
        break;
    case 'c':
        ui_toggle_simple_colors(sim);
        break;
    }
}

static void ui_append_playback_controls_help(DisplayBufferWriter& writer) {
    writer.appendf("[%sP%s]ause/Resume | [%sS%s]tep | [%s+%s]/[%s-%s] Speed | ",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
}

static void ui_append_render_controls_help(DisplayBufferWriter& writer) {
    writer.appendf("[%s[%s]/[%s]%s Render stride | ", C_YEL, C_NRM, C_YEL, C_NRM);
    writer.appendf("[%sF%s]ast render | [%sC%s]olor simple | [%sQ%s]uit\n",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
}

static void ui_append_controls_help(DisplayBufferWriter& writer) {
    writer.appendf("%s--- Controls ---%s\n", C_B_WHT, C_NRM);
    ui_append_playback_controls_help(writer);
    ui_append_render_controls_help(writer);
}

static void ui_position_frame_output(const RendererState& render_state) {
    if (!render_state.fast_render) ui_clear_screen_optimized();
    else agv::internal::text::console_write("\x1b[H");
}

static void ui_write_display_buffer_contents(const Simulation* sim) {
    agv::internal::text::console_write(sim->display_buffer);
}

static void ui_flush_display_buffer(const Simulation* sim) {
    ui_position_frame_output(sim->render_state);
    ui_write_display_buffer_contents(sim);
}

static size_t simulation_render_stride_or_one(const RendererState& render_state) {
    return (render_state.render_stride > 0) ? (size_t)render_state.render_stride : 1u;
}

static void grid_map_render_base_layer(
    GridCharBuffer& view,
    GridColorBuffer& colors,
    const GridMap* map) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            const Node* n = &map->grid[y][x];
            if (n->is_obstacle) {
                view[y][x] = '+';
                colors[y][x] = C_WHT;
            } else {
                view[y][x] = '.';
                colors[y][x] = C_GRY;
            }
        }
    }
}

static void grid_map_render_charge_stations(
    GridCharBuffer& view,
    GridColorBuffer& colors,
    const GridMap* map,
    const AgentManager* am,
    int simple_colors) {
    int ncs = map->num_charge_stations;
    for (int i = 0; i < ncs; i++) {
        Node* cs = map->charge_stations[i];
        view[cs->y][cs->x] = 'e';
        if (!simple_colors) {
            bool charging = false;
            for (int j = 0; j < MAX_AGENTS; j++) {
                if (am->agents[j].state == AgentState::Charging && am->agents[j].pos == cs) {
                    charging = true;
                    break;
                }
            }
            colors[cs->y][cs->x] = charging ? C_B_RED : C_B_YEL;
        }
    }
}

static void grid_map_render_goals(
    GridCharBuffer& view,
    GridColorBuffer& colors,
    const GridMap* map,
    int simple_colors) {
    int ng = map->num_goals;
    for (int i = 0; i < ng; i++) {
        Node* g = map->goals[i];
        if (g->is_parked) {
            view[g->y][g->x] = 'P';
            if (!simple_colors) colors[g->y][g->x] = C_RED;
        } else if (g->is_goal) {
            view[g->y][g->x] = 'G';
            if (!simple_colors) colors[g->y][g->x] = C_GRN;
        }
    }
}

static void grid_map_render_agents(
    GridCharBuffer& view,
    GridColorBuffer& colors,
    const AgentManager* am) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!am->agents[i].pos) continue;
        Node* n = am->agents[i].pos;
        view[n->y][n->x] = am->agents[i].symbol;
        colors[n->y][n->x] = AGENT_COLORS[i % 10];
    }
}

static void grid_map_append_simple_view(
    DisplayBufferWriter& writer,
    const GridCharBuffer& view) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (writer.remaining() == 0) break;
            writer.appendChar(view[y][x]);
        }
        if (writer.remaining() == 0) break;
        writer.appendChar('\n');
    }
}

static void grid_map_append_color_view(
    DisplayBufferWriter& writer,
    const GridCharBuffer& view,
    const GridColorBuffer& colors) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            writer.appendColoredChar(colors[y][x], view[y][x]);
        }
        writer.appendChar('\n');
    }
}

static void grid_map_render_to_writer(
    DisplayBufferWriter& writer,
    const GridMap* map,
    const AgentManager* am,
    const RendererState& render_state) {
    GridCharBuffer view{};
    GridColorBuffer colors{};

    grid_map_render_base_layer(view, colors, map);
    grid_map_render_charge_stations(view, colors, map, am, render_state.simple_colors);
    grid_map_render_goals(view, colors, map, render_state.simple_colors);
    grid_map_render_agents(view, colors, am);

    writer.appendf("%s\n--- D* Lite + WHCA* + WFG(SCC) + partial CBS ---%s\n", C_B_WHT, C_NRM);
    if (render_state.simple_colors) grid_map_append_simple_view(writer, view);
    else grid_map_append_color_view(writer, view, colors);
    writer.appendf("\n");
}

static void simulation_append_custom_status_header(
    DisplayBufferWriter& writer,
    const Simulation* sim,
    const ScenarioManager* sc,
    bool is_paused) {
    if (sc->current_phase_index < sc->num_phases) {
        const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
        writer.appendf("--- Custom Scenario: %d/%d [Speed: %.1fx] ---  (Map #%d)",
            sc->current_phase_index + 1, sc->num_phases, sc->speed_multiplier, sim->map_id);
        if (is_paused) writer.appendf(" %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
        writer.appendf("\n");
        writer.appendf("Time: %d, Current Task: %s (%d/%d)\n",
            sc->time_step, ph->type_name.c_str(), sc->tasks_completed_in_phase, ph->task_count);
        return;
    }

    writer.appendf("--- Custom Scenario: All phases complete ---  (Map #%d)\n", sim->map_id);
}

static void simulation_append_realtime_status_header(
    DisplayBufferWriter& writer,
    const DisplayFrameContext& frame) {
    writer.appendf("--- Real-Time Simulation [Speed: %.1fx] ---  (Map #%d)",
        frame.scenario->speed_multiplier, frame.sim->map_id);
    if (frame.is_paused) writer.appendf(" %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
    writer.appendf("\n");

    int park = 0;
    int exitc = 0;
    for (const TaskNode& task : frame.scenario->task_queue) {
        if (task.type == TaskType::Park) park++;
        else if (task.type == TaskType::Exit) exitc++;
    }

    writer.appendf("Time: %d / %d | Pending Tasks: %d (%sPark: %d%s, %sExit: %d%s)\n",
        frame.scenario->time_step, REALTIME_MODE_TIMELIMIT, frame.scenario->task_count,
        C_B_GRN, park, C_NRM, C_B_YEL, exitc, C_NRM);
}

static std::string_view simulation_display_path_algo_label(PathAlgo path_algo) {
    if (path_algo == PathAlgo::AStarSimple) return "A* (Single-Agent)";
    if (path_algo == PathAlgo::DStarBasic) return "D* Lite (Incremental)";
    return "Default (WHCA*+D*Lite+WFG+CBS)";
}

static void simulation_append_runtime_summary_lines(
    DisplayBufferWriter& writer,
    const DisplayFrameContext& frame) {
    writer.appendf("Parked Cars: %d/%d\n%s", frame.agents->total_cars_parked, frame.map->num_goals, C_NRM);
    writer.appendf("CPU Time (ms) - Last: %.3f | Avg: %.3f | Total: %.2f\n",
        frame.sim->last_step_cpu_time_ms, frame.avg_cpu_ms, frame.sim->total_cpu_time_ms);
}

static void simulation_append_path_algo_line(DisplayBufferWriter& writer, PathAlgo path_algo) {
    writer.appendf("%sPath Algo:%s %s\n",
        C_B_WHT, C_NRM, simulation_display_path_algo_label(path_algo));
}

static void simulation_append_planner_status_line(DisplayBufferWriter& writer, const Simulation* sim) {
    writer.appendf("%sWHCA horizon:%s %d  | wf_edges(last): %d  | SCC(last): %d  | CBS(last): %s (exp:%d)\n",
        C_B_WHT, C_NRM, sim->runtime_tuning.whca_horizon,
        sim->planner_metrics.wf_edges_last,
        sim->planner_metrics.scc_last,
        sim->planner_metrics.cbs_ok_last ? "OK" : "FAIL",
        sim->planner_metrics.cbs_exp_last);
}

static void simulation_append_grid_render(
    DisplayBufferWriter& writer,
    const GridMap* map,
    const AgentManager* am,
    const RendererState& render_state) {
    grid_map_render_to_writer(writer, map, am, render_state);
}

static void simulation_append_agent_status_lines(
    DisplayBufferWriter& writer,
    const AgentManager* am) {
    static constexpr std::array<std::string_view, 8> stS = {
        "IDLE", "GOING_TO_PARK", "RETURN_HOME_EMPTY", "GOING_TO_COLLECT",
        "RETURN_WITH_CAR", "GO_TO_CHARGE", "CHARGING", "RETURN_HOME_MAINT"
    };
    static constexpr std::array<std::string_view, 8> stC = { C_GRY,C_YEL,C_CYN,C_YEL,C_GRN,C_B_RED,C_RED,C_CYN };

    for (int i = 0; i < MAX_AGENTS; i++) {
        const Agent* ag = &am->agents[i];
        const std::string_view c = AGENT_COLORS[i % 10];

        const std::string status_label = (ag->state == AgentState::Charging)
            ? agv::internal::text::printf_like("CHARGING... (%d)", ag->charge_timer)
            : std::string(stS[static_cast<int>(ag->state)]);

        writer.appendf("%sAgent %c%s: (%2d,%d) ",
            c, ag->symbol, C_NRM,
            ag->pos ? ag->pos->x : -1, ag->pos ? ag->pos->y : -1);

        if (ag->goal) writer.appendf("-> (%2d,%d) ", ag->goal->x, ag->goal->y);
        else writer.appendf("-> (none)          ");

        writer.appendf("[Mileage: %6.1f/%d] [%s%-*s%s]  [stuck:%d]\n",
            ag->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
            stC[static_cast<int>(ag->state)], STATUS_STRING_WIDTH, status_label, C_NRM, ag->stuck_steps);
    }
    writer.appendf("\n");
}

static void simulation_append_log_lines(DisplayBufferWriter& writer, const Logger* lg) {
    writer.appendf("%s--- Simulation Log ---%s\n", C_B_WHT, C_NRM);
    for (int i = 0; i < lg->log_count; i++) {
        int idx = (lg->log_head + i) % LOG_BUFFER_LINES;
        writer.appendf("%s%s%s\n", C_GRY, lg->logs[idx].c_str(), C_NRM);
        if (writer.remaining() < 512) break;
    }
}

static void simulation_append_status_header(
    DisplayBufferWriter& writer,
    const DisplayFrameContext& frame) {
    writer.appendf("%s", C_B_WHT);
    if (frame.scenario->mode == SimulationMode::Custom) {
        simulation_append_custom_status_header(writer, frame.sim, frame.scenario, frame.is_paused);
    } else if (frame.scenario->mode == SimulationMode::Realtime) {
        simulation_append_realtime_status_header(writer, frame);
    }
}

class DisplayFrameComposer final {
public:
    DisplayFrameComposer(DisplayBufferWriter& writer, const DisplayFrameContext& frame)
        : writer_(writer), frame_(frame) {}

    void compose() {
        appendHeader();
        appendRuntime();
        appendGrid();
        appendAgents();
        appendLogs();
        appendControls();
    }

private:
    void appendHeader() {
        simulation_append_status_header(writer_, frame_);
    }

    void appendRuntime() {
        simulation_append_runtime_summary_lines(writer_, frame_);
        simulation_append_path_algo_line(writer_, frame_.sim->path_algo);
        simulation_append_planner_status_line(writer_, frame_.sim);
    }

    void appendGrid() {
        simulation_append_grid_render(writer_, frame_.map, frame_.agents, frame_.sim->render_state);
    }

    void appendAgents() {
        simulation_append_agent_status_lines(writer_, frame_.agents);
    }

    void appendLogs() {
        simulation_append_log_lines(writer_, frame_.logger);
    }

    void appendControls() {
        ui_append_controls_help(writer_);
    }

    DisplayBufferWriter& writer_;
    const DisplayFrameContext& frame_;
};

static void simulation_build_display_buffer(Simulation* sim, const DisplayFrameContext& frame) {
    DisplayBufferWriter writer(sim->display_buffer, DISPLAY_BUFFER_SIZE);
    DisplayFrameComposer(writer, frame).compose();
    writer.finish();
}

static bool simulation_should_flush_display_buffer(Simulation* sim) {
    if (!sim || sim->render_state.suppress_flush) {
        return false;
    }

    if (sim->render_state.force_next_flush) {
        sim->render_state.force_next_flush = false;
        return true;
    }

    static int s_frame_counter = 0;
    return (++s_frame_counter % simulation_render_stride_or_one(sim->render_state)) == 0;
}

void simulation_display_status(Simulation* sim, bool is_paused) {
    if (!sim) {
        return;
    }

    const bool should_flush = simulation_should_flush_display_buffer(sim);
    if (!sim->render_state.suppress_flush && !should_flush) {
        return;
    }

    const ScenarioManager* scenario = sim->scenario_manager;
    const int display_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : scenario->time_step;
    const double avg_cpu_ms = (display_steps > 0) ? (sim->total_cpu_time_ms / (double)display_steps) : 0.0;

    const DisplayFrameContext frame{
        sim,
        scenario,
        sim->agent_manager,
        sim->map,
        sim->logger,
        avg_cpu_ms,
        is_paused
    };

    simulation_build_display_buffer(sim, frame);
    if (should_flush) {
        ui_flush_display_buffer(sim);
    }
}

namespace {

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

}  // namespace

RendererFacade renderer_create_facade(void) {
    return RendererFacade(std::make_unique<FunctionRendererStrategy>(simulation_display_status));
}
