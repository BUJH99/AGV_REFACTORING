#define _CRT_SECURE_NO_WARNINGS

#include <cctype>
#include <cstdio>
#include <cstring>

#include "agv_legacy_engine_internal.hpp"

#ifndef APPEND_FMT
#define APPEND_FMT(P, REM, ...)                            \
    do {                                                  \
        int __w = snprintf((P), (REM), __VA_ARGS__);      \
        if (__w < 0) __w = 0;                             \
        if ((size_t)__w > (REM)) __w = (int)(REM);        \
        (P) += __w;                                       \
        (REM) -= __w;                                     \
    } while (0)
#endif

#ifndef C_NRM
#define C_NRM "\x1b[0m"
#define C_RED "\x1b[31m"
#define C_GRN "\x1b[32m"
#define C_YEL "\x1b[33m"
#define C_CYN "\x1b[36m"
#define C_WHT "\x1b[37m"
#define C_GRY "\x1b[90m"
#define C_B_RED "\x1b[1;31m"
#define C_B_GRN "\x1b[1;32m"
#define C_B_YEL "\x1b[1;33m"
#define C_B_MAG "\x1b[1;35m"
#define C_B_CYN "\x1b[1;36m"
#define C_B_WHT "\x1b[1;37m"
#endif

#ifndef STATUS_STRING_WIDTH
#define STATUS_STRING_WIDTH 25
#endif

#ifndef DISTANCE_BEFORE_CHARGE
#define DISTANCE_BEFORE_CHARGE 300.0
#endif

#ifndef REALTIME_MODE_TIMELIMIT
#define REALTIME_MODE_TIMELIMIT 1000000
#endif

#ifndef MAX_SPEED_MULTIPLIER
#define MAX_SPEED_MULTIPLIER 10000.0f
#endif

#ifndef RENDER_STRIDE_MAX
#define RENDER_STRIDE_MAX 8
#endif

#ifndef RENDER_STRIDE_MIN
#define RENDER_STRIDE_MIN 1
#endif

static const char* AGENT_COLORS[10] = {
    C_B_CYN, C_B_YEL, C_B_MAG, C_B_GRN, C_B_RED,
    C_B_WHT, C_CYN,   C_YEL,   C_B_MAG, C_GRN
};

void ui_clear_screen_optimized();
void logger_log(Logger* logger, const char* format, ...);

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

void ui_handle_control_key(Simulation* sim, int ch, int* is_paused, int* quit_flag) {
    switch (tolower(ch)) {
    case 'p':
        *is_paused = !*is_paused;
        logger_log(sim->logger, *is_paused ? "[CTRL] Simulation Paused." : "[CTRL] Simulation Resumed.");
        break;
    case 's':
        if (*is_paused) logger_log(sim->logger, "[CTRL] Advancing one step.");
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
        *quit_flag = TRUE;
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

static void ui_append_playback_controls_help(char** p, size_t* rem) {
    APPEND_FMT(*p, *rem, "[%sP%s]ause/Resume | [%sS%s]tep | [%s+%s]/[%s-%s] Speed | ",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
}

static void ui_append_render_controls_help(char** p, size_t* rem) {
    APPEND_FMT(*p, *rem, "[%s[%s]/[%s]%s Render stride | ", C_YEL, C_NRM, C_YEL, C_NRM);
    APPEND_FMT(*p, *rem, "[%sF%s]ast render | [%sC%s]olor simple | [%sQ%s]uit\n",
        C_YEL, C_NRM, C_YEL, C_NRM, C_YEL, C_NRM);
}

static void ui_append_controls_help(char** p, size_t* rem) {
    APPEND_FMT(*p, *rem, "%s--- Controls ---%s\n", C_B_WHT, C_NRM);
    ui_append_playback_controls_help(p, rem);
    ui_append_render_controls_help(p, rem);
}

static void ui_position_frame_output(const RendererState& render_state) {
    if (!render_state.fast_render) ui_clear_screen_optimized();
    else fputs("\x1b[H", stdout);
}

static void ui_write_display_buffer_contents(const Simulation* sim) {
    const char* display_buffer = sim->display_buffer.data();
    size_t cur_len = strlen(display_buffer);
    fwrite(display_buffer, 1, cur_len, stdout);
    fflush(stdout);
}

static void ui_flush_display_buffer(const Simulation* sim) {
    ui_position_frame_output(sim->render_state);
    ui_write_display_buffer_contents(sim);
}

static void grid_map_render_base_layer(
    char view[GRID_HEIGHT][GRID_WIDTH],
    const char* colors[GRID_HEIGHT][GRID_WIDTH],
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
    char view[GRID_HEIGHT][GRID_WIDTH],
    const char* colors[GRID_HEIGHT][GRID_WIDTH],
    const GridMap* map,
    const AgentManager* am,
    int simple_colors) {
    int ncs = map->num_charge_stations;
    for (int i = 0; i < ncs; i++) {
        Node* cs = map->charge_stations[i];
        view[cs->y][cs->x] = 'e';
        if (!simple_colors) {
            int charging = FALSE;
            for (int j = 0; j < MAX_AGENTS; j++) {
                if (am->agents[j].state == CHARGING && am->agents[j].pos == cs) {
                    charging = TRUE;
                    break;
                }
            }
            colors[cs->y][cs->x] = charging ? C_B_RED : C_B_YEL;
        }
    }
}

static void grid_map_render_goals(
    char view[GRID_HEIGHT][GRID_WIDTH],
    const char* colors[GRID_HEIGHT][GRID_WIDTH],
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
    char view[GRID_HEIGHT][GRID_WIDTH],
    const char* colors[GRID_HEIGHT][GRID_WIDTH],
    const AgentManager* am) {
    for (int i = 0; i < MAX_AGENTS; i++) {
        if (!am->agents[i].pos) continue;
        Node* n = am->agents[i].pos;
        view[n->y][n->x] = am->agents[i].symbol;
        colors[n->y][n->x] = AGENT_COLORS[i % 10];
    }
}

static void grid_map_append_simple_view(
    char** p,
    size_t* rem,
    char view[GRID_HEIGHT][GRID_WIDTH]) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            if (*rem <= 1) break;
            *(*p)++ = view[y][x];
            (*rem)--;
        }
        if (*rem <= 1) break;
        *(*p)++ = '\n';
        (*rem)--;
    }
}

static void grid_map_append_color_view(
    char** p,
    size_t* rem,
    char view[GRID_HEIGHT][GRID_WIDTH],
    const char* colors[GRID_HEIGHT][GRID_WIDTH]) {
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            APPEND_FMT(*p, *rem, "%s%c%s", colors[y][x], view[y][x], C_NRM);
        }
        APPEND_FMT(*p, *rem, "\n");
    }
}

static int GridMap_renderToBuffer(
    char* buffer,
    size_t buffer_size,
    const GridMap* map,
    const AgentManager* am,
    const RendererState& render_state) {
    static char view[GRID_HEIGHT][GRID_WIDTH];
    static const char* colors[GRID_HEIGHT][GRID_WIDTH];
    char* p = buffer;
    size_t rem = buffer_size;

    grid_map_render_base_layer(view, colors, map);
    grid_map_render_charge_stations(view, colors, map, am, render_state.simple_colors);
    grid_map_render_goals(view, colors, map, render_state.simple_colors);
    grid_map_render_agents(view, colors, am);

    APPEND_FMT(p, rem, "%s\n--- D* Lite + WHCA* + WFG(SCC) + partial CBS ---%s\n", C_B_WHT, C_NRM);
    if (render_state.simple_colors) grid_map_append_simple_view(&p, &rem, view);
    else grid_map_append_color_view(&p, &rem, view, colors);
    APPEND_FMT(p, rem, "\n");
    return (int)(p - buffer);
}

static void simulation_append_custom_status_header(
    char** p,
    size_t* rem,
    const Simulation* sim,
    const ScenarioManager* sc,
    int is_paused) {
    if (sc->current_phase_index < sc->num_phases) {
        const DynamicPhase* ph = &sc->phases[sc->current_phase_index];
        APPEND_FMT(*p, *rem, "--- Custom Scenario: %d/%d [Speed: %.1fx] ---  (Map #%d)",
            sc->current_phase_index + 1, sc->num_phases, sc->speed_multiplier, sim->map_id);
        if (is_paused) APPEND_FMT(*p, *rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
        APPEND_FMT(*p, *rem, "\n");
        APPEND_FMT(*p, *rem, "Time: %d, Current Task: %s (%d/%d)\n",
            sc->time_step, ph->type_name, sc->tasks_completed_in_phase, ph->task_count);
        return;
    }

    APPEND_FMT(*p, *rem, "--- Custom Scenario: All phases complete ---  (Map #%d)\n", sim->map_id);
}

static void simulation_append_realtime_status_header(
    char** p,
    size_t* rem,
    const Simulation* sim,
    const ScenarioManager* sc,
    int is_paused) {
    APPEND_FMT(*p, *rem, "--- Real-Time Simulation [Speed: %.1fx] ---  (Map #%d)",
        sc->speed_multiplier, sim->map_id);
    if (is_paused) APPEND_FMT(*p, *rem, " %s[ PAUSED ]%s", C_B_YEL, C_B_WHT);
    APPEND_FMT(*p, *rem, "\n");

    int park = 0;
    int exitc = 0;
    for (const TaskNode& task : sc->task_queue) {
        if (task.type == TASK_PARK) park++;
        else if (task.type == TASK_EXIT) exitc++;
    }

    APPEND_FMT(*p, *rem, "Time: %d / %d | Pending Tasks: %d (%sPark: %d%s, %sExit: %d%s)\n",
        sc->time_step, REALTIME_MODE_TIMELIMIT, sc->task_count,
        C_B_GRN, park, C_NRM, C_B_YEL, exitc, C_NRM);
}

static const char* simulation_display_path_algo_label(PathAlgo path_algo) {
    if (path_algo == PATHALGO_ASTAR_SIMPLE) return "A* (Single-Agent)";
    if (path_algo == PATHALGO_DSTAR_BASIC) return "D* Lite (Incremental)";
    return "Default (WHCA*+D*Lite+WFG+CBS)";
}

static void simulation_append_runtime_summary_lines(
    char** p,
    size_t* rem,
    const Simulation* sim,
    const AgentManager* am,
    const GridMap* map,
    double avg_cpu_ms) {
    APPEND_FMT(*p, *rem, "Parked Cars: %d/%d\n%s", am->total_cars_parked, map->num_goals, C_NRM);
    APPEND_FMT(*p, *rem, "CPU Time (ms) - Last: %.3f | Avg: %.3f | Total: %.2f\n",
        sim->last_step_cpu_time_ms, avg_cpu_ms, sim->total_cpu_time_ms);
}

static void simulation_append_path_algo_line(char** p, size_t* rem, PathAlgo path_algo) {
    APPEND_FMT(*p, *rem, "%sPath Algo:%s %s\n",
        C_B_WHT, C_NRM, simulation_display_path_algo_label(path_algo));
}

static void simulation_append_planner_status_line(char** p, size_t* rem, const Simulation* sim) {
    APPEND_FMT(*p, *rem, "%sWHCA horizon:%s %d  | wf_edges(last): %d  | SCC(last): %d  | CBS(last): %s (exp:%d)\n",
        C_B_WHT, C_NRM, sim->runtime_tuning.whca_horizon,
        sim->planner_metrics.wf_edges_last,
        sim->planner_metrics.scc_last,
        sim->planner_metrics.cbs_ok_last ? "OK" : "FAIL",
        sim->planner_metrics.cbs_exp_last);
}

static void simulation_append_grid_render(
    char** p,
    size_t* rem,
    const GridMap* map,
    const AgentManager* am,
    const RendererState& render_state) {
    int w = GridMap_renderToBuffer(*p, *rem, map, am, render_state);
    if (w < 0) w = 0;
    if ((size_t)w >= *rem) {
        *p += *rem - 1;
        *rem = 1;
    } else {
        *p += w;
        *rem -= w;
    }
}

static void simulation_append_agent_status_lines(
    char** p,
    size_t* rem,
    const AgentManager* am) {
    static const char* stS[] = {
        "IDLE", "GOING_TO_PARK", "RETURN_HOME_EMPTY", "GOING_TO_COLLECT",
        "RETURN_WITH_CAR", "GO_TO_CHARGE", "CHARGING", "RETURN_HOME_MAINT"
    };
    static const char* stC[] = { C_GRY,C_YEL,C_CYN,C_YEL,C_GRN,C_B_RED,C_RED,C_CYN };

    for (int i = 0; i < MAX_AGENTS; i++) {
        const Agent* ag = &am->agents[i];
        const char* c = AGENT_COLORS[i % 10];

        char sbuf[100];
        if (ag->state == CHARGING) snprintf(sbuf, sizeof(sbuf), "CHARGING... (%d)", ag->charge_timer);
        else snprintf(sbuf, sizeof(sbuf), "%s", stS[ag->state]);

        APPEND_FMT(*p, *rem, "%sAgent %c%s: (%2d,%d) ",
            c, ag->symbol, C_NRM,
            ag->pos ? ag->pos->x : -1, ag->pos ? ag->pos->y : -1);

        if (ag->goal) APPEND_FMT(*p, *rem, "-> (%2d,%d) ", ag->goal->x, ag->goal->y);
        else APPEND_FMT(*p, *rem, "-> (none)          ");

        APPEND_FMT(*p, *rem, "[Mileage: %6.1f/%d] [%s%-*s%s]  [stuck:%d]\n",
            ag->total_distance_traveled, (int)DISTANCE_BEFORE_CHARGE,
            stC[ag->state], STATUS_STRING_WIDTH, sbuf, C_NRM, ag->stuck_steps);
    }
    APPEND_FMT(*p, *rem, "\n");
}

static void simulation_append_log_lines(char** p, size_t* rem, const Logger* lg) {
    APPEND_FMT(*p, *rem, "%s--- Simulation Log ---%s\n", C_B_WHT, C_NRM);
    for (int i = 0; i < lg->log_count; i++) {
        int idx = (lg->log_head + i) % LOG_BUFFER_LINES;
        APPEND_FMT(*p, *rem, "%s%s%s\n", C_GRY, lg->logs[idx], C_NRM);
        if (*rem < 512) break;
    }
}

static void simulation_maybe_flush_display_buffer(const Simulation* sim) {
    static int s_frame_counter = 0;
    s_frame_counter++;
    if (!sim->render_state.suppress_flush &&
        (s_frame_counter % (sim->render_state.render_stride > 0 ? sim->render_state.render_stride : 1)) == 0) {
        ui_flush_display_buffer(sim);
    }
}

void simulation_display_status(Simulation* sim, int is_paused) {
    const ScenarioManager* sc = sim->scenario_manager;
    const AgentManager* am = sim->agent_manager;
    const GridMap* map = sim->map;
    const Logger* lg = sim->logger;
    const int display_steps = (sim->total_executed_steps > 0) ? sim->total_executed_steps : sc->time_step;
    const double avg_cpu_ms = (display_steps > 0) ? (sim->total_cpu_time_ms / (double)display_steps) : 0.0;

    char* p = sim->display_buffer.data();
    size_t rem = sim->display_buffer.size();

    APPEND_FMT(p, rem, "%s", C_B_WHT);
    if (sc->mode == MODE_CUSTOM) simulation_append_custom_status_header(&p, &rem, sim, sc, is_paused);
    else if (sc->mode == MODE_REALTIME) simulation_append_realtime_status_header(&p, &rem, sim, sc, is_paused);

    simulation_append_runtime_summary_lines(&p, &rem, sim, am, map, avg_cpu_ms);
    simulation_append_path_algo_line(&p, &rem, sim->path_algo);
    simulation_append_planner_status_line(&p, &rem, sim);
    simulation_append_grid_render(&p, &rem, map, am, sim->render_state);
    simulation_append_agent_status_lines(&p, &rem, am);
    simulation_append_log_lines(&p, &rem, lg);
    ui_append_controls_help(&p, &rem);
    simulation_maybe_flush_display_buffer(sim);
}
