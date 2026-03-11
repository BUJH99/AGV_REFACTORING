#define _CRT_SECURE_NO_WARNINGS

#include <algorithm>
#include <cstring>

#include "agv/internal/engine_internal.hpp"

#define grid_map_create Grid_create
#define grid_map_destroy Grid_destroy
#define grid_is_valid_coord Grid_isValidCoord

int grid_is_valid_coord(int x, int y) { return (x >= 0 && x < GRID_WIDTH&& y >= 0 && y < GRID_HEIGHT); }

static void grid_map_clear(GridMap* map) {
    memset(map, 0, sizeof(*map));
    for (int y = 0; y < GRID_HEIGHT; ++y)
        for (int x = 0; x < GRID_WIDTH; ++x) {
            map->grid[y][x].x = x;
            map->grid[y][x].y = y;
            map->grid[y][x].is_obstacle = FALSE;
            map->grid[y][x].is_goal = FALSE;
            map->grid[y][x].is_temp = FALSE;
            map->grid[y][x].is_parked = FALSE;
            map->grid[y][x].reserved_by_agent = -1;
        }
    map->num_goals = 0;
    map->num_charge_stations = 0;
}

static void map_all_free(GridMap* m) {
    grid_map_clear(m);
}

static void map_fill_interior(GridMap* map, int is_obstacle) {
    if (!map) return;
    for (int y = 1; y < GRID_HEIGHT - 1; ++y) {
        for (int x = 1; x < GRID_WIDTH - 1; ++x) {
            map->grid[y][x].is_obstacle = is_obstacle;
            if (is_obstacle) {
                map->grid[y][x].is_goal = FALSE;
            }
        }
    }
}

static void map_add_border_walls(GridMap* m) {
    for (int x = 0; x < GRID_WIDTH; ++x) {
        m->grid[0][x].is_obstacle = TRUE;
        m->grid[GRID_HEIGHT - 1][x].is_obstacle = TRUE;
    }
    for (int y = 0; y < GRID_HEIGHT; ++y) {
        m->grid[y][0].is_obstacle = TRUE;
        m->grid[y][GRID_WIDTH - 1].is_obstacle = TRUE;
    }
}

static void map_open_cell(GridMap* map, int x, int y) {
    if (!map || !grid_is_valid_coord(x, y)) return;
    map->grid[y][x].is_obstacle = FALSE;
}

static void map_open_row(GridMap* map, int y, int x_begin = 1, int x_end = GRID_WIDTH - 2) {
    if (!map || y < 0 || y >= GRID_HEIGHT) return;
    const int clamped_begin = std::max(0, x_begin);
    const int clamped_end = std::min(GRID_WIDTH - 1, x_end);
    for (int x = clamped_begin; x <= clamped_end; ++x) {
        map_open_cell(map, x, y);
    }
}

static void map_open_column(GridMap* map, int x, int y_begin = 1, int y_end = GRID_HEIGHT - 2) {
    if (!map || x < 0 || x >= GRID_WIDTH) return;
    const int clamped_begin = std::max(0, y_begin);
    const int clamped_end = std::min(GRID_HEIGHT - 1, y_end);
    for (int y = clamped_begin; y <= clamped_end; ++y) {
        map_open_cell(map, x, y);
    }
}

static void map_clear_goal_rect(GridMap* map, int x_begin, int y_begin, int x_end, int y_end) {
    if (!map) return;
    const int clamped_x_begin = std::max(0, x_begin);
    const int clamped_x_end = std::min(GRID_WIDTH - 1, x_end);
    const int clamped_y_begin = std::max(0, y_begin);
    const int clamped_y_end = std::min(GRID_HEIGHT - 1, y_end);
    for (int y = clamped_y_begin; y <= clamped_y_end; ++y) {
        for (int x = clamped_x_begin; x <= clamped_x_end; ++x) {
            map->grid[y][x].is_goal = FALSE;
        }
    }
}

static void map_place_goal(GridMap* m, int x, int y) {
    if (!grid_is_valid_coord(x, y)) return;
    Node* n = &m->grid[y][x];
    if (!n->is_obstacle && !n->is_goal) {
        n->is_goal = TRUE;
        if (m->num_goals < MAX_GOALS) m->goals[m->num_goals++] = n;
    }
}

static void map_open_goal(GridMap* map, int x, int y) {
    map_open_cell(map, x, y);
    map_place_goal(map, x, y);
}

static void map_place_charge(GridMap* m, int x, int y) {
    if (!grid_is_valid_coord(x, y)) return;
    Node* n = &m->grid[y][x];
    if (!n->is_obstacle) {
        if (m->num_charge_stations < MAX_CHARGE_STATIONS)
            m->charge_stations[m->num_charge_stations++] = n;
    }
}

static void map_open_charge(GridMap* map, int x, int y) {
    map_open_cell(map, x, y);
    map_place_charge(map, x, y);
}

static void map_place_agent_at(AgentManager* am, GridMap* m, int idx, int x, int y) {
    if (!am || !m || idx < 0 || idx >= MAX_AGENTS || !grid_is_valid_coord(x, y)) return;
    Node* n = &m->grid[y][x];
    am->agents[idx].pos = n;
    am->agents[idx].home_base = n;
    am->agents[idx].symbol = 'A' + idx;
    am->agents[idx].heading = DIR_NONE;
    am->agents[idx].rotation_wait = 0;
}

static void map_reserve_area_as_start(GridMap* m, int x0, int y0, int w, int h) {
    for (int y = y0; y < y0 + h && y < GRID_HEIGHT; ++y)
        for (int x = x0; x < x0 + w && x < GRID_WIDTH; ++x) {
            Node* n = &m->grid[y][x];
            n->is_obstacle = FALSE;
            n->is_goal = FALSE;
            n->is_temp = FALSE;
        }
}

static void agent_manager_reset_for_new_map(AgentManager* am) {
    if (!am) return;
    for (int i = 0; i < MAX_AGENTS; i++) {
        am->agents[i].pf.reset();
        am->agents[i].id = i;
        am->agents[i].symbol = 'A' + i;
        am->agents[i].pos = NULL;
        am->agents[i].home_base = NULL;
        am->agents[i].goal = NULL;
        am->agents[i].state = IDLE;
        am->agents[i].total_distance_traveled = 0.0;
        am->agents[i].charge_timer = 0;
        am->agents[i].action_timer = 0;
        am->agents[i].heading = DIR_NONE;
        am->agents[i].rotation_wait = 0;
        am->agents[i].stuck_steps = 0;
        am->agents[i].metrics_task_active = 0;
        am->agents[i].metrics_task_start_step = 0;
        am->agents[i].metrics_distance_at_start = 0.0;
        am->agents[i].metrics_turns_current = 0;
    }
    am->total_cars_parked = 0;
}

static void map_rebuild_goal_index(GridMap* map) {
    if (!map) return;
    map->num_goals = 0;
    for (int y = 1; y < GRID_HEIGHT - 1; ++y) {
        for (int x = 1; x < GRID_WIDTH - 1; ++x) {
            Node* node = &map->grid[y][x];
            if (!node->is_goal) continue;
            if (map->num_goals >= MAX_GOALS) return;
            map->goals[map->num_goals++] = node;
        }
    }
}

static void grid_map_fill_from_string(GridMap* map, AgentManager* am, const char* m) {
    grid_map_clear(map);

    int x = 0, y = 0;
    int last_was_cr = 0;

    for (const char* p = m; *p && y < GRID_HEIGHT; ++p) {
        char ch = *p;

        if (ch == '\r') {
            x = 0;
            y++;
            last_was_cr = 1;
            continue;
        }
        if (ch == '\n') {
            if (!last_was_cr) {
                x = 0;
                y++;
            }
            last_was_cr = 0;
            continue;
        }
        last_was_cr = 0;

        if (x >= GRID_WIDTH) continue;
        if (y >= GRID_HEIGHT) break;

        Node* n = &map->grid[y][x];
        n->is_obstacle = FALSE;
        n->is_goal = FALSE;
        n->is_temp = FALSE;
        n->is_parked = FALSE;
        n->reserved_by_agent = -1;

        switch (ch) {
        case '1':
            n->is_obstacle = TRUE;
            break;
        case 'A':
            am->agents[0].pos = n;
            am->agents[0].home_base = n;
            break;
        case 'B':
            am->agents[1].pos = n;
            am->agents[1].home_base = n;
            break;
        case 'C':
            am->agents[2].pos = n;
            am->agents[2].home_base = n;
            break;
        case 'D':
            am->agents[3].pos = n;
            am->agents[3].home_base = n;
            break;
        case 'G':
            n->is_goal = TRUE;
            if (map->num_goals < MAX_GOALS) map->goals[map->num_goals++] = n;
            break;
        case 'e':
            if (map->num_charge_stations < MAX_CHARGE_STATIONS)
                map->charge_stations[map->num_charge_stations++] = n;
            break;
        case '0':
        default:
            break;
        }

        x++;
    }
}

static void map_build_hypermart(GridMap* m, AgentManager* am) {
    int x, y;

    map_all_free(m);
    map_add_border_walls(m);
    map_fill_interior(m, TRUE);

    map_reserve_area_as_start(m, 2, 2, 8, 5);
    map_place_agent_at(am, m, 0, 2, 2);
    map_place_agent_at(am, m, 1, 3, 2);
    map_place_agent_at(am, m, 2, 4, 2);
    map_place_agent_at(am, m, 3, 5, 2);

    map_open_row(m, 6);

    const int vCols[] = { 12, 22, 32, 42, 52, 62, 72 };
    const int nV = (int)(sizeof(vCols) / sizeof(vCols[0]));
    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        map_open_column(m, cx);
    }

    map_open_row(m, 10);
    map_open_row(m, 30);
    for (y = 19; y <= 21; ++y)
        for (x = 1; x < GRID_WIDTH - 1; ++x)
            m->grid[y][x].is_obstacle = TRUE;

    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        for (y = 19; y <= 21; ++y) m->grid[y][cx].is_obstacle = FALSE;
    }

    m->grid[20][34].is_obstacle = FALSE;
    m->grid[20][50].is_obstacle = FALSE;

    map_open_column(m, 4);
    map_open_row(m, 6, 4, 10);

    const int pocketY[] = { 14, 16, 26, 28, 34 };
    const int nP = (int)(sizeof(pocketY) / sizeof(pocketY[0]));
    for (int i = 0; i < nV; ++i) {
        int cx = vCols[i];
        for (int k = 0; k < nP; ++k) {
            int py = pocketY[k];
            if (py >= 19 && py <= 21) continue;
            if (py == 10 || py == 30) continue;
            map_open_cell(m, cx - 1, py);
            map_open_cell(m, cx + 1, py);
        }
    }

    map_open_charge(m, 12, 8);
    map_open_charge(m, 42, 8);
    map_open_charge(m, 42, 32);
    map_open_charge(m, 72, 8);

    int markRow[GRID_HEIGHT] = { 0 };
    markRow[6] = 1;
    markRow[10] = 1;
    markRow[19] = 1;
    markRow[20] = 1;
    markRow[21] = 1;
    markRow[30] = 1;

    const int y_min = 8, y_max = GRID_HEIGHT - 4;
    for (int i = 0; i < nV; ++i) {
        int roadX = vCols[i];
        int leftCol = roadX - 1;
        int rightCol = roadX + 1;
        for (y = y_min; y <= y_max; ++y) {
            if (markRow[y]) continue;
            if (grid_is_valid_coord(leftCol, y)) {
                map_open_goal(m, leftCol, y);
            }
            if (grid_is_valid_coord(rightCol, y)) {
                map_open_goal(m, rightCol, y);
            }
        }
    }

    const int side_right_col = GRID_WIDTH - 4;
    const int side_right_road = GRID_WIDTH - 5;

    map_open_column(m, side_right_road);
    map_open_column(m, side_right_road, 19, 21);

    for (y = y_min; y <= y_max; ++y) {
        if (markRow[y]) continue;
        if (grid_is_valid_coord(side_right_col, y)) {
            map_open_goal(m, side_right_col, y);
        }
    }

    map_clear_goal_rect(m, 2, 2, 8, 8);
    map_clear_goal_rect(m, 4, 1, 4, GRID_HEIGHT - 2);

    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
        for (x = 1; x < GRID_WIDTH - 1; ++x) {
            Node* n = &m->grid[y][x];
            if (!n->is_goal) continue;
            int ok = 0;
            const int dx[4] = { 1, -1, 0, 0 };
            const int dy[4] = { 0, 0, 1, -1 };
            for (int k = 0; k < 4; k++) {
                int nx = x + dx[k], ny = y + dy[k];
                if (!grid_is_valid_coord(nx, ny)) continue;
                if (!m->grid[ny][nx].is_obstacle) {
                    ok = 1;
                    break;
                }
            }
            if (!ok) n->is_goal = FALSE;
        }
    }
    map_rebuild_goal_index(m);
}

static void map_build_10agents_200slots(GridMap* m, AgentManager* am) {
    int x, y;

    map_all_free(m);
    map_add_border_walls(m);
    map_fill_interior(m, TRUE);

    const int sx0 = 2, sy0 = 2;
    const int sW = 16, sH = 6;
    map_reserve_area_as_start(m, sx0, sy0, sW, sH);

    const int staged_agents = (MAX_AGENTS < 16) ? MAX_AGENTS : 16;
    const int slots_per_row = 8;
    for (int i = 0; i < staged_agents; ++i) {
        int row = i / slots_per_row;
        int col = i % slots_per_row;
        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
    }

    const int lane_w = 1;
    const int y_min = 8;
    const int y_max = GRID_HEIGHT - 5;
    const int ax_start = 16;
    const int ax_end = GRID_WIDTH - 6;
    const int ax_step = 4;
    const int cross_start = 10;
    const int cross_end = GRID_HEIGHT - 6;
    const int cross_step = 6;

    map_open_column(m, 2);
    map_open_column(m, 3);
    map_open_row(m, 6);
    map_open_row(m, 7);

    for (x = ax_start; x <= ax_end; x += ax_step)
        map_open_column(m, x);

    for (y = cross_start; y <= cross_end; y += cross_step)
        map_open_row(m, y);

    {
        int cxL = sx0;
        int cyT = sy0 + 1;
        map_open_charge(m, cxL, cyT);
        map_open_charge(m, cxL + 1, cyT);
        map_open_charge(m, cxL, cyT + 2);
        map_open_charge(m, cxL + 1, cyT + 2);
    }

    int markRow[GRID_HEIGHT] = { 0 };
    markRow[6] = 1;
    markRow[7] = 1;
    for (y = cross_start; y <= cross_end; y += cross_step)
        if (y >= 0 && y < GRID_HEIGHT) markRow[y] = 1;

    int markCol[GRID_WIDTH] = { 0 };
    markCol[2] = 1;
    markCol[3] = 1;
    for (x = ax_start; x <= ax_end; x += ax_step)
        if (x >= 0 && x < GRID_WIDTH) markCol[x] = 1;

    const int target = 900;
    int placed = 0;
    for (x = ax_start; x <= ax_end && placed < target; x += ax_step) {
        int leftCol = x - 1;
        int rightCol = x + lane_w;
        for (y = y_min; y <= y_max && placed < target; ++y) {
            if (markRow[y]) continue;

            if (grid_is_valid_coord(leftCol, y) && placed < target) {
                map_open_goal(m, leftCol, y);
                placed++;
            }
            if (grid_is_valid_coord(rightCol, y) && placed < target) {
                map_open_goal(m, rightCol, y);
                placed++;
            }
        }
    }

    if (placed < target) {
        for (y = cross_start; y <= cross_end && placed < target; y += cross_step) {
            int row = y;
            for (x = 2; x < GRID_WIDTH - 2 && placed < target; ++x) {
                if (markCol[x]) continue;
                if (m->grid[row][x].is_obstacle != FALSE) continue;

                if (grid_is_valid_coord(x, row - 1) &&
                    !m->grid[row - 1][x].is_goal && placed < target) {
                    map_open_goal(m, x, row - 1);
                    placed++;
                }
                if (grid_is_valid_coord(x, row + 1) &&
                    !m->grid[row + 1][x].is_goal && placed < target) {
                    map_open_goal(m, x, row + 1);
                    placed++;
                }
            }
        }
    }

    for (y = 1; y < GRID_HEIGHT - 1; ++y) {
        m->grid[y][2].is_goal = FALSE;
        m->grid[y][3].is_goal = FALSE;
    }
}

static void carve_block_1lane(GridMap* m,
    int cx, int cy, int Wg, int Hg,
    int vstep, int hstep, int vx0, int hy0,
    int CX, int CY) {
    int x, y;

    int gx0 = cx - (Wg / 2 - 1);
    int gx1 = gx0 + Wg - 1;
    int gy0 = cy - (Hg / 2);
    int gy1 = gy0 + Hg - 1;

    for (y = gy0; y <= gy1; ++y)
        for (x = gx0; x <= gx1; ++x) {
            if (!grid_is_valid_coord(x, y)) continue;
            m->grid[y][x].is_obstacle = FALSE;
            map_place_goal(m, x, y);
        }

    int rxL = gx0 - 1, rxR = gx1 + 1;
    int ryT = gy0 - 1, ryB = gy1 + 1;

    for (x = gx0 - 1; x <= gx1 + 1; ++x) {
        if (grid_is_valid_coord(x, ryT)) m->grid[ryT][x].is_obstacle = FALSE;
        if (grid_is_valid_coord(x, ryB)) m->grid[ryB][x].is_obstacle = FALSE;
    }
    for (y = gy0 - 1; y <= gy1 + 1; ++y) {
        if (grid_is_valid_coord(rxL, y)) m->grid[y][rxL].is_obstacle = FALSE;
        if (grid_is_valid_coord(rxR, y)) m->grid[y][rxR].is_obstacle = FALSE;
    }

    int kx = (cx - vx0 + vstep / 2) / vstep;
    int vx = vx0 + kx * vstep;
    if (vx < 1) vx = 1;
    if (vx > GRID_WIDTH - 2) vx = GRID_WIDTH - 2;

    int ky = (cy - hy0 + hstep / 2) / hstep;
    int hy = hy0 + ky * hstep;
    if (hy < 1) hy = 1;
    if (hy > GRID_HEIGHT - 2) hy = GRID_HEIGHT - 2;

    {
        int linkY = (abs(ryT - CY) <= abs(ryB - CY)) ? ryT : ryB;
        if (vx <= rxL) {
            for (x = vx; x <= rxL; ++x) m->grid[linkY][x].is_obstacle = FALSE;
        } else if (vx >= rxR) {
            for (x = rxR; x <= vx; ++x) m->grid[linkY][x].is_obstacle = FALSE;
        } else {
            m->grid[linkY][vx].is_obstacle = FALSE;
        }
    }

    {
        int linkX = (abs(rxL - CX) <= abs(rxR - CX)) ? rxL : rxR;
        if (hy <= ryT) {
            for (y = hy; y <= ryT; ++y) m->grid[y][linkX].is_obstacle = FALSE;
        } else if (hy >= ryB) {
            for (y = ryB; y <= hy; ++y) m->grid[y][linkX].is_obstacle = FALSE;
        } else {
            m->grid[hy][linkX].is_obstacle = FALSE;
        }
    }
}

static void map_build_biggrid_onegoal(GridMap* m, AgentManager* am) {
    map_all_free(m);
    map_add_border_walls(m);

    int x, y;

    map_fill_interior(m, TRUE);

    const int CX = GRID_WIDTH / 2;
    const int CY = GRID_HEIGHT / 2;
    const int sW = 10, sH = 4;
    const int sx0 = CX - sW / 2;
    const int sy0 = CY - sH / 2;
    map_reserve_area_as_start(m, sx0, sy0, sW, sH);
    for (int i = 0; i < 10; ++i) {
        int row = i / 5, col = i % 5;
        map_place_agent_at(am, m, i, sx0 + col * 2, sy0 + row * 2);
    }

    const int vstep = 5, hstep = 5;
    const int vx0 = 6;
    const int hy0 = 9;

    for (x = vx0; x < GRID_WIDTH - 1; x += vstep)
        map_open_column(m, x);

    for (y = hy0; y < GRID_HEIGHT - 1; y += hstep)
        map_open_row(m, y);

    map_open_row(m, 6);

    const int Wg = 6, Hg = 2;
    const int bxL = 8;
    const int bxR = GRID_WIDTH - 8;
    const int byT = 8;
    const int byB = GRID_HEIGHT - 8;

    carve_block_1lane(m, bxL, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
    carve_block_1lane(m, bxR, byT, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
    carve_block_1lane(m, bxL, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);
    carve_block_1lane(m, bxR, byB, Wg, Hg, vstep, hstep, vx0, hy0, CX, CY);

    map_open_charge(m, CX, CY - 6);
    map_open_charge(m, CX, CY + 6);
    map_open_charge(m, CX - 6, CY);
    map_open_charge(m, CX + 6, CY);

    map_rebuild_goal_index(m);
}

static void map_build_cross_4agents(GridMap* m, AgentManager* am) {
    int x, y;

    map_all_free(m);
    map_add_border_walls(m);

    map_fill_interior(m, TRUE);

    const int CX = GRID_WIDTH / 2;
    const int CY = GRID_HEIGHT / 2;

    map_open_column(m, CX);
    map_open_row(m, CY);

    map_place_agent_at(am, m, 0, 1, CY);
    map_place_agent_at(am, m, 1, GRID_WIDTH - 2, CY);
    map_place_agent_at(am, m, 2, CX, 1);
    map_place_agent_at(am, m, 3, CX, GRID_HEIGHT - 2);

    map_place_goal(m, 1 + 4, CY);
    map_place_goal(m, GRID_WIDTH - 2 - 4, CY);
    map_place_goal(m, CX, 1 + 4);
    map_place_goal(m, CX, GRID_HEIGHT - 2 - 4);

    map_open_charge(m, CX, CY);
}

void grid_map_load_scenario(GridMap* map, AgentManager* am, int scenario_id) {
    agent_manager_reset_for_new_map(am);
    grid_map_clear(map);

    switch (scenario_id) {
    case 1: {
        static const char* MAP1 =
            "1111111111111111111111111111111111111\n"
            "001GGG1GG1GGG1GGG1GGG1GGG1GGG1G11G111\n"
            "A000000000000000000000000000000000001\n"
            "B000000000000000000000000000000000001\n"
            "C001GG1GG1GGG10001GGG1GGG1GGG1100e111\n"
            "111111111111110001GGG1GGG1GGG11001111\n"
            "100000000000000000000000000000000e111\n"
            "100000000000000000000000000000000e111\n"
            "11111111111111GGG1GGG1GGG1GGG1GG11111\n"
            "1111111111111111111111111111111111111\n"
            "1111111111111111111111111111111111111\n"
            "1111111111111111111111111111111111111\n";
        grid_map_fill_from_string(map, am, MAP1);
        break;
    }
    case 2:
        map_build_hypermart(map, am);
        break;
    case 3:
        map_build_10agents_200slots(map, am);
        break;
    case 4:
        map_build_biggrid_onegoal(map, am);
        break;
    case 5:
        map_build_cross_4agents(map, am);
        break;
    default:
        map_build_hypermart(map, am);
        break;
    }
}

GridMap* grid_map_create(AgentManager* am) {
    GridMap* m = new GridMap();
    grid_map_load_scenario(m, am, 1);
    return m;
}

void grid_map_destroy(GridMap* m) {
    delete m;
}
