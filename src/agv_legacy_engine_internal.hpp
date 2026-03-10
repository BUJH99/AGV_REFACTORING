#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef DISPLAY_BUFFER_SIZE
#define DISPLAY_BUFFER_SIZE 512000
#endif

#ifndef GRID_WIDTH
#define GRID_WIDTH 82
#endif

#ifndef GRID_HEIGHT
#define GRID_HEIGHT 42
#endif

#ifndef MAX_AGENTS
#define MAX_AGENTS 16
#endif

#ifndef MAX_GOALS
#define MAX_GOALS (GRID_WIDTH * GRID_HEIGHT)
#endif

#ifndef MAX_CHARGE_STATIONS
#define MAX_CHARGE_STATIONS 10
#endif

#ifndef MAX_PHASES
#define MAX_PHASES 20
#endif

#ifndef LOG_BUFFER_LINES
#define LOG_BUFFER_LINES 5
#endif

#ifndef LOG_BUFFER_WIDTH
#define LOG_BUFFER_WIDTH 256
#endif

#ifndef MAX_WHCA_HORIZON
#define MAX_WHCA_HORIZON 11
#endif

#ifndef MIN_WHCA_HORIZON
#define MIN_WHCA_HORIZON 5
#endif

#ifndef MAX_CBS_CONS
#define MAX_CBS_CONS 128
#endif

#ifndef TEMP_MARK_MAX
#define TEMP_MARK_MAX 128
#endif

typedef enum {
    DIR_NONE = -1,
    DIR_UP = 0,
    DIR_RIGHT = 1,
    DIR_DOWN = 2,
    DIR_LEFT = 3
} AgentDir;

typedef enum {
    AGV_PHASECFG_PARK = 0,
    AGV_PHASECFG_EXIT = 1
} AgvPhaseType;

typedef enum {
    AGV_MODECFG_CUSTOM = 0,
    AGV_MODECFG_REALTIME = 1
} AgvSimulationMode;

#include "agv/legacy_sim_bridge.hpp"
#include "agv/legacy_engine_model.hpp"
