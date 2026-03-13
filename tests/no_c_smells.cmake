set(AGV_ROOT "${AGV_ROOT}")
if(NOT AGV_ROOT)
    message(FATAL_ERROR "AGV_ROOT is required")
endif()

set(FILES
    "${AGV_ROOT}/src/apps/console_main.cpp"
    "${AGV_ROOT}/src/core/engine_facades.cpp"
    "${AGV_ROOT}/src/core/engine_orchestrator.cpp"
    "${AGV_ROOT}/src/core/simulation_runtime.cpp"
    "${AGV_ROOT}/include/agv/internal/engine_internal.hpp"
    "${AGV_ROOT}/include/agv/internal/engine_model.hpp"
    "${AGV_ROOT}/include/agv/internal/text_format.hpp"
    "${AGV_ROOT}/src/api/simulation_engine.cpp"
    "${AGV_ROOT}/src/maps/map_catalog.cpp"
    "${AGV_ROOT}/src/platform/console_terminal.cpp"
    "${AGV_ROOT}/src/planning/collision_planner.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_default_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_support.hpp"
    "${AGV_ROOT}/src/planning/collision_planner_whca_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_whca_support.hpp"
    "${AGV_ROOT}/src/planning/pathfinder.cpp"
    "${AGV_ROOT}/src/reporting/run_reporting.cpp"
    "${AGV_ROOT}/src/runtime/agent_goal_support.cpp"
    "${AGV_ROOT}/src/runtime/agent_runtime.cpp"
    "${AGV_ROOT}/src/runtime/scenario_runtime.cpp"
    "${AGV_ROOT}/src/runtime/step_runtime.cpp"
    "${AGV_ROOT}/src/ui/simulation_display.cpp"
)

set(DIRECT_RUNTIME_FILES
    "${AGV_ROOT}/src/apps/console_main.cpp"
    "${AGV_ROOT}/src/core/engine_facades.cpp"
    "${AGV_ROOT}/src/core/engine_orchestrator.cpp"
    "${AGV_ROOT}/src/core/simulation_runtime.cpp"
    "${AGV_ROOT}/include/agv/internal/engine_internal.hpp"
    "${AGV_ROOT}/include/agv/internal/engine_model.hpp"
    "${AGV_ROOT}/include/agv/internal/text_format.hpp"
    "${AGV_ROOT}/src/api/simulation_engine.cpp"
    "${AGV_ROOT}/src/maps/map_catalog.cpp"
    "${AGV_ROOT}/src/planning/collision_planner.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_default_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_support.hpp"
    "${AGV_ROOT}/src/planning/collision_planner_whca_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_whca_support.hpp"
    "${AGV_ROOT}/src/planning/pathfinder.cpp"
    "${AGV_ROOT}/src/reporting/run_reporting.cpp"
    "${AGV_ROOT}/src/runtime/agent_goal_support.cpp"
    "${AGV_ROOT}/src/runtime/agent_runtime.cpp"
    "${AGV_ROOT}/src/runtime/scenario_runtime.cpp"
    "${AGV_ROOT}/src/runtime/step_runtime.cpp"
    "${AGV_ROOT}/src/ui/simulation_display.cpp"
)

set(BANNED_LITERALS
    "TRUE"
    "FALSE"
    "NULL"
    "const_cast<"
    "OwnedPtr"
    "sim_bridge"
    "typedef enum"
    "va_list"
    "printf("
    "snprintf("
    "vsnprintf("
    "fgets("
    "sscanf("
    "fputs("
    "fwrite("
    "fflush("
    "strlen("
)

set(BANNED_DIRECT_RUNTIME_LITERALS
    "_getch("
    "_kbhit("
    "Sleep("
    "std::rand("
    "std::srand("
    "srand("
    "strchr("
)

foreach(FILE_PATH IN LISTS FILES)
    if(NOT EXISTS "${FILE_PATH}")
        message(FATAL_ERROR "Missing smell-guard file: ${FILE_PATH}")
    endif()

    file(READ "${FILE_PATH}" FILE_CONTENTS)
    foreach(BANNED_LITERAL IN LISTS BANNED_LITERALS)
        string(FIND "${FILE_CONTENTS}" "${BANNED_LITERAL}" MATCH_INDEX)
        if(NOT MATCH_INDEX EQUAL -1)
            message(FATAL_ERROR "${FILE_PATH} contains banned literal: ${BANNED_LITERAL}")
        endif()
    endforeach()
endforeach()

foreach(FILE_PATH IN LISTS DIRECT_RUNTIME_FILES)
    if(NOT EXISTS "${FILE_PATH}")
        message(FATAL_ERROR "Missing runtime-smell-guard file: ${FILE_PATH}")
    endif()

    file(READ "${FILE_PATH}" FILE_CONTENTS)
    foreach(BANNED_LITERAL IN LISTS BANNED_DIRECT_RUNTIME_LITERALS)
        string(FIND "${FILE_CONTENTS}" "${BANNED_LITERAL}" MATCH_INDEX)
        if(NOT MATCH_INDEX EQUAL -1)
            message(FATAL_ERROR "${FILE_PATH} contains banned direct runtime literal: ${BANNED_LITERAL}")
        endif()
    endforeach()
endforeach()
