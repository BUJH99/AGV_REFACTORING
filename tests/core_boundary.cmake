set(AGV_ROOT "${AGV_ROOT}")
if(NOT AGV_ROOT)
    message(FATAL_ERROR "AGV_ROOT is required")
endif()

set(CORE_FILES
    "${AGV_ROOT}/src/api/simulation_engine.cpp"
    "${AGV_ROOT}/src/core/engine_facades.cpp"
    "${AGV_ROOT}/src/core/engine_orchestrator.cpp"
    "${AGV_ROOT}/src/core/simulation_runtime.cpp"
    "${AGV_ROOT}/src/maps/map_catalog.cpp"
    "${AGV_ROOT}/src/planning/collision_planner.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_default_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_support.cpp"
    "${AGV_ROOT}/src/planning/collision_planner_whca_support.cpp"
    "${AGV_ROOT}/src/planning/pathfinder.cpp"
    "${AGV_ROOT}/src/reporting/debug_support.cpp"
    "${AGV_ROOT}/src/reporting/run_reporting.cpp"
    "${AGV_ROOT}/src/runtime/agent_goal_support.cpp"
    "${AGV_ROOT}/src/runtime/agent_runtime.cpp"
    "${AGV_ROOT}/src/runtime/scenario_runtime.cpp"
    "${AGV_ROOT}/src/runtime/step_runtime.cpp"
    "${AGV_ROOT}/src/ui/render_model.cpp"
)

set(BANNED_INCLUDES
    "agv/console_shell.hpp"
    "agv/render_ipc_server.hpp"
    "agv/internal/console_launch_wizard.hpp"
    "agv/internal/launch_ui_metadata.hpp"
)

foreach(FILE_PATH IN LISTS CORE_FILES)
    if(NOT EXISTS "${FILE_PATH}")
        message(FATAL_ERROR "Missing core-boundary file: ${FILE_PATH}")
    endif()

    file(READ "${FILE_PATH}" FILE_CONTENTS)
    foreach(BANNED_INCLUDE IN LISTS BANNED_INCLUDES)
        string(FIND "${FILE_CONTENTS}" "${BANNED_INCLUDE}" MATCH_INDEX)
        if(NOT MATCH_INDEX EQUAL -1)
            message(FATAL_ERROR "${FILE_PATH} includes shell-only header: ${BANNED_INCLUDE}")
        endif()
    endforeach()
endforeach()
