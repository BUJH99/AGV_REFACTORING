cmake_minimum_required(VERSION 3.24)

project(agv_refactor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(nlohmann_json)

file(GLOB_RECURSE CORE_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
file(GLOB_RECURSE ALL_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

set(CONSOLE_SHELL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/platform/console_terminal.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/console_launch_wizard.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/launch_ui_metadata.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ui/simulation_display.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/shell/console_shell.cpp"
)

set(IPC_SHELL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/api/render_ipc_server.cpp"
)

set(APP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/console_main.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/render_ipc_main.cpp"
)

set(CORE_SOURCES ${ALL_SOURCES})
list(REMOVE_ITEM CORE_SOURCES ${CONSOLE_SHELL_SOURCES})
list(REMOVE_ITEM CORE_SOURCES ${IPC_SHELL_SOURCES})
list(REMOVE_ITEM CORE_SOURCES ${APP_SOURCES})

add_library(agv_engine_core
    ${CORE_HEADERS}
    ${CORE_SOURCES}
)

set_source_files_properties(src/core/engine_orchestrator.cpp PROPERTIES COMPILE_DEFINITIONS AGV_NO_MAIN)

target_include_directories(agv_engine_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(agv_engine_core PUBLIC psapi)

if(MSVC)
    target_compile_options(agv_engine_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(agv_engine_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(agv_console_shell
    ${CONSOLE_SHELL_SOURCES}
)
target_include_directories(agv_console_shell
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(agv_console_shell
    PUBLIC
        agv_engine_core
        nlohmann_json::nlohmann_json
        shell32
        ole32
)
if(MSVC)
    target_compile_options(agv_console_shell PRIVATE /W4 /permissive-)
else()
    target_compile_options(agv_console_shell PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(agv_ipc_shell
    ${IPC_SHELL_SOURCES}
)
target_include_directories(agv_ipc_shell
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(agv_ipc_shell
    PUBLIC
        agv_engine_core
        nlohmann_json::nlohmann_json
)
if(MSVC)
    target_compile_options(agv_ipc_shell PRIVATE /W4 /permissive-)
else()
    target_compile_options(agv_ipc_shell PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(agv_console src/apps/console_main.cpp)
target_link_libraries(agv_console PRIVATE agv_console_shell)

add_executable(agv_render_ipc src/apps/render_ipc_main.cpp)
target_link_libraries(agv_render_ipc PRIVATE agv_ipc_shell)

if(BUILD_TESTING)
    add_test(
        NAME agv_no_c_smells
        COMMAND ${CMAKE_COMMAND}
            -DAGV_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/no_c_smells.cmake
    )
    add_test(
        NAME agv_core_boundary
        COMMAND ${CMAKE_COMMAND}
            -DAGV_ROOT=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/core_boundary.cmake
    )
endif()

if(BUILD_TESTING)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(agv_core_tests
        tests/simulation_engine_test.cpp
        tests/debug_support_test.cpp
        tests/render_model_test.cpp
        tests/launch_workflow_test.cpp
    )
    target_link_libraries(
        agv_core_tests
        PRIVATE
            agv_engine_core
            agv_console_shell
            agv_ipc_shell
            GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(agv_core_tests)
endif()
