cmake_minimum_required(VERSION 3.24)

project(agv_refactor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)

add_library(agv_core
    include/agv/legacy_engine_model.hpp
    src/agv_legacy_engine.cpp
    src/simulation_engine.cpp
)

set_source_files_properties(src/agv_legacy_engine.cpp PROPERTIES COMPILE_DEFINITIONS AGV_NO_MAIN)

target_include_directories(agv_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(agv_core PUBLIC psapi)

if(MSVC)
    target_compile_options(agv_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(agv_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(agv_console src/main.cpp)
target_link_libraries(agv_console PRIVATE agv_core)

if(BUILD_TESTING)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(agv_core_tests tests/simulation_engine_test.cpp)
    target_link_libraries(agv_core_tests PRIVATE agv_core GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(agv_core_tests)
endif()
