# Compile
g++ -std=c++20 -Iinclude -Isrc -DAGV_NO_MAIN `
    src/apps/console_main.cpp `
    src/core/engine_orchestrator.cpp `
    src/platform/console_terminal.cpp `
    src/planning/collision_planner.cpp `
    src/ui/simulation_display.cpp `
    src/runtime/agent_runtime.cpp `
    src/maps/map_catalog.cpp `
    src/planning/pathfinder.cpp `
    src/reporting/run_reporting.cpp `
    src/runtime/scenario_runtime.cpp `
    src/runtime/step_runtime.cpp `
    src/api/simulation_engine.cpp `
    -o agv_console.exe -lpsapi

if ($?) {
    # Run if compilation succeeded
    .\agv_console.exe
} else {
    Write-Host "Compilation failed." -ForegroundColor Red
}
