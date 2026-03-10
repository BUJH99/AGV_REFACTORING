# Compile
g++ -std=c++20 -Iinclude -DAGV_NO_MAIN src/main.cpp src/agv_legacy_engine.cpp src/simulation_engine.cpp -o agv_console.exe -lpsapi

if ($?) {
    # Run if compilation succeeded
    .\agv_console.exe
} else {
    Write-Host "Compilation failed." -ForegroundColor Red
}
