---
description: Build and Run C++ Project
---

1. Compile the C++ files using g++
// turbo
g++ -std=c++20 -Iinclude -DAGV_NO_MAIN src/main.cpp src/agv_legacy_engine.cpp src/agv_legacy_console.cpp src/agv_legacy_display.cpp src/agv_legacy_maps.cpp src/agv_legacy_reports.cpp src/simulation_engine.cpp -o agv_console.exe -lpsapi

// turbo
2. Run the compiled executable
.\agv_console.exe
