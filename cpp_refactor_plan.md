# C++ Refactor Plan

## Goal

Refactor the AGV C++ codebase for real logic and structure improvement while preserving the existing planning algorithms and runtime behavior. The focus is on:

- separating responsibilities in setup/run/config paths
- simplifying control flow and state transitions
- reducing repeated allocation and unnecessary work in hot paths
- improving ownership clarity and modern C++ usage where it has practical value
- making the execution flow easier to understand and extend

## Current Structure Diagnosis

### Core execution flow

1. [`src/apps/console_main.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/apps/console_main.cpp)
   - parses CLI arguments
   - chooses interactive vs headless execution
   - configures `SimulationEngine`

2. [`src/api/simulation_engine.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/api/simulation_engine.cpp)
   - wraps the legacy bridge API
   - owns the `Simulation*`
   - rebuilds simulation when config changes
   - exposes snapshots and run APIs

3. [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp)
   - interactive scenario setup
   - default config initialization
   - applying config into runtime state
   - task dispatch logic

4. [`src/core/engine_orchestrator.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/core/engine_orchestrator.cpp)
   - simulation lifecycle
   - per-step execution orchestration
   - completion logic
   - realtime dashboard

5. Hot logic modules
   - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
   - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
   - [`src/runtime/step_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/step_runtime.cpp)
   - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)

### Major coupling / responsibility problems

- [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp)
  mixes input parsing, interactive menu rendering, scenario reset, config translation, and dispatch policy.
- [`src/api/simulation_engine.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/api/simulation_engine.cpp)
  mixes ownership, config translation, rebuild policy, and snapshot conversion.
- [`src/core/engine_orchestrator.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/core/engine_orchestrator.cpp)
  contains multiple lifecycle responsibilities and several stateful helpers that are hard to test in isolation.
- [`src/apps/console_main.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/apps/console_main.cpp)
  has a long argument parser with repeated mode toggling and value extraction logic.

### Data flow and state issues

- Scenario state reset is spread across manual field assignments.
- Scenario phase setup duplicates phase-type translation and name assignment logic.
- `SimulationEngine::snapshotFrame()` allocates a fresh large buffer every call.
- Interactive setup and headless config application follow separate but partially overlapping initialization paths.
- The bridge still relies heavily on primitive fields instead of cohesive helper operations.

### Bottleneck candidates

- [`src/api/simulation_engine.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/api/simulation_engine.cpp): repeated render buffer allocation in `snapshotFrame()`
- [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp): per-frame buffer build and flush
- [`src/core/engine_orchestrator.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/core/engine_orchestrator.cpp): hot per-step orchestration and dashboard logic
- [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp): repeated queue scans in `tryDequeueAssignableTask()`

## Core Problems

1. Configuration application is monolithic and stateful.
2. Interactive setup uses duplicated branch-heavy menu logic.
3. Snapshot and conversion paths do unnecessary allocation/copy work.
4. Several functions expose low-level field mutation instead of cohesive operations.
5. Ownership and lifecycle are mostly correct but not explicit enough in wrapper code.

## Prioritized Work Items

### P0. Refactor scenario/config setup flow

Targets:

- [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp)
- [`include/agv/internal/engine_internal.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/include/agv/internal/engine_internal.hpp)

Planned changes:

- extract scenario reset into dedicated helper(s)
- extract custom vs realtime config application into focused helpers
- centralize phase translation and phase-name assignment
- reduce branch nesting inside `simulation_setup()` and `agv_apply_config()`
- use standard library helpers such as `std::clamp` and `std::array` where useful

Expected effect:

- clearer state transitions
- less duplicated mutation logic
- easier future testing of setup/config behavior

### P1. Refactor SimulationEngine lifecycle and snapshot path

Targets:

- [`src/api/simulation_engine.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/api/simulation_engine.cpp)
- [`include/agv/simulation_engine.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/include/agv/simulation_engine.hpp)

Planned changes:

- extract simulation creation/rebuild helpers in `Impl`
- reuse render buffer instead of allocating a large temporary vector per frame
- isolate legacy summary -> `MetricsSnapshot` conversion
- make ownership and “dirty rebuild” flow easier to read

Expected effect:

- reduced heap churn in frame capture
- better ownership clarity
- simpler wrapper maintenance

### P2. Simplify console option parsing

Targets:

- [`src/apps/console_main.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/apps/console_main.cpp)

Planned changes:

- consolidate next-value parsing helpers
- reduce repeated `options.interactive = false`
- clarify headless option semantics

Expected effect:

- less repetitive control flow
- easier extension of CLI behavior

### P3. Review hot runtime/control-loop helpers for targeted cleanup

Targets:

- [`src/core/engine_orchestrator.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/core/engine_orchestrator.cpp)
- [`src/runtime/step_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/step_runtime.cpp)

Planned changes:

- extract small helper functions around run loop / completion checks if a safe seam is available
- avoid repeated work where behavior is preserved

Expected effect:

- improved readability in the runtime loop
- smaller risk surface for future optimization work

## Performance Improvement Points

- reuse the render capture buffer in `SimulationEngine::snapshotFrame()`
- avoid repeated initialization work by extracting reusable config helpers
- remove redundant writes/branches in scenario reset paths
- keep an eye on repeated queue scans in `ScenarioManager::tryDequeueAssignableTask()` for a later pass

## Standard Library / Modern C++ Candidates

- `std::clamp` for bounded config values
- `std::array` / helper tables for menu/config mappings
- `std::string_view` for fixed string labels and parsing helpers where useful
- stronger `const` usage in helpers that do not mutate state
- move long field-mutation blocks behind functions with explicit names

## File / Responsibility Separation Strategy

- keep algorithm implementation files intact in spirit
- refactor around setup, lifecycle, and bridge boundaries first
- prefer extracting cohesive static helpers before introducing new classes
- only split files if a clean seam emerges without destabilizing the project

## Risks and Regression Possibilities

- interactive setup behavior could change if menu/control flow is altered carelessly
- scenario reset mistakes could affect completion or dispatch behavior
- wrapper lifecycle refactors could break lazy rebuild semantics
- Windows-specific build/runtime verification is limited in the current environment

## Validation Method

- reference scans with `rg` after each refactor to ensure obsolete paths are removed
- focused code review of changed control flow and state mutation order
- static inspection of call sites and ownership transitions
- build/test only where available; otherwise document the verification gap explicitly

## Current Verification Result

- reference scans confirm the removed report/output APIs are no longer referenced from the refactored paths
- setup/config helpers in [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp) are now the only mutation points for scenario reset and mode-specific config application
- `SimulationEngine` frame capture no longer allocates a fresh render buffer per call
- Windows-specific compile verification is still pending because the current environment does not provide the project’s Windows `g++` toolchain

## Remaining Hotspots / Recommended Follow-up

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  still contains the largest concentration of algorithm-adjacent logic and should be split by reservation/CBS/WHCA responsibilities without changing the chosen algorithms
- [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
  remains a candidate for render buffer / formatting decomposition and potential flush throttling review
- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
  still mixes state transitions and metrics accumulation and would benefit from smaller per-state handlers

## Progress Checklist

- [x] Analyze execution flow and identify high-value refactor targets
- [x] Refactor scenario/config setup flow
- [x] Refactor SimulationEngine lifecycle and frame snapshot allocation
- [x] Simplify console parsing path
- [x] Run reference scans / static verification and update results
- [x] Summarize remaining hotspots and next refactor candidates
