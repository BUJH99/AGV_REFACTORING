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
- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  mixes ordered planning helpers, temporary obstacle management, priority ordering, CBS support logic, and WHCA reservation flow in one translation unit.

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

### P4. Split ordered-planning support out of collision planner

Targets:

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- [`src/planning/collision_planner_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.hpp)
- [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)

Planned changes:

- move priority ordering helpers out of `collision_planner.cpp`
- replace manual temp-mark cleanup with RAII scoped cleanup
- isolate ordered A*/D* single-step planning helper calls
- keep CBS / WHCA core logic in place for now

Expected effect:

- smaller hot file with clearer responsibility boundaries
- fewer manual cleanup paths and lower leak / stale-temp-mark risk
- easier next pass on CBS and reservation-table logic

## Performance Improvement Points

- reuse the render capture buffer in `SimulationEngine::snapshotFrame()`
- avoid repeated initialization work by extracting reusable config helpers
- remove redundant writes/branches in scenario reset paths
- reduce manual temp-obstacle cleanup churn by using RAII in ordered planning paths
- keep an eye on repeated queue scans in `ScenarioManager::tryDequeueAssignableTask()` for a later pass

## Standard Library / Modern C++ Candidates

- `std::clamp` for bounded config values
- `std::array` / helper tables for menu/config mappings
- `std::string_view` for fixed string labels and parsing helpers where useful
- stronger `const` usage in helpers that do not mutate state
- move long field-mutation blocks behind functions with explicit names

## OOP / SOLID / DDD Lens

### OOP / SOLID target

- SRP:
  planner setup, display composition, and runtime goal/state policy should each have one obvious owner instead of being scattered across multiple free-function chains
- OCP:
  algorithm choice, renderer choice, and goal-assignment policy should be extendable by adding a new strategy/policy object rather than rewriting long conditional paths
- LSP:
  strategy objects should stay thin and behavior-focused so planner/renderer substitutions remain safe
- ISP:
  helper interfaces should prefer small context objects or focused methods instead of long parameter lists that expose unrelated dependencies
- DIP:
  high-level orchestration should talk to planner/renderer/policy abstractions, while low-level implementation details stay behind those seams

### DDD fit / boundary goal

- the project is not being converted into full enterprise-style DDD, but we do want domain concepts to be explicit
- target domain concepts:
  - `GoalAssignment`
  - `PlannerStep`
  - `DisplayFrame`
  - `ScenarioPhase`
  - `ChargingCycle`
- preferred direction:
  orchestration code should read in terms of these domain actions/policies instead of raw field mutation and mixed infrastructure concerns

### Remaining OOP / SOLID / DDD gaps

- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp):
  still has more state-transition and task-completion logic than ideal in one translation unit
- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp):
  planner fallback policy combinations are clearer than before but still not fully expressed as dedicated domain/policy objects
- [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp):
  section composition is cleaner, but the file still bundles rendering policy and low-level formatting details together
- test coverage:
  the code is structurally cleaner than before, but the DDD/OOP direction will not feel “finished” until policy-level tests exist

## File / Responsibility Separation Strategy

- keep algorithm implementation files intact in spirit
- refactor around setup, lifecycle, and bridge boundaries first
- prefer extracting cohesive static helpers before introducing new classes
- split planning support code only when the seam is operationally clear, such as ordered-planning support vs CBS / WHCA core

## Risks and Regression Possibilities

- interactive setup behavior could change if menu/control flow is altered carelessly
- scenario reset mistakes could affect completion or dispatch behavior
- wrapper lifecycle refactors could break lazy rebuild semantics
- Windows-specific build/runtime verification is limited in the current environment

## Validation Method

- reference scans with `rg` after each refactor to ensure obsolete paths are removed
- focused code review of changed control flow and state mutation order
- static inspection of call sites and ownership transitions
- build and headless runtime verification with the project’s Windows `g++` toolchain when the change affects runtime behavior

## Current Verification Result

- reference scans confirm the removed report/output APIs are no longer referenced from the refactored paths
- setup/config helpers in [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp) are now the only mutation points for scenario reset and mode-specific config application
- `SimulationEngine` frame capture no longer allocates a fresh render buffer per call
- ordered planning support in [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) now delegates priority sorting, temp obstacle scoping, ordered pathfinder moves, and conflict resolution to [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)
- reservation table handling, WHCA support, and partial CBS support now delegate from [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) into [`src/planning/collision_planner_whca_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.cpp)
- the remaining WHCA planner orchestration inside [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) is now decomposed into per-agent planning, immediate conflict recording, group collection, CBS solution application, and pairwise first-step conflict resolution helpers
- rendering in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) now separates frame-context construction, display-buffer building, header/body composition, and flush gating
- layout assembly in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) now uses a typed `DisplayBufferWriter` instead of propagating raw `char**` / `size_t*` cursor state through helper functions
- map construction helpers in [`src/maps/map_catalog.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/maps/map_catalog.cpp) now centralize interior fill, row/column opening, goal/charge opening, and goal-index rebuild behavior
- agent goal completion flow in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) now routes through state-specific helpers instead of one monolithic completion switch
- goal selection and charging-transition policy in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) now route through dedicated helper functions instead of a single branch-heavy goal assignment function
- charge-state maintenance in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) now routes through timer-tick and completion helpers instead of a monolithic loop body
- Windows `g++ 15.2.0` compile verification now passes in the current environment
- headless runtime verification passes with unchanged smoke-test results:
  - `--algo default` -> `steps=22 tasks=1 movement=8 deadlocks=11`
  - `--algo 2` -> `steps=20 tasks=1 movement=8 deadlocks=10`

## Remaining Hotspots / Recommended Follow-up

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  still contains the largest concentration of algorithm-adjacent logic; ordered-planning support, WHCA/CBS support, and orchestration helpers are now extracted, but the file still owns the top-level planner flow and strategy wiring
- [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
  is cleaner in both frame building and buffer writing paths, but rendering policy and layout composition are still concentrated in one file
- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
  now has smaller completion handlers, separated goal-selection policy helpers, and charge-state maintenance helpers, but some temporary path-cost selection logic and overall file size remain
- [`src/maps/map_catalog.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/maps/map_catalog.cpp)
  is now cleaner in its low-level map-carving helpers, but scenario-specific layout builders still deserve a higher-level data-driven pass later

## Progress Checklist

- [x] Analyze execution flow and identify high-value refactor targets
- [x] Refactor scenario/config setup flow
- [x] Refactor SimulationEngine lifecycle and frame snapshot allocation
- [x] Simplify console parsing path
- [x] Extract ordered-planning support helpers from collision planner
- [x] Extract WHCA / reservation / partial CBS support helpers from collision planner
- [x] Consolidate repeated map-building operations in map catalog helpers
- [x] Split agent goal completion flow into state-specific helpers
- [x] Verify refactored code with Windows `g++` build and headless execution
- [x] Run reference scans / static verification and update results
- [x] Summarize remaining hotspots and next refactor candidates

## Iteration Log

### 2026-03-11 - Ordered planning support extraction

- Current task:
  extract ordered-planning helper responsibilities from [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Selection reason:
  this was the safest high-impact seam inside the largest remaining hotspot because it reduced file size and cleanup complexity without altering the planning algorithms
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`src/planning/collision_planner_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.hpp)
  - [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)
- Change summary:
  moved priority ordering, ordered pathfinder stepping, temp obstacle marking, and ordered conflict resolution into a dedicated support module; replaced manual temp-mark cleanup with the RAII class `TempObstacleScope`
- C++ improvement points:
  - RAII for temp obstacle cleanup
  - clearer ownership boundaries by moving support behavior behind explicit interfaces
  - lower side-effect surface inside the main planner translation unit
- Performance impact:
  neutral to slightly positive; the main gain is fewer manual cleanup branches in the hot ordered-planning path
- Readability impact:
  high; `collision_planner.cpp` now keeps more algorithm-core code and less support boilerplate
- Risk:
  medium; path planning support functions are hot-path code and small ordering changes could affect runtime behavior
- Verification result:
  Windows `g++ 15.2.0` build passed and headless smoke tests matched previous results for `--algo default` and `--algo 2`
- Next task:
  split CBS / reservation / WHCA responsibilities inside [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) or move to [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) if planner risk needs to be reduced first

### 2026-03-11 - Map catalog helper consolidation

- Current task:
  consolidate repeated map-building operations in [`src/maps/map_catalog.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/maps/map_catalog.cpp)
- Selection reason:
  this file had repeated low-level grid mutations across multiple scenarios, making layout intent harder to read and increasing the chance of inconsistent goal/charge handling
- Changed files:
  - [`src/maps/map_catalog.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/maps/map_catalog.cpp)
- Change summary:
  introduced shared helpers for filling the interior, opening rows/columns/cells, opening goal and charge cells, clearing goal regions, rebuilding the goal index, and validating agent placement coordinates
- C++ improvement points:
  - stronger error handling in `map_place_agent_at()`
  - shared helper boundaries instead of repeated primitive field mutations
  - targeted standard library use with `std::max` / `std::min` for clamped helper ranges
- Performance impact:
  neutral; the primary gain is consistency and lower maintenance cost, not runtime speed
- Readability impact:
  medium to high; each scenario builder now reads more like a layout recipe and less like a long series of raw cell mutations
- Risk:
  low to medium; changes affect map initialization paths but not planner algorithms
- Verification result:
  Windows `g++ 15.2.0` build passed and the same headless smoke tests still produced `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  revisit [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) for state-handler extraction or return to [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) to split CBS / reservation responsibilities

### 2026-03-11 - WHCA / CBS support extraction

- Current task:
  extract reservation-table, WHCA support, pull-over, and partial CBS helpers from [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Selection reason:
  after ordered-planning support extraction, this was the next largest cohesive helper cluster and a high-value way to shrink planner responsibilities without altering algorithms
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`src/planning/collision_planner_whca_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.hpp)
  - [`src/planning/collision_planner_whca_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.cpp)
- Change summary:
  moved reservation table operations, SCC detection from wait edges, WHCA horizon adjustment, pull-over fallback, constrained single-agent planning, and partial CBS search into a dedicated support module
- C++ improvement points:
  - clearer file ownership boundaries for planner support code
  - better const correctness in CBS conflict detection inputs
  - lower translation-unit complexity in the main planner file
- Performance impact:
  neutral; the main benefit is maintainability and safer future optimization work
- Readability impact:
  high; planner orchestration is easier to follow because CBS / WHCA support details are no longer mixed into the same file
- Risk:
  medium; this touches hot-path planner support and fallback behavior
- Verification result:
  Windows `g++ 15.2.0` build passed after a const-correctness fix, and the headless smoke tests remained unchanged
- Next task:
  split the remaining orchestration/fallback block in [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) or continue reducing state-transition coupling in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)

### 2026-03-11 - Agent completion handler split

- Current task:
  split goal-completion state transitions in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Selection reason:
  `updateStateAfterMove()` was still mixing arrival gating, reservation cleanup, task accounting, logging, and state transitions in one long switch
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Change summary:
  extracted helpers for task completion accounting, action-timer advancement, reservation clearing, and each goal-completion transition so `updateStateAfterMove()` now orchestrates instead of owning all state mutation directly
- C++ improvement points:
  - smaller single-purpose helpers
  - clearer side-effect boundaries
  - less duplicated task-completion bookkeeping
- Performance impact:
  neutral; this is a structure/maintainability refactor
- Readability impact:
  medium to high; state transitions are easier to review independently
- Risk:
  low to medium; changes affect core arrival handling but remain within existing state semantics
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still matched prior outputs
- Next task:
  separate goal-selection policy from charging/home-return policy in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) or return to UI/rendering cleanup in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)

### 2026-03-11 - Planner orchestration helper split

- Current task:
  decompose the remaining WHCA planner orchestration inside [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Selection reason:
  after moving support modules out, the main planner function was still too long and mixed per-agent path planning, fallback grouping, CBS application, and final pairwise conflict resolution
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Change summary:
  extracted helpers for active-agent classification, waiting-agent reservation, per-agent WHCA planning, first-step conflict recording, SCC/fallback group collection, CBS solution application, pull-over fallback, deadlock detection, and pairwise first-step conflict resolution
- C++ improvement points:
  - smaller helpers with explicit responsibility
  - less repeated control flow and fewer inline side effects
  - clearer top-level planner call order
- Performance impact:
  neutral; this is primarily a readability and maintainability improvement
- Readability impact:
  high; the main planner function now reads as orchestration steps instead of a single monolithic block
- Risk:
  medium; helper extraction touches the main planning path, but the smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  move to [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) for rendering-path cleanup or continue splitting goal-selection policy inside [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)

### 2026-03-11 - Rendering path decomposition

- Current task:
  decompose the frame-building and output path in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
- Selection reason:
  the renderer still mixed frame-context calculation, string assembly, flush gating, and output concerns in one function
- Changed files:
  - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
- Change summary:
  introduced `DisplayFrameContext`, separated status-header dispatch, display-buffer building, buffer finalization, and render-stride handling so `simulation_display_status()` now acts as a short orchestration function
- C++ improvement points:
  - clearer data flow with an explicit frame context
  - stronger ownership of display-buffer finalization
  - simpler top-level rendering control flow
- Performance impact:
  neutral to slightly positive; render-stride handling and buffer finalization are clearer, though this is mainly a maintainability improvement
- Readability impact:
  medium to high; the renderer now reads as context creation -> buffer build -> conditional flush
- Risk:
  low to medium; display rendering changed structurally but smoke tests and build still passed
- Verification result:
  Windows `g++ 15.2.0` build passed and headless smoke tests remained unchanged
- Next task:
  continue with goal-selection policy separation in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) or split remaining rendering layout responsibilities into smaller components if UI maintenance is the priority

### 2026-03-11 - Goal selection policy split

- Current task:
  split goal-selection and charging-transition policy inside [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Selection reason:
  `agent_set_goal_local()` still mixed charge-mode switching, state-based goal resolution, reservation ownership, and missing-goal fallback in one branch-heavy function
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Change summary:
  introduced `GoalTypeLocal` as `enum class`, a `GoalSelectionPolicyLocal` struct, and helpers for policy construction, charge-mode switching, state-based goal resolution, reservation assignment, and missing-goal handling
- C++ improvement points:
  - stronger typing with `enum class`
  - clearer policy data via a dedicated struct
  - reduced side effects in the top-level goal-assignment path
- Performance impact:
  neutral; this is primarily a structure and maintainability improvement
- Readability impact:
  medium to high; the goal-assignment flow now reads as charge transition -> resolve -> reserve -> fallback
- Risk:
  low to medium; this affects planning setup decisions, but smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the same headless smoke tests remained unchanged
- Next task:
  split charge-state maintenance from charging-completion side effects in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) or continue decomposing rendering layout composition in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)

### 2026-03-11 - Charge-state maintenance split

- Current task:
  split charge-state maintenance inside [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Selection reason:
  `updateChargeState()` still mixed timer ticking, reservation clearing, broadcast updates, path reset, and state transition side effects inside one loop body
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Change summary:
  extracted helpers for ticking the charge timer, clearing position reservation, resetting goal/path state, and finalizing the charging cycle so the update loop now reads as tick -> complete
- C++ improvement points:
  - smaller single-purpose helpers
  - lower side-effect density in the update loop
  - clearer naming of charge-cycle responsibilities
- Performance impact:
  neutral; this is a structure/maintainability improvement
- Readability impact:
  medium; charge-state flow is now easier to audit
- Risk:
  low; the behavior remained stable under the same smoke tests
- Verification result:
  Windows `g++ 15.2.0` build passed and headless smoke tests remained unchanged
- Next task:
  target the remaining high-complexity utility flow in temporary path-cost evaluation or continue splitting UI layout composition in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)

### 2026-03-11 - Display buffer writer migration

- Current task:
  replace raw cursor/remaining-pointer plumbing in [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) with a typed writer object
- Selection reason:
  even after buffer-build decomposition, most renderer helpers still shared C-style `char**` / `size_t*` state and a formatting macro, which kept the layout path harder to reason about and more error-prone
- Changed files:
  - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
- Change summary:
  introduced `DisplayBufferWriter`, removed the formatting macro from the rendering path, and migrated the layout/header/grid/log/control helpers to operate on the writer object instead of raw cursor state
- C++ improvement points:
  - stronger encapsulation for buffer mutation
  - fewer raw mutable pointers flowing through helper signatures
  - more explicit finish/finalization behavior on the display buffer
- Performance impact:
  neutral; the goal is safer and clearer buffer assembly, not algorithmic speedup
- Readability impact:
  high; helper signatures are simpler and the render path is more obviously stateful in one place
- Risk:
  low to medium; this changes the rendering write path structure, but build and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed and headless smoke tests remained unchanged
- Next task:
  decide between extracting temporary path-cost evaluation from [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) or breaking rendering layout composition into even smaller domain-specific sections if UI maintenance remains the priority

### 2026-03-11 - Goal candidate evaluator cleanup

- Current task:
  simplify temporary path-cost evaluation and goal candidate selection in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Selection reason:
  the file still mixed raw temporary pathfinder lifetime, candidate filtering rules, temporary parked-state mutation, and logging in loosely connected helper functions
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Change summary:
  replaced raw temporary pathfinder management with an RAII-backed `PathCostEvaluatorLocal`, introduced `GoalCandidateSetLocal` using `std::span<Node*>`, added `TemporaryParkStateScopeLocal` for automatic parked-state restoration, and folded repeated logging back into the shared goal-selection path; also fixed the null-agent branch in `agent_set_goal_local()`
- C++ improvement points:
  - RAII for temporary pathfinder lifetime and temporary parked-state mutation
  - `std::span` for candidate-set views instead of raw pointer/count plumbing
  - lower side-effect density in the goal-evaluation path
  - safer null handling in goal assignment
- Performance impact:
  neutral to slightly positive; the algorithm is unchanged, but temporary-resource handling is tighter and candidate iteration is more explicit
- Readability impact:
  high; goal evaluation now reads as candidate-set construction -> filtering -> scoped evaluation -> selection
- Risk:
  low to medium; this changes a frequently used runtime path, but the same build and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  either continue decomposing [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) into layout-specific sections or reduce remaining strategy wiring in [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)

### 2026-03-11 - Ordered planner core unification

- Current task:
  remove duplicated ordered-planning loops in [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Selection reason:
  the A* and D* ordered planner entry points still duplicated almost the entire per-agent planning loop, which made later behavior changes riskier and harder to review
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- Change summary:
  introduced `run_ordered_planning_core_local()` and `agent_should_skip_ordered_planning_local()` so the A* and D* modes now share one ordered-planning loop and differ only by the metric kind passed into `compute_ordered_pathfinder_move()`
- C++ improvement points:
  - lower branch duplication in strategy entry points
  - clearer separation between shared orchestration and per-mode metric choice
  - smaller wrappers with explicit intent
- Performance impact:
  neutral; the planning algorithm is unchanged, but duplicated code paths are gone and the hot loop is easier to maintain safely
- Readability impact:
  high; ordered planning now has one authoritative loop instead of two near-identical implementations
- Risk:
  medium; this touches the planner hot path, but build and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  continue with [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp) section-level layout decomposition or further reduce top-level strategy wiring in [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)

### 2026-03-11 - Planner/display/runtime final cleanup pass

- Current task:
  close the biggest remaining readability gaps across planner wiring, display section composition, and runtime state-policy mapping
- Selection reason:
  after the earlier refactors, the codebase still had three obvious cleanup points blocking a near-finished structure: duplicated planner entry wrappers, one long display-buffer assembly sequence, and state-to-goal policy branching spread across runtime helpers
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
  - [`cpp_refactor_plan.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_plan.md)
  - [`cpp_refactor_report_ko.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_report_ko.md)
- Change summary:
  unified planner entry setup with `run_planner_entry_local()` and replaced multiple planner strategy classes with a single function-backed strategy, split renderer buffer assembly into explicit section appenders plus a section list, switched the renderer facade to the same function-backed strategy style, and mapped runtime states to goal types with `std::optional<GoalTypeLocal>` plus shared state-group helpers
- C++ improvement points:
  - less boilerplate strategy code through function-backed strategy objects
  - clearer orchestration/data flow via explicit display sections
  - `std::optional` for state-to-goal mapping instead of branch-heavy ad hoc control flow
  - smaller and more coherent top-level helpers in planner, renderer, and runtime code
- Performance impact:
  mostly neutral; the main gain is structural clarity and lower maintenance risk, though the reduced wrapper duplication also lowers the chance of divergent hot-path edits
- Readability impact:
  high; top-level planner and renderer setup paths now read more like pipelines than scattered helper calls
- Risk:
  medium; the work touched core runtime/planning/rendering entry paths, but repeated Windows builds and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed after the changes, and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  remaining work is now mostly “last-mile” refinement such as adding more targeted tests, profiling hot paths, or shrinking a few still-large files rather than fixing major structural tangles

### 2026-03-11 - More aggressive C++ surface cleanup

- Current task:
  replace remaining C-style implementation surfaces in planner support, WHCA scratch handling, display buffers, and goal-assignment plumbing with stronger C++ structures
- Selection reason:
  after the previous structural cleanup, several hot-path implementations still exposed manual sorting, global raw buffers, or repeated multi-parameter helper signatures that made the code feel more “legacy C in C++” than necessary
- Changed files:
  - [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)
  - [`src/planning/collision_planner_whca_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.cpp)
  - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Change summary:
  migrated planner candidate sorting and agent-order sorting to `std::array` plus standard algorithms, wrapped WHCA search scratch buffers in a typed `SpaceTimeSearchBuffers` object, moved display color/grid/section tables to `std::array`-backed types, and introduced `GoalAssignmentContextLocal` so goal-selection helpers pass a cohesive context object instead of repeatedly threading raw parameters
- C++ improvement points:
  - stronger container semantics with `std::array`
  - standard-library sorting and initialization instead of manual swap loops
  - typed scratch-buffer ownership instead of loose global raw arrays
  - cleaner helper interfaces through an explicit goal-assignment context object
- Performance impact:
  neutral to slightly positive; the algorithms are unchanged, but scratch-buffer ownership and local ordering logic are clearer and no extra dynamic allocation was introduced
- Readability impact:
  high; several remaining low-level C-style surfaces now look and behave more like idiomatic C++ implementation code
- Risk:
  medium; this touched planner/runtime/renderer internals, but repeated Windows builds and smoke tests stayed stable
- Verification result:
  Windows `g++ 15.2.0` build passed after the changes, and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  the main remaining gap to a “10/10” state is no longer large structural confusion, but rather the lack of deeper targeted tests and the still-large size of a few files

### 2026-03-11 - Goal assignment policy object pass

- Current task:
  make runtime goal assignment more explicitly object/policy-oriented in [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
- Selection reason:
  even after introducing a context object, the goal-assignment path still spread policy behavior across several free functions; this was a good candidate to move closer to SRP/OCP-style object boundaries
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
  - [`cpp_refactor_plan.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_plan.md)
  - [`cpp_refactor_report_ko.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_report_ko.md)
- Change summary:
  introduced `GoalAssignmentPolicyLocal` to own goal resolution, reservation assignment, missing-goal handling, and charge-station selection around a shared `GoalAssignmentContextLocal`
- C++ improvement points:
  - stronger object boundary for a runtime domain policy
  - smaller free-function surface
  - clearer alignment with SRP and ISP through a cohesive context + policy object
- Performance impact:
  neutral; behavior is unchanged and no new dynamic allocation was introduced
- Readability impact:
  medium to high; `agent_set_goal_local()` now reads more like orchestration over a domain policy object
- Risk:
  medium; this touches a central runtime path, but build and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed after the changes, and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  continue applying the same OOP/SOLID lens to planner fallback policy or renderer composition ownership, and add more targeted tests to support a genuine “10/10” assessment

### 2026-03-11 - Planner fallback policy and public-api smoke coverage

- Current task:
  move planner fallback combination logic behind a clearer policy object and add another public-API test seam
- Selection reason:
  the remaining planner complexity was concentrated in fallback decision wiring, and the codebase still needed at least one more test that validates algorithm switching through the public `SimulationEngine` API
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`tests/simulation_engine_test.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/tests/simulation_engine_test.cpp)
  - [`cpp_refactor_plan.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_plan.md)
  - [`cpp_refactor_report_ko.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_report_ko.md)
- Change summary:
  introduced `FallbackResolutionPolicyLocal` to own SCC and waiting-deadlock fallback handling in the default planner path, and added a second public-API smoke test that exercises algorithm switching through `SimulationEngine`
- C++ improvement points:
  - better SRP around planner fallback policy
  - more object-oriented orchestration in the planner hot path
  - stronger verification of high-level API behavior after structural refactors
- Performance impact:
  neutral; the planner behavior is unchanged and the added object is stack-local only
- Readability impact:
  medium to high; the planner core now reads more clearly as planning -> fallback policy -> final conflict cleanup
- Risk:
  medium; this touches a hot planner path, but Windows smoke tests stayed stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`; CTest execution could not be completed in this environment because the installed `cmake` version is `3.22.1` while the project requires `3.24+`
- Next task:
  continue by pushing renderer composition or planner/runtime policy seams further, or validate the new tests in an environment with `cmake >= 3.24`

### 2026-03-11 - Dead duplicate removal and frame composer object

- Current task:
  remove leftover duplicate planner logic and make display-frame composition more explicitly object-oriented
- Selection reason:
  the remaining gap to a truly clean structure was no longer huge monoliths, but smaller inconsistencies such as duplicated helper logic across modules and renderer composition still expressed as a free-function list rather than a cohesive object
- Changed files:
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`src/planning/collision_planner_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.hpp)
  - [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)
  - [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)
  - [`cpp_refactor_plan.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_plan.md)
  - [`cpp_refactor_report_ko.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_report_ko.md)
- Change summary:
  removed the duplicate `best_candidate_order()` implementation from `collision_planner.cpp`, made the support module own that helper through its public header, fixed the support module’s independent linkage boundary, and replaced the renderer’s free-function section list with a `DisplayFrameComposer` object that owns frame composition order
- C++ improvement points:
  - less accidental duplication across translation units
  - clearer module ownership for planner support behavior
  - more explicit object-oriented frame composition in the renderer
- Performance impact:
  neutral; the changes are structural and do not add dynamic allocation
- Readability impact:
  high; hidden duplication is gone and the display path now has a single composer object that reflects the domain concept of a frame
- Risk:
  medium; planner support linkage changed, but repeated builds and smoke tests stayed stable
- Verification result:
  Windows `g++ 15.2.0` build passed and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  the main remaining work is now closer to “proof” than “structure”: run the policy-level tests in a suitable toolchain environment, or split a few still-large files further if the goal is a stricter 10/10 code-health standard

### 2026-03-11 - Final large-file decomposition pass

- Current task:
  reduce the remaining size of [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp) and [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp) by moving cohesive policy/support logic into dedicated translation units
- Selection reason:
  after the earlier refactors, the biggest gap to a “10/10 structure” was no longer tangled logic but the fact that two core files were still larger than they needed to be and still contained implementation detail that belonged to dedicated support modules
- Changed files:
  - [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)
  - [`src/runtime/agent_goal_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_goal_support.hpp)
  - [`src/runtime/agent_goal_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_goal_support.cpp)
  - [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
  - [`src/planning/collision_planner_default_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_default_support.hpp)
  - [`src/planning/collision_planner_default_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_default_support.cpp)
  - [`cpp_refactor_plan.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_plan.md)
  - [`cpp_refactor_report_ko.md`](/mnt/d/AGV_RRRR/AGV_REFACTORING/cpp_refactor_report_ko.md)
- Change summary:
  moved goal-assignment policy implementation out of `agent_runtime.cpp` into `agent_goal_support.cpp`, moved default planner support/fallback/conflict helpers out of `collision_planner.cpp` into `collision_planner_default_support.cpp`, and rewired the main translation units to act as orchestrators over those support modules
- C++ improvement points:
  - clearer module boundaries with smaller translation units
  - stronger SRP around runtime goal assignment and default planner fallback behavior
  - cleaner OOP/SOLID story because orchestration files now delegate to explicit support/policy modules instead of carrying their full implementation detail inline
- Performance impact:
  neutral; the refactor is structural and keeps the same algorithmic behavior
- Readability impact:
  very high; `collision_planner.cpp` dropped to about 200 lines and `agent_runtime.cpp` dropped to about 320 lines, making their top-level control flow much easier to audit
- Risk:
  medium; multiple new translation-unit seams were introduced, but the Windows build and smoke tests remained stable
- Verification result:
  Windows `g++ 15.2.0` build passed, `src/planning/collision_planner.cpp` is now about 202 lines, `src/runtime/agent_runtime.cpp` is now about 323 lines, and the headless smoke tests still returned `steps=22 tasks=1 movement=8 deadlocks=11` and `steps=20 tasks=1 movement=8 deadlocks=10`
- Next task:
  structurally, the codebase is now very close to the target end state; the remaining improvement is mainly stronger automated verification in an environment that can run the CMake/GTest suite
