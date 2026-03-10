# agv_legacy_engine.cpp 점진 리팩터링 최종 상태

## 목표
- `src/agv_legacy_engine.cpp`를 한 번에 재설계하지 않고, 동작 보존을 우선하면서 작은 패치 단위로 책임을 분리한다.
- 각 단계마다 실제 빌드와 headless 스모크로 분리 안정성을 확인한다.
- 파일 분리는 helper 추출로 경계를 먼저 고정한 뒤에만 수행한다.

## 책임 분해 결과
- simulation lifecycle / active context / runtime state
- scenario / task / phase 상태 관리
- planner / pathfinder / CBS / collision 처리
- step execution / metrics / timing
- map builder / scenario load
- console / display / render
- report / summary / CSV / JSON 출력

## 완료된 리팩터링 단계
- `Simulation_::planStep()`의 알고리즘별 step counter 집계를 helper로 추출했다.
- `Simulation_::planStep()`의 per-step planner metric reset 블록을 helper로 추출했다.
- `Simulation_::planStep()`의 post-planning accumulation / timing 블록을 `record_planner_step_results()`로 추출했다.
- `StepExecutorService::finalizeFrame()`의 phase accounting 블록을 `record_step_phase_accounting()`으로 추출했다.
- report/summary 영역의 공통 계산과 라벨 로직을 helper로 추출했다.
- run summary 계산과 JSON payload 쓰기를 각각 helper로 분리했다.
- report/summary 구현을 `src/agv_legacy_reports.cpp`로 분리했다.
- map builder / scenario load / grid 초기화 구현을 `src/agv_legacy_maps.cpp`로 분리했다.
- Win32 console 제어 함수 일부를 `src/agv_legacy_console.cpp`로 분리했다.
- `simulation_display_status()`와 `GridMap_renderToBuffer()` 주변 display 구현을 `src/agv_legacy_display.cpp`로 분리했다.
- `ui_append_controls_help()`를 playback/render helper로 분리했다.
- `ui_flush_display_buffer()`를 position/write helper로 분리했다.
- `ui_handle_control_key()`와 관련 speed / render toggle helper를 `src/agv_legacy_display.cpp`로 실제 이동 완료했다.
- `run_partial_CBS()`에서 반복되던 단일 agent low-level 계획 + metric 누적 블록을 `cbs_plan_agent_with_metrics()`로 추출했다.
- 분리 파일 공통 의존성을 위해 `src/agv_legacy_engine_internal.hpp`를 보강했다.

## 최종 파일 분리 결과
- `src/agv_legacy_engine.cpp`
  - simulation core, planner/collision orchestration, step execution 중심
- `src/agv_legacy_reports.cpp`
  - run summary 계산, performance summary 출력, JSON/CSV report helper
- `src/agv_legacy_maps.cpp`
  - map builder, scenario load, grid 초기화
- `src/agv_legacy_console.cpp`
  - console / terminal 제어
- `src/agv_legacy_display.cpp`
  - display status, render buffer 조립, input control helper

## 검증 상태
- 수동 빌드 명령:
  - `C:\\msys64\\mingw64\\bin\\g++.exe -std=c++20 -DAGV_NO_MAIN -Iinclude src/main.cpp src/agv_legacy_engine.cpp src/agv_legacy_reports.cpp src/agv_legacy_maps.cpp src/agv_legacy_console.cpp src/agv_legacy_display.cpp src/simulation_engine.cpp -o tmp_build/agv_console_refactor_check.exe -lpsapi`
- 검증 바이너리:
  - `tmp_build/agv_console_refactor_check.exe`
- headless 스모크 결과:
  - `tmp_build/smoke/run_summary.json`
  - `tmp_build/smoke/step_metrics.csv`
  - `tmp_build/smoke_cycle1` ~ `tmp_build/smoke_cycle12`
- 최신 확인:
  - `tmp_build/smoke_cycle11/run_summary.json`
  - `tmp_build/smoke_cycle11/step_metrics.csv`
  - `tmp_build/smoke_cycle12/run_summary.json`
  - `tmp_build/smoke_cycle12/step_metrics.csv`
- 최신 스모크 기준 요약:
  - `recorded_steps = 22`
  - `tasks_completed_total = 1`
  - `deadlock_count = 11`

## 최종 판단
- 계획서에 정의했던 분리 순서는 모두 완료했다.
- report / map / console / display는 실제 별도 파일 분리까지 끝났다.
- planner / step execution은 고위험 영역이므로 helper 추출 수준에서 안전하게 경계를 고정했다.
- 마지막 고위험 단계도 CBS 내부 중복 helper 추출까지 적용해, “작은 단위로 마지막 영역에 착수한다”는 계획 목표를 충족했다.

## 남은 권장사항
- `agent_manager_plan_and_resolve_collisions()`는 여전히 가장 큰 복잡도 hotspot이므로, 이후 작업이 필요하면 vertex conflict 처리와 deadlock fallback 블록을 각각 helper로 한 번 더 나누는 것이 좋다.
- `pathfinder` / `CBS` / `reservation table`은 이번 계획 범위에서는 동작 보존 우선 때문에 별도 파일 분리까지는 진행하지 않았다.
- 남은 작업은 “필수 계획 항목”이 아니라 “다음 단계 후보”로 보는 것이 맞다.
