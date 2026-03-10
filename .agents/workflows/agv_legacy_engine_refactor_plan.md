# engine 분해 최종 계획 및 완료 상태

## 1. 목표
- 목표는 `src/core/engine_orchestrator.cpp`를 알고리즘 구현 파일이 아니라 orchestration 중심 파일로 정리하는 것이다.
- engine은 시뮬레이션 생명주기, active context, facade wiring, 상위 실행 흐름만 남긴다.
- pathfinder, collision/CBS, scenario/setup, step execution, agent runtime, report, display, console, maps는 각 전용 구현 파일로 분리한다.

## 2. 최종 결과
- 구현 파일: `10개`
- 전체 관련 구현 파일: `11개`
  - `src/api/simulation_engine.cpp` 포함
- `src/core/engine_orchestrator.cpp`: `1017줄`
- 필수 목표였던 `1200줄 이하` 달성
- 권장 목표였던 `900~1100줄`도 달성

## 3. 최종 파일 구성
### 헤더 배치 원칙
- 모든 public / internal `.hpp`는 `include/` 아래에 둔다.
- 모든 구현 `.cpp`는 `src/` 아래에 둔다.
- `legacy` 접두가 남아 있던 헤더 이름은 기능 기준 이름으로 재매핑한다.

### 최종 include 구성
1. `include/agv/simulation_engine.hpp`
- 외부 C++ API 선언

2. `include/agv/sim_bridge.hpp`
- 엔진 브리지 / 공용 facade 선언

3. `include/agv/internal/engine_model.hpp`
- 내부 모델 / 상태 / 메트릭 타입 선언

4. `include/agv/internal/engine_internal.hpp`
- 내부 구현 파일 사이의 bridge 선언

### 최종 src 구성
1. `src/core/engine_orchestrator.cpp`
- simulation lifecycle
- active context
- planner / renderer facade wiring
- high-level run orchestration

2. `src/reporting/run_reporting.cpp`
- run summary 계산
- performance summary 출력
- JSON / CSV report helper

3. `src/maps/map_catalog.cpp`
- map builder
- scenario load
- grid 초기화

4. `src/platform/console_terminal.cpp`
- Win32 console / terminal 제어
- console 준비 helper

5. `src/ui/simulation_display.cpp`
- display status
- render buffer 조립
- input control helper

6. `src/planning/pathfinder.cpp`
- D* Lite / pathfinder core
- shortest path
- next-step 계산
- low-level search helper

7. `src/planning/collision_planner.cpp`
- reservation table
- WHCA planning
- collision resolution
- wait-for graph
- deadlock fallback
- CBS orchestration

8. `src/runtime/scenario_runtime.cpp`
- simulation setup
- custom / realtime setup
- phase / task queue 초기화
- scenario runtime configuration
- task 시작 helper

9. `src/runtime/step_runtime.cpp`
- execute-one-step flow
- move apply
- deadlock counter
- frame finalize helper
- runtime metric sampling

10. `src/runtime/agent_runtime.cpp`
- agent state after move
- charging / maintenance 전이
- task completion bookkeeping
- goal selection / workload snapshot / runtime helper

11. `src/api/simulation_engine.cpp`
- 외부 C++ wrapper API

## 4. 단계별 완료 상태
### 단계 A. pathfinder 완전 분리
- 완료
- `pathfinder_create/destroy/reset_goal/update_start/notify_cell_change/compute_shortest_path/get_next_step`
- pathfinder core helper가 `src/planning/pathfinder.cpp`로 이동

### 단계 B. collision / CBS 완전 분리
- 완료
- reservation table, WHCA, wait-for graph, deadlock fallback, CBS orchestration 이동
- `agent_manager_plan_and_resolve_collisions*_core`가 collision 전용 파일로 분리

### 단계 C. scenario / setup 분리
- 완료
- `simulation_setup`, custom/realtime 설정, phase/task 초기화 이동
- manager create/destroy 및 task begin helper도 scenario 쪽으로 정리

### 단계 D. step execution 분리
- 완료
- execute-one-step 보조, move apply, deadlock counter, finalize helper, runtime sampling 이동

### 단계 E. agent runtime 분리
- 완료
- `AgentManager::updateStateAfterMove`
- `AgentManager::updateChargeState`
- goal selection / charge selection / workload snapshot / set-goal helper 이동

### 단계 F. engine 마감 정리
- 완료
- 미사용 static helper 제거
- 얇은 wrapper 정리
- console 준비 helper 이동
- `engine` 줄 수 `1200 이하` 달성

## 5. 현재 줄 수
- `src/core/engine_orchestrator.cpp`: `1017`
- `src/reporting/run_reporting.cpp`: `291`
- `src/maps/map_catalog.cpp`: `635`
- `src/platform/console_terminal.cpp`: `72`
- `src/ui/simulation_display.cpp`: `480`
- `src/planning/pathfinder.cpp`: `320`
- `src/planning/collision_planner.cpp`: `1388`
- `src/runtime/scenario_runtime.cpp`: `501`
- `src/runtime/step_runtime.cpp`: `351`
- `src/runtime/agent_runtime.cpp`: `434`
- `src/api/simulation_engine.cpp`: `276`

## 6. 검증 상태
- 수동 재빌드 통과
  - `tmp_build/agv_console_refactor_check.exe`
- headless smoke 비교 통과
  - `tmp_build/smoke_cycle42/run_summary.json`
- 유지 확인 지표
  - `recorded_steps = 22`
  - `tasks_completed_total = 1`
  - `deadlock_count = 11`
  - `total_movement_cost = 8.0`
  - `algo_nodes_expanded_total = 14`
  - `algo_heap_moves_total = 35`
  - `algo_generated_nodes_total = 33`
  - `algo_valid_expansions_total = 14`

## 7. 최종 판단
- 구조 목표는 달성했다
- 파일 수 목표도 달성했다
- `engine`은 이제 orchestration 파일로 읽힌다
- 권장 줄 수 목표도 달성했다
- 추가 작업은 필수 분해가 아니라 선택적 후속 정리로 보는 것이 맞다

## 8. 선택적 후속 후보
- `src/planning/collision_planner.cpp` 추가 내부 분해
- `src/planning/pathfinder.cpp` 내부 helper 재배치
- `engine`의 공백 / 주석 스타일 추가 정돈
- `900~1100줄` 목표를 노리는 추가 감축 여부 재평가
