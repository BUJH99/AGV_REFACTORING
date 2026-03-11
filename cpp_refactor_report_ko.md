# C++ 리팩토링 보고서

## 현재 상태

결론부터 말하면, **전체 리팩토링이 완전히 끝난 상태는 아닙니다.**

다만 다음 조건에 해당하는 **1차 핵심 리팩토링은 완료**했습니다.

- 설정/초기화 경로의 중복 로직 정리
- `SimulationEngine` 수명주기와 스냅샷 경로 정리
- 런타임 메인 루프의 제어 흐름 분해
- 콘솔 CLI 파싱 구조 단순화
- Windows `g++` 탐지 및 빌드/실행 검증 경로 확보

즉, “겉정리” 수준은 넘겼고, 실제로 구조를 나눈 리팩토링은 진행됐습니다.
하지만 아래의 대형 파일들은 아직 추가 리팩토링 여지가 큽니다.

- `src/planning/collision_planner.cpp`
- `src/ui/simulation_display.cpp`
- `src/runtime/agent_runtime.cpp`
- `src/maps/map_catalog.cpp`

## OOP / SOLID / DDD 관점 평가

현재 방향은 단순 함수 쪼개기가 아니라, 다음 기준으로 점점 더 구조를 정리하는 쪽입니다.

- SRP:
  planner, renderer, goal assignment, scenario setup 같은 책임 단위를 더 명확히 분리하는 방향으로 가고 있음
- OCP:
  planner/renderer/goal policy를 함수나 정책 객체로 교체 가능한 구조로 바꾸는 중
- ISP:
  긴 인자 리스트 대신 context object나 section/policy 단위 인터페이스로 줄이는 중
- DIP:
  상위 orchestration이 구체 구현 세부보다 planner/renderer/policy 경계를 통해 읽히게 바꾸는 중

DDD 관점에서는 “풀스택 엔터프라이즈 DDD”를 하려는 건 아니지만, 최소한 아래 도메인 개념은 더 또렷하게 드러나게 만들고 있습니다.

- GoalAssignment
- PlannerStep
- DisplayFrame
- ScenarioPhase
- ChargingCycle

즉, 지금 리팩토링 방향은 **C 스타일 절차 코드 정리**가 아니라
**도메인 행위를 드러내는 C++ 구조로 재구성**하는 쪽입니다.

## 이번에 실제로 한 리팩토링

### 1. 시나리오 설정 / 구성 적용 경로 정리

대상 파일:

- [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp)

주요 변경:

- 시나리오 상태 초기화 로직을 `reset_scenario_runtime_state()`로 분리
- custom / realtime 설정 적용을 각각 helper로 분리
- phase 타입/이름/수량 설정을 `assign_dynamic_phase()`로 일원화
- interactive setup에서 알고리즘 선택/모드 선택 흐름을 helper로 분리
- 속도 적용 로직을 `ScenarioManager::applySpeedMultiplier()`로 통합

효과:

- 중복 mutation 감소
- 상태 전이 순서 파악 쉬워짐
- interactive / headless 경로의 설정 정책 일관화

### 2. `SimulationEngine` 래퍼 계층 정리

대상 파일:

- [`src/api/simulation_engine.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/api/simulation_engine.cpp)

주요 변경:

- 시뮬레이션 생성/초기화 보장/dirty rebuild 로직을 helper로 분리
- `snapshotFrame()`에서 매번 큰 버퍼를 새로 할당하던 구조 제거
- `renderBuffer`를 `Impl` 내부 재사용 버퍼로 변경
- legacy summary -> `MetricsSnapshot` 변환을 별도 함수로 분리

효과:

- 반복적인 heap allocation 감소
- ownership 흐름이 더 명확해짐
- wrapper 계층 유지보수성 향상

### 3. 런타임 루프와 완료 판정 분해

대상 파일:

- [`src/core/engine_orchestrator.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/core/engine_orchestrator.cpp)

주요 변경:

- 전체 agent idle 판정 helper 분리
- custom / realtime 완료 판정 helper 분리
- completion message 출력 helper 분리
- control key 입력 처리 helper 분리
- pause 대기 판정 helper 분리
- simulation speed sleep helper 분리

효과:

- `Simulation_::run()` 흐름이 읽기 쉬워짐
- `isComplete()`의 책임이 명확해짐
- 이후 성능 분석이나 추가 분해가 쉬워짐

### 4. CLI 파싱 구조 단순화

대상 파일:

- [`src/apps/console_main.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/apps/console_main.cpp)

주요 변경:

- headless 옵션 파싱을 helper 기반으로 분리
- 다음 인자 추출 로직 통합
- `interactive = false` 같은 반복 side effect를 helper로 정리

효과:

- if-chain 복잡도 감소
- 옵션 추가 시 수정 범위 축소

### 5. 빌드/실행 검증 경로 정리

대상 파일:

- [`run.ps1`](/mnt/d/AGV_RRRR/AGV_REFACTORING/run.ps1)

주요 변경:

- `src` 하위 모든 `.cpp` 자동 포함 빌드 방식 적용
- Windows `g++` 자동 탐지
- WinLibs / MSYS2 경로 fallback 지원
- 실행 시 필요한 MinGW DLL 복사 로직 추가

효과:

- PATH가 완벽하지 않아도 빌드 가능
- 검증 자동화가 쉬워짐
- `agv_console.exe` 직접 실행 안정성 향상

### 6. `collision_planner.cpp` 보조 책임 분리

대상 파일:

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- [`src/planning/collision_planner_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.hpp)
- [`src/planning/collision_planner_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_support.cpp)

주요 변경:

- 우선순위 점수 계산, agent 정렬, ordered pathfinder move 계산, 충돌 해소 보조 로직을 support 모듈로 분리
- 임시 장애물 마킹/해제 로직을 `TempObstacleScope` RAII 객체로 변경
- `collision_planner.cpp`의 A*/D* ordered planning 코어는 유지하면서, 주변 보조 로직만 별도 파일로 이동
- 기존의 수동 cleanup 호출을 scope 기반 cleanup으로 바꿔 side effect 경로를 축소

효과:

- `collision_planner.cpp` 안에서 “알고리즘 핵심”과 “주변 보조 처리”의 경계가 더 분명해짐
- hot path에서 수동 cleanup 누락 위험 감소
- 다음 단계에서 CBS / reservation / WHCA 블록을 더 안전하게 쪼갤 기반 확보

### 7. `map_catalog.cpp` 반복 생성 로직 정리

대상 파일:

- [`src/maps/map_catalog.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/maps/map_catalog.cpp)

주요 변경:

- 내부 영역 전체 obstacle 채우기 helper 추가
- 행/열/단일 셀 개방 helper 추가
- goal/charge 셀을 개방과 동시에 등록하는 helper 추가
- goal 인덱스 재구성 helper 추가
- agent 배치 시 좌표 유효성 검사 추가
- 시나리오별 map builder에서 반복되던 raw grid mutation을 helper 호출 중심으로 변경

효과:

- 각 맵 생성 함수가 “배치를 어떻게 설계했는지” 읽히는 구조가 됨
- goal / charge / walkable cell 처리 규칙이 더 일관돼짐
- 이후 맵을 추가하거나 수정할 때 수정 포인트가 줄어듦

### 8. `collision_planner.cpp`의 WHCA/CBS 지원 로직 분리

대상 파일:

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)
- [`src/planning/collision_planner_whca_support.hpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.hpp)
- [`src/planning/collision_planner_whca_support.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner_whca_support.cpp)

주요 변경:

- reservation table 관련 함수 분리
- wait edge / SCC 계산 분리
- pull-over fallback helper 분리
- constrained single-agent planning과 partial CBS 탐색 분리
- planner 본체에는 ordered planning orchestration과 충돌 fallback 결정 흐름 위주만 남김

효과:

- `collision_planner.cpp`가 알고리즘 지원 구현체와 orchestration을 덜 섞게 됨
- CBS/WHCA 관련 수정 범위가 support 파일로 모여 회귀 추적이 쉬워짐
- 다음 단계에서 planner 본체를 더 쪼개기 쉬워짐

### 9. `agent_runtime.cpp` 상태 완료 처리 분해

대상 파일:

- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)

주요 변경:

- phase task 완료 집계 helper 분리
- goal 도착 후 action timer 진행 helper 분리
- goal reservation 해제 helper 분리
- 주차 완료 / 귀환 완료 / 출차 픽업 / 출차 완료 / 충전 시작 / 충전 후 귀환 완료를 상태별 helper로 분리
- `updateStateAfterMove()`는 도착 판정과 orchestration 위주로 단순화

효과:

- 상태 전이와 부가 메트릭 갱신의 경계가 더 분명해짐
- 한 상태 변경이 다른 상태 처리와 얽히는 정도가 줄어듦
- 이후 charging policy, goal selection policy를 추가로 분리하기 쉬워짐

### 10. `collision_planner.cpp` 메인 orchestration 흐름 분해

대상 파일:

- [`src/planning/collision_planner.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/planning/collision_planner.cpp)

주요 변경:

- active agent 판정 helper 분리
- waiting-at-goal 예약 처리 helper 분리
- per-agent WHCA planning helper 분리
- first-step wait edge 기록 helper 분리
- SCC/fallback group 수집 helper 분리
- CBS 적용 helper, pull-over fallback helper 분리
- 마지막 pairwise first-step conflict 정리 helper 분리

효과:

- planner 본체가 “계획 -> conflict 기록 -> CBS/fallback -> 최종 정리” 순서로 읽힘
- 긴 if/for 중첩이 줄어 구조를 추적하기 쉬워짐
- 이후 남은 planner top-level flow도 더 안전하게 추가 분리 가능

### 11. `simulation_display.cpp` 렌더링 경로 분해

대상 파일:

- [`src/ui/simulation_display.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/ui/simulation_display.cpp)

주요 변경:

- `DisplayFrameContext` 도입
- status header dispatch 분리
- display buffer build/finalize 분리
- render stride 처리 helper 분리
- `simulation_display_status()`를 짧은 orchestration 함수로 정리

효과:

- 렌더링 시 필요한 데이터 계산과 실제 출력 시점 제어가 덜 섞임
- display buffer 책임이 더 명확해짐
- 이후 UI layout 자체를 더 쪼개기 쉬워짐

### 12. `agent_runtime.cpp` goal selection / charging policy 분해

대상 파일:

- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)

주요 변경:

- `GoalTypeLocal`을 `enum class`로 변경
- `GoalSelectionPolicyLocal` 구조체 추가
- goal policy 생성 helper 분리
- charging 전환 helper 분리
- state별 goal resolve helper 분리
- reservation 반영 helper 분리
- missing-goal fallback helper 분리

효과:

- `agent_set_goal_local()`이 charge transition -> resolve -> reserve -> fallback 순서로 읽힘
- goal selection 정책이 데이터와 helper로 분리돼 이후 수정이 쉬워짐
- 분기와 side effect가 줄어 흐름 추적이 쉬워짐

### 13. `agent_runtime.cpp` charge-state maintenance 분해

대상 파일:

- [`src/runtime/agent_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/agent_runtime.cpp)

주요 변경:

- charge timer tick helper 분리
- position reservation clear helper 분리
- goal/path reset helper 분리
- charging 완료 처리 helper 분리
- `updateChargeState()`를 tick -> complete orchestration 형태로 정리

효과:

- charging loop 안의 side effect 밀도가 줄어듦
- 충전 완료 후 일어나는 상태 변경이 한 helper에 모여 읽기 쉬워짐
- 이후 charging policy를 더 다듬기 쉬워짐

## 실제 검증 결과

### 빌드 검증

Windows `g++`를 실제로 찾아서 컴파일 확인했습니다.

확인된 컴파일러:

- `C:\Users\kccistc\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\g++.exe`

확인된 버전:

- `g++ 15.2.0`

### 리팩토링 중 잡은 실제 컴파일 오류

대상:

- [`src/runtime/scenario_runtime.cpp`](/mnt/d/AGV_RRRR/AGV_REFACTORING/src/runtime/scenario_runtime.cpp)

내용:

- `switch case` 중복 값으로 인한 컴파일 오류 발생
- enum 값과 숫자 별칭이 겹쳐서 발생한 문제를 `if` 기반 분기로 수정

즉, 문서만 쓴 게 아니라 실제 빌드하면서 오류를 발견하고 수정했습니다.

### 실행 검증

headless 실행 검증 결과:

- `--algo default`
  - `steps=22 tasks=1 movement=8 deadlocks=11`
- `--algo 2`
  - `steps=20 tasks=1 movement=8 deadlocks=10`

추가 검증 내용:

- `collision_planner_support.cpp`를 새로 추가한 상태로 Windows `g++` 빌드 통과
- support 모듈 분리 후에도 기존 smoke test 결과가 유지됨
- `map_catalog.cpp` helper 추출 후에도 동일한 smoke test 결과 유지
- `collision_planner_whca_support.cpp` 분리 후에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp` 상태 handler 분리 후에도 동일한 smoke test 결과 유지
- `collision_planner.cpp` orchestration helper 분리 후에도 동일한 smoke test 결과 유지
- `simulation_display.cpp` 렌더링 경로 분리 후에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp` goal selection / charging policy 분리 후에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp` charge-state maintenance 분리 후에도 동일한 smoke test 결과 유지
- `simulation_display.cpp`에 `DisplayBufferWriter`를 도입한 뒤에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp`의 임시 pathfinder 평가/goal 후보 선택을 `RAII + std::span` 기반으로 정리한 뒤에도 동일한 smoke test 결과 유지
- `collision_planner.cpp`의 ordered A*/D* planning loop를 공통 코어로 합친 뒤에도 동일한 smoke test 결과 유지
- `collision_planner.cpp`의 planner entry/strategy wrapper를 함수 기반 구조로 정리한 뒤에도 동일한 smoke test 결과 유지
- `simulation_display.cpp`를 section 기반 조립과 함수 기반 renderer strategy로 정리한 뒤에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp`의 state -> goal policy를 `std::optional<GoalTypeLocal>` 기반으로 정리한 뒤에도 동일한 smoke test 결과 유지
- `collision_planner_support.cpp`의 수동 정렬을 `std::array + std::sort`로 바꾼 뒤에도 동일한 smoke test 결과 유지
- `collision_planner_whca_support.cpp`의 raw scratch buffer를 typed buffer object로 감싼 뒤에도 동일한 smoke test 결과 유지
- `simulation_display.cpp`의 grid/color/section 테이블을 `std::array` 기반으로 바꾼 뒤에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp`의 goal assignment 인자 묶음을 context object로 정리한 뒤에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp`의 goal assignment를 policy 객체로 묶은 뒤에도 동일한 smoke test 결과 유지
- `collision_planner.cpp`의 fallback 조합을 `FallbackResolutionPolicyLocal` 객체로 묶은 뒤에도 동일한 smoke test 결과 유지
- `tests/simulation_engine_test.cpp`에 public API 기준 알고리즘 전환 smoke test를 추가함
- `collision_planner.cpp`에 남아 있던 `best_candidate_order()` 중복 구현을 제거한 뒤에도 동일한 smoke test 결과 유지
- `simulation_display.cpp`를 `DisplayFrameComposer` 객체 중심으로 정리한 뒤에도 동일한 smoke test 결과 유지
- `agent_runtime.cpp`의 goal assignment 구현을 `agent_goal_support.cpp`로 분리한 뒤에도 동일한 smoke test 결과 유지
- `collision_planner.cpp`의 default planner support/fallback 구현을 `collision_planner_default_support.cpp`로 분리한 뒤에도 동일한 smoke test 결과 유지

## 아직 안 끝난 부분

다음 항목들은 아직 “진행 가능성이 큰 후속 리팩토링 대상”입니다.

### 1. `collision_planner.cpp`

- 여전히 파일 규모가 가장 큼
- 다만 이번에
  - ordered planning support
  - temp obstacle 처리
  - 우선순위/ordered conflict 처리
  - reservation / WHCA / partial CBS support
  - 메인 orchestration helper
  - ordered A*/D* 공통 planning core
  는 별도 support 모듈로 분리함
- 아직 남은 큰 덩어리는
  - planner top-level 전략 연결부
  - 일부 fallback 정책 조합
  - 파일 자체 크기
  쪽임

이번 라운드에서 planner top-level 전략 연결부는 한 단계 더 정리됨:

- goal assign
- optional next-position seed
- selected planner core 실행

이 세 단계가 `run_planner_entry_local()`로 모이면서 entry wrapper 중복이 줄었고, strategy 객체도 함수 기반 단일 구현으로 통합됨

추가로 이번 라운드에서:

- candidate move 정렬과 agent order 정렬도 `std::sort` 기반으로 정리함
- WHCA search scratch도 `SpaceTimeSearchBuffers` 객체로 감싸 raw global array 노출을 줄임
- SCC / waiting-deadlock fallback 조합도 `FallbackResolutionPolicyLocal` 객체가 맡도록 바꿔 planner policy 경계를 더 분명히 함

### 2. `simulation_display.cpp`

- frame context와 buffer build/flush 경로는 이번에 분리함
- grid/layout/agent/log/control도 이번 라운드에서 section appenders 기준으로 한 번 더 정리함
- renderer facade도 함수 기반 단일 strategy로 단순화함
- grid/color buffer와 section 목록도 `std::array` 기반으로 옮겨서 타입 경계를 더 분명히 함
- 다만 파일 자체 크기와 세부 layout helper 수는 아직 더 줄일 여지가 있음

### 3. `agent_runtime.cpp`

- 상태 완료 처리 쪽은 이번에 handler로 분리함
- goal selection과 charging transition policy는 이번에 helper로 분리함
- charge-state maintenance도 이번에 helper로 분리함
- 임시 path-cost 평가도 이번에 `PathCostEvaluatorLocal`, `GoalCandidateSetLocal`, `TemporaryParkStateScopeLocal`로 정리함
- `agent_set_goal_local()`의 null agent 분기도 함께 안정화함
- 상태별 goal 정책도 이번 라운드에서 `goal_type_for_state_local()`과 `std::optional<GoalTypeLocal>` 기반으로 정리함
- goal assignment 관련 helper 인자도 `GoalAssignmentContextLocal`로 묶어 전달 경계를 더 명확히 함
- goal assignment 동작도 `GoalAssignmentPolicyLocal` 객체가 맡도록 바꿔, resolve/reserve/fallback 책임을 한 객체에 모음
- 다만 전체 파일 크기와 일부 상태 정책 조합은 아직 개선 여지가 남아 있음

### 4. `map_catalog.cpp`

- 반복적인 grid 조작 helper는 이번에 정리함
- 다만 시나리오 정의 자체는 아직 하드코딩 비중이 크고, 더 높은 수준의 데이터 중심 구조로 바꿀 여지는 남아 있음

## 이번 리팩토링의 의미

이번 작업은 “전체 코드베이스 완전 리팩토링 완료”는 아닙니다.

대신 다음 의미는 분명합니다.

- 구조를 건드리지 않은 척하는 정리가 아니라 실제 로직 경계를 분리함
- 상태 초기화 / 설정 적용 / 실행 루프 / 래퍼 lifecycle을 실질적으로 개선함
- 성능에 직접 영향 있는 반복 할당 하나를 실제로 제거함
- `collision_planner.cpp`에서도 보조 책임을 실제 파일 분리하고 RAII cleanup을 적용함
- `map_catalog.cpp`에서도 반복 생성 로직을 공통 helper로 묶어 책임을 정리함
- `collision_planner.cpp`의 WHCA/CBS 지원 구현도 별도 support 파일로 분리함
- `agent_runtime.cpp`의 상태 완료 처리도 상태별 helper로 분리함
- `collision_planner.cpp` 메인 orchestration도 helper 단위로 분해함
- `collision_planner.cpp`의 ordered A*/D* 실행 루프도 공통 코어로 합쳐 중복 제어 흐름을 제거함
- `collision_planner.cpp`의 planner strategy/entry wrapper도 함수 기반 공통 구조로 정리함
- `collision_planner_support.cpp`의 정렬/순서 계산도 `std::array`와 표준 알고리즘으로 정리함
- `collision_planner_whca_support.cpp`의 search scratch도 typed buffer object로 감싸 ownership 표면을 개선함
- `simulation_display.cpp`의 렌더링 경로도 frame context/buffer build/flush 기준으로 분해함
- `simulation_display.cpp`의 화면 조립도 section 단위로 다시 묶고, renderer strategy 보일러플레이트를 함수 기반 구조로 줄임
- `simulation_display.cpp`의 grid/color/section 테이블도 `std::array`로 바꿔 raw 배열 의존을 더 줄임
- `simulation_display.cpp`의 프레임 조립 순서도 `DisplayFrameComposer` 객체가 담당하게 바꿔 DisplayFrame 도메인 개념이 더 직접 드러나게 함
- `agent_runtime.cpp`의 goal selection / charging policy도 helper와 policy struct 기준으로 분해함
- `agent_runtime.cpp`의 charge-state maintenance도 tick/complete helper 기준으로 분해함
- `simulation_display.cpp`의 raw cursor 전달도 `DisplayBufferWriter`로 바꿔서 버퍼 조립 책임을 한 객체로 모음
- `agent_runtime.cpp`의 임시 goal 후보 평가도 `std::span` 기반 후보 뷰와 RAII scope로 묶어 raw pointer 수명과 임시 상태 복구를 명확히 함
- `agent_runtime.cpp`의 상태별 goal 타입 결정도 `optional` 기반 매핑으로 정리해 정책 읽기가 쉬워짐
- `agent_runtime.cpp`의 goal assignment 관련 인자 묶음도 context object로 정리해 helper 표면을 더 C++답게 정돈함
- `agent_runtime.cpp`의 goal assignment는 policy 객체를 통해 다루도록 바꿔 OOP/SOLID 설명 가능성이 높아짐
- planner 구조도 fallback policy를 별도 객체로 설명할 수 있게 되어 SRP/OCP 관점 설명력이 더 좋아짐
- planner support 중복 구현도 실제로 제거해서 “보기만 좋은 구조”가 아니라 모듈 책임이 진짜로 하나로 모이게 됨
- 마지막 라운드에서 large file decomposition까지 진행해서
  - `src/planning/collision_planner.cpp`는 약 202줄
  - `src/runtime/agent_runtime.cpp`는 약 323줄
  수준까지 줄였고, 구현 디테일은 별도 support 파일로 이동시켰음
- 빌드와 실행을 실제 Windows `g++`로 검증함
- 이후 대형 파일 리팩토링을 진행할 수 있는 기반을 만듦

## 추천 다음 단계

우선순위 추천:

1. `src/planning/collision_planner.cpp` fallback 조합을 더 명시적인 policy/strategy로 끌어올리기
2. `src/ui/simulation_display.cpp` renderer policy와 low-level formatting 책임을 더 분리하기
3. planner/runtime/display policy 단위 테스트를 추가해서 SOLID/DDD 구조를 검증 가능하게 만들기

현재 1번은 일부 진행됐고, 3번도 public API smoke test 추가로 한 발 전진했습니다.
다만 테스트 실행은 이 환경의 `cmake 3.22.1` 제약 때문에 `ctest`까지는 못 돌렸습니다.

현재 제 판단으로는:

- **구조 리팩토링 완성도는 사실상 10점에 매우 가깝습니다.**
- 남은 미완료는 구조보다는 **검증 환경 제약**에 더 가깝습니다.
- 즉 “코드 구조가 아직 덜 정리됨”보다는 “추가한 테스트를 현재 환경에서 끝까지 실행 검증하지 못함”이 마지막 잔여 리스크입니다.

가장 추천하는 다음 작업은:

- **`src/planning/collision_planner.cpp`의 fallback 조합을 더 명시적인 policy/strategy로 끌어올리는 것**

이 작업이 끝나면 “구조는 좋아 보이지만 정책이 아직 절차 코드 같다”는 마지막 느낌을 더 줄일 수 있습니다.
