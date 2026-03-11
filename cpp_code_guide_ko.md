# AGV C++ 코드 가이드북

## 1. 이 문서의 목적

이 문서는 이 프로젝트를 처음 읽는 사람이 다음 질문에 빠르게 답할 수 있도록 만들었다.

- 프로그램 시작점은 어디인가?
- 한 스텝이 어떤 함수들을 거쳐 실행되는가?
- 파일별 역할은 무엇인가?
- 어떤 파일은 public API이고, 어떤 파일은 내부 구현인가?
- planner/runtime/ui를 어디서부터 읽어야 하는가?

이 문서는 “코드 설명서 + 탐색 가이드” 성격이다.

---

## 2. 가장 먼저 알아야 할 것

### 실제 시작점

보통 우리가 실행하는 엔트리 포인트는 아래 파일이다.

- `src/apps/console_main.cpp`

여기서

- CLI 인자 파싱
- interactive 모드 / headless 모드 선택
- `SimulationEngine` 생성 및 실행

이 이뤄진다.

### 내부 legacy main

아래 파일에도 `main()`이 있다.

- `src/core/engine_orchestrator.cpp`

이건 legacy 스타일 실행 경로다.  
현재 빌드에서는 `AGV_NO_MAIN`으로 제외되는 경우가 많고, 보통은 `src/apps/console_main.cpp`를 먼저 보면 된다.

즉, **읽기 시작은 `console_main.cpp`부터** 가 맞다.

---

## 3. 추천 읽기 순서

처음 읽을 때는 아래 순서가 가장 좋다.

### 1단계. 실행 흐름 큰 그림 보기

1. `src/apps/console_main.cpp`
2. `include/agv/simulation_engine.hpp`
3. `src/api/simulation_engine.cpp`
4. `src/core/engine_orchestrator.cpp`
5. `src/runtime/step_runtime.cpp`

이 다섯 개를 보면 “프로그램이 어떻게 돌기 시작해서 한 스텝을 처리하는지”가 보인다.

### 2단계. 관심사별로 내려가기

- planner 알고리즘이 궁금하면
  - `src/planning/collision_planner.cpp`
  - `src/planning/collision_planner_default_support.cpp`
  - `src/planning/collision_planner_support.cpp`
  - `src/planning/collision_planner_whca_support.cpp`
  - `src/planning/pathfinder.cpp`

- AGV 상태 전이가 궁금하면
  - `src/runtime/agent_runtime.cpp`
  - `src/runtime/agent_goal_support.cpp`
  - `src/runtime/scenario_runtime.cpp`

- 화면 출력이 궁금하면
  - `src/ui/simulation_display.cpp`
  - `src/platform/console_terminal.cpp`

- 맵 구성과 시나리오 레이아웃이 궁금하면
  - `src/maps/map_catalog.cpp`

---

## 4. 전체 실행 흐름

### 4.1 콘솔 앱 시작

`src/apps/console_main.cpp`

- `main(argc, argv)`
- `parseArgs()`
- `runInteractive()` 또는 `runHeadless()`

### 4.2 public wrapper 진입

`src/api/simulation_engine.cpp`

`SimulationEngine`가 내부 `Simulation*`를 감싼다.

주요 메서드:

- `loadMap()`
- `setAlgorithm()`
- `configureScenario()`
- `step()`
- `runUntilComplete()`
- `snapshotMetrics()`
- `snapshotFrame()`

### 4.3 내부 시뮬레이터 생성 / 실행

`src/core/engine_orchestrator.cpp`

여기서 실제 내부 객체 `Simulation_`가 돌아간다.

주요 메서드:

- `Simulation_::planStep()`
- `Simulation_::updateState()`
- `Simulation_::executeOneStep()`
- `Simulation_::isComplete()`
- `Simulation_::run()`

### 4.4 한 스텝 처리

`src/runtime/step_runtime.cpp`

여기서 실제로

- planner 호출
- 이동 적용
- stuck/deadlock 갱신
- task/state 업데이트
- 렌더링/출력 처리

가 한 번에 이어진다.

이 파일은 “한 스텝 서비스 레이어”라고 생각하면 된다.

---

## 5. 파일 계층 구조

이 프로젝트는 크게 아래 계층으로 보면 이해가 쉽다.

### 5.1 App Layer

- `src/apps/console_main.cpp`

사용자 입력, CLI, 실행 모드 선택을 담당한다.

### 5.2 Public API Layer

- `include/agv/simulation_engine.hpp`
- `src/api/simulation_engine.cpp`

외부에서 안전하게 쓰는 C++ API이다.

### 5.3 Bridge / Legacy Interface Layer

- `include/agv/sim_bridge.hpp`

C 스타일 함수 인터페이스와 `Simulation*` 브리지 API를 제공한다.

### 5.4 Core Engine Layer

- `include/agv/internal/engine_model.hpp`
- `include/agv/internal/engine_internal.hpp`
- `src/core/engine_orchestrator.cpp`

엔진의 핵심 모델과 실행 orchestration이 있다.

### 5.5 Runtime Domain Layer

- `src/runtime/scenario_runtime.cpp`
- `src/runtime/step_runtime.cpp`
- `src/runtime/agent_runtime.cpp`
- `src/runtime/agent_goal_support.cpp`

시나리오, 스텝 처리, AGV 상태 전이, goal 정책이 여기 있다.

### 5.6 Planning Layer

- `src/planning/pathfinder.cpp`
- `src/planning/collision_planner.cpp`
- `src/planning/collision_planner_default_support.cpp`
- `src/planning/collision_planner_support.cpp`
- `src/planning/collision_planner_whca_support.cpp`

경로 탐색과 충돌 해소 로직이 있다.

### 5.7 UI / Platform Layer

- `src/ui/simulation_display.cpp`
- `src/platform/console_terminal.cpp`

렌더링 문자열 조립과 콘솔 제어를 담당한다.

### 5.8 Map / Reporting / Test

- `src/maps/map_catalog.cpp`
- `src/reporting/run_reporting.cpp`
- `tests/simulation_engine_test.cpp`

---

## 6. 헤더 파일 설명

### `include/agv/simulation_engine.hpp`

가장 중요한 public C++ API 헤더다.

담당:

- 외부에 공개되는 enum/struct
- `SimulationEngine` 클래스
- `ScenarioConfig`, `MetricsSnapshot`, `RenderFrame`

이 프로젝트를 라이브러리처럼 쓴다면 가장 먼저 보는 헤더다.

### `include/agv/sim_bridge.hpp`

내부 엔진을 C 스타일 함수로 노출하는 브리지 인터페이스다.

담당:

- `simulation_create()`
- `simulation_destroy()`
- `agv_apply_config()`
- `agv_run_to_completion()`
- `agv_copy_render_frame()`
- `agv_collect_run_summary()`

즉, `SimulationEngine`와 내부 엔진 사이의 중간 다리다.

### `include/agv/internal/engine_model.hpp`

핵심 내부 자료형 정의 파일이다.

담당:

- `Node`
- `Agent`
- `GridMap`
- `Pathfinder`
- `ScenarioManager`
- `Logger`
- `Simulation_`
- planner/renderer facade 타입

실질적인 “엔진 내부 모델 정의서”다.

### `include/agv/internal/engine_internal.hpp`

엔진 전체에서 쓰는 내부 상수, 브리지 helper 선언, runtime helper 선언이 모여 있다.

담당:

- 상수 매크로
- 내부 helper 함수 선언
- runtime/planner 메트릭 helper 선언

`engine_model.hpp`보다 한 단계 더 구현 친화적인 내부 헤더다.

---

## 7. 소스 파일 설명

## 7.1 App / API

### `src/apps/console_main.cpp`

책임:

- CLI 인자 파싱
- interactive/headless 모드 선택
- `SimulationEngine`를 통한 실행

이 파일을 보면 “프로그램 사용법”과 “실행 방식”을 이해할 수 있다.

### `src/api/simulation_engine.cpp`

책임:

- `SimulationEngine` 구현
- config 반영
- dirty rebuild 처리
- metrics/frame snapshot 변환

이 파일은 외부 인터페이스를 정리하는 wrapper 계층이다.

---

## 7.2 Core

### `src/core/engine_orchestrator.cpp`

책임:

- 내부 `Simulation_` lifecycle
- planner/renderer facade 구현 연결
- run loop
- complete 판정
- simulation 생성/파괴

이 파일은 내부 엔진의 “대장”이다.

중요 포인트:

- `Simulation_::run()`
- `Simulation_::executeOneStep()`
- `simulation_create()`
- `agv_run_to_completion()`

---

## 7.3 Runtime

### `src/runtime/step_runtime.cpp`

책임:

- 한 스텝 동안 필요한 서비스 orchestration
- planner 호출
- 이동 적용
- stuck/deadlock 반영
- AGV 상태 업데이트 연결

이 파일은 “한 턴 처리 파이프라인”이다.

### `src/runtime/scenario_runtime.cpp`

책임:

- interactive scenario setup
- custom/realtime 설정 적용
- task dispatch
- scenario mode 관련 초기화

시나리오를 어떻게 구성하고 적용하는지 알고 싶으면 여기다.

### `src/runtime/agent_runtime.cpp`

책임:

- AGV의 state transition
- 도착 후 상태 완료 처리
- charging state update
- runtime 관점의 AGV 상태 관리 orchestration

현재는 파일 크기를 줄이기 위해 goal assignment 구현을 별도 support로 뺐다.

### `src/runtime/agent_goal_support.cpp`

책임:

- goal selection policy
- charge station / parking / retrieval target 선택
- goal reservation
- missing goal fallback

`agent_runtime.cpp`보다 정책 중심이다.

즉:

- `agent_runtime.cpp` = 상태 전이 orchestration
- `agent_goal_support.cpp` = goal 결정 policy

---

## 7.4 Planning

### `src/planning/pathfinder.cpp`

책임:

- 단일 agent 경로 탐색기 구현
- D* Lite / 경로 관련 low-level 탐색 자료구조 관리

가장 low-level 알고리즘 구현 파일이다.

### `src/planning/collision_planner.cpp`

책임:

- planner facade
- 기본 planner / astar / dstar 실행 경로 선택
- top-level planner orchestration

현재는 support 분리를 거쳐 “가벼운 진입점 파일”이 됐다.

### `src/planning/collision_planner_default_support.cpp`

책임:

- default planner의 WHCA reservation 흐름
- first-step conflict 기록
- fallback 적용
- pairwise conflict 최종 정리

즉, default planner 본체 구현 디테일이 여기 있다.

### `src/planning/collision_planner_support.cpp`

책임:

- ordered planning support
- priority ordering
- temp obstacle scope
- ordered move 계산

planner 공통 보조 로직 파일이다.

### `src/planning/collision_planner_whca_support.cpp`

책임:

- reservation table
- wait edge
- SCC mask
- partial CBS
- pull-over fallback용 보조 로직

WHCA/CBS 관련 핵심 support 구현이다.

### `src/planning/collision_planner_default_support.hpp`
### `src/planning/collision_planner_support.hpp`
### `src/planning/collision_planner_whca_support.hpp`

각 support cpp의 외부 인터페이스를 선언한다.

읽는 법:

- `.hpp` 먼저 보고
- 어떤 책임이 export되는지 확인한 뒤
- `.cpp` 구현으로 들어가면 덜 힘들다.

---

## 7.5 UI / Platform

### `src/ui/simulation_display.cpp`

책임:

- 화면 문자열 조립
- grid/agent/log/control 섹션 렌더링
- display buffer 생성
- renderer facade 연결

현재는 `DisplayFrameComposer` 중심으로 읽으면 쉽다.

즉:

- 입력: `DisplayFrameContext`
- 출력: `display_buffer`

### `src/platform/console_terminal.cpp`

책임:

- 콘솔 초기화
- 화면 제어
- alt screen, cursor, terminal 관련 platform 함수

렌더링 정책보다는 콘솔 장치 제어에 가깝다.

---

## 7.6 Map / Reporting / Test

### `src/maps/map_catalog.cpp`

책임:

- 맵 layout 생성
- goal/charge station 배치
- agent 시작 위치 배치

시나리오별 레이아웃 정의서라고 보면 된다.

### `src/reporting/run_reporting.cpp`

책임:

- 실행 요약/리포트 관련 helper
- terminal summary 경로 지원

예전보다 리포트 체인은 많이 줄었고, 현재는 summary/support 성격이 강하다.

### `tests/simulation_engine_test.cpp`

책임:

- public API 기준 smoke test
- `SimulationEngine`가 실제로 동작하는지 검증

처음부터 내부 구현을 따라가기 힘들면, 이 테스트를 먼저 보는 것도 좋다.

---

## 8. 실제 호출 흐름 예시

### 8.1 headless 실행

1. `src/apps/console_main.cpp`
2. `runHeadless()`
3. `SimulationEngine::runUntilComplete()`
4. `agv_run_to_completion()`
5. `agv_execute_headless_step()`
6. `Simulation_::executeOneStep()`
7. `agv_execute_step_service()`
8. `Simulation_::planStep()`
9. planner strategy 호출
10. 이동/상태/메트릭 업데이트

### 8.2 interactive 실행

1. `src/apps/console_main.cpp`
2. `runInteractive()`
3. `interactiveSetup()`
4. `runInteractiveConsole()`
5. `Simulation_::run()`
6. 루프 안에서 `executeOneStep()`
7. 매 프레임 `renderer.drawFrame()`

---

## 9. 처음 읽을 때 추천 포인트

### 목적이 “전체 구조 파악”이면

- `console_main.cpp`
- `simulation_engine.hpp`
- `simulation_engine.cpp`
- `engine_orchestrator.cpp`
- `step_runtime.cpp`

### 목적이 “알고리즘 이해”이면

- `collision_planner.cpp`
- `collision_planner_default_support.cpp`
- `collision_planner_support.cpp`
- `collision_planner_whca_support.cpp`
- `pathfinder.cpp`

### 목적이 “AGV 상태 전이 이해”이면

- `agent_runtime.cpp`
- `agent_goal_support.cpp`
- `scenario_runtime.cpp`

### 목적이 “출력/UI 이해”이면

- `simulation_display.cpp`
- `console_terminal.cpp`

---

## 10. 읽을 때 팁

### 팁 1. 큰 파일부터 정면 돌파하지 말 것

바로 `pathfinder.cpp`나 planner support부터 들어가면 힘들다.  
먼저 `console_main -> simulation_engine -> engine_orchestrator -> step_runtime`로 큰 그림을 잡는 게 좋다.

### 팁 2. cpp만 보지 말고 헤더를 먼저 볼 것

특히 아래는 헤더를 먼저 보는 게 좋다.

- `include/agv/simulation_engine.hpp`
- `include/agv/sim_bridge.hpp`
- `src/planning/collision_planner_support.hpp`
- `src/planning/collision_planner_default_support.hpp`
- `src/planning/collision_planner_whca_support.hpp`
- `src/runtime/agent_goal_support.hpp`

헤더를 먼저 보면 “이 모듈이 바깥에 무엇을 제공하는가”가 먼저 보인다.

### 팁 3. 한 번에 모든 디테일을 이해하려 하지 말 것

이 프로젝트는

- 실행 orchestration
- planning
- runtime state
- display

가 분리되어 있으므로, 한 번에 하나씩 보면 된다.

---

## 11. 요약

이 프로젝트를 한 줄로 요약하면:

- `console_main.cpp`가 시작하고
- `SimulationEngine`가 public API wrapper 역할을 하고
- `engine_orchestrator.cpp`가 내부 엔진 lifecycle을 잡고
- `step_runtime.cpp`가 한 스텝을 실행하고
- planner/runtime/ui/map 모듈이 관심사별로 실제 일을 처리한다

처음 읽을 때 가장 추천하는 시작 루트는 이거다.

1. `src/apps/console_main.cpp`
2. `include/agv/simulation_engine.hpp`
3. `src/api/simulation_engine.cpp`
4. `src/core/engine_orchestrator.cpp`
5. `src/runtime/step_runtime.cpp`

그 다음 목적에 따라 planner/runtime/ui로 갈라지면 된다.
