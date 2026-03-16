# AGV 시뮬레이션 Base C++ 알고리즘 분석 보고서

## 1. 분석 범위

이 보고서는 이 저장소의 "시뮬레이션에 실제로 사용되는 base C++ 코드"를 기준으로 알고리즘을 분석한 문서다.  
분석 대상은 UI/Electron이 아니라 아래 엔진 코어 계층이다.

- 엔진 진입점: `src/api/simulation_engine.cpp`
- 시뮬레이션 루프/계측: `src/core/simulation_runtime.cpp`
- step 오케스트레이션: `src/runtime/step_runtime.cpp`
- task dispatch / scenario: `src/runtime/scenario_runtime.cpp`
- goal assignment / 상태 전이: `src/runtime/agent_goal_support.cpp`, `src/runtime/agent_runtime.cpp`
- planner 계층: `src/planning/collision_planner.cpp`
- ordered planner 공통층: `src/planning/collision_planner_support.cpp`
- default hybrid planner: `src/planning/collision_planner_default_support.cpp`
- WHCA / CBS / pull-over 지원: `src/planning/collision_planner_whca_support.cpp`
- pathfinder 본체: `src/planning/pathfinder.cpp`

핵심 결론부터 말하면, 이 엔진은 단순한 "최단 경로 1회 계산기"가 아니라:

1. 시나리오에서 task를 생성/배정하고
2. 각 step마다 goal을 다시 확정한 뒤
3. planner가 다중 AGV 충돌을 완화하며 next move를 계산하고
4. 이동 이후 상태 전이, deadlock 판정, metrics 수집까지 한 번에 처리하는

"step 기반 운영형 AGV 시뮬레이션 엔진"이다.

---

## 2. 호출 체인 요약

실제 1-step 실행 경로는 다음 순서다.

1. `agv::core::SimulationEngine::step()`  
   위치: `src/api/simulation_engine.cpp:719-721`
2. `execute_headless_step()`  
   위치: `src/core/simulation_runtime.cpp:319-326`
3. `Simulation_::executeOneStep()`  
   위치: `src/core/simulation_runtime.cpp:302-304`
4. `agv_execute_step_service()`  
   위치: `src/runtime/step_runtime.cpp:781-783`
5. `StepExecutorService::execute()`  
   위치: `src/runtime/step_runtime.cpp:513-531`

즉, 외부 API는 `SimulationEngine`이지만 실제 알고리즘 오케스트레이션은 `StepExecutorService`가 담당한다.

---

## 3. 엔진 전체 알고리즘 구조

### 3.1 세션 시작과 설정 반영

시뮬레이션 설정 반영은 `SimulationEngine::Impl::rebuildSimulationIfNeeded()`에서 시작된다.

- `validateLaunchConfig()`  
  위치: `src/api/simulation_engine.cpp:497-574`  
  역할: 맵 ID, speed multiplier, realtime 확률, phase 개수/값 검증
- `Impl::rebuildSimulationIfNeeded()`  
  위치: `src/api/simulation_engine.cpp:604-624`  
  역할: dirty 상태면 `Simulation`을 새로 만들고 `apply_simulation_config()` 호출
- `apply_simulation_config()`  
  위치: `src/runtime/scenario_runtime.cpp:274-310`  
  역할:
  - 맵 로드
  - planner 선택
  - custom/realtime 시나리오 설정
  - 속도 배율 적용
  - runtime 통계 초기화

이 단계에서 planner는 `planner_from_pathalgo()`로 결정된다.

- 위치: `src/planning/collision_planner.cpp:281-290`
- 분기:
  - `PathAlgo::AStarSimple`
  - `PathAlgo::DStarBasic`
  - `PathAlgo::Default`

### 3.2 step 시작 전 시나리오 / task 준비

step 안에서 task dispatch는 `TaskDispatchService`가 맡는다.

- `TaskDispatchService::update()`  
  위치: `src/runtime/scenario_runtime.cpp:141-152`
- 내부 핵심 함수:
  - custom phase 진행: `advance_custom_phase_if_needed()` at `src/runtime/scenario_runtime.cpp:155-174`
  - realtime 요청 생성: `generate_realtime_requests()` at `src/runtime/scenario_runtime.cpp:176-206`
  - idle AGV에 task 배정: `assign_idle_agents()` at `src/runtime/scenario_runtime.cpp:208-253`

배정 규칙의 핵심은 다음과 같다.

- 누적 이동거리가 임계값 이상이면 먼저 charging으로 보냄  
  위치: `src/runtime/scenario_runtime.cpp:218-226`
- custom 모드는 현재 phase 목표 수와 이미 투입된 active agent 수를 함께 고려
- realtime 모드는 queue에서 assign 가능한 task만 꺼냄  
  위치: `src/runtime/scenario_runtime.cpp:245-251`

task 상태 전환 시작점은 아래 두 함수다.

- `agent_begin_task_park()` at `src/runtime/scenario_runtime.cpp:15-26`
- `agent_begin_task_exit()` at `src/runtime/scenario_runtime.cpp:28-39`

둘 다 공통적으로:

- AGV 상태 변경
- task metrics 시작 시각/거리 저장
- structured log 남김

### 3.3 goal assignment 알고리즘

planner가 움직이기 전에 각 AGV는 "지금 무엇을 목표로 갈지"를 먼저 확정한다.

- entry: `agent_runtime_assign_goal_if_needed()`  
  위치: `src/runtime/agent_goal_support.cpp:357-378`
- 실제 정책 객체: `GoalAssignmentPolicyLocal`  
  위치: `src/runtime/agent_goal_support.cpp:190-280`

핵심 로직:

- 현재 상태를 goal type으로 변환  
  `goal_type_for_state_local()` at `src/runtime/agent_goal_support.cpp:173-188`
- 후보 goal 집합 생성  
  `make_goal_candidate_set_local()` at `src/runtime/agent_goal_support.cpp:85-106`
- 각 후보에 대해 pathfinder 비용 계산  
  `PathCostEvaluatorLocal::compute()` at `src/runtime/agent_goal_support.cpp:56-71`
- 최저 비용 후보 선택  
  `select_best_candidate_local()` at `src/runtime/agent_goal_support.cpp:129-151`

상태별 목표는 다음처럼 매핑된다.

- `GoingToPark` -> 빈 parking goal
- `GoingToCollect` -> parked vehicle가 있는 goal
- `GoingToCharge` -> charge station
- `ReturningHomeEmpty`, `ReturningWithCar`, `ReturningHomeMaintenance` -> home base

추가 운영 규칙도 있다.

- 귀가 중 이동거리 임계 초과 시 charge 모드 전환  
  `maybe_switch_to_charge_mode_local()` at `src/runtime/agent_goal_support.cpp:330-342`
- maintenance 복귀 AGV가 오래 막히면 임시 holding goal 부여 시도  
  `resolveGoalForCurrentState()` at `src/runtime/agent_goal_support.cpp:195-206`
  `selectTemporaryHoldingGoal()` at `src/runtime/agent_goal_support.cpp:241-252`

즉, 이 엔진의 goal assignment는 "고정 goal"이 아니라 step마다 재평가되는 동적 정책이다.

---

## 4. 1-step 실행 파이프라인

실제 step 오케스트레이션은 `StepExecutorService::execute()`가 담당한다.

- 위치: `src/runtime/step_runtime.cpp:513-531`

세부 순서는 아래와 같다.

1. step frame 시작 정보 수집  
   `begin_frame()` at `src/runtime/step_runtime.cpp:535-550`
2. charging 상태 갱신  
   `agent_manager_update_charge_state()` 호출 at `src/runtime/step_runtime.cpp:517`
3. step 시작 시 idle agent 기록  
   `mark_idle_agents_at_step_start_local()` at `src/runtime/step_runtime.cpp:264-278`
4. task dispatch 실행  
   `agv_update_task_dispatch()` 호출 at `src/runtime/step_runtime.cpp:520`
5. planner로 next position 후보 계산  
   `prepare_movement_plan()` at `src/runtime/step_runtime.cpp:552-607`
6. 실제 이동 적용 및 stuck/oscillation 갱신  
   `agv_apply_moves_and_update_stuck()` at `src/runtime/step_runtime.cpp:693-735`
7. 이동 후 goal 도달 상태 처리  
   `finalize_move_state()` at `src/runtime/step_runtime.cpp:651-657`
8. ordered planner preview overlay 갱신  
   `agv_refresh_ordered_planner_overlay_preview()` 호출 at `src/runtime/step_runtime.cpp:529`
9. idle 유지 agent 집계  
   `record_idle_agents_that_remained_idle_local()` at `src/runtime/step_runtime.cpp:280-293`
10. CPU 시간, deadlock, backlog, render frame, memory sample 반영  
    `finalize_frame()` at `src/runtime/step_runtime.cpp:659-686`

이 함수가 사실상 "시뮬레이션의 한 step 전체 알고리즘"이다.

### 4.1 planner 후보 이동 후 후처리

`prepare_movement_plan()`은 planner 결과를 바로 적용하지 않고 세 단계 후처리를 넣는다.

- rotation stage  
  `apply_rotation_stage()` at `src/runtime/step_runtime.cpp:609-624`
- stationary blocker 해소  
  `resolve_stationary_blockers()` at `src/runtime/step_runtime.cpp:626-649`
- 우선순위 기반 최종 conflict 정리  
  `resolve_conflicts_by_order_local()` at `src/runtime/step_runtime.cpp:72-114`

또한 pipeline별로 취소된 agent ID까지 추적한다.

- `record_pipeline_cancellations_local()`  
  위치: `src/runtime/step_runtime.cpp:322-343`

이 때문에 debug snapshot의 `plannedMoveCount`, `postRotationMoveCount`, `postBlockerMoveCount`, `finalMoveCount`가 실제 알고리즘 pipeline의 각 단계를 반영한다.

---

## 5. Ordered Planner (`astar`, `dstar`) 알고리즘

### 5.1 실제 entry point

- `OrderedPlannerBase::planStep()`  
  위치: `src/planning/collision_planner.cpp:189-194`
- `OrderedPlannerBase::runOrderedPlanning()`  
  위치: `src/planning/collision_planner.cpp:197-229`

동작 순서:

1. active agent의 goal 보정  
   `assign_goals_for_active_agents()` at `src/planning/collision_planner.cpp:55-59`
2. `next_positions`를 현재 위치로 초기화  
   `seed_next_positions_from_current()` at `src/planning/collision_planner.cpp:61-65`
3. priority score 기준으로 AGV 순서 정렬  
   `sort_agents_by_priority()` at `src/planning/collision_planner_support.cpp:191-199`
4. 각 AGV에 대해 pathfinder 준비, 후보 이동 계산, rotation 반영
5. 마지막에 `ConflictResolutionPolicy::resolve()`로 vertex/swap 정리  
   위치: `src/planning/collision_planner_support.cpp:335-375`

### 5.2 우선순위 규칙

- `priority_score()`  
  위치: `src/planning/collision_planner_support.cpp:144-152`

우선순위 특징:

- `ReturningWithCar`가 가장 높음
- charging / maintenance 복귀가 그 다음
- 일반 이동 task가 그 다음
- `stuck_steps`가 많을수록 강한 보정치 부여
- agent id를 빼서 동점 시 낮은 ID를 약간 우선

즉, 단순 shortest-path보다 "운영 우선순위"가 먼저다.

### 5.3 후보 이동 랭킹

- `OrderedMoveRankingPolicy::rank()`  
  위치: `src/planning/collision_planner_support.cpp:90-142`

후보는 다음 5개다.

- 제자리 대기
- 상
- 하
- 좌
- 우

정렬 기준은 다음 순서다.

1. `1 + successor_g + backtrack_penalty`
2. 즉시 backtrack 여부
3. goal까지의 Manhattan distance

즉, 이 planner는 "최단 경로 1칸 전진"만 보지 않고, 바로 직전 칸으로 튕기는 진동을 비용으로 억제한다.

### 5.4 임시 장애물 / 임시 goal 완화

- `TempObstacleScope::markOrderBlockers()`  
  위치: `src/planning/collision_planner_support.cpp:254-268`
- `TemporaryGoalStateScope` 생성/복구  
  위치: `src/planning/collision_planner_support.cpp:270-290`

의미:

- 높은 우선순위 AGV의 예약 칸을 임시 장애물로 본다.
- 낮은 우선순위 AGV의 현재 위치도 가상 blocker로 본다.
- `GoingToCollect`는 parked goal에 진입하기 위해 planning 동안 잠시 `is_parked=false`로 완화한다.

### 5.5 구현상 중요한 관찰

코드 기준으로 보면 `astar`와 `dstar`는 현재 완전히 다른 searcher를 쓰지 않는다.

- `AStarOrderedPlanner` / `DStarOrderedPlanner`  
  위치: `src/planning/collision_planner.cpp:237-255`
- 둘 다 `OrderedPlannerBase`를 그대로 상속
- `OrderedPlannerToolkit::preparePathfinder()` / `computeDesiredMove()`  
  위치: `src/planning/collision_planner_support.cpp:181-189`, `301-309`

특히 `computeDesiredMove()`는 `metric_kind_`를 사용하지 않고 공통 `Pathfinder`를 호출한다.  
즉, 현재 구현 기준으로 `astar`와 `dstar`의 차이는 "별도 search 알고리즘 분기"보다는 메트릭/overlay 분류에 더 가깝다.

---

## 6. Pathfinder 핵심 알고리즘

`Pathfinder`는 이 저장소 planner의 공통 기반이며, 코드 형태는 D* Lite 스타일의 incremental search에 가깝다.

### 6.1 핵심 함수

- `Pathfinder::updateStart()`  
  위치: `src/planning/pathfinder.cpp:53-64`
- `Pathfinder::computeShortestPath()`  
  위치: `src/planning/pathfinder.cpp:78-125`
- `Pathfinder::getNextStep()`  
  위치: `src/planning/pathfinder.cpp:128-164`
- `Pathfinder::updateVertex()`  
  위치: `src/planning/pathfinder.cpp:316-345`
- `Pathfinder::resetCoreState()`  
  위치: `src/planning/pathfinder.cpp:355-360`

### 6.2 왜 D* Lite 스타일인가

다음 코드 특징이 보인다.

- `g`와 `rhs`를 같이 유지  
  `makeInactiveSearchCell()` at `src/planning/pathfinder.cpp:33-42`
- key가 2차원 (`k1`, `k2`)  
  `compareKeys()` at `src/planning/pathfinder.cpp:21-27`
- start가 바뀔 때 `km_ += heuristic(last_start_, new_start)`  
  `updateStart()` at `src/planning/pathfinder.cpp:61-63`

이 패턴은 A*보다는 incremental replanning 계열의 전형에 가깝다.

### 6.3 실제 next-step 선택 방식

`computeShortestPath()`는 전체 경로를 바로 반환하지 않는다.  
대신 `g/rhs`를 정리한 뒤, `getNextStep()`에서 현재 위치의 4방향 이웃을 살피며 가장 좋은 successor를 고른다.

- successor가 blocked인지 검사  
  `src/planning/pathfinder.cpp:145-147`
- 비용은 `1 + g(successor)`  
  `src/planning/pathfinder.cpp:148-153`
- 동률이면 goal에 더 가까운 쪽 선택  
  `src/planning/pathfinder.cpp:154-159`

즉, planner 관점에서 `Pathfinder`는 "전체 path 배열 생성기"가 아니라 "현재 상태에서 다음 한 칸을 뽑기 위한 value function 계산기"에 가깝다.

---

## 7. Default Planner 하이브리드 알고리즘

`default`는 이 프로젝트의 가장 중요한 운영용 planner다.

### 7.1 entry point

- `DefaultPlannerStrategy::planStep()`  
  위치: `src/planning/collision_planner.cpp:257-277`
- `DefaultPlannerSession::execute()`  
  위치: `src/planning/collision_planner_default_support.cpp:883-928`

실행 순서:

1. scratch / reservation table 초기화
2. planner overlay를 `PathAlgo::Default`와 현재 horizon으로 초기화
3. `WhcaPlanner`로 horizon 기반 path 예약
4. first-step rotation 적용
5. wait-edge graph 분석
6. CBS 또는 pull-over fallback 수행
7. first-step conflict 세부 규칙으로 정리
8. 필요 시 post-conflict standstill 추가 해소
9. conflict 정도에 따라 horizon 자동 조정

### 7.2 WHCA planning

- `WhcaPlanner::plan()`  
  위치: `src/planning/collision_planner_default_support.cpp:280-300`
- `WhcaPlanner::planPathForAgent()`  
  위치: `src/planning/collision_planner_default_support.cpp:314-376`

핵심 구조:

- reservation table 기반 time-expanded 계획
- agent priority 순서로 path 예약
- 각 step마다 후보를 순회하면서
  - 같은 시각 같은 칸 점유 충돌
  - swap 충돌
  - goal 도달 후 tail reservation
  을 처리

reservation table 자체는 아래에 있다.

- `ReservationTable::seedCurrent()` at `src/planning/collision_planner_whca_support.cpp:791-799`
- `ReservationTable::isOccupied()` at `src/planning/collision_planner_whca_support.cpp:801-804`
- `ReservationTable::setOccupant()` at `src/planning/collision_planner_whca_support.cpp:816-821`

즉, `default`는 "현재 step만" 보지 않고, horizon 길이의 시간축 위에 AGV 점유 계획을 예약한다.

### 7.3 conflict graph 분석

- `ConflictGraphAnalyzer::analyze()`  
  위치: `src/planning/collision_planner_default_support.cpp:396-400`
- `recordFirstStepConflicts()`  
  위치: `src/planning/collision_planner_default_support.cpp:404-435`
- `analyze_conflict_graph()`  
  위치: `src/planning/collision_planner_whca_support.cpp:856-890`

알고리즘:

- wait edge를 방향 그래프로 만들고
- Floyd-Warshall 형태의 reachability 계산을 돌려
- 상호 도달 가능한 agent 쌍을 SCC 충돌 집합으로 묶는다

즉, deadlock을 단순 "막힘"이 아니라 "wait-for graph cycle"로 해석한다.

### 7.4 Partial CBS

- `run_partial_CBS()`  
  위치: `src/planning/collision_planner_whca_support.cpp:934-1116`

내부 핵심:

- 그룹 AGV만 분리해 ext occupancy를 구성  
  `copy_ext_occ_without_group()` at `src/planning/collision_planner_whca_support.cpp:620-633`
- 각 agent에 대해 space-time A* 실행  
  `st_astar_plan_single()` at `src/planning/collision_planner_whca_support.cpp:348-511`
- 첫 conflict 탐지  
  `detect_first_conflict()` at `src/planning/collision_planner_whca_support.cpp:545-594`
- conflict마다 제약을 분기해서 CBS node 확장
- 예산 내 해를 찾지 못하면 실패 처리

중요한 세부점:

- 첫 step 회전 비용까지 포함  
  `st_astar_plan_single()` at `src/planning/collision_planner_whca_support.cpp:454-463`
- 해를 찾더라도 첫 step에 실제 진전이 없으면 채택하지 않음  
  `tryAcceptCbsSolution()` at `src/planning/collision_planner_default_support.cpp:490-528`

즉, 이 CBS는 "이론적 충돌 해소"보다 "바로 이번 step에서 진전이 있는가"를 더 중시한다.

### 7.5 Pull-over fallback

- `try_pull_over()`  
  위치: `src/planning/collision_planner_whca_support.cpp:893-932`
- 더 강한 탐색 버전: `try_pull_over_via_search()`  
  위치: `src/planning/collision_planner_whca_support.cpp:166-249`
- fallback orchestration: `applyPullOverFallbackForMask()`  
  위치: `src/planning/collision_planner_default_support.cpp:530-568`

동작 개념:

- leader 1대는 재계획하여 전진시킴
- 나머지 AGV는 주변 bay/goal 쪽으로 pull-over 또는 제자리 대기
- 복귀 AGV가 과하게 막히면 home goal reservation을 풀고 탈출 가능하게 완화

이 fallback은 "교통 흐름 회복"이 목표이지, 모든 AGV의 최단 경로 보장은 목표가 아니다.

### 7.6 마지막 first-step conflict 규칙

- `FirstStepConflictResolver::resolve()`  
  위치: `src/planning/collision_planner_default_support.cpp:689-717`
- vertex conflict 규칙  
  `resolveVertexConflict()` at `src/planning/collision_planner_default_support.cpp:775-818`
- swap conflict 규칙  
  `resolveSwapConflict()` at `src/planning/collision_planner_default_support.cpp:820-866`

규칙 우선순위:

1. deadlock leader 우선
2. pull-over/fallback 지정 agent 반영
3. escape move 반영
4. parking flow vs returning-home 흐름 특례
5. 일반 priority score 비교

즉, `default`는 WHCA/CBS만으로 끝나지 않고, 마지막 1-step 예외 처리 계층까지 별도로 둔 hybrid planner다.

### 7.7 동적 horizon 조정

- `WHCA_adjustHorizon()`  
  위치: `src/planning/collision_planner_whca_support.cpp:823-854`

규칙:

- conflict score = 이전 conflict score의 감쇠값 + 현재 wait edge 수 + SCC 보정
- 충돌이 심하면 horizon 증가
- 충돌이 낮으면 horizon 감소

즉, `default`는 정적 horizon이 아니라 혼잡도 적응형 planner다.

---

## 8. 이동 후 상태 전이 알고리즘

실제 move 적용과 상태 전이는 planner 이후 단계에서 수행된다.

### 8.1 이동 적용 / stuck / oscillation

- `agv_apply_moves_and_update_stuck()`  
  위치: `src/runtime/step_runtime.cpp:693-735`

이 함수는:

- 실제 position 갱신
- 이동 거리 및 total movement cost 누적
- immediate backtrack이면 `oscillation_steps` 증가
- 이동하지 못했거나 진동하면 `stuck_steps` 증가

즉, planner 우선순위와 deadlock 해소의 다음 입력값이 여기서 다시 만들어진다.

### 8.2 goal 도달 후 상태 전이

- `AgentManager::updateStateAfterMove()`  
  위치: `src/runtime/agent_runtime.cpp:275-291`
- 상태별 완료 처리 분기  
  `complete_agent_goal_local()` at `src/runtime/agent_runtime.cpp:217-247`

대표 함수:

- parking 완료: `finish_parking_goal_local()` at `src/runtime/agent_runtime.cpp:146-162`
- 빈 차 귀가 완료: `finish_return_home_empty_local()` at `src/runtime/agent_runtime.cpp:164-175`
- 출차 차량 회수 완료: `finish_collect_goal_local()` at `src/runtime/agent_runtime.cpp:177-185`
- 회수 후 복귀 완료: `finish_return_with_car_local()` at `src/runtime/agent_runtime.cpp:187-194`
- charging 도착/완료: `finish_charge_arrival_local()` at `src/runtime/agent_runtime.cpp:196-204`, `finish_charging_cycle_local()` at `src/runtime/agent_runtime.cpp:261-271`

또한 `GoingToPark` / `GoingToCollect`는 도착 즉시 완료되지 않고 action timer를 거친다.

- `advance_goal_action_timer_local()`  
  위치: `src/runtime/agent_runtime.cpp:109-123`

---

## 9. Deadlock 판정과 디버그 데이터

### 9.1 deadlock counter

- `agv_update_deadlock_counter()`  
  위치: `src/runtime/step_runtime.cpp:737-772`

판정 흐름:

- step에 실제 이동이 있으면 streak 리셋
- unresolved task가 없으면 리셋
- goal action 중이면 리셋
- 그 외에는 `no_movement_streak` 증가
- 임계치 도달 시 `deadlock_count++` 및 deadlock event 기록

### 9.2 deadlock event 기록

- `record_deadlock_event_local()`  
  위치: `src/runtime/step_runtime.cpp:380-440`
- reason 생성  
  `build_deadlock_reason_local()` at `src/runtime/step_runtime.cpp:357-378`

기록되는 항목:

- step / phase / pending task 수
- planner wait edges / SCC / CBS 성공 여부 / CBS expansion 수
- planned/post-rotation/post-blocker/final move count
- rotation/blocker/order 단계에서 취소된 agent ID
- deadlock participant agent ID

즉, 이 엔진은 deadlock을 단순 카운트만 하지 않고 "왜 발생했는지"를 step-level 포렌식 데이터로 남긴다.

### 9.3 planner/runtime metrics 집계

- `Simulation_::planStep()`  
  위치: `src/core/simulation_runtime.cpp:282-296`
- `record_planner_step_results()`  
  위치: `src/core/simulation_runtime.cpp:95-126`
- `collectRuntimeDebugSnapshot()`  
  위치: `src/api/simulation_engine.cpp:384-447`

이 계층 덕분에 UI나 debug snapshot에서:

- planning time
- nodes expanded
- wait edge / cycle / CBS stats
- move pipeline 카운트
- backlog / idle / stuck agent 수

를 바로 볼 수 있다.

---

## 10. 주요 코드 위치 인덱스

| 역할 | 파일 | 줄번호 | 핵심 함수 |
|---|---|---:|---|
| 외부 시뮬레이션 API | `src/api/simulation_engine.cpp` | 604-624 | `Impl::rebuildSimulationIfNeeded` |
| 1-step public entry | `src/api/simulation_engine.cpp` | 719-721 | `SimulationEngine::step` |
| headless step 루프 | `src/core/simulation_runtime.cpp` | 319-326 | `execute_headless_step` |
| planner 수행 + metrics | `src/core/simulation_runtime.cpp` | 282-296 | `Simulation_::planStep` |
| step 오케스트레이션 | `src/runtime/step_runtime.cpp` | 513-531 | `StepExecutorService::execute` |
| movement pipeline | `src/runtime/step_runtime.cpp` | 552-607 | `prepare_movement_plan` |
| move 적용/stuck 계산 | `src/runtime/step_runtime.cpp` | 693-735 | `agv_apply_moves_and_update_stuck` |
| deadlock 판정 | `src/runtime/step_runtime.cpp` | 737-772 | `agv_update_deadlock_counter` |
| task dispatch | `src/runtime/scenario_runtime.cpp` | 141-152 | `TaskDispatchService::update` |
| idle agent task 배정 | `src/runtime/scenario_runtime.cpp` | 208-253 | `assign_idle_agents` |
| goal assignment | `src/runtime/agent_goal_support.cpp` | 357-378 | `agent_runtime_assign_goal_if_needed` |
| goal 선택 정책 | `src/runtime/agent_goal_support.cpp` | 190-280 | `GoalAssignmentPolicyLocal` |
| ordered planner 본체 | `src/planning/collision_planner.cpp` | 197-229 | `OrderedPlannerBase::runOrderedPlanning` |
| agent priority | `src/planning/collision_planner_support.cpp` | 144-152 | `priority_score` |
| candidate ranking | `src/planning/collision_planner_support.cpp` | 90-142 | `OrderedMoveRankingPolicy::rank` |
| incremental pathfinder | `src/planning/pathfinder.cpp` | 78-125 | `Pathfinder::computeShortestPath` |
| next-step 추출 | `src/planning/pathfinder.cpp` | 128-164 | `Pathfinder::getNextStep` |
| default hybrid planner | `src/planning/collision_planner_default_support.cpp` | 883-928 | `DefaultPlannerSession::execute` |
| WHCA reservation planning | `src/planning/collision_planner_default_support.cpp` | 280-376 | `WhcaPlanner::plan`, `planPathForAgent` |
| conflict graph | `src/planning/collision_planner_whca_support.cpp` | 856-890 | `analyze_conflict_graph` |
| partial CBS | `src/planning/collision_planner_whca_support.cpp` | 934-1116 | `run_partial_CBS` |
| pull-over fallback | `src/planning/collision_planner_whca_support.cpp` | 893-932 | `try_pull_over` |
| post-move state transition | `src/runtime/agent_runtime.cpp` | 275-291 | `AgentManager::updateStateAfterMove` |

---

## 11. 최종 정리

이 시뮬레이션의 base C++ 알고리즘은 아래처럼 이해하는 것이 가장 정확하다.

- 외형상은 `SimulationEngine` API로 감싸져 있지만, 실제 알고리즘의 중심은 `StepExecutorService`
- 경량 모드인 `astar`, `dstar`는 priority-ordered next-step planner
- 공통 기반 탐색기는 `Pathfinder`이며, 구현은 incremental replanning 성격이 강함
- 운영 모드인 `default`는 WHCA reservation + wait-for graph + partial CBS + pull-over + first-step rule resolver를 순차 결합한 hybrid planner
- planner 결과는 그대로 끝나지 않고, move 적용 후 stuck/oscillation/state transition/deadlock metrics까지 다음 step의 입력으로 다시 환류됨

즉, 이 엔진의 본질은 "최단 경로 찾기"가 아니라 "복수 AGV의 교통 흐름을 step 단위로 안정적으로 운영하는 제어 알고리즘"이다.
