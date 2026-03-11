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

## 아직 안 끝난 부분

다음 항목들은 아직 “진행 가능성이 큰 후속 리팩토링 대상”입니다.

### 1. `collision_planner.cpp`

- 파일 규모가 가장 큼
- 알고리즘 자체는 유지하되
  - reservation
  - CBS 관련 처리
  - WHCA 보조 처리
  - 충돌 해소 유틸
  로직을 더 작은 단위로 분리할 필요가 큼

### 2. `simulation_display.cpp`

- 렌더링 문자열 조립/flush/UI 표시 책임이 많이 섞여 있음
- 렌더링 성능 및 가독성 관점에서 분해 여지 큼

### 3. `agent_runtime.cpp`

- 상태 전이와 메트릭 누적 로직이 붙어 있음
- state handler 형태로 더 나눌 수 있음

### 4. `map_catalog.cpp`

- 맵 생성 함수군이 길고 응집도가 낮은 편
- 공통 패턴 추출 여지가 큼

## 이번 리팩토링의 의미

이번 작업은 “전체 코드베이스 완전 리팩토링 완료”는 아닙니다.

대신 다음 의미는 분명합니다.

- 구조를 건드리지 않은 척하는 정리가 아니라 실제 로직 경계를 분리함
- 상태 초기화 / 설정 적용 / 실행 루프 / 래퍼 lifecycle을 실질적으로 개선함
- 성능에 직접 영향 있는 반복 할당 하나를 실제로 제거함
- 빌드와 실행을 실제 Windows `g++`로 검증함
- 이후 대형 파일 리팩토링을 진행할 수 있는 기반을 만듦

## 추천 다음 단계

우선순위 추천:

1. `src/planning/collision_planner.cpp` 내부 역할 분해
2. `src/ui/simulation_display.cpp` 렌더링 조립 책임 분리
3. `src/runtime/agent_runtime.cpp` 상태 전이 handler 분리

가장 추천하는 다음 작업은:

- **알고리즘은 유지한 채 `collision_planner.cpp`를 기능군 단위로 분할하는 것**

이 작업이 끝나면 코드베이스 전체 난이도가 크게 내려갈 가능성이 높습니다.
