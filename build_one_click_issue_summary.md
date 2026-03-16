# build_one_click.bat 오류 정리 및 Codex 작업 프롬프트

## 상황 요약

- `build_one_click.bat` 실행 시 C++ 소스 컴파일 전에 CMake configure 단계에서 실패함
- 실패 지점은 프로젝트 코드가 아니라 `FetchContent`로 받는 외부 의존성 다운로드 단계였음
- 협업자 환경에서는 기존 방식이 잘 동작하므로, 수정 시 **기본 동작을 깨지 않도록 범용성**을 고려해야 함

## 대표 에러 메시지

아래와 같은 메시지가 출력되면서 configure가 중단됨.

```text
mingw32-make[2]: *** [CMakeFiles/nlohmann_json-populate.dir/build.make:99:
nlohmann_json-populate-prefix/src/nlohmann_json-populate-stamp/nlohmann_json-populate-download] Error 1
mingw32-make[1]: *** [CMakeFiles/Makefile2:86: CMakeFiles/nlohmann_json-populate.dir/all] Error 2
mingw32-make: *** [Makefile:90: all] Error 2

CMake Error at .../Modules/FetchContent.cmake:1928 (message):
  Build step for nlohmann_json failed: 2
Call Stack (most recent call first):
  .../Modules/FetchContent.cmake:1619 (__FetchContent_populateSubbuild)
  .../Modules/FetchContent.cmake:2155:EVAL:2 (__FetchContent_doPopulation)
  .../Modules/FetchContent.cmake:2155 (cmake_language)
  .../Modules/FetchContent.cmake:2394 (__FetchContent_Populate)
  CMakeLists.txt:16 (FetchContent_MakeAvailable)

-- Configuring incomplete, errors occurred!
Exception: D:\01_active\AGV_REFACTORING\scripts\codex_build.ps1:117:9
```

## 실제 원인

원인은 `build_one_click.bat` 자체가 아니라, 그 안에서 호출하는 `scripts/codex_build.ps1` + `CMake FetchContent` 조합에 있었음.

핵심 포인트:

- `scripts/codex_build.ps1`는 일반 `build` 작업에서도 `-DBUILD_TESTING=ON`을 항상 넘기고 있었음
- `CMakeLists.txt`는 `nlohmann_json`를 `FetchContent`로 항상 다운로드함
- 테스트가 켜져 있으면 `googletest`도 추가 다운로드 대상이 됨
- 실패 당시 `build-codex/_deps/.../json.tar.xz` 파일이 `0`바이트였음
- 즉, **프로젝트 빌드 실패가 아니라 외부 dependency 다운로드 실패**였음

정리하면:

1. CMake configure 단계에서 GitHub로부터 `nlohmann_json`를 받으려 함
2. 로컬 환경에서 그 다운로드가 비정상 종료됨
3. 그래서 `FetchContent_MakeAvailable(nlohmann_json)`에서 중단됨
4. 결과적으로 `build_one_click.bat` 전체가 실패한 것처럼 보였음

## 협업 환경까지 고려한 해결 방향

협업자는 기존 방식으로 잘 된다고 했으므로, 가장 좋은 방향은 아래와 같음.

- 기본값은 기존 방식 유지
- 문제 있는 환경에서만 fallback 사용
- 일반 `build`에서는 테스트 dependency를 굳이 받지 않도록 분리

권장 설계:

1. `DependencyMode` 개념 추가
   - `Auto`
   - `FetchContent`
   - `Prefetch`
   - `Offline`

2. `Auto` 동작
   - 먼저 기존 `FetchContent` 방식으로 configure 시도
   - `nlohmann_json` 또는 `googletest` 다운로드 실패 시 `Prefetch` 방식으로 재시도

3. `Prefetch` 동작
   - PowerShell이 외부 archive를 먼저 다운로드
   - 로컬에 압축 해제
   - CMake에는 `FETCHCONTENT_SOURCE_DIR_*`를 넘겨서 로컬 소스를 사용하게 함

4. `build`와 `test` 분리
   - `build`에서는 `BUILD_TESTING=OFF`
   - `test`에서만 `googletest` 처리

## 빠른 수정안

최소 수정 기준으로는 아래 두 가지가 핵심임.

### 1. 일반 build에서 테스트 끄기

기존:

```text
-DBUILD_TESTING=ON
```

권장:

```text
build -> BUILD_TESTING=OFF
test  -> BUILD_TESTING=ON
```

### 2. 외부 dependency를 직접 FetchContent에만 의존하지 않도록 만들기

권장 방법:

- `nlohmann_json`는 사전 다운로드 + 로컬 압축 해제 지원
- 필요 시 `googletest`도 동일 방식 지원
- CMake configure 시:

```text
-DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=<local_path>
-DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<local_path>
```

## 수정 후 기대 결과

- 네트워크가 불안정한 환경에서도 `build_one_click.bat`가 성공 가능
- 협업자 환경에서는 기존 FetchContent 경로를 계속 사용 가능
- 일반 빌드에서 불필요한 `googletest` 다운로드 제거
- 문제 재현 시에도 원인 추적이 쉬워짐

## 검증 방법

아래 순서로 검증하면 됨.

### 1. configure만 먼저 확인

```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex_build.ps1 -Task configure -BuildType Debug
```

### 2. one-click 빌드 확인

```bat
build_one_click.bat
```

### 3. 성공 기준

- configure 단계가 끝까지 완료됨
- `build-codex`에 빌드 파일 생성됨
- 최종적으로 아래 exe가 생성됨
  - `build-codex\agv_console.exe`
  - `build-codex\agv_render_ipc.exe`

## Codex에게 맡길 때 사용할 프롬프트

아래 프롬프트를 그대로 사용하면 됨.

```text
`build_one_click.bat` 실행 시 CMake configure 단계에서 `FetchContent_MakeAvailable(nlohmann_json)`가 실패합니다.

현재 확인된 증상:
- 에러는 프로젝트 소스 컴파일 단계가 아니라 external dependency download 단계에서 발생
- `nlohmann_json-populate` download step 실패
- `build-codex/_deps/.../json.tar.xz`가 0바이트였음
- 협업자 환경에서는 기존 방식이 잘 동작하므로, 수정 시 기본 동작을 깨면 안 됨

목표:
1. `scripts/codex_build.ps1`를 범용적으로 개선
2. 기본 동작은 기존 `FetchContent` 방식과 호환되게 유지
3. 문제 있는 환경에서는 fallback으로 local prefetch 방식을 사용할 수 있게 구현
4. 일반 `build`에서는 `BUILD_TESTING=OFF`, `test`에서만 `BUILD_TESTING=ON` 되게 수정
5. 필요하면 `DependencyMode` 파라미터(`Auto`, `FetchContent`, `Prefetch`, `Offline`)를 추가
6. `Auto`에서는 기존 방식 configure 실패 시 prefetch 방식으로 재시도
7. 수정 후 `-Task configure`와 `build_one_click.bat` 기준으로 검증

작업 지침:
- 먼저 `build_one_click.bat`, `scripts/codex_build.ps1`, `CMakeLists.txt`를 읽고 현재 dependency 흐름을 정리
- 불필요하게 기존 동작을 깨지 말 것
- 하드코딩을 늘리기보다 파라미터/환경변수로 override 가능한 구조를 선호
- unrelated 변경은 건드리지 말 것
- 수정 후 원인, 변경사항, 검증 결과를 간단히 정리할 것

산출물:
- 수정된 PowerShell build script
- 필요 시 최소한의 CMake 보완
- 최종적으로 왜 실패했고 어떻게 범용적으로 해결했는지 설명
```

## 한 줄 결론

이 이슈는 프로젝트 코드 오류가 아니라 `FetchContent` 기반 외부 dependency 다운로드 실패이며, 협업 환경을 고려하면 **기존 경로 유지 + 실패 환경에서만 fallback하는 구조**로 정리하는 것이 가장 안전함.
