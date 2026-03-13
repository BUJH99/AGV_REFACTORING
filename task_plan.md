# Task Plan

## Goal
Fix the `build_one_click.bat` failure by identifying the root cause, updating the build flow, and documenting the explanation.

## Phases
- [x] Inspect the batch script, PowerShell build script, CMake setup, and existing build artifacts.
- [x] Update the build script so dependency fetching is more reliable and unnecessary test downloads do not run for a normal build.
- [x] Validate the script changes with configure and a full `build_one_click.bat` run, then summarize the cause and fix.

## Errors Encountered
- `FetchContent_MakeAvailable(nlohmann_json)` fails during the CMake configure stage while downloading from GitHub.
- The generated `json.tar.xz` artifact in `build-codex/_deps/...` is `0` bytes, which matches an interrupted or failed download.
- The first extraction implementation used `$LASTEXITCODE` under `Set-StrictMode`, which caused a runtime error and was replaced with `Start-Process ... -PassThru`.
