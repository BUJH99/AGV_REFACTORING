# Findings

- `build_one_click.bat` defaults to `scripts/codex_build.ps1 -Task build -BuildType Debug`.
- `scripts/codex_build.ps1` always passes `-DBUILD_TESTING=ON`, even for a normal build.
- `CMakeLists.txt` uses `FetchContent` for `nlohmann_json` unconditionally and for `googletest` when `BUILD_TESTING` is enabled.
- The failure happens in the `nlohmann_json` populate subbuild before project compilation starts.
- The downloaded archive path exists but is empty, which indicates the configure step depends on a fragile network download path.
- A robust fix is to download and extract dependency archives in PowerShell first, then pass `FETCHCONTENT_SOURCE_DIR_*` to CMake so MinGW CMake no longer has to fetch from GitHub directly.
- For a normal build, tests do not need to be enabled, so skipping `BUILD_TESTING` during `-Task build` avoids an unnecessary `googletest` fetch.
