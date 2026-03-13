@echo off
setlocal

set "ROOT=%~dp0"
set "PS7=%ProgramFiles%\PowerShell\7\pwsh.exe"
set "WINPS=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe"

if "%~1"=="" (
    set "USE_DEFAULTS=1"
) else (
    set "USE_DEFAULTS=0"
)

if exist "%PS7%" (
    set "PS_CMD=%PS7%"
) else (
    set "PS_CMD=%WINPS%"
)

if not exist "%PS_CMD%" (
    echo PowerShell executable not found.
    exit /b 1
)

if "%USE_DEFAULTS%"=="1" (
    "%PS_CMD%" -NoProfile -ExecutionPolicy Bypass -File "%ROOT%scripts\codex_build.ps1" -Task build -BuildType Debug
) else (
    "%PS_CMD%" -NoProfile -ExecutionPolicy Bypass -File "%ROOT%scripts\codex_build.ps1" %*
)

set "EXIT_CODE=%ERRORLEVEL%"
if not "%EXIT_CODE%"=="0" (
    echo Build failed with exit code %EXIT_CODE%.
    exit /b %EXIT_CODE%
)

echo Build completed successfully.
exit /b 0
