param(
    [ValidateSet("configure", "build", "test", "console")]
    [string]$Task = "test",
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$BuildType = "Debug",
    [switch]$Clean,
    [string[]]$ConsoleArgs = @()
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Find-CMake {
    $candidates = @(
        "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe",
        "C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    )

    foreach ($candidate in $candidates) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    $command = Get-Command cmake.exe -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    throw "cmake.exe를 찾지 못했습니다."
}

function Find-CppCompiler {
    $command = Get-Command g++.exe -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $fallbacks = @(
        "C:\msys64\mingw64\bin\g++.exe",
        "C:\msys64\ucrt64\bin\g++.exe",
        "C:\Users\$env:USERNAME\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\g++.exe"
    )

    foreach ($candidate in $fallbacks) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    throw "g++.exe를 찾지 못했습니다."
}

function Find-MakeProgram {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CompilerPath
    )

    $compilerDirectory = Split-Path -Path $CompilerPath -Parent
    $candidate = Join-Path $compilerDirectory "mingw32-make.exe"
    if (Test-Path $candidate) {
        return $candidate
    }

    $command = Get-Command mingw32-make.exe -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    throw "mingw32-make.exe를 찾지 못했습니다."
}

function Copy-CompilerRuntimeDlls {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CompilerPath,
        [Parameter(Mandatory = $true)]
        [string]$OutputDirectory
    )

    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null

    $compilerDirectory = Split-Path -Path $CompilerPath -Parent
    foreach ($dllName in @("libstdc++-6.dll", "libgcc_s_seh-1.dll", "libwinpthread-1.dll")) {
        $sourcePath = Join-Path $compilerDirectory $dllName
        if (Test-Path $sourcePath) {
            Copy-Item -Path $sourcePath -Destination (Join-Path $OutputDirectory $dllName) -Force
        }
    }
}

function Quote-Argument {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Value
    )

    if ($Value -notmatch '[\s"]') {
        return $Value
    }

    return '"' + $Value.Replace('"', '\"') + '"'
}

function Invoke-External {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [Parameter(Mandatory = $true)]
        [string[]]$Arguments
    )

    $argumentString = ($Arguments | ForEach-Object { Quote-Argument $_ }) -join " "
    $process = Start-Process -FilePath $FilePath -ArgumentList $argumentString -NoNewWindow -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "명령 실행 실패: $FilePath $($Arguments -join ' ')"
    }
}

$repoRoot = Split-Path -Path $PSScriptRoot -Parent
$buildDir = Join-Path $repoRoot "build-codex"
$cmakePath = Find-CMake
$compilerPath = Find-CppCompiler
$makePath = Find-MakeProgram -CompilerPath $compilerPath
$compilerDirectory = Split-Path -Path $compilerPath -Parent

if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item -Path $buildDir -Recurse -Force
}

if ($Task -eq "configure" -or $Task -eq "build" -or $Task -eq "test" -or $Task -eq "console") {
    Invoke-External -FilePath $cmakePath -Arguments @(
        "-S", $repoRoot,
        "-B", $buildDir,
        "-G", "MinGW Makefiles",
        "-DCMAKE_BUILD_TYPE=$BuildType",
        "-DBUILD_TESTING=ON",
        "-DCMAKE_CXX_COMPILER=$compilerPath",
        "-DCMAKE_MAKE_PROGRAM=$makePath"
    )
}

if ($Task -eq "build" -or $Task -eq "test" -or $Task -eq "console") {
    Invoke-External -FilePath $cmakePath -Arguments @(
        "--build", $buildDir,
        "--parallel", "4"
    )
    Copy-CompilerRuntimeDlls -CompilerPath $compilerPath -OutputDirectory $buildDir
}

if ($Task -eq "test") {
    $testsExe = Join-Path $buildDir "agv_core_tests.exe"
    if (-not (Test-Path $testsExe)) {
        throw "agv_core_tests.exe를 찾지 못했습니다."
    }

    Invoke-External -FilePath $testsExe -Arguments @("--gtest_color=no")
    Invoke-External -FilePath $cmakePath -Arguments @(
        "-DAGV_ROOT=$repoRoot",
        "-P", (Join-Path $repoRoot "tests/no_c_smells.cmake")
    )
}

if ($Task -eq "console") {
    $consolePath = Join-Path $buildDir "agv_console.exe"
    if (-not (Test-Path $consolePath)) {
        throw "agv_console.exe를 찾지 못했습니다."
    }

    if ($ConsoleArgs.Count -gt 0) {
        $env:Path = "$compilerDirectory;$env:Path"
        & $consolePath @ConsoleArgs
        if ($LASTEXITCODE -ne 0) {
            throw "agv_console.exe 실행 실패"
        }
    }
}

Write-Host "Task '$Task' completed in $buildDir" -ForegroundColor Green
