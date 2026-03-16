param(
    [ValidateSet("configure", "build", "test", "console")]
    [string]$Task = "test",
    [ValidateSet("Debug", "Release", "RelWithDebInfo")]
    [string]$BuildType = "Debug",
    [switch]$Clean,
    [ValidateSet("Auto", "FetchContent", "Prefetch", "Offline")]
    [string]$DependencyMode = $(if ($env:AGV_DEPENDENCY_MODE) { $env:AGV_DEPENDENCY_MODE } else { "Auto" }),
    [ValidateSet("Auto", "On", "Off")]
    [string]$BuildTesting = $(if ($env:AGV_BUILD_TESTING) { $env:AGV_BUILD_TESTING } else { "Auto" }),
    [string]$DependencyCacheDir = $(if ($env:AGV_DEPENDENCY_CACHE_DIR) { $env:AGV_DEPENDENCY_CACHE_DIR } else { "" }),
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

function Resolve-BuildTesting {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Mode,
        [Parameter(Mandatory = $true)]
        [string]$TaskName
    )

    switch ($Mode) {
        "On" { return $true }
        "Off" { return $false }
        default { return $TaskName -eq "test" }
    }
}

function Resolve-DependencyCacheRoot {
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyString()]
        [string]$RequestedPath,
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot,
        [Parameter(Mandatory = $true)]
        [string]$BuildDirectory
    )

    if ([string]::IsNullOrWhiteSpace($RequestedPath)) {
        return Join-Path $BuildDirectory "_prefetch"
    }

    if ([System.IO.Path]::IsPathRooted($RequestedPath)) {
        return $RequestedPath
    }

    return Join-Path $RepositoryRoot $RequestedPath
}

function Get-FetchContentUrls {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CMakeListsPath
    )

    $content = Get-Content -Path $CMakeListsPath -Raw
    $matches = [regex]::Matches(
        $content,
        'FetchContent_Declare\(\s*([A-Za-z0-9_]+)\s+URL\s+([^\s\)]+)',
        [System.Text.RegularExpressions.RegexOptions]::IgnoreCase
    )

    $urls = @{}
    foreach ($match in $matches) {
        $urls[$match.Groups[1].Value] = $match.Groups[2].Value
    }

    return $urls
}

function Get-RequiredDependencyNames {
    param(
        [Parameter(Mandatory = $true)]
        [bool]$BuildTestingEnabled
    )

    $names = @("nlohmann_json")
    if ($BuildTestingEnabled) {
        $names += "googletest"
    }
    return $names
}

function Reset-ConfigureState {
    param(
        [Parameter(Mandatory = $true)]
        [string]$BuildDirectory
    )

    foreach ($name in @("CMakeCache.txt", "CMakeFiles", "_deps", "Testing", "DartConfiguration.tcl")) {
        $path = Join-Path $BuildDirectory $name
        if (Test-Path $path) {
            Remove-Item -Path $path -Recurse -Force
        }
    }
}

function Invoke-DependencyDownload {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Uri,
        [Parameter(Mandatory = $true)]
        [string]$OutputPath,
        [Parameter(Mandatory = $true)]
        [bool]$Offline
    )

    if (Test-Path $OutputPath) {
        $item = Get-Item -Path $OutputPath
        if ($item.Length -gt 0) {
            return
        }
        Remove-Item -Path $OutputPath -Force
    }

    if ($Offline) {
        throw "오프라인 모드에서는 캐시되지 않은 의존성을 다운로드할 수 없습니다: $Uri"
    }

    New-Item -ItemType Directory -Path (Split-Path -Path $OutputPath -Parent) -Force | Out-Null
    Write-Host "Downloading dependency archive: $Uri" -ForegroundColor Cyan

    try {
        Invoke-WebRequest -Uri $Uri -OutFile $OutputPath -UseBasicParsing
    } catch {
        if (Test-Path $OutputPath) {
            Remove-Item -Path $OutputPath -Force -ErrorAction SilentlyContinue
        }
        throw
    }

    if ((Get-Item -Path $OutputPath).Length -le 0) {
        Remove-Item -Path $OutputPath -Force -ErrorAction SilentlyContinue
        throw "다운로드된 파일이 비어 있습니다: $OutputPath"
    }
}

function Expand-DependencyArchive {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ArchivePath,
        [Parameter(Mandatory = $true)]
        [string]$DestinationPath,
        [Parameter(Mandatory = $true)]
        [string]$CMakePath
    )

    if (Test-Path $DestinationPath) {
        Remove-Item -Path $DestinationPath -Recurse -Force
    }

    New-Item -ItemType Directory -Path $DestinationPath -Force | Out-Null

    Push-Location $DestinationPath
    try {
        Invoke-External -FilePath $CMakePath -Arguments @("-E", "tar", "xf", $ArchivePath)
    } finally {
        Pop-Location
    }
}

function Resolve-ExtractedSourceDir {
    param(
        [Parameter(Mandatory = $true)]
        [string]$DependencyRoot
    )

    if (Test-Path (Join-Path $DependencyRoot "CMakeLists.txt")) {
        return $DependencyRoot
    }

    $rootCandidates = @(Get-ChildItem -Path $DependencyRoot -Directory -ErrorAction SilentlyContinue)
    if ($rootCandidates.Count -eq 1 -and (Test-Path (Join-Path $rootCandidates[0].FullName "CMakeLists.txt"))) {
        return $rootCandidates[0].FullName
    }

    $cmakeLists = Get-ChildItem -Path $DependencyRoot -Filter "CMakeLists.txt" -File -Recurse -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($cmakeLists) {
        return Split-Path -Path $cmakeLists.FullName -Parent
    }

    throw "추출된 의존성 디렉터리에서 CMakeLists.txt를 찾지 못했습니다: $DependencyRoot"
}

function Get-DependencySourceDir {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name,
        [Parameter(Mandatory = $true)]
        [string]$Url,
        [Parameter(Mandatory = $true)]
        [string]$CacheRoot,
        [Parameter(Mandatory = $true)]
        [string]$CMakePath,
        [Parameter(Mandatory = $true)]
        [bool]$Offline
    )

    $dependencyRoot = Join-Path $CacheRoot $Name
    $extractRoot = Join-Path $dependencyRoot "source"
    if (Test-Path $extractRoot) {
        try {
            return Resolve-ExtractedSourceDir -DependencyRoot $extractRoot
        } catch {
        }
    }

    $archiveUri = [System.Uri]$Url
    $archiveName = [System.IO.Path]::GetFileName($archiveUri.LocalPath)
    if ([string]::IsNullOrWhiteSpace($archiveName)) {
        throw "의존성 URL에서 아카이브 파일 이름을 추출하지 못했습니다: $Url"
    }

    $archivePath = Join-Path $dependencyRoot $archiveName
    Invoke-DependencyDownload -Uri $Url -OutputPath $archivePath -Offline $Offline
    Expand-DependencyArchive -ArchivePath $archivePath -DestinationPath $extractRoot -CMakePath $CMakePath
    return Resolve-ExtractedSourceDir -DependencyRoot $extractRoot
}

function Get-DependencySourceOverrides {
    param(
        [Parameter(Mandatory = $true)]
        [string[]]$DependencyNames,
        [Parameter(Mandatory = $true)]
        [hashtable]$UrlMap,
        [Parameter(Mandatory = $true)]
        [string]$CacheRoot,
        [Parameter(Mandatory = $true)]
        [string]$CMakePath,
        [Parameter(Mandatory = $true)]
        [bool]$Offline
    )

    $overrides = @{}
    foreach ($name in $DependencyNames) {
        if (-not $UrlMap.ContainsKey($name)) {
            throw "CMakeLists.txt에서 FetchContent URL을 찾지 못했습니다: $name"
        }

        $overrides[$name] = Get-DependencySourceDir `
            -Name $name `
            -Url $UrlMap[$name] `
            -CacheRoot $CacheRoot `
            -CMakePath $CMakePath `
            -Offline $Offline
    }

    return $overrides
}

function New-ConfigureArguments {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot,
        [Parameter(Mandatory = $true)]
        [string]$BuildDirectory,
        [Parameter(Mandatory = $true)]
        [string]$BuildType,
        [Parameter(Mandatory = $true)]
        [bool]$BuildTestingEnabled,
        [Parameter(Mandatory = $true)]
        [string]$CompilerPath,
        [Parameter(Mandatory = $true)]
        [string]$MakePath,
        [Parameter(Mandatory = $true)]
        [hashtable]$DependencyOverrides,
        [Parameter(Mandatory = $true)]
        [bool]$FullyDisconnected
    )

    $arguments = @(
        "-S", $RepositoryRoot,
        "-B", $BuildDirectory,
        "-G", "MinGW Makefiles",
        "-DCMAKE_BUILD_TYPE=$BuildType",
        "-DBUILD_TESTING=$(if ($BuildTestingEnabled) { 'ON' } else { 'OFF' })",
        "-DCMAKE_CXX_COMPILER=$CompilerPath",
        "-DCMAKE_MAKE_PROGRAM=$MakePath"
    )

    if ($DependencyOverrides.Count -gt 0) {
        $arguments += "-DFETCHCONTENT_UPDATES_DISCONNECTED=ON"
        foreach ($entry in $DependencyOverrides.GetEnumerator()) {
            $arguments += "-DFETCHCONTENT_SOURCE_DIR_$($entry.Key.ToUpperInvariant())=$($entry.Value)"
        }
    }

    if ($FullyDisconnected) {
        $arguments += "-DFETCHCONTENT_FULLY_DISCONNECTED=ON"
    }

    return $arguments
}

function Invoke-ConfigureStep {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Mode,
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot,
        [Parameter(Mandatory = $true)]
        [string]$BuildDirectory,
        [Parameter(Mandatory = $true)]
        [string]$BuildType,
        [Parameter(Mandatory = $true)]
        [bool]$BuildTestingEnabled,
        [Parameter(Mandatory = $true)]
        [string]$CompilerPath,
        [Parameter(Mandatory = $true)]
        [string]$MakePath,
        [Parameter(Mandatory = $true)]
        [string]$CMakePath,
        [Parameter(Mandatory = $true)]
        [hashtable]$FetchContentUrls,
        [Parameter(Mandatory = $true)]
        [string]$CacheRoot
    )

    $usePrefetch = $Mode -eq "Prefetch" -or $Mode -eq "Offline"
    $fullyDisconnected = $Mode -eq "Offline"
    $dependencyOverrides = @{}

    if ($usePrefetch) {
        $dependencyOverrides = Get-DependencySourceOverrides `
            -DependencyNames (Get-RequiredDependencyNames -BuildTestingEnabled $BuildTestingEnabled) `
            -UrlMap $FetchContentUrls `
            -CacheRoot $CacheRoot `
            -CMakePath $CMakePath `
            -Offline $fullyDisconnected
    }

    Write-Host "Configuring with DependencyMode=$Mode BUILD_TESTING=$(if ($BuildTestingEnabled) { 'ON' } else { 'OFF' })" -ForegroundColor Cyan
    $arguments = New-ConfigureArguments `
        -RepositoryRoot $RepositoryRoot `
        -BuildDirectory $BuildDirectory `
        -BuildType $BuildType `
        -BuildTestingEnabled $BuildTestingEnabled `
        -CompilerPath $CompilerPath `
        -MakePath $MakePath `
        -DependencyOverrides $dependencyOverrides `
        -FullyDisconnected $fullyDisconnected
    Invoke-External -FilePath $CMakePath -Arguments $arguments
}

$repoRoot = Split-Path -Path $PSScriptRoot -Parent
$buildDir = Join-Path $repoRoot "build-codex"
$cmakeListsPath = Join-Path $repoRoot "CMakeLists.txt"
$cmakePath = Find-CMake
$compilerPath = Find-CppCompiler
$makePath = Find-MakeProgram -CompilerPath $compilerPath
$compilerDirectory = Split-Path -Path $compilerPath -Parent
$buildTestingEnabled = Resolve-BuildTesting -Mode $BuildTesting -TaskName $Task
$dependencyCacheRoot = Resolve-DependencyCacheRoot `
    -RequestedPath $DependencyCacheDir `
    -RepositoryRoot $repoRoot `
    -BuildDirectory $buildDir
$fetchContentUrls = Get-FetchContentUrls -CMakeListsPath $cmakeListsPath

if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item -Path $buildDir -Recurse -Force
}

if ($Task -eq "configure" -or $Task -eq "build" -or $Task -eq "test" -or $Task -eq "console") {
    New-Item -ItemType Directory -Path $buildDir -Force | Out-Null

    switch ($DependencyMode) {
        "FetchContent" {
            Invoke-ConfigureStep `
                -Mode "FetchContent" `
                -RepositoryRoot $repoRoot `
                -BuildDirectory $buildDir `
                -BuildType $BuildType `
                -BuildTestingEnabled $buildTestingEnabled `
                -CompilerPath $compilerPath `
                -MakePath $makePath `
                -CMakePath $cmakePath `
                -FetchContentUrls $fetchContentUrls `
                -CacheRoot $dependencyCacheRoot
        }
        "Prefetch" {
            Invoke-ConfigureStep `
                -Mode "Prefetch" `
                -RepositoryRoot $repoRoot `
                -BuildDirectory $buildDir `
                -BuildType $BuildType `
                -BuildTestingEnabled $buildTestingEnabled `
                -CompilerPath $compilerPath `
                -MakePath $makePath `
                -CMakePath $cmakePath `
                -FetchContentUrls $fetchContentUrls `
                -CacheRoot $dependencyCacheRoot
        }
        "Offline" {
            Invoke-ConfigureStep `
                -Mode "Offline" `
                -RepositoryRoot $repoRoot `
                -BuildDirectory $buildDir `
                -BuildType $BuildType `
                -BuildTestingEnabled $buildTestingEnabled `
                -CompilerPath $compilerPath `
                -MakePath $makePath `
                -CMakePath $cmakePath `
                -FetchContentUrls $fetchContentUrls `
                -CacheRoot $dependencyCacheRoot
        }
        default {
            try {
                Invoke-ConfigureStep `
                    -Mode "FetchContent" `
                    -RepositoryRoot $repoRoot `
                    -BuildDirectory $buildDir `
                    -BuildType $BuildType `
                    -BuildTestingEnabled $buildTestingEnabled `
                    -CompilerPath $compilerPath `
                    -MakePath $makePath `
                    -CMakePath $cmakePath `
                    -FetchContentUrls $fetchContentUrls `
                    -CacheRoot $dependencyCacheRoot
            } catch {
                Write-Warning "FetchContent configure failed. Retrying with local prefetch cache."
                Reset-ConfigureState -BuildDirectory $buildDir
                Invoke-ConfigureStep `
                    -Mode "Prefetch" `
                    -RepositoryRoot $repoRoot `
                    -BuildDirectory $buildDir `
                    -BuildType $BuildType `
                    -BuildTestingEnabled $buildTestingEnabled `
                    -CompilerPath $compilerPath `
                    -MakePath $makePath `
                    -CMakePath $cmakePath `
                    -FetchContentUrls $fetchContentUrls `
                    -CacheRoot $dependencyCacheRoot
            }
        }
    }
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
