param(
    [switch]$Interactive,
    [string[]]$ExeArgs
)

$ErrorActionPreference = "Stop"

function Find-CppCompiler {
    $command = Get-Command g++ -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    $fallbacks = @(
        "C:\msys64\ucrt64\bin\g++.exe"
        "C:\msys64\mingw64\bin\g++.exe"
        "C:\Users\$env:USERNAME\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin\g++.exe"
        "C:\Strawberry\c\bin\g++.exe"
    )

    foreach ($candidate in $fallbacks) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    return $null
}

function Copy-CompilerRuntimeDlls {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CompilerPath,
        [Parameter(Mandatory = $true)]
        [string]$OutputDirectory
    )

    $compilerDirectory = Split-Path -Path $CompilerPath -Parent
    $runtimeDlls = @(
        "libstdc++-6.dll"
        "libgcc_s_seh-1.dll"
        "libwinpthread-1.dll"
    )

    foreach ($dllName in $runtimeDlls) {
        $sourcePath = Join-Path $compilerDirectory $dllName
        if (Test-Path $sourcePath) {
            Copy-Item -Path $sourcePath -Destination (Join-Path $OutputDirectory $dllName) -Force
        }
    }
}

function Start-ExeInNewTerminal {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ExePath,
        [Parameter(Mandatory = $true)]
        [string]$WorkingDirectory,
        [string[]]$ExeArguments = @(),
        [string]$CompilerDirectory = ""
    )

    $pwsh = Get-Command pwsh.exe -ErrorAction SilentlyContinue
    if (-not $pwsh) {
        Write-Host "pwsh.exe를 찾을 수 없습니다. PowerShell 7이 설치되어 있고 PATH에 등록되어 있는지 확인해 주세요." -ForegroundColor Red
        exit 1
    }

    $resolvedExePath = (Resolve-Path $ExePath).Path
    $escapedExePath = $resolvedExePath.Replace("'", "''")
    $escapedArgs = $ExeArguments | ForEach-Object { "'" + $_.Replace("'", "''") + "'" }
    $launchCommand = ""
    if ($CompilerDirectory) {
        $escapedCompilerDirectory = $CompilerDirectory.Replace("'", "''")
        $launchCommand += "`$env:Path = '$escapedCompilerDirectory;' + `$env:Path; "
    }
    $launchCommand += "& '$escapedExePath'"
    if ($escapedArgs.Count -gt 0) {
        $launchCommand += " " + ($escapedArgs -join " ")
    }

    Start-Process -FilePath $pwsh.Source `
        -WorkingDirectory $WorkingDirectory `
        -ArgumentList @(
            "-NoExit"
            "-Command"
            $launchCommand
        )
}

if (-not $PSBoundParameters.ContainsKey("ExeArgs")) {
    $ExeArgs = @()
}

$compiler = Find-CppCompiler
if (-not $compiler) {
    Write-Host "g++를 찾을 수 없습니다. PATH 또는 MSYS2/WinLibs 경로를 확인해 주세요." -ForegroundColor Red
    exit 1
}
$compilerDirectory = Split-Path -Path $compiler -Parent

$sources = Get-ChildItem -Path src -Filter *.cpp -Recurse | Select-Object -ExpandProperty FullName

$compileArgs = @(
    "-std=c++20",
    "-O3",
    "-DNDEBUG",
    "-Iinclude",
    "-Isrc",
    "-DAGV_NO_MAIN",
    "-static-libstdc++",
    "-static-libgcc",
    "-o",
    "agv_console.exe"
) + $sources + @(
    "-lpsapi"
)

Write-Host "Compiling with $compiler ..." -ForegroundColor Cyan
& $compiler @compileArgs

if ($LASTEXITCODE -ne 0) {
    Write-Host "Compilation failed." -ForegroundColor Red
    exit $LASTEXITCODE
}

Copy-CompilerRuntimeDlls -CompilerPath $compiler -OutputDirectory $PWD.Path

if ($ExeArgs.Count -gt 0) {
    Write-Host "Launching agv_console.exe with preset arguments in a new terminal window ..." -ForegroundColor Green
    Write-Host ("Args: " + ($ExeArgs -join " ")) -ForegroundColor DarkGray
} else {
    Write-Host "Launching agv_console.exe in interactive mode in a new terminal window ..." -ForegroundColor Green
}

Start-ExeInNewTerminal -ExePath ".\agv_console.exe" -WorkingDirectory $PWD.Path -ExeArguments $ExeArgs -CompilerDirectory $compilerDirectory
exit 0
