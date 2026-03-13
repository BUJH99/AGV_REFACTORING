#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pwsh.exe -NoProfile -File "$(wslpath -w "$script_dir/codex_build.ps1")" "$@"
