from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from analysis_pipeline import DEFAULT_ANALYSIS_DIR, DEFAULT_RESULTS_DIR
from analyze_results import generate_analysis_bundle


REPRESENTATIVE_CASES = [
    {"name": "map1_default", "map": 1, "algo": "default"},
    {"name": "map1_astar", "map": 1, "algo": "astar"},
    {"name": "map1_dstar", "map": 1, "algo": "dstar"},
    {"name": "map3_default", "map": 3, "algo": "default"},
    {"name": "map3_astar", "map": 3, "algo": "astar"},
    {"name": "map3_dstar", "map": 3, "algo": "dstar"},
    {"name": "map5_default", "map": 5, "algo": "default"},
    {"name": "map5_astar", "map": 5, "algo": "astar"},
    {"name": "map5_dstar", "map": 5, "algo": "dstar"},
]


def run_case(executable: Path, output_dir: Path, seed: int, max_steps: int, case: dict[str, object]) -> None:
    case_dir = output_dir / str(case["name"])
    case_dir.mkdir(parents=True, exist_ok=True)

    summary_path = case_dir / "run_summary.json"
    steps_path = case_dir / "step_metrics.csv"

    command = [
        str(executable),
        "--headless",
        "--seed",
        str(seed),
        "--map",
        str(case["map"]),
        "--algo",
        str(case["algo"]),
        "--mode",
        "custom",
        "--speed",
        "0",
        "--phase",
        "park:2",
        "--phase",
        "exit:1",
        "--summary",
        str(summary_path),
        "--steps",
        str(steps_path),
        "--max-steps",
        str(max_steps),
    ]
    subprocess.run(command, check=True)


def run_batch_suite(
    executable: Path,
    output_dir: Path,
    analysis_output_dir: Path,
    seed: int,
    max_steps: int,
    skip_analysis: bool = False,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)

    for case in REPRESENTATIVE_CASES:
        run_case(executable, output_dir, seed, max_steps, case)

    manifest: dict[str, object] = {
        "seed": seed,
        "max_steps": max_steps,
        "cases": [case["name"] for case in REPRESENTATIVE_CASES],
    }
    if not skip_analysis:
        manifest["analysis"] = generate_analysis_bundle(output_dir, analysis_output_dir, "AGV Batch Analysis Report")

    manifest_path = output_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    manifest["manifest"] = str(manifest_path)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Run representative AGV baseline scenarios and build an analysis report.")
    parser.add_argument("executable", type=Path, help="Path to agv_console executable")
    parser.add_argument("output_dir", type=Path, nargs="?", default=DEFAULT_RESULTS_DIR, help="Directory where raw scenario outputs will be written")
    parser.add_argument("--analysis-output", type=Path, default=DEFAULT_ANALYSIS_DIR, help="Directory where the HTML analysis bundle will be written")
    parser.add_argument("--seed", type=int, default=1, help="Deterministic seed")
    parser.add_argument("--max-steps", type=int, default=10000, help="Per-scenario headless step limit")
    parser.add_argument("--skip-analysis", action="store_true", help="Skip HTML analysis generation")
    args = parser.parse_args()

    manifest = run_batch_suite(
        executable=args.executable,
        output_dir=args.output_dir,
        analysis_output_dir=args.analysis_output,
        seed=args.seed,
        max_steps=args.max_steps,
        skip_analysis=args.skip_analysis,
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
