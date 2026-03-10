from __future__ import annotations

import argparse
from pathlib import Path

from analysis_pipeline import load_results_bundle
from analyze_results import write_single_case_report


def resolve_case_dir(input_path: Path) -> Path:
    if input_path.is_file():
        if input_path.name != "step_metrics.csv":
            raise SystemExit("input file must be step_metrics.csv or pass a case directory")
        return input_path.parent
    return input_path


def summarize_case(case_dir: Path) -> str:
    bundle = load_results_bundle(case_dir)
    row = bundle.case_summary.iloc[0]
    return (
        f"case={row['case']} steps={int(row['recorded_steps'])} tasks={int(row['tasks_completed_total'])} "
        f"deadlocks={int(row['deadlock_count'])} max_zero_streak={int(row['max_zero_progress_streak'])} "
        f"planning_spikes={int(row['planning_spike_count'])}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate or summarize a single-case AGV deep-dive report.")
    parser.add_argument("input_path", type=Path, help="Case directory or step_metrics.csv path")
    parser.add_argument("--output", type=Path, help="Optional HTML output path")
    args = parser.parse_args()

    case_dir = resolve_case_dir(args.input_path)
    if args.output is None:
        print(summarize_case(case_dir))
        return

    output_path = args.output
    if output_path.suffix.lower() != ".html":
        output_path = output_path / f"{case_dir.name}_deep_dive.html"
    write_single_case_report(case_dir, output_path, f"AGV Deep Dive - {case_dir.name}")
    print(f"deep-dive report written to {output_path}")


if __name__ == "__main__":
    main()
