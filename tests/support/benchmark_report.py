from __future__ import annotations

import argparse
from pathlib import Path

from analysis_pipeline import load_results_bundle


def format_markdown(results_dir: Path) -> str:
    bundle = load_results_bundle(results_dir)
    rows = bundle.case_summary.sort_values(["deadlock_count", "avg_planning_time_ms"], ascending=[False, False])
    lines = [
        "| case | tasks | throughput | deadlocks | avg planning ms | peak mem kb | max zero streak | planning spikes |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows.to_dict(orient="records"):
        lines.append(
            "| {case} | {tasks_completed_total} | {throughput:.4f} | {deadlock_count} | "
            "{avg_planning_time_ms:.4f} | {memory_usage_peak_kb:.2f} | {max_zero_progress_streak} | {planning_spike_count} |".format(**row)
        )
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a markdown benchmark report from AGV summaries.")
    parser.add_argument("results_dir", type=Path)
    parser.add_argument("--output", type=Path, help="Optional markdown output file")
    args = parser.parse_args()

    report = format_markdown(args.results_dir)
    if args.output:
        args.output.write_text(report, encoding="utf-8")
    else:
        print(report)


if __name__ == "__main__":
    main()
