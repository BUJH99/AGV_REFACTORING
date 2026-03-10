from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


STRICT_KEYS = {
    "map_id",
    "path_algo",
    "mode",
    "active_agents",
    "recorded_steps",
    "tasks_completed_total",
    "deadlock_count",
    "algo_nodes_expanded_total",
    "algo_heap_moves_total",
    "algo_generated_nodes_total",
    "algo_valid_expansions_total",
    "requests_created_total",
    "request_wait_ticks_sum",
    "remaining_parked_vehicles",
}

FLOAT_KEYS = {
    "throughput",
    "total_movement_cost",
    "total_cpu_time_ms",
    "avg_cpu_time_ms",
    "total_planning_time_ms",
    "avg_planning_time_ms",
    "memory_usage_sum_kb",
    "avg_memory_usage_kb",
    "memory_usage_peak_kb",
    "valid_expansion_ratio",
}


def load_summary(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def compare_case(expected: Path, actual: Path, tolerance: float) -> list[str]:
    expected_summary = load_summary(expected / "run_summary.json")
    actual_summary = load_summary(actual / "run_summary.json")
    errors: list[str] = []

    for key in STRICT_KEYS:
        if expected_summary.get(key) != actual_summary.get(key):
            errors.append(f"{actual.name}:{key} expected {expected_summary.get(key)} got {actual_summary.get(key)}")

    for key in FLOAT_KEYS:
        lhs = float(expected_summary.get(key, 0.0))
        rhs = float(actual_summary.get(key, 0.0))
        if not math.isclose(lhs, rhs, rel_tol=tolerance, abs_tol=tolerance):
            errors.append(f"{actual.name}:{key} expected {lhs} got {rhs}")

    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare AGV golden outputs.")
    parser.add_argument("expected_dir", type=Path)
    parser.add_argument("actual_dir", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    args = parser.parse_args()

    errors: list[str] = []
    for expected_case in sorted(path for path in args.expected_dir.iterdir() if path.is_dir()):
        actual_case = args.actual_dir / expected_case.name
        if not actual_case.exists():
            errors.append(f"missing case: {expected_case.name}")
            continue
        errors.extend(compare_case(expected_case, actual_case, args.tolerance))

    if errors:
        raise SystemExit("\n".join(errors))

    print("golden comparison passed")


if __name__ == "__main__":
    main()
