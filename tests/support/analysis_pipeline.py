from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Final

try:
    import pandas as pd
except ImportError as exc:  # pragma: no cover - guarded at runtime
    pd = None
    _PANDAS_IMPORT_ERROR = exc
else:
    _PANDAS_IMPORT_ERROR = None


PROJECT_ROOT: Final[Path] = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_DIR: Final[Path] = PROJECT_ROOT / "tests" / "artifacts" / "results"
DEFAULT_ANALYSIS_DIR: Final[Path] = PROJECT_ROOT / "tests" / "artifacts" / "analysis"
SUMMARY_FILENAME: Final[str] = "run_summary.json"
STEP_FILENAME: Final[str] = "step_metrics.csv"
ROLLING_WINDOW: Final[int] = 5
ZERO_PROGRESS_ALERT_STREAK: Final[int] = 5

SUMMARY_NUMERIC_COLUMNS: Final[list[str]] = [
    "seed",
    "map_id",
    "path_algo",
    "mode",
    "active_agents",
    "recorded_steps",
    "tasks_completed_total",
    "throughput",
    "total_movement_cost",
    "deadlock_count",
    "total_cpu_time_ms",
    "avg_cpu_time_ms",
    "total_planning_time_ms",
    "avg_planning_time_ms",
    "memory_usage_sum_kb",
    "avg_memory_usage_kb",
    "memory_usage_peak_kb",
    "algo_nodes_expanded_total",
    "algo_heap_moves_total",
    "algo_generated_nodes_total",
    "algo_valid_expansions_total",
    "valid_expansion_ratio",
    "requests_created_total",
    "request_wait_ticks_sum",
    "remaining_parked_vehicles",
]

STEP_NUMERIC_COLUMNS: Final[list[str]] = [
    "step",
    "tasks_completed_total",
    "total_movement_cost",
    "deadlock_count",
    "last_step_cpu_ms",
    "total_cpu_ms",
    "last_planning_ms",
    "total_planning_ms",
    "algo_nodes_expanded_last",
    "algo_heap_moves_last",
    "algo_generated_nodes_last",
    "algo_valid_expansions_last",
    "requests_created_total",
    "request_wait_ticks_sum",
]

DERIVED_STEP_COLUMNS: Final[list[str]] = [
    "tasks_completed_delta",
    "movement_delta",
    "deadlock_delta",
    "cpu_delta_ms",
    "planning_delta_ms",
    "requests_created_delta",
    "request_wait_ticks_delta",
    "movement_delta_rolling_avg",
    "cpu_delta_ms_rolling_avg",
    "planning_delta_ms_rolling_avg",
    "deadlock_delta_rolling_avg",
    "nodes_expanded_rolling_avg",
    "zero_progress_flag",
    "zero_progress_streak",
    "deadlock_spike_flag",
    "planning_spike_flag",
    "cpu_spike_flag",
    "nodes_expanded_spike_flag",
]


@dataclass(frozen=True)
class AnalysisResults:
    results_dir: Path
    case_summary: "pd.DataFrame"
    step_enriched: "pd.DataFrame"


def require_pandas() -> None:
    if pd is None:  # pragma: no cover - exercised only when dependency is missing
        raise RuntimeError(
            "pandas is required for AGV analysis tooling. "
            "Install it with: python -m pip install pandas plotly pytest"
        ) from _PANDAS_IMPORT_ERROR


def _coerce_numeric(frame: "pd.DataFrame", columns: list[str]) -> "pd.DataFrame":
    for column in columns:
        if column not in frame.columns:
            frame[column] = 0
        frame[column] = pd.to_numeric(frame[column], errors="coerce").fillna(0)
    return frame


def _case_directories(results_dir: Path) -> list[Path]:
    if (results_dir / SUMMARY_FILENAME).exists():
        return [results_dir]
    return sorted(path for path in results_dir.iterdir() if path.is_dir() and (path / SUMMARY_FILENAME).exists())


def _load_summary(case_dir: Path) -> dict[str, object]:
    payload = json.loads((case_dir / SUMMARY_FILENAME).read_text(encoding="utf-8"))
    payload["case"] = case_dir.name
    payload["case_path"] = str(case_dir)
    return payload


def _load_steps(case_dir: Path, summary: dict[str, object]) -> "pd.DataFrame":
    steps_path = case_dir / STEP_FILENAME
    if not steps_path.exists():
        frame = pd.DataFrame(columns=STEP_NUMERIC_COLUMNS)
    else:
        frame = pd.read_csv(steps_path)
    frame["case"] = case_dir.name
    frame["map_id"] = summary.get("map_id", 0)
    frame["path_algo"] = summary.get("path_algo", 0)
    frame["path_algo_label"] = summary.get("path_algo_label", "unknown")
    frame["mode"] = summary.get("mode", 0)
    frame["mode_label"] = summary.get("mode_label", "unknown")
    frame["active_agents"] = summary.get("active_agents", 0)
    return _coerce_numeric(frame, STEP_NUMERIC_COLUMNS + ["map_id", "path_algo", "mode", "active_agents"])


def _positive_delta(series: "pd.Series") -> "pd.Series":
    if series.empty:
        return series.copy()
    delta = series.diff().fillna(series.iloc[0])
    return delta.clip(lower=0)


def _rolling_average(series: "pd.Series") -> "pd.Series":
    return series.rolling(ROLLING_WINDOW, min_periods=1).mean()


def _flag_spikes(series: "pd.Series") -> "pd.Series":
    baseline = pd.to_numeric(series, errors="coerce").fillna(0.0)
    if baseline.empty:
        return pd.Series(dtype=bool)
    mean = float(baseline.mean())
    std = float(baseline.std(ddof=0))
    quantile_90 = float(baseline.quantile(0.90)) if len(baseline) > 1 else float(baseline.iloc[0])
    threshold = max(mean + (1.5 * std), quantile_90)
    if threshold <= 0:
        return baseline > 0
    return (baseline > 0) & (baseline >= threshold)


def _compute_zero_progress_streak(flags: "pd.Series") -> "pd.Series":
    streak: list[int] = []
    current = 0
    for flag in flags.astype(bool):
        current = current + 1 if flag else 0
        streak.append(current)
    return pd.Series(streak, index=flags.index, dtype="int64")


def _enrich_case_steps(frame: "pd.DataFrame") -> "pd.DataFrame":
    frame = frame.sort_values("step").reset_index(drop=True).copy()
    frame["tasks_completed_delta"] = _positive_delta(frame["tasks_completed_total"])
    frame["movement_delta"] = _positive_delta(frame["total_movement_cost"])
    frame["deadlock_delta"] = _positive_delta(frame["deadlock_count"])
    frame["cpu_delta_ms"] = _positive_delta(frame["total_cpu_ms"])
    frame["planning_delta_ms"] = _positive_delta(frame["total_planning_ms"])
    frame["requests_created_delta"] = _positive_delta(frame["requests_created_total"])
    frame["request_wait_ticks_delta"] = _positive_delta(frame["request_wait_ticks_sum"])

    frame["movement_delta_rolling_avg"] = _rolling_average(frame["movement_delta"])
    frame["cpu_delta_ms_rolling_avg"] = _rolling_average(frame["cpu_delta_ms"])
    frame["planning_delta_ms_rolling_avg"] = _rolling_average(frame["planning_delta_ms"])
    frame["deadlock_delta_rolling_avg"] = _rolling_average(frame["deadlock_delta"])
    frame["nodes_expanded_rolling_avg"] = _rolling_average(frame["algo_nodes_expanded_last"])

    frame["zero_progress_flag"] = (frame["tasks_completed_delta"] <= 0) & (frame["movement_delta"] <= 0)
    frame["zero_progress_streak"] = _compute_zero_progress_streak(frame["zero_progress_flag"])

    frame["deadlock_spike_flag"] = _flag_spikes(frame["deadlock_delta"])
    frame["planning_spike_flag"] = _flag_spikes(frame["planning_delta_ms"])
    frame["cpu_spike_flag"] = _flag_spikes(frame["cpu_delta_ms"])
    frame["nodes_expanded_spike_flag"] = _flag_spikes(frame["algo_nodes_expanded_last"])
    return frame


def enrich_step_metrics(step_frame: "pd.DataFrame") -> "pd.DataFrame":
    require_pandas()
    if step_frame.empty:
        return pd.DataFrame(columns=[*STEP_NUMERIC_COLUMNS, "case", "map_id", "path_algo", "path_algo_label", "mode", "mode_label", "active_agents", *DERIVED_STEP_COLUMNS])

    grouped = [_enrich_case_steps(group) for _, group in step_frame.groupby("case", sort=True)]
    enriched = pd.concat(grouped, ignore_index=True)
    bool_columns = [
        "zero_progress_flag",
        "deadlock_spike_flag",
        "planning_spike_flag",
        "cpu_spike_flag",
        "nodes_expanded_spike_flag",
    ]
    for column in bool_columns:
        enriched[column] = enriched[column].astype(bool)
    enriched["zero_progress_streak"] = enriched["zero_progress_streak"].astype(int)
    return enriched


def _augment_case_summary(case_summary: "pd.DataFrame", step_enriched: "pd.DataFrame") -> "pd.DataFrame":
    case_summary = _coerce_numeric(case_summary, SUMMARY_NUMERIC_COLUMNS)
    if step_enriched.empty:
        defaults = {
            "max_zero_progress_streak": 0,
            "zero_progress_steps": 0,
            "deadlock_spike_count": 0,
            "planning_spike_count": 0,
            "cpu_spike_count": 0,
            "nodes_expanded_spike_count": 0,
            "max_deadlock_delta": 0.0,
            "max_planning_delta_ms": 0.0,
            "max_cpu_delta_ms": 0.0,
        }
        for column, default in defaults.items():
            case_summary[column] = default
    else:
        aggregated = (
            step_enriched.groupby("case", sort=True)
            .agg(
                max_zero_progress_streak=("zero_progress_streak", "max"),
                zero_progress_steps=("zero_progress_flag", "sum"),
                deadlock_spike_count=("deadlock_spike_flag", "sum"),
                planning_spike_count=("planning_spike_flag", "sum"),
                cpu_spike_count=("cpu_spike_flag", "sum"),
                nodes_expanded_spike_count=("nodes_expanded_spike_flag", "sum"),
                max_deadlock_delta=("deadlock_delta", "max"),
                max_planning_delta_ms=("planning_delta_ms", "max"),
                max_cpu_delta_ms=("cpu_delta_ms", "max"),
            )
            .reset_index()
        )
        case_summary = case_summary.merge(aggregated, on="case", how="left")
        case_summary = case_summary.fillna(0)

    safe_tasks = case_summary["tasks_completed_total"].replace(0, pd.NA)
    safe_steps = case_summary["recorded_steps"].replace(0, pd.NA)
    case_summary["movement_per_task"] = (case_summary["total_movement_cost"] / safe_tasks).fillna(case_summary["total_movement_cost"])
    case_summary["zero_progress_ratio"] = (case_summary["zero_progress_steps"] / safe_steps).fillna(0.0)
    case_summary["anomaly_score"] = (
        (case_summary["deadlock_spike_count"] * 2)
        + case_summary["planning_spike_count"]
        + case_summary["cpu_spike_count"]
        + (case_summary["max_zero_progress_streak"] >= ZERO_PROGRESS_ALERT_STREAK).astype(int) * 2
    )
    return case_summary.sort_values(["anomaly_score", "deadlock_count", "avg_planning_time_ms", "case"], ascending=[False, False, False, True]).reset_index(drop=True)


def load_results_bundle(results_dir: Path) -> AnalysisResults:
    require_pandas()
    results_dir = results_dir.resolve()
    case_dirs = _case_directories(results_dir)
    if not case_dirs:
        raise FileNotFoundError(f"No AGV result cases were found in {results_dir}")

    summary_records: list[dict[str, object]] = []
    step_frames: list["pd.DataFrame"] = []

    for case_dir in case_dirs:
        summary = _load_summary(case_dir)
        summary_records.append(summary)
        step_frames.append(_load_steps(case_dir, summary))

    case_summary = pd.DataFrame(summary_records)
    step_frame = pd.concat(step_frames, ignore_index=True) if step_frames else pd.DataFrame()
    step_enriched = enrich_step_metrics(step_frame)
    case_summary = _augment_case_summary(case_summary, step_enriched)
    return AnalysisResults(results_dir=results_dir, case_summary=case_summary, step_enriched=step_enriched)


def anomaly_rows(step_enriched: "pd.DataFrame") -> "pd.DataFrame":
    require_pandas()
    if step_enriched.empty:
        return step_enriched.copy()
    mask = (
        step_enriched["deadlock_spike_flag"]
        | step_enriched["planning_spike_flag"]
        | step_enriched["cpu_spike_flag"]
        | step_enriched["nodes_expanded_spike_flag"]
        | (step_enriched["zero_progress_streak"] >= ZERO_PROGRESS_ALERT_STREAK)
    )
    columns = [
        "case",
        "step",
        "tasks_completed_delta",
        "movement_delta",
        "deadlock_delta",
        "planning_delta_ms",
        "cpu_delta_ms",
        "algo_nodes_expanded_last",
        "zero_progress_streak",
        "deadlock_spike_flag",
        "planning_spike_flag",
        "cpu_spike_flag",
        "nodes_expanded_spike_flag",
    ]
    available = [column for column in columns if column in step_enriched.columns]
    return step_enriched.loc[mask, available].sort_values(["case", "step"]).reset_index(drop=True)
