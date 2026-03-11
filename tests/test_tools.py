from __future__ import annotations

import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SUPPORT = PROJECT_ROOT / "tests" / "support"
if str(SUPPORT) not in sys.path:
    sys.path.insert(0, str(SUPPORT))

import analysis_pipeline
import analyze_results
import batch_runner
import benchmark_report
import plot_metrics


def write_case(case_dir: Path, summary: dict[str, object], step_rows: list[dict[str, object]]) -> None:
    case_dir.mkdir(parents=True, exist_ok=True)
    (case_dir / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    with (case_dir / "step_metrics.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(step_rows[0].keys()))
        writer.writeheader()
        writer.writerows(step_rows)


def make_summary(*, algo_label: str = "default") -> dict[str, object]:
    return {
        "seed": 1,
        "map_id": 1,
        "path_algo": 0,
        "path_algo_label": algo_label,
        "mode": 0,
        "mode_label": "custom",
        "active_agents": 3,
        "recorded_steps": 5,
        "tasks_completed_total": 2,
        "throughput": 0.4,
        "total_movement_cost": 8.0,
        "deadlock_count": 4,
        "total_cpu_time_ms": 25.0,
        "avg_cpu_time_ms": 5.0,
        "total_planning_time_ms": 20.0,
        "avg_planning_time_ms": 4.0,
        "memory_usage_sum_kb": 500.0,
        "avg_memory_usage_kb": 100.0,
        "memory_usage_peak_kb": 150.0,
        "algo_nodes_expanded_total": 20,
        "algo_heap_moves_total": 30,
        "algo_generated_nodes_total": 40,
        "algo_valid_expansions_total": 18,
        "valid_expansion_ratio": 0.45,
        "requests_created_total": 3,
        "request_wait_ticks_sum": 12,
        "remaining_parked_vehicles": 1,
    }


def make_steps() -> list[dict[str, object]]:
    return [
        {"step": 1, "tasks_completed_total": 0, "total_movement_cost": 1, "deadlock_count": 0, "last_step_cpu_ms": 1, "total_cpu_ms": 1, "last_planning_ms": 1, "total_planning_ms": 1, "algo_nodes_expanded_last": 2, "algo_heap_moves_last": 3, "algo_generated_nodes_last": 4, "algo_valid_expansions_last": 2, "requests_created_total": 0, "request_wait_ticks_sum": 0},
        {"step": 2, "tasks_completed_total": 1, "total_movement_cost": 3, "deadlock_count": 0, "last_step_cpu_ms": 2, "total_cpu_ms": 3, "last_planning_ms": 2, "total_planning_ms": 3, "algo_nodes_expanded_last": 2, "algo_heap_moves_last": 4, "algo_generated_nodes_last": 5, "algo_valid_expansions_last": 2, "requests_created_total": 1, "request_wait_ticks_sum": 1},
        {"step": 3, "tasks_completed_total": 1, "total_movement_cost": 3, "deadlock_count": 0, "last_step_cpu_ms": 0, "total_cpu_ms": 3, "last_planning_ms": 0, "total_planning_ms": 3, "algo_nodes_expanded_last": 0, "algo_heap_moves_last": 0, "algo_generated_nodes_last": 0, "algo_valid_expansions_last": 0, "requests_created_total": 1, "request_wait_ticks_sum": 1},
        {"step": 4, "tasks_completed_total": 1, "total_movement_cost": 3, "deadlock_count": 1, "last_step_cpu_ms": 0, "total_cpu_ms": 3, "last_planning_ms": 0, "total_planning_ms": 3, "algo_nodes_expanded_last": 0, "algo_heap_moves_last": 0, "algo_generated_nodes_last": 0, "algo_valid_expansions_last": 0, "requests_created_total": 1, "request_wait_ticks_sum": 1},
        {"step": 5, "tasks_completed_total": 2, "total_movement_cost": 8, "deadlock_count": 4, "last_step_cpu_ms": 22, "total_cpu_ms": 25, "last_planning_ms": 17, "total_planning_ms": 20, "algo_nodes_expanded_last": 16, "algo_heap_moves_last": 18, "algo_generated_nodes_last": 20, "algo_valid_expansions_last": 10, "requests_created_total": 3, "request_wait_ticks_sum": 12},
    ]


def test_load_results_enriches_steps_and_derives_metrics(tmp_path: Path) -> None:
    write_case(tmp_path / "case_a", make_summary(), make_steps())
    bundle = analysis_pipeline.load_results_bundle(tmp_path)

    assert list(bundle.case_summary["case"]) == ["case_a"]
    assert {"tasks_completed_delta", "movement_delta", "planning_delta_ms", "zero_progress_streak"}.issubset(bundle.step_enriched.columns)

    step2 = bundle.step_enriched.loc[bundle.step_enriched["step"] == 2].iloc[0]
    assert step2["tasks_completed_delta"] == 1
    assert step2["movement_delta"] == 2
    assert step2["planning_delta_ms"] == 2


def test_anomaly_detection_flags_spikes_and_zero_progress(tmp_path: Path) -> None:
    plateau_steps = [
        {"step": 1, "tasks_completed_total": 0, "total_movement_cost": 1, "deadlock_count": 0, "last_step_cpu_ms": 1, "total_cpu_ms": 1, "last_planning_ms": 1, "total_planning_ms": 1, "algo_nodes_expanded_last": 2, "algo_heap_moves_last": 3, "algo_generated_nodes_last": 4, "algo_valid_expansions_last": 2, "requests_created_total": 0, "request_wait_ticks_sum": 0},
        {"step": 2, "tasks_completed_total": 0, "total_movement_cost": 1, "deadlock_count": 0, "last_step_cpu_ms": 0, "total_cpu_ms": 1, "last_planning_ms": 0, "total_planning_ms": 1, "algo_nodes_expanded_last": 0, "algo_heap_moves_last": 0, "algo_generated_nodes_last": 0, "algo_valid_expansions_last": 0, "requests_created_total": 0, "request_wait_ticks_sum": 0},
        {"step": 3, "tasks_completed_total": 0, "total_movement_cost": 1, "deadlock_count": 0, "last_step_cpu_ms": 0, "total_cpu_ms": 1, "last_planning_ms": 0, "total_planning_ms": 1, "algo_nodes_expanded_last": 0, "algo_heap_moves_last": 0, "algo_generated_nodes_last": 0, "algo_valid_expansions_last": 0, "requests_created_total": 0, "request_wait_ticks_sum": 0},
        {"step": 4, "tasks_completed_total": 0, "total_movement_cost": 1, "deadlock_count": 0, "last_step_cpu_ms": 0, "total_cpu_ms": 1, "last_planning_ms": 0, "total_planning_ms": 1, "algo_nodes_expanded_last": 0, "algo_heap_moves_last": 0, "algo_generated_nodes_last": 0, "algo_valid_expansions_last": 0, "requests_created_total": 0, "request_wait_ticks_sum": 0},
        {"step": 5, "tasks_completed_total": 2, "total_movement_cost": 8, "deadlock_count": 5, "last_step_cpu_ms": 24, "total_cpu_ms": 25, "last_planning_ms": 19, "total_planning_ms": 20, "algo_nodes_expanded_last": 16, "algo_heap_moves_last": 18, "algo_generated_nodes_last": 20, "algo_valid_expansions_last": 10, "requests_created_total": 3, "request_wait_ticks_sum": 12},
    ]
    write_case(tmp_path / "case_a", make_summary(), plateau_steps)
    bundle = analysis_pipeline.load_results_bundle(tmp_path)
    case_row = bundle.case_summary.iloc[0]
    anomalies = analysis_pipeline.anomaly_rows(bundle.step_enriched)

    assert int(case_row["max_zero_progress_streak"]) >= 3
    assert int(case_row["planning_spike_count"]) >= 1
    assert int(case_row["deadlock_spike_count"]) >= 1
    assert not anomalies.empty


def test_generate_analysis_bundle_writes_html_and_exports(tmp_path: Path) -> None:
    results_dir = tmp_path / "results"
    write_case(results_dir / "case_a", make_summary(), make_steps())

    output_dir = tmp_path / "analysis"
    manifest = analyze_results.generate_analysis_bundle(results_dir, output_dir, "Synthetic AGV Report")

    assert (output_dir / "report.html").exists()
    assert (output_dir / "case_summary.csv").exists()
    assert (output_dir / "step_enriched.csv").exists()
    report_html = (output_dir / "report.html").read_text(encoding="utf-8")
    assert "Synthetic AGV Report" in report_html
    assert "case_a" in report_html
    assert "Batch Comparison" in report_html
    assert manifest["case_count"] == 1


def test_benchmark_report_uses_shared_loader(tmp_path: Path) -> None:
    results_dir = tmp_path / "results"
    write_case(results_dir / "case_a", make_summary(), make_steps())

    report = benchmark_report.format_markdown(results_dir)
    assert "| case_a |" in report
    assert "max zero streak" in report.lower()


def test_batch_runner_auto_generates_analysis(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    def fake_run(command: list[str], check: bool) -> None:
        summary_path = Path(command[command.index("--summary") + 1])
        case_name = summary_path.parent.name
        write_case(summary_path.parent, make_summary(algo_label=str(command[command.index("--algo") + 1])), make_steps())

    monkeypatch.setattr(batch_runner.subprocess, "run", fake_run)
    manifest = batch_runner.run_batch_suite(
        executable=Path("fake.exe"),
        output_dir=tmp_path / "results",
        analysis_output_dir=tmp_path / "analysis",
        seed=7,
        max_steps=200,
    )

    assert (tmp_path / "analysis" / "report.html").exists()
    assert "analysis" in manifest
    assert len(manifest["cases"]) == len(batch_runner.REPRESENTATIVE_CASES)


def test_plot_metrics_generates_single_case_html(tmp_path: Path) -> None:
    case_dir = tmp_path / "case_a"
    write_case(case_dir, make_summary(), make_steps())
    output = tmp_path / "case_a_deep_dive.html"

    plot_metrics.write_single_case_report(case_dir, output, "Case A Deep Dive")
    assert output.exists()
    assert "Case A Deep Dive" in output.read_text(encoding="utf-8")


def test_real_smoke_artifacts_can_be_analyzed(tmp_path: Path) -> None:
    executable = PROJECT_ROOT / "tests" / "artifacts" / "bin" / "agv_console_cpp_only_modern.exe"
    if not executable.exists():
        pytest.skip("smoke executable is not available")

    results_dir = tmp_path / "smoke"
    case_dir = results_dir / "map1_default"
    case_dir.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        [
            str(executable),
            "--headless",
            "--map",
            "1",
            "--algo",
            "default",
            "--mode",
            "custom",
            "--phase",
            "park:1",
            "--max-steps",
            "50",
            "--summary",
            str(case_dir / "run_summary.json"),
            "--steps",
            str(case_dir / "step_metrics.csv"),
        ],
        check=True,
    )

    output_dir = tmp_path / "analysis"
    manifest = analyze_results.generate_analysis_bundle(results_dir, output_dir, "Smoke Analysis")
    assert Path(manifest["assets"]["report"]).exists()
