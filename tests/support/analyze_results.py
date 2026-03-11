from __future__ import annotations

import argparse
from datetime import UTC, datetime
import html
import json
from pathlib import Path

try:
    import plotly.express as px
except ImportError as exc:  # pragma: no cover - guarded at runtime
    px = None
    _PLOTLY_IMPORT_ERROR = exc
else:
    _PLOTLY_IMPORT_ERROR = None

from analysis_pipeline import (
    DEFAULT_ANALYSIS_DIR,
    AnalysisResults,
    anomaly_rows,
    load_results_bundle,
    require_pandas,
)


def require_plotly() -> None:
    if px is None:  # pragma: no cover - exercised only when dependency is missing
        raise RuntimeError(
            "plotly is required for AGV HTML visualization. "
            "Install it with: python -m pip install pandas plotly pytest"
        ) from _PLOTLY_IMPORT_ERROR


def _figure_html(fig, *, include_js: bool) -> str:
    return fig.to_html(full_html=False, include_plotlyjs="inline" if include_js else False, config={"displaylogo": False})


def _render_cards(bundle: AnalysisResults) -> str:
    case_summary = bundle.case_summary
    total_tasks = int(case_summary["tasks_completed_total"].sum())
    avg_throughput = float(case_summary["throughput"].mean()) if not case_summary.empty else 0.0
    total_deadlocks = int(case_summary["deadlock_count"].sum())
    total_movement = float(case_summary["total_movement_cost"].sum())
    avg_planning = float(case_summary["avg_planning_time_ms"].mean()) if not case_summary.empty else 0.0
    peak_memory = float(case_summary["memory_usage_peak_kb"].max()) if not case_summary.empty else 0.0

    cards = [
        ("Tasks Completed", f"{total_tasks}"),
        ("Average Throughput", f"{avg_throughput:.4f}"),
        ("Deadlocks", f"{total_deadlocks}"),
        ("Total Movement", f"{total_movement:.2f}"),
        ("Avg Planning (ms)", f"{avg_planning:.4f}"),
        ("Peak Memory (KB)", f"{peak_memory:.2f}"),
    ]
    inner = "".join(
        f"<div class='card'><div class='card-label'>{html.escape(label)}</div><div class='card-value'>{html.escape(value)}</div></div>"
        for label, value in cards
    )
    return f"<section><h2>Overview</h2><div class='cards'>{inner}</div></section>"


def _render_batch_charts(bundle: AnalysisResults) -> str:
    case_summary = bundle.case_summary.copy()
    if case_summary.empty:
        return "<section><h2>Batch Comparison</h2><p>No case summaries were available.</p></section>"

    figures = []
    include_js = True

    tasks_fig = px.bar(
        case_summary.sort_values("tasks_completed_total", ascending=False),
        x="case",
        y="tasks_completed_total",
        color="path_algo_label",
        title="Tasks Completed by Case",
        hover_data=["map_id", "deadlock_count", "avg_planning_time_ms"],
    )
    figures.append(_figure_html(tasks_fig, include_js=include_js))
    include_js = False

    planning_scatter = px.scatter(
        case_summary,
        x="avg_planning_time_ms",
        y="throughput",
        size="memory_usage_peak_kb",
        color="path_algo_label",
        hover_name="case",
        title="Planning Time vs Throughput",
    )
    figures.append(_figure_html(planning_scatter, include_js=include_js))

    anomaly_fig = px.bar(
        case_summary.sort_values("anomaly_score", ascending=False),
        x="case",
        y=["deadlock_count", "max_zero_progress_streak", "planning_spike_count"],
        barmode="group",
        title="Deadlocks, Zero-Progress Streaks, and Planning Spikes",
    )
    figures.append(_figure_html(anomaly_fig, include_js=include_js))

    memory_fig = px.bar(
        case_summary.sort_values("memory_usage_peak_kb", ascending=False),
        x="case",
        y="memory_usage_peak_kb",
        color="path_algo_label",
        title="Peak Memory Ranking",
        hover_data=["avg_planning_time_ms", "throughput", "deadlock_count"],
    )
    figures.append(_figure_html(memory_fig, include_js=include_js))

    return "<section><h2>Batch Comparison</h2>{}</section>".format("".join(figures))


def _render_case_table(bundle: AnalysisResults) -> str:
    columns = [
        "case",
        "map_id",
        "path_algo_label",
        "tasks_completed_total",
        "throughput",
        "deadlock_count",
        "avg_planning_time_ms",
        "memory_usage_peak_kb",
        "max_zero_progress_streak",
        "planning_spike_count",
    ]
    table = bundle.case_summary[columns].to_html(index=False, classes=["dataframe", "summary-table"], border=0)
    return f"<section><h2>Case Summary Table</h2>{table}</section>"


def _render_deep_dives(bundle: AnalysisResults) -> str:
    step_frame = bundle.step_enriched
    if step_frame.empty:
        return "<section><h2>Single-Run Deep Dive</h2><p>No step-level metrics were available.</p></section>"

    parts = ["<section><h2>Single-Run Deep Dive</h2>"]
    include_js = False
    for case_name in bundle.case_summary["case"].tolist():
        case_steps = step_frame.loc[step_frame["case"] == case_name].copy()
        if case_steps.empty:
            continue
        parts.append(f"<article><h3>{html.escape(case_name)}</h3>")

        task_deadlock_fig = px.line(
            case_steps,
            x="step",
            y=["tasks_completed_total", "deadlock_count"],
            title=f"{case_name}: tasks and deadlocks over time",
        )
        parts.append(_figure_html(task_deadlock_fig, include_js=include_js))

        perf_fig = px.line(
            case_steps,
            x="step",
            y=["movement_delta_rolling_avg", "planning_delta_ms_rolling_avg", "cpu_delta_ms_rolling_avg"],
            title=f"{case_name}: rolling movement, planning, and CPU",
        )
        parts.append(_figure_html(perf_fig, include_js=include_js))

        nodes_fig = px.line(
            case_steps,
            x="step",
            y=["algo_nodes_expanded_last", "algo_heap_moves_last", "algo_generated_nodes_last"],
            title=f"{case_name}: search effort by step",
        )
        parts.append(_figure_html(nodes_fig, include_js=include_js))
        parts.append("</article>")
    parts.append("</section>")
    return "".join(parts)


def _render_anomalies(bundle: AnalysisResults) -> str:
    anomalies = anomaly_rows(bundle.step_enriched)
    if anomalies.empty:
        return "<section><h2>Anomalies</h2><p>No anomaly conditions were detected.</p></section>"
    table = anomalies.to_html(index=False, classes=["dataframe", "anomaly-table"], border=0)
    return f"<section><h2>Anomalies</h2>{table}</section>"


def build_report_html(bundle: AnalysisResults, title: str) -> str:
    require_pandas()
    require_plotly()

    sections = [
        _render_cards(bundle),
        _render_batch_charts(bundle),
        _render_case_table(bundle),
        _render_deep_dives(bundle),
        _render_anomalies(bundle),
    ]
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>{html.escape(title)}</title>
  <style>
    body {{ font-family: Segoe UI, Helvetica, Arial, sans-serif; margin: 24px; background: #f5f7fb; color: #1d2433; }}
    h1, h2, h3 {{ color: #10233f; }}
    section, article {{ background: #ffffff; border-radius: 12px; padding: 18px; margin-bottom: 20px; box-shadow: 0 8px 24px rgba(16, 35, 63, 0.08); }}
    .meta {{ color: #55637a; margin-bottom: 20px; }}
    .cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; }}
    .card {{ background: linear-gradient(135deg, #f3f7ff, #eef4ea); border: 1px solid #d9e4f2; border-radius: 10px; padding: 14px; }}
    .card-label {{ font-size: 0.85rem; color: #5f6f86; }}
    .card-value {{ font-size: 1.5rem; font-weight: 700; margin-top: 6px; }}
    table {{ width: 100%; border-collapse: collapse; }}
    th, td {{ padding: 8px 10px; border-bottom: 1px solid #e4e8f0; text-align: left; }}
    th {{ background: #f4f7fb; }}
  </style>
</head>
<body>
  <h1>{html.escape(title)}</h1>
  <p class="meta">Source: {html.escape(str(bundle.results_dir))} | Generated: {html.escape(datetime.now(UTC).isoformat())}</p>
  {''.join(sections)}
</body>
</html>
"""


def write_single_case_report(case_dir: Path, output_path: Path, title: str | None = None) -> Path:
    bundle = load_results_bundle(case_dir)
    report_title = title or f"AGV Deep Dive - {bundle.case_summary.iloc[0]['case']}"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(build_report_html(bundle, report_title), encoding="utf-8")
    return output_path


def generate_analysis_bundle(results_dir: Path, output_dir: Path = DEFAULT_ANALYSIS_DIR, title: str = "AGV Analysis Report") -> dict[str, object]:
    bundle = load_results_bundle(results_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    case_summary_path = output_dir / "case_summary.csv"
    step_enriched_path = output_dir / "step_enriched.csv"
    report_path = output_dir / "report.html"
    manifest_path = output_dir / "manifest.json"

    bundle.case_summary.to_csv(case_summary_path, index=False, encoding="utf-8")
    bundle.step_enriched.to_csv(step_enriched_path, index=False, encoding="utf-8")
    report_path.write_text(build_report_html(bundle, title), encoding="utf-8")

    manifest = {
        "title": title,
        "generated_at_utc": datetime.now(UTC).isoformat(),
        "results_dir": str(Path(results_dir).resolve()),
        "case_count": int(len(bundle.case_summary)),
        "step_row_count": int(len(bundle.step_enriched)),
        "assets": {
            "report": str(report_path),
            "case_summary_csv": str(case_summary_path),
            "step_enriched_csv": str(step_enriched_path),
            "manifest": str(manifest_path),
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate an AGV HTML analysis report from simulation results.")
    parser.add_argument("results_dir", type=Path, help="Results directory containing case subfolders or a single case folder")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_ANALYSIS_DIR, help="Directory where the report bundle will be written")
    parser.add_argument("--title", type=str, default="AGV Analysis Report", help="Custom report title")
    args = parser.parse_args()

    manifest = generate_analysis_bundle(args.results_dir, args.output_dir, args.title)
    print(f"analysis report written to {manifest['assets']['report']}")


if __name__ == "__main__":
    main()
