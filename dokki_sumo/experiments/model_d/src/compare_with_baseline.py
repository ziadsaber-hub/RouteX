"""
compare_with_baseline.py -- Model D vs Scenario 0 (Route 1).

    Time saved      = baseline travel time - Model D travel time
    Improvement (%) = time saved / baseline travel time * 100

Baseline = the validated Scenario 0 results in
    dokki_sumo/experiments/scenario_0/outputs/final/route_1/
Model D  = the folder produced by run_model_d_route1.py.

"Stopped on approach" is measured the same way for both: seconds the ambulance had speed < 0.1 m/s
while on the approach edge of a signalized intersection of its route (from ambulance_positions.csv).
It includes waiting behind a queue; Model D additionally reports the part that happened on a
non-green signal (model_d_ambulance_signal_wait_s).

Usage:  python compare_with_baseline.py <model_d_output_folder>
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

MODEL_D_DIR = Path(__file__).resolve().parents[1]
BASELINE_DIR = MODEL_D_DIR.parent / "scenario_0" / "outputs" / "final" / "route_1"


def _read(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _stopped_on_edges(positions_csv, edges, threshold=0.1):
    if not Path(positions_csv).exists():
        return None
    return float(sum(
        1 for r in _read(positions_csv)
        if r["road_id"] in edges and (_f(r["speed_mps"]) or 0.0) < threshold
    ))


def _pct(delta, base):
    return round(delta / base * 100.0, 1) if base else ""


def build_comparison(out_dir, baseline_dir=BASELINE_DIR, route_label="Route 1", route_tag="route1"):
    out_dir, baseline_dir = Path(out_dir), Path(baseline_dir)
    d_rows = {r["scenario_id"]: r for r in _read(out_dir / "all_scenarios_summary.csv")}
    b_rows = {r["scenario_id"]: r for r in _read(baseline_dir / "all_scenarios_summary.csv")}

    approach_edges = set()
    inter_path = out_dir / "model_d_intersection_analysis_all.csv"
    if inter_path.exists():
        approach_edges = {r["approach_edge"] for r in _read(inter_path)}

    rows = []
    for sid in sorted(d_rows):
        d, b = d_rows[sid], b_rows.get(sid)
        if b is None:
            continue
        bt, dt = _f(b["ambulance_travel_time_s"]), _f(d["ambulance_travel_time_s"])
        bw, dw = _f(b["ambulance_waiting_time_s"]), _f(d["ambulance_waiting_time_s"])
        b_stop = _stopped_on_edges(baseline_dir / f"{sid}_{b['scenario']}" / "ambulance_positions.csv", approach_edges)
        d_stop = _stopped_on_edges(out_dir / f"{sid}_{d['scenario']}" / "ambulance_positions.csv", approach_edges)
        btw, dtw = _f(b.get("traffic_average_waiting_s")), _f(d.get("traffic_average_waiting_s"))
        saved = None if bt is None or dt is None else bt - dt
        row = {
            "scenario_id": sid, "scenario": d["scenario"], "route_id": route_label,
            "baseline_travel_time_s": bt, "model_d_travel_time_s": dt,
            "time_saved_s": saved, "improvement_pct": "" if saved is None else _pct(saved, bt),
            "baseline_waiting_s": bw, "model_d_waiting_s": dw,
            "waiting_reduction_s": None if bw is None or dw is None else bw - dw,
            "waiting_reduction_pct": "" if bw is None or dw is None else _pct(bw - dw, bw),
            "baseline_stopped_on_approach_s": b_stop, "model_d_stopped_on_approach_s": d_stop,
            "signal_stop_reduction_s": None if b_stop is None or d_stop is None else b_stop - d_stop,
            "model_d_signal_wait_s": d.get("model_d_ambulance_signal_wait_s"),
            "priority_activations": d.get("model_d_priority_activations"),
            "signalized_intersections_encountered": d.get("model_d_intersections_encountered"),
            "intersections_receiving_priority": d.get("model_d_intersections_receiving_priority"),
            "ambulance_collisions_model_d": d.get("ambulance_collision_count"),
            "baseline_traffic_avg_waiting_s": btw, "model_d_traffic_avg_waiting_s": dtw,
            "traffic_waiting_change_s": None if btw is None or dtw is None else round(dtw - btw, 2),
            "model_d_eta_mae_s": d.get("model_d_eta_mae_s"),
        }
        rows.append(row)

    path = out_dir / f"model_d_vs_baseline_{route_tag}.csv"
    if rows:
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    print(f"\nMODEL D vs SCENARIO 0 ({route_label})")
    print(f"{'Scn':4} {'baseline':>9} {'model D':>8} {'saved':>7} {'impr%':>7} {'signal wait':>12}")
    for r in rows:
        print(f"{r['scenario_id']:4} {r['baseline_travel_time_s']:>9} {r['model_d_travel_time_s']:>8} "
              f"{r['time_saved_s']:>7} {r['improvement_pct']:>7} {r['model_d_signal_wait_s']:>12}")
    print(f"Saved: {path}")
    return rows


if __name__ == "__main__":
    build_comparison(sys.argv[1])
