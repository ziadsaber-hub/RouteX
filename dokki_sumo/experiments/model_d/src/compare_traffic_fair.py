"""
compare_traffic_fair.py -- effect of Model D on ORDINARY traffic, compared fairly.

The averages in model_d_vs_baseline_route1.csv are taken when each run stops (= when the ambulance
arrives). Model D's ambulance arrives earlier, so those averages cover a shorter period and different
vehicles. Here the SAME vehicles are compared: only vehicles that finished their trip in BOTH runs
(baseline and Model D), same scenario, same vehicle id. The ambulance is excluded.

    change = Model D value - baseline value      (negative = better for ordinary traffic)

Usage (from the RouteX folder):
    python dokki_sumo/experiments/model_d/src/compare_traffic_fair.py
    python dokki_sumo/experiments/model_d/src/compare_traffic_fair.py <model_d_output_folder> [<baseline_folder> [<route_tag>]]
Route 2: ... compare_traffic_fair.py <outputs/route_2_model_d> <scenario_0/outputs/final/route_2> route2
Output: <model_d_output_folder>/model_d_traffic_fair_route1.csv
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

MODEL_D_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUT = MODEL_D_DIR / "outputs" / "route_1_model_d"
DEFAULT_BASE = MODEL_D_DIR.parent / "scenario_0" / "outputs" / "final" / "route_1"
AMBULANCE_PREFIX = "ambulance"
METRICS = ("waiting_time_s", "duration_s", "time_loss_s")


def _trips(path):
    path = Path(path)
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8") as f:
        return {r["vehicle_id"]: r for r in csv.DictReader(f)
                if not r["vehicle_id"].startswith(AMBULANCE_PREFIX)}


def _mean(values):
    return sum(values) / len(values) if values else float("nan")


def main(out_dir=DEFAULT_OUT, base_dir=DEFAULT_BASE, route_tag="route1"):
    out_dir, base_dir = Path(out_dir), Path(base_dir)
    rows = []
    for scen_dir in sorted(p for p in out_dir.iterdir() if p.is_dir() and p.name[:2] in
                           {"S1", "S2", "S3", "S4", "S5", "S6"}):
        sid = scen_dir.name[:2]
        base = _trips(base_dir / scen_dir.name / "traffic_tripinfo.csv")
        new = _trips(scen_dir / "traffic_tripinfo.csv")
        if base is None or new is None:
            print(f"{sid}: tripinfo file missing, skipped")
            continue
        common = sorted(set(base) & set(new))
        row = {"scenario_id": sid, "vehicles_finished_baseline": len(base),
               "vehicles_finished_model_d": len(new), "vehicles_compared": len(common)}
        for m in METRICS:
            b = _mean([float(base[v][m]) for v in common])
            n = _mean([float(new[v][m]) for v in common])
            row[f"baseline_{m}"] = round(b, 2)
            row[f"model_d_{m}"] = round(n, 2)
            row[f"change_{m}"] = round(n - b, 2)
        worse = sum(1 for v in common if float(new[v]["waiting_time_s"]) > float(base[v]["waiting_time_s"]) + 5)
        better = sum(1 for v in common if float(new[v]["waiting_time_s"]) < float(base[v]["waiting_time_s"]) - 5)
        row["vehicles_waiting_5s_worse"] = worse
        row["vehicles_waiting_5s_better"] = better
        rows.append(row)

    if not rows:
        print("No scenario folders compared. Check the paths.")
        return
    path = out_dir / f"model_d_traffic_fair_{route_tag}.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("\nORDINARY TRAFFIC, SAME VEHICLES IN BOTH RUNS (change = Model D - baseline, negative = better)")
    print(f"{'Scn':4}{'compared':>9}{'wait base':>11}{'wait D':>9}{'change':>8}{'worse>5s':>10}{'better>5s':>11}")
    for r in rows:
        print(f"{r['scenario_id']:4}{r['vehicles_compared']:>9}{r['baseline_waiting_time_s']:>11}"
              f"{r['model_d_waiting_time_s']:>9}{r['change_waiting_time_s']:>8}"
              f"{r['vehicles_waiting_5s_worse']:>10}{r['vehicles_waiting_5s_better']:>11}")
    print(f"Saved: {path}")


if __name__ == "__main__":
    args = sys.argv[1:]
    main(*(args[:3]) if args else ())
