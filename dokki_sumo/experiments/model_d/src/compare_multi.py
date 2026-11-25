"""
compare_multi.py -- multi-ambulance experiment: Model D ON vs OFF, per ambulance.

    python dokki_sumo/experiments/model_d/src/compare_multi.py [<on_folder>] [<off_folder>]
Defaults: outputs/multi_3_model_d  vs  outputs/multi_3_off.
Output: <on_folder>/model_d_multi_vs_off.csv
time saved = OFF travel time - ON travel time (positive = Model D is faster).
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

MODEL_D_DIR = Path(__file__).resolve().parents[1]


def _read(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        return {(r["scenario_id"], r["ambulance_id"]): r for r in csv.DictReader(f)}


def main(on_dir=MODEL_D_DIR / "outputs" / "multi_3_model_d", off_dir=MODEL_D_DIR / "outputs" / "multi_3_off"):
    on = _read(Path(on_dir) / "model_d_multi_ambulances_all.csv")
    off = _read(Path(off_dir) / "model_d_multi_ambulances_all.csv")
    rows = []
    for key in sorted(on):
        a, b = on[key], off.get(key)
        row = {"scenario_id": key[0], "ambulance_id": key[1],
               "completed_off": b["completed"] if b else "", "completed_on": a["completed"],
               "travel_time_off_s": b["travel_time_s"] if b else "", "travel_time_on_s": a["travel_time_s"],
               "time_saved_s": "", "improvement_pct": "",
               "signal_wait_on_s": a["ambulance_signal_wait_s"]}
        if b and a["travel_time_s"] != "" and b["travel_time_s"] != "":
            saved = float(b["travel_time_s"]) - float(a["travel_time_s"])
            row["time_saved_s"] = round(saved, 1)
            row["improvement_pct"] = round(100 * saved / float(b["travel_time_s"]), 1)
        rows.append(row)
    if not rows:
        print("Nothing to compare. Check the two folders.")
        return
    path = Path(on_dir) / "model_d_multi_vs_off.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print("\nMULTI-AMBULANCE: Model D ON vs OFF (same ambulances, same departures)")
    print(f"{'Scn':4}{'Ambulance':14}{'off (s)':>9}{'on (s)':>9}{'saved':>8}{'impr%':>8}{'sig wait':>10}")
    for r in rows:
        print(f"{r['scenario_id']:4}{r['ambulance_id']:14}{str(r['travel_time_off_s']):>9}"
              f"{str(r['travel_time_on_s']):>9}{str(r['time_saved_s']):>8}{str(r['improvement_pct']):>8}"
              f"{str(r['signal_wait_on_s']):>10}")
    print(f"Saved: {path}")


if __name__ == "__main__":
    args = sys.argv[1:]
    main(*args[:2]) if args else main()
