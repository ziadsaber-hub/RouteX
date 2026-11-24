"""
Model C - turn one run folder (outputs/model_c_route_N) into the six deliverable CSVs,
in the same format as the Route 1 deliverables:

    model_c_main_results.csv, baseline_comparison.csv, reroute_events.csv,
    alternatives_log.csv, initial_route_selection.csv, route_history_all.csv

Called automatically at the end of model_c_runner.py. Can also be run by hand:
    python model_c_export.py --run-dir ../outputs/model_c_route_2 --route-id "Route 2"
"""
import argparse
import csv
from pathlib import Path

SIMILAR_PCT = 2.0
SCEN_ORDER = ["S1", "S2", "S3", "S4", "S5", "S6"]


def _read(path):
    path = Path(path)
    if not path.exists():
        return []
    with path.open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def _write(path, rows, fields):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def _f(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _travelled_route(history):
    """Edges actually used: each route version up to the edge where the next one started."""
    versions = {}
    for h in history:
        versions.setdefault(int(h["route_version"]), []).append(h)
    order = sorted(versions)
    edges_of = {v: [r["edge_id"] for r in sorted(versions[v], key=lambda r: int(r["route_index"]))]
                for v in order}
    out = []
    for i, v in enumerate(order):
        edges = edges_of[v]
        if i + 1 < len(order):
            nxt_start = edges_of[order[i + 1]][0]
            cut = edges.index(nxt_start) if nxt_start in edges else len(edges)
            out += edges[:cut]
        else:
            out += edges
    return edges_of.get(order[0], []) if order else [], out


def export(run_dir, route_id, baseline_csv, out_dir):
    run_dir, out_dir = Path(run_dir), Path(out_dir)
    base = {r["scenario_id"]: r for r in _read(baseline_csv) if r.get("route_id") == route_id}
    scen_dirs = sorted((d for d in run_dir.iterdir() if d.is_dir() and d.name[:2] in SCEN_ORDER),
                       key=lambda d: SCEN_ORDER.index(d.name[:2]))

    main_rows, comp_rows, events, alts, history_all = [], [], [], [], []
    for d in scen_dirs:
        summ = _read(d / "scenario_summary.csv")
        if not summ:
            continue
        s = summ[0]
        sid = s["scenario_id"]
        hist = _read(d / "ambulance_route_history.csv")
        history_all += hist
        initial, travelled = _travelled_route(hist)
        ev = [e for e in _read(d / "model_c_reroute_events.csv") if e.get("reroute_event_id")]
        for e in ev:
            e["route_id"] = route_id
        events += ev
        alts += [a for a in _read(d / "model_c_alternatives_log.csv") if a.get("reroute_event_id") not in (None, "")]

        completed = str(s.get("ambulance_completed")).lower() == "true"
        tt = _f(s.get("ambulance_travel_time_s"), None)
        b = base.get(sid, {})
        bt = _f(b.get("ambulance_travel_time_s"), None)
        saved = round(bt - tt, 2) if bt is not None and tt is not None else ""
        pct = round(saved / bt * 100, 2) if saved != "" and bt else ""
        coll = int(_f(s.get("ambulance_collision_count")))
        if not completed or coll > 0:
            cls = "fail"
        elif pct == "":
            cls = ""
        else:
            cls = "similar" if abs(pct) < SIMILAR_PCT else ("improved" if pct > 0 else "worse")

        main_rows.append({
            "scenario": sid, "scenario_name": s.get("scenario", ""), "route_id": route_id,
            "initial_route": " ".join(initial), "final_route": " ".join(travelled),
            "actual_travel_time_s": tt, "waiting_time_s": _f(s.get("ambulance_waiting_time_s")),
            "route_distance_m": _f(s.get("ambulance_distance_m")),
            "route_updates": int(_f(s.get("route_update_count"))),
            "completion_status": "completed" if completed else "not_completed",
            "ambulance_collisions": coll, "baseline_travel_time_s": bt if bt is not None else "",
            "time_saved_s": saved, "improvement_pct": pct,
        })
        bw = _f(b.get("ambulance_waiting_time_s"), None)
        comp_rows.append({
            "scenario": sid, "route_id": route_id, "baseline_travel_s": bt, "model_c_travel_s": tt,
            "time_saved_s": saved, "improvement_pct": pct, "classification": cls,
            "baseline_waiting_s": bw, "model_c_waiting_s": _f(s.get("ambulance_waiting_time_s")),
            "waiting_reduction_s": round(bw - _f(s.get("ambulance_waiting_time_s")), 2) if bw is not None else "",
            "baseline_distance_m": _f(b.get("ambulance_distance_m"), ""),
            "model_c_distance_m": _f(s.get("ambulance_distance_m")),
            "baseline_route_updates": int(_f(b.get("route_update_count"))),
            "model_c_route_updates": int(_f(s.get("route_update_count"))),
            "baseline_ambulance_collisions": int(_f(b.get("ambulance_collision_count"))),
            "model_c_ambulance_collisions": coll,
        })

    alt_fields = ["scenario_id", "reroute_event_id", "alt_index", "selected", "predicted_time_s",
                  "detour_ratio", "length_m", "overlap_with_fastest", "edge_count", "edges"]
    _write(out_dir / "model_c_main_results.csv", main_rows,
           ["scenario", "scenario_name", "route_id", "initial_route", "final_route",
            "actual_travel_time_s", "waiting_time_s", "route_distance_m", "route_updates",
            "completion_status", "ambulance_collisions", "baseline_travel_time_s",
            "time_saved_s", "improvement_pct"])
    _write(out_dir / "baseline_comparison.csv", comp_rows, list(comp_rows[0]) if comp_rows else ["scenario"])
    _write(out_dir / "reroute_events.csv", events,
           ["scenario_id", "route_id", "reroute_event_id", "simulation_time_s", "current_edge",
            "current_route", "trigger_reason", "trigger_value", "threshold", "number_of_alternatives",
            "selected_alternative", "selected_route", "current_route_estimate_s", "predicted_time_s",
            "result", "actual_remaining_time_s", "prediction_error_s"])
    _write(out_dir / "alternatives_log.csv", alts, alt_fields)
    _write(out_dir / "initial_route_selection.csv",
           [a for a in alts if str(a["reroute_event_id"]) == "0"], alt_fields)
    _write(out_dir / "route_history_all.csv", history_all,
           ["scenario_id", "route_version", "route_start_time_s", "route_reason",
            "trigger_edge", "route_index", "edge_id"])

    print(f"\nModel C deliverables written to {out_dir}")
    for r in comp_rows:
        print(f"  {r['scenario']}: baseline {r['baseline_travel_s']} s -> Model C {r['model_c_travel_s']} s "
              f"({r['improvement_pct']}%, {r['classification']}), reroutes {r['model_c_route_updates']}, "
              f"ambulance collisions {r['model_c_ambulance_collisions']}")
    return out_dir


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default=str(here.parent / "outputs" / "model_c_route_2"))
    ap.add_argument("--route-id", default="Route 2")
    ap.add_argument("--baseline", default=str(here.parent / "outputs" / "final" / "all_routes_summary.csv"))
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    export(a.run_dir, a.route_id, a.baseline, a.out or Path(a.run_dir) / "deliverables")
