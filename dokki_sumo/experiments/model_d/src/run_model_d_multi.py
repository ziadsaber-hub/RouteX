"""
run_model_d_multi.py -- Model D with SEVERAL ambulances on the same route (Route 1: 181 -> hospital_168).

Extra experiment requested in the Scenario 0 conditions ("try to add multiple ambulances in the same route
and start and destination points to see the controller priority").

Set-up (everything else is the Scenario 0 runner, imported and NOT edited):
  * ambulance_1 departs at 600 s exactly as in Scenario 0 and ends the run when it arrives.
  * ambulance_2 .. ambulance_N depart BEFORE it, every --gap seconds (default 15 s), on the SAME route
    (one route is computed once and shared). So ambulance_1 is the LAST vehicle of the convoy.
  * One Model D controller per ambulance; they share a "fleet" so a green that is being held also covers
    ambulances following within convoy_window_s. All ambulances use the same approach, so no two
    requests ever clash (rule R6 is not needed here).
  * Closures (S5, S6) are not used: only ambulance_1 has the Scenario 0 reactive reroute.

Usage (from the RouteX folder, venv active):
    python dokki_sumo/experiments/model_d/src/run_model_d_multi.py --controller-off --output-tag multi_3_off
    python dokki_sumo/experiments/model_d/src/run_model_d_multi.py --output-tag multi_3_model_d
    python dokki_sumo/experiments/model_d/src/compare_multi.py
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
SRC_DIR = HERE.parent
MODEL_D_DIR = HERE.parents[1]
SCENARIO0_SRC = HERE.parents[2] / "scenario_0" / "src"
sys.path.insert(0, str(SCENARIO0_SRC))
sys.path.insert(0, str(SRC_DIR))

import run_scenarios as base                         # noqa: E402
from model_d_controller import ModelDController, load_config, write_csv_rows  # noqa: E402

ROUTE1_ORIGIN = "181"
ROUTE1_DESTINATION = "hospital_168"

STATE = {"cfg": None, "controller_on": True, "n": 3, "gap": 15.0,
         "controllers": [], "extras": [], "route": None, "scenario_id": None, "errors": [],
         "main": [], "events": [], "eta": [], "inter": []}

_orig_run_scenario = base.run_scenario
_orig_update_virtual_stop_line = base.update_virtual_stop_line
_orig_find_initial_route = base.find_initial_route


def _shared_route():
    """The first call computes the route; every later call (ambulance_1 at 600 s) reuses it."""
    if STATE["route"] is None:
        STATE["route"] = _orig_find_initial_route()
    return STATE["route"]


def _insert_due_extras(now):
    for item in STATE["extras"]:
        if item["inserted"] or now < item["depart"]:
            continue
        item["inserted"] = True
        try:
            route = _shared_route()
            route_id = f"ambulance_route_{STATE['scenario_id']}_multi"
            if not item.get("route_added") and not STATE.get("route_id_added"):
                base.traci.route.add(route_id, list(route.edges))
                STATE["route_id_added"] = route_id
            base.traci.vehicle.add(
                vehID=item["id"], routeID=STATE["route_id_added"], typeID=base.AMBULANCE_TYPE,
                depart="now", departLane="best", departSpeed="max",
            )
        except Exception as exc:                       # keep the scenario running; report it
            STATE["errors"].append(f"{item['id']}: {type(exc).__name__}: {exc}")


def _hooked_update_virtual_stop_line(stopped_vehicles):
    _orig_update_virtual_stop_line(stopped_vehicles)
    now = float(base.traci.simulation.getTime())
    _insert_due_extras(now)
    for controller in STATE["controllers"]:
        controller.step(now)


def _run_scenario_multi(scenario, selections, edge_data):
    sid = scenario["id"]
    fleet = {}
    ids = [f"ambulance_{k}" for k in range(STATE["n"], 1, -1)] + [base.AMBULANCE_ID]   # earliest first
    STATE["controllers"] = [ModelDController(lambda: base.traci, STATE["cfg"], amb, sid,
                                             enabled=STATE["controller_on"], fleet=fleet) for amb in ids]
    STATE["extras"] = [{"id": f"ambulance_{k}", "depart": float(base.DEPARTURE_TIME) - STATE["gap"] * (k - 1),
                        "inserted": False} for k in range(2, STATE["n"] + 1)]
    STATE["route"], STATE["scenario_id"], STATE["errors"] = None, sid, []
    STATE.pop("route_id_added", None)
    try:
        summary = _orig_run_scenario(scenario, selections, edge_data)
    finally:
        controllers = STATE["controllers"]
        STATE["controllers"] = []

    out = base.OUTPUT / f"{sid}_{scenario['name']}"
    trips = {r["vehicle_id"]: r for r in base.read_tripinfo(out / "traffic_tripinfo.xml")
             if r["vehicle_id"].startswith("ambulance")}
    rows = []
    for amb, ctl in zip(ids, controllers):
        trip = trips.get(amb)
        info = ctl.summary()
        rows.append({
            "scenario_id": sid, "ambulance_id": amb,
            "depart_s": "" if not trip else trip["depart_s"],
            "arrival_s": "" if not trip else trip["arrival_s"],
            "travel_time_s": "" if not trip else trip["duration_s"],
            "waiting_time_s": "" if not trip else trip["waiting_time_s"],
            "completed": int(trip is not None),
            "controller_on": int(STATE["controller_on"]),
            "priority_activations": info["model_d_priority_activations"],
            "ambulance_signal_wait_s": info["model_d_ambulance_signal_wait_s"],
            "eta_mae_s": info["model_d_eta_mae_s"],
        })
    write_csv_rows(out / "model_d_multi_ambulances.csv", rows)
    STATE["main"].extend(rows)
    for ctl in controllers:
        STATE["events"].extend(ctl.clean_events())
        STATE["eta"].extend(ctl.eta_rows)
        STATE["inter"].extend(ctl.intersection_rows())
        if ctl.errors:
            print("WARNING: controller stopped early:", ctl.errors[0])
    for msg in STATE["errors"]:
        print("WARNING: extra ambulance not inserted:", msg)
    done = sum(r["completed"] for r in rows)
    print(f"[Model D multi] {sid}: {done}/{len(rows)} ambulances finished; "
          + ", ".join(f"{r['ambulance_id']}={r['travel_time_s']}s" for r in rows))
    return summary


def parse_args():
    ap = argparse.ArgumentParser(description="Model D with several ambulances on the same route.")
    ap.add_argument("--ambulances", type=int, default=3)
    ap.add_argument("--gap", type=float, default=15.0, help="seconds between consecutive ambulances")
    ap.add_argument("--scenarios", default="S1,S2,S3,S4")
    ap.add_argument("--output-tag", default="multi_3_model_d")
    ap.add_argument("--config", default=str(MODEL_D_DIR / "config" / "model_d_config.json"))
    ap.add_argument("--controller-off", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    STATE.update(cfg=load_config(args.config), controller_on=not args.controller_off,
                 n=max(args.ambulances, 1), gap=args.gap)
    base.BASE = MODEL_D_DIR
    base.run_scenario = _run_scenario_multi
    base.update_virtual_stop_line = _hooked_update_virtual_stop_line
    base.find_initial_route = _shared_route

    sys.argv = [sys.argv[0], "--origin", ROUTE1_ORIGIN, "--destination", ROUTE1_DESTINATION,
                "--output-tag", args.output_tag, "--scenarios", args.scenarios]
    base.main()

    out = MODEL_D_DIR / "outputs" / args.output_tag
    write_csv_rows(out / "model_d_multi_ambulances_all.csv", STATE["main"])
    write_csv_rows(out / "model_d_signal_events_all.csv", STATE["events"])
    write_csv_rows(out / "model_d_eta_records_all.csv", STATE["eta"])
    write_csv_rows(out / "model_d_intersection_analysis_all.csv", STATE["inter"])
    print(f"\nMulti-ambulance results saved in: {out}")


if __name__ == "__main__":
    main()
