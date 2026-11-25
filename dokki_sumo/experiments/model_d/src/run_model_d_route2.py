"""
run_model_d_route2.py -- Model D (traffic-signal priority) on Route 2 (node 4 -> hospital_95).

It re-uses the Route 2 Scenario 0 runner (experiments/scenario_0/src/run_scenarios_pair2.py) WITHOUT editing it:
same network, demand files, seed, step length, departure time, disruptions, virtual stop line and
the same fixed 20-edge Route 2 ambulance route, ambulance overtaking and closure handling. The only difference is that after every
simulation step the Model D controller may change the signals on the ambulance route.

Usage (from the RouteX folder, venv active):
    python dokki_sumo/experiments/model_d/src/run_model_d_route2.py
    python dokki_sumo/experiments/model_d/src/run_model_d_route2.py --scenarios S1,S4
    python dokki_sumo/experiments/model_d/src/run_model_d_route2.py --controller-off --output-tag r2_sanity_off

--controller-off runs the identical pipeline with the controller disabled. It must reproduce the
Scenario 0 numbers exactly (a quick sanity check that the hook itself changes nothing).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
SRC_DIR = HERE.parent
MODEL_D_DIR = HERE.parents[1]                       # dokki_sumo/experiments/model_d
SCENARIO0_SRC = HERE.parents[2] / "scenario_0" / "src"
sys.path.insert(0, str(SCENARIO0_SRC))
sys.path.insert(0, str(SRC_DIR))

import run_scenarios_pair2 as base                   # noqa: E402  (Scenario 0 Route 2 baseline runner)
from model_d_controller import ModelDController, load_config, write_csv_rows  # noqa: E402

ROUTE2_ORIGIN = "4"
ROUTE2_DESTINATION = "hospital_95"
BASELINE_ROUTE2 = MODEL_D_DIR.parent / "scenario_0" / "outputs" / "final" / "route_2"

STATE = {"controller": None, "cfg": None, "controller_on": True,
         "events": [], "eta": [], "inter": [], "main": []}

_orig_run_scenario = base.run_scenario
_orig_update_virtual_stop_line = base.update_virtual_stop_line


def _hooked_update_virtual_stop_line(stopped_vehicles):
    """Called by the baseline once per simulation step. Baseline behaviour first, then Model D."""
    _orig_update_virtual_stop_line(stopped_vehicles)
    controller = STATE["controller"]
    if controller is not None:
        controller.step(float(base.traci.simulation.getTime()))


def _run_scenario_with_model_d(scenario, selections, edge_data, *extra_args):
    controller = ModelDController(
        lambda: base.traci, STATE["cfg"], base.AMBULANCE_ID, scenario["id"],
        enabled=STATE["controller_on"],
    )
    STATE["controller"] = controller
    try:
        summary = _orig_run_scenario(scenario, selections, edge_data, *extra_args)
    finally:
        STATE["controller"] = None

    out = base.OUTPUT / f"{scenario['id']}_{scenario['name']}"
    write_csv_rows(out / "model_d_eta_records.csv", controller.eta_rows)
    write_csv_rows(out / "model_d_signal_events.csv", controller.clean_events())
    if controller.trace_rows:
        write_csv_rows(out / "model_d_signal_trace_1s.csv", controller.trace_rows)
    write_csv_rows(out / "model_d_intersection_analysis.csv", controller.intersection_rows())

    extra = {"model_d_controller_on": int(STATE["controller_on"])}
    extra.update(controller.summary())
    summary.update(extra)

    STATE["events"].extend(controller.clean_events())
    STATE["eta"].extend(controller.eta_rows)
    STATE["inter"].extend(controller.intersection_rows())
    STATE["main"].append({
        "scenario_id": summary["scenario_id"], "route_id": "Route 2",
        "ambulance_completed": summary.get("ambulance_completed"),
        "ambulance_travel_time_s": summary.get("ambulance_travel_time_s"),
        "ambulance_waiting_time_s": summary.get("ambulance_waiting_time_s"),
        "ambulance_collision_count": summary.get("ambulance_collision_count"),
        "route_update_count": summary.get("route_update_count"),
        "traffic_average_waiting_s": summary.get("traffic_average_waiting_s"),
        **extra,
    })
    if controller.errors:
        print("WARNING: Model D controller stopped early:", controller.errors[0])
    print(f"[Model D] {summary['scenario_id']}: activations={extra['model_d_priority_activations']} "
          f"actions={extra['model_d_priority_actions']} "
          f"signal_wait={extra['model_d_ambulance_signal_wait_s']}s")
    return summary


def parse_args():
    ap = argparse.ArgumentParser(description="Model D, Route 2: run Scenario 0 conditions with signal priority.")
    ap.add_argument("--scenarios", default="", help="Comma list, e.g. S1,S4 (default: all six).")
    ap.add_argument("--output-tag", default="route_2_model_d")
    ap.add_argument("--config", default=str(MODEL_D_DIR / "config" / "model_d_config_skip.json"))
    ap.add_argument("--seed", type=int, default=1, help="SUMO random seed (default 1 = Scenario 0).")
    ap.add_argument("--controller-off", action="store_true",
                    help="Disable the controller (must reproduce Scenario 0 exactly).")
    return ap.parse_args()


def main():
    args = parse_args()
    STATE["cfg"] = load_config(args.config)
    STATE["controller_on"] = not args.controller_off

    base.BASE = MODEL_D_DIR                           # outputs go to experiments/model_d/outputs/<tag>
    base.run_scenario = _run_scenario_with_model_d
    base.update_virtual_stop_line = _hooked_update_virtual_stop_line

    sys.argv = [sys.argv[0], "--origin", ROUTE2_ORIGIN, "--destination", ROUTE2_DESTINATION,
                "--output-tag", args.output_tag, "--seed", str(args.seed)]
    if args.scenarios.strip():
        sys.argv += ["--scenarios", args.scenarios]
    base.main()

    out = MODEL_D_DIR / "outputs" / args.output_tag
    write_csv_rows(out / "model_d_main_results.csv", sorted(STATE["main"], key=lambda r: r["scenario_id"]))
    write_csv_rows(out / "model_d_eta_records_all.csv", STATE["eta"])
    write_csv_rows(out / "model_d_signal_events_all.csv", STATE["events"])
    write_csv_rows(out / "model_d_intersection_analysis_all.csv", STATE["inter"])
    print(f"\nModel D results saved in: {out}")

    try:
        from compare_with_baseline import build_comparison
        build_comparison(out, baseline_dir=BASELINE_ROUTE2, route_label="Route 2", route_tag="route2")
    except Exception as exc:                          # comparison is a convenience, never fatal
        print("Comparison with baseline was not built:", exc)


if __name__ == "__main__":
    main()
