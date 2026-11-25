
"""Run the four official seed-1 modes using the original Scenario 0 loop."""
import argparse
import json
import runpy
import sys
from pathlib import Path

import networkx as nx

HERE = Path(__file__).resolve().parent
SCENARIO_SRC = HERE.parents[1] / "scenario_0" / "src"
D_SRC = HERE.parents[1] / "model_d" / "src"
D_CONFIG = D_SRC.parent / "config" / "model_d_config_skip_stopped.json"
# These weights influence route ranking only. SUMO retains the official road settings.
SIGNAL_PRIORITY_CREDIT_S = 45.0
HOSPITAL_APPROACH_RADIUS_M = 1000.0
APPROACH_SLOWDOWN_PENALTY_S = 90.0
sys.path[:0] = [str(SCENARIO_SRC), str(D_SRC)]

from model_d_controller import ModelDController, load_config, write_csv_rows
from official_disruptions import apply_official_disruptions

def load_scenario(options):
    """Attach to the seed-1 baseline or to the existing Model C launcher."""
    forwarded = ["--seed", "1", "--output-tag", options.output_tag]
    if options.scenarios:
        forwarded += ["--scenarios", options.scenarios]
    if options.mode in ("c", "cd"):
        path = SCENARIO_SRC / "run_official_model_c.py"
        sys.argv = [str(path), "--route", str(options.route), *forwarded]
        launcher = runpy.run_path(str(path))
        cfg = json.loads(launcher["m"].CONFIG_FILE.read_text(encoding="utf-8"))
        namespace = launcher["load_baseline"](cfg, options.sumo)
        sys.argv = [str(path), *forwarded]
        return namespace
    name = "run_scenarios.py" if options.route == 1 else "run_scenarios_pair2.py"
    path = SCENARIO_SRC / name
    source = path.read_text(encoding="utf-8")
    namespace = {"__name__": "scenario0_integrated", "__file__": str(path)}
    if options.mode == "d":
        anchor = "            sample_edge_metrics_if_due(timestamp)\n\n            if timestamp - last_progress_time"
        if source.count(anchor) != 1:
            raise RuntimeError("Could not locate the official post-routing step")
        source = source.replace(anchor, "            signal_step(timestamp)\n" + anchor)
    exec(compile(source, str(path), "exec"), namespace)
    apply_official_disruptions(namespace, options.route)
    if options.sumo:
        namespace["SUMO"] = options.sumo
    sys.argv = [str(path), *forwarded]
    return namespace

def attach_signals(namespace, config, enabled):
    """Read the applied route after the routing decision, with one D instance per run."""
    original_run = namespace["run_scenario"]
    controller = None
    def signal_step(timestamp):
        if controller is not None:
            controller.step(float(timestamp))

    if "MC" in namespace:
        mc = namespace["MC"]
        original_costs = mc.costs
        signal_edges = None
        distances_by_destination = {}

        def costs_with_signal_priority(closed):
            nonlocal signal_edges
            costs = original_costs(closed)
            if signal_edges is None:
                signal_edges = {
                    mc.T.lane.getEdgeID(link[0])
                    for tls in mc.T.trafficlight.getIDList()
                    for group in mc.T.trafficlight.getControlledLinks(tls)
                    for link in group
                }
            approach_slowed = set()
            slowed = mc.live_slow_edges & costs.keys()
            if slowed:
                destination = mc.T.vehicle.getRoute(mc.amb)[-1]
                if destination not in distances_by_destination:
                    distances_by_destination[destination] = nx.single_source_dijkstra_path_length(
                        mc.G.reverse(copy=False), destination,
                        cutoff=HOSPITAL_APPROACH_RADIUS_M,
                        weight=lambda _, edge, _link: mc.edge_data[edge]["length"] if edge in mc.edge_data else None,
                    )
                approach_slowed = slowed & distances_by_destination[destination].keys()
            if approach_slowed and closed:
                # Keep the original forced-closure policy when a destination
                # approach is also slowed. The signal controller still runs.
                return costs
            for edge in signal_edges & costs.keys():
                free = mc.edge_data[edge]["length"] / max(mc.T.lane.getMaxSpeed(edge + "_0"), 0.1)
                costs[edge] -= min(SIGNAL_PRIORITY_CREDIT_S, max(0.0, costs[edge] - free))
            for edge in approach_slowed:
                costs[edge] += APPROACH_SLOWDOWN_PENALTY_S
            return costs

        if enabled:
            mc.costs = costs_with_signal_priority
        original_step = namespace["MC"].step
        def step_with_signals(*args, **kwargs):
            original_step(*args, **kwargs)
            signal_step(kwargs["timestamp"])
        namespace["MC"].step = step_with_signals
    else:
        namespace["signal_step"] = signal_step

    def run_with_signals(scenario, *args, **kwargs):
        nonlocal controller
        traci = namespace["traci"]
        controller = ModelDController(lambda: traci, config.copy(), namespace["AMBULANCE_ID"],
                                      scenario["id"], enabled=enabled)
        original_close = traci.close
        def close_with_signal_release(*close_args, **close_kwargs):
            try:
                if enabled and not controller.errors:
                    controller._release_all(float(traci.simulation.getTime()))
            finally:
                original_close(*close_args, **close_kwargs)
        traci.close = close_with_signal_release
        try:
            result = original_run(scenario, *args, **kwargs)
            out = namespace["OUTPUT"] / f"{scenario['id']}_{scenario['name']}"
            write_csv_rows(out / "model_d_signal_events.csv", controller.clean_events())
            write_csv_rows(out / "model_d_eta_records.csv", controller.eta_rows)
            write_csv_rows(out / "model_d_intersection_analysis.csv", controller.intersection_rows())
            (out / "model_d_status.json").write_text(
                json.dumps({"enabled": enabled, "errors": controller.errors, **controller.summary()}, indent=2),
                encoding="utf-8",
            )
            return result
        finally:
            traci.close = original_close
            controller = None
    namespace["run_scenario"] = run_with_signals

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--route", type=int, choices=(1, 2), required=True)
    parser.add_argument("--mode", choices=("baseline", "c", "d", "cd"), required=True)
    parser.add_argument("--output-tag", required=True)
    parser.add_argument("--scenarios", default="")
    parser.add_argument("--sumo", default="")
    parser.add_argument("--controller-off", action="store_true", help="Disable the attached D controller for parity checks.")
    options = parser.parse_args()
    if options.controller_off and options.mode not in ("d", "cd"):
        parser.error("--controller-off requires mode d or cd")
    namespace = load_scenario(options)
    if options.mode in ("d", "cd"):
        attach_signals(namespace, load_config(D_CONFIG), not options.controller_off)
    namespace["main"]()

if __name__ == "__main__":
    main()
