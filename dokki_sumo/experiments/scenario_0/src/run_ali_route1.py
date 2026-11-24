# Ali's Model C (models/model_C/routing.py, saved here as ali_routing.py) run on Route 1.
# Same settings as his config.json; his stored routes are for Route 2 only, so none are used here.
# Run from the src folder:  python run_ali_route1.py [--seed N] [--output-tag NAME]
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import networkx as nx
from sumolib.net import readNet

from ali_routing import RoutingPolicy

CONFIG = {
    "candidate_count": 4, "check_interval_s": 15, "reroute_cooldown_s": 45,
    "minimum_saving_s": 16, "minimum_saving_fraction": 0.12,
    "observed_speed_weight": 0.65, "minimum_speed_fraction": 0.2,
    "ambulance_max_speed_mps": 16.67, "gui_delay_ms": 12, "gui_focus_pause_s": 2,
    "avoid_after_closure": ["e31to144k0", "e170to38k0"],
    "lane_change_lookahead_m": 70,
    "calibration": {"initial_by_demand_file": {}, "detour_by_disruption": {}},
}

if "--output-tag" not in sys.argv:
    sys.argv += ["--output-tag", "ali_route_1"]

HERE = Path(__file__).resolve().parent
src = (HERE / "run_scenarios.py").read_text(encoding="utf-8")
PATCHES = [
    ("        best_route = find_initial_route()\n"
     "        ambulance_route = list(best_route.edges)\n"
     "        estimated_time = float(best_route.travelTime)\n",
     "        best_route = find_initial_route()\n"
     "        ALI.destination = list(best_route.edges)[-1]\n"
     "        ambulance_route, estimated_time = ALI.initial_route(traci, list(best_route.edges))\n"
     "        ambulance_route = list(ambulance_route)\n"
     "        estimated_time = float(estimated_time)\n"),
    ("            sample_edge_metrics_if_due(timestamp)\n\n            if timestamp - last_progress_time",
     "            _upd = ALI.evaluate(traci, timestamp, closed_edges, congested_edges)\n"
     "            if _upd is not None:\n"
     "                reroute_state['successful_reroutes'] += 1\n"
     "                reroute_state['route_version'] += 1\n"
     "                reroute_state['last_movement_time'] = timestamp\n"
     "                append_route_history(route_history, scenario_id, _upd, reroute_state['route_version'],\n"
     "                                     timestamp, 'model_c_reroute', traci.vehicle.getRoadID(AMBULANCE_ID))\n"
     "            reroute_state['failed_reroutes'] = ALI.failed_updates\n"
     "            sample_edge_metrics_if_due(timestamp)\n\n            if timestamp - last_progress_time"),
]
for old, new in PATCHES:
    if src.count(old) != 1:
        raise RuntimeError("Patch anchor not found exactly once: " + old.strip()[:60])
    src = src.replace(old, new)

ns = {"__name__": "run_scenarios_ali", "__file__": str(HERE / "run_scenarios.py")}
exec(compile(src, str(HERE / "run_scenarios.py"), "exec"), ns)
ns["try_reactive_ambulance_reroute"] = lambda **kw: None      # Model C replaces the baseline reroute

_run = ns["run_scenario"]
_cache = {}


def run_scenario(scenario, selections, edge_data):
    if "graph" not in _cache:
        g = ns["load_edge_transition_graph"]()
        net = readNet(str(ns["NETWORK"]))
        _cache["graph"] = g.subgraph([e for e in g if net.getEdge(e).allows("emergency")])
        moves = set()
        for c in ET.parse(ns["NETWORK"]).getroot().findall("connection"):
            f, t, fl = c.get("from"), c.get("to"), c.get("fromLane")
            if f and t and fl is not None and not f.startswith(":") and not t.startswith(":"):
                moves.add((f, int(fl), t))
        _cache["lanes"] = moves
    shim = types.SimpleNamespace(
        AMBULANCE_ID=ns["AMBULANCE_ID"], PAIR2_REFERENCE_ROUTE=["?"],
        CONGESTION_SPEED_FACTOR=ns["CONGESTION_SPEED_FACTOR"],
        PAIR2_ALL_CLOSURES=list(selections.get("all_closures", [])),
        PAIR2_CONGESTION_EDGES=list(selections.get("congestion_edges", [])),
        write_csv=ns["write_csv"])
    ns["ALI"] = RoutingPolicy(shim, _cache["graph"], edge_data, _cache["lanes"], CONFIG, False,
                              demand_file=scenario["demand"])
    summary = _run(scenario, selections, edge_data)
    return summary


ns["run_scenario"] = run_scenario
ns["main"]()
