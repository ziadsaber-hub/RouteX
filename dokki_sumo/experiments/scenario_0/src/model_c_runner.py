"""
Model C - path planning + dynamic rerouting, running on top of the Scenario 0
runner (Route 1).  The baseline file is NOT modified: it is loaded, two small
patches are applied in memory, and the baseline reactive reroute is replaced.
"""
import argparse
import csv
import json
import sys
import types
from pathlib import Path

import networkx as nx

HERE = Path(__file__).resolve().parent
BASE_FILE = HERE / "run_scenarios_pair2.py"
CONFIG_FILE = HERE / "model_c_config.json"
INF = float("inf")


def write_rows(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


class ModelC:
    def __init__(self, ns, cfg):
        self.ns, self.cfg = ns, cfg
        self.G = None            # edge-to-edge transition graph
        self.edge_data = None
        self.results = []        # one row per finished scenario
        self.begin(None)

    # ---------- helpers ----------
    @property
    def T(self):                 # traci can be swapped by main(), so look it up each time
        return self.ns["traci"]

    @property
    def amb(self):
        return self.ns["AMBULANCE_ID"]

    def _load(self):
        if self.G is None:
            self.G = self.ns["load_edge_transition_graph"]()
            _, self.edge_data = self.ns["load_network_graph"]()

    def begin(self, scenario):
        self.scenario = scenario
        self.events, self.alt_rows = [], []
        self.initial_edges, self.current_route = [], []
        self.last_check, self.last_reroute = -1e9, -1e9
        self.failed_blocked = set()

    # ---------- live travel times ----------
    def costs(self, closed):
        T, c = self.T, {}
        floor = self.cfg["min_edge_travel_time_s"]
        for e, d in self.edge_data.items():
            if e in closed:
                continue
            free = d["length"] / max(d["speed"], 0.1)
            try:
                free = d["length"] / max(float(T.lane.getMaxSpeed(e + "_0")), 0.1)
            except Exception:
                pass
            try:
                tt = float(T.edge.getTraveltime(e))
            except Exception:
                tt = free
            c[e] = min(max(tt, free, floor), free * 10)
        return c

    def search(self, start, dst, costs, closed, pen=None):
        pen = pen or {}

        def w(u, v, d):
            if v in closed or v not in costs:
                return None
            return costs[v] * pen.get(v, 1.0)

        try:
            return nx.dijkstra_path(self.G, start, dst, weight=w)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None

    @staticmethod
    def route_cost(route, costs, first_fraction=1.0):
        if not route:
            return INF
        total = first_fraction * costs.get(route[0], 0.0)
        for e in route[1:]:
            if e not in costs:
                return INF
            total += costs[e]
        return total

    def length(self, route):
        return sum(self.edge_data[e]["length"] for e in route)

    def overlap(self, p, q):
        shared = set(p) & set(q)
        s = sum(self.edge_data[e]["length"] for e in shared)
        return s / max(1e-9, min(self.length(p), self.length(q)))

    def alternatives(self, start, dst, costs, closed, best, best_cost, first_fraction):
        """Penalty method: fastest route + up to k-1 diverse, bounded-detour alternatives."""
        cfg = self.cfg
        routes = [(best, best_cost)]
        pen, last = {}, best
        for _ in range(cfg["penalty_tries"]):
            if len(routes) >= cfg["k_alternatives"]:
                break
            for e in last[1:-1]:
                pen[e] = pen.get(e, 1.0) * cfg["penalty_factor"]
            cand = self.search(start, dst, costs, closed, pen)
            if cand is None:
                break
            last = cand
            if any(cand == r for r, _ in routes):
                continue
            cost = self.route_cost(cand, costs, first_fraction)
            if cost > cfg["max_detour_ratio"] * best_cost:
                continue
            if any(self.overlap(cand, r) > cfg["max_overlap"] for r, _ in routes):
                continue
            routes.append((cand, cost))
        return routes

    def log_alternatives(self, event_id, alts, chosen):
        best_cost = alts[0][1]
        for i, (r, c) in enumerate(alts):
            self.alt_rows.append({
                "scenario_id": self.scenario["id"], "reroute_event_id": event_id,
                "alt_index": i, "selected": int(r == chosen),
                "predicted_time_s": round(c, 2),
                "detour_ratio": round(c / best_cost, 3) if best_cost > 0 else "",
                "length_m": round(self.length(r), 1),
                "overlap_with_fastest": round(self.overlap(r, alts[0][0]), 3),
                "edge_count": len(r), "edges": " ".join(r),
            })

    # ---------- hooks used by the patched baseline ----------
    def initial_route(self):
        self._load()
        costs = self.costs(set())
        outgoing, incoming = self.ns["get_endpoint_edges"]()
        best, best_cost = None, INF
        for s in outgoing:
            for d in incoming:
                p = self.search(s, d, costs, set())
                if p:
                    c = self.route_cost(p, costs)
                    if c < best_cost:
                        best, best_cost = p, c
        if best is None:
            raise RuntimeError("Model C: no initial route found")
        self.initial_edges = self.current_route = list(best)
        alts = self.alternatives(best[0], best[-1], costs, set(), best, best_cost, 1.0)
        self.log_alternatives(0, alts, best)
        print(f"Model C initial route: {len(best)} edges, predicted {best_cost:.1f}s, "
              f"{len(alts)} candidate(s)")
        return types.SimpleNamespace(edges=best, travelTime=best_cost)

    def step(self, timestamp, closed_edges, edge_data, scenario_id,
             route_history, scenario_events, reroute_state):
        T, cfg = self.T, self.cfg
        if timestamp - self.last_check < cfg["check_interval_s"] - 1e-9:
            return
        if self.amb not in T.vehicle.getIDList():
            return
        road = T.vehicle.getRoadID(self.amb)
        route = list(T.vehicle.getRoute(self.amb))
        if not road or road.startswith(":") or road not in route:
            return                                   # inside a junction: try next step
        self.last_check = timestamp

        closed = set(closed_edges)
        near = int(T.vehicle.getRouteIndex(self.amb))
        idx = min((i for i, e in enumerate(route) if e == road), key=lambda i: abs(i - near))
        remaining, dst = route[idx:], route[-1]
        try:
            lane_len = float(T.lane.getLength(T.vehicle.getLaneID(self.amb)))
            frac = max(0.0, (lane_len - float(T.vehicle.getLanePosition(self.amb))) / lane_len)
        except Exception:
            frac = 1.0

        costs = self.costs(closed)
        blocked = [e for e in remaining[1:] if e in closed]
        cur_cost = INF if blocked else self.route_cost(remaining, costs, frac)
        best = self.search(road, dst, costs, closed)

        if blocked:
            reason, trig, thr = "closure", blocked[0], "forced"
            if best is None:
                if blocked[0] not in self.failed_blocked:
                    self.failed_blocked.add(blocked[0])
                    reroute_state["failed_reroutes"] += 1
                    self._record(timestamp, road, remaining, reason, trig, thr, [], None,
                                 INF, "no_valid_alternative", scenario_events)
                return
        else:
            if best is None or best == remaining:
                return
            new_cost = self.route_cost(best, costs, frac)
            saving = (cur_cost - new_cost) / cur_cost if cur_cost > 0 else 0.0
            if saving <= cfg["reroute_threshold"]:
                return
            if timestamp - self.last_reroute < cfg["min_seconds_between_reroutes"]:
                return
            reason, trig, thr = "threshold_exceeded", round(saving, 4), cfg["reroute_threshold"]

        new_cost = self.route_cost(best, costs, frac)
        alts = self.alternatives(road, dst, costs, closed, best, new_cost, frac)
        chosen, chosen_cost = min(alts, key=lambda rc: rc[1])
        try:
            T.vehicle.setRoute(self.amb, chosen)
            if list(T.vehicle.getRoute(self.amb)) == route:
                raise RuntimeError("route unchanged")
        except Exception as exc:
            reroute_state["failed_reroutes"] += 1
            self._record(timestamp, road, remaining, reason, trig, thr, alts, chosen,
                         cur_cost, f"setRoute_failed: {exc}", scenario_events)
            return

        reroute_state["route_version"] += 1
        reroute_state["successful_reroutes"] += 1
        reroute_state["last_movement_time"] = float(timestamp)
        self.last_reroute = timestamp
        self.current_route = list(T.vehicle.getRoute(self.amb))
        self.ns["append_route_history"](
            route_history=route_history, scenario_id=scenario_id,
            route_edges=self.current_route, route_version=reroute_state["route_version"],
            route_start_time=timestamp, route_reason=f"model_c_{reason}", trigger_edge=road)
        self._record(timestamp, road, remaining, reason, trig, thr, alts, chosen,
                     cur_cost, "success", scenario_events, chosen_cost)
        print(f"Model C reroute at {timestamp:.0f}s ({reason}): "
              f"{cur_cost:.0f}s -> {chosen_cost:.0f}s")

    def _record(self, t, road, remaining, reason, trig, thr, alts, chosen,
                cur_cost, result, scenario_events, predicted=None):
        eid = len(self.events) + 1
        self.events.append({
            "scenario_id": self.scenario["id"], "route_id": self.cfg["route_id"],
            "reroute_event_id": eid, "simulation_time_s": t, "current_edge": road,
            "current_route": " ".join(remaining), "trigger_reason": reason,
            "trigger_value": trig, "threshold": thr,
            "number_of_alternatives": len(alts),
            "selected_alternative": (next((i for i, (r, _) in enumerate(alts) if r == chosen), "")
                                     if chosen else ""),
            "selected_route": " ".join(chosen) if chosen else "",
            "current_route_estimate_s": "" if cur_cost == INF else round(cur_cost, 2),
            "predicted_time_s": "" if predicted is None else round(predicted, 2),
            "result": result, "actual_remaining_time_s": "", "prediction_error_s": "",
        })
        if alts:
            self.log_alternatives(eid, alts, chosen)
        scenario_events.append({"time_s": t, "event": "model_c_reroute", "trigger_edge": road,
                                "old_route_edges": " ".join(remaining),
                                "new_route_edges": " ".join(chosen) if chosen else "",
                                "result": result})

    def finish(self, scenario, summary):
        ns = self.ns
        out = ns["OUTPUT"] / f"{scenario['id']}_{scenario['name']}"
        arrival = summary.get("ambulance_arrival_s")
        for ev in self.events:
            if arrival not in ("", None) and ev["result"] == "success":
                ev["actual_remaining_time_s"] = round(float(arrival) - float(ev["simulation_time_s"]), 2)
                ev["prediction_error_s"] = round(
                    ev["actual_remaining_time_s"] - float(ev["predicted_time_s"]), 2)
        write_rows(out / "model_c_reroute_events.csv", self.events or [{"scenario_id": scenario["id"]}])
        write_rows(out / "model_c_alternatives_log.csv", self.alt_rows or [{"scenario_id": scenario["id"]}])

        base_tt = ""
        bfile = (ns["BASE"] / "outputs" / "final" / "route_2" /
                 f"{scenario['id']}_{scenario['name']}" / "scenario_summary.csv")
        if bfile.exists():
            with bfile.open(encoding="utf-8") as f:
                base_tt = next(csv.DictReader(f)).get("ambulance_travel_time_s", "")
        tt = summary.get("ambulance_travel_time_s", "")
        saved = pct = ""
        if base_tt not in ("", None) and tt not in ("", None):
            saved = round(float(base_tt) - float(tt), 2)
            pct = round(saved / float(base_tt) * 100, 2) if float(base_tt) else ""
        self.results.append({
            "scenario": scenario["id"], "route_id": self.cfg["route_id"],
            "initial_route": " ".join(self.initial_edges),
            "final_route": " ".join(self.current_route),
            "completed": summary.get("ambulance_completed"),
            "travel_time_s": tt, "waiting_time_s": summary.get("ambulance_waiting_time_s", ""),
            "distance_m": summary.get("ambulance_distance_m", ""),
            "route_updates": summary.get("route_update_count", 0),
            "route_update_failures": summary.get("route_update_failures", 0),
            "collisions": summary.get("ambulance_collision_count", 0),
            "baseline_travel_time_s": base_tt, "time_saved_s": saved, "improvement_pct": pct,
        })
        write_rows(ns["OUTPUT"] / "model_c_main_results.csv", self.results)


PATCHES = [
    # 1) initial route chosen by Model C
    ("best_route = find_initial_route()", "best_route = MC.initial_route()"),
    # 2) dynamic rerouting check on every simulation step, in ALL scenarios
    ("            sample_edge_metrics_if_due(timestamp)\n\n            if timestamp - last_progress_time",
     "            MC.step(timestamp=timestamp, closed_edges=closed_edges, edge_data=edge_data,\n"
     "                    scenario_id=scenario_id, route_history=route_history,\n"
     "                    scenario_events=scenario_events, reroute_state=reroute_state)\n\n"
     "            sample_edge_metrics_if_due(timestamp)\n\n            if timestamp - last_progress_time"),
]


def load_baseline(cfg, sumo_path):
    src = BASE_FILE.read_text(encoding="utf-8")
    for old, new in PATCHES:
        if src.count(old) != 1:
            raise RuntimeError(f"Patch anchor not found exactly once:\n{old[:70]}...")
        src = src.replace(old, new)
    ns = {"__name__": "scenario0_model_c", "__file__": str(BASE_FILE)}
    mc = ModelC(ns, cfg)
    ns["MC"] = mc
    exec(compile(src, str(BASE_FILE), "exec"), ns)

    if sumo_path:
        ns["SUMO"] = sumo_path
    ns["try_reactive_ambulance_reroute"] = lambda **kw: None      # baseline policy off

    # Disruption locations must stay exactly the Scenario 0 ones.
    conf = json.loads((ns["BASE"] / "outputs" / "final" / "route_2" /
                       "pair_configuration.json").read_text(encoding="utf-8"))
    fixed = {k: list(conf[k]) for k in
             ("route_closures", "nearby_closures", "all_closures")}
    fixed["congestion_edges"] = list(conf["congestion_edges"])
    ns["select_disruption_roads"] = lambda *a, **k: fixed

    original = ns["run_scenario"]

    def wrapped(scenario, selections, edge_data, *a, **k):
        mc.begin(scenario)
        summary = original(scenario, selections, edge_data, *a, **k)
        mc.finish(scenario, summary)
        return summary

    ns["run_scenario"] = wrapped
    return ns


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--sumo", default="")
    known, rest = ap.parse_known_args()
    if "--output-tag" not in rest:
        rest += ["--output-tag", "model_c_route_2"]
    sys.argv = [sys.argv[0]] + rest
    cfg = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    ns = load_baseline(cfg, known.sumo)
    ns["main"]()


if __name__ == "__main__":
    main()