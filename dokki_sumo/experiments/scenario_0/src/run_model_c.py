# Model C v3 launcher, both routes. Run from the src folder:
#   python run_model_c.py --route 1
#   python run_model_c.py --route 2
# v1 algorithm (penalty alternatives, live rerouting, forced closure reroute) plus:
#  1) Travel-time estimate: blend of free-flow speed and observed mean speed (from the
#     Route 2 Model C variant) instead of SUMO's raw edge travel time (less noise).
#  2) Reroute threshold: saving > max(16 s, 12% of the current estimate), confirmed on
#     2 consecutive checks.
#  3) Detour guard: no optional reroute that makes the rest of the trip > 1.25x longer,
#     or the whole trip > 1.30x the shortest O-D path. Closures still force a reroute.
#  4) Lane check: within 70 m of the junction, the next turn must be reachable from the
#     current lane.
#  5) Safety: an unfinished overtake lane-change request is cancelled before the edge end
#     and the normal (safe) lane-change mode is restored.
#  6) v3: queues count - on edges with halted vehicles the cost is at least SUMO's own
#     edge travel time (capped at 10x free flow), and a stuck ambulance (waiting >= 15 s)
#     may reroute at once from any legal lane.
# No route is hard-coded or calibrated per scenario.
import argparse
import importlib
import sys

import networkx as nx

ap = argparse.ArgumentParser(add_help=False)
ap.add_argument("--route", choices=["1", "2"], required=True)
known, rest = ap.parse_known_args()
if "--output-tag" not in rest:
    rest += ["--output-tag", f"model_c_v3_route_{known.route}"]
import xml.etree.ElementTree as ET
sys.argv = [sys.argv[0]] + rest

m = importlib.import_module("model_c_runner_route1" if known.route == "1" else "model_c_runner")
INF = float("inf")

V2 = {"confirm_checks": 2, "max_step_length_ratio": 1.25, "max_trip_length_ratio": 1.30,
      "minimum_saving_s": 16.0, "minimum_saving_fraction": 0.12,
      "observed_speed_weight": 0.65, "minimum_speed_fraction": 0.2,
      "ambulance_max_speed_mps": 16.67, "lane_change_lookahead_m": 70.0,
      "stuck_waiting_s": 15.0, "queue_cost_cap_factor": 10.0}

if known.route == "2":
    OLD = ('        ambulance_route = list(PAIR2_REFERENCE_ROUTE)\n'
           '        estimated_time = sum(\n'
           '            float(edge_data[edge_id]["length"]) / max(float(edge_data[edge_id]["speed"]), 0.1)\n'
           '            for edge_id in ambulance_route\n'
           '        )\n')
    NEW = ('        _mc_route = MC.initial_route()\n'
           '        ambulance_route = list(_mc_route.edges)\n'
           '        estimated_time = float(_mc_route.travelTime)\n')
    m.PATCHES[0] = (OLD, NEW)

MC = m.ModelC
_begin, _initial, _search = MC.begin, MC.initial_route, MC.search


def costs(self, closed):
    T, cfg = self.T, {**V2, **self.cfg}
    out = {}
    for e, d in self.edge_data.items():
        if e in closed:
            continue
        limit = float(d["speed"])
        try:
            limit = float(T.lane.getMaxSpeed(e + "_0"))      # includes the S4/S6 speed cut
        except Exception:
            pass
        free = max(0.1, min(limit, cfg["ambulance_max_speed_mps"]))
        speed = free
        try:
            if int(T.edge.getLastStepVehicleNumber(e)) > 0:
                obs = float(T.edge.getLastStepMeanSpeed(e))
                obs = max(free * cfg["minimum_speed_fraction"], min(free, obs))
                w = cfg["observed_speed_weight"]
                speed = (1 - w) * free + w * obs
        except Exception:
            pass
        cost = d["length"] / speed
        try:
            if int(T.edge.getLastStepHaltingNumber(e)) > 0:      # a queue: trust SUMO's delay
                tt = float(T.edge.getTraveltime(e))
                if tt > 0:
                    cost = max(cost, min(tt, cfg["queue_cost_cap_factor"] * d["length"] / free))
        except Exception:
            pass
        out[e] = max(cost, cfg["min_edge_travel_time_s"])
    return out


def search(self, start, dst, costs, closed, pen=None):
    lane_rule = getattr(self, "lane_rule", None)
    if not lane_rule or lane_rule[0] != start:
        return _search(self, start, dst, costs, closed, pen)
    allowed = lane_rule[1]
    pen = pen or {}

    def w(u, v, d):
        if v in closed or v not in costs:
            return None
        if u == start and v not in allowed:
            return None
        return costs[v] * pen.get(v, 1.0)
    try:
        return nx.dijkstra_path(self.G, start, dst, weight=w)
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return None


def lane_moves(self):
    if getattr(self, "_lane_moves", None) is None:
        moves = set()
        for c in ET.parse(self.ns["NETWORK"]).getroot().findall("connection"):
            f, t, fl = c.get("from"), c.get("to"), c.get("fromLane")
            if f and t and fl is not None and not f.startswith(":") and not t.startswith(":"):
                moves.add((f, int(fl), t))
        MC._lane_moves = moves
    return MC._lane_moves


def begin(self, scenario):
    _begin(self, scenario)
    self.streak, self.streak_route, self.travelled = 0, None, []
    self.shortest_len = None
    self.lane_rule = None


def initial_route(self):
    r = _initial(self)
    ends = (r.edges[0], r.edges[-1])
    try:
        sp = nx.dijkstra_path(self.G, ends[0], ends[1],
                              weight=lambda u, v, d: self.edge_data[v]["length"] if v in self.edge_data else None)
        self.shortest_len = self.length(sp)   # includes the first edge
    except Exception:
        self.shortest_len = self.length(r.edges)
    return r


def step(self, timestamp, closed_edges, edge_data, scenario_id,
         route_history, scenario_events, reroute_state):
    T, cfg = self.T, {**V2, **self.cfg}
    if self.amb not in T.vehicle.getIDList():
        return
    road = T.vehicle.getRoadID(self.amb)
    if road and not road.startswith(":") and (not self.travelled or self.travelled[-1] != road):
        self.travelled.append(road)          # every edge actually driven
    if timestamp - self.last_check < cfg["check_interval_s"] - 1e-9:
        return
    route = list(T.vehicle.getRoute(self.amb))
    if not road or road.startswith(":") or road not in route:
        return
    self.last_check = timestamp

    closed = set(closed_edges)
    if closed and cfg.get("avoid_known_blockage_edges_after_closure", True):
        closed |= {e for e in self.ns.get("AMBULANCE_KNOWN_BLOCKAGE_EDGES", ()) if e != road}
    near = int(T.vehicle.getRouteIndex(self.amb))
    idx = min((i for i, e in enumerate(route) if e == road), key=lambda i: abs(i - near))
    remaining, dst = route[idx:], route[-1]
    try:
        lane_len = float(T.lane.getLength(T.vehicle.getLaneID(self.amb)))
        frac = max(0.0, (lane_len - float(T.vehicle.getLanePosition(self.amb))) / lane_len)
    except Exception:
        frac = 1.0

    try:
        stuck = float(T.vehicle.getWaitingTime(self.amb)) >= cfg["stuck_waiting_s"]
    except Exception:
        stuck = False
    self.lane_rule = None
    try:
        left = float(T.lane.getLength(T.vehicle.getLaneID(self.amb))) - float(T.vehicle.getLanePosition(self.amb))
        if left < cfg["lane_change_lookahead_m"] and not stuck:
            li = int(T.vehicle.getLaneIndex(self.amb))
            self.lane_rule = (road, {t for (f, l, t) in lane_moves(self) if f == road and l == li})
    except Exception:
        self.lane_rule = None

    costs = self.costs(closed)
    blocked = [e for e in remaining[1:] if e in closed]
    cur_cost = INF if blocked else self.route_cost(remaining, costs, frac)
    best = self.search(road, dst, costs, closed)

    if blocked:
        self.streak, self.streak_route = 0, None
        reason, trig, thr = "closure", blocked[0], "forced"
        if best is None and self.lane_rule:
            self.lane_rule = None                      # closure: any legal lane is better than none
            best = self.search(road, dst, costs, closed)
        if best is None:
            if blocked[0] not in self.failed_blocked:
                self.failed_blocked.add(blocked[0])
                reroute_state["failed_reroutes"] += 1
                self._record(timestamp, road, remaining, reason, trig, thr, [], None,
                             INF, "no_valid_alternative", scenario_events)
            return
    else:
        if best is None or best == remaining:
            self.streak, self.streak_route = 0, None
            return
        new_cost = self.route_cost(best, costs, frac)
        saving = (cur_cost - new_cost) / cur_cost if cur_cost > 0 else 0.0
        need = max(cfg["minimum_saving_fraction"], cfg["minimum_saving_s"] / cur_cost) if cur_cost > 0 else 1.0
        if saving <= need:
            self.streak, self.streak_route = 0, None
            return
        # detour guard
        done_len = self.length(self.travelled[:-1])
        if (self.length(best) > cfg["max_step_length_ratio"] * self.length(remaining)
                or (self.shortest_len and done_len + self.length(best)
                    > cfg["max_trip_length_ratio"] * self.shortest_len)):
            self.streak, self.streak_route = 0, None
            return
        # persistence: the same better route must win on consecutive checks
        self.streak = self.streak + 1 if self.streak_route == best else 1
        self.streak_route = best
        if self.streak < (1 if stuck else cfg["confirm_checks"]):
            return
        if timestamp - self.last_reroute < cfg["min_seconds_between_reroutes"]:
            return
        reason, trig, thr = "threshold_exceeded", round(saving, 4), round(need, 4)

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

    self.streak, self.streak_route = 0, None
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
    print(f"Model C reroute at {timestamp:.0f}s ({reason}): {cur_cost:.0f}s -> {chosen_cost:.0f}s")


MC.begin, MC.initial_route, MC.step, MC.costs, MC.search = begin, initial_route, step, costs, search

_load = m.load_baseline


def load_baseline(cfg, sumo_path):
    ns = _load(cfg, sumo_path)
    orig = ns.get("manage_ambulance_overtake")
    if orig is None:
        return ns

    def safe_overtake(timestamp, overtake_state, scenario_events, lane_transition_map):
        T, aid = ns["traci"], ns["AMBULANCE_ID"]
        busy = overtake_state.get("pending") or overtake_state.get("active") or overtake_state.get("returning")
        if busy and aid in T.vehicle.getIDList():
            try:
                lane = T.vehicle.getLaneID(aid)
                left = float(T.lane.getLength(lane)) - float(T.vehicle.getLanePosition(aid))
                speed = float(T.vehicle.getSpeed(aid))
                if lane.startswith(":") or left <= max(2.0 * speed, 10.0):
                    # Cancel the unfinished request: keep the current lane, back to safe mode.
                    T.vehicle.setLaneChangeMode(aid, int(overtake_state.get(
                        "original_lane_change_mode", ns["DEFAULT_LANE_CHANGE_MODE"])))
                    if not lane.startswith(":"):
                        T.vehicle.changeLane(aid, int(T.vehicle.getLaneIndex(aid)), 1.0)
                    scenario_events.append({"time_s": timestamp, "event": "ambulance_overtake_cancelled",
                                            "edge_id": T.vehicle.getRoadID(aid), "remaining_distance_m": round(left, 2),
                                            "result": "request_cancelled_before_edge_end"})
                    overtake_state.clear()
                    overtake_state["retry_after"] = float(timestamp) + 3.0
                    return
            except Exception:
                pass
        return orig(timestamp, overtake_state, scenario_events, lane_transition_map)

    ns["manage_ambulance_overtake"] = safe_overtake
    return ns


m.load_baseline = load_baseline

if __name__ == "__main__":
    m.main()
