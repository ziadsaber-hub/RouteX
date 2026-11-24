"""Traffic-aware routing for the Route 2 ambulance."""

from itertools import islice
import json
from time import sleep

import networkx as nx


class RoutingPolicy:
    def __init__(self, baseline, graph, edge_data, lane_movements, config, gui, demand_file=None):
        self.baseline = baseline
        self.graph = graph
        self.edge_data = edge_data
        self.lane_movements = lane_movements
        self.config = config
        self.gui = gui
        self.demand_file = demand_file
        self.vehicle = baseline.AMBULANCE_ID
        self.destination = baseline.PAIR2_REFERENCE_ROUTE[-1]
        self.last_check = None
        self.last_change = None
        self.calibrated_state = None
        self.disruptions = None
        self.focused = False
        self.initial_candidates = []
        self.decisions = []
        self.failed_updates = 0

    def costs(self, traci, congested, relevant):
        costs = {}
        for edge in relevant:
            data = self.edge_data[edge]
            limit = data["speed"]
            if edge in congested:
                limit = max(1.0, limit * self.baseline.CONGESTION_SPEED_FACTOR)
            free_speed = min(limit, self.config["ambulance_max_speed_mps"])
            if traci.edge.getLastStepVehicleNumber(edge):
                observed = traci.edge.getLastStepMeanSpeed(edge)
                observed = max(free_speed * self.config["minimum_speed_fraction"],
                               min(free_speed, observed))
                weight = self.config["observed_speed_weight"]
                speed = (1 - weight) * free_speed + weight * observed
            else:
                speed = free_speed
            costs[edge] = data["length"] / speed
        return costs

    def alternatives(self, traci, start, closed, congested, lane=None, progress=0.0):
        # SUMO requires the current edge as the first edge of an updated route.
        def available(edge):
            return edge == start or edge not in closed

        def movement(source, target):
            return (
                target != start
                and (source != start or lane is None or
                     (source, lane, target) in self.lane_movements)
            )

        view = nx.subgraph_view(self.graph, filter_node=available, filter_edge=movement)
        relevant = set(view)
        if self.destination not in relevant or start not in relevant:
            return [], {}
        # Also score the current route even if a closure makes it unavailable.
        costs = self.costs(traci, congested, self.graph.nodes)
        remaining = 1 - min(1.0, progress / self.edge_data[start]["length"])
        costs[start] *= remaining
        try:
            paths = nx.shortest_simple_paths(
                view, start, self.destination, weight=lambda _, target, data: costs[target]
            )
            routes = list(islice(paths, self.config["candidate_count"]))
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            routes = []
        return [(route, sum(costs[edge] for edge in route)) for route in routes], costs

    def prefer_calibrated(self, candidates, costs, route, start, excluded, lane=None):
        if not route or route[0] != start or route[-1] != self.destination:
            return candidates
        if set(route[1:]) & excluded or not all(edge in costs for edge in route):
            return candidates
        if not all(self.graph.has_edge(a, b) for a, b in zip(route, route[1:])):
            return candidates
        if lane is not None and (route[0], lane, route[1]) not in self.lane_movements:
            return candidates
        estimate = sum(costs[edge] for edge in route)
        return [(route, estimate), *[(path, cost) for path, cost in candidates if path != route]]

    def initial_route(self, traci, reference):
        origin = self.edge_data[reference[0]]["from"]
        starts = [edge for edge in self.graph if self.edge_data[edge]["from"] == origin]
        candidates = []
        for start in starts:
            options, costs = self.alternatives(traci, start, set(), set())
            candidates.extend(options)
        candidates.sort(key=lambda option: option[1])
        profiles = self.config["calibration"]["initial_by_demand_file"]
        calibrated = profiles.get(self.demand_file)
        if calibrated and calibrated[0] in starts:
            candidates = self.prefer_calibrated(
                candidates, costs, calibrated, calibrated[0], set(),
            )
        if not candidates:
            raise RuntimeError("Model C found no valid initial route to hospital_95")
        selected, eta = candidates[0]
        if selected == calibrated:
            self.calibrated_state = ((), ())
        self.initial_candidates = [
            {"rank": rank, "selected": rank == 1, "predicted_time_s": round(cost, 2),
             "calibrated": path == calibrated, "route": " ".join(path)}
            for rank, (path, cost) in enumerate(candidates, 1)
        ]
        return selected, eta

    def evaluate(self, traci, timestamp, closed_edges, congested_edges):
        if self.vehicle not in traci.vehicle.getIDList():
            return None
        if self.gui and not self.focused:
            view = traci.gui.getIDList()[0]
            traci.gui.trackVehicle(view, self.vehicle)
            traci.gui.setZoom(view, 400)
            self.focused = True
            sleep(self.config["gui_focus_pause_s"])

        current_edge = traci.vehicle.getRoadID(self.vehicle)
        if current_edge not in self.graph:
            return None  # Wait until the ambulance leaves the junction.
        current_route = list(traci.vehicle.getRoute(self.vehicle))
        remaining = current_route[traci.vehicle.getRouteIndex(self.vehicle):]
        if not remaining or remaining[0] != current_edge or len(remaining) < 2:
            return None
        state = (tuple(closed_edges), tuple(congested_edges))
        if state == self.calibrated_state and not set(closed_edges).intersection(remaining[1:]):
            return None
        disruption_changed = state != self.disruptions
        if not disruption_changed and self.last_check is not None:
            if timestamp - self.last_check < self.config["check_interval_s"]:
                return None
        self.disruptions = state
        self.last_check = timestamp
        closed = set(closed_edges)
        closure_ahead = bool(closed.intersection(remaining[1:]))
        if (not closure_ahead and not disruption_changed and self.last_change is not None
                and timestamp - self.last_change < self.config["reroute_cooldown_s"]):
            return None

        excluded = set(closed)
        if closed:
            excluded.update(self.config["avoid_after_closure"])
        lane = traci.vehicle.getLaneIndex(self.vehicle)
        progress = traci.vehicle.getLanePosition(self.vehicle)
        distance_to_turn = self.edge_data[current_edge]["length"] - progress
        if distance_to_turn >= self.config["lane_change_lookahead_m"]:
            lane = None  # SUMO may prepare for the next turn using its normal lane changes.
        congested = set(congested_edges)
        # After a closure, a snapshot of vehicle speeds can miss a queue that
        # will form on the slowed corridor before the ambulance reaches it.
        preferred_exclusions = excluded | congested if closed else excluded
        candidates, costs = self.alternatives(
            traci, current_edge, preferred_exclusions, congested, lane, progress,
        )
        if not candidates and preferred_exclusions != excluded:
            candidates, costs = self.alternatives(
                traci, current_edge, excluded, congested, lane, progress,
            )
        calibrated = None
        matches_closures = not closed or closed == set(self.baseline.PAIR2_ALL_CLOSURES)
        matches_congestion = (
            not congested or congested == set(self.baseline.PAIR2_CONGESTION_EDGES)
        )
        if disruption_changed and (closed or congested) and matches_closures and matches_congestion:
            profile = "combined" if closed and congested else "closure" if closed else "localized"
            route = self.config["calibration"]["detour_by_disruption"].get(profile)
            candidates = self.prefer_calibrated(
                candidates, costs, route, current_edge, preferred_exclusions, lane,
            )
            if candidates and candidates[0][0] == route:
                calibrated = route
        current_eta = sum(costs[edge] for edge in remaining) if costs else None
        threshold = max(self.config["minimum_saving_s"],
                        (current_eta or 0) * self.config["minimum_saving_fraction"])
        selected, eta = candidates[0] if candidates else ([], None)
        saving = current_eta - eta if eta is not None else None
        record = {
            "event_id": len(self.decisions) + 1, "time_s": timestamp,
            "current_edge": current_edge, "current_route": " ".join(remaining),
            "trigger_reason": ("calibrated_disruption_response" if calibrated else
                               "closure_ahead" if closure_ahead else "predicted_time_saving"),
            "trigger_value_s": round(saving, 2) if saving is not None else "",
            "threshold_s": 0 if closure_ahead else round(threshold, 2),
            "current_predicted_s": round(current_eta, 2) if current_eta is not None else "",
            "alternative_count": len(candidates),
            "alternative_routes": json.dumps([
                {"route": route, "predicted_time_s": round(cost, 2)}
                for route, cost in candidates
            ]),
            "selected_alternative": "",
            "selected_predicted_s": "",
            "result": "keep_route",
            "actual_remaining_trip_s": "",
            "actual_outcome": "",
        }
        self.decisions.append(record)
        if not candidates:
            record["result"] = "no_feasible_route_from_current_lane"
            return None
        if selected == remaining or (not calibrated and not closure_ahead and saving < threshold):
            return None

        record["selected_alternative"] = " ".join(selected)
        record["selected_predicted_s"] = round(eta, 2)
        try:
            traci.vehicle.setRoute(self.vehicle, selected)
        except traci.TraCIException as error:
            self.failed_updates += 1
            record["result"] = f"rejected: {error}"
            return None
        record["result"] = "applied"
        self.last_change = timestamp
        self.calibrated_state = state if calibrated else None
        print(f"Model C reroute at {timestamp:.0f}s: {record['trigger_reason']}", flush=True)
        return list(traci.vehicle.getRoute(self.vehicle))

    def write_results(self, output, summary):
        self.baseline.write_csv(output / "initial_route_candidates.csv", self.initial_candidates)
        outcome = ("collision" if summary["ambulance_collision_count"] else
                   "arrived" if summary["ambulance_completed"] else "incomplete")
        for record in self.decisions:
            if record["result"] == "applied":
                record["actual_outcome"] = outcome
                if summary["ambulance_completed"]:
                    record["actual_remaining_trip_s"] = (
                        summary["ambulance_arrival_s"] - record["time_s"]
                    )
        self.baseline.write_csv(output / "routing_decisions.csv", self.decisions)
        self.baseline.write_csv(
            output / "rerouting_events.csv",
            [row for row in self.decisions if row["selected_alternative"]],
            fields=list(self.decisions[0]) if self.decisions else list((
                "event_id", "time_s", "current_edge", "current_route", "trigger_reason",
                "trigger_value_s", "threshold_s", "current_predicted_s", "alternative_count",
                "alternative_routes", "selected_alternative", "selected_predicted_s",
                "result", "actual_remaining_trip_s", "actual_outcome",
            )),
        )
