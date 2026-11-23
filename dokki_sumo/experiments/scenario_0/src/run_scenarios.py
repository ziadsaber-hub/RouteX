
from __future__ import annotations

import argparse
import csv
import math
import statistics
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import networkx as nx
import traci as traci_socket
import sumolib

try:
    import libsumo as traci_libsumo
except (ImportError, OSError):
    traci_libsumo = None

# The default backend remains TraCI. libsumo can be selected explicitly
# when an exactly matching SUMO/libsumo installation is available.
traci = traci_socket


# ============================================================
# PATHS AND CONFIGURATION
# ============================================================

ROOT = Path(__file__).resolve().parents[4]

NETWORK = ROOT / "dokki_sumo/network/dokki.net.xml"
TRAFFIC_LIGHTS = ROOT / "dokki_sumo/simulation/traffic_lights.add.xml"
AMBULANCE = ROOT / "dokki_sumo/experiments/scenario_0/config/ambulance.add.xml"

BASE = ROOT / "dokki_sumo/experiments/scenario_0"
DEMAND_DIR = BASE / "demand"
OUTPUT = BASE / "outputs"

SUMO = r"C:\Program Files (x86)\SUMO\bin\sumo.exe"

ORIGIN = "181"
DESTINATION = "hospital_168"
DEPARTURE_TIME = 600
AMBULANCE_ID = "ambulance_1"
AMBULANCE_TYPE = "ambulance"

SEED = 1
STEP_LENGTH = 1
MAX_SIM_TIME = 7200

# Disruptions activate after the initial ambulance route is calculated.
DISRUPTION_DELAY = 10.0

# Keep the 25% speed factor and extend the affected area into one continuous
# congested corridor. For the current 22-edge route, fractions 0.76, 0.81,
# 0.86 and 0.90 select route edges 16-19.
CONGESTION_SPEED_FACTOR = 0.25
CONGESTION_ROUTE_FRACTIONS = (0.76, 0.81, 0.86, 0.90)

# Reactive baseline recovery: SUMO may stop the ambulance before the
# immediately next edge when a later edge on its fixed route is closed.
REACTIVE_CLOSURE_LOOKAHEAD_M = 125.0
REACTIVE_STOP_SPEED_MPS = 0.1
REACTIVE_STOP_DURATION_S = 3.0
POST_REROUTE_STALL_TIMEOUT_S = 300.0

# Background vehicles receive the same closure information and are allowed
# to avoid newly closed roads. The ambulance itself is deliberately excluded
# and retains the static-route/reactive-recovery baseline policy.
BACKGROUND_REROUTE_INTERVAL_S = 5.0

# Detailed ambulance positions remain sampled every simulation second.
# Network-wide edge statistics are sampled less frequently to reduce Python
# TraCI calls without changing SUMO's simulation step or vehicle behaviour.
EDGE_METRIC_SAMPLE_INTERVAL_S = 5.0

# Print progress every simulated 300 seconds.
PROGRESS_INTERVAL_S = 300.0

# Number of nearby alternative roads to close, subject to connection-level
# reachability and dead-end checks.
NEARBY_CLOSURES = 2

VIRTUAL_STOP = {
    "tls_id": "172",
    "edge_id": "e39to172k0",
    "stop_pos": 90.0,
    "controlled_link_indices": (16, 17, 18, 19),
    "stop_duration": 1_000_000.0,
}

OUTPUT.mkdir(parents=True, exist_ok=True)


SCENARIOS = [
    {
        "id": "S1",
        "name": "normal",
        "demand": "normal.rou.xml",
        "disruption": "none",
    },
    {
        "id": "S2",
        "name": "moderate",
        "demand": "moderate.rou.xml",
        "disruption": "none",
    },
    {
        "id": "S3",
        "name": "heavy",
        "demand": "heavy.rou.xml",
        "disruption": "none",
    },
    {
        "id": "S4",
        "name": "localized_congestion",
        "demand": "moderate.rou.xml",
        "disruption": "congestion",
    },
    {
        "id": "S5",
        "name": "road_closure",
        "demand": "moderate.rou.xml",
        "disruption": "closure",
    },
    {
        "id": "S6",
        "name": "combined",
        "demand": "moderate.rou.xml",
        "disruption": "combined",
    },
]


# ============================================================
# XML / CSV HELPERS
# ============================================================

def write_csv(path: Path, rows: list[dict], fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)

    if fields is None:
        # Event rows may contain different optional columns (e.g. disruption,
        # reroute, and teleport events). Preserve the union of all columns.
        fields = list(dict.fromkeys(
            key for row in rows for key in row.keys()
        ))

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def count_demand_vehicles(route_file: Path) -> int:
    root = ET.parse(route_file).getroot()
    return len(root.findall("vehicle"))


def read_tripinfo(path: Path) -> list[dict]:
    if not path.exists():
        return []

    root = ET.parse(path).getroot()
    rows = []

    for trip in root.findall("tripinfo"):
        rows.append({
            "vehicle_id": trip.get("id"),
            "depart_s": float(trip.get("depart", 0)),
            "arrival_s": float(trip.get("arrival", 0)),
            "duration_s": float(trip.get("duration", 0)),
            "waiting_time_s": float(trip.get("waitingTime", 0)),
            "time_loss_s": float(trip.get("timeLoss", 0)),
            "distance_m": float(trip.get("routeLength", 0)),
            "depart_delay_s": float(trip.get("departDelay", 0)),
        })

    return rows


def mean_or_zero(values):
    return statistics.mean(values) if values else 0.0


# ============================================================
# NETWORK ANALYSIS
# ============================================================

def load_network_graph():
    """
    Directed graph of external roads.
    Used only to verify that the hospital remains reachable
    after the selected closures.
    """
    root = ET.parse(NETWORK).getroot()
    graph = nx.DiGraph()
    edge_data = {}

    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        source = edge.get("from")
        target = edge.get("to")

        if not edge_id or edge_id.startswith(":"):
            continue

        lanes = edge.findall("lane")
        if not lanes:
            continue

        length = float(lanes[0].get("length", 1))
        speed = float(lanes[0].get("speed", 1))
        speed = max(speed, 0.1)

        weight = length / speed

        edge_data[edge_id] = {
            "from": source,
            "to": target,
            "length": length,
            "speed": speed,
        }

        # Keep the cheapest parallel edge for node connectivity.
        if not graph.has_edge(source, target):
            graph.add_edge(source, target, weight=weight, edge_ids=[edge_id])
        else:
            existing = graph[source][target]
            existing["edge_ids"].append(edge_id)
            existing["weight"] = min(existing["weight"], weight)

    return graph, edge_data


def load_edge_transition_graph():
    """
    Build a directed graph whose nodes are external SUMO edges and whose
    arcs are legal external-edge-to-external-edge connections.
    """
    root = ET.parse(NETWORK).getroot()
    transition_graph = nx.DiGraph()

    for connection in root.findall("connection"):
        from_edge = connection.get("from")
        to_edge = connection.get("to")
        if not from_edge or not to_edge:
            continue
        if from_edge.startswith(":") or to_edge.startswith(":"):
            continue
        transition_graph.add_edge(from_edge, to_edge)

    return transition_graph


def closure_set_is_safe(
    transition_graph,
    closed_edges,
    start_edge,
    destination_edge,
    traffic_od_pairs=None,
):
    """
    Validate closures using actual SUMO edge-to-edge connections.

    Conditions:
      1. Ambulance start and destination edges remain open and connected.
      2. Every moderate-demand traffic OD pair remains connected.
      3. No remaining approach edge has all of its legal next edges closed.
    """
    closed = set(closed_edges)

    if start_edge in closed or destination_edge in closed:
        return False

    remaining = transition_graph.copy()
    remaining.remove_nodes_from(
        edge_id for edge_id in closed if remaining.has_node(edge_id)
    )

    if (
        start_edge not in remaining
        or destination_edge not in remaining
        or not nx.has_path(remaining, start_edge, destination_edge)
    ):
        return False

    for approach_edge in remaining.nodes:
        outgoing = list(transition_graph.successors(approach_edge))
        if outgoing and all(next_edge in closed for next_edge in outgoing):
            return False

    if traffic_od_pairs:
        for traffic_start, traffic_destination in traffic_od_pairs:
            if (
                traffic_start in closed
                or traffic_destination in closed
                or traffic_start not in remaining
                or traffic_destination not in remaining
                or not nx.has_path(
                    remaining,
                    traffic_start,
                    traffic_destination,
                )
            ):
                return False

    return True


def load_traffic_od_pairs(route_file):
    """Return start/end edge pairs for all vehicles in a demand route file."""
    root = ET.parse(route_file).getroot()
    pairs = []

    for vehicle in root.findall("vehicle"):
        route_element = vehicle.find("route")
        if route_element is None:
            continue

        edges = route_element.get("edges", "").split()
        if len(edges) >= 2:
            pairs.append((edges[0], edges[-1]))

    return pairs


def route_is_reachable(graph, edge_data, closed_edges):
    test_graph = graph.copy()

    for edge_id in closed_edges:
        data = edge_data.get(edge_id)
        if not data:
            continue

        u, v = data["from"], data["to"]

        if not test_graph.has_edge(u, v):
            continue

        remaining = [
            e for e in test_graph[u][v]["edge_ids"]
            if e not in closed_edges
        ]

        if not remaining:
            test_graph.remove_edge(u, v)

    try:
        return nx.has_path(test_graph, ORIGIN, DESTINATION)
    except nx.NodeNotFound:
        return False


def select_disruption_roads(
    reference_route,
    graph,
    edge_data,
    transition_graph,
    traffic_od_pairs,
):
    """
    Select route closures and nearby closures that preserve actual road
    connections and every background traffic OD pair.

    Congestion remains on the four-edge corridor used by the validated S4
    experiment. Closure candidates are never allowed to overlap that corridor.
    """
    route = list(reference_route)

    if len(route) < 8:
        raise RuntimeError(
            "Reference ambulance route is too short for disruption selection."
        )

    start_edge = route[0]
    destination_edge = route[-1]
    route_set = set(route)

    # Preserve the validated S4 corridor. These edges are slowed, not closed.
    congestion_indices = sorted(set(
        min(len(route) - 2, int(len(route) * fraction))
        for fraction in CONGESTION_ROUTE_FRACTIONS
    ))
    congestion = list(dict.fromkeys(route[index] for index in congestion_indices))
    congestion_set = set(congestion)

    traffic_endpoint_edges = {
        edge_id
        for pair in traffic_od_pairs
        for edge_id in pair
    }
    protected_edges = traffic_endpoint_edges | congestion_set

    # Select two route closures near the original target fractions. If a
    # candidate is a traffic origin/destination, part of the S4 corridor, or
    # would create a dead-end, move to the nearest safe route edge.
    target_indices = sorted(set([
        max(1, int(len(route) * 0.35)),
        min(len(route) - 2, int(len(route) * 0.65)),
    ]))

    route_closures = []
    for target_index in target_indices:
        candidate_indices = sorted(
            range(1, len(route) - 1),
            key=lambda index: (abs(index - target_index), index),
        )

        selected = None
        for index in candidate_indices:
            candidate = route[index]
            if candidate in route_closures or candidate in protected_edges:
                continue

            proposed = route_closures + [candidate]
            if closure_set_is_safe(
                transition_graph,
                proposed,
                start_edge,
                destination_edge,
                traffic_od_pairs,
            ):
                selected = candidate
                break

        if selected is not None:
            route_closures.append(selected)

    if len(route_closures) < 2:
        raise RuntimeError(
            "Could not select two route closures without blocking a traffic "
            "OD pair or creating a connection-level dead end. Inspect the "
            "reference route or adjust the disruption targets."
        )

    closed = list(route_closures)
    accepted_nearby = []

    # Choose nearby closures from the neighbourhood of each route closure.
    # Do not close traffic origin/destination edges or S4 congestion edges.
    for route_closure in route_closures:
        data = edge_data[route_closure]
        endpoints = {data["from"], data["to"]}

        nearby_candidates = [
            candidate
            for candidate, candidate_data in edge_data.items()
            if (
                candidate not in route_set
                and candidate not in closed
                and candidate not in protected_edges
                and candidate in transition_graph
                and (
                    candidate_data["from"] in endpoints
                    or candidate_data["to"] in endpoints
                )
            )
        ]

        for candidate in nearby_candidates:
            proposed = closed + [candidate]
            if closure_set_is_safe(
                transition_graph,
                proposed,
                start_edge,
                destination_edge,
                traffic_od_pairs,
            ):
                closed.append(candidate)
                accepted_nearby.append(candidate)
                break

        if len(accepted_nearby) >= NEARBY_CLOSURES:
            break

    # Fill any remaining nearby-closure slots from the combined neighbourhood
    # while retaining all connection and traffic-OD checks.
    if len(accepted_nearby) < NEARBY_CLOSURES:
        all_candidates = []
        for route_closure in route_closures:
            data = edge_data[route_closure]
            endpoints = {data["from"], data["to"]}
            for candidate, candidate_data in edge_data.items():
                if (
                    candidate not in route_set
                    and candidate not in closed
                    and candidate not in protected_edges
                    and candidate in transition_graph
                    and (
                        candidate_data["from"] in endpoints
                        or candidate_data["to"] in endpoints
                    )
                ):
                    all_candidates.append(candidate)

        for candidate in dict.fromkeys(all_candidates):
            if len(accepted_nearby) >= NEARBY_CLOSURES:
                break
            proposed = closed + [candidate]
            if closure_set_is_safe(
                transition_graph,
                proposed,
                start_edge,
                destination_edge,
                traffic_od_pairs,
            ):
                closed.append(candidate)
                accepted_nearby.append(candidate)

    if not closure_set_is_safe(
        transition_graph,
        closed,
        start_edge,
        destination_edge,
        traffic_od_pairs,
    ):
        raise RuntimeError(
            "Final closure selection failed connection-level or traffic-OD validation."
        )

    if len(accepted_nearby) < NEARBY_CLOSURES:
        print(
            "Warning: fewer than the requested number of nearby closures "
            "could be selected without blocking traffic or creating a dead end."
        )

    # A Model C route can avoid all four slowed roads while still avoiding
    # the closures. Congestion roads remain open in the SUMO baseline.
    avoidance_graph = transition_graph.copy()
    avoidance_graph.remove_nodes_from(
        edge_id
        for edge_id in set(closed + congestion)
        if avoidance_graph.has_node(edge_id)
    )
    if (
        start_edge not in avoidance_graph
        or destination_edge not in avoidance_graph
        or not nx.has_path(avoidance_graph, start_edge, destination_edge)
    ):
        raise RuntimeError(
            "The closure/congestion configuration does not leave a legal "
            "route that avoids the congestion corridor."
        )

    return {
        "route_closures": route_closures,
        "nearby_closures": accepted_nearby,
        "all_closures": closed,
        "congestion_edges": congestion,
    }


# ============================================================
# AMBULANCE ROUTE AND VIRTUAL STOP LINE
# ============================================================

def get_endpoint_edges():
    root = ET.parse(NETWORK).getroot()

    outgoing = []
    incoming = []

    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        source = edge.get("from")
        target = edge.get("to")

        if not edge_id or edge_id.startswith(":"):
            continue

        if source == ORIGIN:
            outgoing.append(edge_id)

        if target == DESTINATION:
            incoming.append(edge_id)

    return outgoing, incoming


def find_initial_route():
    outgoing, incoming = get_endpoint_edges()
    candidates = []

    for start_edge in outgoing:
        for end_edge in incoming:
            try:
                route = traci.simulation.findRoute(
                    start_edge,
                    end_edge,
                    vType=AMBULANCE_TYPE,
                )

                if route.edges and route.travelTime > 0:
                    candidates.append(route)

            except traci.TraCIException:
                continue

    if not candidates:
        raise RuntimeError("No valid ambulance route found.")

    return min(candidates, key=lambda r: r.travelTime)


def update_virtual_stop_line(stopped_vehicles):
    """Stop approaching vehicles only when they can brake safely."""
    state = traci.trafficlight.getRedYellowGreenState(
        VIRTUAL_STOP["tls_id"]
    )

    indices = VIRTUAL_STOP["controlled_link_indices"]
    approach_states = [
        state[i] for i in indices if i < len(state)
    ]

    if not approach_states:
        return

    is_green = all(c in ("G", "g") for c in approach_states)
    live_ids = set(traci.vehicle.getIDList())
    stopped_vehicles.intersection_update(live_ids)

    if is_green:
        for veh_id in tuple(stopped_vehicles):
            if traci.vehicle.getStopState(veh_id) & 1:
                traci.vehicle.resume(veh_id)
            stopped_vehicles.discard(veh_id)
        return

    edge_id = VIRTUAL_STOP["edge_id"]
    stop_pos = VIRTUAL_STOP["stop_pos"]

    # Only inspect vehicles actually on the approach edge rather than
    # iterating over every vehicle in the network on every simulation step.
    for veh_id in traci.edge.getLastStepVehicleIDs(edge_id):
        if veh_id in stopped_vehicles:
            continue

        distance = stop_pos - traci.vehicle.getLanePosition(veh_id)
        if distance <= 0:
            continue

        speed = traci.vehicle.getSpeed(veh_id)
        decel = max(traci.vehicle.getDecel(veh_id), 0.1)
        braking_distance = (speed ** 2) / (2.0 * decel) + 2.0

        # Too close to stop safely: leave it to the original TLS.
        if distance < braking_distance:
            continue

        traci.vehicle.setStop(
            veh_id,
            edge_id,
            pos=stop_pos,
            laneIndex=traci.vehicle.getLaneIndex(veh_id),
            duration=VIRTUAL_STOP["stop_duration"],
        )
        stopped_vehicles.add(veh_id)


# ============================================================
# DISRUPTION CONTROL
# ============================================================

def apply_disruptions(disruption, selections, edge_data):
    """Apply closures and speed reductions using known network edge speeds."""
    closed = []
    congested = []

    if disruption in ("closure", "combined"):
        closed = list(selections["all_closures"])

        # Both passenger traffic and the ambulance are prohibited from
        # entering a closed road. Vehicles already on an edge can leave it.
        for edge_id in closed:
            traci.edge.setDisallowed(
                edge_id,
                ["passenger", "emergency"],
            )

    if disruption in ("congestion", "combined"):
        congested = list(selections["congestion_edges"])

        for edge_id in congested:
            if edge_id not in edge_data:
                raise KeyError(
                    f"Congestion edge {edge_id!r} is missing from network edge data."
                )

            original_speed = float(edge_data[edge_id]["speed"])
            reduced_speed = max(
                1.0,
                original_speed * CONGESTION_SPEED_FACTOR,
            )

            traci.edge.setMaxSpeed(edge_id, reduced_speed)

    return closed, congested


def append_route_history(
    route_history,
    scenario_id,
    route_edges,
    route_version,
    route_start_time,
    route_reason,
    trigger_edge="",
):
    """Append one route version as edge-level rows."""
    for index, edge_id in enumerate(route_edges):
        route_history.append({
            "scenario_id": scenario_id,
            "route_version": route_version,
            "route_start_time_s": route_start_time,
            "route_reason": route_reason,
            "trigger_edge": trigger_edge,
            "route_index": index,
            "edge_id": edge_id,
        })


def reroute_background_traffic_around_closures(closed_edges, unroutable_ids=None):
    """
    Reroute ordinary traffic vehicles whose remaining route contains a
    newly closed edge. The ambulance is deliberately excluded.

    This represents the common traffic response to a known road closure and
    prevents vehicles with obsolete routes from queuing indefinitely at a
    blocked approach. The ambulance baseline remains static until its own
    route becomes infeasible and the stop-triggered recovery runs.
    """
    closed_set = set(closed_edges)
    rerouted = 0
    failed = 0
    if unroutable_ids is None:
        unroutable_ids = set()

    if not closed_set:
        return rerouted, failed

    for vehicle_id in traci.vehicle.getIDList():
        if vehicle_id == AMBULANCE_ID or vehicle_id in unroutable_ids:
            continue

        try:
            road_id = traci.vehicle.getRoadID(vehicle_id)

            # Vehicles already on a closed road are allowed to clear it.
            # Internal-junction vehicles are checked again at the next batch.
            if not road_id or road_id.startswith(":") or road_id in closed_set:
                continue

            route = list(traci.vehicle.getRoute(vehicle_id))
            if len(route) < 2:
                continue

            route_index = int(traci.vehicle.getRouteIndex(vehicle_id))
            if road_id not in route:
                continue

            matching = [
                i for i, edge_id in enumerate(route)
                if edge_id == road_id
            ]
            current_index = min(
                matching,
                key=lambda i: abs(i - route_index),
            )

            if not any(
                edge_id in closed_set
                for edge_id in route[current_index + 1:]
            ):
                continue

            destination_edge = route[-1]
            vehicle_type = traci.vehicle.getTypeID(vehicle_id)

            alternative = traci.simulation.findRoute(
                road_id,
                destination_edge,
                vType=vehicle_type,
            )
            candidate = list(alternative.edges)

            if (
                len(candidate) < 2
                or candidate[0] != road_id
                or any(edge_id in closed_set for edge_id in candidate[1:])
            ):
                failed += 1
                unroutable_ids.add(vehicle_id)
                continue

            if candidate != route[current_index:]:
                traci.vehicle.setRoute(vehicle_id, candidate)
                if not traci.vehicle.isRouteValid(vehicle_id):
                    failed += 1
                    unroutable_ids.add(vehicle_id)
                    continue
                rerouted += 1

        except traci.TraCIException:
            failed += 1
            unroutable_ids.add(vehicle_id)

    return rerouted, failed


def try_reactive_ambulance_reroute(
    closed_edges,
    edge_data,
    scenario_id,
    timestamp,
    route_history,
    scenario_events,
    reroute_state,
):
    """
    Static-route baseline with reactive recovery.

    Keep the original route after closure activation. SUMO can stop the
    ambulance before the immediately blocked edge, even when the closure is
    several route positions ahead. Inspect the remaining route and reroute
    only after the ambulance has been stationary near the first closed edge.
    """
    if not closed_edges or AMBULANCE_ID not in traci.vehicle.getIDList():
        reroute_state["stopped_since"] = None
        return

    closed_set = set(closed_edges)

    try:
        current_route = list(traci.vehicle.getRoute(AMBULANCE_ID))
        route_index = int(traci.vehicle.getRouteIndex(AMBULANCE_ID))
        road_id = traci.vehicle.getRoadID(AMBULANCE_ID)

        if not current_route or route_index < 0 or route_index >= len(current_route):
            reroute_state["stopped_since"] = None
            return

        # Resolve the external route edge occupied by the ambulance.
        if road_id and not road_id.startswith(":") and road_id in current_route:
            matching_indices = [
                index for index, edge_id in enumerate(current_route)
                if edge_id == road_id
            ]
            current_edge_index = min(
                matching_indices,
                key=lambda index: abs(index - route_index),
            )
        elif road_id.startswith(":") and current_route[route_index] in closed_set:
            current_edge_index = max(0, route_index - 1)
        else:
            current_edge_index = route_index

        current_edge_index = min(current_edge_index, len(current_route) - 1)
        current_edge = current_route[current_edge_index]

        # Find the first closed edge anywhere ahead, not just route_index + 1.
        blocked_index = next(
            (
                index
                for index in range(current_edge_index + 1, len(current_route))
                if current_route[index] in closed_set
            ),
            None,
        )

        if blocked_index is None:
            reroute_state["stopped_since"] = None
            return

        blocked_edge = current_route[blocked_index]

        # Distance from current position to the beginning of the first
        # closed edge, following the still-assigned route.
        lane_id = traci.vehicle.getLaneID(AMBULANCE_ID)
        lane_length = float(traci.lane.getLength(lane_id))
        lane_position = float(traci.vehicle.getLanePosition(AMBULANCE_ID))
        distance_to_closure = max(0.0, lane_length - lane_position)

        for edge_id in current_route[current_edge_index + 1 : blocked_index]:
            data = edge_data.get(edge_id)
            if data is not None:
                distance_to_closure += float(data["length"])

        if distance_to_closure > REACTIVE_CLOSURE_LOOKAHEAD_M:
            reroute_state["stopped_since"] = None
            return

        # Do not reroute simply because the closure exists. Wait until SUMO
        # has actually stopped the ambulance near the obstruction.
        speed = float(traci.vehicle.getSpeed(AMBULANCE_ID))
        if speed > REACTIVE_STOP_SPEED_MPS:
            reroute_state["stopped_since"] = None
            return

        if reroute_state["stopped_since"] is None:
            reroute_state["stopped_since"] = float(timestamp)
            return

        stopped_duration = float(timestamp) - float(reroute_state["stopped_since"])
        if stopped_duration < REACTIVE_STOP_DURATION_S:
            return

        if blocked_edge in reroute_state["attempted_edges"]:
            return

        reroute_state["attempted_edges"].add(blocked_edge)

        old_route = list(current_route)
        destination_edge = old_route[-1]

        # Calculate an alternative with SUMO's router and the ambulance vType.
        alternative = traci.simulation.findRoute(
            current_edge,
            destination_edge,
            vType=AMBULANCE_TYPE,
        )
        candidate = list(alternative.edges)

        valid_candidate = (
            len(candidate) >= 2
            and candidate[0] == current_edge
            and not any(edge_id in closed_set for edge_id in candidate[1:])
        )

        if not valid_candidate:
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_reactive_reroute",
                "trigger_edge": blocked_edge,
                "distance_to_closure_m": round(distance_to_closure, 2),
                "stopped_duration_s": round(stopped_duration, 2),
                "old_route_edges": " ".join(old_route),
                "new_route_edges": " ".join(candidate),
                "result": "no_valid_alternative",
            })
            reroute_state["failed_reroutes"] += 1
            print(
                f"WARNING: SUMO found no valid ambulance route at "
                f"{timestamp:.1f}s around closed edge {blocked_edge}"
            )
            return

        # setRoute requires the replacement route to start with the current edge.
        traci.vehicle.setRoute(AMBULANCE_ID, candidate)
        new_route = list(traci.vehicle.getRoute(AMBULANCE_ID))

        if new_route == old_route:
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_reactive_reroute",
                "trigger_edge": blocked_edge,
                "distance_to_closure_m": round(distance_to_closure, 2),
                "stopped_duration_s": round(stopped_duration, 2),
                "old_route_edges": " ".join(old_route),
                "new_route_edges": " ".join(new_route),
                "result": "route_unchanged",
            })
            reroute_state["failed_reroutes"] += 1
            return

        route_version = int(reroute_state["route_version"]) + 1
        reroute_state["route_version"] = route_version

        append_route_history(
            route_history=route_history,
            scenario_id=scenario_id,
            route_edges=new_route,
            route_version=route_version,
            route_start_time=timestamp,
            route_reason="reactive_closure_recovery_after_stop",
            trigger_edge=blocked_edge,
        )

        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_reactive_reroute",
            "trigger_edge": blocked_edge,
            "distance_to_closure_m": round(distance_to_closure, 2),
            "stopped_duration_s": round(stopped_duration, 2),
            "old_route_edges": " ".join(old_route),
            "new_route_edges": " ".join(new_route),
            "result": "success",
        })

        reroute_state["successful_reroutes"] += 1
        reroute_state["stopped_since"] = None
        reroute_state["last_movement_time"] = float(timestamp)

        print(
            f"Ambulance reactively rerouted at {timestamp:.1f}s after "
            f"stopping near closed edge {blocked_edge} "
            f"({distance_to_closure:.1f} m away)"
        )

    except traci.TraCIException as exc:
        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_reactive_reroute_error",
            "trigger_edge": "",
            "old_route_edges": "",
            "new_route_edges": "",
            "result": str(exc),
        })
        reroute_state["failed_reroutes"] += 1
        reroute_state["stopped_since"] = None


# ============================================================
# EDGE-LEVEL TRAFFIC DATA
# ============================================================

def collect_edge_metrics(edge_accumulator, timestamp, edge_ids=None):
    if edge_ids is None:
        edge_ids = traci.edge.getIDList()
    for edge_id in edge_ids:
        if edge_id.startswith(":"):
            continue

        count = traci.edge.getLastStepVehicleNumber(edge_id)

        if count <= 0:
            continue

        speed = traci.edge.getLastStepMeanSpeed(edge_id)
        occupancy = traci.edge.getLastStepOccupancy(edge_id)

        if edge_id not in edge_accumulator:
            edge_accumulator[edge_id] = {
                "vehicle_samples": 0,
                "speed_sum": 0.0,
                "occupancy_sum": 0.0,
                "max_vehicles": 0,
                "last_seen_s": timestamp,
            }

        item = edge_accumulator[edge_id]
        item["vehicle_samples"] += 1
        item["speed_sum"] += speed
        item["occupancy_sum"] += occupancy
        item["max_vehicles"] = max(item["max_vehicles"], count)
        item["last_seen_s"] = timestamp


def save_edge_metrics(path, accumulator):
    rows = []

    for edge_id, data in accumulator.items():
        n = data["vehicle_samples"]

        rows.append({
            "edge_id": edge_id,
            "observed_steps": n,
            "average_speed_mps": data["speed_sum"] / n,
            "average_occupancy_percent": data["occupancy_sum"] / n,
            "max_vehicles": data["max_vehicles"],
            "last_seen_s": data["last_seen_s"],
        })

    write_csv(path, rows)


def build_ambulance_edge_times(position_rows, scenario_id):
    """Collapse per-second ambulance positions into contiguous road segments."""
    segments = []
    current = None

    def finish_segment(segment):
        if segment is None:
            return

        speeds = segment.pop("_speeds")
        start_time = float(segment["entry_time_s"])
        end_time = float(segment["exit_time_s"])

        segments.append({
            "scenario_id": scenario_id,
            "segment_index": len(segments),
            "edge_id": segment["edge_id"],
            "route_index": segment["route_index"],
            "entry_time_s": start_time,
            "exit_time_s": end_time,
            "observed_duration_s": max(
                float(STEP_LENGTH),
                end_time - start_time + float(STEP_LENGTH),
            ),
            "position_samples": len(speeds),
            "average_speed_mps": mean_or_zero(speeds),
            "minimum_speed_mps": min(speeds) if speeds else 0.0,
            "maximum_speed_mps": max(speeds) if speeds else 0.0,
            "is_internal_edge": int(str(segment["edge_id"]).startswith(":")),
        })

    for row in position_rows:
        edge_id = row["road_id"]
        route_index = row["route_index"]
        timestamp = float(row["time_s"])
        speed = float(row["speed_mps"])

        if (
            current is None
            or current["edge_id"] != edge_id
            or current["route_index"] != route_index
        ):
            finish_segment(current)
            current = {
                "edge_id": edge_id,
                "route_index": route_index,
                "entry_time_s": timestamp,
                "exit_time_s": timestamp,
                "_speeds": [speed],
            }
        else:
            current["exit_time_s"] = timestamp
            current["_speeds"].append(speed)

    finish_segment(current)
    return segments


# ============================================================
# SINGLE SCENARIO EXECUTION
# ============================================================

def run_scenario(scenario, selections, edge_data):
    scenario_id = scenario["id"]
    scenario_name = scenario["name"]
    disruption = scenario["disruption"]

    out = OUTPUT / f"{scenario_id}_{scenario_name}"
    out.mkdir(parents=True, exist_ok=True)

    traffic_file = DEMAND_DIR / scenario["demand"]

    if not traffic_file.exists():
        raise FileNotFoundError(traffic_file)

    tripinfo_path = out / "traffic_tripinfo.xml"
    collision_path = out / "collisions.xml"

    ambulance_positions_path = out / "ambulance_positions.csv"
    route_path = out / "ambulance_route.csv"

    edge_accumulator = {}
    position_rows = []
    scenario_events = []
    route_history = []

    reroute_state = {
        "route_version": 0,
        "attempted_edges": set(),
        "successful_reroutes": 0,
        "failed_reroutes": 0,
        "stopped_since": None,
        "last_movement_time": None,
    }

    demand_count = count_demand_vehicles(traffic_file)
    wall_start = time.perf_counter()
    background_reroute_count = 0
    background_reroute_failures = 0
    background_unroutable_ids = set()
    last_background_reroute_time = -BACKGROUND_REROUTE_INTERVAL_S
    last_progress_time = 0.0
    last_edge_metric_sample = -EDGE_METRIC_SAMPLE_INTERVAL_S

    command = [
        SUMO,
        "-n", str(NETWORK),
        "-r", str(traffic_file),
        "-a", f"{TRAFFIC_LIGHTS},{AMBULANCE}",
        "--begin", "0",
        "--step-length", str(STEP_LENGTH),
        "--seed", str(SEED),
        "--time-to-teleport", "-1",
        "--collision.action", "warn",
        "--collision.check-junctions", "true",
        "--collision-output", str(collision_path),
        "--tripinfo-output", str(tripinfo_path),
        "--no-step-log", "true",
        "--duration-log.disable", "true",
    ]

    print(f"\n{'=' * 60}")
    print(f"Starting {scenario_id}: {scenario_name}")
    print(f"Demand vehicles: {demand_count}")
    print(f"Disruption: {disruption}")

    traci.start(command)
    sim_edge_ids = traci.edge.getIDList()

    def sample_edge_metrics_if_due(timestamp):
        nonlocal last_edge_metric_sample
        if (
            timestamp - last_edge_metric_sample
            >= EDGE_METRIC_SAMPLE_INTERVAL_S - 1e-9
        ):
            collect_edge_metrics(
                edge_accumulator,
                timestamp,
                sim_edge_ids,
            )
            last_edge_metric_sample = timestamp

    stopped_vehicles = set()

    ambulance_route = []
    estimated_time = None
    ambulance_inserted = False
    ambulance_start = None
    ambulance_arrival = None
    disruption_time = None
    disruptions_applied = False

    closed_edges = []
    congested_edges = []

    try:
        # Accumulate traffic until the common ambulance departure time.
        while traci.simulation.getTime() < DEPARTURE_TIME:
            traci.simulationStep()
            update_virtual_stop_line(stopped_vehicles)

            sample_edge_metrics_if_due(float(traci.simulation.getTime()))

        # Calculate the initial route exactly once.
        best_route = find_initial_route()
        ambulance_route = list(best_route.edges)
        estimated_time = float(best_route.travelTime)

        route_id = f"ambulance_route_{scenario_id}"

        traci.route.add(route_id, ambulance_route)

        traci.vehicle.add(
            vehID=AMBULANCE_ID,
            routeID=route_id,
            typeID=AMBULANCE_TYPE,
            depart="now",
            departLane="best",
            departSpeed="max",
        )

        ambulance_inserted = True
        ambulance_start = float(traci.simulation.getTime())

        route_rows = [
            {
                "scenario_id": scenario_id,
                "route_index": i,
                "edge_id": edge_id,
                "is_selected_closure": int(edge_id in selections["all_closures"]),
                "is_selected_congestion": int(edge_id in selections["congestion_edges"]),
            }
            for i, edge_id in enumerate(ambulance_route)
        ]

        write_csv(route_path, route_rows)

        append_route_history(
            route_history=route_history,
            scenario_id=scenario_id,
            route_edges=ambulance_route,
            route_version=0,
            route_start_time=ambulance_start,
            route_reason="initial_route",
        )

        print(f"Initial route edges: {len(ambulance_route)}")
        print(f"Initial estimated travel time: {estimated_time:.2f}s")

        # Apply disruption after the initial route is calculated.
        disruption_time = ambulance_start + DISRUPTION_DELAY

        while (
            ambulance_arrival is None
            and traci.simulation.getTime() < MAX_SIM_TIME
        ):
            traci.simulationStep()

            timestamp = float(traci.simulation.getTime())

            update_virtual_stop_line(stopped_vehicles)

            if (
                disruption != "none"
                and not disruptions_applied
                and timestamp >= disruption_time
            ):
                closed_edges, congested_edges = apply_disruptions(
                    disruption,
                    selections,
                    edge_data,
                )

                disruptions_applied = True

                scenario_events.append({
                    "time_s": timestamp,
                    "event": "disruption_activated",
                    "closed_edges": " ".join(closed_edges),
                    "congested_edges": " ".join(congested_edges),
                })

                print(f"Disruptions activated at {timestamp:.1f}s")

                if closed_edges:
                    batch_rerouted, batch_failed = (
                        reroute_background_traffic_around_closures(
                            closed_edges, background_unroutable_ids
                        )
                    )
                    background_reroute_count += batch_rerouted
                    background_reroute_failures += batch_failed
                    last_background_reroute_time = timestamp
                    scenario_events.append({
                        "time_s": timestamp,
                        "event": "background_traffic_closure_rerouting",
                        "rerouted_vehicle_count": batch_rerouted,
                        "failed_vehicle_count": batch_failed,
                        "result": "completed",
                    })
                    print(
                        f"Background traffic closure response: "
                        f"{batch_rerouted} rerouted, {batch_failed} without a route"
                    )

            # The baseline does not reroute at disruption activation. It keeps
            # its initial route until SUMO brings the ambulance to a sustained
            # stop near a closure anywhere ahead on that route.
            if disruption in ("closure", "combined") and disruptions_applied:
                if (
                    timestamp - last_background_reroute_time
                    >= BACKGROUND_REROUTE_INTERVAL_S - 1e-9
                ):
                    batch_rerouted, batch_failed = (
                        reroute_background_traffic_around_closures(
                            closed_edges, background_unroutable_ids
                        )
                    )
                    background_reroute_count += batch_rerouted
                    background_reroute_failures += batch_failed
                    last_background_reroute_time = timestamp
                    if batch_rerouted or batch_failed:
                        scenario_events.append({
                            "time_s": timestamp,
                            "event": "background_traffic_closure_rerouting",
                            "rerouted_vehicle_count": batch_rerouted,
                            "failed_vehicle_count": batch_failed,
                            "result": "completed",
                        })

                try_reactive_ambulance_reroute(
                    closed_edges=closed_edges,
                    edge_data=edge_data,
                    scenario_id=scenario_id,
                    timestamp=timestamp,
                    route_history=route_history,
                    scenario_events=scenario_events,
                    reroute_state=reroute_state,
                )

            sample_edge_metrics_if_due(timestamp)

            if timestamp - last_progress_time >= PROGRESS_INTERVAL_S:
                wall_elapsed = time.perf_counter() - wall_start
                print(
                    f"[{scenario_id}] simulation t={timestamp:.0f}s "
                    f"| ambulance speed="
                    f"{traci.vehicle.getSpeed(AMBULANCE_ID) if AMBULANCE_ID in traci.vehicle.getIDList() else 0.0:.2f} m/s "
                    f"| wall={wall_elapsed:.1f}s"
                )
                last_progress_time = timestamp

            live_ids = set(traci.vehicle.getIDList())

            if AMBULANCE_ID in live_ids:
                road_id = traci.vehicle.getRoadID(AMBULANCE_ID)

                ambulance_speed = float(traci.vehicle.getSpeed(AMBULANCE_ID))
                ambulance_lane = traci.vehicle.getLaneID(AMBULANCE_ID)
                ambulance_lane_position = float(
                    traci.vehicle.getLanePosition(AMBULANCE_ID)
                )

                position_rows.append({
                    "time_s": timestamp,
                    "vehicle_id": AMBULANCE_ID,
                    "road_id": road_id,
                    "lane_id": ambulance_lane,
                    "lane_position_m": ambulance_lane_position,
                    "x": traci.vehicle.getPosition(AMBULANCE_ID)[0],
                    "y": traci.vehicle.getPosition(AMBULANCE_ID)[1],
                    "speed_mps": ambulance_speed,
                    "acceleration_mps2": traci.vehicle.getAcceleration(AMBULANCE_ID),
                    "route_index": traci.vehicle.getRouteIndex(AMBULANCE_ID),
                })

                # Fail fast if a successful reroute still cannot move the
                # ambulance for five minutes. Save leader/position evidence
                # instead of wasting the remaining 7200-second simulation.
                if reroute_state["successful_reroutes"] > 0:
                    if ambulance_speed > REACTIVE_STOP_SPEED_MPS:
                        reroute_state["last_movement_time"] = timestamp
                    elif reroute_state["last_movement_time"] is not None:
                        stopped_after_reroute = (
                            timestamp - reroute_state["last_movement_time"]
                        )
                        if stopped_after_reroute >= POST_REROUTE_STALL_TIMEOUT_S:
                            leader = traci.vehicle.getLeader(
                                AMBULANCE_ID,
                                250.0,
                            )
                            leader_id = leader[0] if leader else ""
                            leader_gap = leader[1] if leader else ""

                            scenario_events.append({
                                "time_s": timestamp,
                                "event": "ambulance_post_reroute_stall_timeout",
                                "current_edge": road_id,
                                "lane_id": ambulance_lane,
                                "lane_position_m": ambulance_lane_position,
                                "speed_mps": ambulance_speed,
                                "route_index": traci.vehicle.getRouteIndex(AMBULANCE_ID),
                                "stopped_duration_s": stopped_after_reroute,
                                "leader_id": leader_id,
                                "leader_gap_m": leader_gap,
                                "result": "stopped_after_successful_reroute",
                            })
                            print(
                                f"WARNING: ambulance made no progress for "
                                f"{stopped_after_reroute:.0f}s after rerouting; "
                                "ending this scenario early with diagnostics."
                            )
                            break

            if AMBULANCE_ID in traci.simulation.getArrivedIDList():
                ambulance_arrival = timestamp
                print(f"Ambulance arrived at {timestamp:.1f}s")
                break  # End this scenario immediately; the next scenario can start.

            starting_teleports = traci.simulation.getStartingTeleportIDList()
            ending_teleports = traci.simulation.getEndingTeleportIDList()

            if starting_teleports:
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "teleport_start",
                    "vehicles": " ".join(starting_teleports),
                })

            if ending_teleports:
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "teleport_end",
                    "vehicles": " ".join(ending_teleports),
                })

        simulation_end = float(traci.simulation.getTime())
        if simulation_end > last_edge_metric_sample + 1e-9:
            collect_edge_metrics(
                edge_accumulator,
                simulation_end,
                sim_edge_ids,
            )

    finally:
        traci.close()

    # Save the initial route and every reactive route update separately.
    write_csv(out / "ambulance_route_history.csv", route_history)

    # ========================================================
    # PROCESS RESULTS
    # ========================================================

    trip_rows = read_tripinfo(tripinfo_path)

    ambulance_trip = next(
        (row for row in trip_rows if row["vehicle_id"] == AMBULANCE_ID),
        None,
    )

    completed_traffic = [
        row for row in trip_rows
        if row["vehicle_id"] != AMBULANCE_ID
    ]

    if ambulance_trip:
        ambulance_duration = ambulance_trip["duration_s"]
        ambulance_waiting = ambulance_trip["waiting_time_s"]
        ambulance_distance = ambulance_trip["distance_m"]
        actual_departure = ambulance_trip["depart_s"]
        actual_arrival = ambulance_trip["arrival_s"]
        ambulance_completed = True
    else:
        ambulance_duration = ""
        ambulance_waiting = ""
        ambulance_distance = ""
        actual_departure = ambulance_start
        actual_arrival = ""
        ambulance_completed = False

    completed_count = len(completed_traffic)
    unfinished_count = demand_count - completed_count

    traffic_summary = {
        "scenario_id": scenario_id,
        "scenario": scenario_name,
        "demand_file": scenario["demand"],
        "demand_vehicles": demand_count,
        "completed_vehicles": completed_count,
        "unfinished_vehicles": unfinished_count,
        "vehicles_arrived_by_ambulance_stop": completed_count,
        "vehicles_not_arrived_by_ambulance_stop": unfinished_count,
        "average_trip_duration_s": mean_or_zero(
            [r["duration_s"] for r in completed_traffic]
        ),
        "average_waiting_time_s": mean_or_zero(
            [r["waiting_time_s"] for r in completed_traffic]
        ),
        "average_time_loss_s": mean_or_zero(
            [r["time_loss_s"] for r in completed_traffic]
        ),
        "average_trip_distance_m": mean_or_zero(
            [r["distance_m"] for r in completed_traffic]
        ),
        "average_trip_speed_mps": mean_or_zero([
            r["distance_m"] / r["duration_s"]
            for r in completed_traffic
            if r["duration_s"] > 0
        ]),
    }

    write_csv(
        out / "traffic_tripinfo.csv",
        completed_traffic,
        fields=[
            "vehicle_id", "depart_s", "arrival_s", "duration_s",
            "waiting_time_s", "time_loss_s", "distance_m", "depart_delay_s",
        ],
    )

    write_csv(
        out / "traffic_summary.csv",
        [traffic_summary],
    )

    write_csv(
        ambulance_positions_path,
        position_rows,
        fields=[
            "time_s", "vehicle_id", "road_id", "lane_id", "lane_position_m",
            "x", "y", "speed_mps", "acceleration_mps2", "route_index",
        ],
    )

    write_csv(
        out / "ambulance_edge_times.csv",
        build_ambulance_edge_times(position_rows, scenario_id),
        fields=[
            "scenario_id", "segment_index", "edge_id", "route_index",
            "entry_time_s", "exit_time_s", "observed_duration_s",
            "position_samples", "average_speed_mps", "minimum_speed_mps",
            "maximum_speed_mps", "is_internal_edge",
        ],
    )

    # Event types have different optional fields. Let write_csv infer the
    # union of keys across all events so reroute diagnostics (including
    # distance_to_closure_m and stopped_duration_s) are never discarded
    # or rejected by DictWriter.
    write_csv(
        out / "scenario_events.csv",
        scenario_events,
    )

    save_edge_metrics(
        out / "edge_metrics.csv",
        edge_accumulator,
    )

    if collision_path.exists():
        collision_root = ET.parse(collision_path).getroot()
        collisions = collision_root.findall("collision")
    else:
        collisions = []

    collision_rows = [
        {
            "time_s": c.get("time"),
            "type": c.get("type"),
            "lane": c.get("lane"),
            "position_m": c.get("pos"),
            "collider": c.get("collider"),
            "victim": c.get("victim"),
            "collider_speed_mps": c.get("colliderSpeed"),
            "victim_speed_mps": c.get("victimSpeed"),
        }
        for c in collisions
    ]

    write_csv(
        out / "collisions.csv",
        collision_rows,
        fields=[
            "time_s", "type", "lane", "position_m", "collider", "victim",
            "collider_speed_mps", "victim_speed_mps",
        ],
    )

    ambulance_collisions = [
        collision for collision in collisions
        if (
            collision.get("collider") == AMBULANCE_ID
            or collision.get("victim") == AMBULANCE_ID
        )
    ]

    summary = {
        "scenario_id": scenario_id,
        "scenario": scenario_name,
        "demand_file": scenario["demand"],
        "disruption": disruption,
        "seed": SEED,
        "step_length_s": STEP_LENGTH,
        "edge_metric_sample_interval_s": EDGE_METRIC_SAMPLE_INTERVAL_S,
        "simulation_backend": "libsumo" if traci is traci_libsumo else "traci",
        "scenario_wall_time_s": round(time.perf_counter() - wall_start, 3),
        "background_traffic_reroutes": background_reroute_count,
        "background_traffic_reroute_failures": background_reroute_failures,
        "origin": ORIGIN,
        "destination": DESTINATION,
        "scheduled_departure_s": DEPARTURE_TIME,
        "actual_departure_s": actual_departure,
        "ambulance_completed": ambulance_completed,
        "ambulance_arrival_s": actual_arrival,
        "ambulance_travel_time_s": ambulance_duration,
        "ambulance_waiting_time_s": ambulance_waiting,
        "ambulance_distance_m": ambulance_distance,
        "initial_estimated_route_time_s": estimated_time,
        "initial_route_edge_count": len(ambulance_route),
        "baseline_routing_policy": (
            "static_ambulance_route_with_stop_triggered_reactive_recovery"
            "_and_common_background_traffic_closure_response"
            if disruption in ("closure", "combined")
            else "static_route_without_rerouting"
        ),
        "reactive_reroute_lookahead_m": (
            REACTIVE_CLOSURE_LOOKAHEAD_M
            if disruption in ("closure", "combined")
            else ""
        ),
        "congestion_speed_factor": (
            CONGESTION_SPEED_FACTOR
            if disruption in ("congestion", "combined")
            else ""
        ),
        "route_update_count": reroute_state["successful_reroutes"],
        "route_update_failures": reroute_state["failed_reroutes"],
        "post_reroute_stall_timeout_s": POST_REROUTE_STALL_TIMEOUT_S,
        "reactive_reroute_stop_duration_s": (
            REACTIVE_STOP_DURATION_S
            if disruption in ("closure", "combined")
            else ""
        ),
        "ambulance_collision_count": len(ambulance_collisions),
        "disruption_activation_s": (
            disruption_time if disruption != "none" else ""
        ),
        "closed_edges": " ".join(closed_edges),
        "congested_edges": " ".join(congested_edges),
        "traffic_completed": completed_count,
        "traffic_unfinished": unfinished_count,
        "traffic_arrived_by_ambulance_stop": completed_count,
        "traffic_not_arrived_by_ambulance_stop": unfinished_count,
        "traffic_average_duration_s": traffic_summary["average_trip_duration_s"],
        "traffic_average_waiting_s": traffic_summary["average_waiting_time_s"],
        "traffic_average_time_loss_s": traffic_summary["average_time_loss_s"],
        "traffic_average_speed_mps": traffic_summary["average_trip_speed_mps"],
        "collision_count": len(collisions),
        "simulation_end_s": simulation_end,
    }

    write_csv(out / "scenario_summary.csv", [summary])

    print(
        f"Traffic arrivals by ambulance stop time: "
        f"{completed_count}/{demand_count}"
    )
    print(f"Ambulance completed: {ambulance_completed}")
    print(f"Collisions: {len(collisions)}")
    print(f"Saved: {out}")

    return summary


# ============================================================
# MAIN
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Run the six Scenario 0 baseline experiments for a selected "
            "origin/destination pair."
        )
    )
    parser.add_argument(
        "--origin",
        default=ORIGIN,
        help=f"Origin node ID (default: {ORIGIN})",
    )
    parser.add_argument(
        "--destination",
        default=DESTINATION,
        help=f"Destination node ID (default: {DESTINATION})",
    )
    parser.add_argument(
        "--output-tag",
        default="",
        help=(
            "Optional output subfolder under scenario_0/outputs. "
            "Use a different tag for each origin/destination pair."
        ),
    )
    parser.add_argument(
        "--edge-metric-interval",
        type=float,
        default=EDGE_METRIC_SAMPLE_INTERVAL_S,
        help=(
            "Seconds between network-wide edge metric samples "
            f"(default: {EDGE_METRIC_SAMPLE_INTERVAL_S}; use 1 for full sampling)."
        ),
    )
    parser.add_argument(
        "--backend",
        choices=("traci", "libsumo"),
        default="traci",
        help="Simulation interface (libsumo is optional and may be faster).",
    )
    parser.add_argument(
        "--scenarios",
        default="",
        help=(
            "Optional comma-separated scenario IDs to run after the fresh S2 "
            "reference (e.g. S5,S6 for a faster closure test)."
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=SEED,
        help=f"SUMO random seed (default: {SEED})",
    )
    return parser.parse_args()


def main():
    global ORIGIN, DESTINATION, OUTPUT, EDGE_METRIC_SAMPLE_INTERVAL_S, SEED, traci

    args = parse_args()
    requested_scenarios = None
    if args.scenarios.strip():
        requested_scenarios = {
            item.strip().upper()
            for item in args.scenarios.split(",")
            if item.strip()
        }
        valid_ids = {scenario["id"] for scenario in SCENARIOS}
        invalid_ids = requested_scenarios - valid_ids
        if invalid_ids:
            raise ValueError(
                "Unknown scenario IDs: " + ", ".join(sorted(invalid_ids))
            )

    ORIGIN = str(args.origin)
    DESTINATION = str(args.destination)
    SEED = int(args.seed)

    if args.edge_metric_interval <= 0:
        raise ValueError("--edge-metric-interval must be greater than zero.")
    EDGE_METRIC_SAMPLE_INTERVAL_S = float(args.edge_metric_interval)

    if args.output_tag:
        tag = args.output_tag.strip().replace("\\", "_").replace("/", "_")
        if not tag or tag in (".", ".."):
            raise ValueError("Invalid --output-tag.")
        OUTPUT = BASE / "outputs" / tag
    else:
        OUTPUT = BASE / "outputs"

    OUTPUT.mkdir(parents=True, exist_ok=True)

    if args.backend == "libsumo":
        if traci_libsumo is None:
            raise RuntimeError(
                "libsumo is not installed. Install the version matching "
                "your SUMO 1.28.0 installation (for example: "
                "pip install libsumo==1.28.0), or use --backend traci."
            )
        traci = traci_libsumo
    else:
        traci = traci_socket

    print("Loading network and selecting disruption roads...")
    print(f"Origin: {ORIGIN} | Destination: {DESTINATION}")
    print(f"Backend: {'libsumo' if traci is traci_libsumo else 'traci'}")
    print(f"Edge metric sample interval: {EDGE_METRIC_SAMPLE_INTERVAL_S:g}s")

    graph, edge_data = load_network_graph()
    transition_graph = load_edge_transition_graph()

    # Always regenerate S2 once. The same fresh S2 result is reused as the
    # reference and is not simulated a second time in this batch.
    moderate = next(s for s in SCENARIOS if s["id"] == "S2")
    empty_selections = {
        "route_closures": [],
        "nearby_closures": [],
        "all_closures": [],
        "congestion_edges": [],
    }

    print("\nRunning a fresh S2 reference experiment...")
    reference_summary = run_scenario(
        moderate,
        empty_selections,
        edge_data,
    )

    if not reference_summary.get("ambulance_completed"):
        raise RuntimeError(
            "Fresh S2 ambulance did not reach the destination. "
            "Fix S2 before selecting disruption roads or running comparisons."
        )

    reference_route_file = OUTPUT / "S2_moderate" / "ambulance_route.csv"
    with reference_route_file.open(
        newline="", encoding="utf-8"
    ) as f:
        reference_route = [
            row["edge_id"] for row in csv.DictReader(f)
        ]

    if not reference_route:
        raise RuntimeError(
            "Fresh S2 did not produce an ambulance route. "
            "Cannot select disruption roads."
        )

    traffic_od_pairs = load_traffic_od_pairs(
        DEMAND_DIR / moderate["demand"]
    )
    selections = select_disruption_roads(
        reference_route,
        graph,
        edge_data,
        transition_graph,
        traffic_od_pairs,
    )

    print("\nSELECTED DISRUPTION ROADS")
    print("Route closures:", selections["route_closures"])
    print("Nearby closures:", selections["nearby_closures"])
    print("Congestion corridor edges:", selections["congestion_edges"])
    print("All closures:", selections["all_closures"])

    disruption_selection_rows = []

    for role, edge_list in (
        ("ambulance_route_closure", selections["route_closures"]),
        ("nearby_alternative_closure", selections["nearby_closures"]),
        ("localized_congestion", selections["congestion_edges"]),
    ):
        for edge_id in edge_list:
            data = edge_data[edge_id]
            original_speed = float(data["speed"])
            applied_speed = (
                max(1.0, original_speed * CONGESTION_SPEED_FACTOR)
                if role == "localized_congestion"
                else original_speed
            )

            disruption_selection_rows.append({
                "edge_id": edge_id,
                "role": role,
                "from_node": data["from"],
                "to_node": data["to"],
                "length_m": data["length"],
                "original_speed_limit_mps": original_speed,
                "applied_speed_limit_mps": applied_speed,
            })

    write_csv(
        OUTPUT / "disruption_edge_selection.csv",
        disruption_selection_rows,
    )

    # Run each remaining scenario once; S2 was already generated freshly.
    summaries = [reference_summary]

    for scenario in SCENARIOS:
        if scenario["id"] == "S2":
            continue
        if (
            requested_scenarios is not None
            and scenario["id"] not in requested_scenarios
        ):
            continue

        summaries.append(
            run_scenario(scenario, selections, edge_data)
        )

    summaries.sort(key=lambda row: row["scenario_id"])

    s2_time = reference_summary.get("ambulance_travel_time_s", "")
    comparison_rows = []

    for row in summaries:
        travel_time = row.get("ambulance_travel_time_s", "")
        delta = ""
        percent = ""

        if s2_time not in ("", None) and travel_time not in ("", None):
            delta = float(travel_time) - float(s2_time)
            if float(s2_time) > 0:
                percent = (delta / float(s2_time)) * 100.0

        comparison_rows.append({
            "scenario_id": row["scenario_id"],
            "scenario": row["scenario"],
            "ambulance_completed": row["ambulance_completed"],
            "ambulance_travel_time_s": travel_time,
            "delta_vs_fresh_S2_s": delta,
            "percent_change_vs_fresh_S2": percent,
            "route_update_count": row.get("route_update_count", 0),
            "ambulance_collision_count": row.get("ambulance_collision_count", 0),
            "background_traffic_reroutes": row.get("background_traffic_reroutes", 0),
        })

    write_csv(
        OUTPUT / "scenario_0_ambulance_comparison.csv",
        comparison_rows,
    )
    write_csv(
        OUTPUT / "all_scenarios_summary.csv",
        summaries,
    )

    summaries_by_id = {row["scenario_id"]: row for row in summaries}

    print("\n" + "=" * 60)
    print("SCENARIO 0 RESULTS")
    print(f"Fresh S2 ambulance travel time: {s2_time} s")

    if "S4" in summaries_by_id:
        s4_time = summaries_by_id["S4"].get("ambulance_travel_time_s", "")
        if s4_time not in ("", None) and s2_time not in ("", None):
            delta = float(s4_time) - float(s2_time)
            print(f"S4 congestion change vs S2: {delta:+.1f} s")
            if delta > 0:
                print("S4 validation: congestion increased ambulance travel time.")
            else:
                print(
                    "S4 validation: congestion did NOT increase total ambulance "
                    "travel time. Inspect route-level data before calling S4 finished."
                )
        else:
            print("S4 validation: unavailable because S2 or S4 ambulance did not arrive.")

    for scenario_id in ("S5", "S6"):
        if scenario_id in summaries_by_id:
            row = summaries_by_id[scenario_id]
            print(
                f"{scenario_id} ambulance completed: {row['ambulance_completed']} "
                f"(ambulance reroutes: {row.get('route_update_count', 0)}; "
                f"background reroutes: {row.get('background_traffic_reroutes', 0)})"
            )

    print("\n" + "=" * 60)
    print(
        "SCENARIO BATCH COMPLETE: "
        + ", ".join(row["scenario_id"] for row in summaries)
    )
    print(f"Master summary: {OUTPUT / 'all_scenarios_summary.csv'}")
    print(f"Ambulance comparison: {OUTPUT / 'scenario_0_ambulance_comparison.csv'}")
    print(f"Disruption selection: {OUTPUT / 'disruption_edge_selection.csv'}")
    print(
        "Note: edge_metrics.csv is sampled every "
        f"{EDGE_METRIC_SAMPLE_INTERVAL_S:g}s; ambulance positions remain 1s."
    )


if __name__ == "__main__":
    main()