from __future__ import annotations

import argparse
import csv
import json
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
OUTPUT = BASE / "outputs" / "pair_4_to_hospital_95_final"

SUMO = r"C:\Program Files (x86)\SUMO\bin\sumo.exe"

ORIGIN = "4"
DESTINATION = "hospital_95"
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

# Ambulance-only local overtaking.
# TraCI lane-change requests use SUMO collision-avoidance mode 256. This
# disables autonomous lane changes while allowing an explicit ambulance
# lane-change request with immediate-collision avoidance.
OVERTAKE_TRIGGER_GAP_M = 25.0
OVERTAKE_TRIGGER_SPEED_MPS = 2.5
OVERTAKE_MIN_REMAINING_M = 12.0
OVERTAKE_LANE_CHANGE_DURATION_S = 8.0
OVERTAKE_PASS_CLEARANCE_M = 3.0
OVERTAKE_MAX_PENDING_S = 8.0
OVERTAKE_RETRY_INTERVAL_S = 2.0
OVERTAKE_COLLISION_AVOIDANCE_MODE = 256

# Observed physical bottlenecks from the S5/S6 diagnostic runs.
# Rerouting avoids these roads before the ambulance reaches them.
AMBULANCE_KNOWN_BLOCKAGE_EDGES = ("e31to144k0", "e170to38k0")
DEFAULT_LANE_CHANGE_MODE = 1621

# Ordinary traffic is not actively rerouted. Closures affect the whole network;
# unfinished background trips are accepted as a realistic consequence.

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

# Second ambulance pair: fixed 20-edge reference route for all six scenarios.
# This is the first complete Route 2 experiment; S1-S3 must be run, not imported.
PAIR2_REFERENCE_ROUTE = [
    "e4to6k0",
    "e6to21k0",
    "e21to22k0",
    "e22to24k0",
    "e24to200k0",
    "e200to201k0",
    "e201to29k0",
    "e29to31k0",
    "e31to144k0",
    "e144to143k0",
    "e143to146k0",
    "e146to145k0",
    "e145to16k0",
    "e16to142k0",
    "e142to151k0",
    "e151to152k0",
    "e152to153k0",
    "e153to86k0",
    "e86to95k0",
    "e95to89k0__to_hospital_95",
]
PAIR2_EXPECTED_ROUTE_LENGTH_M = 2382.0

# S4: four consecutive roads on the second route (578.61 m total).
PAIR2_CONGESTION_EDGES = [
    "e200to201k0",
    "e201to29k0",
    "e29to31k0",
    "e31to144k0",
]

# S5/S6: two roads on the ambulance route and two nearby roads.
# The original nearby closure e89to142k0 trapped vehicles on e88to89k0,
# and e16to142k0 trapped vehicles approaching from e13to16k0. The revised
# set below passed a full route-state reachability check over all 400
# moderate-demand vehicle routes (including every possible open-edge position).
PAIR2_ROUTE_CLOSURES = [
    "e144to143k0",
    "e86to95k0",
]
PAIR2_NEARBY_CLOSURES = [
    "e13to16k0",
    "e140to142k0",
]
PAIR2_ALL_CLOSURES = PAIR2_ROUTE_CLOSURES + PAIR2_NEARBY_CLOSURES


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


def load_lane_transition_map():
    """Return lane-level movements: (from_edge, from_lane, to_edge)."""
    root = ET.parse(NETWORK).getroot()
    movements = set()
    for connection in root.findall("connection"):
        from_edge = connection.get("from")
        to_edge = connection.get("to")
        from_lane = connection.get("fromLane")
        if not from_edge or not to_edge or from_lane is None:
            continue
        if from_edge.startswith(":") or to_edge.startswith(":"):
            continue
        try:
            movements.add((from_edge, int(from_lane), to_edge))
        except ValueError:
            continue
    return movements


def validate_ambulance_detour(
    scenario_id, edge_data, transition_graph, lane_transition_map
):
    """Preflight-check the S5/S6 detour at edge and lane-connection level."""
    forbidden = (
        set(PAIR2_ALL_CLOSURES)
        | set(AMBULANCE_KNOWN_BLOCKAGE_EDGES)
    )
    if scenario_id == "S6":
        forbidden |= set(PAIR2_CONGESTION_EDGES)

    start_edge = PAIR2_REFERENCE_ROUTE[0]
    destination_edge = PAIR2_REFERENCE_ROUTE[-1]
    graph = nx.DiGraph()

    for first_edge, second_edge in transition_graph.edges:
        if first_edge in forbidden or second_edge in forbidden:
            continue
        if first_edge.startswith("hospital_") or "__to_hospital_" in first_edge:
            continue
        if second_edge.startswith("hospital_") or (
            "__to_hospital_" in second_edge and second_edge != destination_edge
        ):
            continue
        data = edge_data.get(second_edge)
        if data is None:
            continue
        graph.add_edge(
            first_edge,
            second_edge,
            weight=float(data["length"]) / max(float(data["speed"]), 0.1),
        )

    if start_edge not in graph:
        graph.add_node(start_edge)
        for next_edge in transition_graph.successors(start_edge):
            if next_edge in forbidden or next_edge not in edge_data:
                continue
            graph.add_edge(
                start_edge,
                next_edge,
                weight=float(edge_data[next_edge]["length"])
                / max(float(edge_data[next_edge]["speed"]), 0.1),
            )

    try:
        route = nx.shortest_path(
            graph, source=start_edge, target=destination_edge, weight="weight"
        )
    except (nx.NetworkXNoPath, nx.NodeNotFound) as exc:
        raise RuntimeError(
            f"No prevalidated ambulance detour exists for {scenario_id}."
        ) from exc

    invalid_movements = [
        (first, second)
        for first, second in zip(route, route[1:])
        if not any(
            (first, lane_index, second) in lane_transition_map
            for lane_index in range(10)
        )
    ]
    if invalid_movements:
        raise RuntimeError(
            f"{scenario_id} detour has lane-incompatible movements: "
            f"{invalid_movements}"
        )

    return route


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



def try_reactive_ambulance_reroute(
    closed_edges,
    congested_edges,
    edge_data,
    transition_graph,
    scenario_id,
    timestamp,
    route_history,
    scenario_events,
    reroute_state,
):
    """
    Replan the ambulance immediately after a road closure activates.

    The earlier stop-triggered strategy waited until the ambulance had already
    entered physical bottlenecks. Diagnostics showed that a stopped ambulance
    cannot always pass safely when adjacent lanes do not connect to its next
    turn. This version replans at disruption activation and avoids the two
    previously observed blocking edges, while leaving ordinary traffic alone.
    """
    if not closed_edges or AMBULANCE_ID not in traci.vehicle.getIDList():
        return

    closed_set = set(closed_edges)
    congested_set = set(congested_edges or [])
    known_blockage_set = set(AMBULANCE_KNOWN_BLOCKAGE_EDGES)
    forbidden_after_current = closed_set | congested_set | known_blockage_set

    try:
        current_route = list(traci.vehicle.getRoute(AMBULANCE_ID))
        route_index = int(traci.vehicle.getRouteIndex(AMBULANCE_ID))
        road_id = traci.vehicle.getRoadID(AMBULANCE_ID)

        if not current_route or route_index < 0 or route_index >= len(current_route):
            return

        if road_id and not road_id.startswith(":") and road_id in current_route:
            matches = [i for i, edge in enumerate(current_route) if edge == road_id]
            current_edge_index = min(matches, key=lambda i: abs(i - route_index))
        elif road_id.startswith(":"):
            current_edge_index = min(route_index, len(current_route) - 1)
        else:
            current_edge_index = min(route_index, len(current_route) - 1)

        current_edge = current_route[current_edge_index]
        blocked_index = next(
            (
                i for i in range(current_edge_index + 1, len(current_route))
                if current_route[i] in closed_set
            ),
            None,
        )
        if blocked_index is None:
            return

        blocked_edge = current_route[blocked_index]
        if blocked_edge in reroute_state["attempted_edges"]:
            return
        reroute_state["attempted_edges"].add(blocked_edge)

        lane_id = traci.vehicle.getLaneID(AMBULANCE_ID)
        lane_length = float(traci.lane.getLength(lane_id))
        lane_position = float(traci.vehicle.getLanePosition(AMBULANCE_ID))
        distance_to_closure = max(0.0, lane_length - lane_position)
        for edge_id in current_route[current_edge_index + 1:blocked_index]:
            data = edge_data.get(edge_id)
            if data is not None:
                distance_to_closure += float(data["length"])

        old_route = list(current_route)
        destination_edge = old_route[-1]

        route_graph = nx.DiGraph()
        for first_edge, second_edge in transition_graph.edges:
            if first_edge in forbidden_after_current or second_edge in forbidden_after_current:
                continue
            if first_edge.startswith("hospital_") or "__to_hospital_" in first_edge:
                continue
            if second_edge.startswith("hospital_") or (
                "__to_hospital_" in second_edge and second_edge != destination_edge
            ):
                continue
            second_data = edge_data.get(second_edge)
            if second_data is None:
                continue
            route_graph.add_edge(
                first_edge,
                second_edge,
                weight=float(second_data["length"]) / max(float(second_data["speed"]), 0.1),
            )

        # The ambulance may already occupy an edge that is congested or marked
        # as a known bottleneck. It may leave that edge, but must not re-enter it.
        if current_edge not in route_graph:
            route_graph.add_node(current_edge)
        for next_edge in transition_graph.successors(current_edge):
            if next_edge in forbidden_after_current or next_edge not in edge_data:
                continue
            if next_edge.startswith("hospital_") or (
                "__to_hospital_" in next_edge and next_edge != destination_edge
            ):
                continue
            route_graph.add_edge(
                current_edge,
                next_edge,
                weight=float(edge_data[next_edge]["length"]) / max(
                    float(edge_data[next_edge]["speed"]), 0.1
                ),
            )

        try:
            candidate = nx.shortest_path(
                route_graph,
                source=current_edge,
                target=destination_edge,
                weight="weight",
            )
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            candidate = []

        valid_candidate = (
            len(candidate) >= 2
            and candidate[0] == current_edge
            and not any(edge in forbidden_after_current for edge in candidate[1:])
            and all(
                transition_graph.has_edge(first, second)
                for first, second in zip(candidate, candidate[1:])
            )
        )

        if not valid_candidate:
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_disruption_reroute",
                "trigger_edge": blocked_edge,
                "distance_to_closure_m": round(distance_to_closure, 2),
                "old_route_edges": " ".join(old_route),
                "new_route_edges": " ".join(candidate),
                "avoided_edges": " ".join(sorted(forbidden_after_current)),
                "result": "no_valid_alternative",
            })
            reroute_state["failed_reroutes"] += 1
            print(
                f"WARNING: no valid ambulance detour at {timestamp:.1f}s "
                f"around closed edge {blocked_edge}"
            )
            return

        traci.vehicle.setRoute(AMBULANCE_ID, candidate)
        new_route = list(traci.vehicle.getRoute(AMBULANCE_ID))
        if new_route == old_route:
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_disruption_reroute",
                "trigger_edge": blocked_edge,
                "distance_to_closure_m": round(distance_to_closure, 2),
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
            route_reason="reroute_on_disruption_activation_avoiding_observed_blockages",
            trigger_edge=blocked_edge,
        )

        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_disruption_reroute",
            "trigger_edge": blocked_edge,
            "distance_to_closure_m": round(distance_to_closure, 2),
            "old_route_edges": " ".join(old_route),
            "new_route_edges": " ".join(new_route),
            "avoided_edges": " ".join(sorted(forbidden_after_current)),
            "result": "success",
            "reroute_policy": "immediate_after_disruption_activation",
        })

        reroute_state["successful_reroutes"] += 1
        reroute_state["stopped_since"] = None
        reroute_state["last_movement_time"] = float(timestamp)

        print(
            f"Ambulance rerouted at {timestamp:.1f}s immediately after "
            f"disruption activation; avoided {', '.join(sorted(known_blockage_set))}"
        )

    except traci.TraCIException as exc:
        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_disruption_reroute_error",
            "trigger_edge": "",
            "old_route_edges": "",
            "new_route_edges": "",
            "result": str(exc),
        })
        reroute_state["failed_reroutes"] += 1


def _lane_change_state_text(veh_id: str, direction: int) -> str:
    """Return SUMO's lane-change diagnostic state when supported."""
    try:
        state, state_after_traci = traci.vehicle.getLaneChangeState(
            veh_id, direction
        )
        return f"model={state};traci={state_after_traci}"
    except (AttributeError, traci.TraCIException):
        return "unavailable"


def _restore_ambulance_lane_change_mode(overtake_state):
    """Restore SUMO's normal lane-change mode after the temporary maneuver."""
    if AMBULANCE_ID not in traci.vehicle.getIDList():
        return
    mode = int(
        overtake_state.get(
            "original_lane_change_mode", DEFAULT_LANE_CHANGE_MODE
        )
    )
    traci.vehicle.setLaneChangeMode(AMBULANCE_ID, mode)


def manage_ambulance_overtake(timestamp, overtake_state, scenario_events, lane_transition_map):
    """Use a TraCI-controlled, collision-avoiding lane change to pass a blocker.

    Only the ambulance receives lane-change commands. SUMO's mode 256 disables
    autonomous lane changes and applies immediate-collision avoidance to the
    explicit TraCI request. The ambulance returns to its original lane after
    passing the leader, before reaching the next junction whenever possible.
    """
    live_ids = set(traci.vehicle.getIDList())
    if AMBULANCE_ID not in live_ids:
        overtake_state.clear()
        return

    try:
        road_id = traci.vehicle.getRoadID(AMBULANCE_ID)
        if not road_id or road_id.startswith(":"):
            return

        lane_id = traci.vehicle.getLaneID(AMBULANCE_ID)
        lane_index = int(traci.vehicle.getLaneIndex(AMBULANCE_ID))
        lane_position = float(traci.vehicle.getLanePosition(AMBULANCE_ID))
        lane_length = float(traci.lane.getLength(lane_id))
        remaining = max(0.0, lane_length - lane_position)
        ambulance_speed = float(traci.vehicle.getSpeed(AMBULANCE_ID))
        current_route = list(traci.vehicle.getRoute(AMBULANCE_ID))
        route_index = int(traci.vehicle.getRouteIndex(AMBULANCE_ID))
        next_route_edge = (
            current_route[route_index + 1]
            if 0 <= route_index < len(current_route) - 1
            else None
        )

        tracked_edge = overtake_state.get("edge_id")
        if tracked_edge and road_id != tracked_edge:
            _restore_ambulance_lane_change_mode(overtake_state)
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_overtake_ended",
                "edge_id": tracked_edge,
                "new_edge_id": road_id,
                "result": (
                    "returned_to_original_lane"
                    if lane_index == overtake_state.get("original_lane")
                    else "edge_changed_before_return"
                ),
            })
            overtake_state.clear()
            return

        # Confirm a requested move into the passing lane.
        if overtake_state.get("pending"):
            target_lane = int(overtake_state["target_lane"])
            if lane_index == target_lane:
                overtake_state["pending"] = False
                overtake_state["active"] = True
                overtake_state["last_request_time"] = float(timestamp)
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "ambulance_overtake_started",
                    "edge_id": road_id,
                    "from_lane": overtake_state.get("original_lane"),
                    "to_lane": lane_index,
                    "leader_id": overtake_state.get("leader_id", ""),
                    "result": "lane_change_completed",
                })
            elif (
                timestamp - float(overtake_state.get("request_time", timestamp))
                > OVERTAKE_MAX_PENDING_S
            ):
                direction = 1 if target_lane > lane_index else -1
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "ambulance_overtake_request_failed",
                    "edge_id": road_id,
                    "from_lane": lane_index,
                    "to_lane": target_lane,
                    "leader_id": overtake_state.get("leader_id", ""),
                    "lane_change_state": _lane_change_state_text(
                        AMBULANCE_ID, direction
                    ),
                    "result": "SUMO_did_not_complete_lane_change",
                })
                attempted = set(overtake_state.get("attempted_lanes", []))
                attempted.add(target_lane)
                original_lane = int(overtake_state["original_lane"])
                leader_id = overtake_state.get("leader_id", "")
                original_mode = int(overtake_state.get(
                    "original_lane_change_mode", DEFAULT_LANE_CHANGE_MODE
                ))
                overtake_state.clear()
                overtake_state.update({
                    "attempted_lanes": sorted(attempted),
                    "retry_after": float(timestamp) + 1.0,
                    "original_lane": original_lane,
                    "leader_id": leader_id,
                    "original_lane_change_mode": original_mode,
                    "edge_id": road_id,
                })
            else:
                last_request = float(
                    overtake_state.get("last_request_time", timestamp)
                )
                if timestamp - last_request >= OVERTAKE_RETRY_INTERVAL_S:
                    traci.vehicle.changeLane(
                        AMBULANCE_ID,
                        target_lane,
                        OVERTAKE_LANE_CHANGE_DURATION_S,
                    )
                    overtake_state["last_request_time"] = float(timestamp)
                return

        # Active passing maneuver: return to the original lane after the
        # ambulance's front has cleared the leader's rear.
        if overtake_state.get("active") or overtake_state.get("returning"):
            original_lane = int(overtake_state["original_lane"])
            leader_id = overtake_state.get("leader_id", "")
            passed = not leader_id or leader_id not in live_ids

            if not passed:
                try:
                    leader_road = traci.vehicle.getRoadID(leader_id)
                    leader_position = float(
                        traci.vehicle.getLanePosition(leader_id)
                    )
                    leader_length = float(traci.vehicle.getLength(leader_id))
                    passed = (
                        leader_road != road_id
                        or lane_position
                        > leader_position + leader_length + OVERTAKE_PASS_CLEARANCE_M
                    )
                except traci.TraCIException:
                    passed = True

            if overtake_state.get("returning"):
                if lane_index == original_lane:
                    _restore_ambulance_lane_change_mode(overtake_state)
                    scenario_events.append({
                        "time_s": timestamp,
                        "event": "ambulance_overtake_completed",
                        "edge_id": road_id,
                        "returned_lane": original_lane,
                        "leader_id": leader_id,
                        "result": "returned_to_route_lane",
                    })
                    overtake_state.clear()
                    return

                last_request = float(
                    overtake_state.get("last_request_time", timestamp)
                )
                if timestamp - last_request >= OVERTAKE_RETRY_INTERVAL_S:
                    traci.vehicle.changeLane(
                        AMBULANCE_ID,
                        original_lane,
                        OVERTAKE_LANE_CHANGE_DURATION_S,
                    )
                    overtake_state["last_request_time"] = float(timestamp)
                return

            if passed:
                traci.vehicle.changeLane(
                    AMBULANCE_ID,
                    original_lane,
                    OVERTAKE_LANE_CHANGE_DURATION_S,
                )
                overtake_state["returning"] = True
                overtake_state["last_request_time"] = float(timestamp)
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "ambulance_overtake_return_requested",
                    "edge_id": road_id,
                    "from_lane": lane_index,
                    "to_lane": original_lane,
                    "leader_id": leader_id,
                    "remaining_distance_m": round(remaining, 2),
                    "result": "return_to_route_lane_requested",
                })
            elif remaining <= 5.0:
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "ambulance_overtake_low_merge_distance",
                    "edge_id": road_id,
                    "remaining_distance_m": round(remaining, 2),
                    "leader_id": leader_id,
                    "result": "still_passing_near_edge_end",
                })
            return

        # Trigger while creeping/stopped behind a close, slow leader.
        if ambulance_speed > OVERTAKE_TRIGGER_SPEED_MPS:
            return
        if remaining < OVERTAKE_MIN_REMAINING_M:
            return
        if timestamp < float(overtake_state.get("retry_after", 0.0)):
            return

        leader = traci.vehicle.getLeader(
            AMBULANCE_ID, OVERTAKE_TRIGGER_GAP_M
        )
        if not leader:
            return

        leader_id, leader_gap = leader
        if float(leader_gap) > OVERTAKE_TRIGGER_GAP_M:
            return
        if leader_id not in live_ids:
            return
        leader_speed = float(traci.vehicle.getSpeed(leader_id))
        if leader_speed > OVERTAKE_TRIGGER_SPEED_MPS:
            return

        lane_count = int(traci.edge.getLaneNumber(road_id))
        candidates = sorted(
            (idx for idx in range(lane_count) if idx != lane_index),
            key=lambda idx: abs(idx - lane_index),
        )
        # A passing lane is eligible only if it has a legal connection to the
        # ambulance's next route edge. This prevents the two failed maneuvers
        # observed on e31to144k0 and e170to38k0.
        route_compatible_candidates = []
        for candidate_lane in candidates:
            if next_route_edge is None or (
                road_id, candidate_lane, next_route_edge
            ) in lane_transition_map:
                route_compatible_candidates.append(candidate_lane)
            else:
                scenario_events.append({
                    "time_s": timestamp,
                    "event": "ambulance_overtake_lane_incompatible",
                    "edge_id": road_id,
                    "candidate_lane": candidate_lane,
                    "next_route_edge": next_route_edge,
                    "result": "lane_has_no_connection_to_next_route_edge",
                })
        candidates = route_compatible_candidates
        attempted = set(overtake_state.get("attempted_lanes", []))
        candidates = [idx for idx in candidates if idx not in attempted]

        if not candidates:
            _restore_ambulance_lane_change_mode(overtake_state)
            scenario_events.append({
                "time_s": timestamp,
                "event": "ambulance_overtake_no_lane_available",
                "edge_id": road_id,
                "lane_index": lane_index,
                "leader_id": leader_id,
                "leader_gap_m": round(float(leader_gap), 3),
                "remaining_distance_m": round(remaining, 2),
                "result": "all_lanes_already_attempted",
            })
            overtake_state.clear()
            overtake_state["retry_after"] = float(timestamp) + 10.0
            return

        target_lane = candidates[0]
        direction = 1 if target_lane > lane_index else -1

        if "original_lane_change_mode" not in overtake_state:
            try:
                original_mode = int(
                    traci.vehicle.getLaneChangeMode(AMBULANCE_ID)
                )
            except (AttributeError, traci.TraCIException):
                original_mode = DEFAULT_LANE_CHANGE_MODE
            overtake_state["original_lane_change_mode"] = original_mode

        # Prioritize this explicit ambulance request while retaining
        # SUMO's immediate-collision avoidance (lane-change mode 256).
        traci.vehicle.setLaneChangeMode(
            AMBULANCE_ID, OVERTAKE_COLLISION_AVOIDANCE_MODE
        )
        traci.vehicle.changeLane(
            AMBULANCE_ID,
            target_lane,
            OVERTAKE_LANE_CHANGE_DURATION_S,
        )

        overtake_state.update({
            "pending": True,
            "active": False,
            "returning": False,
            "request_time": float(timestamp),
            "last_request_time": float(timestamp),
            "original_lane": lane_index,
            "target_lane": target_lane,
            "leader_id": leader_id,
            "edge_id": road_id,
            "attempted_lanes": sorted(attempted),
        })
        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_overtake_requested",
            "edge_id": road_id,
            "from_lane": lane_index,
            "to_lane": target_lane,
            "leader_id": leader_id,
            "leader_gap_m": round(float(leader_gap), 3),
            "leader_speed_mps": round(leader_speed, 3),
            "remaining_distance_m": round(remaining, 2),
            "lane_change_state": _lane_change_state_text(
                AMBULANCE_ID, direction
            ),
            "result": "TraCI_request_sent_collision_avoidance_mode_256",
        })

    except (traci.TraCIException, ValueError, KeyError, IndexError) as exc:
        scenario_events.append({
            "time_s": timestamp,
            "event": "ambulance_overtake_error",
            "edge_id": overtake_state.get("edge_id", ""),
            "result": str(exc),
        })
        try:
            _restore_ambulance_lane_change_mode(overtake_state)
        except traci.TraCIException:
            pass
        overtake_state.clear()


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

def run_scenario(scenario, selections, edge_data, transition_graph, lane_transition_map):
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

    overtake_state = {}

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
        # Use the same verified 20-edge reference route in all six scenarios.
        # Do not let a traffic-dependent route choice change the experiment.
        ambulance_route = list(PAIR2_REFERENCE_ROUTE)
        estimated_time = sum(
            float(edge_data[edge_id]["length"]) / max(float(edge_data[edge_id]["speed"]), 0.1)
            for edge_id in ambulance_route
        )

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

            # Only the ambulance receives reactive route recovery. Ordinary
            # traffic is left to respond naturally to the road closures.
            if disruption in ("closure", "combined") and disruptions_applied:
                try_reactive_ambulance_reroute(
                    closed_edges=closed_edges,
                    congested_edges=congested_edges,
                    edge_data=edge_data,
                    transition_graph=transition_graph,
                    scenario_id=scenario_id,
                    timestamp=timestamp,
                    route_history=route_history,
                    scenario_events=scenario_events,
                    reroute_state=reroute_state,
                )

            # Ambulance-only local passing attempt. Ordinary traffic routes,
            # speeds, and positions are otherwise left untouched.
            manage_ambulance_overtake(
                timestamp=timestamp,
                overtake_state=overtake_state,
                scenario_events=scenario_events,
                lane_transition_map=lane_transition_map,
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
            "static_route_with_disruption_activation_rerouting_and_observed_bottleneck_avoidance"
            if disruption in ("closure", "combined")
            else "static_route_without_rerouting"
        ),
        "reroute_trigger_policy": (
            "immediate_after_disruption_activation"
            if disruption in ("closure", "combined")
            else ""
        ),
        "known_blockage_edges_avoided": (
            " ".join(AMBULANCE_KNOWN_BLOCKAGE_EDGES)
            if disruption in ("closure", "combined")
            else ""
        ),
        "reactive_reroute_lookahead_m": "",
        "congestion_speed_factor": (
            CONGESTION_SPEED_FACTOR
            if disruption in ("congestion", "combined")
            else ""
        ),
        "route_update_count": reroute_state["successful_reroutes"],
        "route_update_failures": reroute_state["failed_reroutes"],
        "ambulance_overtake_attempts": sum(
            1 for event in scenario_events
            if event.get("event") == "ambulance_overtake_requested"
        ),
        "ambulance_overtakes_completed": sum(
            1 for event in scenario_events
            if event.get("event") == "ambulance_overtake_completed"
        ),
        "post_reroute_stall_timeout_s": POST_REROUTE_STALL_TIMEOUT_S,
        "reactive_reroute_stop_duration_s": "",
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
            "Run the complete six-scenario experiment for the second RouteX "
            "ambulance pair (node 4 to hospital_95)."
        )
    )
    parser.add_argument("--origin", default=ORIGIN)
    parser.add_argument("--destination", default=DESTINATION)
    parser.add_argument(
        "--output-tag",
        default="pair_4_to_hospital_95_final",
        help="Output subfolder for the complete Route 2 experiment.",
    )
    parser.add_argument(
        "--edge-metric-interval", type=float,
        default=EDGE_METRIC_SAMPLE_INTERVAL_S,
    )
    parser.add_argument(
        "--backend", choices=("traci", "libsumo"), default="traci",
    )
    parser.add_argument(
        "--scenarios", default="S1,S2,S3,S4,S5,S6",
        help="Comma-separated subset of S1,S2,S3,S4,S5,S6 (default: all six).",
    )
    parser.add_argument("--seed", type=int, default=SEED)
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Allow overwriting files in the selected output folder.",
    )
    return parser.parse_args()


def main():
    global ORIGIN, DESTINATION, OUTPUT, EDGE_METRIC_SAMPLE_INTERVAL_S, SEED, traci

    args = parse_args()
    ORIGIN = str(args.origin)
    DESTINATION = str(args.destination)
    if ORIGIN != "4" or DESTINATION != "hospital_95":
        raise ValueError(
            "This configuration is specifically validated for "
            "origin 4 -> hospital_95. A different pair needs its own route "
            "and disruption-road selection."
        )

    requested_scenarios = {
        item.strip().upper() for item in args.scenarios.split(",") if item.strip()
    }
    valid_ids = {"S1", "S2", "S3", "S4", "S5", "S6"}
    invalid_ids = requested_scenarios - valid_ids
    if invalid_ids or not requested_scenarios:
        raise ValueError(
            "Only S1 through S6 can be run by this second-pair script. "
            "Invalid IDs: " + ", ".join(sorted(invalid_ids))
        )

    if args.edge_metric_interval <= 0:
        raise ValueError("--edge-metric-interval must be greater than zero.")
    EDGE_METRIC_SAMPLE_INTERVAL_S = float(args.edge_metric_interval)
    SEED = int(args.seed)

    tag = args.output_tag.strip().replace("\\", "_").replace("/", "_")
    if not tag or tag in (".", ".."):
        raise ValueError("Invalid --output-tag.")
    OUTPUT = BASE / "outputs" / tag
    if OUTPUT.exists() and any(OUTPUT.iterdir()) and not args.overwrite:
        raise FileExistsError(
            f"Output folder already contains files: {OUTPUT}. "
            "Choose a new --output-tag or pass --overwrite to replace it."
        )
    OUTPUT.mkdir(parents=True, exist_ok=True)

    if args.backend == "libsumo":
        if traci_libsumo is None:
            raise RuntimeError(
                "libsumo is not installed. Install the version matching SUMO 1.28.0 "
                "or use --backend traci."
            )
        traci = traci_libsumo
    else:
        traci = traci_socket

    print("Loading second-pair network and fixed disruption configuration...")
    print(f"Origin: {ORIGIN} | Destination: {DESTINATION}")
    print(f"Requested scenarios: {', '.join(sorted(requested_scenarios))}")
    print(f"Backend: {'libsumo' if traci is traci_libsumo else 'traci'}")
    print(f"Edge metric sample interval: {EDGE_METRIC_SAMPLE_INTERVAL_S:g}s")
    print("All selected scenarios will be run fresh for Route 2.")
    print("Ordinary traffic will NOT be rerouted, teleported, or optimized.")
    print("Ambulance reroutes immediately after closure activation and avoids observed blockage edges.")
    print("Any ambulance lane-change request must preserve the next route-edge connection.")

    _, edge_data = load_network_graph()
    transition_graph = load_edge_transition_graph()
    lane_transition_map = load_lane_transition_map()

    # Validate only what matters to this experiment: the ambulance's
    # reference route and the selected disruption edges. Do not inspect,
    # reroute, or guarantee background-traffic OD reachability.
    missing_route_edges = [
        edge for edge in PAIR2_REFERENCE_ROUTE if edge not in edge_data
    ]
    if missing_route_edges:
        raise RuntimeError(
            "Ambulance reference route contains missing SUMO edges: "
            + ", ".join(missing_route_edges)
        )
    disconnected_pairs = [
        (first, second)
        for first, second in zip(PAIR2_REFERENCE_ROUTE, PAIR2_REFERENCE_ROUTE[1:])
        if not transition_graph.has_edge(first, second)
    ]
    if disconnected_pairs:
        raise RuntimeError(
            "Ambulance reference route contains disconnected edge pairs: "
            + repr(disconnected_pairs)
        )
    if edge_data[PAIR2_REFERENCE_ROUTE[0]]["from"] != ORIGIN:
        raise RuntimeError("Ambulance reference route does not start at node 4.")
    if edge_data[PAIR2_REFERENCE_ROUTE[-1]]["to"] != DESTINATION:
        raise RuntimeError("Ambulance reference route does not end at hospital_95.")
    selected_disruption_edges = (
        PAIR2_ROUTE_CLOSURES + PAIR2_NEARBY_CLOSURES + PAIR2_CONGESTION_EDGES
    )
    missing_disruption_edges = [
        edge for edge in selected_disruption_edges if edge not in edge_data
    ]
    if missing_disruption_edges:
        raise RuntimeError(
            "Selected disruption edges are missing from the SUMO network: "
            + ", ".join(missing_disruption_edges)
        )
    reference_route_length_m = sum(
        float(edge_data[edge]["length"]) for edge in PAIR2_REFERENCE_ROUTE
    )
    print(
        f"Ambulance reference route validated: {len(PAIR2_REFERENCE_ROUTE)} edges, "
        f"{reference_route_length_m:.2f} m"
    )
    for detour_scenario in ("S5", "S6"):
        detour = validate_ambulance_detour(
            detour_scenario, edge_data, transition_graph, lane_transition_map
        )
        print(
            f"{detour_scenario} detour prevalidated: {len(detour)} edges; "
            "all edge-to-edge lane connections exist."
        )

    for scenario in SCENARIOS:
        if scenario["id"] in requested_scenarios:
            demand_path = DEMAND_DIR / scenario["demand"]
            if not demand_path.exists():
                raise FileNotFoundError(
                    f"Missing demand file for {scenario['id']}: {demand_path}"
                )

    selections = {
        "route_closures": list(PAIR2_ROUTE_CLOSURES),
        "nearby_closures": list(PAIR2_NEARBY_CLOSURES),
        "all_closures": list(PAIR2_ALL_CLOSURES),
        "congestion_edges": list(PAIR2_CONGESTION_EDGES),
    }

    print("\nSELECTED DISRUPTION ROADS FOR 4 -> hospital_95")
    print("Route closures:", selections["route_closures"])
    print("Nearby closures:", selections["nearby_closures"])
    print("All closures:", selections["all_closures"])
    print("Congestion corridor:", selections["congestion_edges"])

    selection_rows = []
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
                if role == "localized_congestion" else original_speed
            )
            selection_rows.append({
                "edge_id": edge_id,
                "role": role,
                "from_node": data["from"],
                "to_node": data["to"],
                "length_m": data["length"],
                "original_speed_limit_mps": original_speed,
                "applied_speed_limit_mps": applied_speed,
            })
    write_csv(OUTPUT / "disruption_edge_selection.csv", selection_rows)

    config = {
        "origin": ORIGIN,
        "destination": DESTINATION,
        "reference_route_edges": PAIR2_REFERENCE_ROUTE,
        "reference_route_length_m": reference_route_length_m,
        "congestion_edges": PAIR2_CONGESTION_EDGES,
        "congestion_speed_factor": CONGESTION_SPEED_FACTOR,
        "route_closures": PAIR2_ROUTE_CLOSURES,
        "nearby_closures": PAIR2_NEARBY_CLOSURES,
        "all_closures": PAIR2_ALL_CLOSURES,
        "ordinary_traffic_rerouting": False,
        "ordinary_traffic_completion_required": False,
        "ambulance_only_safe_lane_change_overtaking": True,
        "lane_changes_require_connection_to_next_route_edge": True,
        "known_ambulance_blockage_edges_avoided_after_disruption": list(AMBULANCE_KNOWN_BLOCKAGE_EDGES),
        "closure_reroute_policy": "immediate_after_disruption_activation",
        "ambulance_lane_change_mode_during_pass": 256,
        "ambulance_lane_change_strategy": "TraCI request with SUMO immediate-collision avoidance; only lanes connected to the next route edge are eligible",
        "ordinary_traffic_rerouting_or_teleporting": False,
        "virtual_stop_line": VIRTUAL_STOP,
        "seed": SEED,
        "step_length_s": STEP_LENGTH,
        "disruption_activation_delay_s": DISRUPTION_DELAY,
    }
    with (OUTPUT / "pair_configuration.json").open("w", encoding="utf-8") as f:
        json.dump(config, f, indent=2)

    # This is the first full Route 2 experiment. Run S1-S3 from the same
    # fixed route and demand files; never import Route 1 summaries.
    summaries = []
    selected_scenarios = [s for s in SCENARIOS if s["id"] in requested_scenarios]
    for scenario in selected_scenarios:
        summary = run_scenario(scenario, selections, edge_data, transition_graph, lane_transition_map)
        summaries.append(summary)

    summaries.sort(key=lambda row: row.get("scenario_id", ""))
    write_csv(OUTPUT / "all_scenarios_summary.csv", summaries)

    s2_summary = next((r for r in summaries if r.get("scenario_id") == "S2"), None)
    s2_time = s2_summary.get("ambulance_travel_time_s", "") if s2_summary else ""
    comparison_rows = []
    for row in summaries:
        travel_time = row.get("ambulance_travel_time_s", "")
        delta = percent = ""
        if s2_time not in ("", None) and travel_time not in ("", None):
            delta = float(travel_time) - float(s2_time)
            if float(s2_time) > 0:
                percent = 100.0 * delta / float(s2_time)
        comparison_rows.append({
            "scenario_id": row.get("scenario_id", ""),
            "scenario": row.get("scenario", ""),
            "origin": row.get("origin", ORIGIN),
            "destination": row.get("destination", DESTINATION),
            "ambulance_completed": row.get("ambulance_completed", ""),
            "ambulance_travel_time_s": travel_time,
            "delta_vs_S2_s": delta,
            "percent_change_vs_S2": percent,
            "route_update_count": row.get("route_update_count", 0),
            "ambulance_collision_count": row.get("ambulance_collision_count", 0),
        })
    write_csv(OUTPUT / "scenario_0_ambulance_comparison.csv", comparison_rows)

    print("\n" + "=" * 60)
    print("SECOND PAIR S1-S6 BATCH COMPLETE")
    print(f"Output folder: {OUTPUT}")
    print("All six Route 2 scenarios were run fresh; Route 1 outputs were not modified.")
    print(f"Master summary: {OUTPUT / 'all_scenarios_summary.csv'}")
    print(f"Ambulance comparison: {OUTPUT / 'scenario_0_ambulance_comparison.csv'}")
    print(f"Disruption configuration: {OUTPUT / 'pair_configuration.json'}")
    for row in summaries:
        if row.get("scenario_id") in requested_scenarios:
            print(
                f"{row.get('scenario_id')}: completed={row.get('ambulance_completed')} "
                f"travel_time={row.get('ambulance_travel_time_s')}s "
                f"reroutes={row.get('route_update_count', 0)}"
            )

if __name__ == "__main__":
    main()

