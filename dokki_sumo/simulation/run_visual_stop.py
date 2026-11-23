"""RouteX Dokki dynamic traffic-light visualization.

One roadside icon is drawn per incoming controlled lane. Link indices are
merged only when their full TLS phase profiles are identical.
Run from any working directory with the project's SUMO 1.28.0 environment.
"""
from __future__ import annotations

import os
import sys
import warnings
from pathlib import Path

if "SUMO_HOME" in os.environ:
    sys.path.append(str(Path(os.environ["SUMO_HOME"]) / "tools"))

import traci

HERE = Path(__file__).resolve().parent
os.chdir(HERE)
CFG = HERE / "dokki_visual.sumocfg"

MOVEMENT_POIS = {
    "123": [
        {
            "poi_id": "visual_tls_123_e121to123k0_0",
            "indices": [
                0,
                1,
                2,
                3
            ],
            "lane_id": "e121to123k0_0",
            "edge_id": "e121to123k0"
        },
        {
            "poi_id": "visual_tls_123_e124to123k0_0",
            "indices": [
                8,
                9,
                10,
                11
            ],
            "lane_id": "e124to123k0_0",
            "edge_id": "e124to123k0"
        },
        {
            "poi_id": "visual_tls_123_e125to123k0_0",
            "indices": [
                4,
                5,
                6,
                7
            ],
            "lane_id": "e125to123k0_0",
            "edge_id": "e125to123k0"
        },
        {
            "poi_id": "visual_tls_123_e172to123k0_0",
            "indices": [
                12,
                13,
                14,
                15
            ],
            "lane_id": "e172to123k0_0",
            "edge_id": "e172to123k0"
        },
        {
            "poi_id": "visual_tls_123_e193to123k0_0",
            "indices": [
                16,
                17,
                18,
                19
            ],
            "lane_id": "e193to123k0_0",
            "edge_id": "e193to123k0"
        }
    ],
    "142": [
        {
            "poi_id": "visual_tls_142_e140to142k0_0",
            "indices": [
                15,
                16,
                17,
                18,
                19
            ],
            "lane_id": "e140to142k0_0",
            "edge_id": "e140to142k0"
        },
        {
            "poi_id": "visual_tls_142_e16to142k0_0",
            "indices": [
                0,
                1
            ],
            "lane_id": "e16to142k0_0",
            "edge_id": "e16to142k0"
        },
        {
            "poi_id": "visual_tls_142_e16to142k0_1",
            "indices": [
                2
            ],
            "lane_id": "e16to142k0_1",
            "edge_id": "e16to142k0"
        },
        {
            "poi_id": "visual_tls_142_e16to142k0_2",
            "indices": [
                3,
                4
            ],
            "lane_id": "e16to142k0_2",
            "edge_id": "e16to142k0"
        },
        {
            "poi_id": "visual_tls_142_e16to142k0_3",
            "indices": [
                5,
                6
            ],
            "lane_id": "e16to142k0_3",
            "edge_id": "e16to142k0"
        },
        {
            "poi_id": "visual_tls_142_e79to142k0_0",
            "indices": [
                11,
                12,
                13,
                14
            ],
            "lane_id": "e79to142k0_0",
            "edge_id": "e79to142k0"
        },
        {
            "poi_id": "visual_tls_142_e89to142k0_0",
            "indices": [
                7
            ],
            "lane_id": "e89to142k0_0",
            "edge_id": "e89to142k0"
        },
        {
            "poi_id": "visual_tls_142_e89to142k0_1",
            "indices": [
                8,
                9,
                10
            ],
            "lane_id": "e89to142k0_1",
            "edge_id": "e89to142k0"
        }
    ],
    "172": [
        {
            "poi_id": "visual_tls_172_e123to172k0_0",
            "indices": [
                0,
                1,
                2,
                3
            ],
            "lane_id": "e123to172k0_0",
            "edge_id": "e123to172k0"
        },
        {
            "poi_id": "visual_tls_172_e192to172k0_0",
            "indices": [
                12,
                13,
                14,
                15
            ],
            "lane_id": "e192to172k0_0",
            "edge_id": "e192to172k0"
        },
        {
            "poi_id": "visual_tls_172_e193to172k0_0",
            "indices": [
                4,
                5,
                6,
                7
            ],
            "lane_id": "e193to172k0_0",
            "edge_id": "e193to172k0"
        },
        {
            "poi_id": "visual_tls_172_e193to172k1_0",
            "indices": [
                8,
                9,
                10,
                11
            ],
            "lane_id": "e193to172k1_0",
            "edge_id": "e193to172k1"
        },
        {
            "poi_id": "visual_tls_172_e39to172k0_0",
            "indices": [
                16
            ],
            "lane_id": "e39to172k0_0",
            "edge_id": "e39to172k0"
        },
        {
            "poi_id": "visual_tls_172_e39to172k0_1",
            "indices": [
                17
            ],
            "lane_id": "e39to172k0_1",
            "edge_id": "e39to172k0"
        },
        {
            "poi_id": "visual_tls_172_e39to172k0_2",
            "indices": [
                18,
                19
            ],
            "lane_id": "e39to172k0_2",
            "edge_id": "e39to172k0"
        }
    ],
    "203": [
        {
            "poi_id": "visual_tls_203_e168to203k0_0",
            "indices": [
                2,
                3
            ],
            "lane_id": "e168to203k0_0",
            "edge_id": "e168to203k0"
        },
        {
            "poi_id": "visual_tls_203_e201to203k0_0",
            "indices": [
                0,
                1
            ],
            "lane_id": "e201to203k0_0",
            "edge_id": "e201to203k0"
        },
        {
            "poi_id": "visual_tls_203_e215to203k0_0",
            "indices": [
                4,
                5
            ],
            "lane_id": "e215to203k0_0",
            "edge_id": "e215to203k0"
        }
    ],
    "28": [
        {
            "poi_id": "visual_tls_28_e122to28k0_0",
            "indices": [
                0,
                1
            ],
            "lane_id": "e122to28k0_0",
            "edge_id": "e122to28k0"
        },
        {
            "poi_id": "visual_tls_28_e122to28k0_1",
            "indices": [
                2
            ],
            "lane_id": "e122to28k0_1",
            "edge_id": "e122to28k0"
        },
        {
            "poi_id": "visual_tls_28_e122to28k0_2",
            "indices": [
                3
            ],
            "lane_id": "e122to28k0_2",
            "edge_id": "e122to28k0"
        },
        {
            "poi_id": "visual_tls_28_e125to28k0_0",
            "indices": [
                4,
                5
            ],
            "lane_id": "e125to28k0_0",
            "edge_id": "e125to28k0"
        },
        {
            "poi_id": "visual_tls_28_e125to28k0_1",
            "indices": [
                6
            ],
            "lane_id": "e125to28k0_1",
            "edge_id": "e125to28k0"
        }
    ],
    "31": [
        {
            "poi_id": "visual_tls_31_e147to31k0_0",
            "indices": [
                0,
                1
            ],
            "lane_id": "e147to31k0_0",
            "edge_id": "e147to31k0"
        },
        {
            "poi_id": "visual_tls_31_e150to31k0_0",
            "indices": [
                2,
                3
            ],
            "lane_id": "e150to31k0_0",
            "edge_id": "e150to31k0"
        },
        {
            "poi_id": "visual_tls_31_e150to31k0_1",
            "indices": [
                4
            ],
            "lane_id": "e150to31k0_1",
            "edge_id": "e150to31k0"
        },
        {
            "poi_id": "visual_tls_31_e150to31k0_2",
            "indices": [
                5,
                6
            ],
            "lane_id": "e150to31k0_2",
            "edge_id": "e150to31k0"
        },
        {
            "poi_id": "visual_tls_31_e215to31k0_0",
            "indices": [
                11
            ],
            "lane_id": "e215to31k0_0",
            "edge_id": "e215to31k0"
        },
        {
            "poi_id": "visual_tls_31_e215to31k0_1",
            "indices": [
                12
            ],
            "lane_id": "e215to31k0_1",
            "edge_id": "e215to31k0"
        },
        {
            "poi_id": "visual_tls_31_e215to31k0_2",
            "indices": [
                13
            ],
            "lane_id": "e215to31k0_2",
            "edge_id": "e215to31k0"
        },
        {
            "poi_id": "visual_tls_31_e29to31k0_0",
            "indices": [
                7,
                8
            ],
            "lane_id": "e29to31k0_0",
            "edge_id": "e29to31k0"
        },
        {
            "poi_id": "visual_tls_31_e29to31k0_1",
            "indices": [
                9,
                10
            ],
            "lane_id": "e29to31k0_1",
            "edge_id": "e29to31k0"
        }
    ]
}

def image_for_signal(signal_char: str) -> str:
    if signal_char in ("G", "g"):
        return "icons/traffic_green.png"
    if signal_char in ("Y", "y"):
        return "icons/traffic_yellow.png"
    return "icons/traffic_red.png"

# Virtual upstream stop line for the e39 -> 172 approach.
# Position is measured from the start of e39to172k0 (metres). It is placed
# upstream of junction 172; the real TLS 172 remains untouched and continues
# to protect all junction conflicts.
VIRTUAL_STOP = {
    "tls_id": "172",
    "edge_id": "e39to172k0",
    "stop_pos": 90.0,
    "controlled_link_indices": (16, 17, 18, 19),
    "stop_duration": 1_000_000.0,
}


def update_virtual_stop_line(stopped_vehicles: set[str]) -> None:
    """Queue the target approach on red/yellow; safely release on green."""
    state = traci.trafficlight.getRedYellowGreenState(VIRTUAL_STOP["tls_id"])
    indices = VIRTUAL_STOP["controlled_link_indices"]
    approach_states = [state[i] for i in indices if i < len(state)]
    if not approach_states:
        return

    is_green = all(char in ("G", "g") for char in approach_states)
    live_ids = set(traci.vehicle.getIDList())
    stopped_vehicles.intersection_update(live_ids)

    if is_green:
        for veh_id in tuple(stopped_vehicles):
            try:
                # SUMO stop-state bit 0 indicates that the vehicle has stopped.
                if traci.vehicle.getStopState(veh_id) & 1:
                    traci.vehicle.resume(veh_id)
                    stopped_vehicles.discard(veh_id)
                # Otherwise, keep tracking it until it reaches the stop.
            except traci.exceptions.TraCIException:
                # Do not let one vehicle's release error terminate the simulation.
                continue
        return

    edge_id = VIRTUAL_STOP["edge_id"]
    stop_pos = VIRTUAL_STOP["stop_pos"]

    for veh_id in live_ids:
        if veh_id in stopped_vehicles:
            continue
        if traci.vehicle.getRoadID(veh_id) != edge_id:
            continue
        if traci.vehicle.getLanePosition(veh_id) >= stop_pos:
            continue

        lane_index = traci.vehicle.getLaneIndex(veh_id)
        traci.vehicle.setStop(
            veh_id,
            edge_id,
            pos=stop_pos,
            laneIndex=lane_index,
            duration=VIRTUAL_STOP["stop_duration"],
        )
        stopped_vehicles.add(veh_id)

traci.start(["sumo-gui", "-c", str(CFG)])
last_images: dict[str, str] = {}
warned_groups: set[str] = set()
virtual_stopped_vehicles: set[str] = set()

try:
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        update_virtual_stop_line(virtual_stopped_vehicles)
        for tls_id, groups in MOVEMENT_POIS.items():
            state = traci.trafficlight.getRedYellowGreenState(tls_id)
            for group in groups:
                indices = group["indices"]
                chars = [state[i] for i in indices if i < len(state)]
                if not chars:
                    continue
                # The source TLS program was checked to give these indices
                # identical phase profiles. Warn if a runtime program diverges.
                if len(set(chars)) > 1:
                    key = f"{tls_id}:{group['lane_id']}"
                    if key not in warned_groups:
                        warnings.warn(
                            f"TLS {tls_id} has differing runtime states for "
                            f"{group['lane_id']} indices {indices}: {chars}"
                        )
                        warned_groups.add(key)
                image_path = image_for_signal(chars[0])
                poi_id = group["poi_id"]
                if last_images.get(poi_id) != image_path:
                    if poi_id in traci.poi.getIDList(): traci.poi.setImageFile(poi_id, image_path)
                    last_images[poi_id] = image_path
finally:
    traci.close()


