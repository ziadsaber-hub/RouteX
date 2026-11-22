
"""RouteX Dokki dynamic traffic-light icons.

Displays exactly one icon per incoming controlled road approach.
The road network and traffic logic are not modified.
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
SUMO_GUI = os.environ.get("SUMO_GUI", "sumo-gui")


APPROACH_SIGNALS = {
    "123": [
        {
            "poi_id": "visual_tls_123_e121to123k0",
            "edge_id": "e121to123k0",
            "indices": [0, 1, 2, 3],
        },
        {
            "poi_id": "visual_tls_123_e124to123k0",
            "edge_id": "e124to123k0",
            "indices": [8, 9, 10, 11],
        },
        {
            "poi_id": "visual_tls_123_e125to123k0",
            "edge_id": "e125to123k0",
            "indices": [4, 5, 6, 7],
        },
        {
            "poi_id": "visual_tls_123_e172to123k0",
            "edge_id": "e172to123k0",
            "indices": [12, 13, 14, 15],
        },
        {
            "poi_id": "visual_tls_123_e193to123k0",
            "edge_id": "e193to123k0",
            "indices": [16, 17, 18, 19],
        },
    ],

    "142": [
        {
            "poi_id": "visual_tls_142_e140to142k0",
            "edge_id": "e140to142k0",
            "indices": [15, 16, 17, 18, 19],
        },
        {
            "poi_id": "visual_tls_142_e16to142k0",
            "edge_id": "e16to142k0",
            "indices": [0, 1, 2, 3, 4, 5, 6],
        },
        {
            "poi_id": "visual_tls_142_e79to142k0",
            "edge_id": "e79to142k0",
            "indices": [11, 12, 13, 14],
        },
        {
            "poi_id": "visual_tls_142_e89to142k0",
            "edge_id": "e89to142k0",
            "indices": [7, 8, 9, 10],
        },
    ],

    "172": [
        {
            "poi_id": "visual_tls_172_e123to172k0",
            "edge_id": "e123to172k0",
            "indices": [0, 1, 2, 3],
        },
        {
            "poi_id": "visual_tls_172_e192to172k0",
            "edge_id": "e192to172k0",
            "indices": [12, 13, 14, 15],
        },
        {
            "poi_id": "visual_tls_172_e193to172k0",
            "edge_id": "e193to172k0",
            "indices": [4, 5, 6, 7],
        },
        {
            "poi_id": "visual_tls_172_e193to172k1",
            "edge_id": "e193to172k1",
            "indices": [8, 9, 10, 11],
        },
        {
            "poi_id": "visual_tls_172_e39to172k0",
            "edge_id": "e39to172k0",
            "indices": [16, 17, 18, 19],
        },
    ],

    "203": [
        {
            "poi_id": "visual_tls_203_e168to203k0",
            "edge_id": "e168to203k0",
            "indices": [2, 3],
        },
        {
            "poi_id": "visual_tls_203_e201to203k0",
            "edge_id": "e201to203k0",
            "indices": [0, 1],
        },
        {
            "poi_id": "visual_tls_203_e215to203k0",
            "edge_id": "e215to203k0",
            "indices": [4, 5],
        },
    ],

    "28": [
        {
            "poi_id": "visual_tls_28_e122to28k0",
            "edge_id": "e122to28k0",
            "indices": [0, 1, 2, 3],
        },
        {
            "poi_id": "visual_tls_28_e125to28k0",
            "edge_id": "e125to28k0",
            "indices": [4, 5, 6],
        },
    ],

    "31": [
        {
            "poi_id": "visual_tls_31_e147to31k0",
            "edge_id": "e147to31k0",
            "indices": [0, 1],
        },
        {
            "poi_id": "visual_tls_31_e150to31k0",
            "edge_id": "e150to31k0",
            "indices": [2, 3, 4, 5, 6],
        },
        {
            "poi_id": "visual_tls_31_e215to31k0",
            "edge_id": "e215to31k0",
            "indices": [11, 12, 13],
        },
        {
            "poi_id": "visual_tls_31_e29to31k0",
            "edge_id": "e29to31k0",
            "indices": [7, 8, 9, 10],
        },
    ],
}


def image_for_signal(signal_char: str) -> str:
    if signal_char in ("G", "g"):
        return "icons/traffic_green.png"

    if signal_char in ("Y", "y"):
        return "icons/traffic_yellow.png"

    return "icons/traffic_red.png"


def main():
    last_images: dict[str, str] = {}
    warned: set[str] = set()
    connected = False

    if not CFG.exists():
        raise FileNotFoundError(f"SUMO config not found: {CFG}")

    print("Starting RouteX SUMO-GUI...")
    print(f"Configuration: {CFG}")

    try:
        traci.start(
            [
                SUMO_GUI,
                "-c",
                str(CFG),
                "--start",
                "--quit-on-end",
                "--no-step-log",
                "true",
            ],
            label="routex",
        )

        connected = True
        connection = traci.getConnection("routex")

        print("TraCI connected successfully.")
        print("RouteX visualization is running.")

        while connection.simulation.getMinExpectedNumber() > 0:
            connection.simulationStep()

            for tls_id, approaches in APPROACH_SIGNALS.items():

                try:
                    state = (
                        connection.trafficlight
                        .getRedYellowGreenState(tls_id)
                    )
                except Exception as exc:
                    print(
                        f"Could not read traffic light {tls_id}: {exc}"
                    )
                    continue

                for approach in approaches:
                    indices = approach["indices"]

                    chars = [
                        state[i]
                        for i in indices
                        if i < len(state)
                    ]

                    if not chars:
                        continue

                    key = f"{tls_id}:{approach['edge_id']}"

                    if len(set(chars)) != 1:
                        if key not in warned:
                            warnings.warn(
                                f"Approach {key} has different signal "
                                f"states across its lanes: {chars}. "
                                f"Displaying red."
                            )
                            warned.add(key)

                        image = "icons/traffic_red.png"

                    else:
                        image = image_for_signal(chars[0])

                    poi_id = approach["poi_id"]

                    if last_images.get(poi_id) != image:
                        connection.poi.setImageFile(poi_id, image)
                        last_images[poi_id] = image

        print("RouteX simulation finished.")

    except Exception as exc:
        print("\n========== ROUTEX ERROR ==========")
        print(f"{type(exc).__name__}: {exc}")
        print("==================================")
        raise

    finally:
        if connected:
            try:
                traci.getConnection("routex").close()
                print("TraCI connection closed.")
            except Exception as exc:
                print(f"Connection close notice: {exc}")


if __name__ == "__main__":
    main()
