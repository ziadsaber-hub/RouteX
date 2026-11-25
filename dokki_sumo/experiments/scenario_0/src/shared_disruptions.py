"""Scenario inputs shared by both route experiments."""
import json
from pathlib import Path

def apply_shared_disruptions(namespace, route):
    path = Path(__file__).with_name("shared_disruptions.json")
    settings = json.loads(path.read_text(encoding="utf-8"))
    if route == 1:
        namespace["select_disruption_roads"] = lambda *args: settings
    else:
        for field, key in (
            ("PAIR2_ROUTE_CLOSURES", "route_closures"),
            ("PAIR2_NEARBY_CLOSURES", "nearby_closures"),
            ("PAIR2_ALL_CLOSURES", "all_closures"),
            ("PAIR2_CONGESTION_EDGES", "congestion_edges"),
        ):
            namespace[field] = settings[key]
