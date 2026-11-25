"""Official seed-1 RouteX disruption fixtures, independent of routing policy."""
import json
from pathlib import Path

def apply_official_disruptions(namespace, route):
    settings = json.loads(Path(__file__).with_name("official_disruptions.json").read_text(encoding="utf-8"))[str(route)]
    for key in ("all_closures", "congestion_edges"):
        if len(set(settings[key])) != 4:
            raise ValueError(f"Route {route} needs four distinct {key}")
    if set(settings["all_closures"]) & set(settings["congestion_edges"]):
        raise ValueError(f"Route {route} has overlapping closures and slowdowns")
    namespace["select_disruption_roads"] = lambda *args, **kwargs: settings
    if str(route) == "2":
        for key in ("route_closures", "nearby_closures", "all_closures", "congestion_edges"):
            namespace["PAIR2_" + key.upper()] = settings[key]
