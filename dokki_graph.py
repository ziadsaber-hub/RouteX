
from pathlib import Path
import networkx as nx
import osmnx as ox

# 1. Locate the exported OpenStreetMap file
folder = Path(__file__).resolve().parent
osm_file = folder / "map.osm"

# 2. Read the raw OpenStreetMap data
G = ox.graph_from_xml(
    osm_file,
    simplify=False,
    retain_all=True
)

# 3. Keep motor-vehicle road types
road_types = {
    "motorway", "motorway_link",
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
    "residential", "unclassified",
    "living_street", "service", "road"
}

for u, v, key, data in list(G.edges(keys=True, data=True)):
    highway = data.get("highway")
    types = highway if isinstance(highway, list) else [highway]

    permitted = not any(
        data.get(tag) in ("no", "private")
        for tag in ("access", "vehicle", "motor_vehicle", "motorcar")
    )

    if not any(t in road_types for t in types) or not permitted:
        G.remove_edge(u, v, key)

# 4. Remove unused nodes and disconnected fragments
G.remove_nodes_from(list(nx.isolates(G)))

G = G.subgraph(
    max(nx.weakly_connected_components(G), key=len)
).copy()

# 5. Simplify roads while preserving mapped signal nodes
G = ox.simplification.simplify_graph(
    G,
    node_attrs_include=["highway"]
)

# 6. Create an output folder inside RouteX
output_folder = folder / "output"
output_folder.mkdir(exist_ok=True)

# 7. Save the spatial graph for QGIS
ox.save_graph_geopackage(
    G,
    filepath=output_folder / "dokki_graph.gpkg",
    directed=True
)

# 8. Save the mathematical graph
ox.save_graphml(
    G,
    filepath=output_folder / "dokki_graph.graphml"
)

print("Nodes:", G.number_of_nodes())
print("Directed edges:", G.number_of_edges())
print("Files saved to:", output_folder.resolve())
print("Finished!")