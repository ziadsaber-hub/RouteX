
from pathlib import Path
import geopandas as gpd
import pandas as pd
from openpyxl.utils import get_column_letter

folder = Path(__file__).resolve().parent
output = folder / "output"

# Load the graph as edited in QGIS
gpkg = output / "dokki_graph_cleaned.gpkg"

nodes = gpd.read_file(gpkg, layer="nodes")
edges = gpd.read_file(gpkg, layer="edges")

print("Updated nodes:", len(nodes))
print("Updated directed edges:", len(edges))

# Required road attributes
columns = [
    "u", "v", "key",
    "name", "highway", "length",
    "lanes", "maxspeed", "oneway"
]

for col in columns:
    if col not in edges.columns:
        edges[col] = None

road_data = edges[columns].copy()
road_data = road_data.rename(columns={
    "length": "length_m",
    "lanes": "lanes_osm",
    "maxspeed": "maxspeed_osm"
})


road_data["speed_kph_review"] = None
road_data["free_flow_s"] = None
road_data["speed_source"] = None
road_data["notes"] = None

node_data = nodes.drop(columns="geometry", errors="ignore")

# Save workbook
xlsx_path = output / "RouteX_Road_Data.xlsx"

with pd.ExcelWriter(xlsx_path, engine="openpyxl") as writer:
    road_data.to_excel(writer, sheet_name="Edges", index=False)
    node_data.to_excel(writer, sheet_name="Nodes", index=False)

    ws = writer.sheets["Edges"]

    # Calculate free-flow seconds as speeds are filled
    for row in range(2, len(road_data) + 2):
        ws[f"K{row}"] = (
            f'=IF(OR(F{row}="",J{row}="",J{row}<=0),'
            f'"",3.6*F{row}/J{row})'
        )

    ws.freeze_panes = "A2"
    ws.auto_filter.ref = ws.dimensions

print("Excel saved to:", xlsx_path)
