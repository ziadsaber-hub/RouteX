"""RouteX - build a SUMO network from the Model A Excel (same IDs as Model C).

Run from inside model_c/ :   python build_sumo_net.py
Needs: RouteX_Road_Data_completed.xlsx next to this file, and SUMO's netconvert on PATH.
Road shapes (curves) come from dokki_graph_cleaned.gpkg if it is found (next to this file,
in ../output/, or in ./output/); otherwise roads are drawn as straight lines.
Writes into sumo/ : dokki.nod.xml, dokki.edg.xml, dokki.net.xml

ID rule (so Model C and SUMO always agree):
  SUMO junction id = node id (osmid column)
  SUMO edge id     = "u-v-k"  (exactly Model C's edge ID)
Each Excel row is ONE directed edge, so every row becomes one SUMO edge.
"""
import math
import sqlite3
import struct
import subprocess
from pathlib import Path
import pandas as pd

HERE = Path(__file__).resolve().parent
XLSX = HERE / "RouteX_Road_Data_completed.xlsx"
OUT = HERE / "sumo"
OUT.mkdir(exist_ok=True)

nodes = pd.read_excel(XLSX, sheet_name="Nodes")
edges = pd.read_excel(XLSX, sheet_name="Edges")

# lon/lat -> metres (local flat projection; fine for a ~1.5 km area).
lat0, lon0 = nodes["y"].mean(), nodes["x"].mean()
M_LAT = 111_320.0
M_LON = 111_320.0 * math.cos(math.radians(lat0))
xy = {int(r.osmid): ((r.x - lon0) * M_LON, (r.y - lat0) * M_LAT) for r in nodes.itertuples()}



def find_gpkg():
    for p in (HERE / "dokki_graph_cleaned.gpkg", HERE.parent / "output" / "dokki_graph_cleaned.gpkg",
              HERE / "output" / "dokki_graph_cleaned.gpkg"):
        if p.exists():
            return p
    return None


def load_shapes(path):
    """{(u,v,key): [(lon,lat), ...]} read straight from the GeoPackage (no extra libraries)."""
    con = sqlite3.connect(path)
    shapes = {}
    for u, v, k, blob in con.execute("select u, v, key, geom from edges"):
        env = {0: 0, 1: 32, 2: 48, 3: 48, 4: 64}[(blob[3] >> 1) & 7]
        w = blob[8 + env:]
        bo = "<" if w[0] == 1 else ">"
        if struct.unpack(bo + "I", w[1:5])[0] != 2:      # only LineString
            continue
        n = struct.unpack(bo + "I", w[5:9])[0]
        shapes[(int(u), int(v), int(k))] = [struct.unpack(bo + "dd", w[9 + 16 * i:25 + 16 * i]) for i in range(n)]
    con.close()
    return shapes


GPKG = find_gpkg()
SHAPES = load_shapes(GPKG) if GPKG else {}
print(f"shapes: {len(SHAPES)} loaded from {GPKG}" if GPKG else "shapes: gpkg not found -> straight lines")

PRIORITY = {"motorway": 9, "trunk": 8, "primary": 7, "secondary": 6,
            "tertiary": 5, "unclassified": 4, "residential": 3, "living_street": 2, "service": 1}


def main_type(h):
    """highway can be 'primary' or a list text like "['primary', 'primary_link']"; use the first."""
    h = str(h).strip("[]").split(",")[0].strip(" '\"")
    return h.replace("_link", "")


used = set(edges["u"]) | set(edges["v"])
with open(OUT / "dokki.nod.xml", "w", encoding="utf-8") as f:
    f.write('<nodes>\n')
    for n in sorted(used):
        x, y = xy[int(n)]
        f.write(f'  <node id="{int(n)}" x="{x:.2f}" y="{y:.2f}"/>\n')
    f.write('</nodes>\n')

with open(OUT / "dokki.edg.xml", "w", encoding="utf-8") as f:
    f.write('<edges>\n')
    for r in edges.itertuples():
        speed = r.speed_kph_review if pd.notna(r.speed_kph_review) else r.maxspeed_osm
        pr = PRIORITY.get(main_type(r.highway), 3)
        pts = SHAPES.get((int(r.u), int(r.v), int(r.key)))
        shape = ""
        if pts and len(pts) > 2:                      # keep curves; 2-point roads are straight anyway
            shape = ' shape="' + " ".join(f"{(lo - lon0) * M_LON:.2f},{(la - lat0) * M_LAT:.2f}" for lo, la in pts) + '"'
        f.write(f'  <edge id="{int(r.u)}-{int(r.v)}-{int(r.key)}" from="{int(r.u)}" to="{int(r.v)}" '
                f'numLanes="{int(r.lanes_osm)}" speed="{float(speed) / 3.6:.3f}" '
                f'priority="{pr}" length="{float(r.length_m):.2f}"{shape}/>\n')
    f.write('</edges>\n')

print(f"wrote {len(used)} nodes and {len(edges)} edges to {OUT}")

cmd = ["netconvert", "--node-files", str(OUT / "dokki.nod.xml"),
       "--edge-files", str(OUT / "dokki.edg.xml"),
       "-o", str(OUT / "dokki.net.xml")]
print("running:", " ".join(cmd))
subprocess.run(cmd, check=True)
print("done ->", OUT / "dokki.net.xml")
