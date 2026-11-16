"""RouteX - build a SUMO network straight from the real OpenStreetMap data (map.osm),
then map every Model C edge (u-v-k) onto the real SUMO edges.

Run from inside model_c/ :   python build_sumo_net_osm.py
Needs: map.osm  (next to this file, or in ../output/ or ../),  dokki_graph_cleaned.gpkg
       (next to this file, ../output/ or ./output/),  RouteX_Road_Data_completed.xlsx,
       SUMO (netconvert) on PATH, and sumolib (already in requirements.txt).

Writes into sumo/ :
  dokki_osm.net.xml   the real street network (real geometry, real junction shapes, OSM signals)
  edge_map.csv        model_c_edge_id -> the SUMO edge ids that follow the same road
  edge_map_report.txt quality check (how many edges matched, which ones look wrong)
Model C is NOT changed: it keeps routing on u-v-k; edge_map.csv translates a route for SUMO.
"""
import csv
import math
import sqlite3
import struct
import subprocess
from pathlib import Path

import pandas as pd
import sumolib
from sumolib.route import mapTrace

HERE = Path(__file__).resolve().parent
OUT = HERE / "sumo"
OUT.mkdir(exist_ok=True)


def first_existing(name, extra=()):
    for d in (HERE, HERE.parent / "output", HERE.parent, HERE / "output", *extra):
        if (d / name).exists():
            return d / name
    raise FileNotFoundError(f"{name} not found (looked next to this file, ./output, ../output, ..)")


OSM = first_existing("map.osm")
GPKG = first_existing("dokki_graph_cleaned.gpkg")
XLSX = first_existing("RouteX_Road_Data_completed.xlsx")
NET = OUT / "dokki_osm.net.xml"

# ---------- 1) real OSM -> SUMO (cars only, like the Model A filter) ----------
cmd = ["netconvert", "--osm-files", str(OSM), "-o", str(NET),
       "--keep-edges.by-vclass", "passenger",
       "--geometry.remove", "--roundabouts.guess", "--junctions.join",
       "--tls.guess-signals", "--tls.discard-simple",
       "--output.street-names", "--no-warnings"]
print("running:", " ".join(cmd))
subprocess.run(cmd, check=True)
net = sumolib.net.readNet(str(NET))
print(f"SUMO network: {len(net.getEdges())} edges, {len(net.getNodes())} junctions")


# ---------- 2) Model C edge shapes (lon/lat) from the GeoPackage ----------
def load_shapes(path):
    con = sqlite3.connect(path)
    shapes = {}
    for u, v, k, blob in con.execute("select u, v, key, geom from edges"):
        env = {0: 0, 1: 32, 2: 48, 3: 48, 4: 64}[(blob[3] >> 1) & 7]
        w = blob[8 + env:]
        bo = "<" if w[0] == 1 else ">"
        n = struct.unpack(bo + "I", w[5:9])[0]
        shapes[(int(u), int(v), int(k))] = [struct.unpack(bo + "dd", w[9 + 16 * i:25 + 16 * i]) for i in range(n)]
    con.close()
    return shapes


def densify(pts, step=8.0):
    """add a point every ~8 m so map matching follows the road, not just the two ends."""
    out = [pts[0]]
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) // step))
        out += [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(1, n + 1)]
    return out


shapes = load_shapes(GPKG)
edges = pd.read_excel(XLSX, sheet_name="Edges")

# ---------- 3) map every Model C edge onto SUMO edges ----------
rows, bad = [], []
for r in edges.itertuples():
    eid = (int(r.u), int(r.v), int(r.key))
    name = f"{eid[0]}-{eid[1]}-{eid[2]}"
    xy = densify([net.convertLonLat2XY(lon, lat) for lon, lat in shapes[eid]])
    try:
        path = mapTrace(xy, net, delta=25.0, fillGaps=2)
    except Exception:
        path = []
    ids = [e.getID() for e in path]
    sumo_len = sum(e.getLength() for e in path)
    ratio = sumo_len / float(r.length_m) if r.length_m else 0
    ok = bool(ids) and 0.7 <= ratio <= 1.4
    rows.append([name, " ".join(ids), len(ids), round(float(r.length_m), 1), round(sumo_len, 1), round(ratio, 2), int(ok)])
    if not ok:
        bad.append((name, len(ids), round(ratio, 2)))

with open(OUT / "edge_map.csv", "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f)
    w.writerow(["model_c_edge", "sumo_edges", "n_sumo_edges", "model_c_length_m", "sumo_length_m", "length_ratio", "ok"])
    w.writerows(rows)

good = len(rows) - len(bad)
with open(OUT / "edge_map_report.txt", "w", encoding="utf-8") as f:
    f.write(f"matched OK: {good} / {len(rows)}\n")
    f.write("edges to review (name, #sumo edges, length ratio):\n")
    for b in bad:
        f.write(f"  {b}\n")
print(f"mapping: {good}/{len(rows)} Model C edges matched a SUMO path of similar length")
print("details: sumo/edge_map.csv and sumo/edge_map_report.txt")
