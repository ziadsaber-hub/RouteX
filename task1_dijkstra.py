"""ROUTEX - C Model - Task 1 (Ziad)
Build the road graph from the A Model Excel file, compute free-flow time,
and run Dijkstra as the baseline.  Time unit everywhere = seconds."""
import ast
import heapq
import math
import pandas as pd
import networkx as nx

import os
FILE = os.environ.get("ROUTEX_FILE", "RouteX_Road_Data_completed.xlsx")   # set ROUTEX_FILE or put the xlsx next to the scripts


# ---------- 1) read + clean ----------
def first_value(x):
    """Some cells hold lists like "['primary','primary_link']" -> take first."""
    if isinstance(x, str) and x.startswith("["):
        return ast.literal_eval(x)[0]
    return x


edges = pd.read_excel(FILE, sheet_name="Edges")
nodes = pd.read_excel(FILE, sheet_name="Nodes")
edges["highway"] = edges["highway"].map(first_value)

# ---------- 2) free-flow time (mu_e) ----------
# seconds = metres / (km/h / 3.6)
edges["free_flow_s"] = edges["length_m"] / (edges["maxspeed_osm"] / 3.6)

# ---------- 3) build directed graph ----------
# adj[u][v] = (time_s, length_m)  ; parallel edges -> keep the fastest
adj = {int(n): {} for n in nodes["osmid"]}


def add(u, v, t, L):
    if v not in adj[u] or t < adj[u][v][0]:
        adj[u][v] = (t, L)


for r in edges.itertuples():
    u, v, t, L = int(r.u), int(r.v), r.free_flow_s, r.length_m
    add(u, v, t, L)
    if not r.oneway:          # two-way road -> add the reverse direction
        add(v, u, t, L)


# ---------- 4) Dijkstra (our own implementation) ----------
def dijkstra(src, dst, weight="time"):
    """weight = 'time' (index 0) or 'length' (index 1)."""
    w = 0 if weight == "time" else 1
    dist = {src: 0.0}
    prev = {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, math.inf):
            continue                      # stale entry
        if u == dst:
            break
        for v, attrs in adj[u].items():
            nd = d + attrs[w]
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None, math.inf
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1], dist[dst]


def path_stats(path):
    t = sum(adj[a][b][0] for a, b in zip(path, path[1:]))
    L = sum(adj[a][b][1] for a, b in zip(path, path[1:]))
    return t, L


# ---------- 5) strongly connected core (safe start/end nodes) ----------
G = nx.DiGraph()
for u in adj:
    for v, (t, L) in adj[u].items():
        G.add_edge(u, v, time=t, length=L)
core = max(nx.strongly_connected_components(G), key=len)
print(f"nodes={len(adj)}  directed edges={G.number_of_edges()}  "
      f"safe core={len(core)} nodes")

if __name__ == "__main__":
    # pick two far-apart nodes inside the core (by coordinates)
    pos = nodes.set_index("osmid")[["y", "x"]]
    core_pos = pos.loc[list(core)]
    src = int(core_pos["x"].idxmin())     # western-most
    dst = int(core_pos["x"].idxmax())     # eastern-most

    p_time, t_time = dijkstra(src, dst, "time")
    p_len, _ = dijkstra(src, dst, "length")
    t_of_short, L_of_short = path_stats(p_len)
    t_of_fast, L_of_fast = path_stats(p_time)

    print(f"\nfrom node {src} to node {dst}")
    print(f"FASTEST  : {len(p_time)-1} edges, {t_of_fast:.0f} s, {L_of_fast:.0f} m")
    print(f"SHORTEST : {len(p_len)-1} edges, {t_of_short:.0f} s, {L_of_short:.0f} m")

    # sanity check against networkx
    nx_t = nx.dijkstra_path_length(G, src, dst, weight="time")
    print(f"\ncheck vs networkx: ours={t_time:.3f}  nx={nx_t:.3f}  "
          f"{'OK' if abs(t_time-nx_t) < 1e-6 else 'MISMATCH'}")

    # test many random pairs: how often is shortest != fastest?
    import random
    random.seed(0)
    ids = list(core)
    diff, ok, n = 0, 0, 200
    for _ in range(n):
        a, b = random.sample(ids, 2)
        pt, tt = dijkstra(a, b, "time")
        pl, _ = dijkstra(a, b, "length")
        ok += abs(tt - nx.dijkstra_path_length(G, a, b, weight="time")) < 1e-6
        diff += pt != pl
    print(f"{n} random pairs: matches networkx in {ok}/{n}; "
          f"shortest route != fastest route in {diff}/{n}")
