"""ROUTEX - C Model - Task 1: Dijkstra on Model B travel times (the only routing algorithm).

Equation (10):   P*(t) = argmin_{P in P(s,g)}  sum_{e in P} mu_hat_e(t)
mu_hat_e(t) = estimated travel time of edge e in the selected interval, received from Model B.
Time unit everywhere = seconds.

Edge identity
  Every directed edge is kept separately under its full ID  eid = (u, v, k)  exactly as in the Model A
  Edges sheet. Parallel roads between the same two intersections are NOT merged.
  A route is a list of edge IDs (not a list of nodes):  [(u1,v1,k1), (u2,v2,k2), ...]
  Dijkstra stores the chosen predecessor EDGE for every node, so the exact road is rebuilt.

Model B interface (pick one)
  a) file : set ROUTEX_MODEL_B=model_b_times.csv  (columns: u, v, key, mu_hat_s  [, interval])
            and, if the file has several intervals, ROUTEX_INTERVAL=<value of the interval column>
  b) code : from task1_dijkstra import set_travel_times;  set_travel_times({(u,v,k): seconds, ...})
  Without Model B the free-flow time of Model A is used (TEST MODE, printed as a warning).
"""
import heapq
import math
import os
import pandas as pd
import networkx as nx

FILE = os.environ.get("ROUTEX_FILE", "RouteX_Road_Data_completed.xlsx")
MODEL_B_FILE = os.environ.get("ROUTEX_MODEL_B", "")
INTERVAL = os.environ.get("ROUTEX_INTERVAL", "")
INF = math.inf

# ---------- 1) read Model A ----------
edges = pd.read_excel(FILE, sheet_name="Edges")
nodes = pd.read_excel(FILE, sheet_name="Nodes")
edges["free_flow_s"] = edges["length_m"] / (edges["maxspeed_osm"] / 3.6)   # Model A free-flow (reference only)

# ---------- 2) directed keyed edges ----------
E = {}                                        # eid -> dict(length, ff, mu, derived)
out = {int(n): [] for n in nodes["osmid"]}    # node -> [eid leaving it]


def _add(eid, length, ff, derived):
    if eid in E:
        raise ValueError(f"duplicate edge ID {eid}")
    E[eid] = dict(length=float(length), ff=float(ff), mu=float(ff), derived=derived)
    out.setdefault(eid[0], []).append(eid)


_rows = list(edges.itertuples())
_pairs = {(int(r.u), int(r.v)) for r in _rows}
N_DERIVED = 0
for r in _rows:
    u, v, k = int(r.u), int(r.v), int(r.key)
    _add((u, v, k), r.length_m, r.free_flow_s, False)
    # The sheet normally has a row for each direction of a two-way street. Only if the reverse row is
    # missing do we create it (flagged derived=True). N_DERIVED should be 0 for the Dokki data.
    if not r.oneway and (v, u) not in _pairs and (v, u, k) not in E:
        _add((v, u, k), r.length_m, r.free_flow_s, True)
        N_DERIVED += 1


# ---------- 3) Model B travel times ----------
def load_model_b(path, interval=None):
    df = pd.read_csv(path)
    if interval not in (None, "") and "interval" in df.columns:
        df = df[df["interval"].astype(str) == str(interval)]
    if df.empty:
        raise ValueError("no Model B rows for this interval")
    return {(int(r.u), int(r.v), int(r.key)): float(r.mu_hat_s) for r in df.itertuples()}


MISSING_B = 0
MODEL_B_SOURCE = "TEST MODE: Model A free-flow times (no Model B data loaded)"


def set_travel_times(mu, source="Model B (code)"):
    """Install Model B estimates {(u,v,k): seconds}. Edges Model B did not send keep free-flow (counted)."""
    global MISSING_B, MODEL_B_SOURCE
    unknown = [e for e in mu if e not in E]
    if unknown:
        raise KeyError(f"Model B sent {len(unknown)} edge IDs that are not in Model A, e.g. {unknown[0]}")
    bad = [e for e, t in mu.items() if not (t > 0 and t < INF)]
    if bad:
        raise ValueError(f"non-positive or infinite Model B time on {bad[0]}")
    for eid, d in E.items():
        d["mu"] = float(mu[eid]) if eid in mu else d["ff"]
    MISSING_B = sum(1 for e in E if e not in mu)
    MODEL_B_SOURCE = f"{source}: {len(mu)} edges, {MISSING_B} fell back to free-flow"


if MODEL_B_FILE:
    set_travel_times(load_model_b(MODEL_B_FILE, INTERVAL or None), source=f"Model B file {MODEL_B_FILE}")


# ---------- 4) Dijkstra on edges ----------
def dijkstra_edges(src, dst, cost):
    """cost(eid) -> seconds (INF = blocked). Returns (route as list of eids, total cost); (None, INF) if unreachable."""
    dist, pred = {src: 0.0}, {}                 # pred[node] = the EDGE used to reach node
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, INF):
            continue                            # stale entry
        if u == dst:
            break
        for eid in out[u]:
            c = cost(eid)
            if c == INF:
                continue
            v = eid[1]
            nd = d + c
            if nd < dist.get(v, INF):
                dist[v], pred[v] = nd, eid
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None, INF
    route, n = [], dst
    while n != src:
        eid = pred[n]
        route.append(eid)
        n = eid[0]
    return route[::-1], dist[dst]


def dijkstra(src, dst, weight="time"):
    """weight 'time' = Model B mu_hat (equation 10), 'length' = metres."""
    if weight == "time":
        return dijkstra_edges(src, dst, lambda e: E[e]["mu"])
    return dijkstra_edges(src, dst, lambda e: E[e]["length"])


# ---------- 5) route helpers ----------
def route_time(route):
    return sum(E[e]["mu"] for e in route)


def route_length(route):
    return sum(E[e]["length"] for e in route)


def route_nodes(route):
    return [route[0][0]] + [e[1] for e in route]


def route_ids(route):
    """Ordered edge IDs as text 'u-v-k' (what Model D receives)."""
    return [f"{u}-{v}-{k}" for u, v, k in route]


def is_chain(route):
    return all(a[1] == b[0] for a, b in zip(route, route[1:]))


# ---------- 6) strongly connected core (safe start/end nodes) ----------
MG = nx.MultiDiGraph()
for _e, _d in E.items():
    MG.add_edge(_e[0], _e[1], key=_e[2], length=_d["length"])
core = max(nx.strongly_connected_components(MG), key=len)


def _mg_time():
    """networkx copy of the graph with the current Model B times (only used for checks)."""
    g = nx.MultiDiGraph()
    for e, d in E.items():
        g.add_edge(e[0], e[1], key=e[2], time=d["mu"])
    return g


if __name__ == "__main__":
    print(MODEL_B_SOURCE)
    print(f"nodes={len(out)}  directed keyed edges={len(E)}  derived reverse edges={N_DERIVED}  "
          f"parallel (u,v) pairs={sum(1 for u in out for v in {e[1] for e in out[u]} if sum(1 for e in out[u] if e[1] == v) > 1)}  "
          f"safe core={len(core)} nodes")

    pos = nodes.set_index("osmid")[["y", "x"]]
    core_pos = pos.loc[list(core)]
    src = int(core_pos["x"].idxmin())     # western-most
    dst = int(core_pos["x"].idxmax())     # eastern-most
    p_time, t_time = dijkstra(src, dst, "time")
    p_len, _ = dijkstra(src, dst, "length")
    print(f"\nfrom node {src} to node {dst}")
    print(f"FASTEST  : {len(p_time)} edges, {route_time(p_time):.0f} s, {route_length(p_time):.0f} m")
    print(f"SHORTEST : {len(p_len)} edges, {route_time(p_len):.0f} s, {route_length(p_len):.0f} m")
    print("fastest route edge IDs (first 5):", route_ids(p_time)[:5])
    assert is_chain(p_time) and p_time[0][0] == src and p_time[-1][1] == dst

    g = _mg_time()
    nx_t = nx.dijkstra_path_length(g, src, dst, weight="time")
    print(f"\ncheck vs networkx: ours={t_time:.3f}  nx={nx_t:.3f}  {'OK' if abs(t_time - nx_t) < 1e-6 else 'MISMATCH'}")

    import random
    random.seed(0)
    ids = sorted(core)
    diff, ok, n = 0, 0, 200
    for _ in range(n):
        a, b = random.sample(ids, 2)
        pt, tt = dijkstra(a, b, "time")
        pl, _ = dijkstra(a, b, "length")
        ok += abs(tt - nx.dijkstra_path_length(g, a, b, weight="time")) < 1e-6 and is_chain(pt)
        diff += pt != pl
    print(f"{n} random pairs: matches networkx in {ok}/{n}; shortest route != fastest route in {diff}/{n}")
