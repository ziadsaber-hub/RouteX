"""ROUTEX - C Model - Task 1 part 2: A* and comparison with Dijkstra."""
import heapq
import math
import random
import time
import networkx as nx
from task1_dijkstra import adj, nodes, core, G, edges

# ---------- coordinates + heuristic ----------
pos = {int(r.osmid): (r.y, r.x) for r in nodes.itertuples()}
V_MAX = edges["maxspeed_osm"].max() / 3.6      # fastest speed in network, m/s


def haversine_m(a, b):
    (la1, lo1), (la2, lo2) = pos[a], pos[b]
    p1, p2 = math.radians(la1), math.radians(la2)
    dphi, dl = p2 - p1, math.radians(lo2 - lo1)
    h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * 6371000 * math.asin(math.sqrt(h))


def h(n, dst):
    """Straight-line distance / fastest speed = best-case time left.
    Never overestimates -> admissible -> A* stays optimal.
    Note: a few edges are slightly shorter than the straight line between their
    ends (by at most 0.3%), because of small differences in how distance is
    computed. We multiply by 0.99 as a small safety margin so the heuristic
    never exceeds the true travel time."""
    return 0.99 * haversine_m(n, dst) / V_MAX      # 0.99 = safety margin (see note)


# ---------- both searches, with a counter of expanded nodes ----------
def search(src, dst, use_heuristic):
    g = {src: 0.0}
    prev = {}
    pq = [(h(src, dst) if use_heuristic else 0.0, src)]
    closed, expanded = set(), 0
    while pq:
        _, u = heapq.heappop(pq)
        if u in closed:
            continue
        closed.add(u)
        expanded += 1
        if u == dst:
            break
        for v, (t, _L) in adj[u].items():
            ng = g[u] + t
            if ng < g.get(v, math.inf):
                g[v] = ng
                prev[v] = u
                heapq.heappush(pq, (ng + (h(v, dst) if use_heuristic else 0.0), v))
    if dst not in g:
        return None, math.inf, expanded
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1], g[dst], expanded


dijkstra_t = lambda s, d: search(s, d, False)
astar = lambda s, d: search(s, d, True)

if __name__ == "__main__":
    # 1) is the heuristic really admissible? check every edge
    bad = sum(h(u, v) > t + 1e-9
              for u in adj for v, (t, _) in adj[u].items())
    print(f"edges where heuristic overestimates: {bad} of {G.number_of_edges()}")

    # 2) compare on random pairs
    random.seed(1)
    ids = sorted(core)
    pairs = [tuple(random.sample(ids, 2)) for _ in range(300)]
    stats = {"dij": [0, 0.0], "ast": [0, 0.0]}      # [expanded, seconds]
    same_cost, optimal = 0, 0
    for s, d in pairs:
        t0 = time.perf_counter(); _, c1, e1 = dijkstra_t(s, d)
        t1 = time.perf_counter(); _, c2, e2 = astar(s, d)
        t2 = time.perf_counter()
        stats["dij"][0] += e1; stats["dij"][1] += t1 - t0
        stats["ast"][0] += e2; stats["ast"][1] += t2 - t1
        same_cost += abs(c1 - c2) < 1e-9
        optimal += abs(c2 - nx.dijkstra_path_length(G, s, d, weight="time")) < 1e-6

    n = len(pairs)
    print(f"\n{n} random pairs")
    print(f"A* cost == Dijkstra cost : {same_cost}/{n}")
    print(f"A* cost == networkx      : {optimal}/{n}")
    print(f"avg nodes expanded  Dijkstra={stats['dij'][0]/n:.1f}   A*={stats['ast'][0]/n:.1f}"
          f"   (A* explores {100*(1-stats['ast'][0]/stats['dij'][0]):.0f}% fewer)")
    print(f"avg runtime         Dijkstra={1000*stats['dij'][1]/n:.2f} ms   "
          f"A*={1000*stats['ast'][1]/n:.2f} ms")
