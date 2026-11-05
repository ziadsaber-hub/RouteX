"""ROUTEX - C Model - Task 2: alternative routes (penalty method).
Idea: run Dijkstra, make the edges of the found route more expensive,
run again -> a different route. Repeat k times. Judge every route by its
TRUE free-flow time/length (original weights, not the penalized ones)."""
import heapq
import math
import random
from task1_dijkstra import adj, core

PENALTY = 1.5      # multiply time of edges already used by a route
MAX_DETOUR = 1.5   # drop alternatives slower than 1.5x the fastest
MAX_SHARE = 0.8    # drop alternatives sharing >80% of edges with a kept route


def dijkstra_w(w, src, dst):
    """Dijkstra on weights w[u][v] (float). Returns path or None."""
    dist, prev, pq = {src: 0.0}, {}, [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, math.inf):
            continue
        if u == dst:
            break
        for v, c in w[u].items():
            nd = d + c
            if nd < dist.get(v, math.inf):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if dst not in dist:
        return None
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1]


def stats(path):
    """True (time_s, length_m) using the original weights."""
    t = sum(adj[a][b][0] for a, b in zip(path, path[1:]))
    L = sum(adj[a][b][1] for a, b in zip(path, path[1:]))
    return t, L


def edges_of(path):
    return set(zip(path, path[1:]))


def share(p, q):
    """Fraction of p's edges that also appear in q."""
    ep, eq = edges_of(p), edges_of(q)
    return len(ep & eq) / len(ep)


def alternatives(src, dst, k=3, tries=12):
    """Return up to k+1 routes: [fastest, alt1, alt2, ...] (as node lists)."""
    w = {u: {v: tv[0] for v, tv in nb.items()} for u, nb in adj.items()}
    routes = []
    best_t = None
    for _ in range(tries):
        p = dijkstra_w(w, src, dst)
        if p is None:
            break
        for a, b in zip(p, p[1:]):                 # penalize this route
            w[a][b] *= PENALTY
        if p in routes:
            continue
        t, _ = stats(p)
        if best_t is None:
            best_t = t                              # first route = true fastest
        elif t > MAX_DETOUR * best_t:
            continue
        elif any(share(p, q) > MAX_SHARE for q in routes):
            continue
        routes.append(p)
        if len(routes) == k + 1:
            break
    return routes


def shortest_by_length(src, dst):
    w = {u: {v: tv[1] for v, tv in nb.items()} for u, nb in adj.items()}
    return dijkstra_w(w, src, dst)


if __name__ == "__main__":
    random.seed(1)
    ids = sorted(core)
    src, dst = random.sample(ids, 2)
    routes = alternatives(src, dst)
    short = shortest_by_length(src, dst)
    print(f"from {src} to {dst}")
    t0, L0 = stats(routes[0])
    print(f"{'route':<10}{'time_s':>8}{'length_m':>10}{'vs fastest':>12}{'edges':>7}")
    for i, p in enumerate(routes):
        t, L = stats(p)
        name = "fastest" if i == 0 else f"alt {i}"
        print(f"{name:<10}{t:>8.0f}{L:>10.0f}{100*(t/t0-1):>11.0f}%{len(p)-1:>7}")
    t, L = stats(short)
    print(f"{'shortest':<10}{t:>8.0f}{L:>10.0f}{100*(t/t0-1):>11.0f}%{len(short)-1:>7}")

    # over many pairs: how many alternatives do we usually find?
    n, counts = 200, []
    random.seed(1)                       # same 200 pairs as task2_compare.py and the report
    for _ in range(n):
        s, d = random.sample(ids, 2)
        counts.append(len(alternatives(s, d)) - 1)
    print(f"\n{n} random pairs: avg alternatives found = {sum(counts)/n:.2f}; "
          f"pairs with 0 alternatives = {counts.count(0)}")
