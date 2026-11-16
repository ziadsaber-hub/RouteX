"""ROUTEX - C Model - Task 2: alternative routes (penalty method) on directed keyed edges (u,v,k).

Equation (11):   w_e^(r+1) = p * w_e^(r)   if e in P_r      (p = 1.5)
                 w_e^(r+1) = w_e^(r)        otherwise
Run Dijkstra on w, penalize exactly the directed edges (u,v,k) of the route found, run again.
Every candidate is judged with the ORIGINAL, unpenalized Model B times (E[eid]["mu"]).
Approved conditions (unchanged): detour <= 1.5 x fastest, shared directed edges <= 80%, up to 3 alternatives."""
import random
from task1_dijkstra import E, core, dijkstra_edges, route_time, route_length, is_chain

PENALTY = 1.5      # p in equation (11)
MAX_DETOUR = 1.5   # drop alternatives slower than 1.5x the fastest
MAX_SHARE = 0.8    # drop alternatives sharing >80% of directed edges with a kept route


def share(p, q):
    """Fraction of p's directed edges (u,v,k) that also appear in q."""
    ep, eq = set(p), set(q)
    return len(ep & eq) / len(ep)


def alternatives(src, dst, k=3, tries=12):
    """Return up to k+1 routes (lists of edge IDs): [fastest, alt1, alt2, ...]."""
    w = {e: d["mu"] for e, d in E.items()}          # penalized copy; E[...]["mu"] stays untouched
    cost = lambda e: w[e]
    routes, best_t = [], None
    for _ in range(tries):
        p, _ = dijkstra_edges(src, dst, cost)
        if p is None:
            break
        for e in p:                                 # penalize exactly the directed keyed edges used
            w[e] *= PENALTY
        if p in routes:
            continue
        t = route_time(p)                           # TRUE time: original weights, not w
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
    return dijkstra_edges(src, dst, lambda e: E[e]["length"])[0]


def validate(src, dst, routes):
    """Checks used in the report. Returns a list of problems (empty = all good)."""
    bad = []
    fastest, _ = dijkstra_edges(src, dst, lambda e: E[e]["mu"])
    if routes[0] != fastest:
        bad.append("first route is not the true fastest")
    t0 = route_time(routes[0])
    for i, p in enumerate(routes):
        if not (is_chain(p) and p[0][0] == src and p[-1][1] == dst):
            bad.append(f"route {i} is not a connected path src->dst")
        if i and route_time(p) < t0 - 1e-9:
            bad.append(f"alternative {i} is faster than the fastest")
        if i and route_time(p) > MAX_DETOUR * t0 + 1e-9:
            bad.append(f"alternative {i} breaks the detour limit")
        for j in range(i):
            if p == routes[j]:
                bad.append(f"routes {j} and {i} are identical")
            elif share(p, routes[j]) > MAX_SHARE + 1e-9:
                bad.append(f"routes {i} and {j} share more than {MAX_SHARE:.0%}")
    return bad


if __name__ == "__main__":
    random.seed(1)
    ids = sorted(core)
    src, dst = random.sample(ids, 2)
    routes = alternatives(src, dst)
    short = shortest_by_length(src, dst)
    print(f"from {src} to {dst}")
    t0 = route_time(routes[0])
    print(f"{'route':<10}{'time_s':>8}{'length_m':>10}{'vs fastest':>12}{'edges':>7}")
    for i, p in enumerate(routes):
        t, L = route_time(p), route_length(p)
        name = "fastest" if i == 0 else f"alt {i}"
        print(f"{name:<10}{t:>8.0f}{L:>10.0f}{100*(t/t0-1):>11.0f}%{len(p):>7}")
    t, L = route_time(short), route_length(short)
    print(f"{'shortest':<10}{t:>8.0f}{L:>10.0f}{100*(t/t0-1):>11.0f}%{len(short):>7}")
    print("alt 1 edge IDs (first 3):", [f"{u}-{v}-{k}" for u, v, k in routes[1][:3]] if len(routes) > 1 else "none")

    # many pairs: how many alternatives, and are they valid / genuinely different?
    n, counts, problems, parallel_only = 200, [], 0, 0
    random.seed(1)                       # same 200 pairs as the report
    for _ in range(n):
        s, d = random.sample(ids, 2)
        R = alternatives(s, d)
        counts.append(len(R) - 1)
        problems += len(validate(s, d, R))
        # alternatives whose only difference to the fastest is a parallel road (same u,v, other key)
        for p in R[1:]:
            if {(a, b) for a, b, _ in p} == {(a, b) for a, b, _ in R[0]} and p != R[0]:
                parallel_only += 1
    print(f"\n{n} random pairs: avg alternatives found = {sum(counts)/n:.2f}; "
          f"pairs with 0 alternatives = {counts.count(0)}")
    print(f"validation problems (fastest first / connected / detour / distinct / overlap): {problems}")
    print(f"alternatives that differ from the fastest only by a parallel road (other key): {parallel_only}")
