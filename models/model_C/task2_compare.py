"""ROUTEX - C Model - Task 2 experiments: Penalty vs Yen.
Run:  python task2_compare.py
Prints every table used in the Task 2 report (random.seed(1), 200 pairs)."""
import random, time
from itertools import islice
import networkx as nx
import task2_alternatives as T2
from task1_dijkstra import adj, core, G
from task2_alternatives import alternatives, stats, edges_of, share, dijkstra_w

DETOUR, MAX_SHARE = 1.5, 0.8
ids = sorted(core)


def make_pairs(n=200):
    random.seed(1)
    return [tuple(random.sample(ids, 2)) for _ in range(n)]


# ---------- Yen (networkx) ----------
def yen_routes(s, d, k=3):
    """Fastest + next k simple paths by time, within the detour limit (no overlap filter)."""
    out, best = [], None
    for p in islice(nx.shortest_simple_paths(G, s, d, weight="time"), k + 1):
        t = stats(p)[0]
        if best is None:
            best = t
        elif t > DETOUR * best:
            break
        out.append(p)
    return out


# ---------- closure test ----------
def closure(method, pairs):
    hit = tot = 0
    for s, d in pairs:
        R = method(s, d)
        if len(R) < 2:      # pairs with no alternative are not counted (same as the report)
            continue
        for e in edges_of(R[0]):
            tot += 1
            hit += any(e not in edges_of(p) for p in R[1:])
    return hit, tot


if __name__ == "__main__":
    pairs = make_pairs(200)

    # 1) summary table, both methods
    print("== Methods compared (200 pairs) ==")
    print(f"{'method':<10}{'avg alt':>8}{'0 alt':>6}{'slowdown%':>10}{'shared1st%':>11}"
          f"{'pairs>80%':>10}{'closure':>16}{'ms/pair':>9}")
    for name, f in [("Yen", yen_routes), ("Penalty", alternatives)]:
        t0 = time.perf_counter()
        cnt, slow, sh1, over = [], [], [], 0
        for s, d in pairs:
            R = f(s, d); A = R[1:]; cnt.append(len(A))
            b = stats(R[0])[0]
            slow += [100 * (stats(p)[0] / b - 1) for p in A]
            if A:
                x = share(A[0], R[0]); sh1.append(100 * x); over += x > MAX_SHARE
        ms = 1000 * (time.perf_counter() - t0) / len(pairs)
        h, t = closure(f, pairs)
        print(f"{name:<10}{sum(cnt)/len(cnt):>8.2f}{cnt.count(0):>6}{sum(slow)/len(slow):>10.0f}"
              f"{sum(sh1)/len(sh1):>11.0f}{over:>5} of {len(sh1):<3}{h:>6}/{t:<5}={100*h/t:>3.0f}%{ms:>6.1f}")

    # 2) Yen 2nd route vs Penalty 1st alternative (report table in section 3)
    ys = []
    for s, d in pairs:
        p = list(islice(nx.shortest_simple_paths(G, s, d, weight="time"), 2))
        if len(p) == 2: ys.append(share(p[1], p[0]))
    print(f"\nYen 2nd route (unfiltered): avg shared {100*sum(ys)/len(ys):.0f}%, "
          f">80% in {sum(x > .8 for x in ys)} of {len(ys)} pairs")

    # 3) parameter sweep, first 100 pairs
    print("\n== Parameter effect (first 100 pairs) ==")
    base_p, base_d = T2.PENALTY, T2.MAX_DETOUR
    for name, attr, vals in [("Penalty", "PENALTY", [1.2, 1.5, 2.0]),
                             ("Detour", "MAX_DETOUR", [1.25, 1.5, 2.0])]:
        for v in vals:
            T2.PENALTY, T2.MAX_DETOUR = base_p, base_d
            setattr(T2, attr, v)
            c = [len(alternatives(s, d)) - 1 for s, d in pairs[:100]]
            print(f"{name:<8}{v:<6}avg alt {sum(c)/100:.2f}   pairs with none {c.count(0)}")
    T2.PENALTY, T2.MAX_DETOUR = base_p, base_d
