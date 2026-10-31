"""ROUTEX - C Model - Task 3: rerouting when road times change or a road closes.
Run:  python task3_rerouting.py        (needs task1_dijkstra.py, task1_astar.py,
                                         task2_alternatives.py and the xlsx next to it)
Prints every table used in the Task 3 report. Seeds: pairs random.seed(1), fluctuation tests
random.seed(7), placeholder signal delays random.seed(3).
Time unit everywhere = seconds. Task 1 / Task 2 numbers are NOT changed here:
(penalty 1.5, detour 1.5, overlap limit 80% stay in task2_alternatives.py).

Idea: the ambulance drives along its route. At every intersection it checks the
live network. If something changed it searches again from where it is
(Dijkstra or A* on the live weights) and switches route only if the new route
saves more than the Rerouting Threshold. A closed road forces a switch."""
import heapq
import math
import random
import time
from task1_dijkstra import adj, core, nodes
from task1_astar import h as heuristic          # same admissible heuristic (0.99 factor) as Task 1
from task2_alternatives import dijkstra_w, stats

INF = math.inf


# ---------- live network: what is true right now ----------
class Live:
    """Free-flow times from Task 1, plus what changes during the trip."""

    def __init__(self):
        self.slow = {}          # (u,v) -> multiplier >= 1   (congestion)
        self.closed = set()     # (u,v) directed edges that cannot be used
        self.delay = {}         # node -> expected signal delay in s (from Model D)
        self.version = 0        # +1 on every change, so we know when to re-check

    def cost(self, u, v, dst):
        """Live time of edge u->v: free-flow x slowdown + signal delay at v (none at the destination)."""
        if (u, v) in self.closed:
            return INF
        c = adj[u][v][0] * self.slow.get((u, v), 1.0)
        if v != dst:
            c += self.delay.get(v, 0.0)
        return c

    def route_cost(self, path, dst):
        return sum(self.cost(a, b, dst) for a, b in zip(path, path[1:]))

    # --- changes ---
    def close_street(self, u, v):
        self.closed.add((u, v))
        if u in adj[v]:
            self.closed.add((v, u))
        self.version += 1

    def slow_edge(self, u, v, factor):
        self.slow[(u, v)] = factor
        self.version += 1

    def set_delays(self, delays):
        """Entry point for Model D: {intersection: expected delay in seconds}."""
        self.delay = dict(delays)
        self.version += 1


# ---------- search on the live weights (A* or Dijkstra) ----------
def live_search(live, src, dst, use_h=True):
    """Returns (path, cost, expanded_nodes). Times only grow, so the Task 1 heuristic stays admissible."""
    g, prev, closed, expanded = {src: 0.0}, {}, set(), 0
    pq = [(heuristic(src, dst) if use_h else 0.0, src)]
    while pq:
        _, u = heapq.heappop(pq)
        if u in closed:
            continue
        closed.add(u)
        expanded += 1
        if u == dst:
            break
        for v in adj[u]:
            c = live.cost(u, v, dst)
            if c == INF:
                continue
            ng = g[u] + c
            if ng < g.get(v, INF):
                g[v], prev[v] = ng, u
                heapq.heappush(pq, (ng + (heuristic(v, dst) if use_h else 0.0), v))
    if dst not in g:
        return None, INF, expanded
    path = [dst]
    while path[-1] != src:
        path.append(prev[path[-1]])
    return path[::-1], g[dst], expanded


# ---------- one trip ----------
def drive(route, live, events, mode="reroute", threshold=0.10, use_h=True):
    """Simulate one ambulance trip.
    events: list of (time_s, function(live)) applied when the clock reaches them.
    mode 'static'  : never reroute (baseline).
    mode 'reroute' : re-search when the network changed; switch if the saving > threshold x remaining time
                     (a closed road on the route forces a switch whatever the threshold).
    Returns dict: total time, route changes, searches, search time, expanded nodes, finished?"""
    dst = route[-1]
    route, i, clock = route[:], 0, 0.0
    pending = sorted(events, key=lambda e: e[0])
    seen = live.version              # the route was optimal when the trip started
    out = dict(time=0.0, changes=0, searches=0, run_s=0.0, expanded=0, finished=True)
    while route[i] != dst:
        while pending and pending[0][0] <= clock:
            pending.pop(0)[1](live)
        if mode == "reroute" and live.version != seen:
            seen = live.version
            remaining = live.route_cost(route[i:], dst)
            t0 = time.perf_counter()
            new, new_cost, exp = live_search(live, route[i], dst, use_h)
            out["run_s"] += time.perf_counter() - t0
            out["searches"] += 1
            out["expanded"] += exp
            if new is not None and new != route[i:]:
                forced = remaining == INF
                if forced or (remaining - new_cost) > threshold * remaining:
                    route = route[:i] + new
                    out["changes"] += 1
        c = live.cost(route[i], route[i + 1], dst)
        if c == INF:                       # static baseline meets a closed road: stuck
            out["finished"] = False
            break
        clock += c
        i += 1
    out["time"] = clock
    return out


# ---------- scenarios ----------
ids = sorted(core)


def make_pairs(n=200, min_time=60):
    """Same style as Task 2 (random.seed(1)); very short trips (< min_time s) have no room to reroute."""
    random.seed(1)
    out = []
    while len(out) < n:
        s, d = random.sample(ids, 2)
        p, _, _ = live_search(Live(), s, d)
        if stats(p)[0] >= min_time:
            out.append((s, d, p))
    return out


def edge_at(route, dst, frac, ahead=2):
    """The road hit by an event that fires when the ambulance is `frac` of the way through the trip (by time).
    The ambulance's position at that moment is found by driving the route on free-flow times, and the road
    chosen is the one `ahead` roads after that intersection (so it is always still ahead of the ambulance)."""
    te = frac * stats(route)[0]
    clock, i = 0.0, 0
    while i < len(route) - 1 and clock < te:
        clock += adj[route[i]][route[i + 1]][0]
        i += 1
    k = min(i + ahead, len(route) - 2)
    return route[k], route[k + 1]


def event_time(route, frac):
    return frac * stats(route)[0]


def summary(rows):
    n = len(rows)
    done = [r for r in rows if r["finished"]]
    return dict(n=n, finished=len(done),
                time=sum(r["time"] for r in done) / max(len(done), 1),
                changes=sum(r["changes"] for r in rows) / n,
                searches=sum(r["searches"] for r in rows) / n,
                ms=1000 * sum(r["run_s"] for r in rows) / max(sum(r["searches"] for r in rows), 1),
                exp=sum(r["expanded"] for r in rows) / max(sum(r["searches"] for r in rows), 1))


def scenario_closure(pairs):
    rows = {"static": [], "reroute": []}
    base = []
    for s, d, p in pairs:
        u, v = edge_at(p, d, 0.3)
        ev = [(event_time(p, 0.3), lambda L, u=u, v=v: L.close_street(u, v))]
        base.append(stats(p)[0])
        for m in rows:
            rows[m].append(drive(p, Live(), ev, mode=m))
    return rows, base


def scenario_congestion(pairs, factor=3.0):
    rows = {"static": [], "reroute": []}
    base = []
    for s, d, p in pairs:
        u, v = edge_at(p, d, 0.3)
        ev = [(event_time(p, 0.3), lambda L, u=u, v=v: L.slow_edge(u, v, factor))]
        base.append(stats(p)[0])
        for m in rows:
            rows[m].append(drive(p, Live(), ev, mode=m))
    return rows, base


def scenario_fluctuation(pairs, thresholds, amp=0.6, step=15):
    """Every `step` seconds EVERY road gets a new random slowdown in [1, 1+amp]: many small changes."""
    out = {}
    for thr in thresholds:
        random.seed(7)
        rows = []
        for s, d, p in pairs:
            horizon = int(2 * stats(p)[0])
            ev = []
            for t in range(step, horizon, step):
                f = {(u, v): 1 + random.random() * amp for u in adj for v in adj[u]}
                ev.append((t, lambda L, f=f: [L.slow.update(f), setattr(L, "version", L.version + 1)]))
            rows.append(drive(p, Live(), ev, mode="reroute", threshold=thr))
        out[thr] = summary(rows)
    return out


def demo_delays(seed=3, lo=5, hi=30):
    """PLACEHOLDER for Model D: a delay of lo..hi s at every intersection with 4+ streets.
    Replace by the real expected delays from Model D (a dict node -> seconds)."""
    random.seed(seed)
    street = {int(r.osmid): r.street_count for r in nodes.itertuples()}
    return {n: random.uniform(lo, hi) for n in ids if street[n] >= 4}


def scenario_delays(pairs):
    delays = demo_delays()
    live = Live()
    live.set_delays(delays)
    changed, t_old, t_new = 0, 0.0, 0.0
    for s, d, p in pairs:
        p2, c2, _ = live_search(live, s, d)
        c_old = live.route_cost(p, d)           # Task 1 route driven with the delays
        changed += p2 != p
        t_old += c_old
        t_new += c2
    n = len(pairs)
    return dict(changed=changed, n=n, old=t_old / n, new=t_new / n, gain=100 * (1 - t_new / t_old))


if __name__ == "__main__":
    pairs = make_pairs()
    print(f"{len(pairs)} pairs (random.seed(1), fastest time >= 60 s), avg fastest time "
          f"{sum(stats(p)[0] for _, _, p in pairs)/len(pairs):.0f} s")

    for name, fn in (("A) sudden closure (30% into the trip)", scenario_closure),
                     ("B) new congestion x3 on one road (30% into the trip)", scenario_congestion)):
        rows, base = fn(pairs)
        print(f"\n{name}   original ETA avg = {sum(base)/len(base):.0f} s")
        print(f"{'mode':<9}{'finished':>9}{'avg time':>10}{'changes':>9}{'searches':>9}{'ms/search':>10}")
        for m, r in rows.items():
            x = summary(r)
            print(f"{m:<9}{x['finished']:>5}/{x['n']:<3}{x['time']:>10.0f}{x['changes']:>9.2f}"
                  f"{x['searches']:>9.2f}{x['ms']:>10.3f}")

    print("\n3.1 examples (closure at 30% of the trip, rerouting on)")
    print(f"{'pair':<11}{'edges':>6}{'orig s':>8}{'closed road':>14}{'actual s':>10}{'slower':>8}")
    for s_, d_, p in pairs[:4]:
        u, v = edge_at(p, d_, 0.3)
        r = drive(p, Live(), [(event_time(p, 0.3), lambda L, u=u, v=v: L.close_street(u, v))])
        t0 = stats(p)[0]
        print(f"{s_:>4} -> {d_:<4}{len(p) - 1:>6}{t0:>8.0f}{str(u) + '-' + str(v):>14}{r['time']:>10.0f}{100 * (r['time'] / t0 - 1):>7.0f}%")

    # Dijkstra vs A* for the replanning step
    rows, _ = scenario_closure(pairs)
    print("\nreplanning step, closure scenario:")
    for use_h, nm in ((False, "Dijkstra"), (True, "A*")):
        rr = []
        for s, d, p in pairs:
            u, v = edge_at(p, d, 0.3)
            ev = [(event_time(p, 0.3), lambda L, u=u, v=v: L.close_street(u, v))]
            rr.append(drive(p, Live(), ev, use_h=use_h))
        x = summary(rr)
        print(f"{nm:<9} avg nodes expanded/search = {x['exp']:.1f}   {x['ms']:.3f} ms/search   avg time {x['time']:.1f} s")

    print("\nC) many small fluctuations (every 15 s, every road x[1, 1.6]) - Rerouting Threshold sweep")
    print(f"{'threshold':>10}{'avg time':>10}{'changes':>9}{'searches':>9}")
    for thr, x in scenario_fluctuation(pairs, [0, 0.05, 0.10, 0.15, 0.20, 0.30]).items():
        print(f"{thr:>9.0%}{x['time']:>10.0f}{x['changes']:>9.2f}{x['searches']:>9.2f}")

    print("\nE) Rerouting Threshold when ONE road gets much slower (x5 at 30% of the trip)")
    print(f"{'threshold':>10}{'avg time':>10}{'changes':>9}")
    for thr in (0, 0.05, 0.10, 0.20, 0.30, 0.50):
        rr = []
        for s_, d_, p in pairs:
            u, v = edge_at(p, d_, 0.3)
            ev = [(event_time(p, 0.3), lambda L, u=u, v=v: L.slow_edge(u, v, 5.0))]
            rr.append(drive(p, Live(), ev, threshold=thr))
        x = summary(rr)
        print(f"{thr:>9.0%}{x['time']:>10.1f}{x['changes']:>9.2f}")

    print("\n3.6 Robustness: closure, trips that reached the destination (no rerouting -> rerouting)")
    for fr in (0.10, 0.30, 0.60):
        n = {}
        for m in ("static", "reroute"):
            c = 0
            for s_, d_, p in pairs:
                u, v = edge_at(p, d_, fr)
                ev = [(event_time(p, fr), lambda L, u=u, v=v: L.close_street(u, v))]
                c += drive(p, Live(), ev, mode=m)["finished"]
            n[m] = c
        print(f"  closure at {fr:.0%} of the trip: {n['static']} / {len(pairs)} -> {n['reroute']} / {len(pairs)}")
    print("3.6 Robustness: one slow road, avg trip time (no rerouting -> rerouting, threshold 10%)")
    print(f"{'slowdown':>9}{'event at 10%':>16}{'event at 30%':>16}{'event at 60%':>16}")
    for f in (2.0, 3.0, 5.0):
        cells = []
        for fr in (0.10, 0.30, 0.60):
            t = {}
            for m in ("static", "reroute"):
                tot = 0.0
                for s_, d_, p in pairs:
                    u, v = edge_at(p, d_, fr)
                    ev = [(event_time(p, fr), lambda L, u=u, v=v, f=f: L.slow_edge(u, v, f))]
                    tot += drive(p, Live(), ev, mode=m)["time"]
                t[m] = tot / len(pairs)
            cells.append(f"{t['static']:.1f} -> {t['reroute']:.1f}")
        print(f"{'x%g' % f:>9}{cells[0]:>16}{cells[1]:>16}{cells[2]:>16}")

    x = scenario_delays(pairs)
    print(f"\nD) signal delays (placeholder for Model D, 5-30 s at 4+ street intersections)")
    print(f"route changes because of delays: {x['changed']} of {x['n']} pairs; "
          f"avg time Task 1 route {x['old']:.0f} s -> delay-aware route {x['new']:.0f} s ({x['gain']:.1f}% less)")
