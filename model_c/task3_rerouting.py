"""ROUTEX - C Model - Task 3: rerouting when travel times change or a road closes (Dijkstra only).
Run:  python task3_rerouting.py   (needs task1_dijkstra.py, task2_alternatives.py and the xlsx next to it)
Seeds: pairs random.seed(1), fluctuation random.seed(7), Model B update random.seed(11), placeholder deltas random.seed(3).
Time unit everywhere = seconds. A route is a list of directed edge IDs (u,v,k).

Weights
  base weight   = Model B estimate mu_hat_e (E[eid]["mu"]), or a newer one sent with Live.update_travel_times()
                  (it already contains ordinary intersection waiting, so that is NOT added again)
  live weight   = base x congestion factor  (+ Delta T_signal on the approach edge of a signal)
  Delta T_signal = T_priority - T_normal   (section 5.3): ONLY the change caused by the ambulance's
                  priority is added, per approach edge (u,v,k) arriving at the signal; it may be negative.
                  Ordinary traffic delay is never added to the ambulance's time.

Rule (12):  switch route when  T_current - T_new > 0.10 * T_current.
A blocked route forces a switch whatever the threshold."""
import math
import random
import time
from task1_dijkstra import E, out, core, nodes, dijkstra_edges, route_time

INF = math.inf
THRESHOLD = 0.10

# ---------- which intersections have a signal ----------
# Verified lists from the team (columns: node_id, ...). Put the CSVs next to the scripts or set ROUTEX_SIGNALS /
# ROUTEX_NOT_SIGNALS. SIGNALS = checked and HAS a signal; NOT_SIGNALS = checked and has NO signal.
# Every other intersection is "not checked yet" and is treated as having no signal until it is verified.
import os
import pandas as pd


def _ids(path):
    return {int(x) for x in pd.read_csv(path, encoding="utf-8-sig")["node_id"]} if os.path.exists(path) else None


SIGNALS = _ids(os.environ.get("ROUTEX_SIGNALS", "Signalized_Intersections.csv"))
NOT_SIGNALS = _ids(os.environ.get("ROUTEX_NOT_SIGNALS", "Checked_Not_Signalized.csv")) or set()
SIGNALS_FROM_FILE = SIGNALS is not None
if SIGNALS is None:                      # no file: fall back to the old guess (4+ streets) and say so
    SIGNALS = {int(r.osmid) for r in nodes.itertuples() if r.street_count >= 4}
SIGNALS = set(SIGNALS) - NOT_SIGNALS


# ---------- input check for everything that arrives from Model B / Model D / SUMO ----------
def _check(vals, name, positive=True):
    """Stop with a clear message instead of silently using a bad value (NaN would make Dijkstra skip the edge)."""
    unknown = [e for e in vals if e not in E]
    if unknown:
        raise KeyError(f"{name}: {len(unknown)} edge IDs not in Model A, e.g. {unknown[0]}")
    bad = [e for e, t in vals.items() if not math.isfinite(t) or (positive and t <= 0)]
    if bad:
        raise ValueError(f"{name}: non-finite or non-positive value on {bad[0]}")


# ---------- live network: what is true right now ----------
class Live:
    def __init__(self):
        self.mu = {}          # eid -> newer Model B estimate (overrides E[eid]["mu"])
        self.slow = {}        # eid -> congestion multiplier >= 1
        self.closed = set()   # directed keyed edges that cannot be used
        self.signal = {}      # approach eid -> Delta T_signal (s), from Model D
        self.version = 0      # +1 on every change, so we know when to re-check

    def base(self, eid):
        return self.mu.get(eid, E[eid]["mu"])

    def cost(self, eid, dst):
        """Live time of the directed edge: Model B time x slowdown + Delta T_signal (no signal at the destination)."""
        if eid in self.closed:
            return INF
        c = self.base(eid) * self.slow.get(eid, 1.0)
        if eid[1] != dst:
            c += self.signal.get(eid, 0.0)
        return max(c, 0.0)

    def route_cost(self, route, dst):
        return sum(self.cost(e, dst) for e in route)

    # --- changes ---
    def close_edge(self, eid):
        self.closed.add(eid)
        self.version += 1

    def close_street(self, u, v, both=False):
        """Close u->v (every parallel key) in THIS direction only. both=True also closes v->u."""
        self.closed.update(e for e in out[u] if e[1] == v)
        if both:
            self.closed.update(e for e in out[v] if e[1] == u)
        self.version += 1

    def slow_edge(self, eid, factor):
        _check({eid: factor}, "slowdown")
        self.slow[eid] = factor
        self.version += 1

    def set_slowdowns(self, factors):
        _check(factors, "slowdown")
        self.slow.update(factors)
        self.version += 1

    def update_travel_times(self, mu):
        """Entry point for a new Model B estimate {(u,v,k): seconds}."""
        _check(mu, "Model B")
        self.mu.update(mu)
        self.version += 1

    def set_signal_deltas(self, deltas):
        """Entry point for Model D: {approach (u,v,k): T_priority - T_normal in seconds}."""
        _check(deltas, "Model D", positive=False)      # a delta may be negative (priority saves time)
        self.signal = dict(deltas)
        self.version += 1


# ---------- Dijkstra on the live weights ----------
def live_search(live, src, dst):
    """Returns (route as list of edge IDs, cost). (None, INF) if the destination is cut off."""
    return dijkstra_edges(src, dst, lambda e: live.cost(e, dst))


def upcoming_signal(route, i, live, dst):
    """First signal still ahead: its node, approach edge, exit edge and the time until the ambulance has passed it.
    If a closed road comes before the signal, the time is unknown and no signal is reported."""
    t = 0.0
    for j in range(i, len(route)):
        t += live.cost(route[j], dst)
        if t == INF:
            return None
        if route[j][1] in SIGNALS and j + 1 < len(route):
            return dict(node=route[j][1], approach=route[j], exit=route[j + 1], eta_s=t)
    return None


# ---------- one trip ----------
def drive(route, live, events, mode="reroute", threshold=THRESHOLD, log=None):
    """Simulate one ambulance trip along `route` (list of edge IDs).
    events: list of (time_s, function(live)) applied when the clock reaches them.
    mode 'static'  : never reroute (baseline).
    mode 'reroute' : when the network changed, search again (Dijkstra) from the current intersection and switch if
                     T_current - T_new > threshold x T_current. A closed edge on the route forces the switch.
    log: optional dict; if given, the trip is recorded in it (used by task3_export.py, changes no result)."""
    dst = route[-1][1]
    route, i, clock = list(route), 0, 0.0
    pending = sorted(events, key=lambda e: e[0])
    seen = live.version
    res = dict(time=0.0, changes=0, searches=0, run_s=0.0, finished=True)
    if log is not None:
        log.update(positions=[], routes=[(0.0, route[:], "initial", None, None)], changes=[])
    while i < len(route):
        while pending and pending[0][0] <= clock:
            pending.pop(0)[1](live)
        if mode == "reroute" and live.version != seen:
            seen = live.version
            remaining = live.route_cost(route[i:], dst)               # T_current
            t0 = time.perf_counter()
            new, new_cost = live_search(live, route[i][0], dst)       # T_new
            res["run_s"] += time.perf_counter() - t0
            res["searches"] += 1
            if new is not None and new != route[i:]:
                forced = remaining == INF
                if forced or (remaining - new_cost) > threshold * remaining:     # equation (12)
                    route = route[:i] + new
                    res["changes"] += 1
                    if log is not None:
                        why = "closed_road" if forced else "time_saving"
                        log["changes"].append((clock, route[i][0], why, None if forced else remaining, new_cost))
                        log["routes"].append((clock, route[:], why, None if forced else remaining, new_cost))
        eid = route[i]
        c = live.cost(eid, dst)
        if log is not None:
            log["positions"].append((clock, eid[0], eid if c != INF else None, upcoming_signal(route, i, live, dst)))
        if c == INF:                       # static baseline meets a closed road: stuck
            res["finished"] = False
            break
        clock += c
        i += 1
    if log is not None:
        log["positions"].append((clock, route[i][0] if i < len(route) else route[-1][1], None, None))
        log["finished"] = res["finished"]
    res["time"] = clock
    return res


# ---------- scenarios ----------
ids = sorted(core)


def make_pairs(n=200, min_time=60):
    """Same style as Task 2 (random.seed(1)); very short trips (< min_time s) have no room to reroute."""
    random.seed(1)
    res = []
    while len(res) < n:
        s, d = random.sample(ids, 2)
        p, _ = live_search(Live(), s, d)
        if route_time(p) >= min_time:
            res.append((s, d, p))
    return res


def edge_at(route, frac, ahead=2):
    """The directed edge hit by an event that fires `frac` of the way through the trip (by time):
    `ahead` edges after the ambulance's position at that moment, so it is always still ahead of it."""
    te = frac * route_time(route)
    clock, i = 0.0, 0
    while i < len(route) - 1 and clock < te:
        clock += E[route[i]]["mu"]
        i += 1
    return route[min(i + ahead, len(route) - 1)]


def event_time(route, frac):
    return frac * route_time(route)


def summary(rows):
    n = len(rows)
    done = [r for r in rows if r["finished"]]
    s = max(sum(r["searches"] for r in rows), 1)
    return dict(n=n, finished=len(done),
                time=sum(r["time"] for r in done) / max(len(done), 1),
                changes=sum(r["changes"] for r in rows) / n,
                searches=sum(r["searches"] for r in rows) / n,
                ms=1000 * sum(r["run_s"] for r in rows) / s)


def _run(pairs, make_event, frac=0.3, **kw):
    rows = {"static": [], "reroute": []}
    base = []
    for s, d, p in pairs:
        eid = edge_at(p, frac)
        ev = [(event_time(p, frac), make_event(eid))]
        base.append(route_time(p))
        for m in rows:
            rows[m].append(drive(p, Live(), ev, mode=m, **kw))
    return rows, base


def scenario_closure(pairs):
    return _run(pairs, lambda eid: (lambda L: L.close_street(eid[0], eid[1])))      # one direction only


def scenario_congestion(pairs, factor=3.0):
    return _run(pairs, lambda eid: (lambda L: L.slow_edge(eid, factor)))


def scenario_model_b_update(pairs, lo=1.0, hi=2.0, seed=11):
    """At 30% of the trip Model B sends new times: EVERY edge becomes mu x U[lo, hi]."""
    random.seed(seed)
    rows = {"static": [], "reroute": []}
    base = []
    for s, d, p in pairs:
        new = {e: E[e]["mu"] * random.uniform(lo, hi) for e in E}
        ev = [(event_time(p, 0.3), lambda L, new=new: L.update_travel_times(new))]
        base.append(route_time(p))
        for m in rows:
            rows[m].append(drive(p, Live(), ev, mode=m))
    return rows, base


def scenario_fluctuation(pairs, thresholds, amp=0.6, step=15):
    """Every `step` seconds EVERY edge gets a new random slowdown in [1, 1+amp]: many small changes."""
    res = {}
    for thr in thresholds:
        random.seed(7)
        rows = []
        for s, d, p in pairs:
            horizon = int(2 * route_time(p))
            ev = []
            for t in range(step, horizon, step):
                f = {e: 1 + random.random() * amp for e in E}
                ev.append((t, lambda L, f=f: L.set_slowdowns(f)))
            rows.append(drive(p, Live(), ev, mode="reroute", threshold=thr))
        res[thr] = summary(rows)
    return res


def demo_signal_deltas(seed=3, lo=-10.0, hi=5.0):
    """PLACEHOLDER for Model D: Delta T_signal = T_priority - T_normal per approach edge of a VERIFIED signal.
    Replace by the real values from Model D."""
    random.seed(seed)
    return {e: random.uniform(lo, hi) for e in sorted(E) if e[1] in SIGNALS}


def scenario_signal_delta(pairs):
    live = Live()
    live.set_signal_deltas(demo_signal_deltas())
    changed, t_old, t_new = 0, 0.0, 0.0
    for s, d, p in pairs:
        p2, c2 = live_search(live, s, d)
        t_old += live.route_cost(p, d)           # Task 1 route driven with the deltas
        t_new += c2
        changed += p2 != p
    n = len(pairs)
    return dict(changed=changed, n=n, old=t_old / n, new=t_new / n, gain=100 * (1 - t_new / t_old))


def table(title, rows, base):
    print(f"\n{title}   original ETA avg = {sum(base)/len(base):.0f} s")
    print(f"{'mode':<9}{'finished':>9}{'avg time':>10}{'changes':>9}{'searches':>9}{'ms/search':>10}")
    for m, r in rows.items():
        x = summary(r)
        print(f"{m:<9}{x['finished']:>5}/{x['n']:<3}{x['time']:>10.0f}{x['changes']:>9.2f}"
              f"{x['searches']:>9.2f}{x['ms']:>10.3f}")


if __name__ == "__main__":
    # direction test: closing u->v must leave v->u open unless both=True
    pr = make_pairs(5)
    u, v, _k = pr[0][2][0]
    L = Live(); L.close_street(u, v)
    assert all(e in L.closed for e in out[u] if e[1] == v) and not any(e in L.closed for e in out[v] if e[1] == u)
    L2 = Live(); L2.close_street(u, v, both=True)
    assert all(e in L2.closed for e in out[v] if e[1] == u)
    print("close_street direction test: OK (one direction by default, both only on request)")

    pairs = make_pairs()
    print(f"{len(pairs)} pairs (random.seed(1), fastest time >= 60 s), avg fastest time "
          f"{sum(route_time(p) for _, _, p in pairs)/len(pairs):.0f} s")

    table("A) sudden closure of one direction (30% into the trip)", *scenario_closure(pairs))
    table("B) congestion x3 on one edge (30% into the trip)", *scenario_congestion(pairs))
    table("F) Model B sends new times for every edge (x1..x2, at 30% of the trip)", *scenario_model_b_update(pairs))

    print("\n3.1 examples (closure at 30% of the trip, rerouting on)")
    print(f"{'pair':<11}{'edges':>6}{'orig s':>8}{'closed edge':>16}{'actual s':>10}{'slower':>8}")
    for s_, d_, p in pairs[:4]:
        eid = edge_at(p, 0.3)
        r = drive(p, Live(), [(event_time(p, 0.3), lambda L, eid=eid: L.close_street(eid[0], eid[1]))])
        t0 = route_time(p)
        print(f"{s_:>4} -> {d_:<4}{len(p):>6}{t0:>8.0f}{'-'.join(map(str, eid)):>16}{r['time']:>10.0f}{100 * (r['time'] / t0 - 1):>7.0f}%")

    print("\nC) many small fluctuations (every 15 s, every edge x[1, 1.6]) - Rerouting Threshold sweep")
    print(f"{'threshold':>10}{'avg time':>10}{'changes':>9}{'searches':>9}")
    for thr, x in scenario_fluctuation(pairs, [0, 0.05, 0.10, 0.15, 0.20, 0.30]).items():
        print(f"{thr:>9.0%}{x['time']:>10.0f}{x['changes']:>9.2f}{x['searches']:>9.2f}")

    print("\nE) Rerouting Threshold when ONE edge gets much slower (x5 at 30% of the trip)")
    print(f"{'threshold':>10}{'avg time':>10}{'changes':>9}")
    for thr in (0, 0.05, 0.10, 0.20, 0.30, 0.50):
        rr = []
        for s_, d_, p in pairs:
            eid = edge_at(p, 0.3)
            ev = [(event_time(p, 0.3), lambda L, eid=eid: L.slow_edge(eid, 5.0))]
            rr.append(drive(p, Live(), ev, threshold=thr))
        x = summary(rr)
        print(f"{thr:>9.0%}{x['time']:>10.1f}{x['changes']:>9.2f}")

    print("\n3.6 Robustness: closure, trips that reached the destination (no rerouting -> rerouting)")
    for fr in (0.10, 0.30, 0.60):
        n = {}
        for m in ("static", "reroute"):
            c = 0
            for s_, d_, p in pairs:
                eid = edge_at(p, fr)
                ev = [(event_time(p, fr), lambda L, eid=eid: L.close_street(eid[0], eid[1]))]
                c += drive(p, Live(), ev, mode=m)["finished"]
            n[m] = c
        print(f"  closure at {fr:.0%} of the trip: {n['static']} / {len(pairs)} -> {n['reroute']} / {len(pairs)}")
    print("3.6 Robustness: one slow edge, avg trip time (no rerouting -> rerouting, threshold 10%)")
    print(f"{'slowdown':>9}{'event at 10%':>16}{'event at 30%':>16}{'event at 60%':>16}")
    for f in (2.0, 3.0, 5.0):
        cells = []
        for fr in (0.10, 0.30, 0.60):
            t = {}
            for m in ("static", "reroute"):
                tot = 0.0
                for s_, d_, p in pairs:
                    eid = edge_at(p, fr)
                    ev = [(event_time(p, fr), lambda L, eid=eid, f=f: L.slow_edge(eid, f))]
                    tot += drive(p, Live(), ev, mode=m)["time"]
                t[m] = tot / len(pairs)
            cells.append(f"{t['static']:.1f} -> {t['reroute']:.1f}")
        print(f"{'x%g' % f:>9}{cells[0]:>16}{cells[1]:>16}{cells[2]:>16}")

    x = scenario_signal_delta(pairs)
    print(f"\nD) signal feedback Delta T = T_priority - T_normal (PLACEHOLDER for Model D, -10..+5 s per approach edge)")
    print(f"verified signals used: {len(SIGNALS)} ({'from the CSV files' if SIGNALS_FROM_FILE else 'GUESS: files not found, 4+ streets'}), "
          f"checked as no signal: {len(NOT_SIGNALS)}")
    print(f"route changes because of the deltas: {x['changed']} of {x['n']} pairs; "
          f"avg time Task 1 route {x['old']:.0f} s -> delta-aware route {x['new']:.0f} s ({x['gain']:.1f}% less)")
