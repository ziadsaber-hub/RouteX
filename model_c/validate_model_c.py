"""ROUTEX - Model C validation (tasks 1-6). Run:  python validate_model_c.py
Every check prints PASS/FAIL and is saved to validation_results.csv (test, value, status)."""
import csv
import itertools
import math
import os
import random
import tempfile
import pandas as pd

import task2_alternatives as T2
from task1_dijkstra import E, out, core, N_DERIVED, dijkstra_edges, route_time, is_chain
from task3_rerouting import Live, drive, live_search, make_pairs, edge_at, event_time, SIGNALS
from task3_export import run_trip, export_csv, eid_str

INF = math.inf
ROWS = []


def check(name, ok, value=""):
    ROWS.append((name, value, "PASS" if ok else "FAIL"))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ->  {value}" if value != "" else ""))


ids = sorted(core)
random.seed(1)
PAIRS_RAW = [tuple(random.sample(ids, 2)) for _ in range(200)]

# =====================================================================
print("\n=== 1) Unreachable / edge cases in alternative routes ===")
outside = [n for n in out if n not in core]
random.seed(5)
n_unreach, ok = 0, True
for _ in range(400):
    s, d = random.choice(ids), random.choice(outside)
    if dijkstra_edges(s, d, lambda e: E[e]["mu"])[0] is None:
        n_unreach += 1
        ok &= T2.alternatives(s, d) == [] and T2.validate(s, d, []) == []
check("unreachable destinations return [] and validate() does not crash", ok, f"{n_unreach} unreachable pairs tested")
check("src == dst returns []", T2.alternatives(ids[0], ids[0]) == [])
try:
    T2.alternatives(-1, ids[0]); raised = False
except ValueError:
    raised = True
check("unknown node raises ValueError", raised)

# =====================================================================
print("\n=== 2) Alternative routes: 200 pairs, constraints, tries ===")
probs, counts, worst_detour, worst_overlap = 0, [], 0.0, 0.0
for s, d in PAIRS_RAW:
    R = T2.alternatives(s, d)
    probs += len(T2.validate(s, d, R))
    counts.append(len(R) - 1)
    for p in R[1:]:
        worst_detour = max(worst_detour, route_time(p) / route_time(R[0]))
    for a, b in itertools.combinations(R, 2):
        worst_overlap = max(worst_overlap, T2.overlap(a, b))
check("200 pairs: 0 validation problems", probs == 0, probs)
check("detour limit respected (max ratio <= 1.5)", worst_detour <= T2.MAX_DETOUR + 1e-9, f"max {worst_detour:.3f}")
check("overlap limit respected (max <= 0.8, both directions)", worst_overlap <= T2.MAX_SHARE + 1e-9, f"max {worst_overlap:.3f}")
check("at most 3 alternatives per pair", max(counts) <= 3, f"max {max(counts)}")
print(f"      avg alternatives {sum(counts)/200:.2f}, pairs with none {counts.count(0)} (TRIES={T2.TRIES}, symmetric={T2.SYMMETRIC_OVERLAP})")
ROWS.append(("avg alternatives per pair", round(sum(counts) / 200, 3), "INFO"))
ROWS.append(("pairs with no alternative", counts.count(0), "INFO"))

# =====================================================================
print("\n=== 3) Network / Model B interface ===")
check("all 546 directed edges loaded", len(E) == 546, len(E))
check("every edge has finite positive time", all(0 < d["mu"] < INF for d in E.values()))
check("no derived reverse edges", N_DERIVED == 0, N_DERIVED)
check("no negative node ids (so 'u-v-k' splits unambiguously on '-')", all(isinstance(n, int) and n >= 0 for n in out))

# =====================================================================
print("\n=== 4) Rerouting threshold (10%) ===")
PAIRS = make_pairs()          # 200 pairs, fastest >= 60 s


def state_at_event(p, te):
    """Index of the edge the ambulance is about to take when the event fires (same clock logic as drive())."""
    clock, i = 0.0, 0
    while i < len(p) and clock < te:
        clock += E[p[i]]["mu"]
        i += 1
    return i


agree, total, switch_cases, below_cases, boundary_ok, boundary_n = 0, 0, 0, 0, 0, 0
for s, d, p in PAIRS:
    eid, te = edge_at(p, 0.3), event_time(p, 0.3)
    i = state_at_event(p, te)
    if i >= len(p):
        continue
    for f in (1.5, 2.0, 3.0, 5.0):
        ev = [(te, lambda L, eid=eid, f=f: L.slow_edge(eid, f))]
        L = Live(); L.slow_edge(eid, f)
        rem = L.route_cost(p[i:], d)
        new, nc = live_search(L, p[i][0], d)
        saving = None if (new is None or new == p[i:]) else (rem - nc) / rem
        expected = saving is not None and saving > 0.10
        got = drive(p, Live(), ev, threshold=0.10)["changes"] == 1
        total += 1
        agree += expected == got
        switch_cases += expected
        below_cases += (saving is not None and 0 < saving <= 0.10)
        if saving is not None and saving > 1e-6:           # boundary: just below / just above the saving
            boundary_n += 1
            lo = drive(p, Live(), ev, threshold=saving - 1e-9)["changes"] == 1
            hi = drive(p, Live(), ev, threshold=saving + 1e-9)["changes"] == 0
            boundary_ok += lo and hi
check("drive() switches exactly when saving > 10% (equation 12)", agree == total, f"{agree}/{total} cases; {switch_cases} switched, {below_cases} had a saving of 0-10% and correctly stayed")
check("boundary: switch just below the saving, stay just above", boundary_ok == boundary_n, f"{boundary_ok}/{boundary_n}")

# =====================================================================
print("\n=== 5) Blocked route forces rerouting; reroute starts from current position ===")
BIG = 1e9
forced_ok = forced_n = stuck = same_as_default = 0
static_done = reroute_done = 0
for s, d, p in PAIRS:
    eid, te = edge_at(p, 0.3), event_time(p, 0.3)
    ev = [(te, lambda L, eid=eid: L.close_street(eid[0], eid[1]))]
    i = state_at_event(p, te)
    Lc = Live(); Lc.close_street(eid[0], eid[1])
    reachable = live_search(Lc, p[i][0], d)[0] is not None
    r_big = drive(p, Live(), ev, threshold=BIG)
    r_def = drive(p, Live(), ev, threshold=0.10)
    r_sta = drive(p, Live(), ev, mode="static")
    static_done += r_sta["finished"]
    reroute_done += r_def["finished"]
    forced_n += 1
    if reachable:
        forced_ok += r_big["finished"] and r_big["changes"] == 1
        same_as_default += abs(r_big["time"] - r_def["time"]) < 1e-9
    else:
        stuck += 1
        forced_ok += (not r_big["finished"])           # nothing to switch to: cannot finish, and says so
        same_as_default += 1
check("closure forces a switch even with threshold = 1e9 (when a detour exists)", forced_ok == forced_n,
      f"{forced_ok}/{forced_n}; {stuck} trips have NO detour (destination cut off by the one-way closure)")
check("forced switch gives the same result at threshold 10% and 1e9", same_as_default == forced_n, f"{same_as_default}/{forced_n}")
check("rerouting finishes more trips than static", reroute_done > static_done, f"static {static_done}/200 -> reroute {reroute_done}/200")

behind_ok = 0
for s, d, p in PAIRS:
    ev = [(event_time(p, 0.6), lambda L, e=p[0]: L.close_street(e[0], e[1]))]       # closure BEHIND the ambulance
    r = drive(p, Live(), ev)
    behind_ok += r["changes"] == 0 and r["finished"]
check("closing a road already driven does not trigger a change", behind_ok == len(PAIRS), f"{behind_ok}/{len(PAIRS)}")

# start from the current position, checked on the recorded trips
start_ok = chain_ok = prefix_ok = n_changes = 0
trips = []
for s, d, p in PAIRS:
    eid = edge_at(p, 0.3)
    ev = [(event_time(p, 0.3), lambda L, eid=eid: L.close_street(eid[0], eid[1]))]
    tr = run_trip(p, ev)
    trips.append((f"{len(trips):03d}:{s}->{d}", tr))
    routes, pos = tr["routes"], tr["positions"]
    for v in range(1, len(routes)):
        n_changes += 1
        old, new, ch = routes[v - 1]["edges"], routes[v]["edges"], tr["changes"][v - 1]
        k = next(j for j, q in enumerate(pos) if q["time_s"] == ch["time_s"] and q["node"] == ch["at_node"])
        start_ok += new[k][0] == ch["at_node"]
        prefix_ok += new[:k] == old[:k]
        chain_ok += is_chain(new) and new[0][0] == s and new[-1][1] == d
check("new route starts at the node where the ambulance is", start_ok == n_changes, f"{start_ok}/{n_changes}")
check("already-driven edges are never rewritten", prefix_ok == n_changes, f"{prefix_ok}/{n_changes}")
check("every new route is a connected path src -> dst", chain_ok == n_changes, f"{chain_ok}/{n_changes}")

# =====================================================================
print("\n=== 6) Export (positions, next edge, upcoming signal, ETA) ===")
ids_unique = len({t for t, _ in trips}) == len(trips)
check("trip ids are unique", ids_unique, f"{len({t for t, _ in trips})}/{len(trips)}")
check("PAIRS contain a repeated (s,d) pair (why ids need a running number)", len({(s, d) for s, d, _ in PAIRS}) < len(PAIRS), f"{len(PAIRS) - len({(s, d) for s, d, _ in PAIRS})} repeat")

fmt_ok = order_ok = in_E_ok = next_ok = sig_ok = 0
n_routes = n_pos = n_sig = 0
for (tid, tr), (s, d, _) in zip(trips, PAIRS):
    for rt in tr["routes"]:
        n_routes += 1
        order_ok += rt["edge_ids"] == [eid_str(e) for e in rt["edges"]] and is_chain(rt["edges"])
        in_E_ok += all(e in E for e in rt["edges"])
        fmt_ok += all(tuple(map(int, x.split("-"))) == e for x, e in zip(rt["edge_ids"], rt["edges"]))
    for q in tr["positions"]:
        n_pos += 1
        cur = [r for r in tr["routes"] if r["time_s"] <= q["time_s"]][-1]        # route in force at that moment
        if q["next_edge_id"]:
            k = [j for j, e in enumerate(cur["edges"]) if eid_str(e) == q["next_edge_id"]]
            next_ok += bool(k) and cur["edges"][k[0]][0] == q["node"]
        else:
            next_ok += 1                                                           # last row: nothing left to drive
        if q["signal_node"] != "":
            n_sig += 1
            a, x = (int(t) for t in q["signal_approach_edge_id"].split("-")[1:2] + q["signal_exit_edge_id"].split("-")[:1])
            sig_ok += (q["signal_node"] in SIGNALS and a == q["signal_node"] == x
                       and q["signal_eta_s"] != "" and float(q["signal_eta_s"]) >= 0)
check("exported edge IDs are ordered, connected and parse back to (u,v,k)", order_ok == n_routes and fmt_ok == n_routes, f"{order_ok}/{n_routes} route versions")
check("all exported edge IDs exist in Model A", in_E_ok == n_routes)
check("position rows: next edge belongs to the route in force and starts at the current node", next_ok == n_pos, f"{next_ok}/{n_pos} rows")
check("signal rows: verified signal, approach ends at it, exit starts at it, ETA >= 0", sig_ok == n_sig, f"{sig_ok}/{n_sig} rows with a signal ({n_pos - n_sig} rows have none ahead)")

# ETA consistent with the simulated clock, with Model D deltas loaded
from task3_rerouting import demo_signal_deltas
eta_ok = eta_n = 0
for s, d, p in PAIRS[:100]:
    live = Live(); live.set_signal_deltas(demo_signal_deltas())
    tr = run_trip(p, [], live=live)
    P = tr["positions"]
    for a, b in zip(P, P[1:]):
        if a["signal_node"] != "" and a["signal_node"] == b["signal_node"] and a["signal_approach_edge_id"] == b["signal_approach_edge_id"]:
            eta_n += 1
            eta_ok += abs((float(a["signal_eta_s"]) - float(b["signal_eta_s"])) - (b["time_s"] - a["time_s"])) < 0.02
check("ETA falls by exactly the elapsed time between two positions (deltas loaded)", eta_ok == eta_n, f"{eta_ok}/{eta_n}")

# CSV round-trip
with tempfile.TemporaryDirectory() as tmp:
    try:
        export_csv([("dup", trips[0][1]), ("dup", trips[0][1])], folder=tmp); dup_rejected = False
    except ValueError:
        dup_rejected = True
    check("export_csv rejects duplicate trip ids", dup_rejected)
    export_csv(trips, folder=tmp)
    pos_df = pd.read_csv(f"{tmp}/trip_positions.csv")
    rt_df = pd.read_csv(f"{tmp}/trip_routes.csv")
    ch_df = pd.read_csv(f"{tmp}/route_changes.csv")
    check("CSV row counts match the in-memory trips",
          len(pos_df) == sum(len(t["positions"]) for _, t in trips)
          and len(rt_df) == sum(len(r["edges"]) for _, t in trips for r in t["routes"])
          and len(ch_df) == sum(len(t["changes"]) for _, t in trips),
          f"positions {len(pos_df)}, route edges {len(rt_df)}, changes {len(ch_df)}")
    check("trip_routes: derived_reverse all 0", int(rt_df["derived_reverse"].sum()) == 0)
    check("trip_routes: 'order' runs 0..n-1 inside each route version",
          all((g["order"].to_numpy() == range(len(g))).all() for _, g in rt_df.groupby(["trip_id", "route_version"])))
    check("trip_routes: edge_id == from-to-key columns",
          (rt_df["edge_id"] == rt_df["from_node"].astype(str) + "-" + rt_df["to_node"].astype(str) + "-" + rt_df["key"].astype(str)).all())

with open("validation_results.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["test", "value", "status"]); w.writerows(ROWS)
fails = [r for r in ROWS if r[2] == "FAIL"]
print(f"\n{sum(r[2]=='PASS' for r in ROWS)} passed, {len(fails)} failed  -> validation_results.csv")
