"""ROUTEX - C Model - interface for the other models (Dijkstra only): live ambulance position,
ordered route edge IDs, route-change updates, and the intended approach at the next signal (for Model D).

Run:  python task3_export.py        (needs the other task files and the xlsx next to it)
Writes, into the folder  model_c_output/ :
  trip_positions.csv  - where the ambulance is, step by step: current node, next directed edge, next signal
  trip_routes.csv     - the ordered directed edge IDs of the route in force, one row per edge, per route version
  route_changes.csv   - one row every time the route changed, and why

Directed edge ID = "u-v-k" = the (u, v, key) of ONE row of the Model A Edges sheet. The ID already fixes the
direction (u -> v); parallel roads between the same intersections have different k and are never merged.
`derived_reverse` is 1 only if the sheet had no row for the opposite direction of a two-way street.

Upcoming signal (Model D): signal_node, the APPROACH edge arriving at it, the EXIT edge the ambulance will take
leaving it, and the expected time to reach it (signal_eta_s).

Or from code (for Model D / other models):
    from task3_export import run_trip
    trip = run_trip(route, events, live=my_live)   # -> dict with 'positions', 'routes', 'changes', 'summary'
    (my_live is a Live() that already holds Model D's Delta T_signal via set_signal_deltas())
"""
import csv
import os
from task1_dijkstra import E
from task3_rerouting import Live, drive, make_pairs, edge_at, event_time


def eid_str(e):
    return f"{e[0]}-{e[1]}-{e[2]}"


def route_edges(route):
    """Ordered directed edge IDs ('u-v-k') of a route."""
    return [eid_str(e) for e in route]


def _num(x, nd=2):
    return "" if x is None else round(x, nd)


# ---------- one trip, recorded ----------
def run_trip(route, events, threshold=0.10, live=None):
    """Drive one trip with rerouting on (Dijkstra) and return everything the other models need."""
    log = {}
    res = drive(route, live if live is not None else Live(), events, mode="reroute", threshold=threshold, log=log)

    def pos(t, node, eid, sig):
        return dict(time_s=round(t, 2), node=node,
                    next_node=eid[1] if eid else "",
                    next_edge_id=eid_str(eid) if eid else "",
                    signal_node=sig["node"] if sig else "",
                    signal_approach_edge_id=eid_str(sig["approach"]) if sig else "",
                    signal_exit_edge_id=eid_str(sig["exit"]) if sig else "",
                    signal_eta_s=_num(sig["eta_s"]) if sig else "")

    return dict(
        positions=[pos(*p) for p in log["positions"]],
        routes=[dict(time_s=round(t, 2), reason=why, edge_ids=route_edges(rt), edges=rt)
                for t, rt, why, _, _ in log["routes"]],
        changes=[dict(time_s=round(t, 2), at_node=n, reason=why,
                      remaining_on_old_s=_num(rem), new_route_s=round(new, 2))
                 for t, n, why, rem, new in log["changes"]],
        summary=dict(finished=log["finished"], total_time_s=round(res["time"], 2), changes=res["changes"]),
    )


# ---------- write the CSV files ----------
def export_csv(trips, folder="model_c_output"):
    """trips: list of (trip_id, run_trip result)."""
    os.makedirs(folder, exist_ok=True)
    with open(f"{folder}/trip_positions.csv", "w", newline="") as f:
        w = csv.writer(f)
        cols = ["time_s", "node", "next_node", "next_edge_id", "signal_node",
                "signal_approach_edge_id", "signal_exit_edge_id", "signal_eta_s"]
        w.writerow(["trip_id"] + cols)
        for tid, tr in trips:
            for p in tr["positions"]:
                w.writerow([tid] + [p[c] for c in cols])
    with open(f"{folder}/trip_routes.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip_id", "route_version", "valid_from_s", "reason", "order", "edge_id",
                    "from_node", "to_node", "key", "derived_reverse"])
        for tid, tr in trips:
            for ver, rt in enumerate(tr["routes"]):
                for k, e in enumerate(rt["edges"]):
                    w.writerow([tid, ver, rt["time_s"], rt["reason"], k, eid_str(e), e[0], e[1], e[2],
                                int(E[e]["derived"])])
    with open(f"{folder}/route_changes.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip_id", "time_s", "at_node", "reason", "remaining_on_old_route_s", "new_route_s"])
        for tid, tr in trips:
            for c in tr["changes"]:
                w.writerow([tid, c["time_s"], c["at_node"], c["reason"], c["remaining_on_old_s"], c["new_route_s"]])


if __name__ == "__main__":
    pairs = make_pairs()                      # same 200 pairs as the Task 3 report
    trips = []
    for s, d, p in pairs:
        eid = edge_at(p, 0.3)                 # demo event: that road closes (one direction) 30% into the trip
        ev = [(event_time(p, 0.3), lambda L, eid=eid: L.close_street(eid[0], eid[1]))]
        trips.append((f"{s}->{d}", run_trip(p, ev)))
    export_csv(trips)
    ch = sum(len(t["changes"]) for _, t in trips)
    print(f"{len(trips)} trips written to model_c_output/  ({ch} route changes)")
    tid, tr = trips[0]
    print(f"\nexample trip {tid}: {tr['summary']}")
    for c in tr["changes"]:
        print("  change:", c)
    print("  first 2 positions:", tr["positions"][:2])
    print("  route in force at the end:", tr["routes"][-1]["edge_ids"][:5], "...")
