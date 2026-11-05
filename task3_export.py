"""ROUTEX - C Model - interface for the other models: live ambulance position,
ordered route edge IDs, and route-change updates.

Run:  python task3_export.py        (needs the other task files and the xlsx next to it)
Writes, into the folder  model_c_output/ :
  trip_positions.csv  - where the ambulance is, step by step
  trip_routes.csv     - the ordered list of road IDs of the route in force (one row per road)
  route_changes.csv   - one row every time the route changed, and why

Road ID = "u-v-key" = the (u, v, key) of the row in the Model A Edges sheet. For a two-way street
the same ID is used in both directions; the `direction` column says which way the ambulance
drives it (forward = u->v as in the sheet, reverse = v->u).

Or from code (for Model D / other models):
    from task3_export import run_trip
    trip = run_trip(route, events)   # -> dict with 'positions', 'routes', 'changes', 'summary'
"""
import csv
import os
from task1_dijkstra import edges
from task3_rerouting import Live, drive, make_pairs, edge_at, event_time

# ---------- road IDs from the Model A sheet ----------
# Same rule as task1_dijkstra.py: for parallel roads between two intersections the fastest one is kept
# (a tie keeps the first row), so each (u, v) used by the routes maps to exactly one row of the sheet.
_ref, _best = {}, {}


def _set(u, v, t, rid, d):
    if (u, v) not in _best or t < _best[(u, v)]:
        _best[(u, v)] = t
        _ref[(u, v)] = (rid, d)


for r in edges.itertuples():
    rid = f"{int(r.u)}-{int(r.v)}-{int(r.key)}"
    _set(int(r.u), int(r.v), r.free_flow_s, rid, "forward")
    if not r.oneway:
        _set(int(r.v), int(r.u), r.free_flow_s, rid, "reverse")


def edge_id(u, v):
    """(road ID, direction) of the directed road u -> v."""
    return _ref[(u, v)]


def route_edges(route):
    """Ordered road IDs of a route given as a list of intersections."""
    return [edge_id(a, b)[0] for a, b in zip(route, route[1:])]


# ---------- one trip, recorded ----------
def run_trip(route, events, threshold=0.10, use_h=True):
    """Drive one trip with rerouting on and return everything the other models need."""
    log = {}
    res = drive(route, Live(), events, mode="reroute", threshold=threshold, use_h=use_h, log=log)
    return dict(
        positions=[dict(time_s=round(t, 2), node=n,
                        next_edge_id=edge_id(n, nx_)[0] if nx_ is not None else "",
                        next_node=nx_ if nx_ is not None else "")
                   for t, n, nx_ in log["positions"]],
        routes=[dict(time_s=round(t, 2), reason=why, edge_ids=route_edges(rt), nodes=rt)
                for t, rt, why, _, _ in log["routes"]],
        changes=[dict(time_s=round(t, 2), at_node=n, reason=why,
                      remaining_on_old_s="" if rem is None else round(rem, 2), new_route_s=round(new, 2))
                 for t, n, why, rem, new in log["changes"]],
        summary=dict(finished=log["finished"], total_time_s=round(res["time"], 2), changes=res["changes"]),
    )


# ---------- write the CSV files ----------
def export_csv(trips, folder="model_c_output"):
    """trips: list of (trip_id, run_trip result)."""
    os.makedirs(folder, exist_ok=True)
    with open(f"{folder}/trip_positions.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip_id", "time_s", "node", "next_node", "next_edge_id"])
        for tid, tr in trips:
            for p in tr["positions"]:
                w.writerow([tid, p["time_s"], p["node"], p["next_node"], p["next_edge_id"]])
    with open(f"{folder}/trip_routes.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip_id", "route_version", "valid_from_s", "reason", "order", "from_node", "to_node",
                    "edge_id", "direction"])
        for tid, tr in trips:
            for ver, rt in enumerate(tr["routes"]):
                for k, (a, b) in enumerate(zip(rt["nodes"], rt["nodes"][1:])):
                    eid, d = edge_id(a, b)
                    w.writerow([tid, ver, rt["time_s"], rt["reason"], k, a, b, eid, d])
    with open(f"{folder}/route_changes.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trip_id", "time_s", "at_node", "reason", "remaining_on_old_route_s", "new_route_s"])
        for tid, tr in trips:
            for c in tr["changes"]:
                w.writerow([tid, c["time_s"], c["at_node"], c["reason"], c["remaining_on_old_s"], c["new_route_s"]])


if __name__ == "__main__":
    pairs = make_pairs()                      # same 200 pairs as the Task 3 report
    trips = []
    for k, (s, d, p) in enumerate(pairs):
        u, v = edge_at(p, d, 0.3)             # demo event: a road closes 30% into the trip
        ev = [(event_time(p, 0.3), lambda L, u=u, v=v: L.close_street(u, v))]
        trips.append((f"{s}->{d}", run_trip(p, ev)))
    export_csv(trips)
    ch = sum(len(t["changes"]) for _, t in trips)
    print(f"{len(trips)} trips written to model_c_output/  ({ch} route changes)")
    tid, tr = trips[0]
    print(f"\nexample trip {tid}: {tr['summary']}")
    for c in tr["changes"]:
        print("  change:", c)
    print("  first 3 positions:", tr["positions"][:3])
    print("  route in force at the end:", tr["routes"][-1]["edge_ids"][:5], "...")
