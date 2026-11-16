"""
run_model_d.py (Salwa): runs Model D (eta_priority.py) with Haneen's SignalController on Model C's trips.

For every trip that arrives at a usable signalized node (node 142 in the current files) it steps through the
trip once per `dt` seconds:  Model C position/route  ->  ETA + request time  ->  Haneen's controller.

Outputs (in --out):
    model_d_requests.csv       every request submitted to the controller
    model_d_controller.csv     controller results whenever a request's status changes, and at release
    model_d_trip_summary.csv   one row per trip and signal: ETA accuracy, timing, delays, flags

Example:
    python run_model_d.py --model-a RouteX_Model_A_Matrices.xlsx --model-b Model_B_for_Model_D.csv \\
        --signals-clean RouteX_Signalized_Intersections_Clean.csv \\
        --signals-approaches RouteX_Model_D_Approaches_Template.csv \\
        --routes trip_routes.csv --positions trip_positions.csv --haneen <folder with signal_controller.py>
"""
import argparse
import csv
import os
import sys

from eta_priority import (AssumedQueue, ClearanceSource, ModelD, load_model_a_edges, load_model_b_csv,
                          load_signal_mapping, load_trips, run_controller_cycle)

# ---------------------------------------------------------------------------------------------------------
# ASSUMPTIONS for inputs that do not exist yet. Replace each with real data as soon as it is available.
# ---------------------------------------------------------------------------------------------------------
ASSUMPTIONS = {
    "sim_start_s": 25200.0,        # simulation time 0 = 07:00 (Model B first supplied bin)
    # Provisional constant arrival rate applied to EVERY mapped approach unless overridden below.
    # These are synthetic assumptions, not measured Dokki arrival rates.
    "default_lam_veh_s": 0.10,
    "lam_by_approach": {},         # optional per-approach overrides, e.g. {"J142_IN_S": 0.10}
    "red_s": 30.0,                 # assumed red duration for queue estimate (s)
    "l_veh": 7.5,                  # assumed vehicle spacing in queue (m)
    "theta_eta_s": 2.0,            # re-submit threshold (s), provisional
    # No agreed initial phase/green was supplied. Use the first mapped approach in stable CSV order.
    "initial_green_by_node": {},  # optional explicit override once the signal configuration is agreed
    # Haneen's controller default is 30 s per approach; retain as an explicit provisional cycle assumption.
    "cycle_green_s": 30.0,
    "dt": 1.0,                     # simulation step (s), provisional harness setting
    # Haneen controller defaults: yellow=3 s, all-red=2 s (clearance=5 s); not overridden here.
}


def _approach_lam(cfg, approach_name):
    overrides = cfg.get("lam_by_approach", {})
    value = overrides.get(approach_name, cfg["default_lam_veh_s"])
    value = float(value)
    if not (value >= 0.0 and value < float("inf")):
        raise ValueError(f"invalid assumed arrival rate for {approach_name}: {value}")
    return value



def make_intersection(Intersection, node, signals, model_a, cfg):
    """Build every mapped incoming approach using Model A total road lanes_numeric.

    Movement-specific lane counts are intentionally not required: total incoming-road lanes
    are used as the project's explicit modelling assumption for capacity and queue clearance.
    """
    pairs = [(edge, name) for (n, edge), name in signals.approach_names.items() if n == node]
    if not pairs:
        raise ValueError(f"signalized intersection {node} has no mapped incoming approaches")
    approaches = {}
    for edge, name in pairs:
        lanes = model_a[edge].lanes_total
        if lanes is None or int(lanes) <= 0:
            raise ValueError(f"Model A lanes_numeric must be positive for incoming edge {edge} at node {node}")
        approaches[name] = dict(lam=_approach_lam(cfg, name), n=int(lanes),
                                 L_e=model_a[edge].length_m)
    preferred = cfg.get("initial_green_by_node", {}).get(node)
    # Stable mapping order is used when no agreed initial green is configured.
    first = preferred if preferred in approaches else next(iter(approaches))
    return Intersection(node, approaches, green=first,
                        cycle_green_s={a: float(cfg["cycle_green_s"]) for a in approaches})


def mismatch_ratio(trips, model_a, tt):
    """Total Model B time / total time in Model C's trips, over the roads driven."""
    b = c = 0.0
    for t in trips:
        for r0, r1 in zip(t.track, t.track[1:]):
            if r0[3] is not None:
                b += tt.get(r0[3], r0[0])[0]
                c += r1[0] - r0[0]
    return b / c if c else 1.0


def run(args, cfg=ASSUMPTIONS):
    sys.path.insert(0, args.haneen)
    from signal_controller import Intersection, SignalController

    A = load_model_a_edges(args.model_a)
    tt = load_model_b_csv(args.model_b, A, cfg["sim_start_s"])
    signals = load_signal_mapping(args.signals_clean, args.signals_approaches, A)
    trips = load_trips(args.routes, args.positions, A)
    # Build a complete assumed arrival-rate map for every approach at every usable signal.
    lam_by_approach = {name: _approach_lam(cfg, name) for name in signals.approach_names.values()}
    queue = AssumedQueue(signals, A, lam_by_approach, cfg["red_s"], cfg["l_veh"])

    chosen = [t for t in trips.values()
              if any(t.arrival_time_at(n) is not None for n in signals.nodes)]
    if args.max_trips:
        chosen = chosen[:args.max_trips]
    ratio = mismatch_ratio(chosen, A, tt)
    if args.ambulance_times == "model_b":
        chosen = [t.retimed(A, tt) for t in chosen]
    elif not 0.8 <= ratio <= 1.25:
        print(f"WARNING: Model B travel times are {ratio:.1f}x the road times in Model C's trips, so ETAs will be "
              f"off by about that factor until the ambulance is close. Use --ambulance-times model_b to drive the "
              f"ambulance at Model B's times, or ask Model B and C to use the same times.\n")
    os.makedirs(args.out, exist_ok=True)
    req_rows, ctl_rows, summary = [], [], []

    for trip in chosen:
        inters = [make_intersection(Intersection, n, signals, A, cfg) for n in sorted(signals.nodes)]
        clear = ClearanceSource.from_intersections(inters)
        ctrl = SignalController(inters)
        md = ModelD(trip.trip_id, A, signals, tt, queue, clear, trip.route_versions[0][1],
                    theta_eta_s=cfg["theta_eta_s"])
        pending = list(trip.route_versions[1:])
        first_req, last_req, seen_status, stats = {}, {}, {}, {}
        # Start at the first recorded Model C position (not an assumed global zero).
        t = trip.track[0][0]
        while True:
            amb = trip.state_at(t, A)
            if amb is None:
                break
            new_route = None
            while pending and pending[0][0] <= t:
                new_route = pending.pop(0)[1]
            res = md.step(t, amb, new_full_route=new_route)
            for r in res.requests:
                row = dict(trip_id=trip.trip_id, t_s=t, **r)
                req_rows.append(row)
                first_req.setdefault(r["node_id"], row)
                last_req[r["node_id"]] = row
            for out in run_controller_cycle(ctrl, res, t):
                node = out["node_id"]
                st = stats.setdefault(node, dict(max_ev_delay=0.0, ordinary=0.0, flags=set(), statuses=[]))
                st["max_ev_delay"] = max(st["max_ev_delay"], out["ev_signal_delay_s"])
                st["ordinary"] = max(st["ordinary"], out["ordinary_delay_veh_s"])
                st["flags"].update(out["flags"])
                if seen_status.get(node) != out["status"] or out["status"] == "RELEASED":
                    seen_status[node] = out["status"]
                    st["statuses"].append(out["status"])
                    ctl_rows.append(dict(trip_id=trip.trip_id, t_s=t, node_id=node, status=out["status"],
                                         phase=out["phase"], feasible=out["feasible"],
                                         ev_signal_delay_s=out["ev_signal_delay_s"],
                                         ordinary_delay_veh_s=out["ordinary_delay_veh_s"],
                                         t_rec_s=out["t_rec_s"], flags="|".join(out["flags"])))
            t += cfg["dt"]

        for node, fr in first_req.items():
            actual = trip.arrival_time_at(node)
            lr, st = last_req[node], stats.get(node, dict(max_ev_delay=0.0, ordinary=0.0, flags=set(), statuses=[]))
            summary.append(dict(
                trip_id=trip.trip_id, node_id=node, ev_approach=fr["ev_approach"],
                route_changed=len(trip.route_versions) > 1,
                first_request_t_s=fr["t_s"], first_eta_s=fr["eta_j_s"], first_request_time_s=fr["request_time_j_s"],
                first_feasible=fr["feasible"], last_eta_s=lr["eta_j_s"], actual_arrival_s=actual,
                first_eta_error_s=fr["eta_j_s"] - actual if actual is not None else "",
                last_eta_error_s=lr["eta_j_s"] - actual if actual is not None else "",
                statuses=">".join(st["statuses"]), max_ev_signal_delay_s=st["max_ev_delay"],
                peak_predicted_ordinary_delay_veh_s=st["ordinary"], flags="|".join(sorted(st["flags"]))))

    def dump(name, rows):
        if rows:
            with open(os.path.join(args.out, name), "w", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)

    dump("model_d_requests.csv", req_rows)
    dump("model_d_controller.csv", ctl_rows)
    dump("model_d_trip_summary.csv", summary)
    return summary


def print_report(summary, cfg=ASSUMPTIONS):
    print("ASSUMPTIONS USED:")
    for k, v in cfg.items():
        print(f"  {k} = {v}")
    n = len(summary)
    if not n:
        print("\nNo trip reached a usable signalized intersection.")
        return
    err = [abs(r["first_eta_error_s"]) for r in summary if r["first_eta_error_s"] != ""]
    late = sum(1 for r in summary if not r["first_feasible"])
    print(f"\n{n} trip arrivals at a usable signal")
    print(f"  mean |first ETA error| = {sum(err) / len(err):.1f} s   max = {max(err):.1f} s")
    print(f"  first request already late (not feasible): {late} of {n}")
    print(f"  mean ambulance signal delay = {sum(r['max_ev_signal_delay_s'] for r in summary) / n:.1f} s")
    print(f"  mean peak predicted ordinary-traffic cost = {sum(r['peak_predicted_ordinary_delay_veh_s'] for r in summary) / n:.1f} veh-s")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--model-a", default="RouteX_Model_A_Matrices.xlsx")
    p.add_argument("--model-b", default="Model_B_for_Model_D.csv")
    p.add_argument("--signals-clean", default="RouteX_Signalized_Intersections_Clean.csv")
    p.add_argument("--signals-approaches", default="RouteX_Model_D_Approaches_Template.csv")
    p.add_argument("--routes", default="trip_routes.csv")
    p.add_argument("--positions", default="trip_positions.csv")
    p.add_argument("--haneen", default=".", help="folder containing signal_controller.py and traffic_delay.py")
    p.add_argument("--out", default="model_d_output")
    p.add_argument("--max-trips", type=int, default=0)
    p.add_argument("--ambulance-times", choices=["as_given", "model_b"], default="as_given",
                   help="as_given: use Model C's trip_positions times; model_b: re-time every road with Model B")
    a = p.parse_args()
    print_report(run(a))
