import math
from traffic_delay import approach_cost, intersection_cost
from signal_controller import *

def mk(green="N", **kw):
    ap = {"N": dict(lam=0.1, n=1, L_e=100), "E": dict(lam=0.2, n=1, L_e=100)}
    return SignalController([Intersection(1, ap, green, **kw)], G_max=40)

def req(ev="a", eta=60, rt=41, feas=True, ap="N", sig=4):
    return dict(ev_id=ev, node_id=1, ev_approach=ap, eta_j_s=eta, sigma_eta_s=sig, request_time_j_s=rt, feasible=feas)

def test_cost_worked_example():
    c = approach_cost(0.2, 15, 1, 100)
    assert abs(c["delay_veh_s"] - 37.5) < 1e-9 and abs(c["t_rec_s"] - 10) < 1e-9   # corrected (doc's 31.5 omitted drain term)
    assert c["queue_veh"] == 3 and not c["spillback"]

def test_no_recovery_and_spillback():
    assert "no_recovery" in intersection_cost(10, [dict(lam=0.5, n=1, L_e=100)])["flags"]
    assert "spillback" in intersection_cost(100, [dict(lam=0.2, n=1, L_e=100)])["flags"]

def test_R1_before_request():
    c = mk(); c.submit(req()); r = c.step(10)[0]
    assert r["status"] == NORMAL and r["ev_signal_delay_s"] == 0

def test_R2_green_hold():
    c = mk("N"); c.submit(req()); r = c.step(41)[0]
    assert r["status"] == ACTIVE and r["ev_signal_delay_s"] == 0 and r["ordinary_delay_veh_s"] > 0

def test_R3_red_clearance_and_ped():
    c = mk("E", ped_min_end_s=45); c.submit(req(eta=70)); c.step(41)
    I = c.I[1]; assert I.plan[1] == 45 and I.plan[2] == 50            # ped min then 5 s clearance
    assert [I.phase(t)[0] for t in (44, 46, 49, 51)] == ["GREEN", "YELLOW", "ALL_RED", "GREEN"]
    assert I.phase(46)[1] is None and I.phase(49)[1] is None          # never two greens
    assert c.step(41)[0]["ev_signal_delay_s"] == 0

def test_R4_late():
    c = mk("E"); c.submit(req(eta=42, rt=30, feas=False)); r = c.step(40)[0]
    assert r["status"] == BEST_EFFORT and "warn_planner" in r["flags"] and r["ev_signal_delay_s"] == 3

def test_R5_release_passed_and_timeout():
    c = mk(); c.submit(req()); c.step(41)
    assert c.step(50, passed={("a", 1)})[0]["status"] == RELEASED and not c.reqs
    c.submit(req()); assert c.step(60 + 2*4 + 1)[0]["status"] == RELEASED

def test_R6_conflict_earliest_eta():
    c = mk("N"); c.submit(req("a", eta=60)); c.submit(req("b", eta=62, ap="E", rt=41))
    s = {r["ev_id"]: r for r in c.step(41)}
    assert s["a"]["status"] == ACTIVE and s["b"]["status"] == DEFERRED and s["b"]["ev_signal_delay_s"] > 0

def test_R7_eta_update_and_cancel():
    c = mk("E"); c.submit(req(eta=60, rt=41)); c.step(30)
    c.submit(req(eta=90, rt=71)); assert c.step(41)[0]["status"] == NORMAL    # new request replaces old
    c.submit(req(eta=60, rt=41)); c.step(42); res = c.cancel("a", 1, 43)       # started -> clearance completes
    assert res["status"] == RELEASED and c.I[1].plan is not None

def test_cancel_before_switch_aborts():
    c = mk("E", ped_min_end_s=50); c.submit(req(eta=70)); c.step(41)
    c.cancel("a", 1, 42); assert c.I[1].plan is None

def test_gmax_cap_flag():
    c = mk("N"); c.G_max = 10; c.submit(req(eta=60, rt=41)); c.I[1].green_since_s = 0
    assert "G_max_capped" in c.step(41)[0]["flags"]

# ---------- tests for the four fixes ----------
def test_gmax_enforced():
    c = mk("N"); c.G_max = 10; c.I[1].green_since_s = 0
    c.submit(req(eta=60, rt=41)); assert c.step(41)[0]["status"] == ACTIVE
    r = c.step(52)[0]
    assert r["status"] == RELEASED and "G_max_capped" in r["flags"] and not c.reqs
    I = c.I[1]
    assert [I.phase(t)[0] for t in (50.9, 51, 54, 56)] == ["GREEN", "YELLOW", "ALL_RED", "GREEN"]
    assert I.phase(58)[1] == "E"                                      # green really left N

def test_gmax_gives_green_to_waiting_conflicting_ev():
    c = mk("N"); c.G_max = 10; c.I[1].green_since_s = 0
    c.submit(req("a", eta=60, rt=41, ap="N")); c.submit(req("b", eta=70, rt=41, ap="E"))
    s = {r["ev_id"]: r for r in c.step(41)}
    assert s["a"]["status"] == ACTIVE and s["b"]["status"] == DEFERRED
    s = {r["ev_id"]: r for r in c.step(52)}
    assert s["a"]["status"] == RELEASED and s["b"]["status"] == ACTIVE and c.I[1].plan[0] == "E"

def test_R7_unstarted_plan_dropped_on_resubmit():
    c = mk("E", ped_min_end_s=50); c.submit(req(eta=70, rt=41)); c.step(41)
    assert c.I[1].plan == ("N", 50, 55, "a")
    c.submit(req(eta=100, rt=71))                                     # ETA pushed later
    assert c.I[1].plan is None
    assert c.step(60)[0]["status"] == NORMAL and c.I[1].phase(60) == ("GREEN", "E")

def test_R7_started_clearance_then_back_to_cycle():
    c = mk("E"); c.submit(req(eta=60, rt=41)); c.step(41)             # clearance already started: not aborted
    c.submit(req(eta=200, rt=181))
    assert c.step(47)[0]["status"] == NORMAL and c.I[1].green == "N"
    c.step(90); assert c.I[1].phase(90) == ("GREEN", "E")             # normal cycle moved on, not stuck on N

def test_R6_started_plan_keeps_owner():
    c = mk("E"); c.submit(req("a", eta=60, rt=41, ap="N")); c.step(41)
    c.submit(req("b", eta=55, rt=41, ap="E"))
    s = {r["ev_id"]: r for r in c.step(42)}
    assert s["a"]["status"] == ACTIVE and s["b"]["status"] == DEFERRED   # never both deferred

def test_R6_earlier_eta_preempts_unstarted_plan():
    c = mk("E", ped_min_end_s=50); c.submit(req("a", eta=60, rt=41, ap="N")); c.step(41)
    c.submit(req("b", eta=55, rt=41, ap="E"))
    s = {r["ev_id"]: r for r in c.step(42)}
    assert s["b"]["status"] == ACTIVE and s["a"]["status"] == DEFERRED and c.I[1].plan is None

def test_R6_same_approach_shares_green():
    c = mk("N"); c.submit(req("a", eta=60, rt=41)); c.submit(req("c", eta=61, rt=41))
    assert {r["status"] for r in c.step(41)} == {ACTIVE}

def test_R6_not_due_request_does_not_block():
    c = mk("N"); c.submit(req("x", eta=50, rt=45, ap="N")); c.submit(req("b", eta=62, rt=41, ap="E"))
    s = {r["ev_id"]: r for r in c.step(41)}
    assert s["x"]["status"] == NORMAL and s["b"]["status"] == ACTIVE

def test_release_restores_normal_cycle():
    c = mk("N"); c.submit(req()); c.step(41)
    assert c.step(50, passed={("a", 1)})[0]["status"] == RELEASED
    I = c.I[1]; c.step(81)                                            # normal timer restarts at release: 50 + 30 s
    assert [I.phase(t)[0] for t in (79, 81, 84, 86)] == ["GREEN", "YELLOW", "ALL_RED", "GREEN"]
    assert I.phase(86)[1] == "E"

def test_custom_cycle_runs_without_priority():
    ap = {"N": dict(lam=0.1, n=1, L_e=100), "E": dict(lam=0.2, n=1, L_e=100)}
    c = SignalController([Intersection(1, ap, "N", cycle_green_s={"N": 20, "E": 40})])
    c.step(25); assert c.I[1].phase(30) == ("GREEN", "E")
    c.step(66); assert c.I[1].phase(66)[0] == "YELLOW"                # E green 25..65, then N



# ---------- regression tests for exact max-hold timing and EV delay ----------
def test_cap_estimates_remaining_ev_wait_not_zero():
    c = mk("N"); c.G_max = 10
    r = req(eta=60, rt=41, ap="N")
    c.submit(r)
    active = c.step(41)[0]
    assert active["status"] == ACTIVE
    assert active["ev_signal_delay_s"] == 31.0
    assert "post_cap_delay_estimated" in active["flags"]
    released = c.step(52)[0]
    assert released["status"] == RELEASED
    assert released["ev_signal_delay_s"] == 31.0
    assert "warn_planner" in released["flags"]
    assert released["extra_red_s"] == 10.0
    assert c.I[1].phase(60) == ("GREEN", "E")
    assert c.I[1].phase(90) == ("ALL_RED", None)
    assert c.I[1].phase(91) == ("GREEN", "N")

def test_cap_is_scheduled_without_subsequent_steps():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=60, rt=41))
    c.step(41)
    I = c.I[1]
    assert I.phase(50.9) == ("GREEN", "N")
    assert I.phase(51) == ("YELLOW", None)
    assert I.phase(54) == ("ALL_RED", None)
    assert I.phase(56) == ("GREEN", "E")

def test_sparse_step_uses_exact_cap_not_late_step_time():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=60, rt=41))
    c.step(41)
    released = c.step(70)[0]
    assert released["status"] == RELEASED
    assert released["extra_red_s"] == 10.0  # cap at 51, not at 70
    assert released["ev_signal_delay_s"] == 31.0
    assert c.I[1].phase(70) == ("GREEN", "E")

def test_repeated_steps_cannot_extend_cap():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=60, rt=41))
    c.step(41); c.step(43); c.step(47); c.step(49)
    assert c.I[1].hold_cap_s == 51.0
    assert c.step(51)[0]["status"] == RELEASED

def test_pass_before_cap_cancels_cutoff():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=60, rt=41)); c.step(41)
    assert c.step(48, passed={("a", 1)})[0]["status"] == RELEASED
    assert c.I[1].hold_cap_s is None
    assert c.I[1].phase(52) == ("GREEN", "N")
    assert c.I[1].phase(78) == ("YELLOW", None)

def test_cap_delay_is_zero_if_ev_arrived_while_green():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=50, rt=41)); c.step(41)
    r = c.step(52)[0]
    assert r["status"] == RELEASED and r["ev_signal_delay_s"] == 0.0


def test_release_event_has_actual_cutoff_timestamp_and_phase():
    c = mk("N"); c.G_max = 10
    c.submit(req(eta=60, rt=41)); c.step(41)
    r = c.step(90)[0]
    assert r["event_time_s"] == 51
    assert r["phase"] == "YELLOW"
    assert r["ev_signal_delay_s"] == 31.0


def test_pedestrian_safety_overrides_green_hold_cap():
    c = mk("N", ped_min_end_s=60); c.G_max = 10
    c.submit(req(eta=90, rt=41))
    r = c.step(41)[0]
    assert "pedestrian_min_overrides_G_max" in r["flags"]
    assert c.I[1].phase(55) == ("GREEN", "N")
    assert c.I[1].phase(60) == ("YELLOW", None)
    assert c.step(61)[0]["event_time_s"] == 60


def test_r3_cap_schedules_after_legal_clearance_and_sparse_step():
    c = mk("E"); c.G_max = 10
    c.submit(req(eta=60, rt=41)); first=c.step(41)[0]
    assert first["ev_signal_delay_s"] == 36.0
    assert c.I[1].phase(46) == ("GREEN", "N")
    assert c.I[1].phase(56) == ("YELLOW", None)
    r = c.step(75)[0]
    assert r["event_time_s"] == 56.0
    assert r["extra_red_s"] == 15.0
    assert r["ev_signal_delay_s"] == 36.0

def test_earlier_conflicting_ev_can_preempt_future_cap_transition():
    c = mk("N"); c.G_max = 10
    c.submit(req("a", eta=60, rt=41, ap="N")); c.step(41)
    assert c.I[1].plan[1] == 51  # projected cap-induced normal transition
    c.submit(req("b", eta=55, rt=41, ap="E"))
    states = {r["ev_id"]: r for r in c.step(42)}
    assert states["b"]["status"] == ACTIVE
    assert states["b"]["ev_signal_delay_s"] == 0
    assert states["a"]["status"] == DEFERRED
    assert c.I[1].plan[1] == 42  # new EV can start clearance immediately
    assert c.I[1].plan[2] == 47


if __name__ == "__main__":
    for n, f in list(globals().items()):
        if n.startswith("test_"): f(); print("ok", n)
