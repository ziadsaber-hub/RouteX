from dataclasses import dataclass
from traffic_delay import extra_red, intersection_cost

NORMAL = "NORMAL"
ACTIVE = "PRIORITY_ACTIVE"
BEST_EFFORT = "BEST_EFFORT"
DEFERRED = "DEFERRED"
RELEASED = "RELEASED"


@dataclass
class Intersection:
    node_id: int
    approaches: dict
    green: str
    green_since_s: float = 0.0
    ped_min_end_s: float = 0.0
    r_normal_s: float = 0.0
    t_yellow: float = 3.0
    t_allred: float = 2.0
    plan: tuple = None  # (approach, switch_s, green_s, ev_id); ev_id=None means normal switch
    cycle_green_s: dict = None
    default_green_s: float = 30.0
    normal_active: bool = None
    normal_from_s: float = 0.0
    hold_cap_s: float = None  # future *absolute* deadline, not next-step deadline
    hold_approach: str = None

    def __post_init__(self):
        if self.normal_active is None:
            self.normal_active = self.cycle_green_s is not None
        if self.green not in self.approaches:
            raise ValueError("initial green must be one of the intersection approaches")
        if self.t_yellow < 0 or self.t_allred < 0 or self.default_green_s <= 0:
            raise ValueError("clearance times must be nonnegative; normal green must be positive")
        if any(v <= 0 for v in self.cycle().values()):
            raise ValueError("all normal green durations must be positive")

    @property
    def t_clear(self):
        return self.t_yellow + self.t_allred

    def cycle(self):
        return self.cycle_green_s or {a: self.default_green_s for a in self.approaches}

    def green_time(self, approach):
        return self.cycle().get(approach, self.default_green_s)

    def next_approach(self, from_approach=None):
        names = list(self.cycle())
        current = self.green if from_approach is None else from_approach
        return names[0] if current not in names else names[(names.index(current) + 1) % len(names)]

    def _cycle_phase(self, t, approach, start_s):
        """Project a hypothetical normal cycle from a known green onset."""
        for _ in range(1000):
            due = start_s + self.green_time(approach)
            if t < due:
                return "GREEN", approach
            if t < due + self.t_yellow:
                return "YELLOW", None
            if t < due + self.t_clear:
                return "ALL_RED", None
            approach = self.next_approach(approach)
            start_s = due + self.t_clear
        raise RuntimeError("phase projection exceeded 1000 normal cycles")

    def phase(self, t):
        """(phase, green_approach), including scheduled cutoff without step()."""
        # While an EV green is held, a maximum-hold cutoff already has an exact
        # scheduled time. Project the subsequent normal cycle even if no step is called.
        if self.hold_cap_s is not None and t >= self.hold_cap_s:
            nxt = self.next_approach(self.hold_approach)
            if nxt == self.hold_approach:  # only one modeled approach
                return "GREEN", nxt
            ts = self.hold_cap_s
            if t < ts + self.t_yellow:
                return "YELLOW", None
            if t < ts + self.t_clear:
                return "ALL_RED", None
            return self._cycle_phase(t, nxt, ts + self.t_clear)

        if self.plan is not None:
            target, ts, tg, owner = self.plan
            if t < ts:
                return "GREEN", self.green
            if t < ts + self.t_yellow:
                return "YELLOW", None
            if t < tg:
                return "ALL_RED", None
            if self.normal_active and owner is None:
                return self._cycle_phase(t, target, tg)
            return "GREEN", target

        if self.normal_active:
            start_s = max(self.green_since_s, self.normal_from_s, self.ped_min_end_s)
            return self._cycle_phase(t, self.green, start_s)
        return "GREEN", self.green

    def predicted_wait_after_cap(self, ev_approach, eta_s, cap_s):
        """Expected red wait at ETA if the normal cycle resumes at the exact cap.

        Does not account for subsequent requests, traffic queues, or uncertain ETA.
        """
        if eta_s <= cap_s:
            return 0.0
        next_green = self.next_approach(self.hold_approach or ev_approach)
        if next_green == ev_approach:
            return max(0.0, cap_s + self.t_clear - eta_s)
        start = cap_s + self.t_clear
        current = next_green
        for _ in range(1000):
            end = start + self.green_time(current)
            if current == ev_approach and eta_s < end:
                return max(0.0, start - eta_s)
            if eta_s < end and current != ev_approach:
                pass
            current = self.next_approach(current)
            start = end + self.t_clear
            if current == ev_approach and start >= eta_s:
                return start - eta_s
        raise RuntimeError("green-wait projection exceeded 1000 cycles")


class SignalController:
    def __init__(self, intersections, G_max=40.0, C_max=float("inf"),
                 T_pass=3.0, s=0.5, l_veh=7.5, k=2.0):
        self.I = {i.node_id: i for i in intersections}
        self.reqs = {}
        self.G_max, self.C_max, self.T_pass = float(G_max), float(C_max), float(T_pass)
        self.s, self.l_veh, self.k = float(s), float(l_veh), float(k)
        self.now = float("-inf")
        if self.G_max <= 0 or self.T_pass < 0 or self.s <= 0 or self.l_veh <= 0 or self.k < 0:
            raise ValueError("Invalid timing/queueing configuration")

    def submit(self, r):
        """Insert or replace a future EV request (R7)."""
        n, ev = r["node_id"], r["ev_id"]
        I = self.I[n]
        if r["ev_approach"] not in I.approaches:
            raise ValueError("EV approach missing from intersection")
        self.reqs[(ev, n)] = dict(r)
        if I.plan and I.plan[3] == ev and self.now < I.plan[1]:
            I.plan = None  # old transition has not begun; safe to withdraw it
            I.normal_active, I.normal_from_s = True, max(I.normal_from_s, self.now)
        # An ETA revision may cancel the existing priority's cap; a new hold
        # starts when the request becomes due and is actually served again.
        if I.hold_approach == r["ev_approach"] and self.now < (I.hold_cap_s or float("-inf")):
            if r["request_time_j_s"] > self.now and I.plan and I.plan[3] is None and I.plan[1] == I.hold_cap_s:
                I.plan = None
            if r["request_time_j_s"] > self.now:
                I.hold_cap_s, I.hold_approach = None, None

    def cancel(self, ev_id, node_id, t):
        r = self.reqs.get((ev_id, node_id))
        return self._release(self.I[node_id], r, t) if r else None

    def _res(self, I, r, status, t):
        return dict(node_id=I.node_id, ev_id=r["ev_id"], status=status,
                    phase=I.phase(t)[0], event_time_s=t, feasible=r["feasible"],
                    ev_signal_delay_s=0.0, ordinary_delay_veh_s=0.0,
                    t_rec_s=0.0, flags=[])

    def _cost(self, I, r, approach, end_s, res):
        dr = extra_red(r["request_time_j_s"], end_s, I.r_normal_s)
        c = intersection_cost(dr, [v for k, v in I.approaches.items() if k != approach], self.s, self.l_veh)
        res.update(ordinary_delay_veh_s=c["ordinary_delay_veh_s"], t_rec_s=c["t_rec_s"], extra_red_s=dr)
        res["flags"] += c["flags"] + (["over_C_max"] if c["ordinary_delay_veh_s"] > self.C_max else [])

    def _clear_hold_if_unused(self, I, t):
        if I.hold_approach is None:
            return
        if any(r["node_id"] == I.node_id and r["ev_approach"] == I.hold_approach
               and r["request_time_j_s"] <= t for r in self.reqs.values()):
            return
        cap = I.hold_cap_s
        if cap is not None and I.plan and I.plan[3] is None and I.plan[1] == cap and t < cap:
            I.plan = None  # cancel a projected cutoff on early EV passage
        I.hold_cap_s, I.hold_approach = None, None

    def _release(self, I, r, t, flags=()):
        self.reqs.pop((r["ev_id"], r["node_id"]), None)
        if I.plan and I.plan[3] == r["ev_id"] and t < I.plan[1]:
            I.plan = None  # unstarted clearance may safely be canceled
        self._clear_hold_if_unused(I, t)
        I.normal_active, I.normal_from_s = True, max(I.normal_from_s, t)
        res = self._res(I, r, RELEASED, t)
        if t >= r["request_time_j_s"]:
            self._cost(I, r, r["ev_approach"], t, res)
        res["flags"] += list(flags)
        return res

    def _start_normal(self, I, t):
        I.normal_active, I.normal_from_s = True, max(I.normal_from_s, t)
        nxt = I.next_approach()
        if I.plan is None and nxt != I.green:
            ts = max(t, I.ped_min_end_s)
            I.plan = (nxt, ts, ts + I.t_clear, None)

    def _apply_completed_plan(self, I, t):
        if I.plan is not None and t >= I.plan[2]:
            I.green, I.green_since_s, I.plan = I.plan[0], I.plan[2], None

    def _finish_cap_if_due(self, I, t, out):
        cap = I.hold_cap_s
        if cap is None or t < cap:
            return
        approach = I.hold_approach
        # Complete any inbound EV clearance that ends before the green-hold cap.
        self._apply_completed_plan(I, cap)
        if I.plan and I.plan[3] is None and I.plan[1] == cap:
            I.plan = None  # re-create from the exact cutoff time, not step(t)
        victims = [r for r in self.reqs.values()
                   if r["node_id"] == I.node_id and r["ev_approach"] == approach
                   and r["request_time_j_s"] <= cap]
        estimates = {r["ev_id"]: I.predicted_wait_after_cap(r["ev_approach"], r["eta_j_s"], cap)
                     for r in victims}
        I.hold_cap_s, I.hold_approach = None, None
        # Begin the actual normal transition at its scheduled timestamp.
        # The release event therefore correctly reports YELLOW at the cap.
        self._start_normal(I, cap)
        for r in victims:
            result = self._release(I, r, cap, ("G_max_capped", "warn_planner", "post_cap_delay_estimated"))
            result["ev_signal_delay_s"] = estimates[r["ev_id"]]
            out.append(result)

    def _normal_tick(self, I, t):
        for _ in range(1000):
            if not I.normal_active or I.plan is not None:
                return
            due = max(I.green_since_s, I.normal_from_s, I.ped_min_end_s) + I.green_time(I.green)
            nxt = I.next_approach()
            if t < due or nxt == I.green:
                return
            I.plan = (nxt, due, due + I.t_clear, None)
            if t < I.plan[2]:
                return
            I.green, I.green_since_s, I.plan = nxt, I.plan[2], None
        raise RuntimeError("normal-cycle advance exceeded 1000 phases")

    def _defer(self, I, r, t, end_s):
        res = self._res(I, r, DEFERRED, t)
        res["ev_signal_delay_s"] = max(0.0, end_s + I.t_clear - r["eta_j_s"])
        res["flags"].append("conflict_deferred")
        return res

    def _serve(self, I, r, t):
        a, eta = r["ev_approach"], r["eta_j_s"]
        if t < r["request_time_j_s"]:
            return self._res(I, r, NORMAL, t)
        cap_plan_for_this_green = (I.hold_approach == a and I.hold_cap_s is not None
                                   and t < I.hold_cap_s and I.plan is not None
                                   and I.plan[3] is None and I.plan[1] == I.hold_cap_s)
        # An EV with earlier ETA may replace a not-yet-started *normal* or
        # cap-induced transition. Never cancel an already-started clearance.
        if I.plan and I.plan[3] is None and t < I.plan[1] and not cap_plan_for_this_green:
            I.plan = None
            if I.hold_approach is not None and I.hold_approach != a:
                I.hold_cap_s, I.hold_approach = None, None
        if I.plan and I.plan[0] != a and not cap_plan_for_this_green:
            if I.plan[3] is not None and t < I.plan[1]:
                I.plan = None  # R6: the earlier EV may pre-empt an unstarted transition
                I.hold_cap_s, I.hold_approach = None, None
            else:
                return self._defer(I, r, t, I.plan[2] + self.T_pass)
        if I.green == a and (I.plan is None or cap_plan_for_this_green):
            green_start = max(I.green_since_s, r["request_time_j_s"])
            delay = 0.0
        else:
            if I.plan is None:
                ts = max(t, I.ped_min_end_s)
                I.plan = (a, ts, ts + I.t_clear, r["ev_id"])
                I.normal_active = True
            green_start = I.plan[2]
            delay = max(0.0, green_start - eta)
        # Max green is measured from the first priority activation / onset,
        # never extended by successive simulation steps or repeat submissions.
        if I.hold_approach != a or I.hold_cap_s is None:
            I.hold_approach = a
            # A pedestrian minimum is a hard safety constraint: if it
            # conflicts with G_max, defer cutoff and flag the override.
            I.hold_cap_s = max(green_start + self.G_max, I.ped_min_end_s)
        cap = I.hold_cap_s
        hold_end = min(eta + self.T_pass, cap)
        res = self._res(I, r, ACTIVE if r["feasible"] else BEST_EFFORT, t)
        if not r["feasible"]:
            res["flags"].append("warn_planner")
        if cap > green_start + self.G_max:
            res["flags"].append("pedestrian_min_overrides_G_max")
        if cap < eta + self.T_pass:
            res["flags"].append("G_max_capped")
            if cap < eta:
                res["flags"].append("post_cap_delay_estimated")
                delay = max(delay, I.predicted_wait_after_cap(a, eta, cap))
        # If the held phase is already green, schedule its deadline now so
        # phase(t) reflects the legal transition even without step(cap).
        if I.plan is None and I.green == a and cap < eta + self.k * r["sigma_eta_s"]:
            nxt = I.next_approach()
            if nxt != a:
                I.plan = (nxt, cap, cap + I.t_clear, None)
        res["ev_signal_delay_s"] = delay
        self._cost(I, r, a, max(hold_end, green_start), res)
        return res

    def step(self, t, passed=()):
        if t < self.now:
            raise ValueError("Controller.step() requires nondecreasing simulation times")
        self.now = t
        out = []
        for n, I in self.I.items():
            # Expire at the *scheduled time*, even when the caller skips steps.
            # A pass report is checked separately; without a pass timestamp,
            # an earlier cap is conservatively assumed to have occurred.
            self._finish_cap_if_due(I, t, out)
            self._apply_completed_plan(I, t)
            live = []
            for (ev, nn), r in list(self.reqs.items()):
                if nn != n:
                    continue
                if (ev, n) in passed or t > r["eta_j_s"] + self.k * r["sigma_eta_s"]:
                    out.append(self._release(I, r, t))
                else:
                    live.append(r)
            due = [r for r in live if t >= r["request_time_j_s"]]
            out.extend(self._res(I, r, NORMAL, t) for r in live if t < r["request_time_j_s"])
            owner = I.plan[3] if I.plan and I.plan[3] is not None and t >= I.plan[1] else None
            due.sort(key=lambda r: (r["ev_id"] != owner, r["eta_j_s"]))
            if not due:
                self._normal_tick(I, t)
                continue
            winner = due[0]
            for r in due:
                if r["ev_approach"] == winner["ev_approach"]:
                    out.append(self._serve(I, r, t))
                else:
                    out.append(self._defer(I, r, t, winner["eta_j_s"] + self.T_pass))
        return out
