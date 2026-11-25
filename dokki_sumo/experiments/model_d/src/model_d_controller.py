"""
model_d_controller.py  --  Model D: ETA + traffic-signal priority for ONE ambulance route.

Works directly on a running SUMO/TraCI simulation. It never changes the ambulance route
(Model D keeps the Scenario 0 route), and it only touches the signals through
traci.trafficlight.setPhaseDuration(), so the original program (phase order, yellow and
all-red clearance phases) is never skipped and restores itself automatically.
OPTIONAL (cfg phase_skip=true, off by default): after the current green's yellow has finished in
full, jump straight to the ambulance's green (short all-red in between) instead of walking
through every intermediate green.

HOW IT WORKS (every simulation second, after simulationStep):
  1. Find the signalized intersections still ahead on the ambulance's current route.
  2. ETA_j      = t_now + sum(mu_e) over the remaining route up to the stop line
                  (mu_e = SUMO's current edge travel time)                 [team eq. (2)]
     sigma_ETA  = sqrt(sum((rho * mu_e)^2))                                [team eq. (3)]
  3. T_prepare  = T_clearance + T_queue + k * sigma_ETA                    [team eq. (4)-(6)]
       T_clearance : minimum time the signal program needs to reach the ambulance's green
                     (rest of the current phase + yellow/all-red phases + other greens
                     cut to their minimum green)
       T_queue     : halting vehicles on the approach / (s * lanes)
  4. Priority is requested when t_now >= ETA - T_prepare AND normal operation would not
     already give the ambulance a usable green window.                     [team eq. (7)-(8)]
  5. While priority is active:
       - other green phases are cut short (never below the minimum green;
         yellow and all-red phases are never touched)
       - when the ambulance's own green is running it is extended until the ambulance has
         passed, capped by max_extension_s and g_max_s
  6. When the ambulance crosses the stop line (or its route no longer uses that approach)
     priority ends. The unchanged program simply continues, so normal operation is restored.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

DEFAULT_CONFIG = {
    "eta_uncertainty_ratio": 0.15,
    "queue_discharge_rate_veh_per_s_per_lane": 0.5,
    "safety_margin_k": 2.0,
    "served_sigma_k": 1.0,
    "activation_horizon_s": 90.0,
    "min_green_other_phase_s": 5.0,
    "truncate_to_s": 1.0,
    "pass_margin_s": 3.0,
    "max_extension_s": 20.0,
    "g_max_s": 40.0,
    "min_edge_speed_mps": 1.0,
    "eta_log_interval_s": 5.0,
    "stopped_speed_mps": 0.1,
    "forced_start_margin_s": 3.0,
    "queue_aware_eta": True,
    "queue_pass_factor": 1.0,
    "max_queue_hold_s": 30.0,
    "convoy_window_s": 45.0,
    "phase_skip": False,
    "activate_when_stopped": False,
    "skip_all_red_s": 2.0,
}


def load_config(path=None):
    cfg = dict(DEFAULT_CONFIG)
    if path:
        with open(path, encoding="utf-8") as f:
            cfg.update({k: v for k, v in json.load(f).items() if not k.startswith("_")})
    return cfg


def _is_green(state, links):
    return all(state[i] in "Gg" for i in links)


def _phase_kind(state, links):
    """target = green for the ambulance approach; transition = yellow / all-red;
    other = green for some other movement."""
    if _is_green(state, links):
        return "target"
    if any(c in "yY" for c in state) or not any(c in "Gg" for c in state):
        return "transition"
    return "other"


class ModelDController:
    def __init__(self, traci_getter, cfg, ambulance_id, scenario_id, enabled=True, fleet=None):
        self._get = traci_getter          # callable returning the live traci module
        self.cfg = cfg
        self.amb = ambulance_id
        self.scenario_id = scenario_id
        self.enabled = enabled
        # fleet: dict shared by the controllers of several ambulances on the SAME route.
        # fleet[(tls, approach_edge)][ambulance_id] = predicted stop-line arrival time.
        self.fleet = fleet if fleet is not None else {}
        self.errors = []
        self.events = []
        self.eta_rows = []
        self.keys = {}                    # (tls, approach edge) -> per-intersection record
        self._edge_map = None             # approach edge -> (tls, controlled link indices)
        self._program_ids = {}            # tls -> id of its original program (restored after an all-red)
        self._programs = {}
        self._len_cache = {}
        self._pending_actual = []
        self.trace_rows = []              # optional 1-second signal trace (config trace_signals_1s)
        self._phase_track = {}
        self._seen = False
        self._last_t = None
        self._event_id = 0

    # ------------------------------------------------------------------ setup
    def _build_maps(self):
        tr = self._get()
        self._edge_map = {}
        for tls in tr.trafficlight.getIDList():
            per_edge = {}
            for idx, group in enumerate(tr.trafficlight.getControlledLinks(tls)):
                for link in group:
                    edge = link[0].rsplit("_", 1)[0]
                    per_edge.setdefault(edge, set()).add(idx)
            for edge, idxs in per_edge.items():
                self._edge_map[edge] = (tls, tuple(sorted(idxs)))
            logics = tr.trafficlight.getAllProgramLogics(tls)
            program_id = tr.trafficlight.getProgram(tls)
            self._programs[tls] = next((l for l in logics if l.programID == program_id), logics[0])
            self._program_ids[tls] = self._programs[tls].programID

    def _edge_len(self, edge):
        if edge not in self._len_cache:
            self._len_cache[edge] = float(self._get().lane.getLength(edge + "_0"))
        return self._len_cache[edge]

    def _spent(self, tls, phase, t):
        try:
            return float(self._get().trafficlight.getSpentDuration(tls))
        except Exception:
            rec = self._phase_track.setdefault(tls, [None, t])
            if rec[0] != phase:
                rec[0], rec[1] = phase, t
            return t - rec[1]

    # --------------------------------------------------------------- ETA math
    def _eta(self, route, cur_i, pos, p, t):
        """ETA to the stop line at the end of route[p], plus sigma and distance."""
        tr = self._get()
        rho = self.cfg["eta_uncertainty_ratio"]
        vmin = self.cfg["min_edge_speed_mps"]
        dist, eta, var = 0.0, t, 0.0
        for k in range(cur_i, p + 1):
            edge = route[k]
            length = self._edge_len(edge)
            left = max(length - pos, 0.0) if k == cur_i else length
            frac = left / length if length > 0 else 1.0
            dist += left
            mu = float(tr.edge.getTraveltime(edge))
            if length > 0:
                mu = min(mu, length / vmin)
            part = frac * mu
            if k == cur_i and self.cfg.get("own_speed_eta", False):
                # Optional: on the edge the ambulance is already on, use its OWN speed instead of
                # the edge-average travel time (which includes slow queued ordinary vehicles).
                # Only ever makes the estimate earlier, and never below the free-flow-style floor.
                try:
                    v_own = float(tr.vehicle.getSpeed(self.amb))
                except Exception:
                    v_own = 0.0
                v_use = max(v_own, self.cfg.get("own_speed_floor_mps", 5.0))
                part = min(part, left / v_use)
            eta += part
            var += (rho * part) ** 2
        return eta, math.sqrt(var), dist

    def _queue_ahead_s(self, edge, same_edge, pos):
        """Time the queue AHEAD of the ambulance needs to discharge once the signal is green.
        0 when queue_aware_eta is off, so the first version of Model D can be reproduced."""
        cfg = self.cfg
        if not cfg.get("queue_aware_eta", False):
            return 0.0
        tr = self._get()
        n = None
        if same_edge:
            try:
                n = 0
                for vid in tr.edge.getLastStepVehicleIDs(edge):
                    if vid == self.amb:
                        continue
                    if tr.vehicle.getLanePosition(vid) > pos and tr.vehicle.getSpeed(vid) < cfg["stopped_speed_mps"]:
                        n += 1
            except Exception:
                n = None
        if n is None:
            n = int(tr.edge.getLastStepHaltingNumber(edge))
        lanes = max(int(tr.edge.getLaneNumber(edge)), 1)
        q = n / (cfg["queue_discharge_rate_veh_per_s_per_lane"] * lanes) * cfg.get("queue_pass_factor", 1.0)
        return min(q, cfg.get("max_queue_hold_s", 30.0))

    def _convoy_extra_s(self, key, own_arrival):
        """Extra seconds the green must last so that ambulances following within convoy_window_s
        (same approach, predicted by their own controllers) also get through."""
        window = self.cfg.get("convoy_window_s", 0.0)
        extra = 0.0
        for other, other_arrival in list(self.fleet.get(key, {}).items()):
            if other != self.amb and -1.0 <= other_arrival - own_arrival <= window:
                extra = max(extra, other_arrival - own_arrival)
        return extra

    def _min_time_to_green(self, tls, links, phase, spent, remaining):
        """Shortest time the program needs before the ambulance's green can start."""
        phases = self._programs[tls].phases
        n = len(phases)
        cfg = self.cfg
        total = 0.0
        for step in range(n + 1):
            i = (phase + step) % n
            kind = _phase_kind(phases[i].state, links)
            if kind == "target":
                return total
            if step == 0:
                if kind == "other":
                    total += min(max(cfg["min_green_other_phase_s"] - spent, 0.0) + cfg["truncate_to_s"], remaining)
                else:
                    total += remaining
            else:
                total += (cfg["min_green_other_phase_s"] + cfg["truncate_to_s"]) if kind == "other" \
                    else phases[i].duration
        return total

    def _green_windows(self, tls, links, phase, remaining, spent, t):
        """Green windows (start, end) for the ambulance approach under the UNMODIFIED program."""
        phases = self._programs[tls].phases
        n = len(phases)
        horizon = 2 * sum(p.duration for p in phases)
        windows, cursor, open_start = [], t, None
        for step in range(2 * n + 1):
            i = (phase + step) % n
            dur = remaining if step == 0 else phases[i].duration
            if _phase_kind(phases[i].state, links) == "target":
                if open_start is None:
                    open_start = t - spent if step == 0 else cursor
            elif open_start is not None:
                windows.append((open_start, cursor))
                open_start = None
            cursor += dur
            if cursor - t > horizon:
                break
        if open_start is not None:
            windows.append((open_start, cursor))
        return windows

    def _forced_windows(self, tls, links, phase, remaining, spent, t):
        """Green windows if every other-green is cut to its minimum green from now on."""
        phases = self._programs[tls].phases
        n = len(phases)
        cfg = self.cfg
        windows, cursor, open_start = [], t, None
        for step in range(3 * n + 1):
            i = (phase + step) % n
            kind = _phase_kind(phases[i].state, links)
            if step == 0:
                if kind == "other":
                    dur = min(max(cfg["min_green_other_phase_s"] - spent, 0.0) + cfg["truncate_to_s"], remaining)
                else:
                    dur = remaining
            else:
                dur = (cfg["min_green_other_phase_s"] + cfg["truncate_to_s"]) if kind == "other" \
                    else phases[i].duration
            if kind == "target":
                if open_start is None:
                    open_start = t - spent if step == 0 else cursor
            elif open_start is not None:
                windows.append((open_start, cursor))
                open_start = None
            cursor += dur
        if open_start is not None:
            windows.append((open_start, cursor))
        return windows

    def _plan_normal(self, tls, links, phase, remaining, spent, t, eta, sigma, q=0.0):
        """Returns (served_normally, hold_window_start).

        served_normally : the unmodified program already has a green covering the arrival.
        hold_window_start: start of a natural green that can be HELD (extended, within g_max)
                           until the ambulance has passed; None if there is no such green.
        """
        cfg = self.cfg
        early = eta - cfg["served_sigma_k"] * sigma
        late = eta + q + cfg["pass_margin_s"]
        windows = self._green_windows(tls, links, phase, remaining, spent, t)
        if any(a <= early and late <= b for a, b in windows):
            return True, None
        holdable = [a for a, b in windows
                    if a <= early and a + cfg["g_max_s"] >= late and b > t]
        return False, (max(holdable) if holdable else None)

    def _forced_plan_ok(self, tls, links, phase, remaining, spent, t, eta, q=0.0):
        """True if cutting other greens to minimum from now gives a green that can be held to the ETA."""
        cfg = self.cfg
        late = eta + q + cfg["pass_margin_s"]
        wins = self._forced_windows(tls, links, phase, remaining, spent, t)
        return any(a <= eta - cfg["forced_start_margin_s"] and a + cfg["g_max_s"] >= late
                   for a, b in wins)

    # ------------------------------------------------------------- main step
    def step(self, t):
        if not self.enabled:
            return
        try:
            self._step(t)
        except Exception as exc:          # never break the simulation because of the controller
            self.errors.append(f"t={t}: {type(exc).__name__}: {exc}")
            self.enabled = False
            for st in self.keys.values():                   # never leave a signal stuck in all-red
                try:
                    self._complete_skip(st, st["tls"], t)
                except Exception:
                    pass

    def _step(self, t):
        tr = self._get()
        dt = 1.0 if self._last_t is None else max(t - self._last_t, 0.0)
        self._last_t = t
        if self.amb not in tr.vehicle.getIDList():
            if self._seen:
                self._release_all(t)
            return
        self._seen = True
        if self._edge_map is None:
            self._build_maps()

        route = list(tr.vehicle.getRoute(self.amb))
        cur_i = int(tr.vehicle.getRouteIndex(self.amb))
        road = tr.vehicle.getRoadID(self.amb)
        pos = float(tr.vehicle.getLanePosition(self.amb))
        speed = float(tr.vehicle.getSpeed(self.amb))
        on_junction = road.startswith(":")

        # signal state that SUMO really applied after the previous action
        for ev in self._pending_actual:
            state = tr.trafficlight.getRedYellowGreenState(ev["traffic_light_id"])
            links = self.keys[(ev["traffic_light_id"], ev["_edge"])]["links"]
            ev["actual_signal_state"] = "".join(state[i] for i in links)
        self._pending_actual = []

        ahead = {}
        for p in range(cur_i, len(route)):
            if route[p] in self._edge_map:
                tls, links = self._edge_map[route[p]]
                ahead[(tls, route[p])] = (p, links, p == cur_i and on_junction)

        # intersections already crossed, or no longer on the route (e.g. after a reroute)
        for key, st in list(self.keys.items()):
            if st["status"] == "done":
                continue
            if key not in ahead:
                crossed = key[1] in route[:cur_i + 1]
                self._finish(key, t, passed=crossed, reason="passed" if crossed else "route_changed")
            elif ahead[key][2]:
                self._finish(key, t, passed=True, reason="passed")

        for key, (p, links, crossed) in ahead.items():
            if crossed:
                continue
            tls, edge = key
            st = self.keys.setdefault(key, self._new_key(tls, edge, links))
            if st["status"] == "done":
                continue
            eta, sigma, dist = self._eta(route, cur_i, pos, p, t)
            q_ahead = self._queue_ahead_s(route[p], p == cur_i, pos)
            self.fleet.setdefault(key, {})[self.amb] = eta + q_ahead
            q_need = q_ahead + self._convoy_extra_s(key, eta + q_ahead)   # green must last this long after eta
            phase = tr.trafficlight.getPhase(tls)
            state = tr.trafficlight.getRedYellowGreenState(tls)
            remaining = max(float(tr.trafficlight.getNextSwitch(tls)) - t, 0.0)
            spent = self._spent(tls, phase, t)

            if self.cfg.get("trace_signals_1s", False) and eta - t <= self.cfg.get("trace_window_s", 40.0):
                self.trace_rows.append({
                    "scenario_id": self.scenario_id, "simulation_time_s": t, "traffic_light_id": tls,
                    "approach_edge": edge, "controller_status": st["status"], "phase_index": phase,
                    "ambulance_link_state": "".join(state[i] for i in links),
                    "ambulance_link_green": int(_is_green(state, links)),
                    "ambulance_speed_mps": round(speed, 3), "distance_to_stop_line_m": round(dist, 2),
                    "estimated_arrival_time_s": round(eta, 2),
                })

            if cur_i == p and speed < self.cfg["stopped_speed_mps"]:
                st["stopped_on_approach_s"] += dt
                if not _is_green(state, links):
                    st["signal_wait_s"] += dt

            if st["status"] == "idle" and eta - t > self.cfg["activation_horizon_s"]:
                continue

            halting = int(tr.edge.getLastStepHaltingNumber(edge))
            lanes = max(int(tr.edge.getLaneNumber(edge)), 1)
            t_queue = halting / (self.cfg["queue_discharge_rate_veh_per_s_per_lane"] * lanes)
            t_clear = self._min_time_to_green(tls, links, phase, spent, remaining)
            t_prepare = t_clear + t_queue + self.cfg["safety_margin_k"] * sigma
            t_request = eta - t_prepare
            if st["first_eta_t"] is None:
                st["first_eta_t"] = t

            if st["last_eta_log"] < 0 or t - st["last_eta_log"] >= self.cfg["eta_log_interval_s"] - 1e-9:
                st["last_eta_log"] = t
                row = {
                    "scenario_id": self.scenario_id, "traffic_light_id": tls, "intersection_id": tls,
                    "approach_edge": edge, "ambulance_id": self.amb, "simulation_time_s": t,
                    "distance_to_intersection_m": round(dist, 2), "ambulance_speed_mps": round(speed, 3),
                    "estimated_arrival_time_s": round(eta + q_ahead, 2), "sigma_eta_s": round(sigma, 2),
                    "queue_ahead_s": round(q_ahead, 2),
                    "t_clearance_s": round(t_clear, 2), "t_queue_s": round(t_queue, 2),
                    "t_prepare_s": round(t_prepare, 2), "t_request_s": round(t_request, 2),
                    "feasible": int(t_request >= t), "actual_arrival_time_s": "", "eta_error_s": "",
                }
                self.eta_rows.append(row)
                st["eta_rows"].append(row)

            if st["status"] == "idle":
                served, hold_start = self._plan_normal(tls, links, phase, remaining, spent, t, eta, sigma, q_need)
                running_green = _is_green(state, links)
                # optional: the ambulance is already standing on the approach and the signal is not green for
                # it (typically stuck in the queue the red itself holds): do not wait for the planned time
                stopped_red = bool(self.cfg.get("activate_when_stopped", False)) and cur_i == p \
                    and speed < self.cfg["stopped_speed_mps"] and not running_green
                if served and not stopped_red:
                    continue
                if stopped_red:
                    go = True
                elif hold_start is not None:
                    # a natural green can carry the ambulance through: wait for it, then hold it
                    go = running_green and hold_start <= t
                elif self._forced_plan_ok(tls, links, phase, remaining, spent, t, eta, q_need):
                    go = True                      # a compressed signal cycle can serve the arrival
                else:
                    go = t >= t_request            # best effort (team equation)
                if go:
                    st["status"] = "active"
                    st["activation_t"] = t
                    st["activations"] += 1
                    self._log(st, tls, edge, t, "priority_activated", state, links, eta, pos, speed)
                else:
                    continue
            self._apply(st, tls, edge, links, state, remaining, spent, eta, t, pos, speed, halting, dt, q_need)

    def _new_key(self, tls, edge, links):
        return {"tls": tls, "edge": edge, "links": links, "status": "idle", "activation_t": None,
                "deactivation_t": None, "activations": 0, "actions": 0, "truncations": 0,
                "extensions": 0, "stopped_on_approach_s": 0.0, "signal_wait_s": 0.0,
                "first_eta_t": None, "last_eta_log": -1.0, "ext_end": None,
                "actual_arrival_t": None, "passed": False, "restored": None,
                "halting_vehicle_s": 0.0, "eta_rows": [], "events": []}

    # --------------------------------------------------------- signal actions
    def _apply(self, st, tls, edge, links, state, remaining, spent, eta, t, pos, speed, halting, dt, q=0.0):
        tr = self._get()
        cfg = self.cfg
        st["halting_vehicle_s"] += halting * dt
        kind = _phase_kind(state, links)
        if cfg.get("phase_skip", False) and self._skip_step(st, tls, edge, state, links, remaining, t,
                                                            eta, pos, speed):
            return
        if kind == "transition":
            return                                    # yellow / all-red: never touched
        if kind == "other":
            if spent >= cfg["min_green_other_phase_s"] and remaining > cfg["truncate_to_s"] + 1e-9:
                tr.trafficlight.setPhaseDuration(tls, cfg["truncate_to_s"])
                st["actions"] += 1
                st["truncations"] += 1
                self._log(st, tls, edge, t, "early_green_truncate_other_phase", state, links, eta, pos, speed)
            return
        # the ambulance's own green is running: hold it until the ambulance has passed
        t_end = t + remaining
        need = eta + q + cfg["pass_margin_s"]
        green_start = t - spent
        if need > t_end + 1e-9 and green_start + cfg["g_max_s"] >= need - 1e-9:
            ext = min(need - t_end, cfg["max_extension_s"])
            ext = min(ext, max(cfg["g_max_s"] - (spent + remaining), 0.0))
            if ext >= 1.0:
                tr.trafficlight.setPhaseDuration(tls, remaining + ext)
                st["actions"] += 1
                if st["ext_end"] is None or abs((t_end + ext) - st["ext_end"]) >= 2.0:
                    st["extensions"] += 1
                    self._log(st, tls, edge, t, "green_extension", state, links, eta, pos, speed)
                st["ext_end"] = t_end + ext

    # ------------------------------------------------------ optional phase skipping
    def _skip_step(self, st, tls, edge, state, links, remaining, t, eta, pos, speed):
        """Optional (cfg phase_skip). When the current green has ended and its yellow is about to
        finish, but the NEXT green in the program is not the ambulance's, do not walk through the
        intermediate greens: finish the yellow in full, show a short all-red, then jump straight to
        the ambulance's green. Yellow phases are never shortened. Returns True if it handled this step."""
        tr = self._get()
        cfg = self.cfg
        pend = st.get("skip")
        if pend:
            if not pend["over"] and t >= pend["override_at"] - 1e-9:
                # the yellow has now been shown in full: show all-red
                tr.trafficlight.setRedYellowGreenState(tls, "r" * len(state))
                pend["over"] = True
                st["actions"] += 1
                self._log(st, tls, edge, t, "phase_skip_all_red", state, links, eta, pos, speed)
            if t >= pend["until"] - 1e-9:
                self._complete_skip(st, tls, t, edge, state, links, eta, pos, speed)
            return True
        if _phase_kind(state, links) != "transition" or not any(c in "yY" for c in state):
            return False
        if remaining > 1.0 + 1e-9:
            return False
        phases = self._programs[tls].phases
        n = len(phases)
        cur = int(tr.trafficlight.getPhase(tls))
        nxt = None
        for step in range(1, n + 1):
            i = (cur + step) % n
            if _phase_kind(phases[i].state, links) != "transition":
                nxt = i
                break
        if nxt is None or _phase_kind(phases[nxt].state, links) != "other":
            return False                       # next green is already the ambulance's (or none)
        tgt = None
        for step in range(1, n + 1):
            i = (cur + step) % n
            if _phase_kind(phases[i].state, links) == "target":
                tgt = i
                break
        if tgt is None:
            return False
        allred = float(cfg.get("skip_all_red_s", 2.0))
        # keep the program in this yellow phase for the all-red period too (so it cannot move on to
        # the next green by itself); the yellow itself is shown in full before the all-red starts
        tr.trafficlight.setPhaseDuration(tls, remaining + allred + 1.0)
        st["skip"] = {"override_at": t + remaining, "until": t + remaining + allred, "tgt": tgt, "over": False}
        return True

    def _complete_skip(self, st, tls, t, edge=None, state=None, links=None, eta=0.0, pos=0.0, speed=0.0):
        pend = st.get("skip")
        if not pend:
            return
        tr = self._get()
        st["skip"] = None
        # setRedYellowGreenState() makes SUMO switch the light to a one-phase "online" program, so the
        # original program must be restored first; only then can we jump to the ambulance's phase.
        orig = self._program_ids.get(tls)
        if orig is not None:
            tr.trafficlight.setProgram(tls, orig)
        tr.trafficlight.setPhase(tls, int(pend["tgt"]))
        if state is not None:
            self._log(st, tls, edge, t, "phase_skip_to_ambulance_green", state, links, eta, pos, speed)

    def _log(self, st, tls, edge, t, action, state, links, eta, pos, speed):
        self._event_id += 1
        ev = {
            "scenario_id": self.scenario_id, "event_id": self._event_id, "ambulance_id": self.amb,
            "traffic_light_id": tls, "approach_edge": edge, "simulation_time_s": t, "ambulance_lane_position_m": round(pos, 2),
            "ambulance_speed_mps": round(speed, 3), "estimated_arrival_time_s": round(eta, 2),
            "previous_signal_state": "".join(state[i] for i in links),
            "requested_signal_state": "G" * len(links), "actual_signal_state": "",
            "control_action": action, "priority_activation_time_s": st["activation_t"],
            "priority_deactivation_time_s": "", "normal_operation_restored": "", "_edge": edge,
        }
        self.events.append(ev)
        st["events"].append(ev)
        self._pending_actual.append(ev)

    # --------------------------------------------------------------- release
    def _finish(self, key, t, passed, reason):
        st = self.keys[key]
        if st["status"] == "done":
            return
        was_active = st["status"] == "active"
        if st.get("skip"):                                  # never leave a signal stuck in all-red
            try:
                self._complete_skip(st, st["tls"], t)
            except Exception:
                pass
        self.fleet.get(key, {}).pop(self.amb, None)
        st["status"] = "done"
        st["passed"] = bool(passed)
        if passed:
            st["actual_arrival_t"] = t
            for row in st["eta_rows"]:
                row["actual_arrival_time_s"] = t
                row["eta_error_s"] = round(row["estimated_arrival_time_s"] - t, 2)
        if was_active:
            st["deactivation_t"] = t
            st["restored"] = True
            for ev in st["events"]:
                ev["priority_deactivation_time_s"] = t
                ev["normal_operation_restored"] = True
            self._event_id += 1
            self.events.append({
                "scenario_id": self.scenario_id, "event_id": self._event_id, "ambulance_id": self.amb,
                "traffic_light_id": st["tls"], "approach_edge": st["edge"], "simulation_time_s": t,
                "control_action": "priority_released_" + reason,
                "priority_activation_time_s": st["activation_t"], "priority_deactivation_time_s": t,
                "normal_operation_restored": True, "_edge": st["edge"],
            })

    def _release_all(self, t):
        for key, st in self.keys.items():
            if st["status"] != "done":
                self._finish(key, t, passed=False, reason="ambulance_left_simulation")

    # --------------------------------------------------------------- outputs
    def summary(self):
        sts = list(self.keys.values())
        passed = [s for s in sts if s["passed"]]
        errs = [abs(r["eta_error_s"]) for r in self.eta_rows if r["eta_error_s"] != ""]
        return {
            "model_d_priority_activations": sum(s["activations"] for s in sts),
            "model_d_priority_actions": sum(s["actions"] for s in sts),
            "model_d_intersections_encountered": len(passed),
            "model_d_intersections_receiving_priority": sum(1 for s in passed if s["actions"] > 0),
            "model_d_ambulance_signal_wait_s": round(sum(s["signal_wait_s"] for s in sts), 1),
            "model_d_ambulance_stopped_on_approach_s": round(sum(s["stopped_on_approach_s"] for s in sts), 1),
            "model_d_eta_mae_s": round(sum(errs) / len(errs), 2) if errs else "",
            "model_d_controller_errors": " | ".join(self.errors),
        }

    def intersection_rows(self):
        rows = []
        for (tls, edge), s in self.keys.items():
            rows.append({
                "scenario_id": self.scenario_id, "ambulance_id": self.amb, "traffic_light_id": tls,
                "approach_edge": edge, "encountered": int(s["passed"]), "priority_activations": s["activations"],
                "signal_modifications": s["actions"], "early_green_truncations": s["truncations"],
                "green_extensions": s["extensions"],
                "ambulance_signal_wait_s": round(s["signal_wait_s"], 1),
                "ambulance_stopped_on_approach_s": round(s["stopped_on_approach_s"], 1),
                "priority_activation_time_s": "" if s["activation_t"] is None else s["activation_t"],
                "priority_deactivation_time_s": "" if s["deactivation_t"] is None else s["deactivation_t"],
                "actual_arrival_time_s": "" if s["actual_arrival_t"] is None else s["actual_arrival_t"],
                "normal_operation_restored": "" if s["restored"] is None else int(s["restored"]),
                "approach_halting_vehicle_s_during_priority": round(s["halting_vehicle_s"], 1),
            })
        return rows

    def clean_events(self):
        return [{k: v for k, v in ev.items() if not k.startswith("_")} for ev in self.events]


def write_csv_rows(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(k for r in rows for k in r)) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
