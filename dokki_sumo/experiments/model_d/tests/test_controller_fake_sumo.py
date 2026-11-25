"""
Logic test for model_d_controller.py using a tiny fake TraCI (no SUMO needed).

Scene: ambulance drives a 3-edge route at constant 10 m/s. The last edge ends at a signal whose
program is the real TLS 172 pattern (ambulance green only 13 s of every 90 s).
Checks:
  1. WITHOUT the controller the ambulance meets a red signal.
  2. WITH the controller priority activates, it never touches yellow/all-red phases,
     the ambulance gets green on arrival, and normal operation is released afterwards.

Run:  python dokki_sumo/experiments/model_d/tests/test_controller_fake_sumo.py
"""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from model_d_controller import ModelDController, load_config  # noqa: E402

# TLS 172 program (phase state on the 4 ambulance links; other links shown as 'r'/'G' on a 5th link)
PHASES = [
    (12, "rrrrG"), (4, "rrrry"), (2, "rrrrr"), (13, "rrrrG"), (4, "rrrry"), (1, "rrrrr"),
    (13, "rrrrG"), (4, "rrrry"), (13, "GGGGr"), (4, "yyyyr"), (3, "rrrrr"), (13, "rrrrG"), (4, "rrrry"),
]


class FakeSim:
    EDGES = {"a": 300.0, "b": 300.0, "c": 300.0}
    ROUTE = ["a", "b", "c"]
    SPEED = 10.0

    def __init__(self, tls_offset=0.0):
        self.t = 0.0
        self.x = 0.0                      # distance along route
        self.phase = 0
        self.phase_end = float(PHASES[0][0])
        self.tls_offset = tls_offset
        self.forced = []                  # (time, new remaining)
        self.amb_in = True
        self.state_log = []
        self.arrival = None
        self.arrival_state = None
        self.override = None              # state forced by setRedYellowGreenState
        self.program = "0"                # becomes "online" (one phase) after setRedYellowGreenState, as in SUMO
        self.tick_states = []             # signal state (4 ambulance links + 1) at every second
        self.trafficlight = self
        self.vehicle = self
        self.edge = self
        self.lane = self
        # advance the signal to the requested offset
        for _ in range(int(tls_offset)):
            self._tick_signal()

    # ---- traci-like API
    def getIDList(self):
        return ["tl"] if False else (["ambulance_1"] if self.amb_in else [])

    # trafficlight
    def getControlledLinks(self, tls):
        return [[("c_0", "d_0", "i")]] * 4 + [[("x_0", "y_0", "i")]]

    def getAllProgramLogics(self, tls):
        return [NS(programID="0", phases=[NS(duration=d, state=s) for d, s in PHASES])]

    def getProgram(self, tls):
        return self.program

    def setProgram(self, tls, pid):
        self.program = pid
        self.phase, self.phase_start = 0, self.t
        self.phase_end = self.t + float(PHASES[0][0])
        self.override = None

    def getPhase(self, tls):
        return self.phase

    def getRedYellowGreenState(self, tls):
        return self.override if self.override is not None else PHASES[self.phase][1]

    def setRedYellowGreenState(self, tls, state):
        self.override = state
        self.program = "online"           # SUMO: a custom state switches to a one-phase "online" program

    def setPhase(self, tls, idx):
        if self.program == "online" and int(idx) != 0:
            raise RuntimeError(f"The phase index {idx} is not in the allowed range [0,0].")
        self.phase = int(idx)
        self.phase_start = self.t
        self.phase_end = self.t + float(PHASES[self.phase][0])
        self.override = None

    def getNextSwitch(self, tls):
        return self.phase_end

    def getSpentDuration(self, tls):
        return self.t - self.phase_start

    def setPhaseDuration(self, tls, dur):
        self.phase_end = self.t + float(dur)
        self.forced.append((self.t, dur, PHASES[self.phase][1]))

    phase_start = 0.0

    def _tick_signal(self):
        self.t += 1.0
        if self.program == "online":
            return                        # the online program does not advance by itself
        if self.t >= self.phase_end - 1e-9:
            self.phase = (self.phase + 1) % len(PHASES)
            self.override = None
            self.phase_start = self.t
            self.phase_end = self.t + float(PHASES[self.phase][0])

    # vehicle
    def getRoute(self, v):
        return self.ROUTE

    def getRouteIndex(self, v):
        return min(int(self.x // 300), 2)

    def getRoadID(self, v):
        if self.x >= 900.0:
            return ":j"
        return self.ROUTE[self.getRouteIndex(v)]

    def getLanePosition(self, v):
        return self.x - 300 * self.getRouteIndex(v)

    STALL_X = None                        # queue held by the red: the ambulance stands here until green
    stalled = False
    stall_s = 0

    def getSpeed(self, v):
        return 0.0 if self.stalled else self.SPEED

    # edge / lane
    def getTraveltime(self, e):
        if self.stalled and e == "c":
            return 70.0                   # jammed edge: the ETA stays ~68 s ahead of "now"
        return self.EDGES[e] / self.SPEED

    HALTING = 0

    def getLastStepHaltingNumber(self, e):
        return self.HALTING

    def getLaneNumber(self, e):
        return 2

    def getLength(self, lane_id):
        return self.EDGES[lane_id.rsplit("_", 1)[0]]

    def advance(self):
        before = self.x
        self._tick_signal()
        self.tick_states.append(self.getRedYellowGreenState("tl"))
        st = self.getRedYellowGreenState("tl")
        amb_green = all(c in "Gg" for c in st[:4])
        if self.STALL_X is not None and self.x >= self.STALL_X - 1e-9 and self.x < 900.0 and not amb_green:
            self.stalled = True
            self.stall_s += 1
            return                        # held in the queue by the red
        self.stalled = False
        self.x += self.SPEED
        if before < 900.0 <= self.x and self.arrival is None:      # crossed the stop line
            self.arrival = self.t
            self.arrival_state = "".join(self.getRedYellowGreenState("tl")[i] for i in range(4))
        if self.x >= 900.0:
            self.amb_in = self.x < 915.0


def run(controller_on, offset, **cfg_over):
    sim = FakeSim(tls_offset=offset)
    cfg = load_config()
    cfg.update(cfg_over)
    ctl = ModelDController(lambda: sim, cfg, "ambulance_1", "TEST", enabled=controller_on)
    for _ in range(200):
        sim.advance()
        ctl.step(sim.t)
        if not sim.amb_in and sim.t > 130:
            break
    ctl.step(sim.t + 1)
    return sim, ctl


def run_convoy(offset, gap=12):
    """A second ambulance (not simulated) follows `gap` s behind: the green must also cover it."""
    sim = FakeSim(tls_offset=offset)
    fleet = {}
    cfg = load_config()
    ctl = ModelDController(lambda: sim, cfg, "ambulance_1", "TEST", enabled=True, fleet=fleet)
    follower_state = None
    for _ in range(200):
        sim.advance()
        fleet.setdefault(("ambulance_1", "c"), {})["ambulance_2"] = sim.t + max(900.0 - sim.x, 0.0) / FakeSim.SPEED + gap
        ctl.step(sim.t)
        if sim.arrival is not None and follower_state is None and sim.t >= sim.arrival + gap:
            follower_state = "".join(PHASES[sim.phase][1][i] for i in range(4))
        if follower_state is not None:
            break
    return sim, ctl, follower_state


def is_green(state):
    return state is not None and all(c in "Gg" for c in state)


def sweep(**cfg_over):
    """Returns (offsets red WITHOUT controller, offsets red WITH controller, errors, controllers)."""
    red_off, red_on, ctls = [], [], []
    for off in range(0, 90, 3):
        sim0, _ = run(False, off, **cfg_over)
        sim1, ctl = run(True, off, **cfg_over)
        assert not ctl.errors, ctl.errors
        if not is_green(sim0.arrival_state):
            red_off.append(off)
        if not is_green(sim1.arrival_state):
            red_on.append(off)
        # safety: nothing is ever forced during yellow or all-red phases
        for t, dur, state in sim1.forced:
            assert any(c in "Gg" for c in state) and "y" not in state, f"touched non-green phase {state} at {t}"
            assert dur >= 1.0
        # restoration: every intersection is released, and activated ones are marked restored
        for st in ctl.keys.values():
            assert st["status"] == "done"
            if st["activations"]:
                assert st["restored"] is True
        ctls.append(ctl)
    return red_off, red_on, ctls


def skip_test():
    """phase_skip: the ambulance's green is reached faster, yellow is never shortened, all-red is shown,
    and the signal is never left stuck."""
    red_off, red_noskip, red_skip = [], [], []
    wait_noskip, wait_skip = [], []
    for off in range(0, 90, 3):
        sim0, _ = run(False, off)
        sim1, ctl1 = run(True, off)
        sim2, ctl2 = run(True, off, phase_skip=True)
        assert not ctl2.errors, ctl2.errors
        if not is_green(sim0.arrival_state):
            red_off.append(off)
        if not is_green(sim1.arrival_state):
            red_noskip.append(off)
        if not is_green(sim2.arrival_state):
            red_skip.append(off)
        # safety 1: every yellow that appears lasts at least the program's 4 s
        run_len, first_run = 0, True
        for st in sim2.tick_states:          # recording edges (first / last partial yellow) are ignored
            if "y" in st:
                run_len += 1
            else:
                if not (first_run and run_len):
                    assert run_len == 0 or run_len >= 4, f"yellow shortened to {run_len} s (offset {off})"
                run_len, first_run = 0, False
        # safety 2: the ambulance green never comes straight after another movement's green
        prev = None
        for st in sim2.tick_states:
            if prev is not None and st[:4] == "GGGG" and prev[:4] != "GGGG":
                assert prev == "rrrrr" or "y" in prev, f"green after {prev} (offset {off})"
            prev = st
        # nothing left stuck: the signal is back to a program phase at the end
        assert sim2.override is None and sim2.program == "0", f"signal left in a forced state (offset {off})"
    return red_off, red_noskip, red_skip


def stall_test():
    """activate_when_stopped: an ambulance held in a queue by the red is served no later than without it."""
    FakeSim.STALL_X = 610.0
    tot_off, tot_on, worse = 0, 0, 0
    try:
        for off in range(0, 90, 3):
            simA, ctlA = run(True, off, phase_skip=True)
            simB, ctlB = run(True, off, phase_skip=True, activate_when_stopped=True)
            assert not ctlA.errors and not ctlB.errors, (ctlA.errors, ctlB.errors)
            assert simB.override is None and simB.program == "0", f"signal left forced (offset {off})"
            tot_off += simA.stall_s
            tot_on += simB.stall_s
            worse += int(simB.stall_s > simA.stall_s)
    finally:
        FakeSim.STALL_X = None
    return tot_off, tot_on, worse


def main():
    ok = True
    red_off, red_on, ctls = sweep()
    print(f"default config (min other green 5 s): red arrivals WITHOUT controller {len(red_off)}/30, "
          f"WITH controller {len(red_on)}/30 (offsets {red_on})")
    ok &= len(red_off) > 10                              # the scene really is hard without priority
    ok &= set(red_on) <= set(red_off)                    # never worse than no controller
    ok &= len(red_on) <= len(red_off) // 4               # clear improvement

    _, red_on3, _ = sweep(min_green_other_phase_s=3.0)
    print(f"min other green 3 s: red arrivals WITH controller {len(red_on3)}/30")
    ok &= not red_on3

    r_off, r_noskip, r_skip = skip_test()
    print(f"phase_skip: red arrivals without controller {len(r_off)}/30, controller {len(r_noskip)}/30, "
          f"controller + phase_skip {len(r_skip)}/30")
    ok &= set(r_skip) <= set(r_off)
    ok &= len(r_skip) <= len(r_noskip)

    st_off, st_on, st_worse = stall_test()
    print(f"stopped in the queue at red (30 timings): total stalled seconds without activate_when_stopped {st_off}, "
          f"with it {st_on}, timings where it was worse: {st_worse}")
    ok &= st_on <= st_off and st_worse == 0

    FakeSim.HALTING = 8                                   # queue ahead: 8 / (0.5 * 2 lanes) = 8 s hold
    _, red_q, ctls_q = sweep()
    print(f"with an 8 s queue ahead: red arrivals WITH controller {len(red_q)}/30")
    ok &= set(red_q) <= set(red_off)
    FakeSim.HALTING = 0

    _, ctl_t = run(True, 10, trace_signals_1s=True, trace_window_s=40.0)
    _, ctl_n = run(True, 10)
    tr = ctl_t.trace_rows
    secs = sorted({r["simulation_time_s"] for r in tr})
    print(f"1-second trace: {len(tr)} rows over {len(secs)} distinct seconds, trace off gives {len(ctl_n.trace_rows)} rows")
    ok &= len(tr) > 10 and len(ctl_n.trace_rows) == 0
    ok &= all(b_ - a_ == 1 for a_, b_ in zip(secs, secs[1:]))     # one row per simulated second, no gaps

    ok_convoy, total = 0, 0
    for off in range(0, 90, 3):
        sim1, _ = run(True, off)
        if not is_green(sim1.arrival_state):
            continue                                           # leader itself not served: skip
        total += 1
        simc, ctlc, fstate = run_convoy(off)
        assert not ctlc.errors, ctlc.errors
        for t_, dur, state in simc.forced:
            assert any(c in "Gg" for c in state) and "y" not in state
        ok_convoy += int(is_green(fstate))
    print(f"convoy (follower 12 s behind): follower also met a green in {ok_convoy}/{total} timings")
    ok &= total > 0 and ok_convoy >= 0.8 * total

    errs = [abs(r["eta_error_s"]) for c in ctls for r in c.eta_rows if r["eta_error_s"] != ""]
    print(f"ETA records with actual arrival: {len(errs)}, mean |ETA error| = {sum(errs) / max(len(errs), 1):.2f}s")
    ok &= bool(errs) and sum(errs) / len(errs) < 2.0
    print("ALL OK" if ok else "FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
