# Using Model D together with Model C

Model D (traffic-signal priority) is one Python class. It does not need Model C's code, and Model C does not need to change
how it reroutes. Model D reads the ambulance's current route from SUMO every second, so a route that Model C changes is
picked up automatically; a signal that is no longer on the route is released and the normal program is restored.

## Files to copy
`src/model_d_controller.py` and `config/model_d_config_skip_stopped.json` (the final configuration used in the reports).

## Three lines to add to your runner
```python
from model_d_controller import ModelDController, load_config

cfg = load_config("config/model_d_config_skip_stopped.json")
controller = ModelDController(lambda: traci, cfg, "ambulance_1", "S1")   # once per run, before the loop

# inside the simulation loop, right after traci.simulationStep() and after Model C's reroute code:
controller.step(float(traci.simulation.getTime()))
```
- `"ambulance_1"` is the ambulance's vehicle id; the last argument is only a label used in the log files.
- Create a new controller for every run (every scenario / seed).
- `controller.enabled = False` (or `enabled=False` in the constructor) turns it off, for a run without signal priority.
- After the run: `controller.summary()`, `controller.eta_rows`, `controller.clean_events()` hold the results;
  `controller.errors` is empty if nothing went wrong. If the controller hits an error it restores the original signal
  programs and switches itself off; the simulation continues.

## What it changes in SUMO
Only the traffic lights on the ambulance's route ahead: phase durations, and (phase-skip) a slightly longer yellow, a 2 s all-red
and a jump to the ambulance's green. It does not change vehicle routes, speeds or any other signal.

## Not tested yet
Model D has been tested only on fixed routes (Scenario 0, Route 1 and Route 2). Running it on a route that Model C changes
during the run has **not** been tested. Please run one scenario with Model C + Model D, and one with `enabled=False`,
and check `controller.errors` and the signal events file before trusting the numbers.

## Test without SUMO
`python tests/test_controller_fake_sumo.py` must end with `ALL OK`.
