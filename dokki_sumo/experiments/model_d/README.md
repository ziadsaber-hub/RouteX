# Model D – Traffic Signal Priority (Route 1: 181 → hospital_168, Route 2: node 4 → hospital_95)

**Using it with Model C: see `INTEGRATION_FOR_MODEL_C.md`.** Final configuration: `config/model_d_config_skip_stopped.json` (phase-skip + stopped-ambulance activation), default of the Route 1 runner. The Route 2 runner defaults to `model_d_config_skip.json`.

Model D gives the ambulance green lights on its **fixed Scenario 0 route** (no rerouting, that is Model C).
Everything else is identical to Scenario 0: same network, demand files, seed (1), step (1 s), departure (600 s),
disruptions (610 s), virtual stop line (TLS 172) and closure handling. The Scenario 0 runner
(`experiments/scenario_0/src/run_scenarios.py`) is imported, **not edited**; Model D hooks into it.

## Files
| File | Purpose |
|---|---|
| `src/model_d_controller.py` | ETA calculation + signal-priority controller + logs |
| `src/run_model_d_route1.py` | Runs the six conditions on Route 1 with Model D |
| `src/compare_with_baseline.py` | Time saved / improvement % vs Scenario 0 |
| `config/model_d_config.json` | All thresholds and tunable parameters |
| `tests/test_controller_fake_sumo.py` | Logic test with a fake TraCI (no SUMO needed) |

## Run (Windows, venv active, from the RouteX folder, on the `main`-based branch that contains `dokki_sumo`)
```
python dokki_sumo/experiments/model_d/tests/test_controller_fake_sumo.py
python dokki_sumo/experiments/model_d/src/run_model_d_route1.py --controller-off --scenarios S1 --output-tag sanity_off
python dokki_sumo/experiments/model_d/src/run_model_d_route1.py --scenarios S1 --output-tag quick_test
python dokki_sumo/experiments/model_d/src/run_model_d_route1.py
```
1. The test must print `ALL OK`.
2. `--controller-off` must reproduce the Scenario 0 numbers (S1 = 325 s, S2 = 962 s). If not, the hook is wrong.
3. Then run all six conditions. Results go to `dokki_sumo/experiments/model_d/outputs/route_1_model_d/`.

Dependencies: the same as Scenario 0 (`requirements-scenario0.txt`), SUMO 1.28.0, Python 3.13.

## Signals on Route 1
TLS 172 (approach `e192to172k0`), TLS 28 (`e122to28k0`), TLS 203 (`e168to203k0`). All are static, 90 s cycle.
Ambulance green per cycle: 172 → 13 s, 28 → 41 s, 203 → 27 s.

## Method (per simulation second)
1. ETA to each signalized intersection ahead: `ETA = t + Σ μ_e` (SUMO edge travel time), `σ_ETA = √Σ(ρ μ_e)²`.
2. `T_prepare = T_clearance + T_queue + k σ_ETA`; `t_request = ETA − T_prepare`.
3. Decision, in this order:
   - the unmodified program already gives a green covering the arrival → do nothing;
   - a natural green can be held (≤ `g_max_s`) until the ambulance passes → wait for it, then extend it;
   - a compressed cycle (other greens cut to `min_green_other_phase_s`) can produce such a green → cut other greens from now;
   - otherwise best effort at `t_request`.
4. Phase durations are changed with `setPhaseDuration`. With `phase_skip` the controller can also extend a yellow slightly, apply a 2 s all-red and jump to the ambulance's green; the original program is restored afterwards or after any error.

## Outputs (per scenario folder and merged at the top level)
`model_d_main_results.csv`, `model_d_eta_records*.csv`, `model_d_signal_events*.csv`,
`model_d_intersection_analysis*.csv`, `model_d_vs_baseline_route1.csv` (time saved, improvement %, waiting and
signal-stop reduction, effect on ordinary traffic waiting time).

## Known limits (read before reporting results)
- Route 1 was run on real SUMO for all six conditions (controller off reproduces Scenario 0 exactly). Final results: seeds 1–3 (see the Model D final report).
- ETA is queue-aware (`queue_aware_eta`, default true; set false to reproduce the first version). It did not change the ambulance times: most waiting is at TLS 172 where the ambulance approach has only 13 s of green per 90 s cycle.
- With a 13 s green in a 90 s cycle and 5 s minimum greens, some arrival timings cannot be served exactly
  (the fake-SUMO test: 4 of 30 timings stay red; with 3 s minimum green none do). `min_green_other_phase_s`
  and `g_max_s` are the knobs; changing them must be documented as an experimental variable.
- σ uses the provisional ratio ρ = 0.15 (not measured). Queue discharge rate 0.5 veh/s/lane is an assumption.
- `ambulance_stopped_on_approach_s` counts seconds with speed < 0.1 m/s on the approach edge, so it includes queue waiting,
  not only red-light waiting. `model_d_ambulance_signal_wait_s` is the red-signal part only.
- Route 2 (`run_model_d_route2.py`): controller-off matches the baseline in five of six conditions; S4 differs by 2 s (552 s vs 554 s).
- Not implemented from the design rules: R6 (clashing requests from different approaches), the warning to the route planner (Model C link). R5 and R7 are simplified (release when the ambulance crosses; ETA recomputed every second).

## Multiple ambulances (extra experiment)
Same route and OD pair (181 -> hospital_168), conditions S1-S4 (no closures). ambulance_1 departs at 600 s as in Scenario 0;
ambulance_2..N depart before it, every `--gap` seconds, on the same route. One controller per ambulance; a held green also
covers ambulances following within `convoy_window_s` (45 s).
```
python dokki_sumo/experiments/model_d/src/run_model_d_multi.py --controller-off --output-tag multi_3_off
python dokki_sumo/experiments/model_d/src/run_model_d_multi.py --output-tag multi_3_model_d
python dokki_sumo/experiments/model_d/src/compare_multi.py
```
Output: `model_d_multi_vs_off.csv` (per ambulance: travel time with Model D off/on, time saved, improvement %).
Ambulances that have not arrived when ambulance_1 arrives are reported as not completed.

## Route 2 (node 4 -> hospital_95)
Uses `run_scenarios_pair2.py` (fixed 20-edge route; signals TLS 31 and TLS 142; ambulance overtaking and closure handling as in Scenario 0).
```
python dokki_sumo/experiments/model_d/src/run_model_d_route2.py --controller-off --output-tag r2_sanity_off
python dokki_sumo/experiments/model_d/src/run_model_d_route2.py --output-tag route_2_model_d
python dokki_sumo/experiments/model_d/src/compare_traffic_fair.py dokki_sumo/experiments/model_d/outputs/route_2_model_d dokki_sumo/experiments/scenario_0/outputs/final/route_2 route2
```
1. `r2_sanity_off` must reproduce the Route 2 baseline travel times (`scenario_0/outputs/final/route_2/all_scenarios_summary.csv`).
2. Output: `model_d_vs_baseline_route2.csv` and `model_d_traffic_fair_route2.csv`. Same controller and the same constants as Route 1.
