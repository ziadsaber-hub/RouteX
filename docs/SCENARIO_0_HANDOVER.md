# RouteX — Dokki SUMO Scenario 0 Technical Handover

## 1. Purpose and handover status

This document hands the completed Dokki SUMO Scenario 0 environment to the teams responsible for Model B (traffic/energy or travel-time estimation), Model C (routing/path planning), and Model D (traffic-light control).

**Validation status: 12/12 ambulance experiments completed.**
- Route 1: S1–S6 completed.
- Route 2: S1–S6 completed.
- Ambulance collision count: zero in every validated run.
- Each closure scenario (S5 and S6) produced one successful ambulance route update.

The experiment is designed around ambulance performance. Background vehicles provide the traffic environment. Their individual trip completion is not a pass/fail condition; some trips remain unfinished when the simulation stops after ambulance arrival.

## 2. Repository contents and cleanup

The cleaned repository retains:

- `dokki_sumo/data/model_a/`: Model A Excel source and original/cleaned GeoPackages.
- `dokki_sumo/network/`: generated SUMO network, node/edge/type source XML, mappings, and `netconvert.log`.
- `dokki_sumo/simulation/`: source demand route, traffic-light and hospital additional files, SUMO GUI configs, background settings, and visualization assets.
- `dokki_sumo/experiments/scenario_0/config/`: ambulance vehicle type and SUMO configuration.
- `dokki_sumo/experiments/scenario_0/demand/`: final normal, moderate, and heavy route files.
- `dokki_sumo/experiments/scenario_0/src/`: final Route 1 and Route 2 runners plus the demand generator.
- `dokki_sumo/experiments/scenario_0/outputs/final/`: only the validated final raw outputs and consolidated tables for both routes.

Excluded from the active handover:
- `.bak` demand-file copies.
- The empty `runner.py`.
- Earlier one-off/prototype runners that are not used by the final Scenario 0 workflow.
- Duplicate ZIP exports of output folders.
- Old Route 2 S5/S6 attempts that did not complete.
- Lane-change diagnostic runs and intermediate draft folders.
- Duplicate empty nested directories.

The original uploaded archive is not modified. This cleaned package is a separate, repository-ready copy.

## 3. Network and simulation environment

### 3.1 Map and network

The Dokki SUMO network was built from Model A source data:
- `SOURCE_RouteX_Road_Data_Calculated.xlsx`
- `dokki_graph_cleaned.gpkg`

The map builder is `dokki_sumo/src/make_network.py`. It converts source coordinates from EPSG:4326 to EPSG:32636 (UTM Zone 36N), creates the SUMO node/edge/type XML files, and invokes SUMO network conversion. The generated network is `dokki_sumo/network/dokki.net.xml`.

Network inventory read from the final `.net.xml`:
- 226 non-internal junctions
- 551 external road edges
- 6 traffic-light logic definitions

Keep the Model A source data with the generated `.net.xml`: the source files allow the next team to inspect or regenerate the network rather than treating the binary-like network XML as an unexplained artifact.

### 3.2 Software versions

Validated development environment:
- Windows
- Python 3.13.9
- Eclipse SUMO 1.28.0
- TraCI 1.28.0
- SUMOLib 1.28.0
- NetworkX 3.7

The scenario runners use TraCI by default. They contain a hard-coded SUMO executable path:
`C:\Program Files (x86)\SUMO\bin\sumo.exe`

If SUMO is installed elsewhere, update the `SUMO` constant in both:
- `dokki_sumo/experiments/scenario_0/src/run_scenarios.py`
- `dokki_sumo/experiments/scenario_0/src/run_scenarios_pair2.py`

Python dependencies for the experiment runners are listed in `requirements-scenario0.txt`. Optional map-building dependencies are listed separately in `requirements-map.txt`.

### 3.3 Shared simulation settings

Both route experiments use:
- Random seed: 1
- Simulation step length: 1 s
- Ambulance scheduled departure: 600 s
- Disruption activation: 610 s (10 s after the initial route/departure setup)
- Ambulance vehicle type: `ambulance`
- `time-to-teleport = -1` (teleporting disabled)
- Collision action: warn
- Junction collision checking enabled
- Simulation ends after ambulance arrival for successful runs; the configured maximum simulation horizon is 7,200 s.

The ambulance type is defined in `config/ambulance.add.xml`:
- Acceleration: 2.0 m/s²
- Deceleration: 4.5 m/s²
- Emergency deceleration: 7.0 m/s²
- Length: 6 m
- Maximum speed: 16.67 m/s

### 3.4 Virtual stop line

The same virtual stop-line setup is retained:
- Traffic-light ID: `172`
- Edge: `e39to172k0`
- Stop position: 90 m
- Controlled link indices: 16, 17, 18, 19
- Stop duration: 1,000,000 s

Do not silently change this setup while comparing Model B/C/D results. If Model D changes signal logic or priority behaviour, record the change as an experimental variable.

## 4. Traffic demand and six scenario conditions

The original source demand is `dokki_sumo/simulation/dokki_random.rou.xml`, containing 600 vehicles. `generate_scenarios.py` creates three deterministic subsets by preserving the original vehicle order:
- `normal.rou.xml`: first 200 vehicles
- `moderate.rou.xml`: first 400 vehicles
- `heavy.rou.xml`: all 600 vehicles

The six conditions are:

| ID | Name | Demand | Applied condition |
|---|---|---:|---|
| S1 | Normal | 200 | Baseline normal demand; no disruption |
| S2 | Moderate | 400 | Baseline moderate demand; no disruption; reference for within-route comparisons |
| S3 | Heavy | 600 | Heavy demand; no disruption |
| S4 | Localized congestion | 400 | Moderate demand; four consecutive route edges slowed to 25% of their original speed limit |
| S5 | Road closure | 400 | Moderate demand; two ambulance-route edges and two nearby edges closed |
| S6 | Combined | 400 | Same closures as S5 plus the S4 four-edge congestion corridor at 25% speed |

All disruptions are activated at simulation time 610 s. S2 is the reference condition for the change columns in the result tables below. Comparisons are made against each route's own S2 result, not across the two different origins/destinations.

## 5. Route 1 — origin 181 to hospital_168

### 5.1 Reference route

- Origin node: `181`
- Destination: `hospital_168`
- Reference route: 22 edges
- Route file: `outputs/final/route_1/reference_route.txt`

Ordered edge sequence:

```text
e181to158k0
e158to115k0
e115to33k0
e33to113k0
e113to112k0
e112to110k0
e110to105k0
e105to191k0
e191to189k0
e189to190k0
e190to196k0
e196to192k0
e192to172k0
e172to121k0
e121to122k0
e122to28k0
e28to214k0
e214to168k0
e168to203k0
e203to216k0
e216to167k0
e167to168k0__to_hospital_168
```

### 5.2 S4 congestion corridor

The four consecutive edges slowed to 25% of their original speed limits are:
- `e28to214k0`: 11.11 → 2.7775 m/s
- `e214to168k0`: 11.11 → 2.7775 m/s
- `e168to203k0`: 8.33 → 2.0825 m/s
- `e203to216k0`: 8.33 → 2.0825 m/s

### 5.3 S5/S6 closure set

Route closures:
- `e196to192k0`
- `e192to172k0`

Nearby alternative-road closures:
- `e128to196k0`
- `e172to193k1`

S6 applies the S4 congestion corridor in addition to these four closures.

### 5.4 Route 1 results

| Condition | Traffic input | Disruption | Ambulance travel time | Change vs S2 | Waiting time | Route updates | Ambulance collisions |
|---|---:|---|---:|---:|---:|---:|---:|
| S1 Normal | 200 | None | 325 s | −637 s (−66.2%) | 11 s | 0 | 0 |
| S2 Moderate | 400 | None (reference) | 962 s | 0 s | 562 s | 0 | 0 |
| S3 Heavy | 600 | None | 1,582 s | +620 s (+64.4%) | 1,136 s | 0 | 0 |
| S4 Localized congestion | 400 | Four-edge corridor at 25% speed | 1,283 s | +321 s (+33.4%) | 564 s | 0 | 0 |
| S5 Road closure | 400 | Two route edges + two nearby edges closed | 508 s | −454 s (−47.2%) | 153 s | 1 | 0 |
| S6 Combined | 400 | S5 closures + S4 congestion | 542 s | −420 s (−43.7%) | 155 s | 1 | 0 |

Route 1 S4 validation: 1,283 − 962 = **+321 s** relative to S2. This matches the terminal validation message.

S5 and S6 both completed. Each used one ambulance route update and recorded zero ambulance collisions. The S5/S6 result rows report 26 background-traffic reroutes.

**Route 1 closure recovery policy:** the ambulance uses a stop-triggered reactive recovery policy with a 125 m closure lookahead and a 3 s stop trigger. Background vehicles are also given closure information and are rerouted by the Route 1 runner at 5 s checks. The validated S5/S6 result has 26 such background reroutes.

## 6. Route 2 — origin 4 to hospital_95

### 6.1 Reference route

- Origin node: `4`
- Destination: `hospital_95`
- Reference route: 20 edges
- Configured sum of reference-edge lengths: 2,381.99 m
- Full route is recorded in `outputs/final/route_2/pair_configuration.json`.

Ordered edge sequence:

```text
e4to6k0
e6to21k0
e21to22k0
e22to24k0
e24to200k0
e200to201k0
e201to29k0
e29to31k0
e31to144k0
e144to143k0
e143to146k0
e146to145k0
e145to16k0
e16to142k0
e142to151k0
e151to152k0
e152to153k0
e153to86k0
e86to95k0
e95to89k0__to_hospital_95
```

Note: the configured sum of reference-edge lengths (2,381.99 m) and the simulation summary's `ambulance_distance_m` (2,647.26 m in S1–S4) are different output quantities. Use the same field consistently when comparing model results; do not substitute one for the other.

### 6.2 S4 congestion corridor

The four consecutive edges slowed to 25% of their original speed limits are:
- `e200to201k0`: 11.11 → 2.7775 m/s
- `e201to29k0`: 8.33 → 2.0825 m/s
- `e29to31k0`: 11.11 → 2.7775 m/s
- `e31to144k0`: 16.67 → 4.1675 m/s

### 6.3 S5/S6 closure set

Route closures:
- `e144to143k0`
- `e86to95k0`

Nearby closures:
- `e13to16k0`
- `e140to142k0`

S6 applies the S4 congestion corridor in addition to these four closures.

### 6.4 Route 2 results

| Condition | Traffic input | Disruption | Ambulance travel time | Change vs S2 | Waiting time | Route updates | Ambulance collisions |
|---|---:|---|---:|---:|---:|---:|---:|
| S1 Normal | 200 | None | 288 s | +1 s (+0.3%) | 41 s | 0 | 0 |
| S2 Moderate | 400 | None (reference) | 287 s | 0 s | 41 s | 0 | 0 |
| S3 Heavy | 600 | None | 284 s | −3 s (−1.0%) | 42 s | 0 | 0 |
| S4 Localized congestion | 400 | Four-edge corridor at 25% speed | 554 s | +267 s (+93.0%) | 99 s | 0 | 0 |
| S5 Road closure | 400 | Two route edges + two nearby edges closed | 398 s | +111 s (+38.7%) | 41 s | 1 | 0 |
| S6 Combined | 400 | S5 closures + S4 congestion | 421 s | +134 s (+46.7%) | 54 s | 1 | 0 |

**Route 2 closure recovery policy:** S5/S6 use the later successful activation-reroute implementation. At 610 s, immediately after disruption activation, the ambulance reroutes once and avoids the previously observed physical bottleneck edges `e31to144k0` and `e170to38k0`. The route-2 runner does not reroute, teleport, or optimize ordinary traffic. Background vehicles that do not complete before ambulance arrival are accepted.

The successful Route 2 S5/S6 raw outputs are taken from `pair2_activation_reroute_test`. The older `pair_4_to_hospital_95_final` S5/S6 attempts did not complete and are intentionally excluded from the final validated outputs. Its S1–S4 results are retained because those four conditions completed and were not changed by the later S5/S6 recovery-policy fix.

## 7. Cross-route interpretation and comparison rules

Do not directly rank Route 1 and Route 2 by raw travel time. They have different origins, destinations, road sequences, and route-specific disruption locations. The meaningful comparisons are:

1. Within Route 1: compare each system's result against the Route 1 S2 baseline and the same Route 1 scenario.
2. Within Route 2: compare each system's result against the Route 2 S2 baseline and the same Route 2 scenario.
3. Compare Model B/C/D against the same demand file, seed, departure, route pair, closures, congestion corridor, and signal setup.

There is also a methodology difference in S5/S6:
- Route 1: stop-triggered reactive ambulance reroute; background traffic reroutes (26 observed).
- Route 2: immediate ambulance reroute at disruption activation; no background-traffic rerouting.

Therefore, Route 1 and Route 2 S5/S6 values are valid route-specific baseline results, but the closure-response policies are not identical. Any cross-route comparison of closure performance must disclose this difference. For a strict controlled comparison, use the same recovery policy in both route runners and regenerate the affected baseline runs.

## 8. Output files and what each team should use

Each scenario folder contains detailed raw outputs. The principal files are:

| File | Contents | Best use |
|---|---|---|
| `scenario_summary.csv` | One-row scenario summary: completion, travel time, waiting, distance, reroutes, collisions, demand/traffic totals | Main result table |
| `ambulance_positions.csv` | Ambulance position, lane, coordinates, speed and acceleration sampled every simulation second | Trajectory, speed profile, stops |
| `ambulance_edge_times.csv` | Ambulance edge entry/exit times and per-edge travel duration/speed | Per-edge ground truth for Model B/C |
| `ambulance_route.csv` | Reference route edges and disruption flags | Route geometry and disruption mapping |
| `ambulance_route_history.csv` | Route versions and reroute history | Model C baseline/reroute comparison |
| `scenario_events.csv` | Disruption activation and reroute events | Event timeline |
| `edge_metrics.csv` | Per-edge aggregated average speed, average occupancy, maximum observed vehicles, observed sample count and last-seen time | Traffic-state features and corridor diagnosis |
| `traffic_summary.csv` | Demand, completed/unfinished trips, average duration/waiting/time-loss/speed | Overall traffic context |
| `traffic_tripinfo.csv` / `.xml` | Per-vehicle trip information | Traffic impact and distribution analysis |
| `collisions.csv` / `.xml` | Collision records | Safety validation |

Master files:
- `outputs/final/all_routes_summary.csv`: normalized summary for all 12 route-condition combinations.
- `outputs/final/route_comparison_vs_S2.csv`: within-route changes versus each route's S2 baseline.
- `outputs/final/route_1/all_scenarios_summary.csv` and `route_2/all_scenarios_summary.csv`: route-specific detailed master summaries.
- `outputs/final/route_1/route_configuration.json` and `route_2/pair_configuration.json`: route and disruption configuration.

### Important limitation for Model B

`edge_metrics.csv` is an **aggregated per-edge file**, not a timestamped edge-by-edge time series. `ambulance_positions.csv` is time-indexed for the ambulance, but it is not a full network-wide traffic-state time series.

If Model B requires traffic speed/occupancy on every edge at each time step (or every 5 s), the logger must be extended to write a time-indexed table such as:

```text
time_s,scenario_id,edge_id,vehicle_count,mean_speed_mps,occupancy_percent
```

The current `--edge-metric-interval` option changes how often the network is sampled, but the current output aggregation still summarizes by edge over the run. Do not mistake that for a time-series export.

## 9. How the next teams should use this handover

### Model B — traffic / travel-time estimator

Use:
- the matching scenario demand file;
- `edge_metrics.csv` for aggregated edge-level traffic features;
- `ambulance_edge_times.csv` as actual ambulance edge traversal ground truth;
- `traffic_tripinfo.csv` for vehicle-level traffic context;
- `scenario_events.csv` and the route configuration JSON to identify exactly when and where congestion/closures were applied.

Recommended validation:
- predict per-edge or route travel time;
- compare predicted vs observed edge/route times;
- report MAE and RMSE (and MAPE only where observed values are safely non-zero);
- evaluate separately for S1–S6 and for each route;
- add timestamped network-wide edge logging if the estimator is intended to be dynamic.

### Model C — path planning / rerouting

Use the exact origin/destination and the same six conditions for each route. Keep demand, seed, activation time, and disruption edges fixed.

Compare against the baseline:
- ambulance completion;
- total travel time;
- waiting time;
- distance;
- number and timing of route updates;
- route history and avoided/closed edges;
- ambulance collisions.

Do not count unfinished background trips as a Model C failure. The ambulance is the primary evaluated vehicle.

### Model D — traffic-light control

Keep the same demand files, route pair, closures, congestion corridor, seed, and departure. Change only the signal-control policy being evaluated.

Compare:
- ambulance travel time and waiting time;
- per-edge approach delay and stops;
- ambulance speed profile;
- signal delay at controlled intersections;
- traffic-side impact where relevant.

The baseline virtual stop-line configuration at TLS 172 is part of the recorded setup. If Model D changes it, clearly document the changed signal logic and preserve an unchanged baseline run for comparison.

## 10. Re-running and independently checking the experiments

### 10.1 Install the environment

From the repository root in PowerShell:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements-scenario0.txt
sumo --version
python --version
```

Expected versions: Python 3.13.x and SUMO 1.28.0. The recorded validated Python patch version was 3.13.9.

If SUMO is not found at the path hard-coded in the runner scripts, update the `SUMO` constant in both runners.

### 10.2 Generate demand files

Only needed if the three final demand files are being regenerated from the source 600-vehicle route file:

```powershell
python dokki_sumo/experiments/scenario_0/src/generate_scenarios.py
```

Expected counts:
- normal: 200
- moderate: 400
- heavy: 600

### 10.3 Run all six Route 1 conditions

```powershell
python dokki_sumo/experiments/scenario_0/src/run_scenarios.py --output-tag route1_reproduction
```

Route 1 always creates a fresh S2 reference first, then runs the other requested scenarios. With no `--scenarios` argument, it runs the complete set.

### 10.4 Run all six Route 2 conditions

```powershell
python dokki_sumo/experiments/scenario_0/src/run_scenarios_pair2.py --output-tag route2_reproduction
```

Route 2 defaults to S1–S6. Use a new output tag for each run.

### 10.5 Re-run only closure scenarios

```powershell
python dokki_sumo/experiments/scenario_0/src/run_scenarios.py --scenarios S5,S6 --output-tag route1_closure_recheck
python dokki_sumo/experiments/scenario_0/src/run_scenarios_pair2.py --scenarios S5,S6 --output-tag route2_closure_recheck
```

The Route 1 command still creates a fresh S2 reference before running S5/S6. Route 2 runs only the selected scenarios.

### 10.6 Verify a run

For each route, verify:
1. `all_scenarios_summary.csv` contains the requested scenario IDs.
2. `ambulance_completed` is `True` for each scenario.
3. `ambulance_collision_count` is zero.
4. S5 and S6 each show one successful route update and zero route-update failures.
5. S4 has a higher travel time than S2 because the designated corridor was slowed.
6. `disruption_edge_selection.csv` / `pair_configuration.json` list the expected closure and congestion edges.
7. `scenario_events.csv` shows disruption activation at 610 s and a successful reroute for S5/S6.

A background vehicle remaining unfinished when the simulation stops after ambulance arrival is not a failure.

## 11. GitHub handover

The cleaned package is structured to be committed from the repository root. After extracting/copying it into the team's repository:

```powershell
git status
git add README.md docs requirements-scenario0.txt requirements-map.txt .gitignore dokki_sumo
git commit -m "Prepare Dokki SUMO Scenario 0 handover"
git push origin <your-branch>
```

Before pushing, check that no local virtual environment, temporary output folder, or machine-specific file was added. Keep the validated final outputs under `outputs/final/`; use a distinct output tag for future runs so the handover evidence is not overwritten.

## 12. Final handover checklist

- [x] Model A source data retained.
- [x] Generated Dokki SUMO network and mappings retained.
- [x] Normal/moderate/heavy demand files retained.
- [x] Route 1 reference route and six validated scenario outputs retained.
- [x] Route 2 reference route and six validated scenario outputs retained.
- [x] Successful Route 2 S5/S6 rerun selected instead of the older failed attempts.
- [x] Consolidated 12-row result table and within-route comparisons created.
- [x] Model B/C/D integration guidance documented.
- [x] Reproduction commands and verification checks documented.

**Scenario 0 is ready for handover.** The next team should treat the final raw outputs and configuration files in `outputs/final/` as the baseline evidence and avoid changing them in place.
