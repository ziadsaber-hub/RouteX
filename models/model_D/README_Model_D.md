# Model D — ETA and Priority-Request Scheduling

**Owner:** Salwa  
**Official deliverables:** `eta_priority.py`, `ETA_Tests.py`, `README.md`

## 1. What Model D does

Model D connects the route selected by Model C to Haneen's simulated traffic-signal controller. At every planning update it:

1. Reads the ambulance's current directed edge and distance along that edge.
2. Reads the remaining route and Model B's expected travel time and uncertainty for each remaining edge.
3. Calculates ETA and ETA uncertainty at upcoming signalized intersections in Model A's signal map.
4. Calculates queue-clearance time, safety margin, total preparation time, and the latest preparation-start deadline.
5. Marks a request `ON_TIME` or `LATE_REQUEST` and sends the agreed request fields to Haneen.
6. Re-submits changed ETAs, cancels requests for intersections removed by rerouting, and marks intersections already passed.

The code does not implement physical traffic-light hardware or communication protocols.

## 2. Official files

| File | Purpose |
|---|---|
| `eta_priority.py` | Model D equations, Model A/B/C loaders, dynamic ETA, uncertainty, queue/preparation calculations, request updates/cancellations, Haneen request adapter |
| `ETA_Tests.py` | Automated tests for position changes, traffic-time updates, queue updates, route changes, request timing, and data validation |
| `README.md` | Inputs, outputs, equations, units, assumptions, and run instructions |

`run_model_d_final.py` is included as an optional integration runner. It is not one of the three official deliverables.

## 3. Inputs and data sources

### Model A

- `RouteX_Model_A_Matrices.xlsx`, sheet `Edge index`.
- The loader uses Model A's original `matrix_edge_label`, `edge_index_0based`, `u_node_id`, `v_node_id`, `key`, `length_m`, and lane attributes.
- Edge labels and indices are preserved; the code checks that the indices are unique and contiguous.
- `RouteX_Signalized_Intersections_Clean.csv` identifies signalized nodes.
- `RouteX_Model_D_Approaches_Template.csv` maps the directed incoming edges and approach names to Haneen's controller.

**Current signal-map result:** node 142 is usable for simulated ambulance arrival priority. Node 4 is a field-verified signal, but has no incoming directed edge in the current graph, so Model D does not schedule an arrival request there. The four incoming approaches at node 142 are `J142_IN_S`, `J142_IN_NW`, `J142_IN_W`, and `J142_IN_E`.

For every incoming directed approach, Model D reads `lanes_numeric` directly from that exact Model A edge. The value is the **total number of lanes on the incoming road**. The project explicitly adopts total road lanes as the approach lane-count modelling assumption; movement-specific lane counts are not required. The approach-template `n` field is not used for queue clearance or controller capacity. A missing, zero, or negative Model A lane count is rejected.

### Model B

- `Model_B_for_Model_D.csv` (long format; one row per edge and time bin).
- `mu_hat_s` is the estimated whole-edge travel time in seconds.
- `sigma_hat_s_provisional` is used when supplied. If a row has no sigma, Model D falls back to `sigma_e = rho * mu_e` with provisional `rho = 0.15`.
- Model A edge IDs and edge indices are checked before use.

The currently uploaded export contains **3 time bins**, not the 7 bins mentioned in the project task sheet:

- 07:00–11:00
- 11:00–14:00
- 15:00–19:00

The 14:00–15:00 gap and times outside the supplied bins are not filled or extrapolated. A lookup outside the supplied intervals raises an error instead of silently inventing a time estimate. The simulation clock passed to Model D is seconds elapsed from the chosen simulation start; `load_model_b_csv` shifts the Model B seconds-since-midnight boundaries onto that clock.

### Model C

- `trip_routes.csv`: ordered directed edges and route versions.
- `trip_positions.csv`: timestamped node passages and the next directed edge.
- `route_changes.csv`: route-change metadata for auditing. The executable route update is read from the successive `route_version` records in `trip_routes.csv`.

The route loader maps Model C's `(from_node, to_node, key)` records to Model A's original edge labels. It does not assume that Model C's textual edge ID is the Model A edge label. The current upload contains 199 trips and route versions for 187 trips with a route change. All 6,812 route-edge records matched a directed Model A node pair during integration checking.

Between consecutive recorded node passages, `Trip.state_at()` estimates the current position along the directed edge by linear interpolation of the recorded times. The remaining part of the current edge is used in ETA calculations; a full-edge travel time is not charged again.

### Haneen

Model D passes the following exact fields to `SignalController.submit()`:

- `ev_id`
- `node_id`
- `ev_approach`
- `eta_j_s`
- `sigma_eta_s`
- `request_time_j_s`
- `feasible`

`request_time_j_s` is the latest time the controller should **begin preparing**. `issued_at_s` / `request_send_time_s` records when Model D sends or re-sends the request. These are distinct times. Haneen's current controller interface consumes the preparation deadline and feasibility flag; no communication delay is modelled.

## 4. Equations

All times are in seconds on the same simulation clock. Queue lengths are vehicles, lane counts are lanes, and discharge rates are vehicles/second/lane.

### Function 1 — ETA and uncertainty

For the ambulance's current edge, let `f = 1 - distance_along_m / edge_length_m`, clipped to `[0, 1]`. The remaining mean and standard deviation for that edge are scaled by `f`; later edges use their full values.

If Model B does not supply an edge standard deviation:

\[
\sigma_e = \rho\,\mu_e
\]

For each upcoming signalized intersection `j` on the selected directed route:

\[
ETA_j = t_{now} + \sum_{e\in P_j}\mu_{e,remaining}
\]

\[
\sigma_{ETA,j} = \sqrt{\sum_{e\in P_j}\sigma_{e,remaining}^{2}}
\]

The uncertainty equation assumes independent edge travel-time errors, as specified in the Model D documentation.

### Function 2 — Preparation and request time

\[
T_{queue,j}=\frac{N_{q,j}}{s\,n_{lanes,j}}
\qquad
T_{margin,j}=k\,\sigma_{ETA,j}
\]

\[
T_{prepare,j}=T_{clearance,j}+T_{queue,j}+T_{margin,j}
\]

\[
t_{request,j}=ETA_j-T_{prepare,j}
\]

A request is `ON_TIME` when the preparation deadline has not passed before preparation can start. Otherwise Model D returns `LATE_REQUEST` in its detailed result and passes `feasible=False` to Haneen for best-effort handling.

## 5. Assumptions and provisional values

The following values are used only because the corresponding measured/simulator inputs were not supplied. They must not be described as measured Dokki values.

| Parameter | Current value | Meaning / status |
|---|---:|---|
| `rho` | 0.15 | Fallback relative edge-time uncertainty, provisional; only used if Model B sigma is absent |
| `s` | 0.5 veh/s/lane | Queue discharge rate, provisional |
| `k` | 2 | ETA uncertainty safety multiplier |
| `T_clearance` | 5 s | Haneen default: 3 s yellow + 2 s all-red |
| `lambda` | 0.10 veh/s for every mapped approach by default | Provisional synthetic default; override per approach with `lam_by_approach` when justified |
| `red_s` | 30 s | Assumed red interval used to estimate a queue |
| `l_veh` | 7.5 m | Assumed queued vehicle spacing |
| `theta_eta_s` | 2 s | Provisional ETA/request update threshold |
| Normal green | 30 s/approach | Haneen's controller default, retained as an unverified provisional cycle setting |
| Initial green | First mapped approach unless overridden in `initial_green_by_node` | Provisional deterministic starting state; no agreed/verified initial phase was supplied |

The runner applies an arrival-rate default to **every** mapped approach at **every** usable signalized intersection, with optional per-approach overrides. It does not assume that only node 142 needs configuration. The 30 s normal green per approach is retained from Haneen's controller default, not claimed as a verified Dokki timing plan. Yellow/all-red remain Haneen's 3 s + 2 s defaults.
| Simulation step | 1 s | Integration-runner setting, not a measured property of the real signal |

Queue length is estimated by the optional `AssumedQueue` source:

\[
N_q=\min(\lambda\,T_{red},\;n_{lanes}\,L_e/l_{veh})
\]

This is an explicit provisional queue estimate. Replace it with simulator queue counts when those are available. The queue estimate uses Model A's total incoming-road `lanes_numeric` as the project's lane-count modelling assumption; movement-specific lane data is not required.

## 6. Tests

The revised `ETA_Tests.py` contains 16 tests, including checks that the signal loader takes lane counts from Model A `lanes_numeric`, ignores the approach-template `n` value, rejects non-positive Model A lane counts, and builds all configured approaches with default arrival rates.

Install the required packages:

```bash
pip install pandas numpy openpyxl
```

Run the automated tests:

```bash
python ETA_Tests.py
```

The tests cover:

- ETA from the remaining portion of the current edge;
- changing ambulance position;
- queue and Model B travel-time updates;
- strict time-bin boundaries and gaps;
- ETA uncertainty and provisional fallback;
- preparation and request-time equations;
- explicit late-request status;
- route changes, passed-intersection notifications, and cancellations;
- exact request fields expected by Haneen.

Haneen's supplied `Controller_Tests.py` was also run separately; all 21 controller tests passed.

## 7. Optional full-data integration runner

Place the Model A workbook, Model B CSV, signal CSVs, Model C trip CSVs, and Haneen's controller files in accessible folders. Run:

```bash
python run_model_d.py \
  --model-a RouteX_Model_A_Matrices.xlsx \
  --model-b Model_B_for_Model_D.csv \
  --signals-clean RouteX_Signalized_Intersections_Clean.csv \
  --signals-approaches RouteX_Model_D_Approaches_Template.csv \
  --routes trip_routes.csv \
  --positions trip_positions.csv \
  --haneen ./Haneen \
  --out ./model_d_output
```

By default, the runner uses Model C's recorded ambulance movement times (`--ambulance-times as_given`) while Model D predicts the remaining route with Model B's travel times. Use `--ambulance-times model_b` only for a consistency scenario in which the mock ambulance itself is re-timed using Model B. This second mode changes the mock movement timeline; it is not the raw Model C position trace.

The runner writes:

- `model_d_requests.csv` — requests sent to Haneen;
- `model_d_controller.csv` — controller status changes, feasibility, signal delay, ordinary-traffic cost, and recovery time;
- `model_d_trip_summary.csv` — ETA comparison, rerouting, controller statuses, and cost summary.

## 8. Integration check on the uploaded files

Using the uploaded Model A/B/C inputs and Haneen's controller:

- 546 Model A edges loaded and checked against Model B's edge indices.
- 1,638 Model B rows loaded (546 edges × 3 supplied time bins).
- 199 Model C trips loaded; 85 trips in the supplied trace reach the currently usable signalized node 142.
- All 6,812 Model C route-edge records mapped to Model A directed edges.
- The uploaded files contain 187 unique route-change events. Of the 85 trips that reach node 142, 84 have a route update, and those 84 updates were processed in the integration run.
- The controller was exercised for all 85 trips; request submission, route updates, releases, and deferred requests were observed.
- Full-run outputs: 6,117 request updates and 260 controller status records.
- The revised standalone Model D suite passes 16/16 tests; Haneen's supplied controller suite passes 21/21 tests.

With raw Model C positions, Model B's travel times are approximately 3 times the recorded mock road times. Therefore the first ETA can be substantially later than the recorded arrival. In this run, mean absolute first-ETA error was 152.7 seconds, while mean absolute last-ETA error fell to 1.2 seconds after movement and route updates. There were 347 late request updates; Haneen's controller returned 68 `BEST_EFFORT` status outputs. This is a mismatch between the two mock data sources, not measured model accuracy.

A second consistency run that re-times the mock ambulance using Model B produced effectively zero last-ETA error, as expected by construction. Neither run is a real-world performance result.

## 9. Remaining data needed for a fully validated simulation

These do not prevent the code from running, but they remain provisional or unverified:

1. Model B's remaining four time bins, if the team still intends to deliver seven bins.
2. Verified signal phase groups, initial green state, and legal turn permissions at signal node 142. Total incoming-road lanes from Model A are used by the stated modelling assumption; movement-specific lane counts are not required.
3. Actual signal phase groups, phase sequence, pedestrian minimums, and intersection-specific clearance timings.
4. Measured/simulator queue lengths and conflicting-approach arrival rates (current arrival rates are synthetic assumptions).
5. Validated discharge rates, `T_pass`, `G_max`, `C_max`, and the final ETA update threshold.
6. A live simulator feed for ambulance position/progress and a shared simulation clock.

Until those inputs are available, the numerical assumptions in this README and the integration output must remain labelled provisional/synthetic.
