# Model C → SUMO: export interface (for Ali)

Written by `task3_export.py` into `model_c_output/`. Time is in **seconds**. Example rows are real output.

## Edge IDs
`"u-v-k"` = the (u, v, key) of one row of the Model A Edges sheet (= `u_node_id`, `v_node_id`, `key` in `Edge_Index.csv`).
It fixes the direction (u → v). Parallel roads between the same two intersections differ in `k` and are never merged.
Node IDs are integers ≥ 0, so the ID can be split on `-`. Model A node IDs are **not** SUMO node IDs: map every
edge through `edge_mapping.csv` (one Model A edge may become several SUMO edges, in order).

## Files

**`trip_routes.csv`** – the route in force, one row per edge, per route version. Version 0 is the initial route; every
route change adds a new full version (driven edges unchanged, the rest replaced).

| trip_id | route_version | valid_from_s | reason | order | edge_id | from_node | to_node | key | derived_reverse |
|---|---|---|---|---|---|---|---|---|---|
| 000:37->158 | 0 | 0.0 | initial | 0 | 37-54-0 | 37 | 54 | 0 | 0 |

`reason` is `initial`, `closed_road` (forced, ignores the 10 % rule) or `time_saving` (new route is more than 10 % faster).
To follow the route in SUMO use the version with the largest `valid_from_s` ≤ the current time.

**`trip_positions.csv`** – where the ambulance is, one row per intersection reached (the model moves edge by edge).

| trip_id | time_s | node | next_node | next_edge_id | signal_node | signal_approach_edge_id | signal_exit_edge_id | signal_eta_s |
|---|---|---|---|---|---|---|---|---|
| 001:18->69 | 0.0 | 18 | 20 | 18-20-0 | 142 | 16-142-0 | 142-151-0 | 47.62 |

`signal_*` = the next verified signal still ahead (empty if none): the signal node, the edge arriving at it, the edge
leaving it, and the time until the ambulance has **passed** it (this includes the signal's own Delta T from Model D; to be
agreed with Model D whether it should be the time until it **reaches** it). The last row of a trip has an empty `next_edge_id`.

**`route_changes.csv`** – one row per change: `trip_id, time_s, at_node, reason, remaining_on_old_route_s, new_route_s`
(`remaining_on_old_route_s` is empty for `closed_road`).

**`trip_summary.csv`** – one row per trip: `trip_id, finished, planned_time_s, total_time_s, route_changes`.
`planned_time_s` = time of the initial route; `total_time_s` = simulated time in Model C's own model (not SUMO).
`finished = 0` means the trip got stuck because a closure cut the destination off and no detour exists
(12 of 200 in the demo). Compare these with the SUMO values for *planned vs actual* travel time.

## Trip IDs
`<running number>:<origin>-><destination>`, unique per file. The same origin and destination can occur in two trips.

## From Python instead of CSV
```python
from task3_export import run_trip
trip = run_trip(route, events, live=my_live)   # 'positions', 'routes', 'changes', 'summary'
```
Closures from SUMO: `live.close_street(u, v)` closes every parallel key u → v in that direction only.

## What Model C does not do yet
- It does not run in SUMO; its times come from Model A free-flow (Model B file not received yet).
- Only 2 intersections are verified as signalized (4 and 142); the list will grow with the real and virtual signals.
