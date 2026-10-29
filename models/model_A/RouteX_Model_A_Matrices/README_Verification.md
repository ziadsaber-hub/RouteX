# RouteX — Model A Matrix Verification

Source: `SOURCE_RouteX_Road_Data_Calculated.xlsx` (`Nodes` and `Edges` sheets).

## Generated matrix dimensions and index rules

| Matrix | Dimensions | Meaning | Storage |
|---|---:|---|---|
| A | 222 × 222 | Binary directed adjacency, A[i,j]=1 iff at least one road edge from node i to node j | A_Adjacency_Dense.csv, A_adjacency_sparse.npz |
| B | 222 × 546 | Directed incidence, -1 at departing node, +1 at arriving node | B_Incidence_Dense.csv, B_incidence_sparse.npz |
| F | 546 × 4 | [length_m, lanes_count, reference_speed_kph, reference_free_flow_s] | F_Feature_Matrix.csv, F_features_numeric.npy |

**Zero-based row and column indexing.** Node order is ascending `node_id` as in `Node_Index.csv`. In this particular dataset, node IDs are 0–221, so row index equals node ID. **Edge order follows the 546 input workbook rows**; edge index 0 corresponds to original worksheet row 2. `Edge_Index.csv` maps each edge column to the unique `(u,v,key)` identifier and road name. Do not re-sort the edge list independently.

Each directed road gets its own column in B and row in F, even if several roads share the same start and end nodes. A is deliberately **binary**, so parallel roads occupy one adjacency entry.

## Results of independent checks

- Node IDs unique: PASS (222/222)
- `(u,v,key)` identifiers unique: PASS (546/546)
- Every edge endpoint exists in node index: PASS
- Self-loop edges: 0
- Every edge length and reference speed is positive and finite: PASS
- A is binary and has 504 nonzero entries: PASS
- Each of 546 columns in B has exactly one -1 and one +1: PASS
- All B column sums are zero: PASS
- B rank = **221** = n - 1: PASS
- Weakly connected components = **1**: PASS
- Strongly connected components = **11** (largest component has 206 nodes): **CHECK routing accessibility**; weak connectivity does not guarantee direction-respecting connectivity.
- A and B agree with each edge's endpoints: PASS
- Baseline time t0 = 3.6 × length_m / reference_speed_kph, in **seconds**, recomputed at full precision: PASS
- Original workbook's t0 rounded to 2 decimals; maximum absolute difference = **0.004977 seconds**, within half of 0.01 s: PASS
- Independent directed shortest-path example node 0 → node 5: 22 edges, 240.604747685 s; **Bx=b and vector route cost vs. Dijkstra agree**: PASS
- Save/reload test for SciPy NPZ A/B and NumPy NPY F: PASS
- Blank lane values in F for two unresolved **mixed OSM lane tags**, intentionally preserved rather than guessed: 
  - Edge index 415; source worksheet row 417: `(208,165,0)`, raw `['2', '4']`
  - Edge index 539; source worksheet row 541: `(205,216,0)`, raw `['2', '4']`

## Limitations requiring validation before use in routing

1. **This is a calculation from the cleaned Excel workbook**, not from your teammate's final QGIS GeoPackage. The latter was not supplied for cross-checking. Verify that the workbook edge/node IDs are identical to the final saved QGIS layers.
2. Missing/ambiguous lane counts: F has **NaN** for 2 lanes, and CSV has empty cells. Algorithms that use lanes must impute/resolve them first. Do not treat NaN as zero.
3. Reference speeds of 30, 40 and 60 km/h are copied from the workbook; **their actual provenance/posted status is not established**. They are *not* a measured EV speed or current congestion speed.
4. The time values **exclude** congestion, intersection queues, signals, emergency-vehicle behavior, and turn penalties. Model B must supply changing travel-time estimates.
5. A and B represent the given directed edges, **not turn-restriction logic**; legal turn sequences need additional constraints.
6. The Excel graph includes multiple distinct directed edges between some ordered node pairs. Do not reconstruct B solely from binary A; use Edge_Index.
7. The illustrative test route from node 0 to node 5 is for mathematical verification only; it is **not a validated emergency mission**.

## How to use

- Open `RouteX_Model_A_Matrices.xlsx` for the guide, road-index mapping, F matrix and sparse entries for A and B.
- For actual computation in Python:
```python
import numpy as np
import scipy.sparse as sp

A = sp.load_npz("A_adjacency_sparse.npz")
B = sp.load_npz("B_incidence_sparse.npz")
F = np.load("F_features_numeric.npy")

times = F[:, 3]                  # baseline seconds
assert B.shape[1] == len(times)  # same edge ordering
```
- For viewing the complete numeric matrices without Python, open `A_Adjacency_Dense.csv`, `B_Incidence_Dense.csv` or `F_Feature_Matrix.csv`.
- `build_and_verify_matrices.py` rebuilds and verifies A/B/F from `Node_Index.csv` and `Edge_Index.csv` with `numpy`, `scipy`, and `networkx`.

## Mathematical definitions

A[i,j] = 1 if ∃ e=(i,j,k) in E, else 0.

B[i,e] = -1 for the outgoing/tail vertex of e; +1 for the incoming/head vertex; 0 otherwise.

F[e,:] = [L_e (meters), n_e (lanes), v_e^ref (km/h), t_e^0 (seconds)].

t_e^0 = 3.6 L_e / v_e^ref. The full-precision result is used in matrix F. The source Excel has rounded-to-2-decimal values.

For an ambulance route x∈{0,1}^546 from origin s to destination d:

B x = b, where b[s]=-1, b[d]=+1, and all other b[i]=0.

Baseline path cost: (F[:,3])ᵀ x. This is a conceptual mathematical formulation; route-feasibility constraints beyond simple flow conservation may be required.
