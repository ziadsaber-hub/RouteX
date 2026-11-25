#!/usr/bin/env python3
"""
model_b_estimator.py - the Model B travel-time estimator (Step 3 of the task).

Implements the APPROVED methodology (Model_B_final_.pdf): weighted least squares with
spatial and temporal regularisation, unknowns mu_e(t) = travel time in SECONDS.

    J = alpha * sum_{(e,t) in Omega} w_et (mu_e(t) - y_e(t))^2                     (3.1 fidelity)
      + beta  * sum_t sum_{(e,k) in S} s_ek (mu_e(t)/t0_e - mu_k(t)/t0_k)^2        (3.2 spatial)
      + gamma * sum_e sum_{t>=2} (mu_e(t) - mu_e(t-1))^2                           (3.3 temporal)

t0_e = 3.6 L_e / v_e is the free-flow time (L in m, v in km/h); it equals
model_a_reference_time_s in the data (checked: max difference 7e-15 s).
Minimising J gives the sparse linear system  H mu = alpha W_Omega y  with
    H = alpha W_Omega + beta (I_T (x) D0^-1 L^E D0^-1) + gamma (L_T (x) I_m)
(this code stores mu edge-major, i*T + t, so the Kronecker factors are swapped; same matrix).
H is symmetric positive semidefinite; the solution is unique when the observations
anchor every connected component of the road-similarity graph.

Numerical safeguard (NOT part of the approved objective): a tiny ridge (1e-6)
towards the global mean congestion ratio x t0 is added so the solver never fails
on a component with no observation.  fit() reports how many edges are in such
unanchored components (unanchored_edges_); when it is 0 the ridge has no practical effect.

OBSERVATION WEIGHTS: w_et = 1 for an observed cell and 0 for a missing one.  The method
allows confidence weights; none have been approved, so none are used.

SPATIAL SIMILARITY RULE - STILL A PROPOSAL.  The methodology defines S (neighbouring pairs,
each counted once) and s_ek >= 0 but does not state how they are chosen.  Used here:
    two directed edges i != j are COMPARABLE if they share a junction;
      s_ij = 1.0  if they carry the same street name (incl. the opposite direction)
      s_ij = 0.5  else if they have the same road class (highway_class_used)
      otherwise no pair (s = 0)
The team must approve (or replace) this rule.
"""
from __future__ import annotations

import itertools

import numpy as np
import pandas as pd
import scipy.sparse as sp
from scipy.sparse import csgraph
from scipy.sparse.linalg import spsolve

EPS_RIDGE = 1e-6

# ------------------------------------------------------------------ similarity
def build_similarity(edges: pd.DataFrame, edge_mapping_path: str = "edge_mapping.csv") -> pd.DataFrame:
    """Return DataFrame(edge_i, edge_j, s) with i < j (edge_index values)."""
    em = pd.read_csv(edge_mapping_path, encoding="utf-8-sig")[
        ["u", "v", "key", "name", "highway_class_used"]]
    e = edges[["edge_index", "u", "v", "key"]].merge(em, on=["u", "v", "key"], how="left")
    by_node: dict = {}
    for r in e.itertuples():
        for n in (r.u, r.v):
            by_node.setdefault(n, set()).add(r.edge_index)
    pairs = set()
    for members in by_node.values():
        for a, b in itertools.combinations(sorted(members), 2):
            pairs.add((a, b))
    name = e["name"].to_dict()
    cls = e["highway_class_used"].to_dict()
    rows = []
    for a, b in sorted(pairs):
        same_name = pd.notna(name[a]) and name[a] == name[b]
        if same_name:
            s = 1.0
        elif pd.notna(cls[a]) and cls[a] == cls[b]:
            s = 0.5
        else:
            continue
        rows.append((a, b, s))
    return pd.DataFrame(rows, columns=["edge_i", "edge_j", "s"])


def laplacian(sim: pd.DataFrame, n: int) -> sp.csr_matrix:
    i, j, s = sim["edge_i"].to_numpy(), sim["edge_j"].to_numpy(), sim["s"].to_numpy()
    W = sp.coo_matrix((np.r_[s, s], (np.r_[i, j], np.r_[j, i])), shape=(n, n)).tocsr()
    return sp.diags(np.asarray(W.sum(axis=1)).ravel()) - W


def temporal_laplacian(T: int) -> sp.csr_matrix:
    if T == 1:
        return sp.csr_matrix((1, 1))
    main = np.r_[1, 2 * np.ones(T - 2), 1] if T > 2 else np.array([1.0, 1.0])
    return sp.diags([main, -np.ones(T - 1), -np.ones(T - 1)], [0, 1, -1]).tocsr()


# ------------------------------------------------------------------ estimator
class ModelB:
    """Fit with  .fit(y, w, t0, alpha, beta, gamma)  then read  .estimate_  (N x T, seconds)."""

    def __init__(self, sim: pd.DataFrame, n_edges: int, n_bins: int):
        self.N, self.T = n_edges, n_bins
        self.sim = sim
        self.Ls = laplacian(sim, n_edges)               # L^E
        self.Lt = temporal_laplacian(n_bins)            # L_T
        self.temporal = sp.kron(sp.identity(n_edges), self.Lt, format="csr")
        self._spatial_cache = None
        n_comp, self.component_ = csgraph.connected_components(
            sp.coo_matrix((np.ones(len(sim)), (sim["edge_i"], sim["edge_j"])), shape=(n_edges, n_edges)),
            directed=False)
        self.n_components_ = n_comp

    def _spatial(self, t0: np.ndarray):
        if self._spatial_cache is None or not np.array_equal(self._spatial_cache[0], t0):
            D = sp.diags(1.0 / t0)
            self._spatial_cache = (t0.copy(), sp.kron(D @ self.Ls @ D, sp.identity(self.T), format="csr"))
        return self._spatial_cache[1]

    def fit(self, y: np.ndarray, w: np.ndarray, t0: np.ndarray, alpha: float, beta: float, gamma: float):
        """y: (N,T) seconds (NaN/any value where missing), w: (N,T) 0/1, t0: (N,) free-flow seconds."""
        if w.sum() == 0:
            raise ValueError("no observations to fit")
        T0 = np.repeat(t0[:, None], self.T, axis=1)
        yz = np.where(w > 0, np.nan_to_num(y), 0.0)
        z0 = float(((yz / T0) * w).sum() / w.sum())          # global mean congestion ratio
        wv = w.ravel()
        H = (alpha * sp.diags(wv) + beta * self._spatial(t0) + gamma * self.temporal
             + EPS_RIDGE * sp.identity(self.N * self.T)).tocsc()
        b = alpha * wv * yz.ravel() + EPS_RIDGE * (z0 * T0).ravel()
        mu = np.asarray(spsolve(H, b)).reshape(self.N, self.T)
        if not (np.isfinite(mu).all() and (mu > 0).all()):
            raise FloatingPointError("solution not finite and positive")
        anchored = np.zeros(self.n_components_, bool)
        np.logical_or.at(anchored, self.component_, w.sum(axis=1) > 0)
        self.unanchored_edges_ = int((~anchored[self.component_]).sum())
        self.estimate_ = mu
        self.ratio_ = mu / T0
        self.z_global_ = z0
        return self


# ------------------------------------------------------------------ metrics
def metrics(truth: np.ndarray, pred: np.ndarray) -> dict:
    truth, pred = np.asarray(truth, float), np.asarray(pred, float)
    n = truth.size
    if n == 0:
        return dict(n=0, MAE=np.nan, RMSE=np.nan, MAPE=np.nan)
    err = pred - truth
    ok = truth > 0
    return dict(n=n, MAE=float(np.abs(err).mean()), RMSE=float(np.sqrt((err ** 2).mean())),
                MAPE=float(np.abs(err[ok] / truth[ok]).mean() * 100) if ok.any() else np.nan)


def baselines(y, w_train, ref, mask_eval):
    """Two naive predictors evaluated on the cells in mask_eval (bool N x T)."""
    R = np.repeat(ref[:, None], y.shape[1], axis=1)
    z0 = float((np.where(w_train > 0, y / R, 0) * w_train).sum() / max(w_train.sum(), 1))
    return {"free_flow": R[mask_eval], "mean_ratio": (z0 * R)[mask_eval]}


# ------------------------------------------------------------------ tuning
GRID_ALPHA = [1.0, 10.0]
GRID_BETA = [0.01, 0.1, 1.0, 10.0, 100.0]     # beta has units s^2 (spatial term is dimensionless)
GRID_GAMMA = [0.01, 0.1, 1.0, 10.0]


def grid():
    return list(itertools.product(GRID_ALPHA, GRID_BETA, GRID_GAMMA))


def pick_best(results: pd.DataFrame, by="val_MAE") -> pd.Series:
    """results has columns alpha,beta,gamma,val_MAE,val_RMSE (already averaged over repeats).
    Best = lowest validation MAE, ties broken by validation RMSE.  Training error is never used."""
    return results.sort_values([by, "val_RMSE"]).iloc[0]