"""
model_b.py - RouteX Model B: batch travel-time estimator (final three-term objective,
as in "Model B final" documentation).

  J = alpha * sum_{(e,t) in Omega} w[e,t] (mu_e(t) - y_e(t))^2
    + beta  * sum_t sum_{(e,k) in S} s[e,k] (mu_e(t)/t0_e - mu_k(t)/t0_k)^2
    + gamma * sum_e sum_{t=2..T} (mu_e(t) - mu_e(t-1))^2

Linear system  H mu = alpha * W_Omega * y   with
  H = alpha*W_Omega + beta*(I_T (x) D0^-1 L^E D0^-1) + gamma*(L_T (x) I_m)
  L^E = D_S - S_E (plain weighted graph Laplacian), D0 = diag(t0), L_T = path-graph Laplacian.

BATCH spatiotemporal estimator: all time bins are solved jointly; not real-time/causal.
Unknown vector is time bin first, then edge: x[t*m + e] = mu_e(t) (seconds).
Edge order = Model A edge_index_0based (= edge_id), never renumbered.
Missing observations carry weight 0 (they are NOT zero-second travel times).

Nothing here is measured Dokki traffic. Defaults (similarity rule, alpha/beta/gamma, tuning
grids) are ASSUMPTIONS to be validated; scenarios from make_synthetic_scenario are
SYNTHETIC and for testing only. Time-bin definitions are NOT set in this file: load them
with load_time_bins() from a file whose status is verified or assumed.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np
import pandas as pd
import scipy.sparse as sp
from scipy.sparse.csgraph import connected_components
from scipy.sparse.linalg import splu, onenormest, LinearOperator

M_EDGES = 546
T_BINS = 7


class SingularSystemError(ValueError):
    """The input cannot produce a unique solution."""


# --------------------------------------------------------------------------- #
# Model A inputs
# --------------------------------------------------------------------------- #
def load_edges(xlsx_path: str, sheet: str = "Edge index") -> pd.DataFrame:
    """Read Model A's 'Edge index' sheet; edge_id = edge_index_0based (unchanged).

    Returns a DataFrame sorted by edge_id with columns:
    edge_id, edge_label (Model A label such as 'e0000'), u_node_id, v_node_id, road_name, road_type, length_m, lanes,
    reference_speed_kph, free_flow_s.
    """
    raw = pd.read_excel(xlsx_path, sheet_name=sheet)
    df = pd.DataFrame({
        "edge_id": raw["edge_index_0based"].astype(int),
        "edge_label": raw["matrix_edge_label"].astype(str),
        "u_node_id": raw["u_node_id"].astype(int),
        "v_node_id": raw["v_node_id"].astype(int),
        "road_name": raw["road_name"].astype(str),
        "road_type": raw["road_type"].astype(str),
        "length_m": raw["length_m"].astype(float),
        "lanes": raw["lanes_numeric"].astype(float),
        "reference_speed_kph": raw["reference_speed_kph"].astype(float),
        "free_flow_s": raw["free_flow_s_precise"].astype(float),
    }).sort_values("edge_id").reset_index(drop=True)
    if not np.array_equal(df["edge_id"].to_numpy(), np.arange(len(df))):
        raise ValueError("edge_id must be 0..m-1 in Model A order")
    return df


def load_similarity(csv_path: str, m: int = M_EDGES) -> sp.csr_matrix:
    """Read the team's edge-similarity weights s[e,k] (Model A / Model B definition).

    CSV columns: edge_i, edge_j, s  -- each unordered neighbouring pair listed ONCE
    (set S in the documentation). Returns the symmetric matrix S_E (zero diagonal).
    This module does not invent similarity weights: the file must come from the team.
    """
    d = pd.read_csv(csv_path)
    if not {"edge_i", "edge_j", "s"}.issubset(d.columns):
        raise ValueError("similarity CSV needs columns edge_i, edge_j, s")
    i, j, s = d["edge_i"].to_numpy(int), d["edge_j"].to_numpy(int), d["s"].to_numpy(float)
    if ((i < 0) | (i >= m) | (j < 0) | (j >= m)).any():
        raise ValueError("edge index out of range")
    if (i == j).any():
        raise ValueError("self-pairs are not allowed")
    if not np.all(np.isfinite(s)) or (s < 0).any():
        raise ValueError("similarity weights must be finite and >= 0")
    key = np.minimum(i, j) * m + np.maximum(i, j)
    if len(np.unique(key)) != len(key):
        raise ValueError("a neighbouring pair is listed more than once (each pair once)")
    S = sp.csr_matrix((s, (i, j)), shape=(m, m))
    return (S + S.T).tocsr()


def build_edge_similarity(edges: pd.DataFrame, lane_sigma: float = 1.0,
                          speed_sigma_kph: float = 15.0,
                          other_name_factor: float = 0.5) -> sp.csr_matrix:
    """ASSUMED edge similarity S_E (m x m, symmetric, zero diagonal).

    Two edges are neighbours if they share a node (either endpoint, either direction).
    Weight = exp(-d^2) * (1 if same road_name else other_name_factor),
    d^2 = (dLanes/lane_sigma)^2 + (dSpeed/speed_sigma_kph)^2. Each pair counted once in S.
    If the team defines its own s[e,k], use load_similarity() instead.
    """
    m = len(edges)
    by_node: dict[int, list[int]] = {}
    for e, (u, v) in enumerate(zip(edges["u_node_id"], edges["v_node_id"])):
        by_node.setdefault(int(u), []).append(e)
        by_node.setdefault(int(v), []).append(e)
    lanes = edges["lanes"].to_numpy()
    spd = edges["reference_speed_kph"].to_numpy()
    names = edges["road_name"].to_numpy()
    rows, cols, vals = [], [], []
    seen = set()
    for es in by_node.values():
        for i in es:
            for k in es:
                if i < k and (i, k) not in seen:
                    seen.add((i, k))
                    d2 = ((lanes[i] - lanes[k]) / lane_sigma) ** 2 + \
                         ((spd[i] - spd[k]) / speed_sigma_kph) ** 2
                    s = np.exp(-d2) * (1.0 if names[i] == names[k] else other_name_factor)
                    rows += [i, k]; cols += [k, i]; vals += [s, s]
    return sp.csr_matrix((vals, (rows, cols)), shape=(m, m))


def edge_laplacian(S: sp.spmatrix) -> sp.csr_matrix:
    """L^E = D_S - S_E (weighted graph Laplacian, symmetric, rows sum to 0)."""
    S = sp.csr_matrix(S)
    return (sp.diags(np.asarray(S.sum(axis=1)).ravel()) - S).tocsr()


def path_laplacian(T: int) -> sp.csr_matrix:
    """L_T: path-graph Laplacian over consecutive time bins (T x T)."""
    D = sp.diags([-np.ones(T - 1), np.ones(T - 1)], [0, 1], shape=(T - 1, T))
    return (D.T @ D).tocsr()


def load_time_bins(csv_path: str, T: int | None = None) -> pd.DataFrame:
    """Time-bin definition table (single source of truth for all models/files).

    Columns: time_bin (0..T-1), label, start_hhmm, end_hhmm, day_type, source, status.
    status must be 'verified' or 'assumed'. Bins must be ordered and non-overlapping.
    """
    d = pd.read_csv(csv_path)
    need = ["time_bin", "label", "start_hhmm", "end_hhmm", "day_type", "source", "status"]
    miss = [c for c in need if c not in d.columns]
    if miss:
        raise ValueError(f"time-bin file missing columns {miss}")
    T = len(d) if T is None else T
    if len(d) != T or not np.array_equal(d["time_bin"].to_numpy(), np.arange(T)):
        raise ValueError(f"need exactly {T} rows with time_bin = 0..{T-1} in order")
    if not d["status"].isin(["verified", "assumed"]).all():
        raise ValueError("every bin needs status 'verified' or 'assumed' (not blank/undefined)")
    if d[["start_hhmm", "end_hhmm"]].isna().any().any():
        raise ValueError("start_hhmm/end_hhmm must be filled in")
    return d


# --------------------------------------------------------------------------- #
# Observation I/O  (agree this format with Lojy)
# --------------------------------------------------------------------------- #
def load_observations(csv_path: str, m: int = M_EDGES, T: int = T_BINS):
    """Long CSV: edge_id, time_bin, observed_time_s, weight  ->  y (m,T) with NaN, w (m,T).

    time_bin is 0-based (0..T-1). Missing rows or NaN values get weight 0.
    """
    d = pd.read_csv(csv_path)
    need = {"edge_id", "time_bin", "observed_time_s", "weight"}
    if not need.issubset(d.columns):
        raise ValueError(f"observations CSV needs columns {sorted(need)}")
    if d.duplicated(["edge_id", "time_bin"]).any():
        raise ValueError("duplicate (edge_id, time_bin) rows")
    if not d["edge_id"].between(0, m - 1).all() or not d["time_bin"].between(0, T - 1).all():
        raise ValueError("edge_id or time_bin out of range")
    y = np.full((m, T), np.nan)
    w = np.zeros((m, T))
    y[d["edge_id"].astype(int), d["time_bin"].astype(int)] = d["observed_time_s"].to_numpy(float)
    w[d["edge_id"].astype(int), d["time_bin"].astype(int)] = d["weight"].to_numpy(float)
    return y, w


# --------------------------------------------------------------------------- #
# Core estimator
# --------------------------------------------------------------------------- #
def _check_inputs(LE, t0, y, w, alpha, beta, gamma):
    m, T = y.shape
    if LE.shape != (m, m):
        raise ValueError(f"L^E must be {m}x{m}, got {LE.shape}")
    if t0.shape != (m,) or not np.all(np.isfinite(t0)) or np.any(t0 <= 0):
        raise ValueError("t0 must be a positive finite vector of length m")
    if w.shape != y.shape:
        raise ValueError("w and y must have the same shape (m, T)")
    if not (alpha > 0 and beta >= 0 and gamma >= 0):
        raise ValueError("need alpha > 0, beta >= 0, gamma >= 0")
    if np.any(w < 0) or not np.all(np.isfinite(w)):
        raise ValueError("weights must be finite and >= 0")
    if ((w > 0) & ~np.isfinite(y)).any():
        raise ValueError("positive weight on a missing/non-finite observation")
    if np.any(y[w > 0] < 0):
        raise ValueError("negative observed travel time")


def check_identifiable(S: sp.spmatrix, w: np.ndarray, beta: float, gamma: float) -> None:
    """Raise SingularSystemError when H would be singular (no unique solution).

    The free directions of the regulariser are: constant-in-time (gamma term) and
    constant-per-connected-component in c = mu/t0 (beta term). Anchoring needed:
      beta>0, gamma>0 : >=1 observation per similarity-graph component (any bin)
      beta>0, gamma=0 : >=1 observation per (component, bin)
      beta=0, gamma>0 : >=1 observation per edge (any bin)
      beta=0, gamma=0 : every (edge, bin) observed
    """
    n, lab = connected_components(sp.csr_matrix(S) > 0, directed=False)
    m, T = w.shape
    obs = w > 0
    def fail(msg):
        raise SingularSystemError(msg + "; the solution is not unique.")
    if beta > 0 and gamma > 0:
        have = np.bincount(lab, weights=obs.sum(axis=1), minlength=n)
        bad = np.where(have == 0)[0]
        if len(bad):
            ex = np.where(np.isin(lab, bad))[0][:10].tolist()
            fail(f"{len(bad)} similarity-graph component(s) have no observation in any bin (e.g. edge_ids {ex})")
    elif beta > 0:
        for t in range(T):
            if np.any(np.bincount(lab, weights=obs[:, t].astype(float), minlength=n) == 0):
                fail(f"time bin {t}: a similarity-graph component has no observation")
    elif gamma > 0:
        bad = np.where(~obs.any(axis=1))[0]
        if len(bad):
            fail(f"{len(bad)} edge(s) never observed (e.g. edge_ids {bad[:10].tolist()})")
    elif not obs.all():
        fail("beta = gamma = 0 requires every (edge, bin) to be observed")


def build_system(LE, t0, y, w, alpha, beta, gamma):
    """Return sparse H (mT x mT) and rhs = alpha * W * y.  Bin-major ordering."""
    m, T = y.shape
    D0inv = sp.diags(1.0 / t0)
    Ls = (D0inv @ LE @ D0inv).tocsr()                 # D0^-1 L^E D0^-1
    W = sp.diags(w.T.ravel())                          # W_Omega
    yv = np.where(w > 0, y, 0.0).T.ravel()             # placeholder only; its weight is 0
    H = (alpha * W + beta * sp.kron(sp.eye(T), Ls)
         + gamma * sp.kron(path_laplacian(T), sp.eye(m))).tocsc()
    return H, alpha * (W @ yv)


@dataclass
class Result:
    mu_hat_s: np.ndarray        # (m, T) seconds
    cond1_est: float            # estimated 1-norm condition number of H
    rel_residual: float         # ||H x - rhs|| / ||rhs||
    n_obs: int


def estimate(LE, t0, y, w, alpha=1.0, beta=100.0, gamma=1.0, S=None,
             max_cond=1e12, max_residual=1e-6) -> Result:
    """Solve H mu = alpha W y. y: (m,T) NaN allowed; w: (m,T) >= 0; t0: (m,) seconds.

    Default alpha=1, beta=100, gamma=1 are TENTATIVE (tune with tune_hyperparameters).
    S (similarity matrix) is used for the identifiability check; it defaults to the
    off-diagonal pattern of L^E. max_cond / max_residual are numerical tolerances.
    Raises SingularSystemError rather than altering the objective when no unique
    solution exists.
    """
    LE = sp.csr_matrix(LE)
    t0 = np.asarray(t0, float)
    _check_inputs(LE, t0, y, w, alpha, beta, gamma)
    pattern = S if S is not None else (abs(LE) - sp.diags(abs(LE).diagonal()))
    check_identifiable(pattern, w, beta, gamma)
    H, rhs = build_system(LE, t0, y, w, alpha, beta, gamma)
    if abs(H - H.T).max() > 1e-9 * abs(H).max():
        raise SingularSystemError("H is not symmetric")
    try:
        lu = splu(H)
    except RuntimeError as exc:
        raise SingularSystemError(f"factorisation failed (singular): {exc}") from exc
    x = lu.solve(rhs)
    if not np.all(np.isfinite(x)):
        raise SingularSystemError("non-finite values in solution")
    res = np.linalg.norm(H @ x - rhs) / max(np.linalg.norm(rhs), 1e-300)
    inv = LinearOperator(H.shape, matvec=lu.solve, rmatvec=lu.solve, dtype=float)
    cond = float(onenormest(H) * onenormest(inv))
    if cond > max_cond or res > max_residual:
        raise SingularSystemError(f"ill-conditioned or inaccurate: cond~{cond:.2e}, residual {res:.2e}")
    m, T = y.shape
    return Result(x.reshape(T, m).T, cond, float(res), int((w > 0).sum()))


def tune_hyperparameters(LE, t0, y, w, grid_alpha=(1.0,), grid_beta=(10.0, 100.0, 1000.0),
                         grid_gamma=(0.1, 1.0, 10.0), val_frac=0.2, seed=0, S=None):
    """Pick (alpha, beta, gamma) by hiding val_frac of the OBSERVED entries.

    Returns (best_params, table) where table lists validation MAE/RMSE per setting.
    """
    rng = np.random.default_rng(seed)
    obs = np.argwhere(w > 0)
    hide = obs[rng.random(len(obs)) < val_frac]
    w_tr = w.copy()
    w_tr[hide[:, 0], hide[:, 1]] = 0.0
    rows = []
    for a in grid_alpha:
        for b_ in grid_beta:
            for g in grid_gamma:
                try:
                    r = estimate(LE, t0, y, w_tr, a, b_, g, S=S)
                except SingularSystemError:
                    continue
                err = r.mu_hat_s[hide[:, 0], hide[:, 1]] - y[hide[:, 0], hide[:, 1]]
                rows.append(dict(alpha=a, beta=b_, gamma=g,
                                 mae=float(np.mean(np.abs(err))),
                                 rmse=float(np.sqrt(np.mean(err ** 2)))))
    tab = pd.DataFrame(rows).sort_values("rmse").reset_index(drop=True)
    if tab.empty:
        raise SingularSystemError("no hyper-parameter setting produced a valid solution")
    return tab.iloc[0][["alpha", "beta", "gamma"]].to_dict(), tab


def to_long(mu: np.ndarray) -> pd.DataFrame:
    """(m,T) -> long table edge_id, time_bin, mu_hat_s."""
    m, T = mu.shape
    return pd.DataFrame({"edge_id": np.repeat(np.arange(m), T),
                         "time_bin": np.tile(np.arange(T), m),
                         "mu_hat_s": mu.ravel()})


def write_output(mu: np.ndarray, path: str, time_bins: pd.DataFrame | None = None,
                 edges: pd.DataFrame | None = None) -> None:
    """Model_B_Output.csv: edge_id, time_bin, mu_hat_s (+ Model A label / time-bin label)."""
    if mu.ndim != 2 or mu.shape[0] != M_EDGES:
        raise ValueError(f"expected ({M_EDGES}, T), got {mu.shape}")
    if time_bins is not None and len(time_bins) != mu.shape[1]:
        raise ValueError("time_bins rows must equal the number of bins in mu")
    if not np.all(np.isfinite(mu)) or np.any(mu <= 0):
        raise ValueError("non-finite or non-positive travel time in output")
    out = to_long(mu)
    if edges is not None:
        out.insert(1, "edge_label", edges["edge_label"].to_numpy()[out["edge_id"].to_numpy()])
    if time_bins is not None:
        out = out.merge(time_bins[["time_bin", "label", "status"]]
                        .rename(columns={"label": "time_bin_label", "status": "time_bin_status"}),
                        on="time_bin", how="left")
    out.to_csv(path, index=False)


def load_tomtom_observations(edge_bins_csv: str, edges: pd.DataFrame,
                             weight_mode: str = "observed_fraction"):
    """Read the TomTom 'Model_B_edge_bins' export (one row per edge x time set).

    Uses the file's own observation_mask (1 = usable observation) and matched_estimate_s
    as y. Returns y (m,T) with NaN where unobserved, w (m,T) and the time-bin table
    (bins ordered by window start; time_bin 0..T-1).

    weight_mode: 'observed_fraction' (share of the edge covered by TomTom data) or 'unit' (1.0).
    The export has no raw sample counts, so no sample-size weighting is applied here.
    """
    d = pd.read_csv(edge_bins_csv)
    need = {"edge_id", "time_set_id", "cairo_time_bin", "observation_mask",
            "matched_estimate_s", "observed_fraction", "model_a_reference_time_s"}
    if not need.issubset(d.columns):
        raise ValueError(f"edge-bins CSV needs columns {sorted(need)}")
    if weight_mode not in ("observed_fraction", "unit"):
        raise ValueError("weight_mode must be 'observed_fraction' or 'unit'")
    lab2idx = dict(zip(edges["edge_label"], edges["edge_id"]))
    if not set(d["edge_id"]).issubset(lab2idx):
        raise ValueError("edge labels in the export are not in Model A")
    # consistency with Model A free-flow time
    t0 = edges.set_index("edge_label")["free_flow_s"]
    if not np.allclose(d["model_a_reference_time_s"], t0.reindex(d["edge_id"]).to_numpy(), rtol=1e-6):
        raise ValueError("model_a_reference_time_s does not match Model A free_flow_s")
    win = (d[["time_set_id", "cairo_time_bin"]].drop_duplicates()
           .assign(start=lambda x: x["cairo_time_bin"].str.split("-").str[0],
                   end=lambda x: x["cairo_time_bin"].str.split("-").str[1]))
    win["_k"] = win["start"].map(lambda t: int(t.split(":")[0]) * 60 + int(t.split(":")[1]))
    win = win.sort_values("_k").reset_index(drop=True)
    win["time_bin"] = np.arange(len(win))
    set2bin = dict(zip(win["time_set_id"], win["time_bin"]))
    T = len(win)
    y = np.full((len(edges), T), np.nan)
    w = np.zeros((len(edges), T))
    for r in d[d["observation_mask"] == 1].itertuples():
        e, t = lab2idx[r.edge_id], set2bin[r.time_set_id]
        y[e, t] = r.matched_estimate_s
        w[e, t] = r.observed_fraction if weight_mode == "observed_fraction" else 1.0
    tb = pd.DataFrame({
        "time_bin": win["time_bin"], "label": win["cairo_time_bin"],
        "start_hhmm": win["start"].map(lambda t: f"{int(t.split(':')[0]):02d}:{t.split(':')[1]}"),
        "end_hhmm": win["end"].map(lambda t: f"{int(t.split(':')[0]):02d}:{t.split(':')[1]}"),
        "day_type": "not stated in export", "source": "TomTom MOVE export (Model_B_edge_bins)",
        "status": "verified"})
    hm = lambda t: int(t.split(":")[0]) * 3600 + int(t.split(":")[1]) * 60
    tb["start_s_since_midnight"] = win["start"].map(hm).to_numpy()
    tb["end_s_since_midnight"] = win["end"].map(hm).to_numpy()
    return y, w, tb


# --------------------------------------------------------------------------- #
# SYNTHETIC test scenario (assumed, not measured) - for tests only
# --------------------------------------------------------------------------- #
def make_synthetic_scenario(edges: pd.DataFrame, T: int = T_BINS, seed: int = 0,
                            obs_fraction: float = 0.3, noise_sd_frac: float = 0.05):
    """SYNTHETIC ground truth + sparse noisy observations. Every value is assumed.

    truth = free_flow * (1 + bin_level[t] * street_factor[street] * road_type_factor).
    bin_level is random per bin index and NOT tied to any clock time.
    """
    rng = np.random.default_rng(seed)
    t0 = edges["free_flow_s"].to_numpy()
    m = len(t0)
    bin_level = rng.uniform(0.1, 0.9, T)
    streets = edges["road_name"].astype("category").cat.codes.to_numpy()
    sf = rng.uniform(0.3, 1.5, streets.max() + 1)[streets]
    tf = edges["road_type"].map(lambda s: 1.3 if "primary" in s else 1.0).to_numpy()
    truth = t0[:, None] * (1 + sf[:, None] * tf[:, None] * bin_level[None, :])
    mask = rng.random((m, T)) < obs_fraction
    y = np.where(mask, truth * (1 + noise_sd_frac * rng.standard_normal((m, T))), np.nan)
    y = np.where(np.isnan(y), np.nan, np.maximum(y, 1e-3))
    return truth, y, mask.astype(float), mask


# --------------------------------------------------------------------------- #
# Evaluation helper
# --------------------------------------------------------------------------- #
def metrics(pred, truth, mask):
    e = (pred - truth)[mask]
    return dict(MAE=float(np.mean(np.abs(e))), RMSE=float(np.sqrt(np.mean(e ** 2))))


# --------------------------------------------------------------------------- #
# Self-tests
# --------------------------------------------------------------------------- #
def run_self_tests(xlsx_path: str | None = None) -> None:
    # 1. tiny example vs a dense solve of the documented system
    S = sp.csr_matrix(np.array([[0, 1, 0], [1, 0, 1], [0, 1, 0]], float))
    LE = edge_laplacian(S)
    t0 = np.array([10., 20., 5.])
    y = np.array([[10, np.nan], [np.nan, 24], [6, 7.]])
    w = np.where(np.isfinite(y), 1.0, 0.0)
    r = estimate(LE, t0, y, w, 1.0, 50.0, 1.0, S=S)
    H, b = build_system(LE, t0, y, w, 1.0, 50.0, 1.0)
    assert np.allclose(r.mu_hat_s.T.ravel(), np.linalg.solve(H.toarray(), b)), "dense mismatch"
    # 2. objective gradient is zero at the solution (checks matrix form == scalar form)
    def J(mu):
        d = np.where(w > 0, mu - np.nan_to_num(y), 0)
        sp_ = sum(S[i, k] * (mu[i, t] / t0[i] - mu[k, t] / t0[k]) ** 2
                  for t in range(2) for i in range(3) for k in range(i + 1, 3) if S[i, k])
        tm = ((mu[:, 1:] - mu[:, :-1]) ** 2).sum()
        return 1.0 * (w * d ** 2).sum() + 50.0 * sp_ + 1.0 * tm
    base = J(r.mu_hat_s)
    rng = np.random.default_rng(1)
    for _ in range(20):
        assert J(r.mu_hat_s + 1e-3 * rng.standard_normal((3, 2))) >= base - 1e-12, "not a minimiser"
    # 3. missing is not zero
    assert r.mu_hat_s[0, 1] > 5, "missing entry treated as zero"
    # 4. singular case: component with no observation
    S2 = sp.block_diag([S, S]).tocsr()
    y2 = np.full((6, 2), np.nan); y2[0, 0] = 10
    try:
        estimate(edge_laplacian(S2), np.r_[t0, t0], y2, np.isfinite(y2).astype(float), S=S2)
        raise AssertionError("singular case not detected")
    except SingularSystemError:
        pass
    # 5. data dominates -> observations recovered
    yy = np.array([[5., 6.], [7., 8.], [9., 10.]])
    rr = estimate(LE, t0, yy, np.ones_like(yy), alpha=1e6, beta=1e-3, gamma=1e-3, S=S)
    assert np.allclose(rr.mu_hat_s, yy, atol=1e-2)
    # 6. similarity-file validation
    import io, tempfile, os
    with tempfile.TemporaryDirectory() as td:
        good = os.path.join(td, "s.csv"); open(good, "w").write("edge_i,edge_j,s\n0,1,0.5\n1,2,0.25\n")
        Sg = load_similarity(good, m=3)
        assert Sg[0, 1] == Sg[1, 0] == 0.5 and Sg.diagonal().sum() == 0
        dup = os.path.join(td, "d.csv"); open(dup, "w").write("edge_i,edge_j,s\n0,1,0.5\n1,0,0.5\n")
        try:
            load_similarity(dup, m=3); raise AssertionError("duplicate pair not rejected")
        except ValueError:
            pass
    # 7. full Dokki graph on SYNTHETIC data
    if xlsx_path:
        edges = load_edges(xlsx_path)
        assert len(edges) == M_EDGES
        Sm = build_edge_similarity(edges)
        Lm = edge_laplacian(Sm)
        t0m = edges["free_flow_s"].to_numpy()
        assert np.allclose(t0m, 3.6 * edges["length_m"] / edges["reference_speed_kph"], rtol=1e-6)
        truth, yo, wo, mask = make_synthetic_scenario(edges)
        res = estimate(Lm, t0m, yo, wo, S=Sm)
        assert res.mu_hat_s.shape == (M_EDGES, T_BINS)
        assert np.all(np.isfinite(res.mu_hat_s)) and np.all(res.mu_hat_s > 0)
        H, _ = build_system(Lm, t0m, yo, wo, 1, 100, 1)
        assert H.shape == (M_EDGES * T_BINS, M_EDGES * T_BINS)
    print("model_b self-tests passed")


def main(argv=None):
    p = argparse.ArgumentParser(description="RouteX Model B estimator")
    p.add_argument("--model-a", help="RouteX_Model_A_Matrices.xlsx")
    p.add_argument("--observations", help="CSV: edge_id,time_bin,observed_time_s,weight")
    p.add_argument("--similarity", help="optional CSV edge_i,edge_j,s (each pair once); default: assumed rule")
    p.add_argument("--time-bins", help="time-bin definition CSV (see load_time_bins)")
    p.add_argument("--out", default="Model_B_Output.csv")
    p.add_argument("--alpha", type=float, default=1.0)
    p.add_argument("--beta", type=float, default=100.0)
    p.add_argument("--gamma", type=float, default=1.0)
    p.add_argument("--tune", action="store_true", help="choose alpha/beta/gamma by held-out validation")
    p.add_argument("--selftest", action="store_true")
    a = p.parse_args(argv)
    if a.selftest or not a.observations:
        run_self_tests(a.model_a)
        return 0
    edges = load_edges(a.model_a)
    S = load_similarity(a.similarity) if a.similarity else build_edge_similarity(edges)
    L = edge_laplacian(S)
    t0 = edges["free_flow_s"].to_numpy()
    y, w = load_observations(a.observations)
    tb = load_time_bins(a.time_bins) if a.time_bins else None
    alpha, beta, gamma = a.alpha, a.beta, a.gamma
    if a.tune:
        best, _ = tune_hyperparameters(L, t0, y, w, S=S)
        alpha, beta, gamma = best["alpha"], best["beta"], best["gamma"]
        print(f"tuned: alpha={alpha}, beta={beta}, gamma={gamma}")
    res = estimate(L, t0, y, w, alpha, beta, gamma, S=S)
    write_output(res.mu_hat_s, a.out, tb)
    print(f"wrote {a.out}; obs={res.n_obs}; cond~{res.cond1_est:.2e}; residual={res.rel_residual:.1e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
