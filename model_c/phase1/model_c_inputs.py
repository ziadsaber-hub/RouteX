"""ROUTEX - Model C input adapters: Model B travel times and Model D signal delays.

Why this file exists
  task1_dijkstra.load_model_b() only accepts a CSV with the exact columns  u, v, key, mu_hat_s [, interval].
  If Model B's final file looks different (edge id as text, edge index, minutes instead of seconds, several rows
  per edge, empty cells ...) use load_model_b() here. It converts, checks, and REPORTS what it did.

Model B  (travel times, seconds)
  from model_c_inputs import install_model_b
  report = install_model_b("model_b_final.csv", interval="08:00")      # sets E[eid]["mu"] for Model C
  print(report)

  Accepted edge identification (first one found is used):
    1) columns u, v, key                       (also u_node_id, v_node_id)
    2) column edge_id = "u-v-k"                (also eid, edge)
    3) column edge_index_0based                (row of Edge_Index.csv)
  Accepted time columns (first found): mu_hat_s, mu_hat, travel_time_s, travel_time, time_s, predicted_time_s, tt_s, mu
  Optional: interval column (use interval=...), unit="s" | "min" | "ms" | "h".

Missing values - how they are treated (documented rule)
  * An edge that is NOT in Model B's file keeps its Model A free-flow time (counted in MISSING_B, listed in the report).
  * A row whose time is empty / NaN / <= 0 / infinite is treated exactly like a missing edge (dropped, listed).
    (task1_dijkstra.set_travel_times would otherwise STOP with an error on such a value.)
  * strict=True turns both cases into an error instead (use it for the final run: no silent fallbacks).
  * An edge ID that is not in Model A, or two different times for the same edge in the same interval, is always an error
    (aggregate="mean"/"last" settles duplicates on request).

Model D  (signal delays, seconds)
  from model_c_inputs import load_signal_deltas
  deltas = load_signal_deltas("model_d_signal_deltas.csv")   # columns: approach_edge_id ("u-v-k"), delta_s
  live = Live(); live.set_signal_deltas(deltas)
  delta_s = T_priority - T_normal for the approach edge arriving at a verified signal; it may be negative.
  make_signal_template() writes the list of approach edges Model D has to fill in (verified signals only).
"""
import math
import os

import pandas as pd

import task1_dijkstra as T1
from task1_dijkstra import E

_HERE = os.path.dirname(os.path.abspath(__file__))
_UNIT = {"s": 1.0, "ms": 0.001, "min": 60.0, "h": 3600.0}
_TIME_COLS = ["mu_hat_s", "mu_hat", "travel_time_s", "travel_time", "time_s", "predicted_time_s", "tt_s", "mu"]


def _read(path):
    if str(path).lower().endswith((".xlsx", ".xlsm", ".xls")):
        return pd.read_excel(path)
    return pd.read_csv(path, encoding="utf-8-sig")


def _edge_ids(df):
    """Return a list of (u, v, k) tuples, one per row, whichever identification the file uses."""
    cols = {c.lower().strip(): c for c in df.columns}
    for cu, cv in (("u", "v"), ("u_node_id", "v_node_id")):
        if cu in cols and cv in cols and "key" in cols:
            return list(zip(df[cols[cu]].astype(int), df[cols[cv]].astype(int), df[cols["key"]].astype(int)))
    for c in ("edge_id", "eid", "edge"):
        if c in cols:
            out = []
            for x in df[cols[c]].astype(str):
                u, v, k = x.strip().strip("()").replace(",", "-").replace(" ", "").split("-")
                out.append((int(u), int(v), int(k)))
            return out
    if "edge_index_0based" in cols:
        idx = pd.read_csv(os.path.join(_HERE, "Edge_Index.csv"))
        look = {int(r.edge_index_0based): (int(r.u_node_id), int(r.v_node_id), int(r.key)) for r in idx.itertuples()}
        return [look[int(i)] for i in df[cols["edge_index_0based"]]]
    raise ValueError("cannot find the edge identification: need u,v,key  or  edge_id ('u-v-k')  or  edge_index_0based")


def load_model_b(path, interval=None, unit="s", aggregate=None, strict=False):
    """Read a Model B file -> ({(u,v,k): seconds}, report dict). Nothing is installed yet."""
    if unit not in _UNIT:
        raise ValueError(f"unit must be one of {sorted(_UNIT)}")
    df = _read(path)
    low = {c.lower().strip(): c for c in df.columns}
    if "interval" in low and interval not in (None, ""):
        df = df[df[low["interval"]].astype(str) == str(interval)]
        if df.empty:
            raise ValueError(f"no Model B rows for interval {interval!r}")
    elif "interval" in low and df[low["interval"]].nunique() > 1:
        raise ValueError("the file has several intervals: pass interval=... (values: "
                         + ", ".join(map(str, sorted(df[low['interval']].astype(str).unique())[:8])) + " ...)")
    tcol = next((low[c] for c in _TIME_COLS if c in low), None)
    if tcol is None:
        raise ValueError(f"cannot find the time column (looked for {_TIME_COLS}); found {list(df.columns)}")
    ids = _edge_ids(df)
    vals = pd.to_numeric(df[tcol], errors="coerce").to_numpy(dtype=float) * _UNIT[unit]

    unknown = sorted({e for e in ids if e not in E})
    if unknown:
        raise KeyError(f"{len(unknown)} edge IDs are not in Model A, e.g. {unknown[:3]}")

    mu, dropped, seen = {}, [], {}
    for e, t in zip(ids, vals):
        if not (math.isfinite(t) and t > 0):
            dropped.append(e)
            continue
        seen.setdefault(e, []).append(t)
    for e, ts in seen.items():
        if len(ts) > 1 and max(ts) - min(ts) > 1e-9:
            if aggregate == "mean":
                ts = [sum(ts) / len(ts)]
            elif aggregate == "last":
                ts = [ts[-1]]
            else:
                raise ValueError(f"edge {e} has {len(ts)} different times in this interval; pass aggregate='mean' or 'last'")
        mu[e] = ts[-1]
    absent = sorted(e for e in E if e not in mu and e not in set(dropped))
    report = dict(file=str(path), interval=interval, unit=unit, rows=len(df), edges_with_time=len(mu),
                  dropped_invalid=sorted(set(dropped)), absent_from_file=absent,
                  fallback_to_free_flow=len(E) - len(mu), total_edges=len(E))
    if strict and report["fallback_to_free_flow"]:
        raise ValueError(f"strict: {report['fallback_to_free_flow']} of {len(E)} edges have no valid Model B time "
                         f"(invalid rows {len(report['dropped_invalid'])}, absent {len(absent)})")
    return mu, report


def install_model_b(path, interval=None, unit="s", aggregate=None, strict=False):
    """Load + install into Model C (task1_dijkstra.E). Returns the report; edges without a valid time keep free-flow."""
    mu, report = load_model_b(path, interval, unit, aggregate, strict)
    T1.set_travel_times(mu, source=f"Model B file {os.path.basename(str(path))}")
    report["source_text"] = T1.MODEL_B_SOURCE
    return report


# ---------------------------------------------------------------- Model D
def load_signal_deltas(path, allow_unverified=False):
    """Model D file -> {approach (u,v,k): delta_s}. Columns: approach_edge_id ('u-v-k') or u,v,key ; delta_s.
    Rows with an empty delta_s are skipped (the template is filled in gradually). A delta on an edge that does not
    end at a VERIFIED signal is an error (it would be ignored by the export), unless allow_unverified=True."""
    from task3_rerouting import SIGNALS
    df = _read(path)
    low = {c.lower().strip(): c for c in df.columns}
    dcol = next((low[c] for c in ("delta_s", "delta_t_s", "delta_t", "delta") if c in low), None)
    if dcol is None:
        raise ValueError("cannot find the delta column (delta_s)")
    if "approach_edge_id" in low:
        df = df.rename(columns={low["approach_edge_id"]: "edge_id"})
    ids = _edge_ids(df)
    vals = pd.to_numeric(df[dcol], errors="coerce").to_numpy(dtype=float)
    out = {}
    for e, d in zip(ids, vals):
        if not math.isfinite(d):
            continue
        if e not in E:
            raise KeyError(f"edge {e} is not in Model A")
        if e[1] not in SIGNALS and not allow_unverified:
            raise ValueError(f"edge {e} does not end at a verified signal (verified: {sorted(SIGNALS)})")
        out[e] = float(d)
    return out


def make_signal_template(path="Model_D_signal_deltas_template.csv"):
    """List every approach edge of every verified signal; Model D fills delta_s = T_priority - T_normal (seconds)."""
    from task3_rerouting import SIGNALS
    rows = [dict(signal_node=e[1], approach_edge_id=f"{e[0]}-{e[1]}-{e[2]}", delta_s="")
            for e in sorted(E) if e[1] in SIGNALS]
    pd.DataFrame(rows).to_csv(path, index=False)
    return len(rows)


if __name__ == "__main__":
    n = make_signal_template(os.path.join(_HERE, "Model_D_signal_deltas_template.csv"))
    print(f"Model_D_signal_deltas_template.csv written: {n} approach edges of the verified signals")
