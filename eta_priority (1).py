"""
eta_priority.py  (Model D, Salwa): dynamic ETA + preparation/request time for an ambulance route.
Haneen's signal_controller.py refers to this module by this name.

Every input is read from the owner's table or object. If a value is missing the code raises an error
instead of guessing. Times are seconds on the SHARED simulation clock.

INPUTS
  Model A  RouteX_Model_A_Matrices.xlsx, sheet "Edge index" (team v3, 546 edges): edge_index_0based,
           matrix_edge_label, u_node_id, v_node_id, key, length_m, lanes_numeric, lane_status
           edge_id = matrix_edge_label ("e0000" ... "e0545"); Model B's vectors use the same order
  Model A  signalized-intersection and approach mapping. For each incoming directed edge,
           the queue-clearance lane count is Model A lanes_numeric (TOTAL road lanes).
           This is a project modelling assumption; movement-specific lane allocation is not required.
  Model B  travel times: a table  edge_id, mu_s [, sigma_s] [, t_s]   or a matrix mu[T, 546] in Model A order
           (TravelTimes.from_matrix). Seconds, whole edge. If sigma is absent: sigma_e = 0.15 * mu_e
  Model C  route: node path [n0, n1, ...] -> node_path_to_edge_ids(); changes only -> RouteTracker.apply_change()
           or route_change_from_routes(old, new). Edge route ahead: first edge = the ambulance's current edge
  Haneen   Intersection objects (signal_controller.py): T_clearance = Intersection.t_clear;
           approach lane count = total incoming-road lanes_numeric from Model A (project assumption)
           + a map (node_id, approach name) -> Model A approach edge  (SignalMapping.from_haneen)
  Haneen / simulator: queue length N_q per approach (QueueSource) -- not in Haneen's files yet
  Simulator: ev_id, t_now, AmbulanceState(edge_id, distance_along_m)

OUTPUT to Haneen's SignalController (StepResult):
  requests -> controller.submit(r), r = {ev_id, node_id, ev_approach, eta_j_s, sigma_eta_s,
                                         request_time_j_s, feasible}
  cancels  -> controller.cancel(ev_id, node_id, t)            (route no longer passes the intersection)
  passed   -> controller.step(t, passed=passed)               (ambulance has passed the intersection)
  run_controller_cycle() does the three calls in the right order.

EQUATIONS (same labels as the Model D documentation)
  Function 1:  (1) sigma_e = rho * mu_e   (only if Model B gives no sigma)
               (2) ETA_j = t_now + sum(mu_e, e in P_j)
               (3) sigma_ETA,j = sqrt(sum(sigma_e^2, e in P_j))      (independent edge errors)
  Function 2:  (4) T_queue,j = N_q,j / (s * n_lanes,j)
               (5) T_margin,j = k * sigma_ETA,j
               (6) T_prepare,j = T_clearance,j + T_queue,j + T_margin,j
               (7) t_request,j = ETA_j - T_prepare,j      (negligible communication delay, as documented)
               (8) feasible  <=>  t_request,j >= t_now
"""
import io
import math
from bisect import bisect_right
from dataclasses import dataclass, asdict
from typing import Dict, Iterable, List, Optional, Set, Tuple

import pandas as pd

RHO = 0.15    # provisional uncertainty ratio, sigma_e = 0.15 * mu_e (not measured)
S = 0.5       # discharge rate, vehicles per second per lane
K = 2.0       # safety margin, number of sigma


# ----------------------------------------------------------------------------
# Data structures
# ----------------------------------------------------------------------------
@dataclass
class EdgeA:                      # one row of Model A's "Edge index" sheet
    edge_id: str                  # matrix_edge_label, e.g. "e0415"
    index: int                    # 0-based position in Model A's edge order (= Model B vector position)
    key: int
    from_node: int
    to_node: int
    length_m: float
    lanes_total: Optional[int]    # all lanes on the road (Model A "lanes_numeric")
    lanes_note: str = ""          # Model A "lane_status" (resolved / resolved_by_team)


@dataclass
class AmbulanceState:             # from the simulator
    edge_id: str
    distance_along_m: float


@dataclass
class Request:                    # full computed values for one upcoming signalized intersection
    node_id: int
    approach_edge_id: str
    ev_approach: str              # approach name used by Haneen's controller
    eta_s: float
    sigma_eta_s: float
    t_queue_s: float
    t_margin_s: float
    t_clearance_s: float
    t_prepare_s: float
    t_request_s: float            # latest time the controller must START preparing
    feasible: bool                # eq. (8): t_request >= t_now
    issued_at_s: float
    n_lanes_used: int
    lanes_source: str             # "Model A lanes_numeric (total incoming-road lanes; project assumption)"

    @property
    def status(self) -> str:
        """Explicit Model D timing status; Haneen still receives the agreed boolean field."""
        return "ON_TIME" if self.feasible else "LATE_REQUEST"

    @property
    def request_send_time_s(self) -> float:
        """When Model D emits/re-emits the request; t_request_s is the preparation-start deadline."""
        return self.issued_at_s

    def to_haneen(self, ev_id: str) -> dict:
        """Exactly the keys Haneen's SignalController.submit() reads."""
        return dict(ev_id=ev_id, node_id=self.node_id, ev_approach=self.ev_approach,
                    eta_j_s=self.eta_s, sigma_eta_s=self.sigma_eta_s,
                    request_time_j_s=self.t_request_s, feasible=self.feasible)


@dataclass
class StepResult:
    t_now: float
    requests: List[dict]                  # Haneen-format requests to submit (new, or changed by more than theta)
    cancels: List[Tuple[str, int]]        # (ev_id, node_id): route no longer passes the intersection
    passed: Set[Tuple[str, int]]          # (ev_id, node_id): ambulance has passed the intersection
    details: List[Request]                # all current values, for logs and tests


# ----------------------------------------------------------------------------
# Loaders
# ----------------------------------------------------------------------------
def load_model_a_edges(path: str, sheet: str = "Edge index") -> Dict[str, EdgeA]:
    """Model A edges from RouteX_Model_A_Matrices.xlsx. Keys are edge labels (e0000 ...), in Model A order."""
    df = pd.read_excel(path, sheet_name=sheet)
    need = {"edge_index_0based", "matrix_edge_label", "u_node_id", "v_node_id", "key",
            "length_m", "lanes_numeric", "lane_status"}
    missing = need - set(df.columns)
    if missing:
        raise ValueError(f"Model A sheet '{sheet}' lacks columns: {sorted(missing)}")
    edges: Dict[str, EdgeA] = {}
    for r in df.itertuples(index=False):
        label = str(r.matrix_edge_label)
        if label in edges:
            raise ValueError(f"duplicate edge label {label} in Model A")
        lanes = None if pd.isna(r.lanes_numeric) else int(r.lanes_numeric)
        edges[label] = EdgeA(label, int(r.edge_index_0based), int(r.key), int(r.u_node_id),
                             int(r.v_node_id), float(r.length_m), lanes, str(r.lane_status))
    indices = sorted(e.index for e in edges.values())
    if indices != list(range(len(edges))):
        raise ValueError("Model A edge indices must be unique and contiguous from zero")
    return edges


def edge_ids_in_order(model_a: Dict[str, EdgeA]) -> List[str]:
    """Edge labels sorted by Model A index (the order of Model B's vectors)."""
    return [e.edge_id for e in sorted(model_a.values(), key=lambda e: e.index)]


def read_table(path: str) -> pd.DataFrame:
    return pd.read_csv(path) if str(path).lower().endswith(".csv") else pd.read_excel(path)


class SignalMapping:
    """Signalized nodes and incoming directed approaches. Lane counts are total road lanes from Model A;
    using total lanes for the approach is the project's explicit modelling assumption."""

    def __init__(self, df: pd.DataFrame, lanes_label: str = "Model A lanes_numeric (total incoming-road lanes; modelling assumption)"):
        if "node_id" not in df.columns:
            raise ValueError("traffic-light mapping needs a 'node_id' column")
        self.lanes_label = lanes_label                 # how the lane counts are described in every request
        self.nodes: Set[int] = {int(n) for n in df["node_id"]}
        if not self.nodes:
            raise ValueError("traffic-light mapping is empty: ask the Model A owner for it")
        self.lanes_total_by_approach: Dict[Tuple[int, str], int] = {}
        self.approach_names: Dict[Tuple[int, str], str] = {}
        if "approach_edge_id" in df.columns:
            for r in df.dropna(subset=["approach_edge_id"]).to_dict("records"):
                key = (int(r["node_id"]), str(r["approach_edge_id"]))
                if "lanes_total" in r and not pd.isna(r["lanes_total"]):
                    lanes = int(r["lanes_total"])
                    if lanes <= 0:
                        raise ValueError(f"incoming edge {key[1]} at node {key[0]} has non-positive Model A lanes_numeric")
                    self.lanes_total_by_approach[key] = lanes
                if "approach_name" in r and not pd.isna(r["approach_name"]):
                    self.approach_names[key] = str(r["approach_name"])

    @classmethod
    def from_haneen(cls, intersections, approach_edges: Dict[Tuple[int, str], str]):
        """intersections: Haneen's Intersection objects. approach_edges: {(node_id, approach_name): Model A edge
        label of the road that ENTERS the intersection on that approach}. Lanes come from approaches[name]['n']."""
        rows = []
        for (node, name), edge in approach_edges.items():
            I = next((i for i in intersections if i.node_id == node), None)
            if I is None or name not in I.approaches:
                raise KeyError(f"approach {name} at node {node} is not in Haneen's intersections")
            rows.append(dict(node_id=node, approach_edge_id=edge, approach_name=name,
                             lanes_total=I.approaches[name]["n"]))
        return cls(pd.DataFrame(rows))


class TravelTimes:
    """Model B travel times, supporting either timestamped values or strict time-bin intervals.

    For binned data, a value is returned only inside its supplied [start, end) interval.
    Gaps and times outside the supplied bins raise KeyError; no extra bins are invented.
    """

    def __init__(self, df: pd.DataFrame):
        self._rows: Dict[str, List[Tuple[float, float, Optional[float]]]] = {}
        self._bins: Dict[str, List[Tuple[float, float, float, Optional[float]]]] = {}

        if {"edge_id", "mu_s"} <= set(df.columns):
            for r in df.to_dict("records"):
                t = float(r["t_s"]) if "t_s" in df.columns and not pd.isna(r.get("t_s")) else 0.0
                sg = r.get("sigma_s") if "sigma_s" in df.columns else None
                sg = None if sg is None or pd.isna(sg) else float(sg)
                mu = float(r["mu_s"])
                if not math.isfinite(mu) or mu <= 0 or (sg is not None and (not math.isfinite(sg) or sg < 0)):
                    raise ValueError("Model B travel times must be positive and uncertainties non-negative")
                self._rows.setdefault(str(r["edge_id"]), []).append((t, mu, sg))
            for rows in self._rows.values():
                rows.sort(key=lambda x: x[0])
                if len({r[0] for r in rows}) != len(rows):
                    raise ValueError("duplicate Model B update time for an edge")

        elif {"edge_id", "mu_hat_s", "start_s_since_midnight", "end_s_since_midnight"} <= set(df.columns):
            sigma_col = "sigma_hat_s_provisional" if "sigma_hat_s_provisional" in df.columns else None
            for r in df.to_dict("records"):
                start, end, mu = (float(r["start_s_since_midnight"]),
                                  float(r["end_s_since_midnight"]), float(r["mu_hat_s"]))
                sg = r.get(sigma_col) if sigma_col else None
                sg = None if sg is None or pd.isna(sg) else float(sg)
                if not (math.isfinite(start) and math.isfinite(end) and start < end):
                    raise ValueError("Model B time bins must have finite start < end")
                if not math.isfinite(mu) or mu <= 0 or (sg is not None and (not math.isfinite(sg) or sg < 0)):
                    raise ValueError("Model B travel times must be positive and uncertainties non-negative")
                self._bins.setdefault(str(r["edge_id"]), []).append((start, end, mu, sg))
            for rows in self._bins.values():
                rows.sort(key=lambda x: x[0])
                for prev, cur in zip(rows, rows[1:]):
                    if cur[0] < prev[1]:
                        raise ValueError("overlapping Model B time bins for an edge")
        else:
            raise ValueError("Model B needs edge_id/mu_s or edge_id/mu_hat_s/start_s_since_midnight/end_s_since_midnight")

    @classmethod
    def from_matrix(cls, mu, edge_ids: List[str], t_starts=None, sigma=None):
        """Model B output as arrays: mu[T][m] (or mu[m]) in Model A edge order."""
        import numpy as np
        mu = np.atleast_2d(np.asarray(mu, dtype=float))
        T, m = mu.shape
        if m != len(edge_ids):
            raise ValueError(f"Model B vector has {m} entries but Model A has {len(edge_ids)} edges")
        t_starts = [0.0] * T if t_starts is None else list(t_starts)
        if len(t_starts) != T:
            raise ValueError("t_starts must have one entry per Model B update")
        sg = None if sigma is None else np.atleast_2d(np.asarray(sigma, dtype=float))
        if sg is not None and sg.shape != mu.shape:
            raise ValueError("sigma matrix must have the same shape as mu")
        recs = []
        for ti in range(T):
            for j, eid in enumerate(edge_ids):
                rec = {"edge_id": eid, "mu_s": mu[ti, j], "t_s": t_starts[ti]}
                if sg is not None:
                    rec["sigma_s"] = sg[ti, j]
                recs.append(rec)
        return cls(pd.DataFrame(recs))

    def get(self, edge_id: str, t_now: float) -> Tuple[float, Optional[float]]:
        if self._bins:
            rows = self._bins.get(edge_id)
            if not rows:
                raise KeyError(f"Model B has no travel time for edge {edge_id}")
            for start, end, mu, sigma in rows:
                if start <= t_now < end:
                    return mu, sigma
            raise KeyError(f"Model B has no estimate for edge {edge_id} at t={t_now}; outside supplied bins or in a gap")
        rows = self._rows.get(edge_id)
        if not rows:
            raise KeyError(f"Model B has no travel time for edge {edge_id}")
        i = bisect_right([r[0] for r in rows], t_now) - 1
        if i < 0:
            raise KeyError(f"Model B has no estimate for edge {edge_id} at t = {t_now}")
        return rows[i][1], rows[i][2]


class QueueSource:
    """Haneen: queue length N_q (vehicles). Latest value valid at t_now."""

    def __init__(self, df: pd.DataFrame):
        if not {"node_id", "queue_veh"} <= set(df.columns):
            raise ValueError("queue table needs columns 'node_id' and 'queue_veh'")
        by_edge = "approach_edge_id" in df.columns
        self._rows: Dict[Tuple[int, Optional[str]], List[Tuple[float, float]]] = {}
        for r in df.to_dict("records"):
            t = float(r["t_s"]) if "t_s" in df.columns and not pd.isna(r.get("t_s")) else 0.0
            key = (int(r["node_id"]), str(r["approach_edge_id"]) if by_edge else None)
            self._rows.setdefault(key, []).append((t, float(r["queue_veh"])))
        for rows in self._rows.values():
            rows.sort(key=lambda x: x[0])

    def get(self, node_id: int, approach_edge_id: str, t_now: float) -> float:
        rows = self._rows.get((node_id, approach_edge_id)) or self._rows.get((node_id, None))
        if not rows:
            raise KeyError(f"Haneen's queue table has no value for node {node_id}, edge {approach_edge_id}")
        i = bisect_right([r[0] for r in rows], t_now) - 1
        if i < 0:
            raise KeyError(f"no queue value for node {node_id} at t = {t_now}")
        return rows[i][1]


class ClearanceSource:
    """Haneen: yellow + all-red time per intersection."""

    def __init__(self, df: pd.DataFrame):
        if not {"node_id", "t_clearance_s"} <= set(df.columns):
            raise ValueError("clearance table needs columns 'node_id' and 't_clearance_s'")
        self._v = {int(r.node_id): float(r.t_clearance_s) for r in df.itertuples(index=False)}

    @classmethod
    def from_intersections(cls, intersections):
        """T_clearance = yellow + all-red = Intersection.t_clear in Haneen's controller."""
        return cls(pd.DataFrame({"node_id": [i.node_id for i in intersections],
                                 "t_clearance_s": [i.t_clear for i in intersections]}))

    def get(self, node_id: int) -> float:
        if node_id not in self._v:
            raise KeyError(f"Haneen's clearance table has no value for node {node_id}")
        return self._v[node_id]


# ----------------------------------------------------------------------------
# Loaders for the team's CSV files
# ----------------------------------------------------------------------------
def read_csv_clean(path: str) -> pd.DataFrame:
    """The team's CSVs have stray carriage returns (\\r\\r\\n) and a BOM; remove them before parsing."""
    with open(path, encoding="utf-8-sig", newline="") as f:
        text = f.read().replace("\r", "")
    df = pd.read_csv(io.StringIO(text))
    df.columns = [c.strip() for c in df.columns]
    return df


def edges_by_uvk(model_a: Dict[str, EdgeA]) -> Dict[Tuple[int, int, int], str]:
    return {(e.from_node, e.to_node, e.key): e.edge_id for e in model_a.values()}


def load_model_b_csv(path: str, model_a: Dict[str, EdgeA], sim_start_s: float) -> "TravelTimes":
    """Load Model_B_for_Model_D.csv without changing its supplied time-bin coverage.

    sim_start_s is the seconds-since-midnight value corresponding to simulation time 0.
    The start/end boundaries are shifted onto the simulation clock; gaps and out-of-range times remain gaps.
    """
    df = read_csv_clean(path)
    need = {"edge_index", "edge_id", "start_s_since_midnight", "end_s_since_midnight", "mu_hat_s"}
    if need - set(df.columns):
        raise ValueError(f"Model B file lacks columns: {sorted(need - set(df.columns))}")
    expected_ids = set(model_a)
    actual_ids = set(df["edge_id"].astype(str))
    if actual_ids != expected_ids:
        raise ValueError(f"Model B edge coverage differs from Model A: missing={len(expected_ids-actual_ids)}, extra={len(actual_ids-expected_ids)}")
    for r in df.drop_duplicates("edge_id").itertuples(index=False):
        if r.edge_id not in model_a or model_a[r.edge_id].index != int(r.edge_index):
            raise ValueError(f"Model B edge {r.edge_id} (index {r.edge_index}) does not match Model A")
    bins = df[["edge_id", "mu_hat_s", "start_s_since_midnight", "end_s_since_midnight"]].copy()
    if df.duplicated(["edge_id", "start_s_since_midnight", "end_s_since_midnight"]).any():
        raise ValueError("Model B contains duplicate edge estimates within a time bin")
    if "sigma_hat_s_provisional" in df.columns:
        bins["sigma_hat_s_provisional"] = df["sigma_hat_s_provisional"]
    bins["start_s_since_midnight"] = bins["start_s_since_midnight"].astype(float) - float(sim_start_s)
    bins["end_s_since_midnight"] = bins["end_s_since_midnight"].astype(float) - float(sim_start_s)
    # Every supplied time bin must cover every Model A edge exactly once.
    counts = bins.groupby(["start_s_since_midnight", "end_s_since_midnight"])["edge_id"].nunique()
    if not (counts == len(model_a)).all():
        raise ValueError("each Model B time bin must contain exactly one estimate for every Model A edge")
    return TravelTimes(bins)


def load_signal_mapping(clean_csv: str, approaches_csv: str, model_a: Dict[str, EdgeA]) -> SignalMapping:
    """RouteX_Signalized_Intersections_Clean.csv + RouteX_Model_D_Approaches_Template.csv.
    Only signals with model_d_ev_priority_usable = TRUE are mapped (node 4 has no incoming edge)."""
    clean = read_csv_clean(clean_csv)
    usable = {int(r.node_id) for r in clean.itertuples(index=False)
              if str(r.model_d_ev_priority_usable).strip().upper() == "TRUE"}
    if not usable:
        raise ValueError("no usable signalized intersection in the Model A signal file")
    uvk = edges_by_uvk(model_a)
    rows = []
    for r in read_csv_clean(approaches_csv).itertuples(index=False):
        if int(r.signal_node_id) not in usable:
            continue
        label = uvk[(int(r.u_node_id), int(r.v_node_id), int(r.key))]
        if model_a[label].index != int(r.edge_index_0based) or model_a[label].to_node != int(r.signal_node_id):
            raise ValueError(f"approach {r.approach_id}: edge index does not match Model A")
        lanes = model_a[label].lanes_total
        if lanes is None or int(lanes) <= 0:
            raise ValueError(f"incoming Model A edge {label} at node {r.signal_node_id} must have positive lanes_numeric")
        rows.append(dict(node_id=int(r.signal_node_id), approach_edge_id=label,
                         approach_name=str(r.approach_id), lanes_total=int(lanes)))
    return SignalMapping(pd.DataFrame(rows),
                         lanes_label="Model A lanes_numeric (total incoming-road lanes; project modelling assumption)")


@dataclass
class Trip:
    trip_id: str
    route_versions: List[Tuple[float, List[str]]]                 # (valid_from_s, FULL route as edge labels)
    track: List[Tuple[float, int, Optional[int], Optional[str]]]  # (t, node, next_node, edge label being entered)

    def end_time(self) -> float:
        return self.track[-1][0]

    def arrival_time_at(self, node_id: int) -> Optional[float]:
        for t, node, _, _ in self.track:
            if node == node_id and t > self.track[0][0]:
                return t
        return None

    def retimed(self, model_a: Dict[str, EdgeA], tt: "TravelTimes") -> "Trip":
        """Same path and same route changes, but every road is driven in its Model B travel time (the ambulance
        moves like ordinary traffic). Route-change times move to the new time at which the same node is reached."""
        new_track, mapping, t = [], {}, self.track[0][0]
        for t_old, node, nxt, label in self.track:
            mapping[round(t_old, 6)] = t
            new_track.append((t, node, nxt, label))
            if label is not None:
                t += tt.get(label, t)[0]
        versions = []
        for valid_from, route in self.route_versions:
            if round(valid_from, 6) not in mapping:
                raise ValueError(f"route change at {valid_from} s does not match a node passage in the trip")
            versions.append((mapping[round(valid_from, 6)], route))
        return Trip(self.trip_id, versions, new_track)

    def state_at(self, t: float, model_a: Dict[str, EdgeA]) -> Optional[AmbulanceState]:
        """Current edge and distance along it (linear in time between two node passages). None after arrival."""
        if t < self.track[0][0]:
            raise ValueError("time is before the trip starts")
        if t >= self.track[-1][0]:
            return None
        times = [r[0] for r in self.track]
        i = bisect_right(times, t) - 1
        t0, t1, label = times[i], times[i + 1], self.track[i][3]
        frac = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return AmbulanceState(label, frac * model_a[label].length_m)


def load_trips(routes_csv: str, positions_csv: str, model_a: Dict[str, EdgeA]) -> Dict[str, Trip]:
    """Model C's trip_routes.csv and trip_positions.csv. The directed edge is found from (from_node, to_node, key):
    for 'reverse' rows Model C's edge_id names the OPPOSITE directed edge, so edge_id is used only for its key."""
    uvk = edges_by_uvk(model_a)

    def label(frm, to, edge_id):
        key = int(str(edge_id).split("-")[2])
        if (int(frm), int(to), key) not in uvk:
            raise KeyError(f"no Model A edge ({frm},{to},{key})")
        return uvk[(int(frm), int(to), key)]

    routes, tracks = {}, {}
    # Model C's files contain exact duplicate rows (e.g. trip 103->215 is listed twice): keep one copy of each
    rt = read_csv_clean(routes_csv).drop_duplicates().sort_values(["trip_id", "route_version", "order"])
    for (trip, ver), g in rt.groupby(["trip_id", "route_version"], sort=True):
        routes.setdefault(trip, []).append(
            (float(g["valid_from_s"].iloc[0]),
             [label(r.from_node, r.to_node, r.edge_id) for r in g.itertuples(index=False)]))
    for trip, g in read_csv_clean(positions_csv).drop_duplicates().groupby("trip_id", sort=False):
        rows = []
        for r in g.itertuples(index=False):
            has_next = not pd.isna(r.next_node)
            rows.append((float(r.time_s), int(r.node), int(r.next_node) if has_next else None,
                         label(r.node, r.next_node, r.next_edge_id) if has_next else None))
        tracks[trip] = rows
    if set(routes) != set(tracks):
        raise ValueError("trip_routes.csv and trip_positions.csv list different trips")
    return {t: Trip(t, routes[t], tracks[t]) for t in routes}


class AssumedQueue:
    """ASSUMPTION (no queue data exists). Queue ahead of the ambulance on its approach:
        N_q = min( lam * red_s , n_lanes * L_e / l_veh )
    lam = assumed arrival rate of that approach (veh/s), red_s = assumed red duration (s),
    n_lanes*L_e/l_veh = storage limit of the approach (l_veh = vehicle spacing in m, as in Haneen's controller)."""

    def __init__(self, signals: SignalMapping, model_a: Dict[str, EdgeA],
                 lam_by_approach: Dict[str, float], red_s: float = 30.0, l_veh: float = 7.5):
        self.sig, self.A, self.lam, self.red_s, self.l_veh = signals, model_a, lam_by_approach, red_s, l_veh

    def get(self, node_id: int, approach_edge_id: str, t_now: float) -> float:
        key = (node_id, approach_edge_id)
        if key not in self.sig.approach_names or key not in self.sig.lanes_total_by_approach:
            raise KeyError(f"no approach data for node {node_id}, edge {approach_edge_id}")
        name = self.sig.approach_names[key]
        if name not in self.lam:
            raise KeyError(f"no assumed arrival rate for approach {name}")
        storage = self.sig.lanes_total_by_approach[key] * self.A[approach_edge_id].length_m / self.l_veh
        return min(self.lam[name] * self.red_s, storage)


# ----------------------------------------------------------------------------
# Route handling (Model C sends the route AHEAD, then only changes)
# ----------------------------------------------------------------------------
class RouteTracker:
    def __init__(self, ahead_route: Iterable[str]):
        self.route: List[str] = list(ahead_route)

    def advance(self, current_edge_id: str) -> List[str]:
        """Drop edges already driven. Returns the driven edges."""
        if current_edge_id not in self.route:
            raise ValueError(f"ambulance edge {current_edge_id} is not on the route ahead")
        i = self.route.index(current_edge_id)
        driven, self.route = self.route[:i], self.route[i:]
        return driven

    def apply_change(self, change: dict):
        """change = {"remove": [...], "insert_after": edge_id, "add": [...]}  (applied immediately)"""
        remove = set(change.get("remove", []))
        add = list(change.get("add", []))
        after = change.get("insert_after")
        new = [e for e in self.route if e not in remove]
        if add:
            if after is None:
                pos = 0
            elif after in new:
                pos = new.index(after) + 1
            else:
                raise ValueError(f"insert_after edge {after} is not on the route")
            new[pos:pos] = add
        self.route = new


def node_path_to_edge_ids(node_path: List[int], model_a: Dict[str, EdgeA],
                          tt: TravelTimes, t_now: float) -> List[str]:
    """Model C returns routes as node lists. Convert to edge labels. Where parallel edges join the same two
    nodes, the one with the smallest Model B travel time is taken (the same rule as Model C's graph build)."""
    by_pair: Dict[Tuple[int, int], List[str]] = {}
    for e in model_a.values():
        by_pair.setdefault((e.from_node, e.to_node), []).append(e.edge_id)
    out = []
    for u, v in zip(node_path, node_path[1:]):
        cands = by_pair.get((u, v))
        if not cands:
            raise ValueError(f"no directed edge {u} -> {v} in Model A")
        out.append(min(cands, key=lambda eid: tt.get(eid, t_now)[0]))
    return out


def route_change_from_routes(old_route: List[str], new_route: List[str]) -> dict:
    """If Model C sends a full new route, derive the change message for RouteTracker.apply_change()."""
    n = 0
    while n < min(len(old_route), len(new_route)) and old_route[n] == new_route[n]:
        n += 1
    if n == 0:
        raise ValueError("old and new route share no starting edge; cannot express as a change")
    return {"remove": old_route[n:], "insert_after": old_route[n - 1], "add": new_route[n:]}


# ----------------------------------------------------------------------------
# Function 1 and Function 2
# ----------------------------------------------------------------------------
def remaining_route(route_ahead, amb: AmbulanceState, model_a: Dict[str, EdgeA],
                    tt: TravelTimes, t_now: float, rho: float = RHO):
    """[(edge, mu_remaining, sigma_remaining)]: the current edge is scaled to what is left of it."""
    if not route_ahead or route_ahead[0] != amb.edge_id:
        raise ValueError("first edge of the route ahead must be the ambulance's current edge")
    out = []
    for i, eid in enumerate(route_ahead):
        if eid not in model_a:
            raise KeyError(f"edge {eid} is not in Model A")
        e = model_a[eid]
        mu, sigma = tt.get(eid, t_now)
        if sigma is None:
            sigma = rho * mu                                             # eq. I
        frac = 1.0
        if i == 0:
            frac = max(0.0, min(1.0, 1.0 - amb.distance_along_m / e.length_m))
        out.append((e, mu * frac, sigma * frac))
    return out


def compute_eta(remaining, signalized_nodes: Set[int], t_now: float):
    """Function 1. Upcoming signalized intersections only."""
    total_mu, total_var, result = 0.0, 0.0, []
    for edge, mu_rem, sigma_rem in remaining:
        total_mu += mu_rem
        total_var += sigma_rem ** 2
        if edge.to_node in signalized_nodes:
            result.append({"node_id": edge.to_node, "approach_edge_id": edge.edge_id,
                           "eta": t_now + total_mu,                      # eq. II
                           "sigma_eta": math.sqrt(total_var)})           # eq. III
    return result


def compute_preparation(eta, sigma_eta, n_queue, n_lanes, t_clearance, s=S, k=K):
    """Function 2. Returns (T_prepare, t_request)."""
    if n_lanes < 1:
        raise ValueError("n_lanes must be at least 1")
    if s <= 0 or k < 0 or n_queue < 0 or sigma_eta < 0 or t_clearance < 0:
        raise ValueError("s must be positive; k, queue, uncertainty and clearance must be non-negative")
    if not all(math.isfinite(float(v)) for v in (eta, sigma_eta, n_queue, n_lanes, t_clearance, s, k)):
        raise ValueError("preparation inputs must be finite numbers")
    t_queue = n_queue / (s * n_lanes)                                    # eq. I
    t_margin = k * sigma_eta                                             # eq. II
    t_prepare = t_clearance + t_queue + t_margin                         # eq. III
    return t_prepare, eta - t_prepare                                    # eq. IV


# ----------------------------------------------------------------------------
# Model D runner
# ----------------------------------------------------------------------------
class ModelD:
    def __init__(self, ev_id: str, model_a: Dict[str, EdgeA], signals: SignalMapping,
                 travel_times: TravelTimes, queues: QueueSource, clearances: ClearanceSource,
                 initial_route_ahead: List[str], theta_eta_s: float = 0.0,
                 s: float = S, k: float = K, rho: float = RHO):
        """theta_eta_s: a request is re-submitted only when its ETA or request time changes by more than this
        (rule R7). Haneen lists it as 'to be set in the simulator'; 0.0 means any change is re-submitted."""
        self.ev_id, self.A, self.sig, self.tt, self.q, self.cl = ev_id, model_a, signals, travel_times, queues, clearances
        self.tracker = RouteTracker(initial_route_ahead)
        self.theta, self.s, self.k, self.rho = theta_eta_s, s, k, rho
        self.active: Dict[int, Request] = {}
        self._t_start: Dict[int, float] = {}      # time preparation started, per intersection

    def _lanes(self, node_id: int, edge_id: str):
        n = self.sig.lanes_total_by_approach.get((node_id, edge_id))
        if n is not None:
            return n, self.sig.lanes_label
        n = self.A[edge_id].lanes_total
        if n is None or int(n) <= 0:
            raise ValueError(f"Model A lanes_numeric must be positive for incoming edge {edge_id} at node {node_id}")
        return int(n), "Model A lanes_numeric (total incoming-road lanes; project assumption)"

    def step(self, t_now: float, amb: AmbulanceState, route_change: Optional[dict] = None,
             new_full_route: Optional[List[str]] = None) -> StepResult:
        """Call every simulation step. A route update is either a change message (route_change) or a FULL new
        route from the origin (new_full_route, as in Model C's trip_routes.csv); both apply immediately."""
        if new_full_route is not None:
            if amb.edge_id not in new_full_route:
                raise ValueError(f"ambulance edge {amb.edge_id} is not on the new route")
            i = new_full_route.index(amb.edge_id)
            driven_set = set(new_full_route[:i])
            driven = [e for e in self.tracker.route if e in driven_set]
            self.tracker.route = list(new_full_route[i:])
        else:
            driven = self.tracker.advance(amb.edge_id)
        if route_change:
            self.tracker.apply_change(route_change)
            if not self.tracker.route or self.tracker.route[0] != amb.edge_id:
                raise ValueError("route change must keep the ambulance's current edge first")

        remaining = remaining_route(self.tracker.route, amb, self.A, self.tt, t_now, self.rho)
        new: Dict[int, Request] = {}
        for r in compute_eta(remaining, self.sig.nodes, t_now):
            j, e_j = r["node_id"], r["approach_edge_id"]
            n_lanes, src = self._lanes(j, e_j)
            n_q, t_clear = self.q.get(j, e_j, t_now), self.cl.get(j)
            t_prep, t_req = compute_preparation(r["eta"], r["sigma_eta"], n_q, n_lanes, t_clear, self.s, self.k)
            held = self.active.get(j)
            if j not in self._t_start and t_req <= t_now:        # preparation is due now: when did it start?
                on_time = held is not None and held.feasible and held.t_request_s <= t_now
                self._t_start[j] = held.t_request_s if on_time else t_now
            # eq. (8): feasible if the request time is not earlier than the time preparation could start
            # (t_now before it is due; the actual start time once it is due, so a request that is already
            # being served is not reported as late again at every later step)
            feasible = t_req >= self._t_start.get(j, t_now)
            new[j] = Request(j, e_j, self.sig.approach_names.get((j, e_j), e_j), r["eta"], r["sigma_eta"],
                             n_q / (self.s * n_lanes), self.k * r["sigma_eta"], t_clear, t_prep, t_req,
                             feasible, t_now, n_lanes, src)

        submit = []
        for j, req in new.items():
            old = self.active.get(j)
            if (old is None or old.feasible != req.feasible
                    or abs(req.eta_s - old.eta_s) > self.theta
                    or abs(req.t_request_s - old.t_request_s) > self.theta):
                submit.append(req.to_haneen(self.ev_id))
            else:
                new[j] = old                      # unchanged: keep the request the controller already holds
        passed, cancels = set(), []
        for j, old in self.active.items():
            if j not in new:
                if old.approach_edge_id in driven:
                    passed.add((self.ev_id, j))              # driven past it -> controller.step(passed=...)
                else:
                    cancels.append((self.ev_id, j))          # route changed -> controller.cancel(...)
        for j in [j for j in self._t_start if j not in new]:
            del self._t_start[j]
        self.active = new
        return StepResult(t_now, submit, cancels, passed, list(new.values()))


def run_controller_cycle(controller, res: StepResult, t: float):
    """One call per simulation step: cancel, submit, then step Haneen's SignalController.
    Returns the controller's result dicts (status, phase, feasible, ev_signal_delay_s, ...)."""
    out = [c for c in (controller.cancel(ev, node, t) for ev, node in res.cancels) if c]
    for r in res.requests:
        controller.submit(r)
    return out + controller.step(t, passed=res.passed)
