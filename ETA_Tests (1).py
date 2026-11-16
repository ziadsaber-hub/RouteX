"""Automated tests for Model D (Salwa).

Run from this folder:
    python ETA_Tests.py

The fixtures are deliberately small and synthetic. They test the equations and interfaces
without requiring the full project datasets. Full-data integration was also checked against
the uploaded Model A/B/C files and Haneen's controller; see README.md.
"""

import unittest
import pandas as pd
import tempfile
import os
import csv

from eta_priority import load_signal_mapping
from run_model_d_final import make_intersection, _approach_lam

from eta_priority import (
    AmbulanceState,
    ClearanceSource,
    EdgeA,
    ModelD,
    QueueSource,
    SignalMapping,
    TravelTimes,
    compute_preparation,
    edge_ids_in_order,
    node_path_to_edge_ids,
    route_change_from_routes,
)


class ModelDTests(unittest.TestCase):
    def setUp(self):
        # Small directed graph. e0007 is a slower parallel edge 1 -> 2.
        pairs = [
            (1, 2, 0, 200), (2, 3, 0, 300), (3, 4, 0, 250),
            (4, 5, 0, 200), (5, 6, 0, 150), (3, 8, 0, 220),
            (8, 9, 0, 180), (1, 2, 1, 200),
        ]
        self.A = {
            f"e{i:04d}": EdgeA(f"e{i:04d}", i, key, u, v, float(length), 2, "test fixture")
            for i, (u, v, key, length) in enumerate(pairs)
        }
        self.ids = edge_ids_in_order(self.A)
        # 30 km/h test travel times; parallel edge is deliberately slower.
        self.mu = [length / (30.0 / 3.6) for _, _, _, length in pairs]
        self.mu[7] = self.mu[0] * 2
        self.tt_df = pd.DataFrame({"edge_id": self.ids, "mu_s": self.mu})
        self.tt = TravelTimes(self.tt_df)
        self.signals = SignalMapping(pd.DataFrame({"node_id": [2, 4, 5, 8]}))
        self.queues = QueueSource(pd.DataFrame({
            "node_id": [2, 4, 5, 8], "queue_veh": [8, 8, 8, 8]
        }))
        self.clearances = ClearanceSource(pd.DataFrame({
            "node_id": [2, 4, 5, 8], "t_clearance_s": [5.0] * 4
        }))
        self.route = ["e0000", "e0001", "e0002", "e0003", "e0004"]

    def make_model(self, **kwargs):
        return ModelD(
            "EV_TEST", self.A, kwargs.get("signals", self.signals),
            kwargs.get("travel_times", self.tt), kwargs.get("queues", self.queues),
            self.clearances, kwargs.get("route", self.route),
            theta_eta_s=kwargs.get("theta_eta_s", 0.0),
        )

    def test_eta_uses_only_remaining_current_edge(self):
        model = self.make_model()
        start = model.step(0.0, AmbulanceState("e0000", 0.0))
        halfway = model.step(12.0, AmbulanceState("e0000", 100.0))
        eta_start = next(x.eta_s for x in start.details if x.node_id == 2)
        eta_halfway = next(x.eta_s for x in halfway.details if x.node_id == 2)
        self.assertAlmostEqual(eta_start, 24.0)
        self.assertAlmostEqual(eta_halfway, 24.0)

    def test_eta_changes_when_position_changes(self):
        model = self.make_model()
        result = model.step(12.0, AmbulanceState("e0000", 150.0))
        eta = next(x.eta_s for x in result.details if x.node_id == 2)
        self.assertAlmostEqual(eta, 18.0)

    def test_preparation_equations_and_request_deadline(self):
        # Nq=8, s=.5, n=2 -> 8s; margin=2*3.6=7.2s; clearance=5s.
        prepare, request = compute_preparation(24.0, 3.6, 8.0, 2, 5.0)
        self.assertAlmostEqual(prepare, 20.2)
        self.assertAlmostEqual(request, 3.8)

    def test_haneen_request_format_and_send_vs_prepare_times(self):
        model = self.make_model()
        result = model.step(0.0, AmbulanceState("e0000", 0.0))
        request = next(x for x in result.details if x.node_id == 2)
        self.assertEqual(request.status, "ON_TIME")
        self.assertEqual(request.request_send_time_s, 0.0)
        self.assertAlmostEqual(request.t_request_s, 3.8)
        self.assertEqual(
            set(request.to_haneen("EV_TEST")),
            {"ev_id", "node_id", "ev_approach", "eta_j_s", "sigma_eta_s", "request_time_j_s", "feasible"},
        )

    def test_late_request_is_explicitly_flagged(self):
        model = self.make_model()
        result = model.step(0.0, AmbulanceState("e0000", 190.0))
        request = next(x for x in result.details if x.node_id == 2)
        self.assertFalse(request.feasible)
        self.assertEqual(request.status, "LATE_REQUEST")

    def test_model_b_time_bins_update_and_gaps_are_not_filled(self):
        binned = pd.DataFrame([
            {"edge_id": "e0000", "mu_hat_s": 35.0, "sigma_hat_s_provisional": 9.0,
             "start_s_since_midnight": 25200, "end_s_since_midnight": 39600},
            {"edge_id": "e0000", "mu_hat_s": 42.0, "sigma_hat_s_provisional": 10.0,
             "start_s_since_midnight": 39600, "end_s_since_midnight": 50400},
            {"edge_id": "e0000", "mu_hat_s": 50.0, "sigma_hat_s_provisional": 12.0,
             "start_s_since_midnight": 54000, "end_s_since_midnight": 68400},
        ])
        tt = TravelTimes(binned)
        self.assertEqual(tt.get("e0000", 25200), (35.0, 9.0))
        self.assertEqual(tt.get("e0000", 39600), (42.0, 10.0))
        with self.assertRaises(KeyError):
            tt.get("e0000", 50400)  # 14:00-15:00 is not supplied
        self.assertEqual(tt.get("e0000", 54000), (50.0, 12.0))
        with self.assertRaises(KeyError):
            tt.get("e0000", 68400)  # no extrapolation after the last supplied bin

    def test_traffic_condition_update_changes_eta(self):
        matrix_tt = TravelTimes.from_matrix(
            [self.mu, [x * 2 for x in self.mu]], self.ids, t_starts=[0.0, 10.0]
        )
        model = self.make_model(travel_times=matrix_tt)
        eta_early = next(x.eta_s for x in model.step(0.0, AmbulanceState("e0000", 0.0)).details
                         if x.node_id == 2)
        eta_later = next(x.eta_s for x in model.step(10.0, AmbulanceState("e0000", 0.0)).details
                         if x.node_id == 2)
        self.assertAlmostEqual(eta_early, 24.0)
        self.assertAlmostEqual(eta_later, 58.0)

    def test_queue_update_changes_preparation_time(self):
        q = QueueSource(pd.DataFrame([
            {"node_id": 2, "queue_veh": 8, "t_s": 0},
            {"node_id": 2, "queue_veh": 4, "t_s": 10},
            {"node_id": 4, "queue_veh": 8, "t_s": 0},
            {"node_id": 5, "queue_veh": 8, "t_s": 0},
            {"node_id": 8, "queue_veh": 8, "t_s": 0},
        ]))
        model = self.make_model(queues=q)
        a = next(x for x in model.step(0.0, AmbulanceState("e0000", 0.0)).details if x.node_id == 2)
        b = next(x for x in model.step(10.0, AmbulanceState("e0000", 0.0)).details if x.node_id == 2)
        self.assertAlmostEqual(a.t_queue_s, 8.0)
        self.assertAlmostEqual(b.t_queue_s, 4.0)
        self.assertLess(b.t_prepare_s, a.t_prepare_s)

    def test_route_change_cancels_removed_signals_and_adds_new_one(self):
        model = self.make_model()
        model.step(0.0, AmbulanceState("e0000", 0.0))
        new_route = ["e0000", "e0001", "e0005", "e0006"]
        result = model.step(40.0, AmbulanceState("e0001", 100.0), new_full_route=new_route)
        self.assertEqual(result.passed, {("EV_TEST", 2)})
        self.assertEqual(sorted(result.cancels), [("EV_TEST", 4), ("EV_TEST", 5)])
        self.assertEqual([r["node_id"] for r in result.requests], [8])
        self.assertEqual(model.tracker.route, ["e0001", "e0005", "e0006"])

    def test_route_change_delta_representation(self):
        change = route_change_from_routes(
            self.route, ["e0000", "e0001", "e0005", "e0006"]
        )
        self.assertEqual(change, {
            "remove": ["e0002", "e0003", "e0004"],
            "insert_after": "e0001",
            "add": ["e0005", "e0006"],
        })

    def test_parallel_edge_mapping_uses_fastest_directed_edge(self):
        self.assertEqual(
            node_path_to_edge_ids([1, 2, 3, 4], self.A, self.tt, 0.0),
            ["e0000", "e0001", "e0002"],
        )
        with self.assertRaises(ValueError):
            node_path_to_edge_ids([1, 3], self.A, self.tt, 0.0)

    def test_model_b_uncertainty_is_used_and_missing_sigma_is_provisional(self):
        supplied = self.tt_df.copy()
        supplied["sigma_s"] = 1.0
        model = self.make_model(travel_times=TravelTimes(supplied))
        detail = next(x for x in model.step(0.0, AmbulanceState("e0000", 0.0)).details if x.node_id == 2)
        self.assertAlmostEqual(detail.sigma_eta_s, 1.0)
        # If sigma is omitted, the fallback is rho * mu (0.15 * 24 = 3.6).
        fallback = next(x for x in self.make_model().step(0.0, AmbulanceState("e0000", 0.0)).details
                        if x.node_id == 2)
        self.assertAlmostEqual(fallback.sigma_eta_s, 0.15 * 24.0)


    def test_signal_mapping_uses_model_a_total_lanes_not_template_movement_lanes(self):
        edges = {
            "e0000": EdgeA("e0000", 0, 0, 10, 142, 100.0, 3, "resolved"),
            "e0001": EdgeA("e0001", 1, 0, 11, 142, 120.0, 2, "resolved"),
        }
        with tempfile.TemporaryDirectory() as d:
            clean = os.path.join(d, "signals.csv")
            approaches = os.path.join(d, "approaches.csv")
            pd.DataFrame([{"node_id": 142, "model_d_ev_priority_usable": "TRUE"}]).to_csv(clean, index=False)
            # n=99 is deliberately different: loader must use Model A lanes_numeric instead.
            pd.DataFrame([
                {"signal_node_id": 142, "u_node_id": 10, "v_node_id": 142, "key": 0,
                 "edge_index_0based": 0, "approach_id": "S", "n": 99},
                {"signal_node_id": 142, "u_node_id": 11, "v_node_id": 142, "key": 0,
                 "edge_index_0based": 1, "approach_id": "N", "n": 88},
            ]).to_csv(approaches, index=False)
            mapping = load_signal_mapping(clean, approaches, edges)
        self.assertEqual(mapping.lanes_total_by_approach[(142, "e0000")], 3)
        self.assertEqual(mapping.lanes_total_by_approach[(142, "e0001")], 2)
        self.assertIn("total incoming-road lanes", mapping.lanes_label)

    def test_signal_mapping_rejects_nonpositive_model_a_lane_count(self):
        edges = {"e0000": EdgeA("e0000", 0, 0, 10, 142, 100.0, 0, "unresolved")}
        with tempfile.TemporaryDirectory() as d:
            clean, approaches = os.path.join(d, "signals.csv"), os.path.join(d, "approaches.csv")
            pd.DataFrame([{"node_id": 142, "model_d_ev_priority_usable": "TRUE"}]).to_csv(clean, index=False)
            pd.DataFrame([{"signal_node_id": 142, "u_node_id": 10, "v_node_id": 142, "key": 0,
                           "edge_index_0based": 0, "approach_id": "S", "n": 1}]).to_csv(approaches, index=False)
            with self.assertRaisesRegex(ValueError, "lanes_numeric"):
                load_signal_mapping(clean, approaches, edges)

    def test_runner_builds_all_approaches_with_total_lanes_and_default_rates(self):
        class CaptureIntersection:
            def __init__(self, node_id, approaches, green=None, cycle_green_s=None):
                self.node_id, self.approaches = node_id, approaches
                self.green, self.cycle_green_s = green, cycle_green_s
        mapping = SignalMapping(pd.DataFrame([
            {"node_id": 142, "approach_edge_id": "e0000", "approach_name": "S", "lanes_total": 3},
            {"node_id": 142, "approach_edge_id": "e0001", "approach_name": "N", "lanes_total": 2},
        ]))
        cfg = {"default_lam_veh_s": 0.1, "lam_by_approach": {},
               "initial_green_by_node": {}, "cycle_green_s": 30.0}
        built = make_intersection(CaptureIntersection, 142, mapping, self.A | {
            "e0000": EdgeA("e0000", 0, 0, 10, 142, 100.0, 3),
            "e0001": EdgeA("e0001", 1, 0, 11, 142, 120.0, 2),
        }, cfg)
        self.assertEqual(set(built.approaches), {"S", "N"})
        self.assertEqual(built.approaches["S"]["n"], 3)
        self.assertEqual(built.approaches["N"]["n"], 2)
        self.assertEqual(built.approaches["S"]["lam"], 0.1)
        self.assertEqual(built.approaches["N"]["lam"], 0.1)
        self.assertEqual(built.green, "S")
        self.assertEqual(built.cycle_green_s, {"S": 30.0, "N": 30.0})

    def test_invalid_or_missing_inputs_fail_loudly(self):
        with self.assertRaises(ValueError):
            compute_preparation(24, 3.6, 8, 2, 5, s=0)
        with self.assertRaises(KeyError):
            self.make_model(travel_times=TravelTimes(self.tt_df.iloc[:2])).step(
                0.0, AmbulanceState("e0000", 0.0)
            )
        with self.assertRaises(ValueError):
            self.make_model().step(0.0, AmbulanceState("not-an-edge", 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
