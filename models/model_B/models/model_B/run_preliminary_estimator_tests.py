#!/usr/bin/env python3
"""Run road-level masked validation across SUMO conditions, seeds, and windows."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ROOT = next((p for p in HERE.parents if (p / "dokki_sumo").is_dir()), HERE.parents[2])
LOCAL_INPUTS = HERE / "inputs"
BIN_CANDIDATES = (
    LOCAL_INPUTS / "Model_B_edge_bins.csv.xls",
    ROOT / "models" / "model_B" / "sumo" / "inputs" / "Model_B_edge_bins.csv.xls",
)
MODEL_B_EDGE_BINS = next((p for p in BIN_CANDIDATES if p.exists()), BIN_CANDIDATES[0])
EDGE_MAPPING_FILE = ROOT / "dokki_sumo" / "network" / "edge_mapping.csv"
ESTIMATOR_DIR = HERE if (HERE / "model_b_estimator.py").exists() else HERE.parent
sys.path.insert(0, str(ESTIMATOR_DIR))

from model_b_estimator import ModelB, build_similarity, metrics  # noqa: E402

RAW_DEFAULT = HERE / "raw_600s_multi_seed"
OUTPUT_DEFAULT = HERE / "required_outputs_road_level"
N_EDGES = 546
CONDITIONS = {"S1": "normal", "S2": "moderate", "S3": "heavy"}
SIMULATION_SEEDS = (1, 2, 3)
WINDOW_SECONDS = (100, 200, 300)
MISSING_PERCENTS = (20, 40, 60)
REPEATS = 5
# A compact search grid for this validation run; values are selected using
# road-level validation data and the estimator source file is left untouched.
ALPHAS = (1.0, 10.0)
BETAS = (0.1, 1.0, 10.0)
GAMMAS = (0.1, 1.0, 10.0)
BASE_SEED = 20261125


def write_csv(path: Path, rows: list[dict], columns: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if columns is None:
        columns = list(rows[0]) if rows else []
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def load_edges_and_reference() -> tuple[pd.DataFrame, np.ndarray, list[str]]:
    bins = pd.read_csv(MODEL_B_EDGE_BINS, encoding="utf-8-sig")
    unique = (bins.sort_values(["edge_index", "time_set_id"])
              .drop_duplicates("edge_index").sort_values("edge_index").reset_index(drop=True))
    if len(unique) != N_EDGES or unique["edge_index"].astype(int).tolist() != list(range(N_EDGES)):
        raise ValueError(f"Expected {N_EDGES} ordered Model A road edges in {MODEL_B_EDGE_BINS}.")
    ref_counts = bins.groupby("edge_index")["model_a_reference_time_s"].nunique()
    if len(ref_counts) != N_EDGES or (ref_counts != 1).any():
        raise ValueError("Each road must have one consistent free-flow reference time.")
    edges = unique[["edge_index", "u", "v", "key"]].copy().reset_index(drop=True)
    reference = unique["model_a_reference_time_s"].to_numpy(dtype=float)
    edge_ids = unique["edge_id"].astype(str).tolist()
    return edges, reference, edge_ids


def tune_coefficients(similarity: pd.DataFrame, y: np.ndarray, reference: np.ndarray,
                      train_roads: np.ndarray, validation_roads: np.ndarray):
    train_weights = np.zeros_like(y, dtype=float)
    train_weights[train_roads, :] = np.isfinite(y[train_roads, :]).astype(float)
    validation_mask = np.zeros_like(y, dtype=bool)
    validation_mask[validation_roads, :] = np.isfinite(y[validation_roads, :])
    truth = y[validation_mask]
    if not len(truth):
        raise ValueError("Validation split has no observed road-time cells.")

    model = ModelB(similarity, N_EDGES, y.shape[1])
    candidates = []
    for alpha in ALPHAS:
        for beta in BETAS:
            for gamma in GAMMAS:
                model.fit(y, train_weights, reference, alpha=alpha, beta=beta, gamma=gamma)
                score = metrics(truth, model.estimate_[validation_mask])
                candidates.append((score["MAE"], score["RMSE"], score["MAPE"], alpha, beta, gamma))
    best = min(candidates)
    return best[3], best[4], best[5], best[0], best[1], best[2]


def run(args) -> None:
    raw_root = Path(args.input_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    if not raw_root.is_absolute():
        raw_root = ROOT / raw_root
    if not output_dir.is_absolute():
        output_dir = ROOT / output_dir
    if not MODEL_B_EDGE_BINS.exists():
        raise FileNotFoundError(f"Model B edge-bin input not found. Checked: {BIN_CANDIDATES}")
    if not EDGE_MAPPING_FILE.exists():
        raise FileNotFoundError(f"Edge mapping file not found: {EDGE_MAPPING_FILE}")

    edges, reference, edge_ids = load_edges_and_reference()
    similarity = build_similarity(edges, edge_mapping_path=str(EDGE_MAPPING_FILE))
    if (similarity["s"] < 0).any():
        raise ValueError("Spatial similarity weights must be nonnegative.")

    experiment_rows: list[dict] = []
    parameter_rows: list[dict] = []
    coverage_rows: list[dict] = []
    prediction_rows: list[dict] = []

    for simulation_seed in SIMULATION_SEEDS:
        seed_raw = raw_root / f"seed_{simulation_seed}" / "Model_B_SUMO_Ground_Truth_Raw.csv"
        if not seed_raw.exists():
            raise FileNotFoundError(f"Missing SUMO capture for seed {simulation_seed}: {seed_raw}")
        raw = pd.read_csv(seed_raw, encoding="utf-8-sig")
        raw["edge_index"] = raw["edge_index"].astype(int)
        raw["travel_time_s"] = raw["travel_time_s"].astype(float)
        if (raw["travel_time_s"] <= 0).any():
            raise ValueError(f"Nonpositive travel-time observations found in {seed_raw}.")

        for scenario_id, condition in CONDITIONS.items():
            scenario = raw.loc[raw["scenario_id"] == scenario_id]
            if scenario.empty:
                raise ValueError(f"{seed_raw} has no data for {scenario_id}.")
            for window_seconds in WINDOW_SECONDS:
                subset = scenario.copy()
                subset["time_bin"] = np.floor(subset["edge_enter_s"] / window_seconds).astype(int)
                n_bins = int(np.ceil(600 / window_seconds))
                subset["time_bin"] = subset["time_bin"].clip(upper=n_bins - 1)
                grouped = subset.groupby(["edge_index", "time_bin"])["travel_time_s"]
                means, counts = grouped.mean(), grouped.size()
                y = np.full((N_EDGES, n_bins), np.nan, dtype=float)
                for (edge_index, time_bin), value in means.items():
                    y[int(edge_index), int(time_bin)] = float(value)
                observed_roads = np.flatnonzero(np.isfinite(y).any(axis=1))
                if len(observed_roads) < 30:
                    raise ValueError(f"Too few observed roads for {scenario_id}, seed {simulation_seed}, {window_seconds}s windows.")

                for road in range(N_EDGES):
                    for time_bin in range(n_bins):
                        coverage_rows.append({
                            "scenario_id": scenario_id,
                            "traffic_condition": condition,
                            "simulation_seed": simulation_seed,
                            "window_seconds": window_seconds,
                            "time_bin": time_bin + 1,
                            "window_start_s": time_bin * window_seconds,
                            "window_end_s": min((time_bin + 1) * window_seconds, 600),
                            "edge_index": road,
                            "model_a_edge_id": edge_ids[road],
                            "traversal_count": int(counts.get((road, time_bin), 0)),
                            "ground_truth_mean_travel_time_s": (
                                float(means.loc[(road, time_bin)]) if (road, time_bin) in means.index else ""
                            ),
                            "has_ground_truth": bool(np.isfinite(y[road, time_bin])),
                        })

                for repeat in range(1, REPEATS + 1):
                    for missing_percent in MISSING_PERCENTS:
                        rng_seed = (BASE_SEED + simulation_seed * 100000 +
                                    int(scenario_id[-1]) * 10000 + window_seconds * 10 +
                                    repeat * 100 + missing_percent)
                        rng = np.random.default_rng(rng_seed)
                        n_test = max(1, int(round(len(observed_roads) * missing_percent / 100)))
                        test_roads = np.sort(rng.choice(observed_roads, size=n_test, replace=False))
                        visible_roads = np.setdiff1d(observed_roads, test_roads)
                        n_validation = max(1, int(round(len(visible_roads) * 0.20)))
                        validation_roads = np.sort(rng.choice(visible_roads, size=n_validation, replace=False))
                        train_roads = np.setdiff1d(visible_roads, validation_roads)

                        alpha, beta, gamma, val_mae, val_rmse, val_mape = tune_coefficients(
                            similarity, y, reference, train_roads, validation_roads
                        )
                        test_mask = np.zeros_like(y, dtype=bool)
                        test_mask[test_roads, :] = np.isfinite(y[test_roads, :])
                        truth = y[test_mask]
                        weights = np.zeros_like(y, dtype=float)
                        weights[visible_roads, :] = np.isfinite(y[visible_roads, :]).astype(float)
                        fitted = ModelB(similarity, N_EDGES, n_bins).fit(
                            y, weights, reference, alpha=alpha, beta=beta, gamma=gamma
                        )
                        predicted = fitted.estimate_[test_mask]
                        scores = metrics(truth, predicted)

                        shared = {
                            "scenario_id": scenario_id,
                            "traffic_condition": condition,
                            "simulation_seed": simulation_seed,
                            "window_seconds": window_seconds,
                            "repeat": repeat,
                            "mask_random_seed": rng_seed,
                            "missing_roads_percent": missing_percent,
                            "observed_roads": len(observed_roads),
                            "masked_roads": len(test_roads),
                            "validation_roads": len(validation_roads),
                            "training_roads": len(train_roads),
                            "scored_road_time_cells": int(test_mask.sum()),
                            "selected_alpha": alpha,
                            "selected_beta": beta,
                            "selected_gamma": gamma,
                            "validation_MAE_s": val_mae,
                            "validation_RMSE_s": val_rmse,
                            "validation_MAPE_percent": val_mape,
                        }
                        parameter_rows.append(dict(shared))
                        experiment_rows.append({
                            **shared,
                            "test_MAE_s": scores["MAE"],
                            "test_RMSE_s": scores["RMSE"],
                            "test_MAPE_percent": scores["MAPE"],
                            "spatial_rule_status": "provisional; requires team approval",
                        })
                        for road in test_roads:
                            for time_bin in np.flatnonzero(np.isfinite(y[road, :])):
                                prediction_rows.append({
                                    **shared,
                                    "time_bin": int(time_bin + 1),
                                    "window_start_s": int(time_bin * window_seconds),
                                    "window_end_s": int(min((time_bin + 1) * window_seconds, 600)),
                                    "edge_index": int(road),
                                    "model_a_edge_id": edge_ids[road],
                                    "actual_travel_time_s": float(y[road, time_bin]),
                                    "estimated_travel_time_s": float(fitted.estimate_[road, time_bin]),
                                    "absolute_error_s": float(abs(fitted.estimate_[road, time_bin] - y[road, time_bin])),
                                })

    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "Model_B_SUMO_Edge_Window_Ground_Truth.csv", coverage_rows)
    write_csv(output_dir / "Model_B_SUMO_Masked_Predictions.csv", prediction_rows)
    write_csv(output_dir / "Model_B_SUMO_Hyperparameter_Selection.csv", parameter_rows)
    write_csv(output_dir / "Model_B_SUMO_Masked_Test_Experiments.csv", experiment_rows)
    summary = (pd.DataFrame(experiment_rows)
               .groupby(["scenario_id", "traffic_condition", "simulation_seed", "window_seconds", "missing_roads_percent"], as_index=False)
               .agg(
                   repeats=("repeat", "count"),
                   mean_selected_alpha=("selected_alpha", "mean"),
                   mean_selected_beta=("selected_beta", "mean"),
                   mean_selected_gamma=("selected_gamma", "mean"),
                   mean_MAE_s=("test_MAE_s", "mean"),
                   sd_MAE_s=("test_MAE_s", "std"),
                   mean_RMSE_s=("test_RMSE_s", "mean"),
                   sd_RMSE_s=("test_RMSE_s", "std"),
                   mean_MAPE_percent=("test_MAPE_percent", "mean"),
                   sd_MAPE_percent=("test_MAPE_percent", "std"),
               ))
    summary.to_csv(output_dir / "Model_B_SUMO_Masked_Test_Summary.csv", index=False, encoding="utf-8-sig")
    write_csv(output_dir / "Model_B_Similarity_Provisional.csv", similarity.to_dict("records"))
    manifest = {
        "status": "road-level masked validation; preliminary pending approval of spatial rule and accuracy threshold",
        "simulation_seeds": list(SIMULATION_SEEDS),
        "traffic_conditions": CONDITIONS,
        "simulation_end_s": 600,
        "window_sizes_seconds": list(WINDOW_SECONDS),
        "mask_unit": "whole observed roads; all available windows for a selected road are hidden together",
        "missing_road_percentages": list(MISSING_PERCENTS),
        "repeats_per_configuration": REPEATS,
        "validation_split": "20% of visible roads after test roads are selected; validation roads are included in final fit but never overlap the test roads",
        "coefficient_grids": {"alpha": ALPHAS, "beta": BETAS, "gamma": GAMMAS},
        "selection_rule": "minimum validation MAE, then RMSE, then MAPE",
        "accuracy_failure_threshold": "not specified in requirements; metrics are reported for team interpretation",
        "spatial_similarity": "pasted estimator rule is provisional pending team approval",
    }
    (output_dir / "Model_B_SUMO_Test_Manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Road-level masked validation complete. Outputs: {output_dir}")
    print(f"Configurations: {len(experiment_rows)}; prediction rows: {len(prediction_rows)}")
    print("Masking holds out all observed time-window values for selected roads.")
    print("No pass/fail accuracy cutoff was specified; inspect the summary across mask levels.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", default=str(RAW_DEFAULT))
    parser.add_argument("--output-dir", default=str(OUTPUT_DEFAULT))
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
