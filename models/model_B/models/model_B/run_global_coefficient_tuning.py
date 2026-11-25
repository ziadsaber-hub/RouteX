#!/usr/bin/env python3
"""Select one shared Model B coefficient tuple and score it on held-out roads.

Copy this file beside model_b_estimator.py and the existing Model B scripts.
It reads the 600-second raw capture, groups observations into three 200-second
windows, reserves the same road IDs from all three conditions for final testing,
and chooses one alpha/beta/gamma tuple by pooled validation MAE.
"""
from __future__ import annotations

import argparse
import itertools
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import pandas as pd


HERE = Path(__file__).resolve().parent
ROOT_DEFAULT = next((p for p in HERE.parents if (p / "dokki_sumo").is_dir()), None)
N_BINS = 3
WINDOW_SECONDS = 200
CONDITIONS = {"S1": "normal", "S2": "moderate", "S3": "heavy"}
MISSING_PERCENTS = (20, 40, 60)
TEST_ROAD_POOL_PERCENT = 40
ALPHAS = (1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0)
BETAS = (1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0, 1000.0, 10000.0)
GAMMAS = (1e-2, 1e-1, 1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0)
BASE_SEED = 20261125


def resolve_inputs(root: Path) -> tuple[Path, Path]:
    bin_candidates = (
        HERE / "inputs" / "Model_B_edge_bins.csv.xls",
        root / "models" / "model_B" / "sumo" / "inputs" / "Model_B_edge_bins.csv.xls",
    )
    bins = next((p for p in bin_candidates if p.exists()), bin_candidates[0])
    mapping_candidates = (
        root / "dokki_sumo" / "network" / "edge_mapping.csv",
        HERE / "inputs" / "edge_mapping.csv",
    )
    mapping = next((p for p in mapping_candidates if p.exists()), mapping_candidates[0])
    if not bins.exists():
        raise FileNotFoundError("Model_B_edge_bins.csv.xls not found. Checked: " + ", ".join(map(str, bin_candidates)))
    if not mapping.exists():
        raise FileNotFoundError("edge_mapping.csv not found. Checked: " + ", ".join(map(str, mapping_candidates)))
    return bins, mapping


def load_estimator():
    estimator_dir = HERE if (HERE / "model_b_estimator.py").exists() else HERE.parent
    if not (estimator_dir / "model_b_estimator.py").exists():
        raise FileNotFoundError(
            "model_b_estimator.py must be beside this script or in its parent folder."
        )
    sys.path.insert(0, str(estimator_dir))
    from model_b_estimator import ModelB, build_similarity, metrics  # noqa: PLC0415
    return ModelB, build_similarity, metrics


def write_csv(path: Path, rows: list[dict]) -> None:
    pd.DataFrame(rows).to_csv(path, index=False, encoding="utf-8-sig")


def aggregate_condition(raw: pd.DataFrame, scenario_id: str, n_edges: int) -> np.ndarray:
    part = raw.loc[raw["scenario_id"] == scenario_id].copy()
    if part.empty:
        raise ValueError(f"Raw SUMO file contains no rows for {scenario_id}.")
    part["time_bin"] = np.floor(part["edge_enter_s"] / WINDOW_SECONDS).astype(int).clip(0, N_BINS - 1)
    means = part.groupby(["edge_index", "time_bin"])["travel_time_s"].mean()
    y = np.full((n_edges, N_BINS), np.nan, dtype=float)
    for (edge_index, time_bin), value in means.items():
        y[int(edge_index), int(time_bin)] = float(value)
    return y


def weights_for_roads(y: np.ndarray, road_ids: np.ndarray) -> np.ndarray:
    weights = np.zeros_like(y, dtype=float)
    if len(road_ids):
        weights[road_ids, :] = np.isfinite(y[road_ids, :]).astype(float)
    return weights


def observed_cells(y: np.ndarray, roads: np.ndarray) -> np.ndarray:
    mask = np.zeros_like(y, dtype=bool)
    if len(roads):
        mask[roads, :] = np.isfinite(y[roads, :])
    return mask


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", default=str(ROOT_DEFAULT or ""), help="RouteX root containing dokki_sumo/.")
    parser.add_argument("--input-dir", default=str(HERE / "raw_600s"), help="Folder containing Model_B_SUMO_Ground_Truth_Raw.csv.")
    parser.add_argument("--output-dir", default=str(HERE / "global_coefficients_600s_extended_search"), help="Destination for tuning results.")
    parser.add_argument("--repeats", type=int, default=5, help="Repeated random validation and masking trials.")
    parser.add_argument("--overwrite", action="store_true", help="Allow result files in the output folder to be replaced.")
    args = parser.parse_args()

    root_text = args.project_root.strip()
    if root_text:
        root = Path(root_text).expanduser().resolve()
    elif ROOT_DEFAULT is not None:
        root = ROOT_DEFAULT
    else:
        raise FileNotFoundError("Could not find RouteX root; pass --project-root with the folder containing dokki_sumo/.")
    raw_dir = Path(args.input_dir).expanduser()
    output_dir = Path(args.output_dir).expanduser()
    if not raw_dir.is_absolute():
        raw_dir = root / raw_dir
    if not output_dir.is_absolute():
        output_dir = root / output_dir
    raw_file = raw_dir / "Model_B_SUMO_Ground_Truth_Raw.csv"
    if not raw_file.exists():
        raise FileNotFoundError(f"Raw SUMO observations not found: {raw_file}. Run the SUMO capture first.")
    if args.repeats < 1:
        raise ValueError("--repeats must be at least 1.")
    if output_dir.exists() and any(output_dir.iterdir()) and not args.overwrite:
        raise FileExistsError(f"Output folder is not empty: {output_dir}. Use --overwrite to replace these files.")
    output_dir.mkdir(parents=True, exist_ok=True)

    bins_file, mapping_file = resolve_inputs(root)
    estimator_file = HERE if (HERE / "model_b_estimator.py").exists() else HERE.parent
    sys.path.insert(0, str(estimator_file))
    ModelB, build_similarity, metrics = load_estimator()

    bins = pd.read_csv(bins_file, encoding="utf-8-sig")
    unique = bins.sort_values(["edge_index", "time_set_id"]).drop_duplicates("edge_index")
    unique = unique.sort_values("edge_index").reset_index(drop=True)
    n_edges = len(unique)
    if n_edges != 546 or unique["edge_index"].astype(int).tolist() != list(range(n_edges)):
        raise ValueError(f"Expected the ordered 546 Model A edges in {bins_file}.")
    reference = unique["model_a_reference_time_s"].to_numpy(dtype=float)
    edge_ids = unique["edge_id"].astype(str).tolist()
    edges = unique[["edge_index", "u", "v", "key"]].copy().reset_index(drop=True)
    similarity = build_similarity(edges, edge_mapping_path=str(mapping_file))

    raw = pd.read_csv(raw_file, encoding="utf-8-sig")
    raw["edge_index"] = raw["edge_index"].astype(int)
    raw["edge_enter_s"] = raw["edge_enter_s"].astype(float)
    raw["travel_time_s"] = raw["travel_time_s"].astype(float)
    if (raw["travel_time_s"] <= 0).any():
        raise ValueError("Raw observations contain nonpositive travel times.")
    y_by_condition = {
        scenario: aggregate_condition(raw, scenario, n_edges)
        for scenario in CONDITIONS
    }
    roads_by_condition = {
        scenario: np.flatnonzero(np.isfinite(y).any(axis=1))
        for scenario, y in y_by_condition.items()
    }

    # Reserve a common road pool exclusively for final evaluation. It is never
    # used while tuning the global tuple. Each test repeat samples 20% roads
    # from this pool, giving fresh held-out roads without validation leakage.
    common_roads = set.intersection(*(set(ids.tolist()) for ids in roads_by_condition.values()))
    common_roads = np.array(sorted(common_roads), dtype=int)
    if len(common_roads) < 30:
        raise ValueError(f"Only {len(common_roads)} roads are observed in all conditions; need at least 30.")
    outer_rng = np.random.default_rng(BASE_SEED)
    outer_order = outer_rng.permutation(common_roads)
    test_road_pool = np.sort(outer_order[:max(1, int(round(TEST_ROAD_POOL_PERCENT / 100 * len(outer_order))))])
    test_set = set(test_road_pool.tolist())

    candidates = list(itertools.product(ALPHAS, BETAS, GAMMAS))
    validation_scores: dict[tuple[float, float, float], list[dict]] = defaultdict(list)
    validation_details: list[dict] = []

    print(f"Evaluating {len(candidates)} coefficient tuples using pooled validation masks...")
    for scenario_number, scenario in enumerate(CONDITIONS):
        y = y_by_condition[scenario]
        observed = roads_by_condition[scenario]
        dev_roads = np.array([r for r in observed if r not in test_set], dtype=int)
        if len(dev_roads) < 20:
            raise ValueError(f"Too few development roads remain for {scenario} after the shared holdout.")
        for repeat in range(1, args.repeats + 1):
            rng = np.random.default_rng(BASE_SEED + scenario_number * 10000 + repeat)
            permuted = rng.permutation(dev_roads)
            n_val = max(1, int(round(0.20 * len(permuted))))
            val_roads = np.sort(permuted[:n_val])
            train_pool = np.sort(permuted[n_val:])
            for percent in MISSING_PERCENTS:
                hidden_dev_count = max(n_val, int(round(len(dev_roads) * percent / 100)))
                extra_count = min(len(train_pool), hidden_dev_count - n_val)
                extra_hidden = (
                    np.sort(rng.choice(train_pool, size=extra_count, replace=False))
                    if extra_count else np.array([], dtype=int)
                )
                train_roads = np.setdiff1d(train_pool, extra_hidden, assume_unique=True)
                weights = weights_for_roads(y, train_roads)
                val_mask = observed_cells(y, val_roads)
                truth = y[val_mask]
                model = ModelB(similarity, n_edges, N_BINS)
                fold = f"{scenario}_r{repeat}_m{percent}"
                for alpha, beta, gamma in candidates:
                    model.fit(y, weights, reference, alpha=alpha, beta=beta, gamma=gamma)
                    score = metrics(truth, model.estimate_[val_mask])
                    row = {
                        "fold": fold,
                        "scenario_id": scenario,
                        "traffic_condition": CONDITIONS[scenario],
                        "repeat": repeat,
                        "validation_mask_percent": percent,
                        "validation_roads": len(val_roads),
                        "additional_hidden_roads": len(extra_hidden),
                        "alpha": alpha,
                        "beta": beta,
                        "gamma": gamma,
                        "validation_MAE_s": score["MAE"],
                        "validation_RMSE_s": score["RMSE"],
                        "validation_MAPE_percent": score["MAPE"],
                    }
                    validation_details.append(row)
                    validation_scores[(alpha, beta, gamma)].append(row)

    pooled = []
    for tuple_values, fold_rows in validation_scores.items():
        pooled.append({
            "alpha": tuple_values[0],
            "beta": tuple_values[1],
            "gamma": tuple_values[2],
            "validation_folds": len(fold_rows),
            "pooled_validation_MAE_s": float(np.mean([r["validation_MAE_s"] for r in fold_rows])),
            "pooled_validation_RMSE_s": float(np.mean([r["validation_RMSE_s"] for r in fold_rows])),
            "pooled_validation_MAPE_percent": float(np.mean([r["validation_MAPE_percent"] for r in fold_rows])),
            "selection_rule": "minimum mean validation MAE; tie-break by mean RMSE, then MAPE",
        })
    pooled.sort(key=lambda r: (r["pooled_validation_MAE_s"], r["pooled_validation_RMSE_s"], r["pooled_validation_MAPE_percent"]))
    chosen = pooled[0]
    best_tuple = (chosen["alpha"], chosen["beta"], chosen["gamma"])

    test_details: list[dict] = []
    prediction_rows: list[dict] = []
    for scenario_number, scenario in enumerate(CONDITIONS):
        y = y_by_condition[scenario]
        observed = roads_by_condition[scenario]
        n_test_roads = max(1, int(round(0.20 * len(observed))))
        if n_test_roads > len(test_road_pool):
            raise ValueError(
                f"Reserved test pool is too small for {scenario}: need {n_test_roads} roads, "
                f"have {len(test_road_pool)}. Increase TEST_ROAD_POOL_PERCENT."
            )
        for repeat in range(1, args.repeats + 1):
            test_rng = np.random.default_rng(BASE_SEED + 500000 + scenario_number * 10000 + repeat)
            condition_test_roads = np.sort(test_rng.choice(test_road_pool, size=n_test_roads, replace=False))
            for percent in MISSING_PERCENTS:
                # Test roads count toward the requested missing-road percentage;
                # the remaining roads are randomly hidden from the fit.
                target_hidden = max(len(condition_test_roads), int(round(len(observed) * percent / 100)))
                additional_count = min(len(observed) - len(condition_test_roads), target_hidden - len(condition_test_roads))
                rng = np.random.default_rng(BASE_SEED + 600000 + scenario_number * 10000 + percent * 10 + repeat)
                eligible_extra = np.array([r for r in observed if r not in set(condition_test_roads.tolist())], dtype=int)
                extra_hidden = (
                    np.sort(rng.choice(eligible_extra, size=additional_count, replace=False))
                    if additional_count else np.array([], dtype=int)
                )
                hidden_roads = np.union1d(condition_test_roads, extra_hidden)
                train_roads = np.setdiff1d(observed, hidden_roads, assume_unique=False)
                weights = weights_for_roads(y, train_roads)
                test_mask = observed_cells(y, condition_test_roads)
                hidden_mask = observed_cells(y, hidden_roads)
                model = ModelB(similarity, n_edges, N_BINS)
                model.fit(y, weights, reference, alpha=best_tuple[0], beta=best_tuple[1], gamma=best_tuple[2])
                truth = y[test_mask]
                prediction = model.estimate_[test_mask]
                score = metrics(truth, prediction)
                test_details.append({
                    "scenario_id": scenario,
                    "traffic_condition": CONDITIONS[scenario],
                    "missing_roads_percent_target": percent,
                    "actual_missing_roads_percent": 100 * len(hidden_roads) / len(observed),
                    "repeat": repeat,
                    "observed_roads": len(observed),
                    "test_roads_scored": len(condition_test_roads),
                    "test_edge_time_cells_scored": int(test_mask.sum()),
                    "additional_hidden_roads": len(extra_hidden),
                    "training_roads": len(train_roads),
                    "alpha": best_tuple[0],
                    "beta": best_tuple[1],
                    "gamma": best_tuple[2],
                    "test_MAE_s": score["MAE"],
                    "test_RMSE_s": score["RMSE"],
                    "test_MAPE_percent": score["MAPE"],
                })
                hidden_cells = np.argwhere(hidden_mask)
                for edge_index, time_bin in hidden_cells:
                    actual = y[edge_index, time_bin]
                    estimate = model.estimate_[edge_index, time_bin]
                    prediction_rows.append({
                        "scenario_id": scenario,
                        "traffic_condition": CONDITIONS[scenario],
                        "missing_roads_percent_target": percent,
                        "repeat": repeat,
                        "edge_index": int(edge_index),
                        "edge_id": edge_ids[int(edge_index)],
                        "time_bin": int(time_bin) + 1,
                        "window_start_s": int(time_bin) * WINDOW_SECONDS,
                        "window_end_s": (int(time_bin) + 1) * WINDOW_SECONDS,
                        "actual_travel_time_s": float(actual),
                        "estimated_travel_time_s": float(estimate),
                        "absolute_error_s": float(abs(estimate - actual)),
                        "absolute_percentage_error": float(abs(estimate - actual) / actual * 100),
                        "included_in_heldout_test_metrics": bool(test_mask[edge_index, time_bin]),
                    })

    test_frame = pd.DataFrame(test_details)
    summary = (test_frame.groupby(["scenario_id", "traffic_condition", "missing_roads_percent_target"], as_index=False)
               .agg(repeats=("repeat", "count"), actual_missing_roads_percent=("actual_missing_roads_percent", "mean"),
                    mean_test_MAE_s=("test_MAE_s", "mean"), sd_test_MAE_s=("test_MAE_s", "std"),
                    mean_test_RMSE_s=("test_RMSE_s", "mean"), sd_test_RMSE_s=("test_RMSE_s", "std"),
                    mean_test_MAPE_percent=("test_MAPE_percent", "mean"), sd_test_MAPE_percent=("test_MAPE_percent", "std")))

    write_csv(output_dir / "Model_B_Global_Coefficient_Validation.csv", pooled)
    write_csv(output_dir / "Model_B_Global_Coefficient_Validation_Detail.csv", validation_details)
    write_csv(output_dir / "Model_B_Global_Coefficient_Test_Detail.csv", test_details)
    write_csv(output_dir / "Model_B_Global_Coefficient_Predictions.csv", prediction_rows)
    summary.to_csv(output_dir / "Model_B_Global_Coefficient_Test_Summary.csv", index=False, encoding="utf-8-sig")
    selection = {
        "selected_alpha": best_tuple[0],
        "selected_beta": best_tuple[1],
        "selected_gamma": best_tuple[2],
        "pooled_validation_MAE_s": chosen["pooled_validation_MAE_s"],
        "pooled_validation_RMSE_s": chosen["pooled_validation_RMSE_s"],
        "pooled_validation_MAPE_percent": chosen["pooled_validation_MAPE_percent"],
        "validation_fold_count": chosen["validation_folds"],
        "reserved_test_road_pool_count_common_to_all_conditions": len(test_road_pool),
        "reserved_test_road_pool_percent_of_common_observed_roads": TEST_ROAD_POOL_PERCENT,
        "test_mask_repeats_per_condition_and_missing_level": args.repeats,
        "search_grid": {"alpha": ALPHAS, "beta": BETAS, "gamma": GAMMAS},
        "time_windows": ["0-200 seconds", "200-400 seconds", "400-600 seconds"],
        "masking_unit": "whole road across all available windows",
        "spatial_similarity": "uses the unchanged rule from model_b_estimator.py",
        "note": "This is one tuple tuned for the current Dokki data and searched grid; it is not proof of transfer to every map.",
    }
    (output_dir / "Model_B_Global_Coefficient_Selected.json").write_text(json.dumps(selection, indent=2) + "\n", encoding="utf-8")

    print(f"Shared tuple: alpha={best_tuple[0]}, beta={best_tuple[1]}, gamma={best_tuple[2]}")
    print("Chosen by pooled validation MAE; final metrics below use held-out roads.")
    print(summary.to_string(index=False))
    print(f"Results saved to: {output_dir}")


if __name__ == "__main__":
    main()
