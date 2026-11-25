#!/usr/bin/env python3
"""Run Model B's traffic-only SUMO conditions and retain raw edge traversals.

The capture stays event-level (one row per vehicle-edge traversal). SUMO time
windows are intentionally not assigned here because their boundaries have not
yet been approved for this experiment.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = next((p for p in HERE.parents if (p / "dokki_sumo").is_dir()), HERE.parents[2])
LOCAL_INPUTS = HERE / "inputs"
NETWORK_DIR = ROOT / "dokki_sumo" / "network"
DEMAND_DIR = ROOT / "dokki_sumo" / "experiments" / "scenario_0" / "demand"
RAW_DEFAULT = HERE / "raw_600s"
MODEL_B_EDGE_BINS_CANDIDATES = (
    LOCAL_INPUTS / "Model_B_edge_bins.csv.xls",
    ROOT / "models" / "model_B" / "sumo" / "inputs" / "Model_B_edge_bins.csv.xls",
)
MODEL_B_EDGE_BINS = next(
    (path for path in MODEL_B_EDGE_BINS_CANDIDATES if path.exists()),
    MODEL_B_EDGE_BINS_CANDIDATES[0],
)
NETWORK_FILE = NETWORK_DIR / "dokki.net.xml"
EDGE_MAPPING_FILE = NETWORK_DIR / "edge_mapping.csv"

CONDITIONS = {
    "S1": ("normal", "normal.rou.xml", 200),
    "S2": ("moderate", "moderate.rou.xml", 400),
    "S3": ("heavy", "heavy.rou.xml", 600),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def locate_sumo() -> str:
    home = os.environ.get("SUMO_HOME")
    if home:
        exe = Path(home) / "bin" / ("sumo.exe" if os.name == "nt" else "sumo")
        if exe.exists():
            return str(exe)
    exe = shutil.which("sumo")
    if exe:
        return exe
    raise FileNotFoundError("SUMO executable not found; set SUMO_HOME or add sumo to PATH.")


def model_a_order() -> tuple[list[dict], dict[tuple[str, str, str], dict]]:
    rows = read_csv(MODEL_B_EDGE_BINS)
    by_index: dict[str, dict] = {}
    for row in rows:
        edge_index = row["edge_index"]
        existing = by_index.get(edge_index)
        item = {
            "edge_index": int(edge_index),
            "model_a_edge_id": row["edge_id"],
            "u": row["u"],
            "v": row["v"],
            "key": row["key"],
        }
        if existing and existing != item:
            raise ValueError(f"Conflicting Model A rows for edge_index {edge_index}.")
        by_index[edge_index] = item

    ordered = [by_index[str(i)] for i in range(len(by_index))]
    if len(ordered) != 546 or [r["edge_index"] for r in ordered] != list(range(546)):
        raise ValueError("Model_B_edge_bins must define the ordered 546 Model A edges.")
    by_tuple = {(r["u"], r["v"], r["key"]): r for r in ordered}
    if len(by_tuple) != 546:
        raise ValueError("Model A edge endpoint/key tuples are not unique.")
    return ordered, by_tuple


def build_edge_mapping(network_path: Path, edge_mapping_path: Path):
    ordered, model_a_by_tuple = model_a_order()
    edge_rows = read_csv(edge_mapping_path)
    if len(edge_rows) != 546:
        raise ValueError(f"Expected 546 edge-mapping rows, found {len(edge_rows)}.")

    network = ET.parse(network_path).getroot()
    network_edges = {
        edge.get("id")
        for edge in network.findall(".//edge")
        if edge.get("id") and not edge.get("id", "").startswith(":")
    }
    if len(network_edges) != 551:
        raise ValueError(f"Expected 551 external SUMO edges, found {len(network_edges)}.")

    actual_to_model: dict[str, dict] = {}
    map_rows = []
    source_ids = set()
    for row in edge_rows:
        key = (row["u"], row["v"], row["key"])
        model = model_a_by_tuple.get(key)
        if model is None:
            raise ValueError(f"SUMO mapping tuple {key} is absent from Model_B_edge_bins.")
        source_id = row["edge_id"]
        if source_id in source_ids:
            raise ValueError(f"Duplicate SUMO source edge ID {source_id}.")
        source_ids.add(source_id)

        if source_id in network_edges:
            sumo_id = source_id
            mapping_rule = "direct_edge_id"
        else:
            candidates = sorted(
                edge_id for edge_id in network_edges
                if edge_id.startswith(source_id + "__to_hospital_")
            )
            if len(candidates) != 1:
                raise ValueError(
                    f"Expected one hospital-bound replacement for {source_id}; found {candidates}."
                )
            sumo_id = candidates[0]
            mapping_rule = "hospital_bound_replacement"

        if sumo_id in actual_to_model:
            raise ValueError(f"SUMO edge {sumo_id} maps to more than one Model A edge.")
        actual_to_model[sumo_id] = model
        map_rows.append({
            "edge_index": model["edge_index"],
            "model_a_edge_id": model["model_a_edge_id"],
            "sumo_edge_id": sumo_id,
            "source_edge_id": source_id,
            "u": model["u"],
            "v": model["v"],
            "key": model["key"],
            "mapping_rule": mapping_rule,
        })

    if len(actual_to_model) != 546:
        raise ValueError("SUMO-to-Model A mapping does not cover all 546 Model A edges.")

    excluded = sorted(network_edges - actual_to_model.keys())
    if len(excluded) != 5 or not all(edge.startswith("hospital_") for edge in excluded):
        raise ValueError(f"Unexpected unmapped network edges; review before proceeding: {excluded}")

    return ordered, actual_to_model, sorted(map_rows, key=lambda r: r["edge_index"]), excluded


def check_demand(route_path: Path, expected_count: int, network_edges: set[str]) -> None:
    root = ET.parse(route_path).getroot()
    vehicles = root.findall("vehicle")
    if len(vehicles) != expected_count:
        raise ValueError(f"{route_path.name}: expected {expected_count} vehicles, found {len(vehicles)}.")
    bad = set()
    for vehicle in vehicles:
        route = vehicle.find("route")
        if route is None:
            continue
        bad.update(edge for edge in route.get("edges", "").split() if edge not in network_edges)
    if bad:
        raise ValueError(f"{route_path.name} refers to missing network edges: {sorted(bad)}")


def parse_vehroute(path: Path, scenario_id: str, condition: str, seed: int,
                   actual_to_model: dict[str, dict]) -> tuple[list[dict], dict]:
    root = ET.parse(path).getroot()
    vehicles = root.findall("vehicle")
    rows = []
    completed = 0
    incomplete = 0
    bad_time = 0
    per_edge = Counter()

    for vehicle in vehicles:
        depart = float(vehicle.get("depart", "0"))
        arrival_value = vehicle.get("arrival")
        arrival = float(arrival_value) if arrival_value not in (None, "") else None
        completed += arrival is not None
        incomplete += arrival is None
        route = vehicle.find("route")
        if route is None:
            continue
        edges = route.get("edges", "").split()
        exit_times_text = route.get("exitTimes", "").split()
        if len(exit_times_text) > len(edges):
            raise ValueError(f"{path.name}: {vehicle.get('id')} has more exit times than edges.")
        exit_times = [float(value) for value in exit_times_text]
        previous_exit = depart

        for position, (sumo_edge_id, exit_time) in enumerate(zip(edges, exit_times)):
            # SUMO writes -1 for planned edges not exited when the 600s cutoff
            # interrupts a vehicle. They are not observed traversals.
            if exit_time < 0:
                if arrival is not None:
                    raise ValueError(
                        f"Completed vehicle {vehicle.get('id')} has a negative edge exit time."
                    )
                break
            model = actual_to_model.get(sumo_edge_id)
            if model is not None:
                travel_time = exit_time - previous_exit
                if travel_time < 0:
                    raise ValueError(
                        f"Negative travel time for {vehicle.get('id')} on {sumo_edge_id}."
                    )
                if travel_time == 0:
                    bad_time += 1
                rows.append({
                    "scenario_id": scenario_id,
                    "traffic_condition": condition,
                    "seed": seed,
                    "step_length_s": 1,
                    "vehicle_id": vehicle.get("id", ""),
                    "vehicle_depart_s": depart,
                    "vehicle_arrival_s": arrival,
                    "route_edge_position": position,
                    "edge_index": model["edge_index"],
                    "model_a_edge_id": model["model_a_edge_id"],
                    "sumo_edge_id": sumo_edge_id,
                    "edge_enter_s": previous_exit,
                    "edge_exit_s": exit_time,
                    "travel_time_s": travel_time,
                })
                per_edge[model["edge_index"]] += 1
            previous_exit = exit_time

    return rows, {
        "scenario_id": scenario_id,
        "traffic_condition": condition,
        "vehicles_in_output": len(vehicles),
        "vehicles_completed": completed,
        "vehicles_unfinished_at_600s": incomplete,
        "mapped_edge_traversals": len(rows),
        "mapped_model_a_edges_observed": len(per_edge),
        "zero_second_traversals_at_1s_resolution": bad_time,
    }


def run(args) -> None:
    sumo = locate_sumo()
    raw_dir = Path(args.output_dir).expanduser()
    if not raw_dir.is_absolute():
        raw_dir = ROOT / raw_dir
    net_path = NETWORK_FILE
    edge_mapping_path = EDGE_MAPPING_FILE
    _, actual_to_model, mapping_rows, excluded = build_edge_mapping(net_path, edge_mapping_path)
    network_edges = {
        edge.get("id")
        for edge in ET.parse(net_path).getroot().findall(".//edge")
        if edge.get("id") and not edge.get("id", "").startswith(":")
    }

    traffic_lights = ROOT / "dokki_sumo" / "simulation" / "traffic_lights.add.xml"
    hospitals = ROOT / "dokki_sumo" / "simulation" / "hospitals.add.xml"
    for path in (traffic_lights, hospitals):
        if not path.exists():
            raise FileNotFoundError(f"Scenario 0 addition file not found: {path}")
    additional_files = f"{traffic_lights},{hospitals}"

    raw_dir.mkdir(parents=True, exist_ok=True)
    expected_outputs = [raw_dir / f"{condition}_vehroute.xml" for condition in args.conditions]
    if not args.overwrite:
        present = [str(path) for path in expected_outputs if path.exists()]
        if present:
            raise FileExistsError(
                "Outputs already exist; use a fresh output folder or pass --overwrite: "
                + ", ".join(present)
            )

    write_csv(
        raw_dir / "Model_B_SUMO_Edge_Mapping.csv",
        mapping_rows,
        ["edge_index", "model_a_edge_id", "sumo_edge_id", "source_edge_id", "u", "v", "key", "mapping_rule"],
    )
    write_csv(
        raw_dir / "Model_B_SUMO_Excluded_Edges.csv",
        [{"sumo_edge_id": edge, "reason": "Hospital connector outside the 546-edge Model A edge universe"}
         for edge in excluded],
        ["sumo_edge_id", "reason"],
    )

    all_rows = []
    summaries = []
    for scenario_id in args.conditions:
        condition, route_name, expected_count = CONDITIONS[scenario_id]
        route_path = DEMAND_DIR / route_name
        check_demand(route_path, expected_count, network_edges)
        vehroute_path = raw_dir / f"{scenario_id}_vehroute.xml"
        log_path = raw_dir / f"{scenario_id}_sumo.log"
        command = [
            sumo,
            "--net-file", str(net_path),
            "--route-files", str(route_path),
            "--additional-files", additional_files,
            "--begin", "0",
            "--end", "600",
            "--seed", str(args.seed),
            "--step-length", "1",
            "--time-to-teleport", "-1",
            "--collision.action", "warn",
            "--collision.check-junctions", "true",
            "--no-step-log", "true",
            "--vehroute-output", str(vehroute_path),
            "--vehroute-output.exit-times",
            "--vehroute-output.internal",
            "--vehroute-output.write-unfinished",
        ]
        result = subprocess.run(command, text=True, capture_output=True)
        log_path.write_text(
            f"COMMAND: {subprocess.list2cmdline(command)}\n"
            f"EXIT_CODE: {result.returncode}\n\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
            encoding="utf-8",
        )
        if result.returncode != 0:
            raise RuntimeError(f"SUMO {scenario_id} failed; see {log_path}.")

        rows, summary = parse_vehroute(
            vehroute_path, scenario_id, condition, args.seed, actual_to_model
        )
        # At a hard 600s cutoff, vehicles that never entered the network
        # (for example, because their departure was blocked) have no vehroute
        # record. Preserve the fixed demand count and report this separately.
        summary["vehicles_expected_in_demand"] = expected_count
        summary["seed"] = args.seed
        summary["vehicles_without_output_record_at_600s"] = (
            expected_count - summary["vehicles_in_output"]
        )
        all_rows.extend(rows)
        summaries.append(summary)
        print(
            f"{scenario_id} {condition}: {summary['vehicles_in_output']}/"
            f"{expected_count} demand vehicles have output records; "
            f"{summary['vehicles_completed']} completed; "
            f"{summary['vehicles_unfinished_at_600s']} unfinished at 600s; "
            f"{summary['mapped_edge_traversals']} "
            f"mapped edge traversals across {summary['mapped_model_a_edges_observed']}/546 "
            "Model A edges."
        )

    traversal_columns = [
        "scenario_id", "traffic_condition", "seed", "step_length_s", "vehicle_id",
        "vehicle_depart_s", "vehicle_arrival_s", "route_edge_position", "edge_index",
        "model_a_edge_id", "sumo_edge_id", "edge_enter_s", "edge_exit_s", "travel_time_s",
    ]
    write_csv(raw_dir / "Model_B_SUMO_Ground_Truth_Raw.csv", all_rows, traversal_columns)
    write_csv(
        raw_dir / "Model_B_SUMO_Run_Summary.csv",
        summaries,
        ["scenario_id", "traffic_condition", "seed", "vehicles_expected_in_demand",
         "vehicles_in_output", "vehicles_without_output_record_at_600s", "vehicles_completed",
         "vehicles_unfinished_at_600s", "mapped_edge_traversals",
         "mapped_model_a_edges_observed", "zero_second_traversals_at_1s_resolution"],
    )

    traversal_counts = Counter((row["scenario_id"], row["edge_index"]) for row in all_rows)
    coverage_rows = []
    ordered, _ = model_a_order()
    for scenario_id in args.conditions:
        for edge in ordered:
            count = traversal_counts[(scenario_id, edge["edge_index"])]
            coverage_rows.append({
                "scenario_id": scenario_id,
                "edge_index": edge["edge_index"],
                "model_a_edge_id": edge["model_a_edge_id"],
                "traversal_count": count,
                "has_ground_truth_observation": count > 0,
            })
    write_csv(
        raw_dir / "Model_B_SUMO_Edge_Coverage.csv",
        coverage_rows,
        ["scenario_id", "edge_index", "model_a_edge_id", "traversal_count", "has_ground_truth_observation"],
    )

    manifest = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "sumo_version": subprocess.run([sumo, "--version"], text=True, capture_output=True).stdout.splitlines()[0],
        "conditions": args.conditions,
        "expected_vehicles": {key: CONDITIONS[key][2] for key in args.conditions},
        "seed": args.seed,
        "step_length_s": 1,
        "simulation_end_s": 600,
        "time_windows": "not aggregated; per-traversal entry/exit times retained pending approved SUMO window boundaries",
        "mapping": "546 Model A edges mapped; five SUMO hospital connectors excluded. Five hospital-bound replacement edge IDs are normalized to their Model A source IDs.",
        "inputs_sha256": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in (
                NETWORK_FILE, EDGE_MAPPING_FILE, MODEL_B_EDGE_BINS,
                *(DEMAND_DIR / route for _, route, _ in CONDITIONS.values()),
            )
        },
        "scenario_0_additional_files": [str(traffic_lights), str(hospitals)],
        "not_used": "The supplied scenario_0.sumocfg ambulance.add.xml is omitted because this is a traffic-only Model B run. Each supplied demand file is run separately to the requested 600s end.",
    }
    (raw_dir / "Model_B_SUMO_Run_Manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"Raw Model B SUMO ground truth written to: {raw_dir}")
    print("No SUMO time windows were assumed; the raw file retains edge entry/exit times.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--conditions", default="S1,S2,S3",
        help="Comma-separated conditions selected from S1,S2,S3 (default: all three).",
    )
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Replace raw SUMO vehicle-route XML outputs for selected conditions.",
    )
    parser.add_argument(
        "--seed", type=int, default=1,
        help="SUMO random seed (default: 1).",
    )
    parser.add_argument(
        "--output-dir", default=str(RAW_DEFAULT),
        help=f"Output folder (default: {RAW_DEFAULT}).",
    )
    args = parser.parse_args()
    args.conditions = [item.strip().upper() for item in args.conditions.split(",") if item.strip()]
    if not args.conditions or set(args.conditions) - set(CONDITIONS):
        parser.error("--conditions must contain one or more of S1,S2,S3.")
    if len(args.conditions) != len(set(args.conditions)):
        parser.error("--conditions contains a duplicate scenario.")
    run(args)


if __name__ == "__main__":
    main()
