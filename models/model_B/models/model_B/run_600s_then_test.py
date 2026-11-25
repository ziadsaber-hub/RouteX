#!/usr/bin/env python3
"""Capture S1-S3 at multiple SUMO seeds, then run road-level masked validation."""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RAW_DIR = HERE / "raw_600s_multi_seed"
TEST_DIR = HERE / "required_outputs_road_level"
SIMULATION_SEEDS = (1, 2, 3)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Replace the existing 600s SUMO capture and preliminary test outputs.",
    )
    args = parser.parse_args()

    existing = [path for path in (RAW_DIR, TEST_DIR) if path.exists() and any(path.iterdir())]
    if existing and not args.overwrite:
        raise FileExistsError(
            "600s outputs already exist; use --overwrite to replace them: "
            + ", ".join(str(path) for path in existing)
        )

    capture = HERE / "run_sumo_ground_truth.py"
    test = HERE / "run_preliminary_estimator_tests.py"
    print("Starting S1-S3 SUMO runs to 600 seconds for seeds 1, 2, and 3.", flush=True)
    for seed in SIMULATION_SEEDS:
        seed_dir = RAW_DIR / f"seed_{seed}"
        print(f"Capturing all three conditions with SUMO seed {seed}.", flush=True)
        subprocess.run(
            [sys.executable, str(capture), "--conditions", "S1,S2,S3",
             "--seed", str(seed), "--output-dir", str(seed_dir)]
            + (["--overwrite"] if args.overwrite else []),
            check=True,
        )
    print("All SUMO captures ended. Starting masked-road validation.", flush=True)
    subprocess.run(
        [sys.executable, str(test), "--input-dir", str(RAW_DIR), "--output-dir", str(TEST_DIR)],
        check=True,
    )
    print("All SUMO captures and Model B validation outputs are complete.", flush=True)


if __name__ == "__main__":
    main()
