# RouteX — Dokki SUMO Scenario 0

This is the cleaned, reproducible handover package for the Dokki SUMO network and the two validated ambulance route pairs used in Scenario 0.

## What's included
- Model A source data and cleaned road geometry.
- Generated SUMO network and node/edge mappings.
- Scenario 0 traffic demand files (200, 400, and 600 vehicles).
- The two final scenario runners and demand generator.
- Final validated raw outputs for both route pairs, all six conditions.
- Consolidated result tables and the full technical handover.

## Start here
Read [`docs/SCENARIO_0_HANDOVER.md`](docs/SCENARIO_0_HANDOVER.md) before changing the simulation. It documents the route definitions, traffic conditions, closures, congestion corridors, baseline policies, results, output schemas, and integration guidance for Models B, C, and D.

## Environment
Validated environment:
- Windows
- Python 3.13.9
- Eclipse SUMO 1.28.0
- TraCI / SUMOLib 1.28.0
- NetworkX 3.7

Create and activate a virtual environment from the repository root:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements-scenario0.txt
```

The runners currently contain the SUMO executable path:

```python
SUMO = r"C:\Program Files (x86)\SUMO\bin\sumo.exe"
```

If SUMO is installed elsewhere, update this constant in both scenario runners before running.

## Run the experiments

From the repository root:

```powershell
python dokki_sumo/experiments/scenario_0/src/run_scenarios.py --output-tag route1_reproduction
python dokki_sumo/experiments/scenario_0/src/run_scenarios_pair2.py --output-tag route2_reproduction
```

Each command writes to:

```text
dokki_sumo/experiments/scenario_0/outputs/<output-tag>/
```

Use a new output tag for each run so validated outputs are not overwritten. See the handover document for subset runs, output interpretation, and verification checks.

## Final validated results
- Route 1: all six conditions completed; S5 = 508 s and S6 = 542 s.
- Route 2: all six conditions completed; S5 = 398 s and S6 = 421 s.
- Ambulance collisions: zero in all 12 validated route-condition combinations.

The authoritative consolidated files are:
- `dokki_sumo/experiments/scenario_0/outputs/final/all_routes_summary.csv`
- `dokki_sumo/experiments/scenario_0/outputs/final/route_comparison_vs_S2.csv`

## GitHub
This package is prepared for a repository commit. From the repository root:

```powershell
git status
git add README.md docs requirements-scenario0.txt requirements-map.txt .gitignore dokki_sumo
git commit -m "Prepare Dokki SUMO Scenario 0 handover"
git push origin <your-branch>
```

Replace `<your-branch>` with the branch your team uses.
