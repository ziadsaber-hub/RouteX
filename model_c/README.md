# Model C – Emergency Vehicle Route Planning and Dynamic Rerouting

Model C chooses the ambulance's route and reroutes it while the simulation runs.
It is evaluated against the Scenario 0 baseline on both routes (Route 1: `181 -> hospital_168`,
Route 2: `4 -> hospital_95`) under the six conditions S1–S6.

Final version: **v2** – Ziad Abosrea's routing framework combined with Ali Salama's travel-time
estimate and rerouting rules, plus safety fixes. No route is hard-coded per scenario.

## Result (3 seeds × 12 route–conditions = 36 runs)

| | Scenario 0 | Model C |
|---|---|---|
| Total ambulance travel time | 22,267 s | **15,029 s (−32.5%)** |
| Better / similar / worse than baseline (2% band) | – | 24 / 5 / 7 |
| Ambulance collisions / failed trips | 0 | **0** |

Details, analysis and limitations: `Model_C_Evaluation_Report.pdf`.

## Folder structure

| Path | Content |
|---|---|
| `Model_C_Evaluation_Report.pdf` | Evaluation report |
| `results/` | Final result tables (main results, baseline comparison, reroute events, alternatives, initial selection, route history, prediction errors), `version_comparison.csv`, `multi_seed_evaluation.csv`, `validation_report.txt` |
| `figures/` | Report figures |
| `tools/` | `model_c_export.py` (run folder -> result CSVs), `build_model_c_deliverables.py` (tables, validation, figures) |
| `phase1/` | Earlier Model C tasks: Dijkstra/A*, alternative routes, rerouting, export interface, Model B/D input adapters, validation |

Simulation code (must stay next to the Scenario 0 runners):
`dokki_sumo/experiments/scenario_0/src/` – `run_model_c.py`, `model_c_runner.py`, `model_c_runner_route1.py`,
`model_c_config.json`, `model_c_config_route1.json`.

Raw outputs of the final runs (seed 1): `dokki_sumo/experiments/scenario_0/outputs/model_c_v2_route_1/` and `.../model_c_v2_route_2/`.

## How to run

From `dokki_sumo/experiments/scenario_0/src` (Python 3.13, SUMO 1.28.0, venv active):

```powershell
python run_model_c.py --route 1            # outputs/model_c_v2_route_1
python run_model_c.py --route 2            # outputs/model_c_v2_route_2
python run_model_c.py --route 2 --seed 2 --output-tag my_run   # other seed / folder
```

Then build the tables and figures:

```powershell
python ..\..\..\..\model_c\tools\model_c_export.py --run-dir ..\outputs\model_c_v2_route_1 --route-id "Route 1" --out r1
python ..\..\..\..\model_c\tools\model_c_export.py --run-dir ..\outputs\model_c_v2_route_2 --route-id "Route 2" --out r2
python ..\..\..\..\model_c\tools\build_model_c_deliverables.py --route1 r1 --route2 r2 --repo ..\..\..\.. --out ..\outputs\model_c_final
```
