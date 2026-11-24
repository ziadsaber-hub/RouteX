# Model C – final (v2)

Run from `dokki_sumo/experiments/scenario_0/src` (venv active):

```powershell
python run_model_c.py --route 1     # outputs/model_c_v2_route_1
python run_model_c.py --route 2     # outputs/model_c_v2_route_2
```

Result vs Scenario 0 (12 combinations): 8 better, 1 similar, 3 worse, 0 fail, 0 ambulance collisions.
All versions compared: `version_comparison.csv`; 3-seed check (v2 confirmed): `multi_seed_evaluation.csv`. Method, analysis, limits: `Model_C_Evaluation_Report.pdf`.

| Folder / file | Content |
|---|---|
| `results/` | main results, baseline comparison, reroute events, alternatives, initial selection, route history, prediction errors |
| `figures/` | report figures |
| `code/` | `run_model_c.py` (Model C v2), `model_c_export.py`, `build_model_c_deliverables.py` |
| `validation_report.txt` | automatic checks (0 issues) |
