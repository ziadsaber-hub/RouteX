#!/usr/bin/env python3
"""
mask_tomtom_json_coefficients.py - mask real TomTom observations and re-estimate them with the
coefficients stored in Model_B_Global_Coefficient_Selected1.json (NO re-tuning).

Protocol
  * data      : the 937 REAL TomTom cells (the other 701 cells are missing in the real data)
  * masking   : hide 20 / 40 / 60 % of the observed data, in two ways
                  cell : random observed cells
                  road : whole roads (all their observed intervals)
  * repeats   : 10 random masks per setting (seeds 200..209)
  * estimator : model_b_estimator.ModelB with the fixed coefficients; the hidden values are
                never given to it; estimates are compared with the hidden TomTom values
  * compared  : the same masks are also run with the coefficients selected earlier (1, 1, 0.1),
                and with two naive baselines
  * numerical : the safeguard ridge is set to 1e-10 (the JSON coefficients have gamma/alpha = 1e10;
                at the default 1e-6 the safeguard would distort the solution)

    python mask_tomtom_json_coefficients.py --data-dir data --out results \
        --json Model_B_Global_Coefficient_Selected1.json --similarity Model_B_Similarity_Provisional_1.csv
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
import pandas as pd

import model_b as mb
import model_b_estimator as est

LEVELS = [20, 40, 60]
REPS = 10
SEED0 = 200
RIDGE = 1e-10
MINE = (1.0, 1.0, 0.1)


def make_mask(W, mode, pct, rng):
    obs = W > 0
    test = np.zeros_like(obs)
    if mode == "cell":
        idx = np.argwhere(obs)
        k = int(round(pct / 100 * len(idx)))
        test[tuple(idx[rng.permutation(len(idx))[:k]].T)] = True
    else:
        roads = np.where(obs.any(axis=1))[0]
        k = int(round(pct / 100 * len(roads)))
        pick = rng.permutation(roads)[:k]
        test[pick, :] = obs[pick, :]
    return test


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=".")
    ap.add_argument("--out", default="results")
    ap.add_argument("--json", default="Model_B_Global_Coefficient_Selected1.json")
    ap.add_argument("--similarity", default="Model_B_Similarity_Provisional_1.csv")
    a = ap.parse_args()
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    jpath, spath = Path(a.json).resolve(), Path(a.similarity).resolve()
    os.chdir(a.data_dir)

    J = json.load(open(jpath))
    sets = {"JSON coefficients": (J["selected_alpha"], J["selected_beta"], J["selected_gamma"]),
            "earlier coefficients": MINE}
    edges = mb.load_edges()
    labels = mb.load_time_bins()["bin_label"].tolist()
    T = len(labels)
    Y, W = mb.to_matrices(mb.load_tomtom_observations())
    Yz = np.nan_to_num(Y)
    ref = edges["model_a_reference_time_s"].to_numpy(float)
    sim = pd.read_csv(spath, encoding="utf-8-sig")
    est.EPS_RIDGE = RIDGE
    model = est.ModelB(sim, mb.N_EDGES, T)
    R = np.repeat(ref[:, None], T, axis=1)

    rows = []
    for mode in ("cell", "road"):
        for pct in LEVELS:
            for rep in range(REPS):
                test = make_mask(W, mode, pct, np.random.default_rng(SEED0 + 100 * pct + rep + (0 if mode == "cell" else 7)))
                Wtr = W.copy()
                Wtr[test] = 0
                z0 = float((np.nan_to_num(Y / R) * Wtr).sum() / Wtr.sum())
                scopes = [("overall", test)] + [(labels[b], test & (np.arange(T)[None, :] == b)) for b in range(T)]
                for name, (al, be, ga) in sets.items():
                    try:
                        model.fit(Yz, Wtr, ref, al, be, ga)
                        P = model.estimate_
                    except Exception:           # numerically singular solve: record as failed
                        continue
                    for scope, m in scopes:
                        e = est.metrics(Y[m], P[m])
                        rows.append(dict(coefficient_set=name, mask_type=mode, masked_pct=pct, rep=rep, interval=scope,
                                         n_train=int(Wtr.sum()), n_test=e["n"], MAE=e["MAE"], RMSE=e["RMSE"], MAPE=e["MAPE"],
                                         base_meanratio_MAE=est.metrics(Y[m], (z0 * R)[m])["MAE"],
                                         base_freeflow_MAE=est.metrics(Y[m], R[m])["MAE"]))
    runs = pd.DataFrame(rows)
    runs.to_csv(out / "Model_B_TomTom_Masking_Runs.csv", index=False, float_format="%.6g")

    # ------------------------------------------------------------ Excel (formulas over the Runs sheet)
    from openpyxl import Workbook
    from openpyxl.styles import Alignment, Font, PatternFill
    from openpyxl.utils import get_column_letter
    F = lambda **k: Font(name="Arial", size=10, **k)
    HF, HFill = Font(name="Arial", size=10, bold=True, color="FFFFFF"), PatternFill("solid", fgColor="1F3864")

    def head(ws, r, names):
        for j, n in enumerate(names, 1):
            c = ws.cell(r, j, n)
            c.font, c.fill, c.alignment = HF, HFill, Alignment(wrap_text=True)

    wb = Workbook()
    ws = wb.active
    ws.title = "Summary"
    ws["A1"] = "Model B - TomTom masking experiment with fixed coefficients"
    ws["A1"].font = Font(name="Arial", size=14, bold=True)
    ws["A2"] = (f"Real TomTom data only (937 observed cells of 1,638). Hidden values are never given to the estimator. "
                f"JSON coefficients: alpha={sets['JSON coefficients'][0]:g}, beta={sets['JSON coefficients'][1]:g}, "
                f"gamma={sets['JSON coefficients'][2]:g} (no re-tuning). {REPS} random masks per setting; safeguard ridge {RIDGE:g}.")
    ws["A2"].font = F()
    ws["A2"].alignment = Alignment(wrap_text=True, vertical="top")
    ws.merge_cells("A2:J4")
    # Runs columns: A set,B mask_type,C pct,D rep,E interval,F n_train,G n_test,H MAE,I RMSE,J MAPE,K base1,L base2
    head(ws, 6, ["Coefficients", "Mask type", "Masked %", "Test cells (mean)", "MAE (s)", "RMSE (s)", "MAPE (%)",
                 "Baseline MAE: mean ratio (s)", "MAE gain vs baseline", "Successful runs of 10"])
    r = 7
    for name in sets:
        for mode in ("cell", "road"):
            for pct in LEVELS:
                ws.cell(r, 1, name).font = F()
                ws.cell(r, 2, mode).font = F()
                ws.cell(r, 3, pct).font = F()
                crit = f'Runs!$A:$A,$A{r},Runs!$B:$B,$B{r},Runs!$C:$C,$C{r},Runs!$E:$E,"overall"'
                for col, src in [(4, "G"), (5, "H"), (6, "I"), (7, "J"), (8, "K")]:
                    c = ws.cell(r, col, f"=AVERAGEIFS(Runs!${src}:${src},{crit})")
                    c.font = F()
                    c.number_format = "0.0" if col == 4 else "0.00"
                c = ws.cell(r, 9, f"=1-E{r}/H{r}")
                c.number_format, c.font = "0.0%", F()
                ws.cell(r, 10, f"=COUNTIFS({crit})").font = F()
                r += 1
    ws.cell(r + 1, 1, "mask type: cell = random observed cells hidden; road = whole roads hidden (all their observed intervals).").font = F()
    for col, w in zip("ABCDEFGHIJ", [24, 11, 10, 14, 10, 10, 10, 18, 14, 14]):
        ws.column_dimensions[col].width = w

    w2 = wb.create_sheet("By_Interval_JSON")
    head(w2, 1, ["Mask type", "Masked %", "Interval", "Test cells (mean)", "MAE (s)", "RMSE (s)", "MAPE (%)",
                 "Baseline MAE (s)", "MAE gain vs baseline"])
    r = 2
    for mode in ("cell", "road"):
        for pct in LEVELS:
            for lab in ["overall"] + labels:
                w2.cell(r, 1, mode).font = F()
                w2.cell(r, 2, pct).font = F()
                w2.cell(r, 3, lab).font = F()
                crit = (f'Runs!$A:$A,"JSON coefficients",Runs!$B:$B,$A{r},Runs!$C:$C,$B{r},Runs!$E:$E,$C{r}')
                for col, src in [(4, "G"), (5, "H"), (6, "I"), (7, "J"), (8, "K")]:
                    c = w2.cell(r, col, f'=IFERROR(AVERAGEIFS(Runs!${src}:${src},{crit}),"n/a")')
                    c.font = F()
                    c.number_format = "0.0" if col == 4 else "0.00"
                c = w2.cell(r, 9, f'=IFERROR(1-E{r}/H{r},"n/a")')
                c.number_format, c.font = "0.0%", F()
                r += 1
    for col, w in zip("ABCDEFGHI", [11, 10, 18, 14, 10, 10, 10, 16, 16]):
        w2.column_dimensions[col].width = w
    w3 = wb.create_sheet("Runs")
    head(w3, 1, list(runs.columns))
    for i, row in enumerate(runs.itertuples(index=False), 2):
        for j, v in enumerate(row, 1):
            w3.cell(i, j, v.item() if hasattr(v, "item") else v).font = F()
    w3.freeze_panes = "A2"
    for j in range(1, len(runs.columns) + 1):
        w3.column_dimensions[get_column_letter(j)].width = 16
    wb.save(out / "Model_B_TomTom_Masking_JSON_Coefficients.xlsx")
    print(f"wrote {out / 'Model_B_TomTom_Masking_JSON_Coefficients.xlsx'} and Model_B_TomTom_Masking_Runs.csv "
          f"({len(runs)} rows)")


if __name__ == "__main__":
    main()
