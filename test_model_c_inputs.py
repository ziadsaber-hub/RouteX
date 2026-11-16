"""Tests for model_c_inputs.py (Model B adapter + Model D deltas). Run: python test_model_c_inputs.py"""
import os, tempfile, math
import pandas as pd
import task1_dijkstra as T1
from task1_dijkstra import E
import model_c_inputs as M

R = []
def check(name, ok, val=""):
    R.append((name, val, "PASS" if ok else "FAIL")); print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  ->  {val}" if val != "" else ""))

edges = sorted(E)
tmp = tempfile.mkdtemp()
def save(df, name): p = os.path.join(tmp, name); df.to_csv(p, index=False); return p

# 1) the standard format, all 546 edges, x2 of free flow
std = pd.DataFrame([dict(u=u, v=v, key=k, mu_hat_s=2 * E[(u, v, k)]["ff"]) for u, v, k in edges])
mu, rep = M.load_model_b(save(std, "std.csv"))
check("standard format u,v,key,mu_hat_s: 546/546 edges read", len(mu) == 546 and rep["fallback_to_free_flow"] == 0, rep["edges_with_time"])

# 2) edge_id text + minutes
txt = pd.DataFrame(dict(edge_id=[f"{u}-{v}-{k}" for u, v, k in edges], travel_time=[E[e]["ff"] / 60 for e in edges]))
mu2, _ = M.load_model_b(save(txt, "txt.csv"), unit="min")
check("edge_id 'u-v-k' + minutes converted to seconds", all(abs(mu2[e] - E[e]["ff"]) < 1e-9 for e in edges))

# 3) edge_index_0based (Edge_Index.csv order)
idx = pd.read_csv("Edge_Index.csv")
byidx = pd.DataFrame(dict(edge_index_0based=idx.edge_index_0based, mu_hat_s=idx.free_flow_s_precise))
mu3, _ = M.load_model_b(save(byidx, "idx.csv"))
check("edge_index_0based resolves through Edge_Index.csv", len(mu3) == 546 and all(abs(mu3[e] - E[e]["ff"]) < 1e-6 for e in edges))

# 4) missing edges + invalid values -> fallback, listed, counted
part = std.iloc[:500].copy()
part.loc[3, "mu_hat_s"] = float("nan"); part.loc[4, "mu_hat_s"] = -5; part.loc[5, "mu_hat_s"] = float("inf")
mu4, rep4 = M.load_model_b(save(part, "part.csv"))
check("absent edges and NaN/negative/inf rows are reported, not silently used",
      len(mu4) == 497 and len(rep4["dropped_invalid"]) == 3 and len(rep4["absent_from_file"]) == 46 and rep4["fallback_to_free_flow"] == 49,
      f"valid {len(mu4)}, invalid {len(rep4['dropped_invalid'])}, absent {len(rep4['absent_from_file'])}")
try:
    M.load_model_b(save(part, "part2.csv"), strict=True); strict_ok = False
except ValueError:
    strict_ok = True
check("strict=True refuses any fallback", strict_ok)

# 5) install: edges without time keep free flow, MISSING_B counted, Dijkstra still runs
rep5 = M.install_model_b(save(part, "part3.csv"))
check("install_model_b: MISSING_B == 49, missing edges keep free-flow", T1.MISSING_B == 49 and all(E[e]["mu"] == E[e]["ff"] for e in rep4["absent_from_file"]), T1.MODEL_B_SOURCE)
check("install_model_b: valid edge got Model B time", abs(E[edges[0]]["mu"] - 2 * E[edges[0]]["ff"]) < 1e-9)
route, cost = T1.dijkstra(edges[0][0], edges[200][1])
check("Dijkstra runs on the installed times", route is not None and abs(cost - T1.route_time(route)) < 1e-9, f"{cost:.1f} s")

# 6) intervals
multi = pd.concat([std.assign(interval="A"), std.assign(interval="B", mu_hat_s=std.mu_hat_s * 3)])
p6 = save(multi, "multi.csv")
try:
    M.load_model_b(p6); need = False
except ValueError:
    need = True
mu6, _ = M.load_model_b(p6, interval="B")
check("several intervals: must choose one; the chosen one is used", need and abs(mu6[edges[0]] - 6 * E[edges[0]]["ff"]) < 1e-9)

# 7) unknown edge / conflicting duplicates
bad = pd.concat([std.iloc[:5], pd.DataFrame([dict(u=-1, v=2, key=0, mu_hat_s=5)])])
try: M.load_model_b(save(bad, "bad.csv")); unk = False
except KeyError: unk = True
check("edge not in Model A -> error", unk)
dup = pd.concat([std.iloc[:3], std.iloc[:1].assign(mu_hat_s=999)])
try: M.load_model_b(save(dup, "dup.csv")); dp = False
except ValueError: dp = True
mu7, _ = M.load_model_b(save(dup, "dup2.csv"), aggregate="mean")
check("two different times for one edge -> error unless aggregate given", dp and len(mu7) == 3)

# 8) Excel input
xp = os.path.join(tmp, "b.xlsx"); std.to_excel(xp, index=False)
mu8, _ = M.load_model_b(xp)
check("xlsx input works", len(mu8) == 546)

# 9) Model D deltas
from task3_rerouting import Live, SIGNALS
tmpl = os.path.join(tmp, "tmpl.csv"); n = M.make_signal_template(tmpl)
t = pd.read_csv(tmpl)
check("template lists exactly the approach edges of verified signals", n == len(t) and set(t.signal_node) <= SIGNALS and n == sum(1 for e in E if e[1] in SIGNALS), f"{n} edges")
t.loc[0, "delta_s"] = -4.5; t.loc[1, "delta_s"] = 2.0
t.to_csv(tmpl, index=False)
d = M.load_signal_deltas(tmpl)
check("deltas: only filled rows read (negative allowed), empty rows skipped", len(d) == 2 and min(d.values()) == -4.5)
L = Live(); L.set_signal_deltas(d)
check("deltas install into Live()", len(L.signal) == 2)
_nonsig = next(e for e in edges if e[1] not in SIGNALS)
wrong = pd.DataFrame(dict(approach_edge_id=[f"{_nonsig[0]}-{_nonsig[1]}-{_nonsig[2]}"], delta_s=[1.0]))
try: M.load_signal_deltas(save(wrong, "w.csv")); w = False
except ValueError: w = True
check("delta on an edge that does not end at a verified signal -> error", w)

# restore free-flow so nothing leaks
T1.set_travel_times({e: E[e]["ff"] for e in E}, source="reset")
print(f"\n{sum(r[2]=='PASS' for r in R)} passed, {sum(r[2]=='FAIL' for r in R)} failed")
