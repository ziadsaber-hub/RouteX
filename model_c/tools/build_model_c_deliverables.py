"""
Model C - build the final 12-combination deliverables (Route 1 + Route 2).

Merges the per-route Model C result CSVs, recomputes every comparison against the
Scenario 0 baseline, validates the results against the SUMO network, and draws the
report figures.

Usage (from the repo root, venv active):
    python build_model_c_deliverables.py ^
        --route1 ModelC_Route1_deliverables/results ^
        --route2 ModelC_Route2_deliverables/results ^
        --repo   .  ^
        --out    ModelC_final_deliverables

Each --routeN folder must hold the six CSVs that model_c_runner.py produces:
model_c_main_results.csv, baseline_comparison.csv, reroute_events.csv,
alternatives_log.csv, initial_route_selection.csv, route_history_all.csv.
--route2 can be omitted; the script then builds Route 1 only and says so.
"""
import argparse
import csv
import json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SIMILAR_PCT = 2.0          # |improvement| below this -> "similar"
SCHEDULED_DEPART_S = 600.0
SCEN = ["S1", "S2", "S3", "S4", "S5", "S6"]
SCEN_NAME = {"S1": "Normal", "S2": "Moderate", "S3": "Heavy", "S4": "Localized",
             "S5": "Closure", "S6": "Combined"}
FILES = ["model_c_main_results.csv", "baseline_comparison.csv", "reroute_events.csv",
         "alternatives_log.csv", "initial_route_selection.csv", "route_history_all.csv"]
ROUTE_DIR = {"Route 1": "route_1", "Route 2": "route_2"}
ROUTE_CFG = {"Route 1": "route_configuration.json", "Route 2": "pair_configuration.json"}


def read(path):
    with open(path, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write(path, rows, fields=None):
    fields = fields or (list(rows[0].keys()) if rows else [])
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def f(x):
    return float(x) if x not in (None, "") else 0.0


def load_net(net_path):
    root = ET.parse(net_path).getroot()
    length, conn = {}, set()
    for e in root.findall("edge"):
        if e.get("function") == "internal":
            continue
        length[e.get("id")] = float(e.find("lane").get("length"))
    for c in root.findall("connection"):
        if not c.get("from", "").startswith(":"):
            conn.add((c.get("from"), c.get("to")))
    return length, conn


def check_route(tag, edges, length, conn, issues):
    for a, b in zip(edges, edges[1:]):
        if a not in length or b not in length:
            issues.append(f"{tag}: unknown edge {a if a not in length else b}")
        elif (a, b) not in conn:
            issues.append(f"{tag}: no SUMO connection {a} -> {b}")
    if len(set(edges)) != len(edges):
        issues.append(f"{tag}: route repeats an edge (loop)")


def load_route(folder, route_id):
    data = {}
    for name in FILES:
        rows = read(Path(folder) / name)
        for r in rows:                      # Route 1 files lack route_id in some tables
            r.setdefault("route_id", route_id)
            if not r["route_id"]:
                r["route_id"] = route_id
            if "scenario_id" not in r and "scenario" in r:
                r["scenario_id"] = r["scenario"]
        data[name] = rows
    return data


def validate(route_id, d, base, cfg, length, conn):
    issues = []
    closed = set(cfg.get("all_closures", []))
    corridor = set(cfg.get("congestion_edges", []))
    events = defaultdict(list)
    for e in d["reroute_events.csv"]:
        events[e["scenario_id"]].append(e)
    hist = defaultdict(lambda: defaultdict(list))
    for h in d["route_history_all.csv"]:
        hist[h["scenario_id"]][int(h["route_version"])].append((int(h["route_index"]), h["edge_id"]))
    alts = defaultdict(list)
    for a in d["alternatives_log.csv"]:
        alts[(a["scenario_id"], a["reroute_event_id"])].append(a)

    main = {m["scenario"]: m for m in d["model_c_main_results.csv"]}
    for s in SCEN:
        tag = f"{route_id} {s}"
        if s not in main:
            issues.append(f"{tag}: missing from model_c_main_results.csv")
            continue
        m, t = main[s], f(main[s]["actual_travel_time_s"])
        if m["completion_status"] != "completed":
            issues.append(f"{tag}: ambulance did not complete ({m['completion_status']})")
        if f(m["ambulance_collisions"]) > 0:
            issues.append(f"{tag}: AMBULANCE COLLISION -> counts as fail")
        if int(f(m["route_updates"])) != len(events[s]):
            issues.append(f"{tag}: route_updates {m['route_updates']} != {len(events[s])} logged events")
        ini, fin = m["initial_route"].split(), m["final_route"].split()
        check_route(tag + " initial", ini, length, conn, issues)
        check_route(tag + " final", fin, length, conn, issues)
        if s in ("S5", "S6") and closed & set(fin):
            issues.append(f"{tag}: final route uses closed edge(s) {sorted(closed & set(fin))}")
        if hist[s]:
            v0 = [e for _, e in sorted(hist[s][0])]
            if v0 != ini:
                issues.append(f"{tag}: route history v0 differs from initial_route")
            if len(hist[s]) - 1 != len(events[s]):
                issues.append(f"{tag}: {len(hist[s]) - 1} route versions vs {len(events[s])} events")
        else:
            issues.append(f"{tag}: no route history")
        a0 = alts[(s, "0")]
        sel = [a for a in a0 if a["selected"] == "1"]
        if not a0 or len(sel) != 1 or f(sel[0]["predicted_time_s"]) > min(f(a["predicted_time_s"]) for a in a0) + 1e-6:
            issues.append(f"{tag}: initial selection is not the lowest-predicted candidate")
        prev = None
        for e in events[s]:
            et = f"{tag} event {e['reroute_event_id']}"
            st, rem = f(e["simulation_time_s"]), f(e["actual_remaining_time_s"])
            if abs(st + rem - (SCHEDULED_DEPART_S + t)) > 1.5:
                issues.append(f"{et}: time + remaining != arrival")
            if abs(f(e["prediction_error_s"]) - (rem - f(e["predicted_time_s"]))) > 0.05:
                issues.append(f"{et}: prediction_error arithmetic")
            if e["trigger_reason"] == "threshold_exceeded" and f(e["trigger_value"]) <= f(e["threshold"]):
                issues.append(f"{et}: trigger value not above threshold")
            if len(alts[(s, e["reroute_event_id"])]) != int(f(e["number_of_alternatives"])):
                issues.append(f"{et}: alternatives logged != number_of_alternatives")
            check_route(et + " selected", e["selected_route"].split(), length, conn, issues)
            if prev is not None and st - prev < 20 - 1e-6 and e["trigger_reason"] != "closure":
                issues.append(f"{et}: less than 20 s after previous reroute")
            prev = st
        b = base.get(s)
        if b is None:
            issues.append(f"{tag}: no Scenario 0 baseline row")
    return issues, events, corridor, closed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route1", required=True)
    ap.add_argument("--route2")
    ap.add_argument("--repo", default=".")
    ap.add_argument("--out", default="ModelC_final_deliverables")
    a = ap.parse_args()

    repo = Path(a.repo)
    final = repo / "dokki_sumo/experiments/scenario_0/outputs/final"
    length, conn = load_net(repo / "dokki_sumo/network/dokki.net.xml")
    base_all = read(final / "all_routes_summary.csv")
    out = Path(a.out)
    (out / "results").mkdir(parents=True, exist_ok=True)
    (out / "figures").mkdir(parents=True, exist_ok=True)

    routes = {"Route 1": a.route1}
    if a.route2:
        routes["Route 2"] = a.route2

    merged = {n: [] for n in FILES}
    comparison, issues, all_events, extra = [], [], [], {}
    for rid, folder in routes.items():
        d = load_route(folder, rid)
        base = {r["scenario_id"]: r for r in base_all if r["route_id"] == rid}
        cfg = json.load(open(final / ROUTE_DIR[rid] / ROUTE_CFG[rid], encoding="utf-8"))
        iss, events, corridor, closed = validate(rid, d, base, cfg, length, conn)
        issues += iss
        for n in FILES:
            merged[n] += d[n]
        for m in d["model_c_main_results.csv"]:
            s, b = m["scenario"], base.get(m["scenario"])
            if b is None:
                continue
            bt, ct = f(b["ambulance_travel_time_s"]), f(m["actual_travel_time_s"])
            saved = bt - ct
            pct = saved / bt * 100
            ok = m["completion_status"] == "completed" and f(m["ambulance_collisions"]) == 0
            cls = "fail" if not ok else ("similar" if abs(pct) < SIMILAR_PCT else ("improved" if pct > 0 else "worse"))
            fin = set(m["final_route"].split())
            comparison.append({
                "route_id": rid, "scenario": s, "scenario_name": m.get("scenario_name", ""),
                "baseline_travel_s": bt, "model_c_travel_s": ct, "time_saved_s": round(saved, 2),
                "improvement_pct": round(pct, 2), "classification": cls,
                "baseline_waiting_s": f(b["ambulance_waiting_time_s"]), "model_c_waiting_s": f(m["waiting_time_s"]),
                "waiting_reduction_s": round(f(b["ambulance_waiting_time_s"]) - f(m["waiting_time_s"]), 2),
                "moving_time_saved_s": round(saved - (f(b["ambulance_waiting_time_s"]) - f(m["waiting_time_s"])), 2),
                "baseline_distance_m": f(b["ambulance_distance_m"]), "model_c_distance_m": f(m["route_distance_m"]),
                "baseline_route_updates": int(f(b["route_update_count"])), "model_c_route_updates": int(f(m["route_updates"])),
                "baseline_ambulance_collisions": int(f(b["ambulance_collision_count"])),
                "model_c_ambulance_collisions": int(f(m["ambulance_collisions"])),
                "model_c_completed": m["completion_status"],
                "final_route_avoids_closures": "" if s not in ("S5", "S6") else (not (closed & fin)),
                "final_route_corridor_edges_used": "" if s not in ("S4", "S6") else f"{len(corridor & fin)}/{len(corridor)}",
            })
        all_events += [(rid, e) for e in d["reroute_events.csv"]]

    for n in FILES:
        if n != "baseline_comparison.csv":
            write(out / "results" / n, merged[n])
    write(out / "results" / "baseline_comparison.csv", comparison)

    # prediction-error summary per route/scenario
    err = defaultdict(list)
    for rid, e in all_events:
        err[(rid, e["scenario_id"])].append(f(e["prediction_error_s"]))
    err_rows = [{"route_id": r, "scenario": s, "reroutes": len(v), "mean_error_s": round(sum(v) / len(v), 1),
                 "min_error_s": round(min(v), 1), "max_error_s": round(max(v), 1),
                 "optimistic_share": round(sum(x > 0 for x in v) / len(v), 2)} for (r, s), v in sorted(err.items())]
    write(out / "results" / "prediction_error_summary.csv", err_rows,
          ["route_id", "scenario", "reroutes", "mean_error_s", "min_error_s", "max_error_s", "optimistic_share"])

    # ---------- figures ----------
    rids = list(routes)
    C = {"base": "#8a8f98", "mc": "#2a6fdb", "wait": "#e0893a", "move": "#2a6fdb"}

    def grouped(key_b, key_c, ylabel, fname, title):
        fig, axes = plt.subplots(1, len(rids), figsize=(6.2 * len(rids), 4), squeeze=False)
        for ax, rid in zip(axes[0], rids):
            rows = [r for r in comparison if r["route_id"] == rid]
            x = range(len(rows))
            ax.bar([i - 0.2 for i in x], [r[key_b] for r in rows], 0.4, label="Scenario 0", color=C["base"])
            ax.bar([i + 0.2 for i in x], [r[key_c] for r in rows], 0.4, label="Model C", color=C["mc"])
            ax.set_xticks(list(x), [f"{r['scenario']}\n{SCEN_NAME[r['scenario']]}" for r in rows], fontsize=8)
            ax.set_title(rid); ax.set_ylabel(ylabel); ax.grid(axis="y", alpha=0.3); ax.spines[["top", "right"]].set_visible(False)
        axes[0][0].legend(frameon=False)
        fig.suptitle(title); fig.tight_layout(); fig.savefig(out / "figures" / fname, dpi=160); plt.close(fig)

    grouped("baseline_travel_s", "model_c_travel_s", "Travel time (s)", "fig1_travel_time.png", "Ambulance travel time")
    grouped("baseline_waiting_s", "model_c_waiting_s", "Waiting time (s)", "fig2_waiting_time.png", "Ambulance waiting time")

    fig, axes = plt.subplots(1, len(rids), figsize=(6.2 * len(rids), 4), squeeze=False)
    for ax, rid in zip(axes[0], rids):
        rows = [r for r in comparison if r["route_id"] == rid]
        x = list(range(len(rows)))
        w = [r["waiting_reduction_s"] for r in rows]; mv = [r["moving_time_saved_s"] for r in rows]
        ax.bar(x, w, color=C["wait"], label="Less waiting")
        ax.bar(x, mv, bottom=[max(v, 0) if m >= 0 else min(v, 0) for v, m in zip(w, mv)], color=C["move"], label="Less moving time")
        ax.axhline(0, color="black", lw=0.8)
        ax.set_xticks(x, [r["scenario"] for r in rows]); ax.set_title(rid); ax.set_ylabel("Time saved (s)")
        ax.grid(axis="y", alpha=0.3); ax.spines[["top", "right"]].set_visible(False)
    axes[0][0].legend(frameon=False); fig.suptitle("Where the time saving comes from")
    fig.tight_layout(); fig.savefig(out / "figures" / "fig3_saving_breakdown.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    for rid, mk in zip(rids, ["o", "s"]):
        pts = [(f(e["simulation_time_s"]) - SCHEDULED_DEPART_S, f(e["prediction_error_s"])) for r, e in all_events if r == rid]
        if pts:
            ax.scatter(*zip(*pts), marker=mk, label=rid, alpha=0.8)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_xlabel("Time since departure at reroute (s)"); ax.set_ylabel("Actual − predicted remaining (s)")
    ax.set_title("Prediction error at each reroute (positive = optimistic)"); ax.legend(frameon=False)
    ax.grid(alpha=0.3); ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout(); fig.savefig(out / "figures" / "fig4_prediction_error.png", dpi=160); plt.close(fig)

    # ---------- validation report ----------
    n = len(comparison)
    lines = [f"Model C validation - {n} of 12 route-condition combinations present."]
    if n < 12:
        lines.append("INCOMPLETE: " + ", ".join(sorted(set(ROUTE_DIR) - set(routes))) + " missing.")
    lines.append(f"Reroute events: {len(all_events)}")
    for c in ("improved", "similar", "worse", "fail"):
        lines.append(f"  {c}: {[r['route_id'][-1] + '-' + r['scenario'] for r in comparison if r['classification'] == c]}")
    lines.append(f"Issues found: {len(issues)}")
    lines += ["  - " + i for i in issues]
    (out / "validation_report.txt").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
