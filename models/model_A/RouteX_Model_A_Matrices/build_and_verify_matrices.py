#!/usr/bin/env python3
"""Rebuild and independently validate RouteX Model A matrices from index CSVs.

Run in the directory containing this file:
    python build_and_verify_matrices.py

Requirements: numpy, scipy, networkx.
The node ordering is Node_Index.csv; the edge ordering is Edge_Index.csv.
"""
from pathlib import Path
import csv
import json
import math
import numpy as np
import scipy.sparse as sparse
import networkx as nx

HERE = Path(__file__).resolve().parent

def read_csv(name):
    with (HERE / name).open("r", newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))

def matrix_triplets(M):
    C = M.tocoo()
    return {(int(r), int(c)): int(v) for r, c, v in zip(C.row, C.col, C.data)}

def main():
    node_rows = read_csv("Node_Index.csv")
    edge_rows = read_csv("Edge_Index.csv")
    n, m = len(node_rows), len(edge_rows)
    assert n > 0 and m > 0, "Empty network"

    node_ids = [int(r["node_id"]) for r in node_rows]
    assert len(set(node_ids)) == n, "Duplicate node ID"
    assert [int(r["node_index_0based"]) for r in node_rows] == list(range(n)), "Node ordering broken"
    node_to_index = {node_id: i for i, node_id in enumerate(node_ids)}

    keys = [(int(r["u_node_id"]), int(r["v_node_id"]), int(r["key"])) for r in edge_rows]
    assert len(set(keys)) == m, "Duplicate (u,v,key) edge"
    assert [int(r["edge_index_0based"]) for r in edge_rows] == list(range(m)), "Edge ordering broken"

    src, dst, frows = [], [], []
    G = nx.MultiDiGraph()
    G.add_nodes_from(range(n))
    for j, (r, (u, v, key)) in enumerate(zip(edge_rows, keys)):
        assert u in node_to_index and v in node_to_index, f"Edge {j} has unknown endpoint"
        assert u != v, f"Edge {j} is a self-loop; recheck incidence convention"
        ui, vi = node_to_index[u], node_to_index[v]
        assert int(r["source_node_index_0based"]) == ui
        assert int(r["target_node_index_0based"]) == vi
        src.append(ui)
        dst.append(vi)

        length = float(r["length_m"])
        speed = float(r["reference_speed_kph"])
        assert math.isfinite(length) and length > 0, f"Invalid length in edge {j}"
        assert math.isfinite(speed) and speed > 0, f"Invalid speed in edge {j}"
        exact_t = 3.6 * length / speed
        assert abs(exact_t - float(r["free_flow_s_precise"])) < 1e-9
        assert abs(exact_t - float(r["workbook_free_flow_s_rounded"])) <= 0.00500001

        raw_lane = r["lanes_numeric"].strip()
        lane = float(raw_lane) if raw_lane else float("nan")
        if raw_lane:
            assert math.isfinite(lane) and lane >= 1 and lane.is_integer()
        frows.append((length, lane, speed, exact_t))
        G.add_edge(ui, vi, key=j, weight=exact_t)

    src, dst = np.asarray(src), np.asarray(dst)
    F = np.asarray(frows, dtype=np.float64)

    # Binary directed adjacency: parallel roads do not increase A_ij beyond 1.
    A = sparse.coo_matrix(
        (np.ones(m, dtype=np.int8), (src, dst)), shape=(n, n)
    ).tocsr()
    A.data[:] = 1
    A.eliminate_zeros()

    # Incidence convention: -1 at tail (outgoing), +1 at head (incoming).
    edge_indices = np.arange(m)
    B = sparse.coo_matrix(
        (
            np.concatenate((-np.ones(m, dtype=np.int8), np.ones(m, dtype=np.int8))),
            (np.concatenate((src, dst)), np.concatenate((edge_indices, edge_indices)))
        ),
        shape=(n, m)
    ).tocsr()

    assert A.shape == (n, n) and B.shape == (n, m) and F.shape == (m, 4)
    assert A.nnz == len(set(zip(src, dst)))
    assert set(np.unique(A.data)) == {1}
    assert B.nnz == 2*m, "Each edge must have exactly two incidence entries"
    assert np.all(B.getnnz(axis=0) == 2)
    assert np.all(np.asarray(B.sum(axis=0)) == 0)
    assert all(B[int(u), j] == -1 and B[int(v), j] == 1
               for j, (u, v) in enumerate(zip(src, dst)))

    # Independent graph vs. matrix consistency.
    assert G.number_of_nodes() == n and G.number_of_edges() == m
    assert nx.number_weakly_connected_components(G) == nx.number_connected_components(G.to_undirected())
    weak = nx.number_weakly_connected_components(G)
    strong = nx.number_strongly_connected_components(G)
    rank = int(np.linalg.matrix_rank(B.toarray().astype(np.float64)))
    assert rank == n - weak, "Incidence matrix rank disagrees with weak connectivity"

    # A route must satisfy B x = b, and its travel time equals t^T x.
    source = node_to_index[node_ids[0]]
    distances, paths = nx.single_source_dijkstra(G, source=source, weight="weight")
    target = max(distances, key=lambda i: distances[i])
    path = paths[target]
    x = np.zeros(m, dtype=np.int8)
    for ui, vi in zip(path[:-1], path[1:]):
        best_edge = min(G[ui][vi], key=lambda k: G[ui][vi][k]["weight"])
        x[best_edge] = 1
    b = np.zeros(n, dtype=np.int64)
    b[source] = -1
    b[target] = +1
    assert np.array_equal(B @ x, b), "Route fails node-flow conservation"
    assert np.isclose(float(F[:, 3] @ x), distances[target], rtol=0, atol=1e-9)

    sparse.save_npz(HERE / "A_adjacency_sparse.npz", A)
    sparse.save_npz(HERE / "B_incidence_sparse.npz", B)
    np.save(HERE / "F_features_numeric.npy", F)

    # Cross-check serialization.
    assert (A != sparse.load_npz(HERE / "A_adjacency_sparse.npz")).nnz == 0
    assert (B != sparse.load_npz(HERE / "B_incidence_sparse.npz")).nnz == 0
    assert np.allclose(F, np.load(HERE / "F_features_numeric.npy"), equal_nan=True)

    summary = {
        "nodes": n, "directed_edges": m,
        "A_shape": list(A.shape), "A_nnz": A.nnz,
        "B_shape": list(B.shape), "B_nnz": B.nnz,
        "F_shape": list(F.shape), "F_unresolved_lanes": int(np.isnan(F[:,1]).sum()),
        "weak_components": weak, "strong_components": strong,
        "incidence_rank": rank,
        "route_test": {
            "origin_node_id": node_ids[source],
            "destination_node_id": node_ids[target],
            "edge_count": int(x.sum()),
            "free_flow_s": float(F[:,3] @ x),
            "Bx_equals_b": True
        }
    }
    (HERE / "Verification_Report.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))

if __name__ == "__main__":
    main()
