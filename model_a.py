from pathlib import Path
import numpy as np
import pandas as pd
from scipy.sparse import load_npz

# Model A matrix directory
BASE = Path(__file__).resolve().parent
DATA = BASE / "RouteX_Model_A_Matrices"

# Load the three matrices
A = load_npz(DATA / "A_adjacency_sparse.npz")
B = load_npz(DATA / "B_incidence_sparse.npz")
F = np.load(DATA / "F_features_numeric.npy")

# Load index mappings
nodes = pd.read_csv(DATA / "Node_Index.csv")
edges = pd.read_csv(DATA / "Edge_Index.csv")

# Verify matrix dimensions
n = len(nodes)
m = len(edges)

assert A.shape == (n, n)
assert B.shape == (n, m)
assert F.shape == (m, 4)

print("Model A loaded successfully.")
print("Adjacency matrix A:", A.shape)
print("Incidence matrix B:", B.shape)
print("Feature matrix F:", F.shape)