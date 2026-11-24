# Model C Route 2 launcher. Run from the src folder:  python run_model_c_route2.py
import model_c_runner as m

OLD = ('        ambulance_route = list(PAIR2_REFERENCE_ROUTE)\n'
       '        estimated_time = sum(\n'
       '            float(edge_data[edge_id]["length"]) / max(float(edge_data[edge_id]["speed"]), 0.1)\n'
       '            for edge_id in ambulance_route\n'
       '        )\n')
NEW = ('        _mc_route = MC.initial_route()\n'
       '        ambulance_route = list(_mc_route.edges)\n'
       '        estimated_time = float(_mc_route.travelTime)\n')
m.PATCHES[0] = (OLD, NEW)

_step = m.ModelC.step
def step(self, timestamp, closed_edges, *a, **k):
    if closed_edges:
        closed_edges = set(closed_edges) | set(self.ns.get("AMBULANCE_KNOWN_BLOCKAGE_EDGES", ()))
    return _step(self, timestamp, closed_edges, *a, **k)
m.ModelC.step = step

if __name__ == "__main__":
    m.main()
