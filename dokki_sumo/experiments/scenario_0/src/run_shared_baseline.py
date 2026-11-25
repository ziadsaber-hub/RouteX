import argparse
import importlib
import sys
from shared_disruptions import apply_shared_disruptions

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('--route', choices=(1,2), type=int, required=True)
known, rest = parser.parse_known_args()
baseline = importlib.import_module('run_scenarios' if known.route == 1 else 'run_scenarios_pair2')
apply_shared_disruptions(vars(baseline), known.route)
sys.argv = [sys.argv[0], *rest]
baseline.main()
