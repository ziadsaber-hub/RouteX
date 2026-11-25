import argparse
import importlib
import sys

from official_disruptions import apply_official_disruptions

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument("--route", choices=(1, 2), type=int, required=True)
parser.add_argument("--sumo", default="")
options, rest = parser.parse_known_args()
module = importlib.import_module("run_scenarios" if options.route == 1 else "run_scenarios_pair2")
apply_official_disruptions(vars(module), options.route)
if options.sumo:
    module.SUMO = options.sumo
sys.argv = [sys.argv[0], *rest]
module.main()
