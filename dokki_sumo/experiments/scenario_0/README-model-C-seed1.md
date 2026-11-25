# RouteX Model C seed 1 delivery

Aggregate waiting in the live edge score is weighted by 0.5 before the 180 s cap. This reduces the influence of stale aggregate waits while retaining queue length, occupancy and observed travel time.

Use Python 3.13 and SUMO 1.28.0. Install requirements.txt and open dokki_sumo/experiments/scenario_0/src.

    python run_official_model_c.py --route 1 --seed 1 --sumo "C:/Program Files (x86)/SUMO/bin/sumo.exe"
    python run_official_model_c.py --route 2 --seed 1 --sumo "C:/Program Files (x86)/SUMO/bin/sumo.exe"
    python run_official_baseline.py --route 1 --seed 1 --sumo "C:/Program Files (x86)/SUMO/bin/sumo.exe"
    python run_official_baseline.py --route 2 --seed 1 --sumo "C:/Program Files (x86)/SUMO/bin/sumo.exe"

Use --output-tag to keep successive simulations separate. Both runners use official_disruptions.json.
