# Model C and D integration

This runner combines Model C routing from commit c91be80 with the Model D controller and final skip-stopped configuration supplied by the team. It keeps both model implementations and their settings intact. The comparison uses SUMO 1.28.0, seed 1, the official Route 1 and Route 2 origin-destination pairs, and Scenario 0 conditions S1 through S6.

## Run the comparison

Use Python 3.13 with the RouteX dependencies installed. From the repository root run:

    python dokki_sumo/experiments/model_c_d/src/run_model_c_d.py --route 1 --mode cd --output-tag cd_r1

Change the route to 2 for Route 2. Use a distinct output tag for each execution. On a machine where the original SUMO path is unavailable, pass --sumo with the full path to the SUMO 1.28.0 executable.

| Mode | Ambulance routing | Signal priority |
| --- | --- | --- |
| baseline | Scenario 0 | Off |
| c | Model C | Off |
| d | Scenario 0 | Model D |
| cd | Model C | Model D |

Use --scenarios S6 for a subset. The Route 1 Scenario 0 engine also runs its S2 reference as part of that subset. Seed 1 is fixed by the launcher.

## Integration order

In cd mode, the runner calls the original Model C step, then calls the original Model D controller. Model D obtains the current ambulance route from TraCI. In d mode the controller is called after the original Scenario 0 routing hook. The runner creates one controller per scenario and saves the existing Model D ETA, signal and intersection outputs before releasing its Python context. It also calls the controller's existing release method before closing TraCI if required.

Outputs are written under dokki_sumo/experiments/scenario_0/outputs by output tag. For connector parity, run cd with --controller-off and compare the resulting scenario summaries against c. For the supplied Model D unit tests, run:

    python dokki_sumo/experiments/model_d/tests/test_controller_fake_sumo.py
