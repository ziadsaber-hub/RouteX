from pathlib import Path
import os, sys
if "SUMO_HOME" in os.environ: sys.path.append(os.path.join(os.environ["SUMO_HOME"],"tools"))
import traci
HERE=Path(__file__).resolve().parent
os.chdir(HERE)
CFG=HERE/"dokki_visual.sumocfg"
traci.start(["sumo-gui","-c",str(CFG)])
ids={"123":"visual_tls_123","142":"visual_tls_142","172":"visual_tls_172","203":"visual_tls_203","28":"visual_tls_28","31":"visual_tls_31"}
last={}
try:
 while traci.simulation.getMinExpectedNumber()>0:
  traci.simulationStep()
  for tls,poi in ids.items():
   s=traci.trafficlight.getRedYellowGreenState(tls)
   # Controller summary: green takes precedence, then yellow, otherwise red.
   state="green" if any(c in "Gg" for c in s) else ("yellow" if any(c in "Yy" for c in s) else "red")
   if last.get(tls)!=state:
    traci.poi.setImageFile(poi,f"icons/traffic_{state}.png"); last[tls]=state
finally: traci.close()
