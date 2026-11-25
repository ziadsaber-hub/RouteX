from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import subprocess
import sys

out=Path('../outputs/transfer_logs')
out.mkdir(parents=True,exist_ok=True)
pairs=[('r1',1,[]),('r2',2,[]),
       ('od158',1,['--origin','158','--destination','hospital_168']),
       ('od6',1,['--origin','6','--destination','hospital_95']),
       ('od115',1,['--origin','115','--destination','hospital_168'])]
def run(args):
    label,engine,od,mode=args
    tag=f'distance_guard_model_{label}' if mode=='model' else f'transfer_baseline_{label}'
    script='run_model_c_queue_recovery.py' if mode=='model' else 'run_shared_baseline.py'
    with (out/f'{tag}.log').open('w',encoding='utf-8') as f:
        rc=subprocess.run([sys.executable,'-B','-u',script,'--route',str(engine),
                         '--seed','2','--output-tag',tag,*od],
                         stdout=f,stderr=subprocess.STDOUT).returncode
    print(tag,rc,flush=True)
    return rc
with ThreadPoolExecutor(max_workers=5) as pool:
    jobs=[pool.submit(run,(label,engine,od,mode)) for label,engine,od in pairs for mode in ('baseline','model')]
    codes=[job.result() for job in as_completed(jobs)]
sys.exit(any(codes))
