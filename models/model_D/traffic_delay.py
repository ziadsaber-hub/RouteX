import math

def extra_red(t_request_s, t_release_s, r_normal_s):          # eq. I (never negative)
    return max(0.0, (t_release_s - t_request_s) - r_normal_s)

def approach_cost(lam, dr, n, L_e, s=0.5, l_veh=7.5):
    mu = s * n
    N = lam * dr                                               # eq. IV (queue at end of extra red)
    out = dict(queue_veh=N, spillback=N >= L_e / l_veh, recoverable=lam < mu)
    if lam < mu:                                               # discharge capacity must exceed arrivals
        out["t_rec_s"] = lam * dr / (mu - lam)                 # eq. III (seconds)
        # triangle build-up lam*dr^2/2 + triangle drain N*T_rec/2 = lam*dr^2/2 * mu/(mu-lam)
        out["delay_veh_s"] = lam * dr**2 / 2 * mu / (mu - lam) # eq. II corrected (veh-s)
    else:
        out["t_rec_s"] = out["delay_veh_s"] = math.inf
    return out

def intersection_cost(dr, conflicts, s=0.5, l_veh=7.5):
    """conflicts: list of dict(lam, n, L_e). Returns separate recovery time (s) and delay (veh-s)."""
    rs = [approach_cost(c["lam"], dr, c["n"], c["L_e"], s, l_veh) for c in conflicts]
    flags = (["no_recovery"] if any(not r["recoverable"] for r in rs) else []) + \
            (["spillback"] if any(r["spillback"] for r in rs) else [])
    return dict(ordinary_delay_veh_s=sum(r["delay_veh_s"] for r in rs),
                t_rec_s=max((r["t_rec_s"] for r in rs), default=0.0), flags=flags)
