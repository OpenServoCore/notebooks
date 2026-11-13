"""Synthetic `osc-lab track` captures for the scorecard's tests: the files
the tool writes (track.csv, segments.csv, meta.json) and the plan.json the
runner puts beside them, from rows a test chooses or from a fake servo
that answers a schedule.

A test fixture, not a model of record. `fake_track` runs each goal as a
first-order approach at a speed cap, its duty the error's sign at a fixed
share of the period, its current a fixed count per duty.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import boards, scorecard

HEADER = ("seg,goal,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,"
          "current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw").split(",")
SEG_HEADER = ("seg,kind,field,value,ms,tick0,rows,rows_dropped,t_start,t_end,vbus_raw,ntc_raw,"
              "theta_hat_q16,omega_hat_cps,t_winding_cc,i_lim_counts,limit_flags,fault_flags").split(",")
ZERO = 1210           # the shunt's raw zero
I_PER_DUTY = 0.2      # raw current counts per duty count
WINDOW_MIN = 64       # a drive window under this many duty counts reads no current
BOARD = boards.BOARDS[boards.DEFAULT]
GOAL_PERIOD_S = scorecard.testset().GOAL_PERIOD_S


def meta(center, guard, tick_hz, config=None, lut_state="IDENTITY"):
    return {"center": center, "guard": list(guard), "tick_hz": tick_hz, "ended": "ok",
            "config": config or {"p_kp_q88": 40212, "pos_deadband_counts": 4},
            "sense": BOARD.sense,
            "plant": {"lut_state": lut_state}, "pwm_period_ticks": 1200}


def write(out, segs, center, guard, tick_hz=20_000, tick0=60_000, gap_ticks=500,
          plan=None, config=None, lut_state="IDENTITY"):
    """One capture into `out`. `segs` are (item, goal, pos, duty) per
    schedule item, the arrays one value a row; tick0 starts at `tick0` and
    each segment opens `gap_ticks` after the last, wrapping at 2^16."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    frames, seg_rows = [], []
    t = tick0
    for k, (item, goal, pos, duty) in enumerate(segs):
        n = len(pos)
        duty = np.asarray(duty, int)
        valid = np.abs(duty) >= WINDOW_MIN
        raw = np.round(ZERO + np.abs(duty) * I_PER_DUTY * valid).astype(int)
        frames.append(pd.DataFrame({
            "seg": k, "goal": goal, "tick": np.arange(n), "window_valid": valid.astype(int),
            "pos": np.round(pos).astype(int), "current_trough": ZERO, "duty_counts": duty,
            "current_raw": raw, "vmotor_a": 600, "vmotor_b": 600}))
        kind, rest = item.split("@")[0], item.split("@")[1]
        value = kind.split(":")[1] if kind.startswith("goal:") else ""
        seg_rows.append({"seg": k, "kind": "goal" if value else "hold", "value": value, "ms": rest,
                         "tick0": t % 65536, "rows": n, "rows_dropped": 0,
                         "t_start": 0.0, "t_end": 0.0})
        t += n + gap_ticks
    pd.concat(frames).reindex(columns=HEADER).to_csv(out / "track.csv", index=False)
    pd.DataFrame(seg_rows).reindex(columns=SEG_HEADER).to_csv(out / "segments.csv", index=False)
    (out / "meta.json").write_text(json.dumps(meta(center, guard, tick_hz, config, lut_state)))
    if plan is not None:
        (out / "plan.json").write_text(json.dumps(plan))
    return out


def rows_of(ms, tick_hz):
    return int(ms * tick_hz / 1000)


def fake_track(out, schedule, start, tick_hz=2_000, tau_s=0.03, max_cps=1500.0,
               duty_counts=600, deadband=4):
    """A servo that answers `schedule` from `start` counts; writes the
    capture (no plan) and returns where the shaft ended."""
    plan = json.loads((Path(out) / "plan.json").read_text())
    center, guard = plan["center_counts"], plan["guard_counts"]
    dt = 1 / tick_hz
    pos = float(start)
    goal = round(start)
    segs = []
    for item in schedule:
        n = rows_of(int(item.split("@")[1]), tick_hz)
        if item.startswith("goal:"):
            goal = int(item.split(":")[1].split("@")[0])
        p = np.empty(n)
        d = np.empty(n, int)
        for i in range(n):
            err = goal - pos
            step = np.clip(err * dt / tau_s, -max_cps * dt, max_cps * dt)
            pos += step
            p[i] = pos
            d[i] = 0 if abs(err) <= deadband else int(np.sign(err)) * duty_counts
        segs.append((item, goal, p, d))
    # the burst and the host's overhead, as a 20 ms goal item really runs
    gap = round((GOAL_PERIOD_S - 0.020) * tick_hz)
    write(out, segs, center, guard, tick_hz=tick_hz, gap_ticks=gap, plan=plan)
    return pos
