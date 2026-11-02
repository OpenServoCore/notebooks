"""Stage 2, rest-resistance: the motor's terminal resistance at rest, over
the rotor's angle, from the board's own current and terminal taps. A
locked-rotor plan, run by `python -m oscnb.sitting locked`, which steps the
wrench bar through the twelve holes of the rig's bolt circle so a reading
belongs to a rotor angle, the same angle the inductance bursts of the same
sitting read. At each hole it drives 100 % laps into the locked rotor at
the three 1S rails, both directions, in laps short enough to leave the
copper cold. The dataset is `capture-N/stall`, `laps.csv` with one row per
lap and `hole-K/<rail>V` sweeps, as the notebook reads it.

The meter's routes for the teardown's brush path, `ohms` and `shake`, are
hand procedures with a logger and stay in `capture.py` beside this file.
"""
import csv
import time

from oscnb import osc
from oscnb.sitting import Plan

LEVELS = [3.3, 3.7, 4.2]     # rail volts, the 1S levels: the stall current stays under an amp
LAP_MS = 300                 # 100 % into the locked rotor, cold: under a joule per lap
_LAPS = {}                   # the open laps.csv of each dataset through the holes


def setup(s, out):
    stall = out / "stall"
    stall.mkdir(exist_ok=True)
    f = open(stall / "laps.csv", "w", newline="")
    w = csv.writer(f)
    w.writerow(["hole", "volts_set", "dps_read", "vsys_read", "t_start", "t_end", "folder"])
    _LAPS[out] = (f, w)


def at_hole(s, out, k):
    f, w = _LAPS[out]
    for v in LEVELS:
        dps_read, vsys = s.rail(v)
        folder = f"hole-{k}/{v:g}V"
        t0 = time.time()
        osc.sweep(out / "stall" / folder, [f"100@{LAP_MS}"], baseline_ms=500, rest_ms=1000, dirs="both")
        w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", f"{t0:.3f}", f"{time.time():.3f}", folder])
        f.flush()


def finish(s, out):
    _LAPS.pop(out)[0].close()


PLANS = [Plan("capture", "rest-resistance", "locked", "100 % laps at 3.3, 3.7 and 4.2 V into every hole", setup=setup, at_hole=at_hole,
              finish=finish, order=10)]
