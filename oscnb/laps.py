"""DC laps: 100 % duty held on a resistor load at a set of supply levels.

At 100 % the bridge stops switching, so a meter on any node and the chip's
own stream read the same DC quantity, and the difference is the chain's.
The capture side steps the DPS through the levels and records one sweep per
level, both directions, with a torque-off baseline before each. The
analysis side turns each lap into one row per direction: the settled means
of the raw columns, the baseline zero, and whatever the meter read in that
window.

Supply levels are rail volts, VSYS, and cover the three supplies a servo
meets: a 1S pack from empty to full, USB, and a 2S pack from its floor to
full. A swap board has no feed diode and its VSYS is the battery itself, so
the levels name the rail, and the capture trims the DPS over the dev
board's diode until the rail reads the level. The sweep tool warns when a
rail reads like a flat 2S pack; the DPS is not one.
"""
import csv
import time
from pathlib import Path

import numpy as np
import pandas as pd

from . import bench, osc, tel

LEVELS_1S = [3.3, 3.7, 4.2]
LEVELS_USB = [4.5, 5.0, 5.5]
LEVELS_2S = [7.2, 7.6, 8.0]
LEVELS_BARE = LEVELS_1S + LEVELS_USB + [7.2, 7.6]     # the bare load stays under its 12 W
LEVELS_ALL = LEVELS_1S + LEVELS_USB + LEVELS_2S
LAP_MS = 3000
LIMIT_COUNTS = 2000     # about 1.8 A, over any lap here; the saved limit comes back after


def run(out, levels, load, lap_ms=LAP_MS, chunks=None, log=print):
    """One lap per level into `out/laps/K`, the rail trimmed to each level
    with the DPS output left on, `levels.csv` with the host time of each
    lap. `levels` are VSYS volts at rest; `volts_set` is the level,
    `dps_read` what the DPS put out to reach it, `vsys_read` what the
    board's tap read at rest before the lap."""
    out = Path(out)
    (out / "laps").mkdir(parents=True, exist_ok=True)
    with open(out / "levels.csv", "a", newline="") as f:
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(["k", "load", "volts_set", "dps_read", "vsys_read", "t_start", "t_end", "folder"])
        k0 = len(list((out / "laps").glob("*")))
        for i, v in enumerate(levels):
            k = k0 + i
            dps_read, vsys = bench.set_vsys(v)
            log(f"lap {k}: {load} at VSYS {v} V (rail reads {vsys:.3f} V, DPS {dps_read:.3f} V)")
            t0 = time.time()
            osc.sweep(out / "laps" / str(k), [f"100@{lap_ms}"], baseline_ms=1000, rest_ms=2000,
                      dirs="both", tel_chunks=chunks)
            w.writerow([k, load, v, f"{dps_read:.3f}", f"{vsys:.3f}", f"{t0:.3f}", f"{time.time():.3f}", f"laps/{k}"])
            f.flush()
    return out


def table(folder, board=None, dmm=None, dmm_margin_s=0.7):
    """One row per lap and direction. `dmm` is a DMM log frame (t_host,
    value); its median inside the lap's driven window joins as `meter`."""
    folder = Path(folder)
    levels = pd.read_csv(folder / "levels.csv")
    rows = []
    for _, L in levels.iterrows():
        df, meta = tel.load(folder / L["folder"], board)
        b = tel.baseline(df)
        zero = float(b["current_raw"].median())
        vb = float(pd.concat([b["vmotor_a"], b["vmotor_b"]]).median())
        rail_rest = float(b["vbus_raw"].mean())
        r = tel.rungs(df)
        for _, g in r.iterrows():
            seg = df[df["seg"] == g["seg"]]
            t_on = L["t_start"] + seg["t"].iloc[0] + (meta.get("baseline_ms", 1000) / 1e3 if "t_start" in L else 0)
            row = {"k": int(L["k"]), "load": L["load"], "volts_set": L["volts_set"], "vsys_read": L.get("vsys_read", np.nan),
                   "dir": int(g["dir"]), "duty_ticks": g["duty_ticks"], "zero": zero, "vb_rest": vb,
                   "rail_rest": rail_rest, "current_raw": g["current_raw"], "vmotor_a": g["vmotor_a"],
                   "vmotor_b": g["vmotor_b"], "vbus_raw": g["vbus_raw"], "current_sd": g["current_raw_sd"],
                   "n": g["n"]}
            if "ntc_raw" in g and np.isfinite(g.get("ntc_raw", np.nan)):
                row["ntc_raw"] = g["ntc_raw"]
            if dmm is not None:
                # the lap's host window: the sweep's own clock is not logged, so the
                # window is the level's span less a margin either side
                lo, hi = L["t_start"] + dmm_margin_s, L["t_end"] - dmm_margin_s
                m = dmm[(dmm["t_host"] >= lo) & (dmm["t_host"] <= hi)]["value"].dropna()
                row["meter"] = float(m.median()) if len(m) else np.nan
                row["meter_n"] = int(len(m))
            rows.append(row)
    return pd.DataFrame(rows)


def line(x, y):
    """Slope, intercept and their standard errors of y on x by least squares."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    n = len(x)
    slope, icpt = np.polyfit(x, y, 1)
    resid = y - (slope * x + icpt)
    s2 = (resid ** 2).sum() / max(n - 2, 1)
    sxx = ((x - x.mean()) ** 2).sum()
    return {"slope": float(slope), "intercept": float(icpt),
            "slope_se": float(np.sqrt(s2 / sxx)) if sxx > 0 else np.nan,
            "intercept_se": float(np.sqrt(s2 * (1 / n + x.mean() ** 2 / sxx))) if sxx > 0 else np.nan,
            "rms": float(np.sqrt((resid ** 2).mean())), "n": n}
