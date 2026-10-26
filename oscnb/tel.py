"""A sweep recording: `sweep.csv` and `meta.json` as `osc sweep` writes them.

One row per control tick at 20 kHz, segments numbered from 0 (the torque-off
baseline) through the rungs. The raw columns are ADC counts: `current_raw`
over the shunt amplifier, `vmotor_a` and `vmotor_b` the terminal taps,
`vbus_raw` the rail tap, `ntc_raw` the board NTC when the row carries it.
`duty_counts` is the applied window in timer ticks either side of the crest,
signed by direction. A dropped frame shows as a jump in `tick`.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from . import boards

FRAME_ROWS = 16
SETTLE_ROWS = 40      # 2 ms after a rung opens, for the rail and the taps to settle
RAW = ["current_raw", "vmotor_a", "vmotor_b", "vbus_raw", "ntc_raw"]


def load(folder, board=None):
    """(frame, meta) of one recording. With a board, its sense block is checked."""
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    df = pd.read_csv(folder / "sweep.csv")
    if board is not None:
        board.check_meta(meta, str(folder))
    arr = meta.get("pwm_period_ticks", 1200)
    df["duty_q15"] = (df["duty_counts"] * 32768 / arr).round()
    df["t"] = (df["tick"] - df["tick"].iloc[0]) / meta.get("tick_hz", 20_000)
    return df, meta


def board_of(meta):
    b = boards.from_meta(meta)
    if b is None:
        raise ValueError(f"no board entry matches the sense block {meta.get('sense')}")
    return b


def gaps(df):
    """Missing samples per segment, from tick steps other than one."""
    step = df.groupby("seg")["tick"].diff().dropna()
    bad = step[step != 1] - 1
    return pd.Series(bad.groupby(df.loc[bad.index, "seg"]).sum(), name="missing").astype(int)


def baseline(df):
    """The torque-off segment's second half, after anything the arm disturbed."""
    b = df[df["seg"] == 0]
    return b.iloc[len(b) // 2:]


def settled(g, skip=SETTLE_ROWS):
    """The rows of one rung after its duty reached the held value and at
    least `skip` rows in. Under the current limit the duty climbs from the
    window floor, so the climb is left out."""
    held = g["duty_counts"].iloc[len(g) // 2:].median()
    at = np.flatnonzero(g["duty_counts"].to_numpy() == held)
    start = max(skip, int(at[0]) if len(at) else 0)
    return g.iloc[start:]


def rungs(df, skip=SETTLE_ROWS):
    """One row per driven segment: the commanded duty, the direction, the
    applied window, and the mean and standard deviation of each raw column
    over the settled part."""
    rows = []
    for seg, g in df[df["seg"] > 0].groupby("seg"):
        s = settled(g, skip)
        if len(s) < 20:
            continue
        row = {"seg": seg, "cmd_duty_q15": int(g["cmd_duty_q15"].iloc[0]), "dir": int(g["dir"].iloc[0]),
               "duty_ticks": float(s["duty_counts"].abs().median()), "n": len(s)}
        for c in RAW:
            if c in s and s[c].notna().any():
                row[c] = float(s[c].mean())
                row[c + "_sd"] = float(s[c].std())
        rows.append(row)
    return pd.DataFrame(rows)
