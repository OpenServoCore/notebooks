"""A sweep recording: `sweep.csv` and `meta.json` as `osc-lab sweep` writes them.

One row per control tick at 20 kHz, segments numbered from 0 (the torque-off
baseline) through the rungs. The raw columns are ADC counts: `current_raw`
over the shunt amplifier, `vmotor_a` and `vmotor_b` the terminal taps,
`vbus_raw` the rail tap, `ntc_raw` the board NTC and `vref_raw` the chip's
1.2 V reference when the row carries them.
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
RAW = ["current_raw", "vmotor_a", "vmotor_b", "vbus_raw", "ntc_raw", "vref_raw"]
# A driven row whose crest current over the zero sits under this share of the window's mean is a dropout, a moment the
# motor's path stood open. One fixed fraction, so every notebook and the firmware count the same rows: a coefficient
# calibrated against the count only means something if the count's definition never moves. The commutation ripple's
# troughs sit 15 to 35 % under the mean, an open path reads near the zero, so the quarter sits in the gap between them.
DROPOUT_FRAC = 0.25


def dropout_rows(i_over_zero, ref=None):
    """the dropout rows of a driven window: the crest current over the zero under DROPOUT_FRAC of the window's mean, or
    of a running reference where the mean itself moves (a run-up)"""
    x = np.asarray(i_over_zero, float)
    return x < DROPOUT_FRAC * (x.mean() if ref is None else np.asarray(ref, float))


def load(folder, board=None, wide_codes=False):
    """(frame, meta) of one recording. With a board, its sense block is checked.
    With `wide_codes` the raw pot code is kept as `pos_raw` and `pos` carries
    the ADC's wide-code step of the board the meta names (`widecodes.compensated`)."""
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    df = pd.read_csv(folder / "sweep.csv")
    if board is not None:
        board.check_meta(meta, str(folder))
    arr = meta.get("pwm_period_ticks", 1200)
    df["duty_q15"] = (df["duty_counts"] * 32768 / arr).round()
    df["t"] = (df["tick"] - df["tick"].iloc[0]) / meta.get("tick_hz", 20_000)
    if wide_codes:
        from . import widecodes    # the bench stack it imports stays out of a plain load
        df["pos_raw"] = df["pos"].to_numpy()
        df["pos"] = widecodes.compensated(df["pos"], widecodes.slots_of_meta(meta))
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


def rest_zeros(df):
    """(current zero, tap difference zero): the baseline's medians."""
    rest = baseline(df)
    zero_i = rest["current_raw"].median()
    zero_e = (rest["vmotor_a"] - rest["vmotor_b"]).median()
    return zero_i, zero_e


def drive_start(seg):
    """The row of a segment where its drive starts, the first nonzero duty."""
    driven = np.flatnonzero(seg["duty_counts"].to_numpy() != 0)
    if len(driven) == 0:
        raise ValueError("the segment has no driven row")
    return int(driven[0])


def drive_rows(seg):
    """A segment's rows from its first driven one; the stream opens early."""
    return seg.iloc[drive_start(seg):]


def segment_edges(df):
    """The row positions where a new segment starts, for a sweep figure."""
    return np.flatnonzero(np.diff(df["seg"].to_numpy()) != 0) + 1


def direction_frame(df, segs):
    """(rows, {seg: first row}): the segments `segs` end to end."""
    parts = [df[df["seg"] == s] for s in segs]
    out = pd.concat(parts, ignore_index=True)
    starts = np.cumsum([0] + [len(p) for p in parts[:-1]])
    return out, dict(zip(segs, starts))


def flip_row(seg_rows):
    """(first row, held duty in ticks) of a chained segment: the stream
    opens before the new duty shows, so the first row showing the held
    duty, the median of the segment's second half, is where it starts."""
    d = seg_rows["duty_counts"].abs().to_numpy()
    held = np.median(d[len(d) // 2:])
    return int(np.flatnonzero(d == held)[0]), held


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
