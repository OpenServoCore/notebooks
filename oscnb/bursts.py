"""`osc-lab burst` captures: the ADC free running through PWM periods
around a duty step, `burst-K.csv` files. Column `code` is the raw sample,
`k` its index; the meta fields repeat on the first row. Samples alternate
the shunt with the taps, 26 ADC clocks apart at 24 MHz, so folding a burst
onto the PWM period draws the current and the terminals inside one window
at a resolution the crest sample never sees.
"""
from pathlib import Path

import numpy as np
import pandas as pd

TICKS_PER_ADC_CLK = 2
APERTURE_CLK = 13.5
CONVERSION_CLK = 26
STEP_TICKS = CONVERSION_CLK * TICKS_PER_ADC_CLK   # timer ticks from one sample to the next
TICKS_PER_US = 48


def load(folder):
    """Every burst in a folder, in capture order."""
    out = []
    for k in range(1000):
        p = Path(folder) / f"burst-{k}.csv"
        if not p.exists():
            break
        out.append(pd.read_csv(p))
    return out


def meta(df):
    return {k: int(df[k].iloc[0]) for k in df.columns[2:]}


def phase(df, arr):
    """(meta, phase, shunt mask, tap A mask, tap B mask): each sample's phase
    in the period, 0 at the trough, and which stream it belongs to."""
    m = meta(df)
    period = 2 * arr
    k = np.arange(len(df))
    phi0 = m["start_cnt"] if m["start_dir"] == 0 else period - m["start_cnt"]
    ph = (phi0 + STEP_TICKS * k) % period
    sh = k % 2 == 0
    if m["frame_len"] == 2:
        ta = (k % 2 == 1) if m["chans"] & 1 else np.zeros(len(k), bool)
        tb = (k % 2 == 1) if m["chans"] & 2 else np.zeros(len(k), bool)
    else:
        ta, tb = k % 4 == 1, k % 4 == 3
    return m, ph, sh, ta, tb


def fold(df, arr, stream, after_step=60):
    """(t, code, meta) of one stream ('sh', 'ta', 'tb') after the step,
    against the time its sample closed in ticks from the crest."""
    m, ph, sh, ta, tb = phase(df, arr)
    mask = {"sh": sh, "ta": ta, "tb": tb}[stream]
    k = np.arange(len(df))
    post = k >= m["step_index"] + after_step
    period = 2 * arr
    t = ((ph + APERTURE_CLK * TICKS_PER_ADC_CLK - arr + period // 2) % period) - period // 2
    sel = mask & post
    return t[sel], df["code"].to_numpy(float)[sel], m


def profile(t, y, lo, hi, w=4):
    """Median of y in w-tick bins of t from lo to hi, empty bins filled from
    their neighbours: a fold as one curve."""
    edges = np.arange(lo, hi + w, w)
    idx = np.clip(((t - lo) // w).astype(int), 0, len(edges) - 2)
    out = np.full(len(edges) - 1, np.nan)
    for i in range(len(out)):
        sel = idx == i
        if sel.any():
            out[i] = np.median(y[sel])
    s = pd.Series(out).interpolate(limit_direction="both")
    return edges[:-1] + w / 2, s.to_numpy()


def zero(df, arr):
    """The shunt's level before the step, the amplifier's zero for this burst."""
    m, ph, sh, ta, tb = phase(df, arr)
    k = np.arange(len(df))
    pre = k < m["step_index"] - 8
    return float(np.median(df["code"].to_numpy(float)[sh & pre]))
