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


Q15_FULL = 32768          # a Q15 duty of 1.0
# the median's standard error over the mean's on normal data, sqrt(pi / 2)
MEDIAN_SE = 1.2533
MIN_SE_SAMPLES = 3        # a bin's standard error needs this many samples
BRAKE_GUARD_TICKS = 10    # the brake read stops this far before the window


def duty_pct(m):
    """A burst's commanded duty in whole percent, from its Q15 step."""
    return int(round(abs(m["step_q15"]) * 100 / Q15_FULL))


def samples(df, board, center):
    """(frame, meta, half): every sample on the grid of full windows."""
    if center not in ("crest", "trough"):
        raise ValueError(f"center is 'crest' or 'trough', not {center!r}")
    arr = board.pwm_arr
    period = 2 * arr
    m, ph, sh, ta, tb = phase(df, arr)
    half = int(board.drive_ticks(m["step_q15"]))
    k = np.arange(len(df))
    if m["start_dir"] == 0:
        phi0 = m["start_cnt"]
    else:
        phi0 = period - m["start_cnt"]
    tick = phi0 + STEP_TICKS * k + APERTURE_CLK * TICKS_PER_ADC_CLK
    # the duty written at the step takes effect at the next trough: the grid
    # starts at the crest window after it, or at the trough window a period
    # later, the first trough's window holding only its second half
    trough = period * np.ceil(tick[m["step_index"]] / period)
    c = arr if center == "crest" else period
    origin = trough + c - half
    n = np.floor((tick - origin) / period).astype(int)
    last = int(np.floor((tick[-1] - origin) / period))
    # ticks from the nearest window center: the crest in slow decay, the
    # trough in fast
    tc = ((tick - c) % period + period // 2) % period - period // 2
    stream = np.where(sh, "sh", np.where(ta, "ta", "tb"))
    s = pd.DataFrame({"k": k, "stream": stream,
                      "code": df["code"].to_numpy(float), "tick": tick,
                      "tc": tc, "n": n,
                      "t_us": (tick - origin) / TICKS_PER_US,
                      "full": n < last})
    return s, m, half


def term_diff(s, sign, scales):
    """(tc, n, volts): tap A less tap B at each tap A sample, B interpolated."""
    a = s[s["stream"] == "ta"]
    b = s[s["stream"] == "tb"]
    vb = np.interp(a["tick"], b["tick"], b["code"])
    diff = sign * (a["code"].to_numpy() - vb)
    volts = scales.volts(diff, scales.terminal_offset_counts)
    return a["tc"].to_numpy(), a["n"].to_numpy(), volts


def binned(x, y, lo, hi, w):
    """(centers, median, median's standard error, count) of y in w-wide bins."""
    edges = np.arange(lo, hi + w, w)
    idx = np.floor((x - lo) / w).astype(int)
    ok = (idx >= 0) & (idx < len(edges) - 1)
    centers = edges[:-1] + w / 2
    med = np.full(len(centers), np.nan)
    se = np.full(len(centers), np.nan)
    count = np.zeros(len(centers), int)
    for i, g in pd.DataFrame({"i": idx[ok], "y": y[ok]}).groupby("i"):
        med[i] = g["y"].median()
        count[i] = len(g)
        if len(g) >= MIN_SE_SAMPLES:
            se[i] = MEDIAN_SE * g["y"].std(ddof=1) / np.sqrt(len(g))
    return centers, med, se, count


def period_table(df, sign, board, scales, lags=None, edge=0.0, drive_off=0.0):
    """One row per full period of a slow-decay burst, inductance's read."""
    lags = scales.edge_lags() if lags is None else lags
    period = 2 * board.pwm_arr
    counts_per_a = scales.counts_per_a
    s, m, half = samples(df, board, "crest")
    zero_counts = zero(df, board.pwm_arr)
    a, b = -half + lags["tap_on"], half + lags["off"]
    on_span = b - a
    s = s[(s["n"] >= 0) & s["full"]].assign(y=lambda d: d["code"] - zero_counts)
    settled = (s["tc"] >= -half + lags["settle"]) & (s["tc"] <= half)
    clear = ((s["tc"] >= b + lags["settle"] - lags["on"])
             | (s["tc"] <= -half - BRAKE_GUARD_TICKS))
    rows = []
    for n, g in s.groupby("n"):
        sh = g[(g["stream"] == "sh") & settled[g.index]]
        line = np.polyfit(sh["tc"], sh["y"], 1)
        i_on = (np.polyval(line, (a + b) / 2) - drive_off) / counts_per_a
        off = g[(g["stream"] == "sh") & clear[g.index]]
        i_off = off["y"].mean() / counts_per_a
        volts = []
        for w in (settled, clear):
            tap_a = g[(g["stream"] == "ta") & w[g.index]]["code"].mean()
            tap_b = g[(g["stream"] == "tb") & w[g.index]]["code"].mean()
            d = sign * (tap_a - tap_b) - scales.terminal_offset_counts
            volts.append(d / scales.terminal_counts_per_v)
        v_on, v_off = volts
        # `edge`: the volt-ticks the two edges hold beyond the two-part mean
        v_mean = (v_on * on_span + v_off * (period - on_span) + edge) / period
        slope = line[0] * TICKS_PER_US * 1e6 / counts_per_a
        rows.append({"n": n,
                     "t_us": (n * period + half + (a + b) / 2) / TICKS_PER_US,
                     "i_a": i_on, "v_v": v_mean, "v_on_v": v_on,
                     "v_off_v": v_off, "i_off_a": i_off,
                     "slope_a_per_s": slope, "on_n": len(sh),
                     "code_lo": sh["code"].min(), "code_hi": sh["code"].max()})
    return pd.DataFrame(rows)
