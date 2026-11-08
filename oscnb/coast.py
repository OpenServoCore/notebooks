"""A spin and its coast: the zero-command segment a sweep chains to a spin.

The coast's stream opens with the drive's tail, so the cut, the first row
with no drive, starts the EMF. With the bridge open the tap difference over
its rest value is the winding's EMF, signed by the spin's direction so it
reads positive. A brush that lifts off its segment opens the path for a row
or two and the taps read near the zero there, an open row; bridged by a line
between its neighbors, the EMF reads as the winding carried it.

The deceleration reduction fits a quadratic of the bridged EMF on time
every few rows down to the stop, its level over Ke the speed and its slope
over Ke the deceleration, and projects a line through the early coast back
to the cut, E0.

Rows count from the cut. The defaults are the reads every notebook makes.
"""
import numpy as np
import pandas as pd

from . import ripple

COAST_FROM_ROWS = 20    # 1 ms past the cut, after the taps settle
SEED_ROWS = 200         # 10 ms of the early coast, where the seed is read
EARLY_ROWS = slice(2, 20)   # 0.1 to 1 ms past the cut: did the rotor turn
TURN_V = 0.05           # V, an EMF under this: the rotor has about stopped
NEIGHBORHOOD_ROWS = 41  # about 2 ms, the stretch an EMF row is compared with
OPEN_FRAC = 0.3         # a row under this share of its neighborhood is open
DECEL_HALF_S = 0.005    # s, the +/- half width of a deceleration fit
DECEL_STEP_S = 0.0005   # s, a fit center every 0.5 ms
E0_ROWS = 100           # 5 ms from COAST_FROM_ROWS, the line E0 projects
DECEL_BIN = 10.0        # rad/s, the speed bin of a deceleration curve


def pairs(df):
    """[(direction, spin rows, coast rows)]: a coast follows its spin."""
    out = []
    for seg, coast in df[df["seg"] > 0].groupby("seg"):
        if int(coast["cmd_duty_q15"].iloc[0]) != 0:
            continue
        spin = df[df["seg"] == seg - 1]
        out.append((int(coast["dir"].iloc[0]), spin, coast))
    return out


def cut_row(coast):
    """The coast's first row with no drive, None if the drive never stops."""
    idle = np.flatnonzero(coast["duty_counts"].to_numpy() == 0)
    return int(idle[0]) if len(idle) else None


def emf_from_cut(coast, direction, cut, zero_counts, v_per_count=None,
                 counts_per_v=None):
    """The EMF in volts from the cut: times v_per_count or over counts_per_v."""
    taps = (coast["vmotor_a"] - coast["vmotor_b"]) * direction
    over = taps.to_numpy(float)[cut:] - zero_counts
    if (v_per_count is None) == (counts_per_v is None):
        raise ValueError("give exactly one of v_per_count and counts_per_v")
    if v_per_count is None:
        return over / counts_per_v
    return over * v_per_count


def early_emf(emf, rows=EARLY_ROWS):
    """The EMF's mean just past the cut, what says the rotor turned."""
    return emf[rows].mean()


def local_mean(emf, rows=NEIGHBORHOOD_ROWS):
    """The EMF's running mean over `rows`, centered."""
    return np.convolve(emf, np.ones(rows) / rows, mode="same")


def open_rows(emf, start=COAST_FROM_ROWS, neighborhood=NEIGHBORHOOD_ROWS,
              frac=OPEN_FRAC, turn_v=TURN_V):
    """The rows under `frac` of their neighborhood while the rotor turns."""
    ref = local_mean(emf, neighborhood)
    bad = (emf < frac * ref) & (ref > turn_v)
    bad[:start] = False
    return bad


def bridge(emf, bad):
    """The EMF with each open row a line between its neighbors."""
    if not bad.any():
        return emf.copy()
    x = np.arange(len(emf))
    out = emf.copy()
    out[bad] = np.interp(x[bad], x[~bad], emf[~bad])
    return out


def stop_row(emf, start=COAST_FROM_ROWS, neighborhood=NEIGHBORHOOD_ROWS,
             turn_v=TURN_V):
    """The first row from `start` the running mean falls under turn_v."""
    below = np.flatnonzero(local_mean(emf, neighborhood)[start:] < turn_v)
    return start + int(below[0]) if len(below) else None


def seed_freq(emf, start=COAST_FROM_ROWS, rows=SEED_ROWS, fs=ripple.ROWS_HZ):
    """The early coast's ripple line, the seed of every count on it."""
    return ripple.line_freq(emf[start:start + rows], fs=fs)


def coast_decel(emf, bad, stop, ke, half=DECEL_HALF_S, step=DECEL_STEP_S,
                start=COAST_FROM_ROWS, fs=ripple.ROWS_HZ):
    """One row per fit center down to `stop`: the bridged EMF's level and
    slope, as rad/s and rad/s^2 through ke, and the open rows under it."""
    t = np.arange(len(emf)) / fs
    rows = []
    h = int(half * fs)
    for j in range(start + h, stop - h, int(step * fs)):
        seg = slice(j - h, j + h + 1)
        p = np.polyfit(t[seg] - t[j], emf[seg], 2)
        rows.append({"t_coast": t[j], "emf": p[2], "dedt": p[1],
                     "w": p[2] / ke, "alpha": p[1] / ke,
                     "open_rows": int(bad[seg].sum())})
    return pd.DataFrame(rows)


def coast_table(emf, ke, per_rev, fs=ripple.ROWS_HZ):
    """(deceleration table, reads) of a coast's EMF from the cut, the open
    rows bridged: E0, the speed at the cut, the stop and the first line."""
    bad = open_rows(emf)
    e = bridge(emf, bad)
    stop = stop_row(e)
    stop = len(e) if stop is None else stop
    t = np.arange(len(e)) / fs
    tab = coast_decel(e, bad, stop, ke, fs=fs)
    early = slice(COAST_FROM_ROWS, COAST_FROM_ROWS + E0_ROWS)
    e0 = np.polyfit(t[early], e[early], 1)[1]
    f_seed = seed_freq(e, fs=fs)
    return tab, {"e0": e0, "w_cut_emf": e0 / ke / (2 * np.pi),
                 "stop_ms": stop / fs * 1e3,
                 "open_rows_coast": int(bad[:stop].sum()),
                 "coast_line_rev_s": f_seed / per_rev, "emf": e, "bad": bad}


def coast_decel_fn(table, bin_w=DECEL_BIN):
    """(a_c(w), top speed) from a coast's deceleration table: the median
    deceleration per speed bin, interpolated, the end values held."""
    h = table.sort_values("w")
    w, a = h["w"].to_numpy(), h["alpha"].to_numpy()
    bins = np.arange(0, w.max() + bin_w, bin_w)
    idx = np.digitize(w, bins)
    ws, as_ = [], []
    for b in np.unique(idx):
        m = idx == b
        ws.append(w[m].mean())
        as_.append(np.median(a[m]))
    ws, as_ = np.array(ws), np.array(as_)

    def fn(x):
        return np.interp(x, ws, as_, left=as_[0], right=as_[-1])
    return fn, ws.max()
