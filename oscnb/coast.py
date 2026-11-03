"""A spin and its coast: the zero-command segment a sweep chains to a spin.

The coast's stream opens with the drive's tail, so the cut, the first row
with no drive, starts the EMF. With the bridge open the tap difference over
its rest value is the winding's EMF, signed by the spin's direction so it
reads positive. A brush that lifts off its segment opens the path for a row
or two and the taps read near the zero there, an open row; bridged by a line
between its neighbors, the EMF reads as the winding carried it.

Rows count from the cut. The defaults are the reads every notebook makes.
"""
import numpy as np

from . import ripple

COAST_FROM_ROWS = 20    # 1 ms past the cut, after the taps settle
SEED_ROWS = 200         # 10 ms of the early coast, where the seed is read
EARLY_ROWS = slice(2, 20)   # 0.1 to 1 ms past the cut: did the rotor turn
TURN_V = 0.05           # V, an EMF under this: the rotor has about stopped
NEIGHBORHOOD_ROWS = 41  # about 2 ms, the stretch an EMF row is compared with
OPEN_FRAC = 0.3         # a row under this share of its neighborhood is open


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
