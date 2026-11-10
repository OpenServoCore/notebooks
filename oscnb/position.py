"""Position estimators: the pot's counts to the output angle, row by row.

Pure functions, numpy in and numpy out, float only, one sample per control
tick; the fixed-point port is the firmware's own work. Each row estimator's
docstring names the state it keeps and its delay in ticks.

The table is a lookup linear between knots, in one of two forms:
  "counts": knots uniform in counts (a power-of-two pitch makes the index a
            shift), values in deg. The firmware form.
  "angle":  knots uniform in angle, values in counts; read backwards, so the
            lookup needs a search.
Both are fitted directly on frames with stage 3's model, pos = T(theta) +
c_s: one free offset per moving segment, the rest points at offset 0.
Outside its knots a lookup follows an end line.

Around the table sit the row estimators: a filter (boxcar, one_pole,
median3), a mask for the converter's carry codes (carry_hold), a direction
source (dir_from_duty, dir_from_pot) and the lash, as a play operator or a
fixed offset per direction (play, lash_offset).
"""
import numpy as np
from scipy.signal import lfilter


def uniform_knots(lo, hi, pitch):
    """Knots on multiples of pitch covering lo..hi."""
    a = np.floor(lo / pitch) * pitch
    b = np.ceil(hi / pitch) * pitch
    return np.arange(a, b + pitch / 2, pitch)


def hat(x, knots):
    """Linear-interpolation weights of x on uniform knots, shape (n, K)."""
    x = np.clip(np.asarray(x, float), knots[0], knots[-1])
    u = (x - knots[0]) / (knots[1] - knots[0])
    i = np.minimum(u.astype(int), len(knots) - 2)
    w = u - i
    A = np.zeros((len(x), len(knots)))
    r = np.arange(len(x))
    A[r, i] = 1 - w
    A[r, i + 1] = w
    return A


def fit_pwl(x, y, sid, x_rest, y_rest, knots, ridge=0.1):
    """y = PWL(x) + offset[sid] on moving frames, rest points at offset 0.

    ridge weights the knots' second difference against one frame's
    residual. At 0.1 it only conditions knots few frames reach (they
    continue their neighbors' line; no measurable effect at a 32 count
    pitch); at 1 it smooths, which pitches of 16 counts and under need to
    stay monotone. Returns (values, {sid: offset}).
    """
    sids, inv = np.unique(np.asarray(sid), return_inverse=True)
    n, nr, K, S = len(x), len(x_rest), len(knots), len(sids)
    A = np.zeros((n + nr + K - 2, K + S))
    A[:n, :K] = hat(x, knots)
    A[np.arange(n), K + inv] = 1.0
    A[n:n + nr, :K] = hat(x_rest, knots)
    j = np.arange(K - 2)
    A[n + nr + j, j] = ridge
    A[n + nr + j, j + 1] = -2 * ridge
    A[n + nr + j, j + 2] = ridge
    b = np.r_[np.asarray(y, float), np.asarray(y_rest, float),
              np.zeros(K - 2)]
    sol = np.linalg.lstsq(A, b, rcond=None)[0]
    return sol[:K], dict(zip(sids, sol[K:]))


class Lookup:
    """counts <-> angle, linear between knots, end lines outside them.
    Stateless, no delay."""

    def __init__(self, kind, knots, values, lo_slope=None, hi_slope=None):
        self.kind = kind
        knots, values = np.asarray(knots, float), np.asarray(values, float)
        self.c, self.a = ((knots, values) if kind == "counts"
                          else (values, knots))
        self.monotone = bool((np.diff(self.c) > 0).all()
                             and (np.diff(self.a) > 0).all())
        self.entries = len(knots)
        # end lines in counts/deg; default = the last cell's own slope
        self.lo_slope = lo_slope or ((self.c[1] - self.c[0])
                                     / (self.a[1] - self.a[0]))
        self.hi_slope = hi_slope or ((self.c[-1] - self.c[-2])
                                     / (self.a[-1] - self.a[-2]))

    def angle(self, pos):
        """deg at pot counts `pos`."""
        pos = np.asarray(pos, float)
        out = np.interp(pos, self.c, self.a)
        out = np.where(pos < self.c[0],
                       self.a[0] + (pos - self.c[0]) / self.lo_slope, out)
        return np.where(pos > self.c[-1],
                        self.a[-1] + (pos - self.c[-1]) / self.hi_slope, out)

    def counts(self, theta):
        """Pot counts at `theta` deg."""
        theta = np.asarray(theta, float)
        out = np.interp(theta, self.a, self.c)
        out = np.where(theta < self.a[0],
                       self.c[0] + (theta - self.a[0]) * self.lo_slope, out)
        return np.where(theta > self.a[-1],
                        self.c[-1] + (theta - self.a[-1]) * self.hi_slope,
                        out)

    def scale(self):
        """Mean counts/deg over the table."""
        return (self.c[-1] - self.c[0]) / (self.a[-1] - self.a[0])


def build(moving, rest, pitch, kind="counts", ridge=0.1):
    """Fit a Lookup at a pitch (counts for "counts", deg for "angle") on
    frames (columns pos, image_deg, sid) and rest points (pos, image_deg).
    Returns (Lookup, offsets); offsets in deg ("counts") or counts
    ("angle")."""
    xk, yk = ("pos", "image_deg") if kind == "counts" else ("image_deg", "pos")
    if len(rest):
        lo = min(moving[xk].min(), rest[xk].min())
        hi = max(moving[xk].max(), rest[xk].max())
    else:
        lo, hi = moving[xk].min(), moving[xk].max()
    knots = uniform_knots(lo, hi, pitch)
    v, off = fit_pwl(moving[xk].to_numpy(), moving[yk].to_numpy(),
                     moving["sid"].to_numpy(), rest[xk].to_numpy(),
                     rest[yk].to_numpy(), knots, ridge)
    return Lookup(kind, knots, v), off


def errors(lk, pos, theta):
    """(deg, counts): the lookup's angle minus the tag, the pot minus the
    table's counts."""
    return (lk.angle(pos) - np.asarray(theta, float),
            np.asarray(pos, float) - lk.counts(theta))


def make_lookup(counts_at, lo_deg, hi_deg, step_deg=0.01):
    """Invert a table angle -> counts into counts -> angle (deg), NaN
    outside lo_deg..hi_deg. Stateless, no delay."""
    th = np.arange(lo_deg, hi_deg + step_deg / 2, step_deg)
    c = np.asarray(counts_at(th), float)
    if c[-1] < c[0]:
        th, c = th[::-1], c[::-1]

    def lookup(counts):
        return np.interp(np.asarray(counts, float), c, th, left=np.nan,
                         right=np.nan)

    return lookup


def boxcar(x, n):
    """Causal mean of the last n samples (warm-up: mean of what exists).
    State: the last n samples. Delay (n - 1) / 2 ticks."""
    x = np.asarray(x, float)
    if n <= 1:
        return x.copy()
    c = np.cumsum(np.concatenate(([0.0], x)))
    i = np.arange(1, len(x) + 1)
    j = np.maximum(i - n, 0)
    return (c[i] - c[j]) / (i - j)


def one_pole(x, delay_ticks):
    """y += a (x - y), a = 1 / (1 + delay_ticks), started on the first
    sample. State: y. DC delay delay_ticks ticks."""
    x = np.asarray(x, float)
    if delay_ticks <= 0:
        return x.copy()
    a = 1.0 / (1.0 + delay_ticks)
    y, _ = lfilter([a], [1.0, a - 1.0], x, zi=[(1.0 - a) * x[0]])
    return y


def median3(x):
    """Causal median of the last 3 samples. State: the last two samples.
    Delay 1 tick on a ramp."""
    x = np.asarray(x, float)
    y = x.copy()
    if len(x) >= 3:
        y[2:] = np.median(np.stack([x[:-2], x[1:-1], x[2:]]), axis=0)
    return y


def carry_hold(x, carries, half=12.0):
    """(held, band): hold the last sample outside +/-half counts of any
    carry code. State: the last sample outside the band. No delay outside
    the band; inside it the output stands still."""
    x = np.asarray(x, float)
    band = np.zeros(len(x), bool)
    for c in carries:
        band |= np.abs(x - c) <= half
    idx = np.where(~band, np.arange(len(x)), 0)
    idx = np.maximum.accumulate(idx)
    y = x[idx]
    return y, band


def dir_from_duty(duty):
    """Sign of the applied duty, the last nonzero sign held through zero.
    State: that sign. No delay."""
    s = np.sign(np.asarray(duty, float))
    idx = np.maximum.accumulate(np.where(s != 0, np.arange(len(s)), 0))
    return s[idx]


def dir_from_pot(x, hyst, d0=0.0):
    """Sign of the pot's own motion: flips when the pot has come back hyst
    counts from its extreme since the last flip. 0 until the first flip
    unless d0 is given. State: the sign and the extremes since the last
    flip. A reversal flips it hyst / speed ticks late."""
    x = np.asarray(x, float)
    d = np.empty(len(x))
    cur, hi, lo = d0, x[0], x[0]
    for i, v in enumerate(x.tolist()):
        if v > hi:
            hi = v
        if v < lo:
            lo = v
        if cur <= 0 and v - lo >= hyst:
            cur, hi = 1.0, v
        elif cur >= 0 and hi - v >= hyst:
            cur, lo = -1.0, v
        d[i] = cur
    return d


def play(x, half):
    """Play operator: the output is dragged by x through a gap of 2 half.
    Steady motion gives x - dir * half; a reversal holds until x has come
    back 2 half. NaN inputs are skipped (the output is NaN there). State:
    the output. No delay in steady motion; a reversal holds 2 half / speed
    ticks."""
    x = np.asarray(x, float)
    y = np.full(len(x), np.nan)
    cur = np.nan
    for i, v in enumerate(x.tolist()):
        if v != v:
            continue
        if cur != cur:
            cur = v
        elif cur < v - half:
            cur = v - half
        elif cur > v + half:
            cur = v + half
        y[i] = cur
    return y


def lash_offset(theta_deg, direction, plus_deg, minus_deg):
    """Remove the pot's lead over the horn: plus_deg while moving +,
    minus_deg (negative) while moving -; nothing while direction is 0.
    Stateless, no delay."""
    d = np.asarray(direction, float)
    return np.asarray(theta_deg, float) - np.where(
        d > 0, plus_deg, np.where(d < 0, minus_deg, 0.0))
