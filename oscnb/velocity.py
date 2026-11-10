"""Velocity estimators: the pot's slope, the back-EMF speed, the rest
verdict and the rules that make one velocity of the two, row by row.

Pure functions, numpy in and numpy out, float only, one row per control
tick; nothing of any servo or board is bound at import, every constant is
an argument. No function looks ahead of the row it writes (but for
ls_line_skip's centering constant, see there); each docstring names the
state it keeps between rows and its nominal delay in rows. A slope is in
x per second (fs_hz is the row rate), x the pot in counts or the lookup's
angle in deg; a velocity is signed, positive where the pot's counts rise,
in whatever unit the two sources share. A slope writes NaN until its
state is full, so nothing is scored on a warm-up.

    pot's slope  two_point     (x[t] - x[t - n]) / n
                 two_box       two boxcars of m rows, n rows apart
                 ls_line       the least-squares line over the last n rows
                 pole_diff     a one-pole on the difference over `step` rows
                 boxcar        oscnb.position's mean of the last m rows
    carries      carry_band    rows whose reading sits in a carry's band
                 carry_center  the wide code read at the middle of its span
                 hold_band     the last reading outside the band
                 ls_line_skip, two_point_skip, pole_diff_skip: the slopes
                 with the band's rows left out
    by name      KINDS, nominal_delay, span_rows, run
    rest         rest_band     the reading stays within a band of an anchor
                 rest_slope    the slope stays under a threshold
                 emf_turning   the veto: the back-EMF reads off the stopped
                               rotor's line
    back-EMF     emf_signed    the estimate in the pot's sign, dropouts held
                 emf_boxcar    the mean of the last n usable rows, and
                               whether it may carry
                 emf_through_slope  the back-EMF through two_box's kernel
    blends       switch        the back-EMF where it carries, else the pot
                 with_rest     zero while the rest verdict is set
                 complementary the back-EMF plus a one-pole of what the two
                               sources disagree on, fixed weights
                 seams         the output's jump where the source changes

Nominal delay (rows) on a ramp of speed: two_point n / 2; two_box
(m - 1) / 2 + n / 2; ls_line (n - 1) / 2; pole_diff tau + step / 2.

None of these fits or trims one source against the other: the weights are
fixed, and the complementary pair's only memory of the pot is its
one-pole, cleared whenever the back-EMF stops carrying.
"""
import numba as nb
import numpy as np
from scipy.signal import fftconvolve, lfilter

from .position import boxcar

PATTERN_ROWS = 16       # rows, box16's boxcar and pole16's difference step
WARM_TAUS = 5           # time constants, a one-pole's warm-up (start < 1 %)
FFT_ROWS = 64           # rows, a longer kernel convolves by FFT
SKIP_KEEP_DIV = 4       # ls_line_skip keeps n / this rows at least, default
DIV_FLOOR = 1e-9        # rows, the kept rows' count floor as a divisor

KINDS = ("two_point", "box16", "box_n", "ls_line", "pole1", "pole16")

__all__ = [
    "KINDS", "boxcar", "carry_band", "carry_center",
    "complementary", "emf_boxcar", "emf_signed", "emf_through_slope",
    "emf_turning", "hold_band", "ls_line", "ls_line_skip", "nominal_delay",
    "pole_diff", "pole_diff_skip", "rest_band", "rest_slope", "run",
    "seams", "span_rows", "switch", "two_box", "two_point",
    "two_point_skip", "with_rest",
]


def _f8(x):
    return np.ascontiguousarray(x, dtype=np.float64)


def _lag(x, n):
    out = np.full(len(x), np.nan)
    if len(x) > n:
        out[n:] = x[:-n]
    return out


def _conv(a, k):
    if len(k) > FFT_ROWS:
        return fftconvolve(a, k, mode="valid")
    return np.convolve(a, k, mode="valid")


def two_point(x, n, fs_hz):
    """(x[t] - x[t - n]) fs / n, NaN on the first n rows. State: the last n
    rows. Delay n / 2 rows. A pattern whose period divides n cancels
    exactly.

    Not oscnb.speed.slope_ends: the same difference from row n on, but NaN
    before it (slope_ends reads the rows that exist)."""
    x = _f8(x)
    return (x - _lag(x, n)) * fs_hz / n


def two_box(x, m, n, fs_hz):
    """The mean of the last m rows less the mean of the m rows n before,
    over n rows; NaN until n + m rows exist. State: the last n + m rows (or
    two running sums and the rows between). Delay (m - 1) / 2 + n / 2 rows.
    m = n is two boxcars end to end (a triangle on the row differences); m
    a multiple of a pattern's period nulls it at any n."""
    x = _f8(x)
    out = np.full(len(x), np.nan)
    if len(x) < n + m:
        return out
    c = np.cumsum(np.r_[0.0, x])
    b = (c[m:] - c[:-m]) / m
    out[n + m - 1:] = (b[n:] - b[:-n]) * fs_hz / n
    return out


def ls_line(x, n, fs_hz):
    """The slope of the least-squares line through the last n rows, NaN on
    the first n - 1. State: the last n rows (or the running sums of x and
    of j x). Delay (n - 1) / 2 rows.

    Not oscnb.speed.slope_line: the same line on a full window, but NaN
    before it (slope_line fits the rows that exist)."""
    x = _f8(x)
    out = np.full(len(x), np.nan)
    if len(x) < n or n < 2:
        return out
    j = np.arange(n) - (n - 1) / 2.0
    k = -j / np.sum(j * j)
    out[n - 1:] = _conv(x - x[0], k)
    return out * fs_hz


def pole_diff(x, tau_rows, fs_hz, step=1):
    """y += a ((x[t] - x[t - step]) fs / step - y), a = 1 / (1 + tau_rows),
    started at 0 on the first difference; NaN on the first
    ceil(WARM_TAUS tau_rows) + step rows (the start's share under 1 %).
    State: y and the last `step` rows. Delay tau_rows + step / 2 rows.

    Not oscnb.speed.slope_pole: that one's gain is pole_alpha's and it
    reads every row; not oscnb.position.one_pole: that one is seeded with
    its first input, this one at zero."""
    x = _f8(x)
    a = 1.0 / (1.0 + tau_rows)
    d = (x - _lag(x, step)) * fs_hz / step
    out = np.full(len(x), np.nan)
    warm = int(np.ceil(WARM_TAUS * tau_rows)) + step
    if len(x) > warm:
        y = lfilter([a], [1.0, -(1.0 - a)], d[step:])
        out[warm:] = y[warm - step:]
    return out


def carry_band(x, codes, missing, half):
    """True on a row whose reading sits from `half` counts under a wide code
    to `half` counts past its missing run. codes: the wide code before each
    carry; missing: the run of codes that never appears after it. half is
    in counts: a few sd of the pot's own row noise, or the longest missing
    run. Stateless, no delay.

    Not oscnb.position.carry_hold's band: that one is +/- half about the
    code alone, this one covers the missing run as well."""
    x = np.asarray(x, float)
    band = np.zeros(len(x), bool)
    for c, m in zip(codes, missing):
        band |= (x >= c - half) & (x <= c + m + half)
    return band


def carry_center(x, codes, missing):
    """The wide code read at the middle of the span it stands for,
    code + missing / 2 (what a lookup that carries the missing run would
    read); every other code as it is. Stateless, no delay."""
    out = np.asarray(x, float).copy()
    for c, m in zip(codes, missing):
        out[out == c] = c + m / 2.0
    return out


def hold_band(x, band):
    """The last reading outside the band (the first row's while none
    exists). State: that reading. No delay outside the band; inside it the
    output stands still, then jumps the band's width. The hold of
    oscnb.position.carry_hold, on a band given."""
    x = _f8(x)
    idx = np.where(~np.asarray(band, bool), np.arange(len(x)), -1)
    idx = np.maximum.accumulate(idx)
    return np.where(idx >= 0, x[np.maximum(idx, 0)], x[0])


def ls_line_skip(x, ok, n, fs_hz, min_rows=None, min_span=0.5):
    """(slope, held): the least-squares line through the rows of the last n
    marked ok. It needs at least min_rows of them (default
    n / SKIP_KEEP_DIV, 2 at least) whose spread in time (the sd of their
    row index) is at least min_span of a full window's; otherwise the last
    slope is held (NaN before any). State: the last n rows and their marks,
    and the last slope. Delay (n - 1) / 2 rows on a full window, the kept
    rows' mean age while rows are left out, plus the rows it held.

    x is centered on the series' median before the sums: a constant, so the
    slope does not depend on it beyond rounding; a NaN row spoils the
    window even when it is not ok."""
    x = _f8(x)
    w = np.asarray(ok, float)
    nn = len(x)
    out = np.full(nn, np.nan)
    held = np.ones(nn, bool)
    if nn < n or n < 2:
        return out, held
    if min_rows is None:
        min_rows = max(2, n // SKIP_KEEP_DIV)
    j = np.arange(n, dtype=float)[::-1]
    one = np.ones(n)
    xw = (x - np.median(x)) * w
    s_w, s_j, s_jj = _conv(w, one), _conv(w, j[::-1]), _conv(w, (j * j)[::-1])
    s_x, s_jx = _conv(xw, one), _conv(xw, j[::-1])
    s_w = np.maximum(s_w, 0.0)
    var_j = (s_jj / np.maximum(s_w, DIV_FLOOR)
             - (s_j / np.maximum(s_w, DIV_FLOOR)) ** 2)
    good = ((s_w >= min_rows - 0.5)
            & (var_j >= (min_span ** 2) * (n * n - 1) / 12.0))
    den = s_w * s_jj - s_j ** 2
    slope = np.where(good, -(s_w * s_jx - s_j * s_x)
                     / np.where(good, den, 1.0), np.nan) * fs_hz
    idx = np.maximum.accumulate(np.where(good, np.arange(len(slope)), -1))
    out[n - 1:] = np.where(idx >= 0, slope[np.maximum(idx, 0)], np.nan)
    held[n - 1:] = ~good
    return out, held


def two_point_skip(x, ok, n, fs_hz):
    """(slope, held): the newest ok row less the newest ok row at least n
    rows before it, over the rows between them. On a row in the band the
    new end stands on the last ok row, and the old end steps back out of
    the band. held marks the rows where either end was moved. State: a
    delay line of the rows and their marks. Delay half the rows between the
    two ends plus the new end's age: n / 2 rows on a full window."""
    x = _f8(x)
    ok = np.asarray(ok, bool)
    nn = len(x)
    t = np.arange(nn)
    last = np.maximum.accumulate(np.where(ok, t, -1))
    out = np.full(nn, np.nan)
    held = np.ones(nn, bool)
    a = last
    b_at = a - n
    valid = (a >= 0) & (b_at >= 0)
    b = np.where(valid, last[np.maximum(b_at, 0)], -1)
    valid &= b >= 0
    av, bv = a[valid], b[valid]
    out[valid] = (x[av] - x[bv]) * fs_hz / (av - bv)
    held[valid] = (av != t[valid]) | (bv != b_at[valid])
    return out, held


def pole_diff_skip(x, ok, tau_rows, fs_hz, step=PATTERN_ROWS):
    """(slope, held): pole_diff, not moved on a row where either end of the
    difference is not ok; NaN until it has been fed
    ceil(WARM_TAUS tau_rows) rows. held marks the rows it stood still.
    State: y and the last `step` rows with their marks. Delay
    tau_rows + step / 2 rows plus the rows it stood still."""
    x = _f8(x)
    ok = np.asarray(ok, bool)
    a = 1.0 / (1.0 + tau_rows)
    d = (x - _lag(x, step)) * fs_hz / step
    good = ok & np.r_[np.zeros(step, bool), ok[:-step]]
    out = np.full(len(x), np.nan)
    y = 0.0
    fed = 0
    warm = int(np.ceil(WARM_TAUS * tau_rows))
    for t in range(step, len(x)):
        if good[t]:
            y += a * (d[t] - y)
            fed += 1
        if fed >= warm:
            out[t] = y
    return out, ~good


def nominal_delay(kind, n, m=PATTERN_ROWS):
    """Rows of delay of the candidate `kind` on a ramp of speed (n: its
    length in rows; the one-poles: the time constant)."""
    return {"two_point": n / 2.0, "box16": (m - 1) / 2.0 + n / 2.0,
            "box_n": (n - 1) / 2.0 + n / 2.0, "ls_line": (n - 1) / 2.0,
            "pole1": n + 0.5, "pole16": n + PATTERN_ROWS / 2.0}[kind]


def span_rows(kind, n, m=PATTERN_ROWS):
    """Rows of history the candidate's output stands on (the one-poles:
    WARM_TAUS time constants and the step). The first output is on row
    span - 1; a one-pole's is one row later, on row span."""
    return int({"two_point": n + 1, "box16": n + m, "box_n": 2 * n,
                "ls_line": n, "pole1": WARM_TAUS * n + 1,
                "pole16": WARM_TAUS * n + PATTERN_ROWS}[kind])


def run(kind, x, n, fs_hz, m=PATTERN_ROWS):
    """The candidate `kind` at its length n."""
    if kind == "two_point":
        return two_point(x, n, fs_hz)
    if kind == "box16":
        return two_box(x, m, n, fs_hz)
    if kind == "box_n":
        return two_box(x, n, n, fs_hz)
    if kind == "ls_line":
        return ls_line(x, n, fs_hz)
    if kind == "pole1":
        return pole_diff(x, n, fs_hz, 1)
    if kind == "pole16":
        return pole_diff(x, n, fs_hz, PATTERN_ROWS)
    raise ValueError(kind)


@nb.njit(cache=True)
def _rest_band(y, band, t_far, t_carry, in_carry, veto, rest0):
    n = len(y)
    rest = np.zeros(n, np.bool_)
    a = y[0]
    c = max(t_far, t_carry) if rest0 else 0
    for t in range(n):
        if veto[t] or abs(y[t] - a) > band:
            a = y[t]
            c = 0
        else:
            c += 1
        rest[t] = c >= (t_carry if in_carry[t] else t_far)
    return rest


def rest_band(y, band, t_rows, in_carry=None, t_carry_rows=None, veto=None,
              rest0=False):
    """Rest by position: True while the reading y (the pot through a
    boxcar, counts) has stayed within +/- band of an anchor for t_rows
    rows. The anchor is the reading on the row the count last restarted; a
    row more than `band` from it, or a row `veto` marks (another source
    says the rotor turns), moves the anchor to that row and restarts the
    count. On a row `in_carry` marks (the reading sits on a carry's wide
    code, where it stands still while the horn moves) the count must reach
    t_carry_rows instead. rest0: the verdict is already set on the first
    row (a start from a known rest). State: the anchor and the count.

    band is measured in the servo's own rest noise (the sd of y at rest);
    t_rows is set by the longest carry dwell, the longest missing run over
    the scale and the slowest speed to be told from rest. Declared on the
    t_rows-th row in the band, dropped on the row y leaves it."""
    y = _f8(y)
    n = len(y)
    ic = (np.zeros(n, np.bool_) if in_carry is None
          else np.ascontiguousarray(in_carry, np.bool_))
    vt = (np.zeros(n, np.bool_) if veto is None
          else np.ascontiguousarray(veto, np.bool_))
    tc = int(t_rows if t_carry_rows is None else t_carry_rows)
    return _rest_band(y, float(band), int(t_rows), tc, ic, vt, bool(rest0))


@nb.njit(cache=True)
def _rest_slope(s, thr, t_hold, veto, rest0):
    n = len(s)
    rest = np.zeros(n, np.bool_)
    c = t_hold if rest0 else 0
    for t in range(n):
        if veto[t] or not (abs(s[t]) < thr):
            c = 0
        else:
            c += 1
        rest[t] = c >= t_hold
    return rest


def rest_slope(s, thr, t_rows, veto=None, rest0=False):
    """Rest by slope: True while |s| (the pot's slope) has stayed under thr
    for t_rows rows; a row at or over the threshold, a NaN, or a row `veto`
    marks restarts the count. rest0 as rest_band. State: the count (and the
    slope's own rows).

    thr is measured in the slope's own rest sd; t_rows as rest_band's.
    Declared on the t_rows-th row under thr, dropped on the row |s| reaches
    it; the slope's own delay comes before both."""
    s = _f8(s)
    vt = (np.zeros(len(s), np.bool_) if veto is None
          else np.ascontiguousarray(veto, np.bool_))
    return _rest_slope(s, float(thr), int(t_rows), vt, bool(rest0))


def emf_turning(w_f, carry, i_w, a, b, margin):
    """The veto: True where the filtered back-EMF speed in the drive's
    direction (w_f, `carry` marking the rows it stands on usable rows only)
    is further than `margin` from the stopped rotor's reading at the row's
    current, max(a + b i_w, 0). a and b are the stopped rotor's line (the
    identification's hold ladder at rest), margin a multiple of the
    back-EMF's own scatter about it. Stateless, no delay (the filter
    carries its own)."""
    line = np.maximum(a + b * np.nan_to_num(np.asarray(i_w, float)), 0.0)
    return np.asarray(carry, bool) & (
        np.abs(np.nan_to_num(np.asarray(w_f, float)) - line) > margin)


def emf_signed(w, drive, tap_ok, started, drop):
    """(signed speed, usable): the estimate in the drive's direction times
    the drive's sign; usable where the on-time reaches the tap floor and the
    start-up is over; a dropout row inside a usable run takes the last
    usable row's value. State: the last usable value. No delay.

    Not oscnb.speed.hold_last: this one holds only across dropout rows and
    reads NaN off the usable rows."""
    w = _f8(w) * np.asarray(drive, float)
    usable = np.asarray(tap_ok, bool) & np.asarray(started, bool) & (
        np.isfinite(w))
    good = usable & ~np.asarray(drop, bool)
    idx = np.maximum.accumulate(np.where(good, np.arange(len(w)), -1))
    held = np.where(idx >= 0, w[np.maximum(idx, 0)], np.nan)
    out = np.where(good, w, np.where(usable, held, np.nan))
    return out, usable & np.isfinite(out)


@nb.njit(cache=True)
def _run_len(usable):
    n = len(usable)
    run = np.zeros(n, np.int64)
    c = 0
    for t in range(n):
        c = c + 1 if usable[t] else 0
        run[t] = c
    return run


def emf_boxcar(w, usable, n):
    """(mean of the last n rows, carry): the boxcar restarts when the
    estimate stops being usable; carry marks the rows where n usable rows
    in a row exist, NaN elsewhere. State: the last n rows and the run's
    length. Delay (n - 1) / 2 rows."""
    usable = np.ascontiguousarray(usable, np.bool_)
    run = _run_len(usable)
    x = np.where(usable, np.nan_to_num(_f8(w)), 0.0)
    c = np.cumsum(np.r_[0.0, x])
    i = np.arange(1, len(x) + 1)
    out = (c[i] - c[np.maximum(i - n, 0)]) / n
    carry = run >= n
    return np.where(carry, out, np.nan), carry


def emf_through_slope(w, usable, m, n, fs_hz):
    """(the back-EMF through two_box's kernel, good): two_box of the running
    sum of w over the usable rows (an angle in units of w x s), and good
    where all of its n + m rows and the row before were usable. State: the
    running sum's last n + m rows and the run's length. Delay that of
    two_box, (m - 1) / 2 + n / 2 rows."""
    usable = np.ascontiguousarray(usable, np.bool_)
    x = np.where(usable, np.nan_to_num(_f8(w)), 0.0)
    ang = np.cumsum(x) / fs_hz
    return two_box(ang, m, n, fs_hz), _run_len(usable) >= n + m + 1


def switch(v_pot, v_emf, carry):
    """(velocity, source): the back-EMF where it carries, the pot's slope
    elsewhere (source 1 / 0). Stateless; each row has its source's delay."""
    carry = np.asarray(carry, bool)
    return np.where(carry, v_emf, v_pot), carry.astype(np.int8)


def with_rest(v, rest):
    """Zero while the rest verdict is set. Stateless, no delay."""
    return np.where(np.asarray(rest, bool), 0.0, v)


@nb.njit(cache=True)
def _comp(v_pot, v_emf, s_emf, carry, good, alpha):
    n = len(v_pot)
    out = np.empty(n)
    src = np.zeros(n, np.int8)
    d = 0.0
    for t in range(n):
        if not carry[t]:
            d = 0.0
            out[t] = v_pot[t]
            continue
        if good[t] and not np.isnan(v_pot[t]):
            d += alpha * (v_pot[t] - s_emf[t] - d)
        else:
            d = 0.0
        out[t] = v_emf[t] + d
        src[t] = 2 if good[t] else 1
    return out, src


def complementary(v_pot, v_emf, s_emf, carry, good, tau_rows):
    """(velocity, source): the fixed-weight complementary pair. The pot's
    slope through the low-pass 1 / (1 + tau s) below the corner, the
    back-EMF through its complement above it; no constant of the back-EMF
    path is touched.

    On a row where the back-EMF carries: v_emf + d, d a one-pole (time
    constant tau_rows, gain 1 / (1 + tau_rows), started at zero) of the
    pot's slope less s_emf, the back-EMF through the pot's slope's own
    kernel (emf_through_slope), so the two agree on any motion and d holds
    only what the sources disagree on below the corner. Where the kernel
    does not yet stand on usable rows (`good` False) d is zero, which is
    the switch. Where the back-EMF does not carry: the pot's slope, and d
    is cleared, so the pair restarts at the next carrying row. State: d
    (and the sources' own rows). Delay: the back-EMF's above the corner,
    the pot's slope's below it. source: 0 pot, 1 back-EMF alone, 2 the
    pair."""
    alpha = 1.0 / (1.0 + tau_rows)
    return _comp(_f8(v_pot), _f8(v_emf), _f8(s_emf),
                 np.ascontiguousarray(carry, np.bool_),
                 np.ascontiguousarray(good, np.bool_), alpha)


def seams(v, source):
    """(row, jump, from, to) of every row where the carrying source
    changes: the output on that row less the output on the row before."""
    source = np.asarray(source)
    r = np.flatnonzero(source[1:] != source[:-1]) + 1
    return r, v[r] - v[r - 1], source[r - 1], source[r]
