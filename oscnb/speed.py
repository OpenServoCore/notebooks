"""Speed estimators: the four-term back-EMF speed, row by row.

Pure functions, numpy in and numpy out, float only, one row per PWM period
(the 20 kHz control tick); nothing of any servo or board is bound at
import, every constant is an argument (`Chain` for the board's sense
chain, `Motor` for the identification's constants, `BrushTable` /
`BrushProduct` for the brush term's two forms). A row is the crest current
sample (counts), the two terminal taps at the crest (counts), the signed
on-ticks of the timer, the direction, the board NTC (C) and the current
zero (counts), each as an array over the rows. No function looks ahead of
the row it writes; each docstring names the state it keeps between rows.

    i_w = (crest - zero) a_per_count - g (1 - D) V_on   zero, reading term
    V_c = s V_on - (1 - s) (b0 + b1 i_w)                edges, brake line
    u   = V_c - V0 - R(T) i_w - L di_w/dt - k_d rate    R(T), L, dropouts
    Ke w + V_b = u                                      the brush term

R(T) = R0 (1 + alpha (T - T0)). The brush term has one of two forms:
  table    V_b(w, dir), a root by bisection, or a fixed number of steps
           seeded from the previous row's speed;
  product  V_b = c0 + c_dir [dir +1] + b w i_w, closed:
           w = (u - c0 - c_dir [dir +1]) / (Ke + b i_w).

`direction` is the drive's direction on that row, the sign of the duty
(not a travel label: through a reversal the two differ). w is the speed in
the direction of the drive (rev/s); the signed shaft speed is dir x w. At
100 % duty nothing switches: s = 1, no brake drop, no reading term.

`chain_of`, `motor_of`, `table_of`, `brush_table` and `brush_product` bind
the constants through the chain: the board's scales, the identification's
result files.

Behind the estimate sit the filters, the L term's slopes and the valid
flag, on the same rows and as causal. `first` marks the rows with nothing
before them (the first row after torque on or after a gap in the rows);
row 0 is always one, and a filter's state restarts on one.
    filters      boxcar, one_pole (gain from pole_alpha), median3,
                 locked_boxcar (its window per row from lock_rows: a whole
                 number of commutation periods at a causal speed)
    L term       slope_line (least-squares line over the last n rows),
                 slope_ends (two rows n apart), slope_pole (one-pole on the
                 row-to-row difference)
    valid flag   valid_rows = tap_floor_valid (the on-time reaches the
                 taps' floor) and holdoff_rows (past the start-up after
                 torque on and a flip) and not dropout_flag (the current
                 under a share of its recent mean); hold_last and
                 one_pole_skip carry the estimate over the rows it leaves
                 out.
"""
from dataclasses import dataclass

import numba as nb
import numpy as np

from .scales import RAIL_LINE

ALPHA_CU = 0.0039       # per C, copper's 0.39 %/C (typed, physical)
LOCK_W_MIN_REV_S = 1e-9     # rev/s, lock_rows reads a slower w_ref as this
NEVER_ROWS = 1 << 30        # rows, rows_since before the first mark


@dataclass(frozen=True)
class Chain:
    """The board's sense chain, from its scales (never typed)."""
    a_per_count: float          # A per count of the current channel
    tap_counts_per_v: float     # counts of tap difference per volt
    tap_offset_counts: float    # counts, the tap difference under drive
    half_period_ticks: int      # ticks at 100 % (the compare's full scale)
    lag_rail_v: tuple           # V, (low, high) rails the lags were graded on
    lag_on_ticks: tuple         # ticks, ON edge lag at those rails
    lag_off_ticks: tuple        # ticks, OFF edge lag at those rails
    fs_hz: float                # rows per second


@dataclass(frozen=True)
class Motor:
    """The identification's constants."""
    r_ohm: float            # ohm, R at t0_c
    t0_c: float             # C
    alpha_per_c: float      # per C, the conductor's slope
    v0_v: float             # V
    g_s: float              # S, the reading term on (1 - D) V_on
    brake_b0_v: float       # V, the brake line's intercept
    brake_b1_ohm: float     # ohm, the brake line's slope
    l_h: float              # H
    ke_v_per_rev_s: float   # V per rev/s
    k_d_v_per_dropout_s: float = 0.0    # V per dropout/s


@dataclass(frozen=True)
class BrushTable:
    """V_b(w, dir) at knots (rev/s), volts; flat under the first knot and
    past the last finite one."""
    knots: np.ndarray
    pos: np.ndarray         # dir +1
    neg: np.ndarray         # dir -1

    def clean(self):
        """The two directions on their finite knots (a null knot past the
        top rung is dropped: the last finite value is held)."""
        out = []
        for tab in (self.pos, self.neg):
            ok = np.isfinite(tab)
            out.append((np.asarray(self.knots, float)[ok],
                        np.asarray(tab, float)[ok]))
        return out


@dataclass(frozen=True)
class BrushProduct:
    """V_b = c0 + c_dir [dir +1] + b w i_w, volts; b in V per (rev/s x A)."""
    c0_v: float
    c_dir_v: float
    b_v_per_rev_s_a: float


def chain_of(board, scales):
    """The sense chain of `board` (an `oscnb.boards` entry: the timer's
    half period and the tick) through its stage 1 `scales` (an
    `oscnb.scales.Scales`: the current scale, the tap scale and offset, and
    the edge lags at the two graded rails)."""
    (_, lo_v), (_, hi_v) = RAIL_LINE
    on_lo, off_lo = scales.edge_lags_at(lo_v)[:2]
    on_hi, off_hi = scales.edge_lags_at(hi_v)[:2]
    return Chain(a_per_count=scales.a_per_count,
                 tap_counts_per_v=scales.terminal_counts_per_v,
                 tap_offset_counts=scales.terminal_offset_counts,
                 half_period_ticks=int(board.pwm_arr),
                 lag_rail_v=(lo_v, hi_v), lag_on_ticks=(on_lo, on_hi),
                 lag_off_ticks=(off_lo, off_hi), fs_hz=float(board.tick_hz))


def _q(d, key):
    return d[key]["value"]


def motor_of(res, ind, lad, k_d=None):
    """The identification's constants from stage 3's result dicts of one
    pair, each the `value` of:
      res (3-servo resistance): R `r_rails_at_measured_e_ohm` (ohm),
          T0 `board_ntc_c` (C), V0 `v0_rails_at_measured_e_v` (V),
          g `meter_g_ms` (mS);
      ind (3-servo inductance): the brake line `brake_line_motor_plain`,
          `intercept_mv` (mV) and `slope_ohm` (ohm), L `l_mh` (mH);
      lad (3-servo ladder): Ke `ke_mv_per_rev_s` (mV per rev/s), k_d
          `k_d_mv_per_dropout_s` route B (mV per dropout/s, the ladder's
          candidate) unless `k_d` (V per dropout/s) is given.
    The conductor's slope is copper's, ALPHA_CU."""
    brake = ind["brake_line_motor_plain"]
    kd = _q(lad["k_d_mv_per_dropout_s"], "B") * 1e-3 if k_d is None else k_d
    return Motor(r_ohm=_q(res, "r_rails_at_measured_e_ohm"),
                 t0_c=_q(res, "board_ntc_c"), alpha_per_c=ALPHA_CU,
                 v0_v=_q(res, "v0_rails_at_measured_e_v"),
                 g_s=_q(res, "meter_g_ms") * 1e-3,
                 brake_b0_v=_q(brake, "intercept_mv") * 1e-3,
                 brake_b1_ohm=_q(brake, "slope_ohm"),
                 l_h=_q(ind, "l_mh") * 1e-3,
                 ke_v_per_rev_s=_q(lad, "ke_mv_per_rev_s") * 1e-3,
                 k_d_v_per_dropout_s=kd)


def table_of(tab, pos=1, neg=-1):
    """A stage 3 table dict (mV at the knots, keys `pos` / `neg` per
    direction) as a BrushTable in volts."""
    return BrushTable(np.asarray(tab["knots"], float),
                      np.asarray(tab[pos], float) * 1e-3,
                      np.asarray(tab[neg], float) * 1e-3)


def brush_table(comp, kind="signed"):
    """The table form of the ladder's fit from stage 3's compensation
    result: `v_b_table_mv[kind]` ("clipped" or "signed"), mV at
    `knots_rev_s`, null past the top rung."""
    tab = comp["v_b_table_mv"][kind]
    return table_of({"knots": tab["knots_rev_s"], 1: tab["+1"],
                     -1: tab["-1"]})


def brush_product(comp):
    """The product form of the ladder's fit from stage 3's compensation
    result: `v_b_product_form`, `c0_mv`, `c_dir_mv` (mV) and
    `b_mv_per_rev_s_a` (mV per (rev/s x A))."""
    p = comp["v_b_product_form"]
    return BrushProduct(_q(p, "c0_mv") * 1e-3, _q(p, "c_dir_mv") * 1e-3,
                        _q(p, "b_mv_per_rev_s_a") * 1e-3)


def tap_volts(tap_a, tap_b, direction, chain):
    """V_on: the terminal difference at the crest in the direction of the
    drive, volts. Stateless, per row."""
    diff = ((np.asarray(tap_a, float) - np.asarray(tap_b, float))
            * np.asarray(direction, float))
    return (diff - chain.tap_offset_counts) / chain.tap_counts_per_v


def edge_lags(rail_v, chain):
    """(ON, OFF) edge lags in ticks on the line through the graded rails.
    Stateless, per row."""
    lo, hi = chain.lag_rail_v
    f = (np.asarray(rail_v, float) - lo) / (hi - lo)
    on = chain.lag_on_ticks[0] + f * (chain.lag_on_ticks[1]
                                      - chain.lag_on_ticks[0])
    off = chain.lag_off_ticks[0] + f * (chain.lag_off_ticks[1]
                                        - chain.lag_off_ticks[0])
    return on, off


def construct(crest, zero, v_on, duty_ticks, chain, motor, rail_v=None):
    """(i_w A, V_c V, D, s) per row. `rail_v` sets the edge lags; None takes
    the row's own V_on (the crest tap reads the rail less the bridge's
    drop, and the lags move 1.4 ticks per volt). Stateless, per row."""
    h = np.abs(np.asarray(duty_ticks, float))
    v_on = np.asarray(v_on, float)
    arr = chain.half_period_ticks
    full = h >= arr
    d_share = h / arr
    on, off = edge_lags(v_on if rail_v is None else rail_v, chain)
    s = np.where(full, 1.0, (2 * h - on + off) / (2 * arr))
    i = ((np.asarray(crest, float) - np.asarray(zero, float))
         * chain.a_per_count)
    i_w = i - motor.g_s * (1 - d_share) * v_on
    drop = np.where(full, 0.0, motor.brake_b0_v + motor.brake_b1_ohm * i_w)
    v_c = s * v_on - (1 - s) * drop
    return i_w, v_c, d_share, s


def r_of_t(ntc_c, motor):
    """R(T) = R0 (1 + alpha (T - T0)), ohm; a NaN temperature reads as T0.
    Stateless, per row."""
    t = np.where(np.isfinite(ntc_c), ntc_c, motor.t0_c)
    return motor.r_ohm * (1 + motor.alpha_per_c * (t - motor.t0_c))


def backward_didt(i_w, fs_hz, first=None):
    """di_w/dt from the row before (A/s); zero on a run's first row (`first`
    marks them, row 0 always). State: the previous row's i_w. Per row."""
    i_w = np.asarray(i_w, float)
    out = np.zeros_like(i_w)
    out[1:] = (i_w[1:] - i_w[:-1]) * fs_hz
    if first is not None:
        out[np.asarray(first, bool)] = 0.0
    return out


def trailing_didt(i_w, fs_hz, n, first=None):
    """di_w/dt over the last n rows (A/s), over fewer after a run's first
    row (`first` marks them), zero on it. State: the trailing n rows of
    i_w. Per row."""
    i_w = np.asarray(i_w, float)
    k = np.arange(len(i_w))
    start = np.zeros(len(i_w), np.int64)
    if first is not None:
        f = np.flatnonzero(np.asarray(first, bool))
        if len(f):
            start = np.maximum.accumulate(np.where(np.isin(k, f), k, 0))
    j = np.maximum(k - int(n), start)
    return np.where(k > j, (i_w - i_w[j]) * fs_hz / np.maximum(k - j, 1),
                    0.0)


def u_rows(i_w, v_c, ntc_c, motor, didt=None, rate=None):
    """u = V_c - V0 - R(T) i_w - L di_w/dt - k_d rate: the volts the EMF and
    the brush term share. Stateless, per row (the slope and the dropout
    rate carry their own)."""
    u = (np.asarray(v_c, float) - motor.v0_v
         - r_of_t(np.asarray(ntc_c, float), motor) * np.asarray(i_w, float))
    if didt is not None:
        u = u - motor.l_h * np.asarray(didt, float)
    if rate is not None and motor.k_d_v_per_dropout_s:
        u = u - motor.k_d_v_per_dropout_s * np.asarray(rate, float)
    return u


def table_eval(table, direction, w):
    """V_b in volts at w for each row (w under zero reads the value at
    rest). Stateless, per row."""
    w = np.maximum(np.asarray(w, float), 0.0)
    d = np.asarray(direction)
    out = np.zeros_like(w)
    (kp, tp), (kn, tn) = table.clean()
    out[d > 0] = np.interp(w[d > 0], kp, tp)
    out[d < 0] = np.interp(w[d < 0], kn, tn)
    return out


def solve_bisect(u, direction, table, ke, iters=50, margin_v=0.3):
    """The root of Ke w + V_b(w, dir) = u by bisection (the reference); u
    under the table's value at rest reads as the plain estimate continued
    through zero. Stateless, per row."""
    u = np.asarray(u, float)
    hi = (np.maximum(u, 0.0) + margin_v) / ke
    lo = np.zeros_like(hi)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        over = ke * mid + table_eval(table, direction, mid) > u
        hi = np.where(over, mid, hi)
        lo = np.where(over, lo, mid)
    vb0 = table_eval(table, direction, np.zeros_like(u))
    return np.where(u <= vb0, (u - vb0) / ke, 0.5 * (lo + hi))


def solve_product(u, i_w, direction, form, ke):
    """The product form closes in one line:
    w = (u - c0 - c_dir [dir +1]) / (Ke + b i_w). Stateless, per row."""
    c = form.c0_v + form.c_dir_v * (np.asarray(direction) > 0)
    return ((np.asarray(u, float) - c)
            / (ke + form.b_v_per_rev_s_a * np.asarray(i_w, float)))


@nb.njit(cache=True)
def _vb_at(knots, tab, w):
    n = knots.shape[0]
    if w <= knots[0]:
        return tab[0], 0.0
    if w >= knots[n - 1]:
        return tab[n - 1], 0.0
    j = np.searchsorted(knots, w) - 1
    if j < 0:
        j = 0
    slope = (tab[j + 1] - tab[j]) / (knots[j + 1] - knots[j])
    return tab[j] + slope * (w - knots[j]), slope


@nb.njit(cache=True)
def _steps(u, d, first, kp, tp, kn, tn, ke, steps, newton, seed_plain, out):
    w = 0.0
    for n in range(u.shape[0]):
        if first[n]:
            w = u[n] / ke if seed_plain else 0.0
        elif d[n] != d[n - 1]:
            w = -w      # the drive's direction flipped: the shaft's did not
        for _ in range(steps):
            if d[n] > 0:
                vb, slope = _vb_at(kp, tp, w)
            else:
                vb, slope = _vb_at(kn, tn, w)
            if newton:
                w = w - (ke * w + vb - u[n]) / (ke + slope)
            else:
                w = (u[n] - vb) / ke
        out[n] = w


def solve_steps(u, direction, first, table, ke, steps=1, kind="newton",
                cold="zero"):
    """A fixed number of steps per row toward the root of
    Ke w + V_b(w, dir) = u, seeded from the row before.

    kind "newton": the step solves the table's line through the seed's
    segment exactly (w -= f / (Ke + slope)); "fixed": the table read at the
    seed, w = (u - V_b(seed)) / Ke. `first` marks the rows with nothing
    before them (the first row after torque on): seeded `cold` = "zero" or
    "plain" (u / Ke). State: the previous row's speed and direction (the
    seed is negated when the drive's direction flips). Per row."""
    (kp, tp), (kn, tn) = table.clean()
    out = np.empty(len(u))
    _steps(np.ascontiguousarray(u, dtype=np.float64),
           np.ascontiguousarray(direction, dtype=np.int64),
           np.ascontiguousarray(first, dtype=np.bool_), kp, tp, kn, tn,
           float(ke), int(steps), kind == "newton", cold == "plain", out)
    return out


def table_slopes(table):
    """Per direction: (knot rev/s, slope V per rev/s of the segment above
    it). Of the table, not of the rows."""
    return [(k[:-1], np.diff(t) / np.diff(k)) for k, t in table.clean()]


def estimate_rows(crest, zero, tap_a, tap_b, duty_ticks, direction, ntc_c,
                  chain, motor, brush, rail_v=None, first=None, with_l=False,
                  rate=None, steps=None, kind="newton"):
    """(w rev/s in the drive's direction, u V, i_w A) per row.

    brush: a BrushTable (bisection, or `steps` fixed steps seeded from the
    row before) or a BrushProduct. first: rows with no row before them
    (after torque on or a gap); row 0 is always one. with_l: the L di_w/dt
    term from the row before. State: the previous row's i_w with `with_l`,
    the previous row's speed with `steps`; none otherwise. Per row."""
    n = len(crest)
    first = (np.zeros(n, bool) if first is None
             else np.asarray(first, bool).copy())
    first[0] = True
    v_on = tap_volts(tap_a, tap_b, direction, chain)
    i_w, v_c, _, _ = construct(crest, zero, v_on, duty_ticks, chain, motor,
                               rail_v)
    didt = backward_didt(i_w, chain.fs_hz, first) if with_l else None
    u = u_rows(i_w, v_c, ntc_c, motor, didt, rate)
    ke = motor.ke_v_per_rev_s
    if isinstance(brush, BrushProduct):
        w = solve_product(u, i_w, direction, brush, ke)
    elif steps is None:
        w = solve_bisect(u, direction, brush, ke)
    else:
        w = solve_steps(u, direction, first, brush, ke, steps, kind)
    return w, u, i_w


def _first(first, n):
    f = (np.zeros(n, np.bool_) if first is None
         else np.ascontiguousarray(first, dtype=np.bool_).copy())
    if n:
        f[0] = True
    return f


def _f8(x):
    return np.ascontiguousarray(x, dtype=np.float64)


@nb.njit(cache=True)
def _boxcar_var(x, n, first, out):
    start = 0
    c = np.empty(len(x) + 1)
    c[0] = 0.0
    for k in range(len(x)):
        c[k + 1] = c[k] + x[k]
        if first[k]:
            start = k
        m = min(n[k], k - start + 1)
        if m < 1:
            m = 1
        out[k] = (c[k + 1] - c[k + 1 - m]) / m


def boxcar(x, n, first=None):
    """The mean of the last n rows (of fewer after a first row). State: the
    last n rows. DC delay (n - 1) / 2 rows. Per row.

    Not oscnb.position.boxcar: this one's window restarts on a first row
    (position's has no first rows). As in position's, a NaN row spoils
    every later output, past a first row too (the sum runs over all rows).
    """
    x = _f8(x)
    out = np.empty(len(x))
    _boxcar_var(x, np.full(len(x), int(n), np.int64), _first(first, len(x)),
                out)
    return out


def locked_boxcar(x, n_rows, first=None):
    """The mean of the last n_rows[k] rows, a window length per row (see
    lock_rows), of fewer after a first row. State: the last max(n_rows)
    rows. Per row."""
    x = _f8(x)
    out = np.empty(len(x))
    _boxcar_var(x, np.ascontiguousarray(n_rows, dtype=np.int64),
                _first(first, len(x)), out)
    return out


def lock_rows(w_ref, fs_hz, events_per_rev, periods, n_min, n_max):
    """The window in rows that spans `periods` commutation periods at the
    speed w_ref (rev/s), clipped to [n_min, n_max]; no speed reads n_max.
    w_ref must be causal (a filtered estimate of the rows before).
    Stateless, per row."""
    w = np.maximum(np.abs(np.asarray(w_ref, float)), LOCK_W_MIN_REV_S)
    n = np.rint(periods * fs_hz / (events_per_rev * w))
    return np.clip(np.nan_to_num(n, nan=n_max, posinf=n_max), n_min,
                   n_max).astype(np.int64)


@nb.njit(cache=True)
def _one_pole(x, alpha, first, out):
    y = 0.0
    for k in range(len(x)):
        if first[k]:
            y = x[k]
        else:
            y += alpha * (x[k] - y)
        out[k] = y


def pole_alpha(tau_rows):
    """The one-pole's gain for a time constant in rows."""
    return 1.0 - np.exp(-1.0 / float(tau_rows))


def one_pole(x, alpha, first=None):
    """y += alpha (x - y), seeded with x on a first row. State: y. DC delay
    (1 - alpha) / alpha rows. Per row.

    Not oscnb.position.one_pole: this one takes the gain (position's takes
    the delay, a = 1 / (1 + delay)) and reseeds on a first row, so a NaN
    row spoils the output only until the next first row (position's
    stays NaN to the end)."""
    x = _f8(x)
    out = np.empty(len(x))
    _one_pole(x, float(alpha), _first(first, len(x)), out)
    return out


@nb.njit(cache=True)
def _median3(x, first, out):
    start = 0
    for k in range(len(x)):
        if first[k]:
            start = k
        if k - start < 2:
            out[k] = x[k]
        else:
            a, b_, c = x[k - 2], x[k - 1], x[k]
            out[k] = max(min(a, b_), min(max(a, b_), c))


def median3(x, first=None):
    """The median of the row and the two before it (the row itself on a
    run's first two rows). State: the two rows before. Delay 1 row on a
    ramp. Per row.

    Not oscnb.position.median3: this one restarts on a first row, and a NaN
    among the three rows may read as a finite one (the min / max network;
    position's np.median reads NaN)."""
    x = _f8(x)
    out = np.empty(len(x))
    _median3(x, _first(first, len(x)), out)
    return out


@nb.njit(cache=True)
def _slope_line(x, n, fs, first, out):
    start = 0
    for k in range(len(x)):
        if first[k]:
            start = k
        m = min(n, k - start + 1)
        if m < 2:
            out[k] = 0.0
            continue
        mid = (m - 1) / 2.0
        acc = 0.0
        for j in range(m):
            acc += (j - mid) * x[k - m + 1 + j]
        out[k] = acc / (m * (m * m - 1.0) / 12.0) * fs


def slope_line(i_w, n, fs_hz, first=None):
    """di/dt (A/s): the least-squares line over the last n rows (fewer
    after a first row, zero on it). It reads the slope (n - 1) / 2 rows
    ago. State: the last n rows. Per row."""
    x = _f8(i_w)
    out = np.empty(len(x))
    _slope_line(x, int(n), float(fs_hz), _first(first, len(x)), out)
    return out


def slope_ends(i_w, n, fs_hz, first=None):
    """di/dt (A/s): the row less the row n before, over n rows (fewer after
    a first row, zero on it); n = 1 is backward_didt. State: the last n
    rows. Per row."""
    return trailing_didt(i_w, float(fs_hz), int(n), _first(first, len(i_w)))


@nb.njit(cache=True)
def _slope_pole(x, alpha, fs, first, out):
    y = 0.0
    for k in range(len(x)):
        if first[k]:
            y = 0.0
        else:
            y += alpha * ((x[k] - x[k - 1]) * fs - y)
        out[k] = y


def slope_pole(i_w, alpha, fs_hz, first=None):
    """di/dt (A/s): a one-pole (gain alpha) on the row-to-row difference,
    zero on a first row. State: the previous row and y. Per row."""
    x = _f8(i_w)
    out = np.empty(len(x))
    _slope_pole(x, float(alpha), float(fs_hz), _first(first, len(x)), out)
    return out


def tap_floor_valid(duty_ticks, floor_ticks):
    """The taps' crest sample is settled: the on-time is at least the floor
    (timer ticks). Stateless, per row."""
    return np.abs(np.asarray(duty_ticks, float)) >= floor_ticks


@nb.njit(cache=True)
def _dropout(x, frac, alpha, floor_a, first, hold, max_rows, out):
    ref = 0.0
    since = 0
    run = 0
    for k in range(len(x)):
        if first[k]:
            ref = x[k]
            since = 0
            run = 0
            out[k] = False
            continue
        since += 1
        hit = since > hold and ref > floor_a and x[k] < frac * ref
        if hit:
            run += 1
            if max_rows > 0 and run > max_rows:
                hit = False
                ref = x[k]
                run = 0
        else:
            run = 0
            ref += alpha * (x[k] - ref)
        out[k] = hit


def dropout_flag(i_over_zero, frac, alpha, floor_a, first=None, hold_rows=0,
                 max_rows=0):
    """A dropout row: the row's current under `frac` of its recent mean.

    The recent mean is a one-pole (gain alpha) over the rows not flagged,
    so a dropout does not pull its own reference down. No row is flagged
    while the reference is under floor_a (a few sd of the current's own
    noise: a ratio means nothing at no current) or in the first hold_rows
    rows of a run (the current is still building). A dropout is short:
    after max_rows flagged rows in a row (0 = no limit) the current's level
    is taken to have moved (the duty fell, the drive flipped), the
    reference restarts at the row and the flag clears. State: the
    reference, the rows since the run's first and the flagged rows running.
    Per row."""
    x = _f8(i_over_zero)
    out = np.zeros(len(x), np.bool_)
    _dropout(x, float(frac), float(alpha), float(floor_a),
             _first(first, len(x)), int(hold_rows), int(max_rows), out)
    return out


@nb.njit(cache=True)
def _since(mark, out):
    c = NEVER_ROWS
    for k in range(len(mark)):
        c = 0 if mark[k] else min(c + 1, NEVER_ROWS)
        out[k] = c


def rows_since(mark):
    """Rows since the last marked row (0 on it, NEVER_ROWS before any).
    State: the count. Per row."""
    out = np.empty(len(mark), np.int64)
    _since(np.ascontiguousarray(mark, dtype=np.bool_), out)
    return out


def holdoff_rows(first, flip, n_first, n_flip):
    """True where the estimate is past its start-up: n_first rows after
    torque on (a first row), n_flip after the drive's direction flips (None
    or n_flip 0: no flip hold-off). State: the rows since each. Per row."""
    f = _first(first, len(first))
    ok = rows_since(f) >= n_first
    if flip is not None and n_flip:
        ok &= rows_since(np.asarray(flip, bool)) >= n_flip
    return ok


@nb.njit(cache=True)
def _hold_last(x, valid, first, out):
    last = np.nan
    for k in range(len(x)):
        if first[k]:
            last = np.nan
        if valid[k]:
            last = x[k]
        out[k] = last


def hold_last(x, valid, first=None):
    """x on a valid row, else the last valid row's value (NaN before the
    first one of a run). State: the last valid value. Per row."""
    x = _f8(x)
    out = np.empty(len(x))
    _hold_last(x, np.ascontiguousarray(valid, dtype=np.bool_),
               _first(first, len(x)), out)
    return out


@nb.njit(cache=True)
def _one_pole_skip(x, alpha, valid, first, out):
    y = np.nan
    for k in range(len(x)):
        if first[k]:
            y = np.nan
        if valid[k]:
            if np.isnan(y):
                y = x[k]
            else:
                y += alpha * (x[k] - y)
        out[k] = y


def one_pole_skip(x, alpha, valid, first=None):
    """A one-pole (gain alpha) that an invalid row does not move, seeded
    with a run's first valid row (NaN before it). State: y. Per row."""
    x = _f8(x)
    out = np.empty(len(x))
    _one_pole_skip(x, float(alpha), np.ascontiguousarray(valid,
                                                         dtype=np.bool_),
                   _first(first, len(x)), out)
    return out


def valid_rows(duty_ticks, i_over_zero, first, flip, floor_ticks, n_first,
               n_flip, frac, alpha, floor_a, hold_rows=0, max_rows=0):
    """(valid, tap ok, past start-up, dropout) per row: the estimate's
    valid flag from the servo's own signals, tap_floor_valid and
    holdoff_rows and not dropout_flag. State: theirs. Per row."""
    tap_ok = tap_floor_valid(duty_ticks, floor_ticks)
    started = holdoff_rows(first, flip, n_first, n_flip)
    drop = dropout_flag(i_over_zero, frac, alpha, floor_a, first, hold_rows,
                        max_rows)
    return tap_ok & started & ~drop, tap_ok, started, drop
