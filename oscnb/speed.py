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
"""
from dataclasses import dataclass

import numba as nb
import numpy as np

from .scales import RAIL_LINE

ALPHA_CU = 0.0039       # per C, copper's 0.39 %/C (typed, physical)


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
