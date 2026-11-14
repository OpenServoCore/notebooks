"""The board NTC two ways: the part's beta equation, and a fixed-point
quadratic in the count, the form a small chip can carry, so a notebook can
fit the quadratic and price what the form costs."""
import numpy as np
from scipy.optimize import linprog


def quadratic_cc(raw, raw_ref, t_ref_cc, k1_q88, k2_q24):
    """Centi-degrees from the fixed-point quadratic: t_ref + k1 x d + k2 x d^2
    with d = raw_ref - raw, k1 in Q8.8 and k2 in Q24, truncating like the
    fixed-point multiply."""
    raw = np.asarray(raw, dtype=np.int64)
    d = raw_ref - raw
    return t_ref_cc + ((d * k1_q88) >> 8) + ((d * d * k2_q24) >> 24)


def fit_quadratic(raw, t_c, raw_ref):
    """(t_ref_cc, k1_q88, k2_q24) that put the fixed-point quadratic through
    measured (raw, degrees C) points, with the reference count given."""
    d = raw_ref - np.asarray(raw, float)
    coef = np.polyfit(d, np.asarray(t_c, float) * 100, 2)
    k2, k1, t_ref = coef
    return int(round(t_ref)), int(round(k1 * 256)), int(round(k2 * (1 << 24)))


K2_TOP_Q24 = 0xFFFF     # the curvature's field is an unsigned 16-bit word (core regions/calib.rs ntc_k2_q24)
FIT_SCALE = 1000.0      # counts, the pinned fit's own unit of the deviation: conditioning only


def fit_pinned(board, raw_ref, pin_c, range_c, floor_c, low_range_c, step_c=0.1, k2_top_q24=K2_TOP_Q24):
    """(t_ref_cc, k1_q88, k2_q24) of the fixed-point quadratic about
    `raw_ref` pinned at a true `pin_c`, the part's nominal beta
    equation through the board's divider as the truth: its residual
    against that equation is 0 at `pin_c`, no less than `floor_c` (a
    negative number of degrees) over `range_c` (lo, hi), and inside
    that its largest over-read over `low_range_c` (lo, hi) is as small
    as it can be, with the curvature between 0 and its field's top. A
    linear program on true temperatures `step_c` apart, each word then
    rounded to nearest. A request no such quadratic meets is refused."""
    def grid(lo, hi):
        # the small overshoot keeps hi on the grid
        t = np.arange(lo, hi + 1e-6, step_c)
        x = (raw_ref - raw_of_c(board, t)) / FIT_SCALE
        return np.column_stack([np.ones_like(x), x, x * x]), 100 * t

    rows, rhs = [], []

    def bound(points, sign, slack_cc, z):
        # sign x (quadratic - truth) <= slack_cc + 100 z at every point; z is the fourth unknown, in C
        basis, truth_cc = points
        rows.append(np.column_stack([sign * basis, np.full(len(basis), -100.0 * z)]))
        rhs.append(sign * truth_cc + slack_cc)

    pin, held, low = grid(pin_c, pin_c), grid(*range_c), grid(*low_range_c)
    bound(pin, +1, 0, 0)                    # at the pin: no higher than the truth
    bound(pin, -1, 0, 0)                    # and no lower
    bound(held, -1, -100 * floor_c, 0)      # over the range: no further under the truth than the floor
    bound(low, +1, 0, 1)                    # over the low stretch: no further over it than z
    top = k2_top_q24 / (1 << 24) * FIT_SCALE ** 2
    r = linprog([0, 0, 0, 1], A_ub=np.vstack(rows), b_ub=np.concatenate(rhs),
                bounds=[(None, None), (None, None), (0, top), (0, None)], method="highs")
    if r.status != 0:
        raise ValueError(f"no quadratic about count {raw_ref} with its curvature inside 0..{k2_top_q24} (Q24) reads "
                         f"the beta equation exactly at {pin_c:g} C and stays no more than {-floor_c:g} C under it "
                         f"from {range_c[0]:g} to {range_c[1]:g} C")
    t_ref, k1, k2 = r.x[0], r.x[1] / FIT_SCALE, r.x[2] / FIT_SCALE ** 2
    return int(round(t_ref)), int(round(k1 * 256)), int(round(k2 * (1 << 24)))


def fit_beta(r_ohm, t_c, r25_ohm):
    """The beta that puts the part's equation through measured (ohm, C) points."""
    inv = 1 / (np.asarray(t_c, float) + 273.15) - 1 / 298.15
    x = np.log(np.asarray(r_ohm, float) / r25_ohm)
    return float(np.sum(x * inv) / np.sum(inv * inv))


ADC_FULL = 4096         # the 12-bit converter's span, counts
RAIL_COUNTS = 8         # a count this close to a rail is no temperature (thermal.rs NTC_RAIL_COUNTS)


def ntc_cc(raw, raw_ref, t_ref_cc, k1_q88, k2_q24):
    """Centi-degrees as the kernel reads one count through its curve
    (core estimator/thermal.rs ntc_cc): the quadratic about `raw_ref`
    with the deviation held inside the converter's span and each product
    shifted down like the fixed-point multiply, which floors. None at a
    rail, without a curve (`k1_q88` 0), and where the result leaves the
    field the kernel keeps it in."""
    if k1_q88 == 0 or raw == 0 or raw >= ADC_FULL - 1:
        return None
    span = ADC_FULL - 1
    d = max(-span, min(span, raw_ref - raw))
    t = t_ref_cc + ((d * k1_q88) >> 8) + ((d * d * k2_q24) >> 24)
    return t if -32767 <= t <= 32767 else None


def raw_of_c(board, t_c):
    """The count the board's NTC reads at `t_c` by the part's nominal
    beta equation through its pull-up divider."""
    r = board.ntc_r25_ohm * np.exp(board.ntc_beta * (1 / (np.asarray(t_c, float) + 273.15) - 1 / 298.15))
    return board.adc_counts * r / (board.ntc_pullup_ohm + r)


def reads_c(board, curve, true_c):
    """The temperature the kernel reports through `curve` (raw_ref,
    t_ref_cc, k1_q88, k2_q24) with the board at a true `true_c`, the
    part's nominal beta equation as the truth; None where it reads
    none."""
    t = ntc_cc(int(round(float(raw_of_c(board, true_c)))), *curve)
    return None if t is None else t / 100


def reaches_c(board, curve, threshold_cc):
    """The true temperature, by the nominal beta equation, at which a
    warming board first reads `threshold_cc` through `curve`: the
    counts walked down from the reference to the last one short of the
    rail check. None for a curve that never reads it there."""
    for raw in range(int(curve[0]), RAIL_COUNTS, -1):
        t = ntc_cc(raw, *curve)
        if t is not None and t >= threshold_cc:
            return float(board.ntc_c_beta(raw))
    return None
