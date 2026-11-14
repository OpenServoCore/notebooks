"""The board NTC two ways: the part's beta equation, and a fixed-point
quadratic in the count, the form a small chip can carry, so a notebook can
fit the quadratic and price what the form costs."""
import numpy as np


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
