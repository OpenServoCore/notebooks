"""The board NTC two ways: the part's beta equation, and the quadratic the
firmware converts with (`ntc_cc` in the core's thermal estimator), so a
notebook can grade the firmware's curve and refit it."""
import numpy as np


def firmware_cc(raw, raw_ref, t_ref_cc, k1_q88, k2_q24):
    """Centi-degrees as the firmware computes them: t_ref + k1 x d + k2 x d^2
    with d = raw_ref - raw, k1 in Q8.8 and k2 in Q24, truncating like the
    fixed-point multiply."""
    raw = np.asarray(raw, dtype=np.int64)
    d = raw_ref - raw
    return t_ref_cc + ((d * k1_q88) >> 8) + ((d * d * k2_q24) >> 24)


def fit_quadratic(raw, t_c, raw_ref):
    """(t_ref_cc, k1_q88, k2_q24) that put the firmware's quadratic through
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
