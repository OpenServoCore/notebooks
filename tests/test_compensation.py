import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stages" / "2-motor" / "compensation"))
import compensation as cp  # noqa: E402


def test_fixed_point_meets_the_closed_form_for_a_linear_drop():
    # V_b = a + b w (mV, w in rev/s) makes w = (u - V_b(w)) / (2 pi Ke) linear, w = (u - a/1000) / (2 pi Ke + b/1000)
    ke, a, b = 1.17e-3, 5.0, 0.2
    u = np.array([0.5, 1.5, 3.0])
    w = cp.solve_w(u, lambda w: a + b * w, ke)
    assert w == pytest.approx((u - a * 1e-3) / (2 * np.pi * ke + b * 1e-3), rel=1e-9)


def test_fixed_point_with_no_drop_is_the_plain_estimate():
    u = np.array([0.2, 2.0])
    assert cp.solve_w(u, lambda w: np.zeros_like(w), cp.KE) == pytest.approx(u / cp.KE_REV)


def test_fixed_point_meets_a_power_law_drop():
    # w (2 pi Ke) + A (w/300)^n / 1000 = u at the solution
    A, n, ke = 60.0, 4.0, cp.KE
    u = np.array([1.0, 2.5, 3.6])
    w = cp.solve_w(u, lambda w: A * (w / 300) ** n, ke)
    assert w * 2 * np.pi * ke + A * (w / 300) ** n * 1e-3 == pytest.approx(u, rel=1e-6)


def test_f_table_is_the_rail_mean_per_volt_flat_below_and_zero_at_full_duty():
    GR = pd.DataFrame(dict(loop=["no meter"] * 4 + ["meter"] * 2, rail=[5.0, 5.0, 7.4, 7.4, 5.0, 7.4],
                           duty=[20, 60, 20, 60, 20, 20], pub=[20.0, 15.0, 29.6, 22.2, 99.0, 99.0], b=[100.0, 50.0, 148.0, 74.0, 0.0, 0.0]))
    d, v = cp.f_table(GR)
    assert list(d) == [0.0, 20.0, 60.0, 100.0]
    assert list(v) == pytest.approx([4.0, 4.0, 3.0, 0.0])          # 20/5 and 29.6/7.4 are both 4 mA/V
    # moving G by 1 mS adds b G / rail, here 0.02 and 0.01 mA/V at each rail
    d2, v2 = cp.f_table(GR, 1e-3)
    assert list(v2) == pytest.approx([4.02, 4.02, 3.01, 0.0])
    assert np.interp(40, d, v) == pytest.approx(3.5)


def test_vb_eval_interpolates_the_table_and_holds_its_last_knot():
    tab = np.linspace(0, 500, len(cp.VB_KNOTS))
    fit = {1: dict(power=(60.0, 4.0, np.nan), table=tab), -1: dict(power=(30.0, 3.0, np.nan), table=tab / 2)}
    k = cp.VB_KNOTS
    w = np.array([k[0], (k[1] + k[2]) / 2, k[-1] + 100])
    assert cp.vb_eval(fit, 1, w, "table") == pytest.approx([tab[0], (tab[1] + tab[2]) / 2, tab[-1]])
    assert cp.vb_eval(fit, -1, w, "table") == pytest.approx([tab[0] / 2, (tab[1] + tab[2]) / 4, tab[-1] / 2])
    assert cp.vb_eval(fit, np.array([1, -1]), np.array([300.0, 600.0]), "power") == pytest.approx([60.0, 30.0 * 8])
    assert cp.vb_eval(fit, 1, np.array([-5.0]), "power") == pytest.approx([0.0])   # a negative speed reads as zero


def test_band_edges_belong_to_the_upper_band():
    assert cp.band_of(99.9) == "0-100" and cp.band_of(100.0) == "100-160"
    assert cp.band_of(450.0) == "450-600" and cp.band_of(650.0) == "other"


def test_score_table_gives_mean_and_rms_in_percent_and_rev_s():
    df = pd.DataFrame(dict(g=["a", "a"], truth=[100.0, 200.0], w_x=[102.0, 196.0]))
    r = cp.score_table(df, ["g"], variants=("x",)).iloc[0]
    assert r["x_pct"] == pytest.approx(0.0) and r["x_rms_pct"] == pytest.approx(2.0)
    assert r["x_revs"] == pytest.approx(-1.0) and r["x_rms_revs"] == pytest.approx(np.sqrt(10.0))


def test_g_fit_recovers_a_known_g_and_its_weighting():
    a = np.array([10.0, 12.0, 20.0, 25.0])
    b = np.array([3000.0, 3500.0, 5000.0, 6000.0])
    g = 3.3e-3
    gap = a + b * g
    sig = np.ones(4)
    G, se, chi2 = cp.g_fit(gap, a, b, sig, np.ones(4, bool))
    assert G == pytest.approx(g) and chi2 == pytest.approx(0.0, abs=1e-18)
    assert se == pytest.approx(1 / np.sqrt(np.sum(b ** 2)))       # chi2 under one leaves the se unscaled


def test_quadrature_and_the_unexplained_part():
    assert cp.quadrature(3.0, 4.0) == pytest.approx(5.0)
    assert cp.unexplained(13.0, 5.0) == pytest.approx(12.0)
    assert cp.unexplained(4.0, 5.0) == 0.0
