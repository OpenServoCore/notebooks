import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stages" / "2-motor" / "motor-model"))
import brush_drop as bd  # noqa: E402


def test_edge_lags_hit_sample_timing_at_its_two_rails_and_the_line_between():
    assert bd.edge_lags(3.7) == pytest.approx((bd.LAG_ON[0], bd.LAG_OFF[0]))
    assert bd.edge_lags(7.4) == pytest.approx((bd.LAG_ON[1], bd.LAG_OFF[1]))
    mid = bd.edge_lags((3.7 + 7.4) / 2)
    assert mid == pytest.approx((np.mean(bd.LAG_ON), np.mean(bd.LAG_OFF)))


def test_share_is_the_driven_ticks_over_the_period():
    # a half window of h ticks drives 2h ticks, the ON edge late by on_lag and the OFF edge late by off_lag
    assert bd.share_of(300, 40.0, 15.0) == pytest.approx((600 - 40 + 15) / bd.P)
    assert bd.share_of(bd.ARR, 0.0, 0.0) == pytest.approx(1.0)


def test_open_share_inverts_the_interruption_model():
    # a path open a share f with the drive held gives V - E = R i / (1 - f), so x = R i f / (1 - f)
    r, i = 4.455, 0.108
    for f in (0.01, 0.2, 0.43, 0.9):
        x = r * i * f / (1 - f)
        assert bd.open_share(x, r, i) == pytest.approx(f, rel=1e-12)
    assert bd.open_share(0.0, r, i) == 0.0


def test_arc_volts_are_half_l_i_rate():
    # 1/2 L i^2 per opening times the rate, over i
    l, i, rate = 0.5e-3, 0.1, 1000.0
    assert bd.arc_mv(l, i, rate) == pytest.approx(0.5 * l * i ** 2 * rate / i * 1e3)
    assert bd.arc_mv(l, i, rate) == pytest.approx(25.0)


def test_gap_rate_is_segments_times_speed():
    assert bd.gap_hz(300.0) == pytest.approx(bd.MOTOR.slots * 300.0)
    assert bd.gap_hz(300.0, segments=3) == pytest.approx(900.0)


def test_step_read_tends_to_r_between_the_two_time_constants():
    # well after L/R and well before J R / Ke^2 the step read is R; at t = 0 it diverges, late it inflates
    te = 0.5e-3 / bd.R_LOCKED
    tm = bd.J * bd.R_LOCKED / bd.KE ** 2
    assert te < 1e-3 < tm
    t = 20 * te
    assert bd.step_read(0.5e-3, t) == pytest.approx(1 / (1 - (1 - np.exp(-t / tm))), rel=1e-6)
    assert bd.step_read(0.5e-3, 3e-3) > bd.step_read(0.5e-3, 1e-3) > 1.0


def test_power_law_recovers_an_exact_law():
    w = np.array([215.0, 240.0, 330.0, 490.0, 550.0])
    h = pd.DataFrame(dict(rev_s=w, x_b=45.0 * (w / 300) ** 3.55))
    f = bd.power_law(h)
    assert f["mv_300"] == pytest.approx(45.0, rel=1e-9)
    assert f["exponent"] == pytest.approx(3.55, rel=1e-9)
    assert bd.v_b(300.0, f["mv_300"], f["exponent"]) == pytest.approx(45.0, rel=1e-9)
    # 1 % of the EMF: 45 (w/300)^3.55 mV = 0.01 Ke 2 pi w V
    w1 = f["one_pct_rev_s"]
    assert 45.0 * (w1 / 300) ** 3.55 == pytest.approx(0.01 * bd.KE * 2 * np.pi * w1 * 1e3, rel=1e-3)


def test_wls_fits_a_line_exactly():
    x = np.array([1.0, 2.0, 3.0, 4.0])
    f = bd.wls(x, 2.0 + 0.5 * x, np.ones(4))
    assert (f["a"], f["b"]) == pytest.approx((2.0, 0.5))
    assert np.abs(f["res"]).max() < 1e-12


def test_fit_r_through_the_origin():
    i = np.array([0.1, 0.3, 0.7])
    r, res = bd.fit_r(4.5 * i, i)
    assert r == pytest.approx(4.5)
    assert np.abs(res).max() < 1e-12


def test_phase_fit_reads_a_one_cycle_term():
    ph = np.linspace(0, 1, 60, endpoint=False)
    amp, se, rs = bd.phase_fit(ph, 1.0 + 0.3 * np.cos(2 * np.pi * ph) - 0.4 * np.sin(2 * np.pi * ph))
    assert amp == pytest.approx(0.5)
    assert rs < 1e-12
