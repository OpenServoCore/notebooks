from dataclasses import astuple
from types import SimpleNamespace

import numpy as np
import pytest

from oscnb import speed as sp

FS = 20_000
ARR = 1200
CHAIN = sp.Chain(a_per_count=1e-3, tap_counts_per_v=250.0,
                 tap_offset_counts=2.0, half_period_ticks=ARR,
                 lag_rail_v=(4.0, 8.0), lag_on_ticks=(40.0, 44.0),
                 lag_off_ticks=(12.0, 16.0), fs_hz=FS)
KE = 7e-3
MOTOR = sp.Motor(r_ohm=4.0, t0_c=25.0, alpha_per_c=sp.ALPHA_CU, v0_v=0.03,
                 g_s=5e-3, brake_b0_v=0.01, brake_b1_ohm=0.25, l_h=1e-3,
                 ke_v_per_rev_s=KE)
KNOTS = np.arange(50.0, 451.0, 50.0)
# segment slopes as shares of Ke; the steep one (350 to 400) is 0.65 Ke
SHARES = np.array([0.1, 0.2, 0.05, 0.3, 0.4, 0.65, 0.2, 0.1])
POS = np.r_[0.02, 0.02 + np.cumsum(SHARES * KE * 50.0)]
NEG = POS / 2
TABLE = sp.BrushTable(KNOTS, POS, NEG)
PRODUCT = sp.BrushProduct(c0_v=-0.03, c_dir_v=0.015, b_v_per_rev_s_a=2e-3)
ZERO = 2048.0
RAIL = 6.0


def brush_volts(brush, d, w, i_w):
    if isinstance(brush, sp.BrushProduct):
        return (brush.c0_v + brush.c_dir_v * (d > 0)
                + brush.b_v_per_rev_s_a * w * i_w)
    return sp.table_eval(brush, d, w)


def rows_of(w, i_w, h, d, ntc, brush, rail_v=RAIL, motor=MOTOR):
    """crest, tap_a, tap_b of rows at speed w, current i_w, half on-ticks
    h and direction d, steady (no L di/dt), by inverting the model."""
    w, i_w, h, d = (np.broadcast_to(np.asarray(x, float), np.shape(w))
                    for x in (w, i_w, h, d))
    full = h >= ARR
    on, off = sp.edge_lags(np.full(len(w), rail_v), CHAIN)
    s = np.where(full, 1.0, (2 * h - on + off) / (2 * ARR))
    u = KE * w + brush_volts(brush, d, w, i_w)
    v_c = u + motor.v0_v + sp.r_of_t(ntc, motor) * i_w
    drop = np.where(full, 0.0, motor.brake_b0_v + motor.brake_b1_ohm * i_w)
    v_on = (v_c + (1 - s) * drop) / s
    crest = ZERO + (i_w + motor.g_s * (1 - h / ARR) * v_on) / CHAIN.a_per_count
    diff = (v_on * CHAIN.tap_counts_per_v + CHAIN.tap_offset_counts) * d
    return crest, 1500.0 + diff, np.full(len(w), 1500.0)


def estimate(w, i_w, h, d, ntc, brush, **kw):
    n = len(w)
    crest, ta, tb = rows_of(w, i_w, h, d, ntc, brush)
    return sp.estimate_rows(crest, np.full(n, ZERO), ta, tb, h * d, d,
                            np.full(n, ntc), CHAIN, MOTOR, brush,
                            rail_v=np.full(n, RAIL), **kw)


@pytest.mark.parametrize("brush", [TABLE, PRODUCT], ids=["table", "product"])
@pytest.mark.parametrize("h", [480.0, 900.0, ARR], ids=["40", "75", "100"])
def test_rows_from_a_known_speed_come_back_as_that_speed(brush, h):
    w = np.array([30.0, 120.0, 260.0, 380.0, 470.0, 120.0, 260.0])
    i_w = np.array([0.05, 0.10, 0.15, 0.20, 0.25, 0.12, 0.30])
    d = np.array([1, 1, 1, 1, 1, -1, -1])
    got, u, i_got = estimate(w, i_w, np.full(len(w), h), d, 40.0, brush)
    assert got == pytest.approx(w, abs=1e-9)
    assert i_got == pytest.approx(i_w, abs=1e-12)
    assert u == pytest.approx(KE * w + brush_volts(brush, d, w, i_w),
                              abs=1e-12)


def test_at_full_duty_there_is_no_reading_term_and_no_brake_drop():
    crest = np.array([2148.0, 2348.0, 1948.0])
    v_on = np.array([5.0, 7.4, 3.7])
    duty = np.array([ARR, -ARR, ARR])
    other = sp.Motor(r_ohm=4.0, t0_c=25.0, alpha_per_c=sp.ALPHA_CU,
                     v0_v=0.03, g_s=0.05, brake_b0_v=0.5, brake_b1_ohm=3.0,
                     l_h=1e-3, ke_v_per_rev_s=KE)
    for motor in (MOTOR, other):
        i_w, v_c, d_share, s = sp.construct(crest, ZERO, v_on, duty, CHAIN,
                                            motor, rail_v=RAIL)
        assert i_w == pytest.approx((crest - ZERO) * CHAIN.a_per_count)
        assert v_c == pytest.approx(v_on)
        assert list(s) == [1.0, 1.0, 1.0]
        assert list(d_share) == [1.0, 1.0, 1.0]


def test_r_of_t_is_r0_at_t0_and_rises_0p39_percent_per_c():
    assert sp.ALPHA_CU == 0.0039
    t0, r0 = MOTOR.t0_c, MOTOR.r_ohm
    r = sp.r_of_t(np.array([t0, t0 + 1, t0 + 10, t0 - 20, np.nan]), MOTOR)
    assert r == pytest.approx([r0, r0 * 1.0039, r0 * 1.039, r0 * 0.922, r0])


def test_the_product_solve_meets_its_equation_exactly():
    rng = np.random.default_rng(3)
    u = rng.uniform(-0.5, 4.5, 1000)
    i_w = rng.uniform(-0.2, 0.6, 1000)
    d = rng.choice([1, -1], 1000)
    w = sp.solve_product(u, i_w, d, PRODUCT, KE)
    assert KE * w + brush_volts(PRODUCT, d, w, i_w) == pytest.approx(
        u, rel=0, abs=1e-14)


def ramp_u(d):
    """u of a speed ramp from 20 to 470 rev/s, 0.1 rev/s per row, on the
    table in direction d, and that speed."""
    w = np.arange(20.0, 470.0, 0.1)
    dd = np.full(len(w), d)
    return KE * w + sp.table_eval(TABLE, dd, w), dd, w


@pytest.mark.parametrize("d", [1, -1])
def test_two_newton_steps_from_the_row_before_meet_the_bisection(d):
    u, dd, w = ramp_u(d)
    first = np.zeros(len(u), bool)
    first[0] = True
    ref = sp.solve_bisect(u, dd, TABLE, KE)
    assert ref == pytest.approx(w, abs=1e-9)
    two = sp.solve_steps(u, dd, first, TABLE, KE, steps=2, kind="newton")
    assert np.abs(two - ref).max() < 1e-6


def test_one_fixed_step_does_not_meet_the_bisection_on_a_steep_segment():
    u, dd, w = ramp_u(1)
    first = np.zeros(len(u), bool)
    first[0] = True
    ref = sp.solve_bisect(u, dd, TABLE, KE)
    one = sp.solve_steps(u, dd, first, TABLE, KE, steps=1, kind="fixed")
    steep = (w > 360) & (w < 400)
    assert np.abs(one - ref)[steep].min() > 1e-2
    newton = sp.solve_steps(u, dd, first, TABLE, KE, steps=1, kind="newton")
    assert np.abs(newton - ref)[steep].max() < 1e-6


def test_table_slopes_are_each_segment_as_a_share_of_ke():
    tab = sp.BrushTable(np.r_[KNOTS, 500.0], np.r_[POS, np.nan],
                        np.r_[NEG, np.nan])
    (kp, sp_pos), (kn, sp_neg) = sp.table_slopes(tab)
    assert list(kp) == list(KNOTS[:-1]) and list(kn) == list(KNOTS[:-1])
    assert sp_pos / KE == pytest.approx(SHARES)
    assert sp_neg / KE == pytest.approx(SHARES / 2)


def test_trailing_didt_recovers_a_ramp_and_looks_at_no_later_row():
    k = np.arange(400)
    i_w = 0.1 + 50.0 * k / FS           # 50 A/s
    first = np.zeros(len(k), bool)
    first[200] = True
    out = sp.trailing_didt(i_w, FS, 16, first)
    assert out[0] == 0.0 and out[200] == 0.0
    assert np.delete(out, [0, 200]) == pytest.approx(50.0)
    for m in (5, 100, 210, 399):
        later = i_w.copy()
        later[m:] += np.random.default_rng(m).normal(0, 1, len(k) - m)
        assert np.array_equal(sp.trailing_didt(later, FS, 16, first)[:m],
                              out[:m])


@pytest.mark.parametrize("kw", [dict(brush=TABLE), dict(brush=PRODUCT),
                                dict(brush=TABLE, steps=2),
                                dict(brush=TABLE, with_l=True)],
                         ids=["bisect", "product", "steps", "with_l"])
def test_estimate_rows_looks_at_no_later_row(kw):
    rng = np.random.default_rng(7)
    n = 600
    w = 200 + np.cumsum(rng.normal(0, 1, n))
    i_w = 0.15 + rng.normal(0, 0.01, n)
    h = rng.uniform(400, ARR, n)
    d = np.where(np.arange(n) < 300, 1, -1)
    brush = kw.pop("brush")
    crest, ta, tb = rows_of(w, i_w, h, d, 40.0, brush)
    first = np.zeros(n, bool)
    first[450] = True
    cols = [crest, np.full(n, ZERO), ta, tb, h * d, d, np.full(n, 40.0)]
    base = sp.estimate_rows(*cols, CHAIN, MOTOR, brush, first=first, **kw)
    for m in (1, 299, 300, 451, 599):
        later = [c.copy() for c in cols]
        for c in later[:5]:
            c[m:] += rng.normal(0, 5, n - m)
        got = sp.estimate_rows(*later, CHAIN, MOTOR, brush, first=first,
                               **kw)
        for a, b in zip(got, base):
            assert np.array_equal(a[:m], b[:m])


def q(v):
    return {"value": v, "lo": None, "hi": None, "unit": ""}


def test_binding_reads_the_result_keys_and_the_scales():
    res = {"r_rails_at_measured_e_ohm": q(4.2), "board_ntc_c": q(26.0),
           "v0_rails_at_measured_e_v": q(0.033), "meter_g_ms": q(4.7)}
    ind = {"brake_line_motor_plain": {"intercept_mv": q(8.0),
                                      "slope_ohm": q(0.24)},
           "l_mh": q(0.82)}
    lad = {"ke_mv_per_rev_s": q(7.28),
           "k_d_mv_per_dropout_s": {"A": q(0.8), "B": q(0.78)}}
    mo = sp.motor_of(res, ind, lad)
    assert astuple(mo) == pytest.approx((4.2, 26.0, sp.ALPHA_CU, 0.033,
                                         4.7e-3, 8.0e-3, 0.24, 0.82e-3,
                                         7.28e-3, 0.78e-3))
    assert sp.motor_of(res, ind, lad, k_d=0.0).k_d_v_per_dropout_s == 0.0
    scales = SimpleNamespace(
        a_per_count=1 / 1135.0, terminal_counts_per_v=248.0,
        terminal_offset_counts=2.4,
        edge_lags_at=lambda v: (37.0 + (v - 3.7) * 5 / 3.7,
                                13.0 + (v - 3.7) * 2 / 3.7, 0.0, 0.0))
    board = SimpleNamespace(pwm_arr=1200, tick_hz=20_000)
    ch = sp.chain_of(board, scales)
    assert ch.lag_rail_v == (3.7, 7.4)
    assert ch.lag_on_ticks == pytest.approx((37.0, 42.0))
    assert ch.lag_off_ticks == pytest.approx((13.0, 15.0))
    assert (ch.half_period_ticks, ch.fs_hz) == (1200, 20_000.0)
    comp = {"v_b_table_mv": {"signed": {"knots_rev_s": [50.0, 100.0, 150.0],
                                        "+1": [-5.0, 10.0, None],
                                        "-1": [-4.0, 8.0, 20.0]}},
            "v_b_product_form": {"c0_mv": q(-35.0), "c_dir_mv": q(14.0),
                                 "b_mv_per_rev_s_a": q(1.96)}}
    t = sp.brush_table(comp)
    assert list(t.pos[:2]) == pytest.approx([-5e-3, 10e-3])
    assert np.isnan(t.pos[2]) and t.neg[2] == pytest.approx(20e-3)
    assert sp.table_eval(t, np.array([1]), np.array([200.0])) == (
        pytest.approx([10e-3]))
    assert astuple(sp.brush_product(comp)) == pytest.approx(
        (-35e-3, 14e-3, 1.96e-3))


EVENTS_PER_REV = 6


def test_a_boxcar_and_a_one_pole_delay_a_ramp_by_their_dc_delay():
    k = np.arange(4000.0)
    for n in (1, 8, 16, 100):
        y = sp.boxcar(k, n)
        assert y[n - 1:] == pytest.approx(k[n - 1:] - (n - 1) / 2, abs=1e-9)
    for tau in (4.0, 16.0, 100.0):
        a = sp.pole_alpha(tau)
        y = sp.one_pole(k, a)
        assert y[0] == 0.0
        assert k[-200:] - y[-200:] == pytest.approx((1 - a) / a, abs=1e-9)


def ripple(period_rows):
    """5 + a sine of the given period per row, phase continuous."""
    phase = np.cumsum(2 * np.pi / np.asarray(period_rows, float))
    return 5.0 + np.sin(phase)


def test_a_boxcar_of_one_commutation_period_nulls_its_ripple():
    period = 40
    x = ripple(np.full(2000, period))
    assert np.abs(sp.boxcar(x, period)[period:] - 5.0).max() < 1e-12
    assert np.abs(sp.boxcar(x, period + 7)[period:] - 5.0).max() > 1e-2


def test_the_locked_window_follows_the_commutation_period():
    periods = np.r_[np.full(1000, 40), np.full(1000, 20)]
    w = FS / (EVENTS_PER_REV * periods)
    n = sp.lock_rows(w, FS, EVENTS_PER_REV, 1, 4, 800)
    assert np.array_equal(n, periods)
    assert np.array_equal(sp.lock_rows(w, FS, EVENTS_PER_REV, 2, 4, 800),
                          2 * periods)
    y = sp.locked_boxcar(ripple(periods), n)
    assert np.abs(y[40:1000] - 5.0).max() < 1e-12
    assert np.abs(y[1020:] - 5.0).max() < 1e-12
    edge = sp.lock_rows(np.array([0.0, np.nan, 1.0, -1.0, 1e6, -1e6]), FS,
                        EVENTS_PER_REV, 1, 4, 800)
    assert list(edge) == [800, 800, 800, 800, 4, 4]


def test_slope_line_recovers_a_ramp_restarts_and_looks_at_no_later_row():
    k = np.arange(400)
    i_w = 0.1 + 50.0 * k / FS           # 50 A/s
    i_w[200:] += 0.3                    # a jump the restart must not see
    first = np.zeros(len(k), bool)
    first[200] = True
    for n in (2, 20, 100):
        out = sp.slope_line(i_w, n, FS, first)
        assert out[0] == 0.0 and out[200] == 0.0
        assert np.delete(out, [0, 200]) == pytest.approx(50.0, rel=1e-9)
        for m in (5, 100, 210, 399):
            later = i_w.copy()
            later[m:] += np.random.default_rng(m).normal(0, 1, len(k) - m)
            assert np.array_equal(sp.slope_line(later, n, FS, first)[:m],
                                  out[:m])


def test_the_tap_floor_is_false_under_its_ticks_and_true_at_them():
    duty = np.array([0.0, 83.0, -83.0, 84.0, -84.0, 85.0, ARR, -ARR])
    assert list(sp.tap_floor_valid(duty, 84)) == [False, False, False, True,
                                                  True, True, True, True]


DROP = dict(frac=0.25, alpha=sp.pole_alpha(32), floor_a=5e-3, hold_rows=39,
            max_rows=8)


def test_dropout_flags_a_single_low_row_under_a_steady_current():
    i_w = np.full(1000, 0.2)
    i_w[[20, 500]] = 0.02               # row 20 is inside the hold
    fl = sp.dropout_flag(i_w, **DROP)
    assert list(np.flatnonzero(fl)) == [500]


def test_dropout_does_not_flag_while_the_reference_is_under_its_floor():
    i_w = np.full(1000, 4e-3)
    i_w[500] = 0.0
    assert not sp.dropout_flag(i_w, **DROP).any()


def test_dropout_does_not_latch_after_a_step_down_of_the_current():
    i_w = np.r_[np.full(600, 0.2), np.full(600, 0.02)]
    fl = sp.dropout_flag(i_w, **DROP)
    assert fl[600:608].all()
    assert not fl[:600].any() and not fl[608:].any()
    latched = sp.dropout_flag(i_w, **{**DROP, "max_rows": 0})
    assert latched[600:].all()


def test_holdoff_marks_n_rows_after_torque_on_and_after_a_flip():
    first = np.zeros(500, bool)
    first[300] = True
    flip = np.zeros(500, bool)
    flip[150] = True
    ok = sp.holdoff_rows(first, flip, 16, 10)
    held = np.r_[0:16, 150:160, 300:316]
    assert list(np.flatnonzero(~ok)) == list(held)
    assert sp.holdoff_rows(first, flip, 16, 0)[150:160].all()


def test_hold_last_holds_the_last_valid_value():
    x = np.arange(10.0)
    valid = np.array([0, 1, 1, 0, 0, 1, 0, 1, 1, 0], bool)
    first = np.zeros(10, bool)
    first[6] = True
    out = sp.hold_last(x, valid, first)
    assert np.isnan(out[[0, 6]]).all()
    assert list(np.delete(out, [0, 6])) == [1, 2, 2, 2, 5, 7, 8, 8]


def test_one_pole_skip_does_not_move_on_a_flagged_row():
    rng = np.random.default_rng(5)
    x = rng.normal(100, 5, 2000)
    valid = rng.random(2000) > 0.1
    valid[:2] = False, True
    a = sp.pole_alpha(16)
    out = sp.one_pole_skip(x, a, valid)
    assert np.isnan(out[0]) and out[1] == x[1]
    k = np.flatnonzero(~valid[1:]) + 1
    k = k[k > 1]
    assert np.array_equal(out[k], out[k - 1])
    assert np.array_equal(out[valid], sp.one_pole(x[valid], a))


def test_valid_rows_is_the_and_of_the_three_rules():
    rng = np.random.default_rng(9)
    n = 3000
    duty = rng.integers(-ARR, ARR + 1, n).astype(float)
    i_w = np.full(n, 0.2)
    i_w[rng.random(n) < 0.01] = 0.01
    first = rng.random(n) < 0.003
    flip = (rng.random(n) < 0.005) & ~first
    args = (84, 16, 16, DROP["frac"], DROP["alpha"], DROP["floor_a"],
            DROP["hold_rows"], DROP["max_rows"])
    valid, tap_ok, started, drop = sp.valid_rows(duty, i_w, first, flip,
                                                 *args)
    assert np.array_equal(tap_ok, sp.tap_floor_valid(duty, 84))
    assert np.array_equal(started, sp.holdoff_rows(first, flip, 16, 16))
    assert np.array_equal(drop, sp.dropout_flag(
        i_w, DROP["frac"], DROP["alpha"], DROP["floor_a"], first,
        DROP["hold_rows"], DROP["max_rows"]))
    assert np.array_equal(valid, tap_ok & started & ~drop)
    assert tap_ok.any() and started.any() and drop.any()
    assert (~valid).sum() > max((~tap_ok).sum(), (~started).sum(),
                                drop.sum())
