import numpy as np
import pytest

from oscnb import velocity as vel

FS = 20_000.0
N = 64
POLES = ("pole1", "pole16")


def ramp(rows, slope_per_s, x0=100.0):
    return x0 + slope_per_s * np.arange(rows) / FS


@pytest.mark.parametrize("kind", vel.KINDS)
def test_ramp_reads_its_slope_after_the_span(kind):
    slope = 37.5
    span = vel.span_rows(kind, N)
    v = vel.run(kind, ramp(span + 400, slope), N, FS)
    first = int(np.flatnonzero(np.isfinite(v))[0])
    assert first == span - 1 + (kind in POLES)
    assert np.isnan(v[:first]).all()
    tol = 1e-2 if kind in POLES else 1e-9
    np.testing.assert_allclose(v[first:], slope, rtol=tol)


@pytest.mark.parametrize("kind", vel.KINDS)
def test_delay_on_a_step_in_slope_is_nominal(kind):
    n = 40
    t0 = 6000
    rows = t0 + 40 * n + 2000
    t = np.arange(rows)
    x = np.where(t < t0, 0.0, (t - t0) / FS)
    v = vel.run(kind, x, n, FS)
    u_true = np.where(t < t0, 0.0, 1.0)
    u_true[t0] = 0.5
    sel = t >= t0 - vel.span_rows(kind, n) - 10
    assert np.isfinite(v[sel]).all()
    delay = float(np.sum(u_true[sel] - v[sel]))
    assert abs(delay - vel.nominal_delay(kind, n)) < 1e-6


def test_two_point_of_a_multiple_of_the_pattern_nulls_it():
    rng = np.random.default_rng(3)
    pattern = rng.normal(0, 1, 16)
    x = 500.0 + np.tile(pattern, 40)
    assert np.all(vel.two_point(x, 32, FS)[32:] == 0.0)
    assert np.abs(vel.two_point(x, 20, FS)[20:]).max() > 1.0


def test_two_point_skip_steps_over_a_carry():
    slope_rows = 0.05
    t = np.arange(4000)
    p = 1000.0 + slope_rows * t
    code, missing = 1050, 6
    x = np.where((p >= code) & (p < code + missing + 1), float(code), p)
    ok = ~vel.carry_band(x, [code], [missing], 2)
    n = 100
    plain = vel.two_point(x, n, FS)
    skip, held = vel.two_point_skip(x, ok, n, FS)
    want = slope_rows * FS
    fin = np.isfinite(skip)
    assert fin[n:].all()
    np.testing.assert_allclose(skip[fin], want, rtol=1e-9)
    assert held[~ok].all()
    assert np.nanmax(plain) > 1.5 * want


def test_rest_band_declares_after_the_hold_and_drops_leaving_the_band():
    t_rows, band, r0 = 300, 2.0, 1000
    t = np.arange(1500)
    y = np.where(t < r0, 7.0, 7.0 + 0.01 * (t - r0))
    rest = vel.rest_band(y, band, t_rows)
    assert not rest[:t_rows - 1].any()
    assert rest[t_rows - 1:r0].all()
    leave = int(np.flatnonzero(np.abs(y - 7.0) > band)[0])
    assert rest[leave - 1] and not rest[leave:].any()


def test_rest_band_carry_rows_need_the_longer_hold():
    y = np.full(1000, 3.0)
    in_carry = np.zeros(1000, bool)
    in_carry[200:] = True
    rest = vel.rest_band(y, 1.0, 100, in_carry, 600)
    assert rest[99:200].all()
    assert not rest[200:599].any()
    assert rest[599:].all()


def test_rest_slope_declares_after_the_hold_and_drops_at_the_threshold():
    t_rows, thr, r0 = 300, 5.0, 1000
    t = np.arange(1500)
    s = np.where(t < r0, 0.0, 0.02 * (t - r0))
    rest = vel.rest_slope(s, thr, t_rows)
    assert not rest[:t_rows - 1].any()
    assert rest[t_rows - 1:r0].all()
    over = int(np.flatnonzero(np.abs(s) >= thr)[0])
    assert rest[over - 1] and not rest[over:].any()


def test_rest_slope_restarts_on_nan():
    s = np.zeros(800)
    s[400] = np.nan
    rest = vel.rest_slope(s, 1.0, 100)
    assert rest[99:400].all()
    assert not rest[400:500].any()
    assert rest[500:].all()


def test_veto_blocks_a_declaration():
    y = np.zeros(1000)
    s = np.zeros(1000)
    veto = np.zeros(1000, bool)
    veto[500] = True
    for rest in (vel.rest_band(y, 1.0, 100, veto=veto),
                 vel.rest_slope(s, 1.0, 100, veto=veto)):
        assert rest[99:500].all()
        assert not rest[500:600].any()
        assert rest[600:].all()
    always = np.ones(1000, bool)
    assert not vel.rest_band(y, 1.0, 100, veto=always).any()
    assert not vel.rest_slope(s, 1.0, 100, veto=always).any()


def test_rest0_starts_declared():
    y = np.zeros(50)
    assert vel.rest_band(y, 1.0, 100, rest0=True).all()
    assert vel.rest_slope(y, 1.0, 100, rest0=True).all()


def test_emf_turning_reads_off_the_stopped_rotors_line():
    i_w = np.array([0.0, 0.1, 0.2, 0.2, 0.2, -1.0])
    a, b, margin = 1.0, 10.0, 0.5
    w_f = np.array([1.0, 2.0, 3.6, 3.4, 2.0, 0.6])
    carry = np.array([True, True, True, True, False, True])
    got = vel.emf_turning(w_f, carry, i_w, a, b, margin)
    np.testing.assert_array_equal(got, [False, False, True, False, False,
                                        True])


def test_emf_signed_holds_dropouts_inside_a_usable_run():
    w = np.array([5.0, 6.0, 7.0, 8.0, 9.0, 10.0])
    drive = np.array([-1, -1, -1, -1, 1, 1])
    tap_ok = np.array([True, True, True, True, False, True])
    started = np.ones(6, bool)
    drop = np.array([False, False, True, False, False, False])
    out, usable = vel.emf_signed(w, drive, tap_ok, started, drop)
    np.testing.assert_array_equal(out, [-5.0, -6.0, -6.0, -8.0, np.nan,
                                        10.0])
    np.testing.assert_array_equal(usable, [True] * 4 + [False, True])


def test_emf_boxcar_restarts_after_a_gap():
    w = np.arange(20, dtype=float)
    usable = np.ones(20, bool)
    usable[8] = False
    out, carry = vel.emf_boxcar(w, usable, 4)
    want_carry = np.zeros(20, bool)
    want_carry[3:8] = True
    want_carry[12:] = True
    np.testing.assert_array_equal(carry, want_carry)
    assert np.isnan(out[~carry]).all()
    np.testing.assert_allclose(out[12], np.mean(w[9:13]))
    np.testing.assert_allclose(out[7], np.mean(w[4:8]))


def test_switch_takes_the_emf_on_its_carry_rows():
    rng = np.random.default_rng(5)
    v_pot, v_emf = rng.normal(size=200), rng.normal(size=200) + 10.0
    carry = rng.random(200) < 0.5
    v, src = vel.switch(v_pot, v_emf, carry)
    np.testing.assert_array_equal(v[carry], v_emf[carry])
    np.testing.assert_array_equal(v[~carry], v_pot[~carry])
    np.testing.assert_array_equal(src, carry.astype(np.int8))
    rest = np.zeros(200, bool)
    rest[50:60] = True
    z = vel.with_rest(v, rest)
    assert np.all(z[50:60] == 0.0)
    np.testing.assert_array_equal(z[~rest], v[~rest])


def pair_inputs(rows, step_row, bias, m=16, n=48):
    t = np.arange(rows)
    w = np.where(t < step_row, 20.0, 35.0)
    usable = np.ones(rows, bool)
    angle = np.cumsum(w) / FS
    v_pot = vel.two_box(angle, m, n, FS)
    v_emf = w + bias
    s_emf, good = vel.emf_through_slope(v_emf, usable, m, n, FS)
    return w, v_pot, v_emf, s_emf, good


def test_complementary_reads_the_pot_at_dc_and_the_emf_step_at_once():
    tau, bias, rows, step_row = 200.0, 1.5, 20_000, 15_000
    w, v_pot, v_emf, s_emf, good = pair_inputs(rows, step_row, bias)
    carry = np.ones(rows, bool)
    v, src = vel.complementary(v_pot, v_emf, s_emf, carry, good, tau)
    assert abs(v[step_row - 1] - w[step_row - 1]) < 1e-6
    jump = v[step_row] - v[step_row - 1]
    assert abs(jump - (w[step_row] - w[step_row - 1])) < 1e-6
    assert abs(v[step_row] - w[step_row]) < 1e-6
    assert (src[good] == 2).all()
    assert (src[~good] == 1).all()


def test_complementary_restarts_after_a_gap_in_the_carry():
    tau, bias, rows = 200.0, 1.5, 12_000
    _, v_pot, v_emf, s_emf, good = pair_inputs(rows, rows, bias)
    carry = np.ones(rows, bool)
    carry[6000:6100] = False
    v, src = vel.complementary(v_pot, v_emf, s_emf, carry, good, tau)
    np.testing.assert_array_equal(v[6000:6100], v_pot[6000:6100])
    assert (src[6000:6100] == 0).all()
    fresh, _ = vel.complementary(v_pot[6100:], v_emf[6100:], s_emf[6100:],
                                 carry[6100:], good[6100:], tau)
    np.testing.assert_array_equal(v[6100:], fresh)
    assert abs(v[6100] - v_emf[6100]) < abs(v[5999] - v_emf[5999])


def test_seams_finds_where_the_source_changes():
    v = np.arange(10, dtype=float) ** 2
    source = np.array([0, 0, 1, 1, 1, 2, 2, 0, 0, 0], np.int8)
    r, jump, s_from, s_to = vel.seams(v, source)
    np.testing.assert_array_equal(r, [2, 5, 7])
    np.testing.assert_array_equal(jump, v[r] - v[r - 1])
    np.testing.assert_array_equal(s_from, [0, 1, 2])
    np.testing.assert_array_equal(s_to, [1, 2, 0])


def test_two_box_is_two_boxcars_n_apart():
    rng = np.random.default_rng(7)
    x = np.cumsum(rng.normal(size=500))
    m, n = 16, 40
    v = vel.two_box(x, m, n, FS)
    assert np.isnan(v[:n + m - 1]).all()
    t = 300
    want = (x[t - m + 1:t + 1].mean() - x[t - n - m + 1:t - n + 1].mean())
    np.testing.assert_allclose(v[t], want * FS / n, rtol=1e-9)
    assert np.isnan(vel.two_box(x[:n + m - 1], m, n, FS)).all()


def test_carry_center_and_hold_band():
    x = np.array([10.0, 11.0, 12.0, 12.0, 12.0, 16.0, 17.0])
    np.testing.assert_array_equal(vel.carry_center(x, [12], [3]),
                                  [10.0, 11.0, 13.5, 13.5, 13.5, 16.0, 17.0])
    band = vel.carry_band(x, [12], [3], 1)
    np.testing.assert_array_equal(band, [False, True, True, True, True, True,
                                         False])
    np.testing.assert_array_equal(vel.hold_band(x, band),
                                  [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 17.0])


def test_ls_line_skip_reads_the_slope_across_left_out_rows():
    slope = 12.0
    x = ramp(1000, slope)
    ok = np.ones(1000, bool)
    ok[500:520] = False
    v, held = vel.ls_line_skip(x, ok, 100, FS)
    assert np.isnan(v[:99]).all()
    np.testing.assert_allclose(v[99:], slope, rtol=1e-6)
    assert not held[99:].any()
    ok[300:400] = False
    v, held = vel.ls_line_skip(x, ok, 100, FS)
    assert held[399]
    np.testing.assert_allclose(v[99:], slope, rtol=1e-6)


def test_pole_diff_skip_stands_still_on_left_out_rows():
    x = ramp(3000, 10.0)
    x[2000:2010] = 0.0
    ok = np.ones(3000, bool)
    ok[2000:2010] = False
    v, held = vel.pole_diff_skip(x, ok, 20, FS)
    assert held[2000:2010].all() and held[2016:2026].all()
    assert not held[2010:2016].any() and not held[2026:].any()
    assert np.all(v[2000:2010] == v[1999])
    assert np.all(v[2016:2026] == v[2015])
    np.testing.assert_allclose(v[1500:], 10.0, rtol=1e-6)
