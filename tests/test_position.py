"""The position estimators on synthetic inputs: the lookup fitted with one
offset per segment, its knots, and the row filters, masks, direction
sources and lash."""
import numpy as np
import pandas as pd

from oscnb import position as ps

TAG_SD_DEG = 0.02       # deg, the noise put on the synthetic tag
PITCH = 32.0            # counts


def pot_curve(theta):
    """A monotone pot: about 19 counts/deg with a slow bend."""
    return 2000 + 19.0 * (theta + 95) + 30 * np.sin(theta / 20)


def lookup_frames(seed=1):
    rng = np.random.default_rng(seed)
    offsets = {f"s{i}": off for i, off in
               enumerate((0.3, -0.25, 0.12, -0.4, 0.05, 0.2))}
    moving = []
    for i, (sid, off) in enumerate(offsets.items()):
        theta = np.linspace(-160 + 3 * i, -30 - 2 * i, 600)
        moving.append(pd.DataFrame({
            "sid": sid, "pos": pot_curve(theta),
            "image_deg": theta + off + rng.normal(0, TAG_SD_DEG, len(theta))}))
    moving = pd.concat(moving, ignore_index=True)
    theta_r = np.linspace(-150, -40, 30)
    rest = pd.DataFrame({"pos": pot_curve(theta_r), "image_deg": theta_r})
    return moving, rest, offsets


def test_lookup_recovers_the_curve_and_the_offsets():
    moving, rest, offsets = lookup_frames()
    lk, off = ps.build(moving, rest, PITCH)
    assert lk.monotone
    for sid, want in offsets.items():
        assert abs(off[sid] - want) < 0.005, (sid, off[sid], want)
    theta = np.linspace(-155, -35, 500)
    curve = lk.angle(pot_curve(theta)) - theta
    assert np.sqrt(np.mean(curve ** 2)) < TAG_SD_DEG / 3
    resid = (moving["image_deg"] - moving["sid"].map(off)
             - lk.angle(moving["pos"]))
    assert abs(resid.std() / TAG_SD_DEG - 1) < 0.1


def test_lookup_round_trips_inside_and_follows_the_end_lines_outside():
    moving, rest, _ = lookup_frames()
    lk, _ = ps.build(moving, rest, PITCH)
    c = np.linspace(lk.c[0], lk.c[-1], 1000)
    np.testing.assert_allclose(lk.counts(lk.angle(c)), c, atol=1e-9)
    lo_slope = (lk.c[1] - lk.c[0]) / (lk.a[1] - lk.a[0])
    hi_slope = (lk.c[-1] - lk.c[-2]) / (lk.a[-1] - lk.a[-2])
    below, above = lk.c[0] - np.array([10, 200.0]), lk.c[-1] + np.array(
        [10, 200.0])
    np.testing.assert_allclose(lk.angle(below),
                               lk.a[0] + (below - lk.c[0]) / lo_slope)
    np.testing.assert_allclose(lk.angle(above),
                               lk.a[-1] + (above - lk.c[-1]) / hi_slope)
    given = ps.Lookup("counts", lk.c, lk.a, lo_slope=22.3, hi_slope=22.9)
    np.testing.assert_allclose(given.angle(lk.c[0] - 223.0), lk.a[0] - 10)
    np.testing.assert_allclose(given.counts(lk.a[-1] + 10), lk.c[-1] + 229)
    np.testing.assert_allclose(given.angle(given.counts(lk.a[-1] + 10)),
                               lk.a[-1] + 10)


def test_uniform_knots_land_on_multiples_of_the_pitch_and_cover_the_span():
    for lo, hi, pitch in ((577.3, 3101.9, 32.0), (512.0, 1024.0, 16.0),
                          (-163.2, -28.6, 1.5)):
        k = ps.uniform_knots(lo, hi, pitch)
        np.testing.assert_allclose(k / pitch, np.round(k / pitch),
                                   atol=1e-9)
        np.testing.assert_allclose(np.diff(k), pitch)
        assert k[0] <= lo < k[0] + pitch
        assert k[-1] - pitch < hi <= k[-1]


def test_boxcar_delays_a_ramp_by_half_its_length_less_one():
    x = 2.5 * np.arange(300)
    for n in (1, 4, 16, 41):
        y = ps.boxcar(x, n)
        np.testing.assert_allclose(y[n:], x[n:] - 2.5 * (n - 1) / 2)


def test_boxcar_nulls_a_period_16_pattern_at_16_and_32_not_20():
    i = np.arange(400)
    x = np.sin(2 * np.pi * i / 16) + 0.5 * np.cos(2 * np.pi * 3 * i / 16)
    for n in (16, 32):
        assert np.abs(ps.boxcar(x, n)[n:]).max() < 1e-9
    assert np.abs(ps.boxcar(x, 20)[20:]).max() > 0.05


def test_one_pole_delays_a_ramp_by_delay_ticks_and_passes_the_first():
    x = 123.0 + 0.7 * np.arange(2000)
    for d in (0, 3, 20):
        y = ps.one_pole(x, d)
        assert y[0] == x[0]
        np.testing.assert_allclose(y[-200:], x[-200:] - 0.7 * d, atol=1e-6)


def test_median3_removes_a_one_row_spike_and_delays_a_ramp_one_tick():
    x = np.full(50, 800.0)
    x[20] = 900.0
    np.testing.assert_allclose(ps.median3(x), 800.0)
    r = 3.0 * np.arange(50)
    np.testing.assert_allclose(ps.median3(r)[2:], r[1:-1])


def test_carry_hold_holds_the_last_sample_outside_the_band():
    carries = (2047, 3071)
    x = np.arange(2000.0, 3100.0)
    y, band = ps.carry_hold(x, carries, half=12)
    want_band = (np.abs(x - 2047) <= 12) | (np.abs(x - 3071) <= 12)
    np.testing.assert_array_equal(band, want_band)
    np.testing.assert_array_equal(y[~band], x[~band])
    assert (y[(x >= 2035) & (x <= 2059)] == 2034).all()
    assert (y[(x >= 3059) & (x <= 3083)] == 3058).all()


def test_dir_from_pot_flips_only_after_the_hysteresis():
    hyst = 5
    up = np.arange(0.0, 101.0)
    wobble = np.array([98.0, 96.0, 99.0, 100.0])  # back 4, under hyst
    down = np.arange(99.0, 80.0, -1)
    x = np.r_[up, wobble, down]
    d = ps.dir_from_pot(x, hyst)
    first_up = int(np.flatnonzero(d == 1)[0])
    assert x[first_up] == 5 and (d[:first_up] == 0).all()
    first_down = int(np.flatnonzero(d == -1)[0])
    assert x[first_down] == 95
    assert (d[first_up:first_down] == 1).all()
    assert (d[first_down:] == -1).all()
    assert (ps.dir_from_pot(up[:3], hyst, d0=-1.0) == -1).all()


def test_dir_from_duty_holds_the_last_sign_through_zero():
    duty = [0, 0, 5, 3, 0, 0, -2, 0, 4]
    np.testing.assert_array_equal(ps.dir_from_duty(duty),
                                  [0, 0, 1, 1, 1, 1, -1, -1, 1])


def test_play_lags_by_half_and_holds_through_a_reversal():
    half = 3.0
    up = np.arange(0.0, 51.0)
    down = np.arange(49.0, 20.0, -1)
    x = np.r_[up, down]
    y = ps.play(x, half)
    np.testing.assert_allclose(y[10:51], up[10:] - half)    # dir +1
    peak_out = 50 - half
    held = (x[51:] >= 50 - 2 * half)
    np.testing.assert_allclose(y[51:][held], peak_out)
    np.testing.assert_allclose(y[51:][~held], x[51:][~held] + half)  # dir -1
    z = ps.play(np.array([0.0, 5.0, np.nan, 9.0]), half)
    assert np.isnan(z[2]) and z[1] == 2.0 and z[3] == 6.0
