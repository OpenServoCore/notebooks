"""The ripple tach on a synthetic coast: the line found, its crossings, the
window of whole cycles and the windows walked down the coast."""
import coast_synth as synth
import numpy as np
import pytest

from oscnb import ripple

FS = synth.FS
STEADY_HZ = 2437.3          # off any whole number of rows per cycle
PHASE0 = 0.3


def steady(rows=6000):
    t = np.arange(rows) / FS
    return np.sin(2 * np.pi * STEADY_HZ * t + PHASE0)


def steady_crossings(rows=6000):
    m = np.arange(1, int(rows / FS * STEADY_HZ) + 2)
    return (m - PHASE0 / (2 * np.pi)) / STEADY_HZ * FS


def coast_crossings():
    # the chirp's upward crossings: phase(t) = 2 pi m solved for t, in rows
    f0 = synth.REV_S * synth.PER_REV
    m = np.arange(1, 400)
    c = (m - PHASE0 / (2 * np.pi)) / f0
    c = c[c < synth.STOP_S / 2]
    t = synth.STOP_S * (1 - np.sqrt(1 - 2 * c / synth.STOP_S))
    return t * FS


def nearest(rows, truth):
    j = np.clip(np.searchsorted(truth, rows), 1, len(truth) - 1)
    lo, hi = truth[j - 1], truth[j]
    return np.where(np.abs(rows - lo) < np.abs(rows - hi), lo, hi)


def test_line_freq_finds_the_line_within_a_bin():
    x = steady(200)
    f = ripple.line_freq(x)
    assert abs(f - STEADY_HZ) <= FS / (ripple.ZERO_PAD * 200) / 2
    assert ripple.line_freq(x, 150, 1000) < 1000
    freqs, amplitude = ripple.spectrum(x)
    assert len(freqs) == len(amplitude) == ripple.ZERO_PAD * 200 // 2 + 1
    assert ripple.line_height(x, f) == pytest.approx(amplitude.max())


def test_crossings_sit_on_the_true_crossings():
    tc = ripple.crossings(steady(), STEADY_HZ)
    truth = steady_crossings()
    interior = tc[5:-5]
    assert np.abs(interior - nearest(interior, truth)).max() < 0.02
    assert len(tc) == np.sum((truth > 0) & (truth < 6000)) - 2


def test_count_cycles_on_a_steady_line():
    f, n = ripple.count_cycles(steady(), STEADY_HZ)
    assert n == len(ripple.crossings(steady(), STEADY_HZ)) - 1
    assert f == pytest.approx(STEADY_HZ, rel=1e-4)
    f_short, n_short = ripple.count_cycles(steady(30), STEADY_HZ)
    assert np.isnan(f_short) and n_short == 0


def test_read_window_holds_the_whole_cycles_closest_to_the_target():
    e = synth.emf(dropouts=())
    truth = coast_crossings()
    for start in (20, 517, 2033):
        seed = ripple.line_freq(e[start:start + 200])
        f, i0, i1, n = ripple.read_window(e, start, seed, snap=False)
        assert n == max(ripple.MIN_CYCLES, round(ripple.WINDOW_MS / 1e3 * f))
        assert np.abs(np.array([i0, i1]) - nearest(np.array([i0, i1]),
                                                   truth)).max() < 0.1
        assert np.sum((truth > i0 + 0.5) & (truth < i1 + 0.5)) == n
        snapped = ripple.read_window(e, start, seed)
        assert snapped == (f, int(np.ceil(i0)), int(np.floor(i1)), n)


@pytest.mark.xfail(strict=True, reason="reads up to 1.1e-3 off the truth "
                   "here, median 4e-4; the lift keeps bemf-coast's method "
                   "bit for bit")
def test_read_window_recovers_the_frequency_within_1e_4():
    e = synth.emf(dropouts=())
    for start in range(20, 3000, 137):
        seed = ripple.line_freq(e[start:start + 200])
        f, i0, i1, n = ripple.read_window(e, start, seed, snap=False)
        t_mid = (i0 + i1) / 2 / FS
        assert f == pytest.approx(synth.ripple_hz(t_mid), rel=1e-4)


def test_read_window_gives_up_past_the_end():
    e = synth.emf(dropouts=())
    assert ripple.read_window(e, len(e) - 50, 3000.0)[1:] == (0, 0, 0)
    assert np.isnan(ripple.read_window(e, len(e) - 50, 3000.0)[0])


def test_walk_steps_down_the_coast_to_the_floor():
    e = synth.emf(dropouts=())
    floor_rev_s = 60.0
    seed = ripple.line_freq(e[20:220])
    windows = ripple.walk(e, 20, seed, 5, floor_rev_s, len(e), synth.PER_REV)
    assert len(windows) > 20
    starts = [w[0] for w in windows]
    ends = [w[1] for w in windows]
    f = np.array([w[2] for w in windows])
    gaps = np.array(starts[1:]) - np.ceil(ends[:-1])
    assert np.all(gaps >= 0)
    # a window opens on its stretch's second crossing, two cycles at most
    # past the last window's end
    assert np.all(gaps <= 2 * FS / f[1:] + 2)
    assert np.all(np.diff(f) < 0)
    assert np.all(f / synth.PER_REV >= floor_rev_s)
    t_mid = np.array([(w[0] + w[1]) / 2 for w in windows]) / FS
    assert f == pytest.approx(synth.ripple_hz(t_mid), rel=1e-2)
    i1, f_last = windows[-1][1], windows[-1][2]
    f_next = ripple.read_window(e, int(np.ceil(i1)), f_last, 5)[0]
    assert not f_next / synth.PER_REV >= floor_rev_s


def test_walk_stops_before_a_window_passes_the_stop():
    e = synth.emf(dropouts=())
    seed = ripple.line_freq(e[20:220])
    windows = ripple.walk(e, 20, seed, 5, 1.0, 1000, synth.PER_REV)
    assert windows
    assert all(w[1] <= 1000 for w in windows)
    free = ripple.walk(e, 20, seed, 5, 1.0, len(e), synth.PER_REV)
    assert free[:len(windows)] == windows
    assert free[len(windows)][1] > 1000


# the tracked count through a duty step: the line jumps a third at STEP_ROW
STEP_ROW = 4000
F_BASE, F_STEP = 1500.0, 2000.0


def stepped(rows=8000):
    f = np.where(np.arange(rows) < STEP_ROW, F_BASE, F_STEP)
    phase = 2 * np.pi * np.cumsum(f) / FS
    return np.sin(phase), phase


def test_tracked_crossings_follow_a_step_in_the_line():
    x, phase = stepped()
    tc = ripple.tracked_crossings(x, 200, len(x) - 100, F_BASE)
    assert np.all(np.diff(tc) > 0) and tc[0] > 199 and tc[-1] < len(x) - 100
    t_mid, span, rev_s = ripple.cycle_speeds(tc, 6)
    assert span == pytest.approx(np.diff(tc) / FS)
    before = t_mid < (STEP_ROW - 50) / FS
    after = t_mid > (STEP_ROW + 50) / FS
    assert rev_s[before] == pytest.approx(F_BASE / 6, rel=1e-3)
    assert rev_s[after] == pytest.approx(F_STEP / 6, rel=1e-3)
    cycles = (phase[int(tc[-1])] - phase[int(tc[0])]) / (2 * np.pi)
    assert abs(len(tc) - 1 - cycles) < 1
    held = tc[tc < STEP_ROW - 50]
    assert ripple.speed_of(held, 6) == pytest.approx(F_BASE / 6, rel=1e-4)
    assert np.isnan(ripple.speed_of(held[:2], 6))


def test_ripple_line_takes_the_mean_out_first():
    x, _ = stepped()
    near = FS / (ripple.ZERO_PAD * 2000) / 2
    assert abs(ripple.ripple_line(x[:2000] + 40.0) - F_BASE) <= near


# the order count on a synthetic run-up from rest: the speed rises as
# 1 - exp(-t / TAU), the current falls with it over its drag, and a
# six-per-revolution ripple rides on the current
PER_REV = 6
W_TOP = 2 * np.pi * 480.0      # rad/s
TAU = 0.012                    # s
I_DRAG, I_STALL, I_RIPPLE = 0.1, 0.9, 0.05
R_OHM, L_H, KE = 4.2, 0.8e-3, 1.6e-3


def run_up(rows=1100):
    t = np.arange(rows) / FS
    w = W_TOP * (1 - np.exp(-t / TAU))
    theta = W_TOP * (t - TAU * (1 - np.exp(-t / TAU)))
    current = (I_DRAG + I_STALL * np.exp(-t / TAU)
               + I_RIPPLE * np.cos(PER_REV * theta))
    return t, w, theta, current


def test_count_speed_reads_a_run_up_from_a_rough_seed():
    t, w, theta, current = run_up()
    count = ripple.count_speed(current, 1.1 * w, I_DRAG, PER_REV)
    assert count["added"] == count["removed"] == 0
    tc = count["tc"]
    # the median's 31 intervals sit whole inside the crossings
    inside = (t > tc[16]) & (t < tc[-16])
    assert count["speed"][inside] == pytest.approx(w[inside], rel=2e-3)
    early = t < tc[ripple.INTEGRAL_UNTIL]
    assert np.all(np.diff(count["speed"][early]) > 0)
    up = (3 * np.pi / 2 + 2 * np.pi * np.arange(400)) / PER_REV
    truth = np.interp(up, theta, t)
    nearest = truth[np.abs(truth[None, :] - tc[:, None]).argmin(axis=1)]
    assert np.abs(tc - nearest).max() < 0.5 / FS
    late = (truth > tc[0] + 0.002) & (truth < tc[-1] - 0.002)
    assert np.all(np.abs(tc[:, None] - truth[late][None, :]).min(axis=0)
                  < 0.5 / FS)


def test_rough_speed_is_the_emf_the_volts_leave_over_ke():
    t, w, _, current = run_up()
    volts = (R_OHM * current + L_H * np.gradient(current) * FS + KE * w)
    rough, scale = ripple.rough_speed(current, volts, 2 * w, R_OHM, L_H, KE,
                                      (0.015, 0.045))
    assert scale == pytest.approx(2.0, rel=2e-3)
    inner = (t > 0.005) & (t < 0.05)
    assert rough[inner] == pytest.approx(scale * w[inner], rel=5e-3)
    emf_only, same = ripple.rough_speed(current, volts, 2 * w, R_OHM, L_H,
                                        KE, (0.015, 0.045), seed="emf")
    assert same == scale
    assert emf_only[inner] == pytest.approx(w[inner], rel=5e-3)


def test_angle_crossings_sit_on_the_ripple_s_angle():
    per = 2 * np.pi / PER_REV
    theta = np.linspace(0, 40 * per, 4000) ** 1.2 / (40 * per) ** 0.2
    tc = ripple.angle_crossings(np.sin(PER_REV * theta), theta, PER_REV)
    assert np.round(tc / per).tolist() == list(range(2, 39))
    inner = tc[2:-2] / per
    assert inner == pytest.approx(np.round(inner), abs=1e-3)


def test_local_speed_is_a_cycle_over_the_median_interval():
    tc = np.arange(50) * 1e-3
    tc[20] += 3e-4
    w = ripple.local_speed(tc, np.array([0.0, 0.01, 0.03]), PER_REV)
    assert np.isnan(w[0])
    assert w[1:] == pytest.approx(2 * np.pi / PER_REV / 1e-3)


def test_repair_restores_a_missed_and_an_extra_crossing():
    truth = np.arange(100) * 1e-3
    tc = np.insert(np.delete(truth, 50), 69, 70.5e-3)
    fixed, added, removed = ripple.repair(tc)
    assert (added, removed) == (1, 1)
    assert fixed == pytest.approx(truth, abs=1e-12)
    assert ripple.repair(truth)[1:] == (0, 0)


def test_local_line_slope_reads_a_line_s_slope():
    slope = ripple.local_line_slope(3.0 * np.arange(50) + 7.0, 4)
    assert np.isnan(slope[:4]).all() and np.isnan(slope[-4:]).all()
    assert slope[4:-4] == pytest.approx(3.0)


def test_rev_and_fit_tables_on_a_constant_acceleration():
    w0, alpha = 2 * np.pi * 200.0, 2 * np.pi * 5000.0
    per = 2 * np.pi / PER_REV
    angle = np.arange(400) * per
    tc = (-w0 + np.sqrt(w0 ** 2 + 2 * alpha * angle)) / alpha
    rows = int(tc[-1] * FS) + 2
    current = np.full(rows, 0.5)
    dropped = np.zeros(rows, bool)
    current[300], dropped[300] = 99.0, True
    means = {"i": current, "u": np.full(rows, 2.0)}
    rev = ripple.rev_table(tc, PER_REV, current, means, dropped)
    assert len(rev) == len(tc) - PER_REV
    assert rev["w"].to_numpy() == pytest.approx(
        w0 + alpha * rev["t"].to_numpy(), rel=1e-3)
    assert (rev["i"] == 0.5).all() and (rev["u"] == 2.0).all()
    assert np.abs(rev["didt"]).max() == 0
    fit = ripple.fit_table(tc, PER_REV, means, dropped)
    assert fit["alpha"].to_numpy() == pytest.approx(alpha, rel=1e-6)
    assert fit["w"].to_numpy() == pytest.approx(
        w0 + alpha * fit["t"].to_numpy(), rel=1e-6)
    assert (fit["n"] >= 2 * PER_REV + 1).all()
    assert fit["res_deg"].max() < 1e-6
    assert fit["open_rows"].isin([0, 1]).all() and fit["open_rows"].any()
    step = np.diff(fit["t"].to_numpy())
    assert step == pytest.approx(ripple.FIT_STEP_S)


def test_flag_windows_flags_a_window_a_crossing_off():
    t = np.arange(60) * 0.002
    w = 300.0 + 1000.0 * t
    w[30] *= 7 / 6
    ok = ripple.flag_windows(t, w)
    assert np.flatnonzero(~ok).tolist() == [30]


def test_sharp_drops_are_the_first_falls_past_the_rise():
    current = np.ones(200)
    for row in (20, 40, 41, 60, 100, 150):
        current[row:] -= 0.2
    drops = ripple.sharp_drops(current, 60 / FS)
    assert drops.tolist() == [40, 60, 100]


# the phase count: a ripple on a sweeping angle, 200 to 600 Hz at the rows
SWEEP_ROWS = 4000
RIPPLE_COUNTS, NOISE_COUNTS = 8.0, 2.0
# cycles of the count's own centers left out at each end: passes 2 and 3
# pull the end centers off a 20 % seed (see the xfail below)
END_CYCLES = 2.0


def sweep_phase(rows=SWEEP_ROWS):
    r = np.arange(rows)
    return 0.01 * r + 2.5e-6 * r ** 2 + 0.37


def sweep_current(mean, seed=7, ripple_counts=RIPPLE_COUNTS):
    rng = np.random.default_rng(seed)
    phi = sweep_phase(len(mean))
    return (ripple_counts * np.cos(2 * np.pi * phi) + mean
            + rng.normal(0, NOISE_COUNTS, len(mean)))


def inner_change(count, phi):
    first = phi[count.c] - phi[count.c[0]] >= END_CYCLES
    last = phi[count.c[-1]] - phi[count.c] >= END_CYCLES
    r0, r1 = count.c[first][0], count.c[last][-1]
    got = count.phi([r0, r1])
    return (got[1] - got[0]) - (phi[r1] - phi[r0]), r1 - r0


@pytest.mark.parametrize("rate", [0.8, 1.2])
def test_phase_count_reads_a_sweep_from_a_seed_20_pct_off(rate):
    phi = sweep_phase()
    ramp = 50 + 0.01 * np.arange(SWEEP_ROWS)
    count = ripple.PhaseCount(sweep_current(ramp), rate * phi, 0, SWEEP_ROWS)
    assert count.ok
    err, span = inner_change(count, phi)
    assert span > 0.8 * SWEEP_ROWS
    assert abs(err) < 0.02


@pytest.mark.parametrize("rate", [0.8, 1.2])
def test_phase_count_reads_through_the_current_s_change_of_sign(rate):
    phi = sweep_phase()
    through_zero = -30 + 60 * np.arange(SWEEP_ROWS) / SWEEP_ROWS
    count = ripple.PhaseCount(sweep_current(through_zero), rate * phi, 0,
                              SWEEP_ROWS)
    assert count.ok
    err, _ = inner_change(count, phi)
    assert abs(err) < 0.02


@pytest.mark.xfail(strict=True, reason="the later passes pull the end "
                   "centers off a seed whose rate is 20 % off: 0.08 cycle "
                   "between the first and last centers, 0.016 after one pass")
def test_phase_count_end_centers_read_a_seed_20_pct_off():
    phi = sweep_phase()
    ramp = 50 + 0.01 * np.arange(SWEEP_ROWS)
    count = ripple.PhaseCount(sweep_current(ramp), 0.8 * phi, 0, SWEEP_ROWS)
    got = count.phi([count.c[0], count.c[-1]])
    err = (got[1] - got[0]) - (phi[count.c[-1]] - phi[count.c[0]])
    assert abs(err) < 0.02


def test_phase_is_nan_outside_the_centers():
    phi = sweep_phase()
    count = ripple.PhaseCount(sweep_current(np.zeros(SWEEP_ROWS)), phi, 0,
                              SWEEP_ROWS)
    c0, c1 = count.c[0], count.c[-1]
    out = count.phi([0, c0 - 0.5, c0, c1, c1 + 0.5, SWEEP_ROWS - 1])
    assert np.isnan(out[[0, 1, 4, 5]]).all()
    assert np.isfinite(out[[2, 3]]).all()
    # the first center's window of one cycle back lies inside the stretch
    assert phi[c0] - phi[0] >= ripple.PHASE_HALF_CYCLES


def test_phase_count_crossings_are_one_row_per_whole_cycle():
    phi = sweep_phase()
    count = ripple.PhaseCount(sweep_current(np.zeros(SWEEP_ROWS)), phi, 0,
                              SWEEP_ROWS)
    rows = count.crossings()
    truth = np.interp(rows, np.arange(SWEEP_ROWS), phi)
    k = np.round(truth)
    assert np.all(np.diff(k) == 1)
    assert len(rows) == int(np.floor(count.phi_c[-1])
                            - np.ceil(count.phi_c[0])) + 1
    assert np.abs(truth - k).max() < 0.05


def test_a_seed_under_two_cycles_gives_no_count():
    seed = np.linspace(0, 1.5, SWEEP_ROWS)
    count = ripple.PhaseCount(sweep_current(np.zeros(SWEEP_ROWS)), seed, 0,
                              SWEEP_ROWS)
    assert not count.ok


def test_the_trust_numbers_flag_pure_noise():
    phi = sweep_phase()
    zero = np.zeros(SWEEP_ROWS)
    noise = ripple.PhaseCount(sweep_current(zero, ripple_counts=0), phi, 0,
                              SWEEP_ROWS)
    signal = ripple.PhaseCount(sweep_current(zero), phi, 0, SWEEP_ROWS)
    assert noise.ok and signal.ok
    assert noise.min_snr(0, SWEEP_ROWS) < 0.5 < 2 < signal.min_snr(
        0, SWEEP_ROWS)
    assert signal.conv < 0.05 < noise.conv
    assert signal.max_step(0, SWEEP_ROWS) < 0.05


def test_angle_seed_keeps_a_quadratic_angle():
    r = np.arange(3000.0)
    theta = 0.3 + 0.004 * r + 1e-6 * r ** 2
    seed = ripple.angle_seed(theta, 0.2)
    assert seed == pytest.approx(theta / 0.2, rel=1e-9)


def test_order_crossing_rows_sit_on_the_whole_cycles():
    phi = sweep_phase()
    x = np.sin(2 * np.pi * phi)
    a, b = 500, 3500
    rows = ripple.order_crossing_rows(x, phi[a:b + 1], 0, b + 1 - a, PER_REV,
                                      offset=a)
    truth = np.interp(rows, np.arange(SWEEP_ROWS), phi)
    assert np.all((rows > a) & (rows < b))
    assert np.all(np.diff(np.round(truth)) == 1)
    assert np.abs(truth - np.round(truth)).max() < 0.01
