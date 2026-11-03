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
