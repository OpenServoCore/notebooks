"""The ripple tach: a brushed motor's speed counted in its commutation ripple.

Each commutation event puts a dip in the winding current and, with the
bridge open, in the terminal EMF, slots x brushes of them per revolution, so
the ripple's line sits at that many times the shaft's turn rate. The line is
found as the strongest peak of the spectrum, then counted in time: a
band-pass around it run forward and back, so it shifts no crossing, and its
upward zero crossings interpolated to a fraction of a row. A window of whole
cycles between two crossings reads the mean speed over its rows.

Every function takes the row rate `fs`, a sweep's 20 kHz by default, and
works in rows from the start of the array it is given.
"""
import numpy as np
from scipy.signal import butter, sosfiltfilt

ROWS_HZ = 20_000        # a sweep's rows per second, the control tick
ZERO_PAD = 8            # the spectrum's zero padding, a line read to 1/8 bin
# Hz, where a ripple line is looked for: 25 to 1000 rev/s at six events
# per revolution
BAND_HZ = (150.0, 6000.0)
# the band-pass half-width as a share of the line: the lines at half and at
# one and a half times it stay outside
BAND_FRAC = 0.35
FILTER_ORDER = 2
NYQUIST_SHARE = 0.95    # the band's top under this share of Nyquist
WINDOW_MS = 10          # a window spans the whole cycles closest to this
MIN_CYCLES = 3          # and never fewer, which the slowest coasts need
# the band-pass runs over the window plus this many cycles, stretched by
# SPAN_MARGIN, so the window's crossings are never the filter's edge
SPAN_CYCLES = 3
SPAN_MARGIN = 1.3
PASSES = 6              # counts at most, the band centered on the last
CONVERGED = 1e-4        # a count this close to the last ends the passes


def spectrum(x, fs=ROWS_HZ):
    """(frequencies, amplitudes): Hann-windowed, zero-padded ZERO_PAD x."""
    x = np.asarray(x, float) - np.mean(x)
    n = len(x)
    f = np.fft.rfftfreq(ZERO_PAD * n, 1 / fs)
    amplitude = np.abs(np.fft.rfft(x * np.hanning(n), ZERO_PAD * n))
    return f, amplitude


def line_freq(x, lo=BAND_HZ[0], hi=BAND_HZ[1], fs=ROWS_HZ):
    """The frequency of the strongest line in [lo, hi] Hz."""
    f, amplitude = spectrum(x, fs)
    band = (f >= lo) & (f <= hi)
    return float(f[band][int(np.argmax(amplitude[band]))])


def line_height(x, f0, frac=0.1, fs=ROWS_HZ):
    """The tallest point of the spectrum within f0 x (1 +/- frac)."""
    f, amplitude = spectrum(x, fs)
    near = (f >= f0 * (1 - frac)) & (f <= f0 * (1 + frac))
    return float(amplitude[near].max())


def crossings(x, f0, frac=BAND_FRAC, fs=ROWS_HZ):
    """Upward crossings of the line near f0, in rows, the two ends dropped."""
    hi = min(f0 * (1 + frac), NYQUIST_SHARE * fs / 2)
    sos = butter(FILTER_ORDER, [f0 * (1 - frac), hi], btype="band", fs=fs,
                 output="sos")
    y = sosfiltfilt(sos, np.asarray(x, float) - np.mean(x))
    up = np.flatnonzero((y[:-1] < 0) & (y[1:] >= 0))
    return (up + (-y[up]) / (y[up + 1] - y[up]))[1:-1]


def count_cycles(x, f0, frac=BAND_FRAC, fs=ROWS_HZ):
    """(mean frequency, cycles) over every whole cycle of the line in x."""
    tc = crossings(x, f0, frac, fs)
    if len(tc) < 3:
        return np.nan, 0
    return (len(tc) - 1) / ((tc[-1] - tc[0]) / fs), len(tc) - 1


def read_window(x, start, f_seed, target_ms=WINDOW_MS, frac=BAND_FRAC,
                min_cycles=MIN_CYCLES, snap=True, fs=ROWS_HZ):
    """(f, first row, last row, cycles) of the window from `start`."""
    f = f_seed
    for _ in range(PASSES):
        n = max(min_cycles, int(round(target_ms / 1e3 * f)))
        stop = start + int((n + SPAN_CYCLES) * fs / f * SPAN_MARGIN)
        if stop > len(x):
            return np.nan, 0, 0, 0
        tc = crossings(x[start:stop], f, frac, fs)
        if len(tc) < n + 1:
            return np.nan, 0, 0, 0
        tc = tc[:n + 1]
        f_new = n / ((tc[-1] - tc[0]) / fs)
        done = abs(f_new / f - 1) < CONVERGED
        f = f_new
        if done:
            break
    if not snap:
        return f, start + tc[0], start + tc[-1], n
    return f, start + int(np.ceil(tc[0])), start + int(np.floor(tc[-1])), n


def walk(x, start, f_seed, target_ms, floor_rev_s, stop, per_rev,
         frac=BAND_FRAC, min_cycles=MIN_CYCLES, snap=True, fs=ROWS_HZ):
    """[(i0, i1, f, n)]: windows end to end down x, each seeded by the last."""
    out = []
    f = f_seed
    while True:
        f, i0, i1, n = read_window(x, start, f, target_ms, frac, min_cycles,
                                   snap, fs)
        if not np.isfinite(f) or f / per_rev < floor_rev_s or i1 > stop:
            return out
        out.append((i0, i1, f, n))
        start = int(np.ceil(i1))
