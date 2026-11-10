"""The ripple tach: a brushed motor's speed counted in its commutation ripple.

Each commutation event puts a dip in the winding current and, with the
bridge open, in the terminal EMF, slots x brushes of them per revolution, so
the ripple's line sits at that many times the shaft's turn rate. The line is
found as the strongest peak of the spectrum, then counted in time: a
band-pass around it run forward and back, so it shifts no crossing, and its
upward zero crossings interpolated to a fraction of a row. A window of whole
cycles between two crossings reads the mean speed over its rows.

A line that moves faster than a window holds is counted two more ways. The
tracked count re-centers its band every few cycles on the line the last
chunk read, which follows a duty step. The order count, for a run-up from
rest whose line sweeps up from zero, resamples the current onto a rough
angle, where the ripple sits at one fixed frequency whatever the speed,
and counts its crossings there, each pass on the angle the last one
counted.

The phase count reads the ripple's own phase, cycle by cycle, inside one
stretch of constant duty. Around each center row the current is fitted with
a constant, a slope and one sinusoid at one cycle per commutation period of
a seed angle, Hann-tapered over +/- one cycle of it. On angle the line sits
at order 1 at any speed, and a fit this local has no filter edge to ring on
the current's step at a duty change: the stretch's first rows cost one
cycle, not a filter's settling. The sinusoid's phase against the seed,
unwrapped along the centers and added to the seed, is the phase in cycles.
Each later pass re-reads the current on the phase the last one counted,
smoothed, so the seed only has to keep the first pass from slipping a cycle.

Every function takes the row rate `fs`, a sweep's 20 kHz by default, and
works in rows from the start of the array it is given, except the order
count's, which work in seconds.
"""
import numpy as np
import pandas as pd
from scipy.signal import butter, savgol_filter, sosfiltfilt

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
TRACK_CYCLES = 4        # cycles per chunk of the tracked count
# the order count: the ripple resampled to GRID_SAMPLES per cycle and
# band-passed at +/- ORDER_BAND of its line
GRID_SAMPLES = 16
ORDER_BAND = 0.25
ORDER_PASSES = 5
RISE_ROWS = 20          # 1 ms, the current's L/R rise, no crossing read in it
INTEGRAL_UNTIL = 9      # the crossing before which speed is the integral
SEED_BOX_ROWS = 41      # about 2 ms, the EMF seed's boxcar
FIT_HALF_S = 0.005      # s, the +/- half width of an acceleration fit
FIT_STEP_S = 0.0005     # s, a fit center every 0.5 ms
PHASE_HALF_CYCLES = 1.0     # cycles of the seed either side of a center
PHASE_STEP_ROWS = 4         # rows between centers (0.2 ms)
PHASE_MIN_ROWS = 8          # rows, the fewest a fit window holds
PHASE_MAX_ROWS = 6000       # rows, a fit window longer than this is a stall
PHASE_PASSES = 3
PHASE_SMOOTH_ROWS = 100     # rows (5 ms), a pass's phase smoothed for the next
ANGLE_SEED_ROWS = 401       # rows (20 ms), an angle seed's quadratic


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


def ripple_line(x, fs=ROWS_HZ):
    """The strongest line in BAND_HZ of x, its mean taken out first."""
    return line_freq(np.asarray(x, float) - np.mean(x), *BAND_HZ, fs=fs)


def tracked_crossings(x, start, stop, f_seed, cycles=TRACK_CYCLES,
                      frac=BAND_FRAC, fs=ROWS_HZ):
    """Upward crossings from `start` to `stop`, in rows, the band
    re-centered every `cycles` cycles on the last chunk's line."""
    x = np.asarray(x, float)
    out = []
    f = f_seed
    last = start - 0.5
    while last < stop:
        lo = max(0, int(last - 3 * fs / f))     # 3 cycles of lead-in
        hi = min(len(x), int(last + (cycles + 4) * fs / f))
        if hi - lo < 6 * fs / f:
            break
        tc = crossings(x[lo:hi], f, frac, fs) + lo
        tc = tc[(tc > last + 0.3 * fs / f) & (tc <= stop)]
        if len(tc) == 0:
            break
        take = tc[:cycles]
        out.extend(take.tolist())
        if len(out) >= 2:
            recent = out[-(cycles + 1):]
            f = (len(recent) - 1) / ((recent[-1] - recent[0]) / fs)
        last = out[-1]
        if hi >= len(x) and len(tc) <= len(take):
            break
    return np.array(out)


def cycle_speeds(tc, per_rev, fs=ROWS_HZ):
    """(mid time s, span s, rev/s) of each cycle between two crossings."""
    tc = np.asarray(tc, float)
    span = np.diff(tc) / fs
    t_mid = (tc[:-1] + tc[1:]) / 2 / fs
    return t_mid, span, 1 / span / per_rev


def speed_of(tc, per_rev, fs=ROWS_HZ):
    """rev/s over the whole cycles of a run of crossings, nan under three."""
    if len(tc) < 3:
        return np.nan
    return (len(tc) - 1) / ((tc[-1] - tc[0]) / fs) / per_rev


def angle_crossings(x, theta, per_rev, frac=ORDER_BAND, ns=GRID_SAMPLES):
    """Upward ripple crossings in rad of the angle `theta`, x resampled to
    `ns` samples per ripple cycle, the two ends dropped."""
    per = 2 * np.pi / per_rev
    keep = np.r_[True, np.diff(theta) > 0]
    grid = np.arange(theta[0], theta[-1], per / ns)
    xs = np.interp(grid, theta[keep], x[keep])
    sos = butter(FILTER_ORDER, [1 - frac, 1 + frac], btype="band", fs=ns,
                 output="sos")
    y = sosfiltfilt(sos, xs - xs.mean())
    s = np.flatnonzero((y[:-1] < 0) & (y[1:] >= 0))
    return (grid[s] + (-y[s]) / (y[s + 1] - y[s]) * (per / ns))[1:-1]


def local_speed(tc, t, per_rev, n=31):
    """rad/s at the times t from the crossing times tc, one ripple cycle
    over the median of the n intervals around."""
    per = 2 * np.pi / per_rev
    intervals = np.diff(tc)
    h = n // 2
    med = np.array([np.median(intervals[max(0, j - h):j + h + 1])
                    for j in range(len(intervals))])
    return np.interp(t, (tc[1:] + tc[:-1]) / 2, per / med, left=np.nan)


def repair(tc, short=0.7, long=1.5, merge_tol=0.35, n=15, fixes=500):
    """(crossings, added, removed): one missed or extra crossing at a time,
    each interval held against the rolling median of n."""
    tc = np.asarray(tc, float).copy()
    added = removed = 0
    for _ in range(fixes):
        iv = np.diff(tc)
        med = pd.Series(iv).rolling(n, center=True, min_periods=3).median()
        ratio = iv / med.to_numpy()
        j = int(np.argmin(ratio))
        if ratio[j] < short:
            cands = []
            if j > 0:
                cands.append((abs((iv[j - 1] + iv[j]) / med[j] - 1), j))
            if j + 1 < len(iv):
                cands.append((abs((iv[j] + iv[j + 1]) / med[j] - 1),
                              j + 1))
            err, drop = min(cands)
            if err < merge_tol:
                tc = np.delete(tc, drop)
                removed += 1
                continue
        j = int(np.argmax(ratio))
        if ratio[j] > long:
            m = int(round(ratio[j]))
            tc = np.insert(tc, j + 1, tc[j] + iv[j] * np.arange(1, m) / m)
            added += m - 1
            continue
        break
    return tc, added, removed


def local_line_slope(y, half):
    """The slope of y on the row index by a line over +/- half rows."""
    n = len(y)
    out = np.full(n, np.nan)
    x = np.arange(-half, half + 1, dtype=float)
    sxx = (x ** 2).sum()
    for j in range(half, n - half):
        seg = y[j - half:j + half + 1]
        out[j] = (x * (seg - seg.mean())).sum() / sxx
    return out


def rough_speed(current, volts, w_pot, r_ohm, l_h, ke, scale_s, seed="pot",
                fs=ROWS_HZ):
    """(rad/s, scale): the EMF the volts leave through R and L over ke,
    smoothed and clipped at zero, times the scale that puts its mean over
    the times `scale_s` on w_pot's for seed "pot", unscaled for "emf"."""
    t = np.arange(len(current)) / fs
    emf = volts - r_ohm * current - l_h * np.gradient(current) * fs
    emf = np.convolve(emf, np.ones(SEED_BOX_ROWS) / SEED_BOX_ROWS,
                      mode="same")
    w_emf = np.maximum(emf, 0) / ke
    win = (t >= scale_s[0]) & (t <= scale_s[1])
    scale = np.nanmean(w_pot[win]) / w_emf[win].mean()
    if seed == "pot":
        return w_emf * scale, scale
    return w_emf, scale


def count_speed(current, rough, i_drag0, per_rev, passes=ORDER_PASSES,
                rise_rows=RISE_ROWS, integral_until=INTEGRAL_UNTIL,
                fs=ROWS_HZ):
    """{"tc", "added", "removed", "speed"}: the order count of a run-up
    from rest seeded by `rough`, speed in rad/s per row, before the crossing
    `integral_until` the current's integral over i_drag0."""
    per = 2 * np.pi / per_rev
    t = np.arange(len(current)) / fs
    speed = rough
    for _ in range(passes):
        theta = np.cumsum(speed) / fs
        c = angle_crossings(current[rise_rows:], theta[rise_rows:], per_rev)
        # the first two crossings sit on the band-pass's edge
        c = c[c > theta[rise_rows] + 2 * per]
        tc, added, removed = repair(np.interp(c, theta, t))
        speed = local_speed(tc, t, per_rev)
        t_int = tc[integral_until]
        w_int = np.cumsum(current - i_drag0) / fs
        w_int *= np.interp(t_int, t, speed) / np.interp(t_int, t, w_int)
        speed = np.where(t < t_int, w_int, speed)
    return {"tc": tc, "added": added, "removed": removed, "speed": speed}


def rev_table(tc, per_rev, current, means, dropped, fs=ROWS_HZ):
    """One row per turn window of per_rev crossing times: its center, rad/s,
    each of `means` over its rows less `dropped`, the current's slope."""
    rows = []
    for j in range(len(tc) - per_rev):
        a = int(np.ceil(tc[j] * fs))
        b = int(np.floor(tc[j + per_rev] * fs))
        keep = ~dropped[a:b]
        row = {"t": (tc[j] + tc[j + per_rev]) / 2,
               "w": 2 * np.pi / (tc[j + per_rev] - tc[j])}
        for k, x in means.items():
            row[k] = x[a:b][keep].mean()
        row["didt"] = (current[b] - current[a]) / ((b - a) / fs)
        rows.append(row)
    return pd.DataFrame(rows)


def fit_table(tc, per_rev, means, dropped, half=FIT_HALF_S, step=FIT_STEP_S,
              fs=ROWS_HZ):
    """The acceleration fits, a quadratic of the counted angle on time over
    +/- half every step, two turns at least: rad/s, rad/s^2, the angle
    residual's sd, the dropped rows and each of `means` over the fit."""
    angle = np.arange(len(tc)) * (2 * np.pi / per_rev)
    cum = {k: np.r_[0, np.cumsum(x)] for k, x in means.items()}
    cum_open = np.r_[0, np.cumsum(dropped)]
    rows = []
    for tm in np.arange(tc[0] + half, tc[-1] - half, step):
        m = (tc >= tm - half) & (tc <= tm + half)
        if m.sum() < 2 * per_rev + 1:
            continue
        p = np.polyfit(tc[m] - tm, angle[m], 2)
        res = angle[m] - np.polyval(p, tc[m] - tm)
        a = int(round((tm - half) * fs))
        b = int(round((tm + half) * fs))
        row = {"t": tm, "w": p[1], "alpha": 2 * p[0], "n": int(m.sum()),
               "res_deg": np.degrees(res.std()),
               "open_rows": int(cum_open[b] - cum_open[a])}
        for k in cum:
            row[k] = (cum[k][b] - cum[k][a]) / (b - a)
        rows.append(row)
    return pd.DataFrame(rows)


def flag_windows(t, w, n=15, tol=0.05):
    """True where a turn window sits within tol of the quadratic in time
    through the 2n windows around it, refitted without its outliers."""
    ok = np.ones(len(w), bool)
    for j in range(len(w)):
        near = slice(max(0, j - n), j + n + 1)
        tt, ww = t[near] - t[j], w[near]
        keep = np.ones(len(ww), bool)
        p = None
        for _ in range(3):
            if keep.sum() < 5:
                break
            p = np.polyfit(tt[keep], ww[keep], 2)
            keep = np.abs(ww / np.polyval(p, tt) - 1) <= tol
        ok[j] = p is not None and abs(w[j] / np.polyval(p, 0.0) - 1) <= tol
    return ok


def sharp_drops(current, first_crossing, dip_a=0.12, from_s=0.0015,
                past_s=0.004, merge_rows=3, n=7, fs=ROWS_HZ):
    """The first n rows where the current falls over dip_a in one row, from
    from_s to past_s past the first crossing, a commutation each."""
    t = np.arange(len(current)) / fs
    k = np.flatnonzero(np.diff(current) < -dip_a) + 1
    k = k[(t[k] > from_s) & (t[k] < first_crossing + past_s)]
    return k[np.r_[True, np.diff(k) > merge_rows]][:n]


def _centered_mean(x, n):
    x = np.asarray(x, float)
    k = np.ones(n)
    return np.convolve(x, k, "same") / np.convolve(np.ones_like(x), k, "same")


def phase_pass(x, seed, lo, hi, half=PHASE_HALF_CYCLES, step=PHASE_STEP_ROWS,
               max_rows=PHASE_MAX_ROWS):
    """(centers, delta, amp, res): per center row in [lo, hi) whose window
    of +/- half cycles of `seed` lies inside [lo, hi), the ripple's phase
    less the seed (cycles, unwrapped along the centers), the sinusoid's
    amplitude and the fit's residual rms (x's unit)."""
    sm = np.maximum.accumulate(np.asarray(seed[lo:hi], float))
    xs = np.asarray(x[lo:hi], float)
    n = len(sm)
    cs, ph, amp, res = [], [], [], []
    for c in range(0, n, step):
        if sm[c] - sm[0] < half or sm[-1] - sm[c] < half:
            continue
        a = int(np.searchsorted(sm, sm[c] - half, "left"))
        b = int(np.searchsorted(sm, sm[c] + half, "right"))
        if b - a > max_rows or b - a < PHASE_MIN_ROWS:
            continue
        u = sm[a:b] - sm[c]
        m = np.column_stack([np.ones(b - a), u, np.cos(2 * np.pi * u),
                             np.sin(2 * np.pi * u)])
        # a Hann taper over the two cycles: its response falls to nothing
        # at orders 0 and 2, and it bears a seed whose rate is off
        wt = np.sqrt(0.5 * (1 + np.cos(np.pi * u / half)).clip(0, None))
        p, *_ = np.linalg.lstsq(m * wt[:, None], xs[a:b] * wt, rcond=None)
        r = (xs[a:b] - m @ p) * wt / np.sqrt(np.mean(wt ** 2))
        # x ~ A cos(2 pi (u + seed_c + delta)): the phase at the center
        cs.append(c + lo)
        ph.append(-np.arctan2(p[3], p[2]) / (2 * np.pi) - sm[c])
        amp.append(float(np.hypot(p[2], p[3])))
        res.append(float(r.std()))
    cs = np.array(cs, int)
    if len(cs) < 2:
        return cs, np.array([]), np.array([]), np.array([])
    delta = np.unwrap(2 * np.pi * np.array(ph)) / (2 * np.pi)
    return cs, delta, np.array(amp), np.array(res)


class PhaseCount:
    """The ripple's phase over the rows [lo, hi) of x: `phi(rows)` in
    cycles, NaN outside the centers, with the count's trust numbers.

    `ok` is False when a pass finds under two centers, a seed that moves
    under two windows over the stretch. `conv` is the most the last pass
    differs from the one before at a center, a whole-cycle label aside."""

    def __init__(self, x, seed, lo, hi, passes=PHASE_PASSES,
                 half=PHASE_HALF_CYCLES, step=PHASE_STEP_ROWS,
                 max_rows=PHASE_MAX_ROWS, smooth_rows=PHASE_SMOOTH_ROWS):
        self.ok = False
        self.conv = np.nan
        self.lo, self.hi = lo, hi
        seed = np.asarray(seed, float).copy()
        last = None
        for _ in range(passes):
            cs, delta, amp, res = phase_pass(x, seed, lo, hi, half, step,
                                             max_rows)
            if len(cs) < 2:
                return
            rows = np.arange(lo, hi)
            sm = np.maximum.accumulate(seed[lo:hi])
            if last is not None:
                both = np.intersect1d(cs, last[0])
                a = np.interp(both, cs, sm[cs - lo] + delta)
                b = np.interp(both, last[0], last[1])
                d = a - b
                self.conv = float(np.max(np.abs(d - np.round(np.median(d)))))
            last = (cs, sm[cs - lo] + delta)
            # smoothed so no center's noise is fed back to the next pass
            n_s = max(1, smooth_rows // step) | 1
            d_s = _centered_mean(delta, n_s)
            seed = seed.copy()
            seed[lo:hi] = sm + np.interp(rows, cs, d_s)
        self.c = cs
        self.phi_c = sm[cs - lo] + delta
        self.amp, self.res = amp, res
        self.step = np.abs(np.diff(delta))
        self.ok = True

    def phi(self, rows):
        rows = np.asarray(rows, float)
        out = np.interp(rows, self.c, self.phi_c)
        out[(rows < self.c[0]) | (rows > self.c[-1])] = np.nan
        return out

    def max_step(self, r0, r1):
        """The largest step of the phase against the seed between two
        centers inside [r0, r1], cycles."""
        m = (self.c[:-1] >= r0) & (self.c[1:] <= r1)
        return float(self.step[m].max()) if m.any() else np.nan

    def min_snr(self, r0, r1):
        """The least sinusoid amplitude over the fit's residual rms at a
        center inside [r0, r1]."""
        m = (self.c >= r0) & (self.c <= r1)
        return float((self.amp[m] / self.res[m]).min()) if m.any() else np.nan

    def crossings(self):
        """Rows where the phase passes a whole number: one per commutation."""
        k = np.arange(np.ceil(self.phi_c[0]), np.floor(self.phi_c[-1]) + 1)
        return np.interp(k, self.phi_c, self.c)


def angle_seed(theta, cycle, rows=ANGLE_SEED_ROWS):
    """An angle as a phase count's seed, cycles: theta over one commutation
    period in theta's unit, a quadratic over `rows` around each row."""
    return savgol_filter(theta, rows, 2) / cycle


def order_crossing_rows(x, theta_cycles, lo, hi, per_rev, offset=0,
                        frac=ORDER_BAND, ns=GRID_SAMPLES):
    """Rows of the order count's upward crossings over [lo, hi):
    `angle_crossings` on the angle given in cycles, x read `offset` rows
    on, no repair."""
    th = np.maximum.accumulate(np.asarray(theta_cycles[lo:hi], float))
    th_rad = th * (2 * np.pi / per_rev)
    xs = np.asarray(x[lo + offset:hi + offset], float)
    c = angle_crossings(xs, th_rad, per_rev, frac, ns)
    keep = np.r_[True, np.diff(th_rad) > 0]
    return np.interp(c, th_rad[keep], np.arange(lo, hi)[keep]) + offset
