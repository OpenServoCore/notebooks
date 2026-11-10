"""The current zero: stage 1's two-zero tracker, as a float estimator.

The current channel reads the shunt amplifier against a zero that moves
with the board's state, so the servo learns two zeros from its own rows:
the asleep zero (torque off) and the awake zero (the driver on). Each is
a one-pole average of the samples that feed it, z += (x - z) x alpha,
once per control tick. A settled torque-off tick feeds the asleep zero; a
settled coast or a drive no wider than the gate feeds the awake zero. A
zero unfed for STALE_TICKS ticks is stale and takes the other zero's
moves, so it follows the board while it is not fed.

The float tracker (`Rule`, `replay`, `zero_rows`, `score`, `track`) is
the estimator of record: one numba core carries every variant (alpha, the
drive-feed constant, the start rules) as parameters, with the defaults
stage 1's tracker in float. The integer port of stage 1's tracker (Q16
state, an arithmetic shift for alpha: `run_tracker`, `Tracker`,
`kinds_from_bridge`, `IntRule`, `replay_int`, `score_int`) is kept as the
reference the float form is compared with.

Replays run over a pair's RowStore (oscnb.sweeps) in time order, the
state carried from sweep to sweep; `with_truth` adds the truth they are
scored against, the crest column's mean over a long coast's tail.
"""
from dataclasses import dataclass

import numba as nb
import numpy as np
import pandas as pd

from .sweeps import ASLEEP, COAST, DRIVE, SETTLE_TICKS, TAIL_S, RowStore

ALPHA_SHIFT = 13          # alpha = 2^-13 per fed tick
FRAC = 16                 # fraction bits of the integer port's Q16 zeros
STALE_TICKS = 65535       # ticks unfed before a zero takes the other's moves
ZERO_SETTLE_TICKS = SETTLE_TICKS   # ticks after a bridge change, no feed
GATE_REC_PCT = 30         # % of the period, the widest drive that feeds
GAP_FEED_TAUS = 100       # tau of asleep feed across a gap: converged
GAP_FEED_TICKS = GAP_FEED_TAUS << ALPHA_SHIFT
START_S = 0.005           # s into a run, where its start error is read
LONG_GAP_STALES = 200     # stale periods of rest that make a sweep cold
K_NONE, K_ASLEEP, K_AWAKE = -1, 0, 1    # which zero a tick feeds
BIG_Q = np.int64(1) << 40               # no current gate, Q16


def gate_ticks(pct, pwm_arr):
    """The widest drive window, in timer ticks, that still feeds."""
    return int(pwm_arr * pct // 100)


@nb.njit
def run_tracker(kind, sample, awake_q, asleep_q, awake_fresh, asleep_fresh,
                out_awake, out_asleep):
    """Stage 1's tracker, tick-exact: Q16 zeros, alpha an arithmetic
    shift. kind per tick (K_*), sample the fed counts; writes both zeros
    per tick, returns the state."""
    for i in range(kind.shape[0]):
        k = kind[i]
        if k == 1:
            delta = ((np.int64(sample[i]) << FRAC) - awake_q) >> ALPHA_SHIFT
            awake_q += delta
            if asleep_fresh > 1:
                asleep_fresh -= 1
            else:
                asleep_fresh = 0
                asleep_q += delta
            awake_fresh = STALE_TICKS
        elif k == 0:
            delta = ((np.int64(sample[i]) << FRAC) - asleep_q) >> ALPHA_SHIFT
            asleep_q += delta
            if awake_fresh > 1:
                awake_fresh -= 1
            else:
                awake_fresh = 0
                awake_q += delta
            asleep_fresh = STALE_TICKS
        else:
            if awake_fresh > 0:
                awake_fresh -= 1
            if asleep_fresh > 0:
                asleep_fresh -= 1
        out_awake[i] = awake_q
        out_asleep[i] = asleep_q
    return awake_q, asleep_q, awake_fresh, asleep_fresh


def counts(q):
    """The rounded output, as a reading would subtract it."""
    return np.maximum((np.asarray(q, np.int64) + (1 << (FRAC - 1))) >> FRAC,
                      0)


def q_to_float(q):
    return np.asarray(q, np.float64) / (1 << FRAC)


class Tracker:
    """The integer port's state: the two zeros in Q16 and their fresh
    counters (ticks left before the zero goes stale). One update per
    control tick."""

    def __init__(self):
        self.awake_q = self.asleep_q = np.int64(0)
        self.awake_fresh = self.asleep_fresh = np.int64(0)

    def seed(self, c):
        """Both zeros at the rest measurement, the step unknown."""
        self.awake_q = np.int64(int(c)) << FRAC
        self.asleep_q = self.awake_q

    def run(self, kind, sample):
        out_a = np.empty(len(kind), np.int64)
        out_s = np.empty(len(kind), np.int64)
        state = run_tracker(kind.astype(np.int8),
                            np.asarray(sample).astype(np.int64),
                            self.awake_q, self.asleep_q, self.awake_fresh,
                            self.asleep_fresh, out_a, out_s)
        (self.awake_q, self.asleep_q, self.awake_fresh,
         self.asleep_fresh) = state
        return out_a, out_s


def kinds_from_bridge(bridge, ticks, gate):
    """Stage 1's feed rule per tick: a drive of 1..gate timer ticks feeds
    the awake zero, a coast or torque off feeds after the settle ticks."""
    n = len(bridge)
    change = np.ones(n, bool)
    change[1:] = bridge[1:] != bridge[:-1]
    idx = np.arange(n)
    since = idx - np.maximum.accumulate(np.where(change, idx, 0))
    kind = np.full(n, K_NONE, np.int8)
    kind[(bridge == DRIVE) & (ticks > 0) & (ticks <= gate)] = K_AWAKE
    settled = since >= ZERO_SETTLE_TICKS
    kind[(bridge == ASLEEP) & settled] = K_ASLEEP
    kind[(bridge == COAST) & settled] = K_AWAKE
    return kind


@dataclass
class IntRule:
    """A feed rule for the integer port, from the servo's own signals.

    gate_pct: widest drive (percent of the period) whose trough feeds the
    awake zero; 0 = no drive feeds. i_gate: feed a drive tick only while
    crest - awake zero <= i_gate counts (None = no current gate). share:
    subtract share x (crest - awake zero) from the fed sample (0 = stage
    1's feed, verbatim). coast: a settled coast feeds. gap: 'age' (no rows
    between sweeps: no feed, the zeros go stale) or 'asleep' (torque off,
    fed at the next baseline's samples). coast_raw: a coast feeds the
    crest column (the reading's own). drive_off: counts taken off a
    drive's fed sample.
    """
    gate_pct: float = GATE_REC_PCT
    i_gate: float | None = None
    share: float = 0.0
    coast: bool = True
    gap: str = "age"
    coast_raw: bool = False
    drive_off: float = 0.0
    name: str = ""


@nb.njit(cache=True)
def _core_int(bridge, ticks, trough, raw, pre_gap, s_i0, s_i1, gap_ticks,
              base_n, gate, i_gate_q, share_q, feed_coast, gap_feed, seed_q,
              coast_raw, drive_off_q, out, kind_out):
    awake_q = seed_q
    asleep_q = seed_q
    awake_fresh = 0
    asleep_fresh = 0
    for s in range(s_i0.shape[0]):
        g = gap_ticks[s]
        since = 0
        if g > 0:
            if gap_feed:
                gg = min(g, GAP_FEED_TICKS)
                for j in range(gg):
                    if awake_fresh > 0 and j < ZERO_SETTLE_TICKS:
                        awake_fresh -= 1
                    if j < ZERO_SETTLE_TICKS:
                        if asleep_fresh > 0:
                            asleep_fresh -= 1
                        continue
                    x = trough[s_i0[s] + ZERO_SETTLE_TICKS
                               + (j % (base_n[s] - ZERO_SETTLE_TICKS))]
                    delta = ((np.int64(x) << FRAC) - asleep_q) >> ALPHA_SHIFT
                    asleep_q += delta
                    if awake_fresh > 1:
                        awake_fresh -= 1
                    else:
                        awake_fresh = 0
                        awake_q += delta
                    asleep_fresh = STALE_TICKS
                since = ZERO_SETTLE_TICKS
            else:
                awake_fresh = max(0, awake_fresh - g)
                asleep_fresh = max(0, asleep_fresh - g)
        prev = -1
        for i in range(s_i0[s], s_i1[s]):
            pg = pre_gap[i]
            if pg > 0:
                awake_fresh = max(0, awake_fresh - pg)
                asleep_fresh = max(0, asleep_fresh - pg)
                prev = -1
            b = bridge[i]
            if b != prev:
                if not (i == s_i0[s] and gap_feed and b == ASLEEP):
                    since = 0
                prev = b
            else:
                since += 1
            k = K_NONE
            x_q = np.int64(trough[i]) << FRAC
            if b == DRIVE:
                if ticks[i] > 0 and ticks[i] <= gate:
                    i_q = (np.int64(raw[i]) << FRAC) - awake_q
                    if i_q <= i_gate_q:
                        k = K_AWAKE
                        x_q -= ((share_q * i_q) >> FRAC) + drive_off_q
            elif b == ASLEEP:
                if since >= ZERO_SETTLE_TICKS:
                    k = K_ASLEEP
            elif feed_coast and since >= ZERO_SETTLE_TICKS:
                k = K_AWAKE
                if coast_raw:
                    x_q = np.int64(raw[i]) << FRAC
            if k == K_AWAKE:
                delta = (x_q - awake_q) >> ALPHA_SHIFT
                awake_q += delta
                if asleep_fresh > 1:
                    asleep_fresh -= 1
                else:
                    asleep_fresh = 0
                    asleep_q += delta
                awake_fresh = STALE_TICKS
            elif k == K_ASLEEP:
                delta = (x_q - asleep_q) >> ALPHA_SHIFT
                asleep_q += delta
                if awake_fresh > 1:
                    awake_fresh -= 1
                else:
                    awake_fresh = 0
                    awake_q += delta
                asleep_fresh = STALE_TICKS
            else:
                if awake_fresh > 0:
                    awake_fresh -= 1
                if asleep_fresh > 0:
                    asleep_fresh -= 1
            out[i] = awake_q
            kind_out[i] = k


def _gap_ticks(store):
    """Ticks of rest before each sweep, from the index's gap_s."""
    gap = np.nan_to_num(store.sweeps["gap_s"].to_numpy(), nan=0.0)
    return np.minimum(gap * store.tick_hz, 2**31 - 1).astype(np.int64)


def _seed(store):
    """The first rows' trough, the zero both zeros start at (counts)."""
    return round(float(store.arrays["trough"][:ZERO_SETTLE_TICKS].mean()))


def replay_int(rule, store):
    """The integer port over every sweep of the store, state carried: the
    awake zero (Q16, after each row's update) and the feed kind per row.
    One update per row, a row per control tick."""
    arr, sw = store.arrays, store.sweeps
    n = len(arr["bridge"])
    out = np.empty(n, np.int64)
    kind = np.empty(n, np.int8)
    i_gate = (BIG_Q if rule.i_gate is None
              else np.int64(round(rule.i_gate * 65536)))
    _core_int(arr["bridge"], arr["duty"], arr["trough"], arr["raw"],
              arr["pre_gap"], sw["i0"].to_numpy(np.int64),
              sw["i1"].to_numpy(np.int64), _gap_ticks(store),
              sw["base_n"].to_numpy(np.int64),
              gate_ticks(rule.gate_pct, store.pwm_arr), i_gate,
              np.int64(round(rule.share * 65536)), rule.coast,
              rule.gap == "asleep", np.int64(int(_seed(store)) << FRAC),
              rule.coast_raw, np.int64(round(rule.drive_off * 65536)),
              out, kind)
    return out, kind


SWEEP_COLS = ("experiment", "day", "gap_s", "ds", "sub", "stall", "sload")


def _drive_runs(store):
    if "truth" not in store.runs:
        raise ValueError("the store has no truth: score with_truth(store)")
    return store.runs[store.runs["kind"] == DRIVE].copy()


def score_int(rule, store, keep=False):
    """Per-drive-run score table of the integer port.

    err_first / err_last / err_mean: the awake zero the tracker holds
    (float counts, q_to_float) less the truth, at the run's first row,
    last row and its mean. fed: the share of the run's rows that fed.
    """
    arr, sw = store.arrays, store.sweeps
    out, kind = replay_int(rule, store)
    d = _drive_runs(store)
    cs = np.r_[0.0, np.cumsum(q_to_float(out))]
    cf = np.r_[0, np.cumsum(kind == K_AWAKE)]
    i0, i1 = d["i0"].to_numpy(), d["i1"].to_numpy()
    d["z_first"] = q_to_float(out[i0])
    d["z_last"] = q_to_float(out[i1 - 1])
    d["z_mean"] = (cs[i1] - cs[i0]) / (i1 - i0)
    for k in ("first", "last", "mean"):
        d[f"err_{k}"] = d[f"z_{k}"] - d["truth"]
    d["fed"] = (cf[i1] - cf[i0]) / (i1 - i0)
    for col in SWEEP_COLS:
        d[col] = sw[col].to_numpy()[d["sweep"].to_numpy()]
    bridge = np.asarray(arr["bridge"])
    awake = bridge != ASLEEP
    d.attrs["feed_share_awake"] = float((kind[awake] == K_AWAKE).mean())
    drive = bridge == DRIVE
    d.attrs["feed_share_drive"] = float((kind[drive] == K_AWAKE).mean())
    return (d, out, kind) if keep else d


def with_truth(store):
    """The store with the truth its drive runs are scored against.

    A coast of at least TAIL_S plus the settle ticks gives a tail: the
    mean of the crest column over its last TAIL_S (tail_raw; tail_trough
    the trough column's, tail_raw_sd the crest's sd). A drive run's truth
    is the tail nearest its first row in its sweep; a sweep's truth is
    the mean of its tails, truth_rng their range, n_tails their count.
    """
    arr = store.arrays
    raw, tr = arr["raw"], arr["trough"]
    tail = int(round(TAIL_S * store.tick_hz))
    runs, sw = store.runs.copy(), store.sweeps.copy()
    for k in ("tail_raw", "tail_trough", "tail_raw_sd", "truth"):
        runs[k] = np.nan
    long = ((runs["kind"] == COAST)
            & (runs["n"] >= tail + ZERO_SETTLE_TICKS)).to_numpy()
    ends = runs["i1"].to_numpy()
    t_raw = np.full(len(runs), np.nan)
    t_tr, t_sd = t_raw.copy(), t_raw.copy()
    for j in np.flatnonzero(long):
        e = int(ends[j])
        t_raw[j] = float(raw[e - tail:e].mean())
        t_tr[j] = float(tr[e - tail:e].mean())
        t_sd[j] = float(raw[e - tail:e].std())
    runs["tail_raw"], runs["tail_trough"] = t_raw, t_tr
    runs["tail_raw_sd"] = t_sd
    truth = np.full(len(runs), np.nan)
    sv = runs["sweep"].to_numpy()
    i0 = runs["i0"].to_numpy()
    drive = (runs["kind"] == DRIVE).to_numpy()
    s_truth = np.full(len(sw), np.nan)
    s_rng = np.full(len(sw), np.nan)
    s_n = np.zeros(len(sw), np.int64)
    for s in range(len(sw)):
        m = sv == s
        tj = np.flatnonzero(m & long)
        if not len(tj):
            continue
        dj = np.flatnonzero(m & drive)
        near = np.argmin(np.abs(i0[dj][:, None] - ends[tj][None, :]), axis=1)
        truth[dj] = t_raw[tj][near]
        tv = list(t_raw[tj])
        s_truth[s], s_rng[s], s_n[s] = float(np.mean(tv)), np.ptp(tv), len(tv)
    runs["truth"] = truth
    sw["truth"], sw["truth_rng"], sw["n_tails"] = s_truth, s_rng, s_n
    return RowStore(arr, sw, runs, store.board, store.tick_hz, store.pwm_arr)


@dataclass
class Rule:
    """The float tracker. Defaults = stage 1's tracker, verbatim, in float.

    shift: alpha = 2^-shift. gate_pct, coast, gap, coast_raw: as IntRule.
    drive_off: counts taken off a drive's fed sample; a float, or a
    per-sweep array (held-out fits); drive_off_v: its slope on the rail
    about v_c (counts/V), float or per-sweep array.
    Start rules (all need the step line, per sweep: step = a + b x V):
      seed_boot   seed awake = asleep + step(V) at the first rail sample
                  when no awake zero was ever learned;
      seed_stale  seed whenever the awake zero is stale at a rail sample;
      seed_thr_v  seed when the rail sits further than this from the
                  rail the awake zero was learned at (NaN = never),
                  for `debounce` rail rows in a row (seeded at their mean);
      shift_only  on a rail move, shift the zero by b x (V - V_learned)
                  instead of re-seeding it from the asleep zero;
      ff          report awake + b x (V - V_learned) on every row (the
                  slope fed forward, no threshold).
    Fast start: fast_shift (alpha = 2^-fast_shift) for the first fast_n
    fed ticks after: fast_on = 'stale' (the awake zero was stale when
    the feed came), 'seed' (a seed or shift fired), 'both'. fast_mean:
    instead of a fixed fast alpha, the running-mean gain 1 / (k0 + n)
    floored at alpha (k0 = fast_n: the prior's weight in ticks).
    no_seed: a rail move only starts the fast alpha. f32: the state in
    float32.
    """
    shift: float = ALPHA_SHIFT
    gate_pct: float = GATE_REC_PCT
    coast: bool = True
    gap: str = "age"
    coast_raw: bool = False
    drive_off: object = 0.0
    drive_off_v: object = 0.0
    v_c: float = 0.0
    step_a: object = 0.0
    step_b: object = 0.0
    step_c: object = 0.0     # a curved step: a + b V + c V^2
    seed_boot: bool = False
    seed_stale: bool = False
    seed_thr_v: float = np.nan
    shift_only: bool = False
    ff: bool = False
    fast_shift: float = ALPHA_SHIFT
    fast_n: int = 0
    fast_on: str = ""
    fast_mean: bool = False
    debounce: int = ZERO_SETTLE_TICKS   # rail rows a move must hold
    no_seed: bool = False
    f32: bool = False
    name: str = ""


@nb.njit(cache=True)
def _core(bridge, ticks, trough, raw, rail, pre_gap, s_i0, s_i1, gap_ticks,
          base_n, gate, alpha, feed_coast, gap_feed, seed0, coast_raw,
          off0, off1, v_c, step_a, step_b, step_c, seed_boot, seed_stale,
          thr_v, shift_only, ff, fast_alpha, fast_n, fast_stale, fast_seed,
          fast_mean, f32, debounce, no_seed, out, kind_out, asleep_out,
          n_seed):
    awake = seed0
    asleep = seed0
    awake_fresh = 0
    asleep_fresh = 0
    v_now = np.nan          # the last rail sample
    v_ref = np.nan          # the rail the awake zero was learned at
    fast_left = 0
    fast_k = 0
    mv_cnt = 0
    mv_sum = 0.0
    a32 = np.float32(alpha)
    for s in range(s_i0.shape[0]):
        g = gap_ticks[s]
        since = 0
        if g > 0:
            if gap_feed:
                gg = min(g, GAP_FEED_TICKS)
                for j in range(gg):
                    if awake_fresh > 0 and j < ZERO_SETTLE_TICKS:
                        awake_fresh -= 1
                    if j < ZERO_SETTLE_TICKS:
                        if asleep_fresh > 0:
                            asleep_fresh -= 1
                        continue
                    x = float(trough[s_i0[s] + ZERO_SETTLE_TICKS
                                     + (j % (base_n[s]
                                             - ZERO_SETTLE_TICKS))])
                    delta = (x - asleep) * alpha
                    asleep += delta
                    if awake_fresh > 1:
                        awake_fresh -= 1
                    else:
                        awake_fresh = 0
                        awake += delta
                    asleep_fresh = STALE_TICKS
                since = ZERO_SETTLE_TICKS
            else:
                awake_fresh = max(0, awake_fresh - g)
                asleep_fresh = max(0, asleep_fresh - g)
        prev = -1
        for i in range(s_i0[s], s_i1[s]):
            pg = pre_gap[i]
            if pg > 0:
                awake_fresh = max(0, awake_fresh - pg)
                asleep_fresh = max(0, asleep_fresh - pg)
                prev = -1
            b = bridge[i]
            if b != prev:
                if not (i == s_i0[s] and gap_feed and b == ASLEEP):
                    since = 0
                prev = b
            else:
                since += 1
            # the start rules run only on a row that carries a rail
            v = rail[i]
            if not np.isnan(v):
                v_now = v
                fire = False
                moved = False
                if np.isnan(v_ref):
                    if seed_boot:
                        fire = True
                    elif ff or not np.isnan(thr_v):
                        # no seed asked: the zero as held is the reference
                        v_ref = v
                elif seed_stale and awake_fresh == 0:
                    fire = True
                elif not np.isnan(thr_v) and abs(v - v_ref) > thr_v:
                    moved = True
                v_seed = v
                if moved:
                    mv_cnt += 1
                    mv_sum += v
                    if mv_cnt >= debounce:
                        fire = True
                        v_seed = mv_sum / mv_cnt
                if fire or not moved:
                    mv_cnt = 0
                    mv_sum = 0.0
                if fire:
                    if no_seed:
                        pass
                    elif shift_only and not np.isnan(v_ref):
                        awake += ((step_b[s] + step_c[s] * (v_seed + v_ref))
                                  * (v_seed - v_ref))
                    else:
                        awake = (asleep + step_a[s] + step_b[s] * v_seed
                                 + step_c[s] * v_seed * v_seed)
                    v_ref = v_seed
                    if not no_seed:
                        awake_fresh = STALE_TICKS
                    n_seed[s] += 1
                    if fast_seed:
                        fast_left = fast_n
                        fast_k = 0
            k = K_NONE
            x = float(trough[i])
            if b == DRIVE:
                if ticks[i] > 0 and ticks[i] <= gate:
                    k = K_AWAKE
                    x -= off0[s]
                    if not np.isnan(v_now):
                        x -= off1[s] * (v_now - v_c)
            elif b == ASLEEP:
                if since >= ZERO_SETTLE_TICKS:
                    k = K_ASLEEP
            elif feed_coast and since >= ZERO_SETTLE_TICKS:
                k = K_AWAKE
                if coast_raw:
                    x = float(raw[i])
            if k == K_AWAKE:
                if awake_fresh == 0 and fast_stale:
                    fast_left = fast_n
                    fast_k = 0
                al = alpha
                if fast_left > 0:
                    if fast_mean:
                        al = max(alpha, 1.0 / (fast_n + fast_k))
                        fast_k += 1
                        if al <= alpha:
                            fast_left = 0
                    else:
                        al = fast_alpha
                        fast_left -= 1
                if f32:
                    z32 = np.float32(awake)
                    delta = float(np.float32(z32 + (np.float32(x) - z32)
                                             * a32)) - awake
                else:
                    delta = (x - awake) * al
                awake += delta
                if not np.isnan(v_now):
                    if np.isnan(v_ref):
                        v_ref = v_now
                    else:
                        v_ref += al * (v_now - v_ref)
                if asleep_fresh > 1:
                    asleep_fresh -= 1
                else:
                    asleep_fresh = 0
                    asleep += delta
                awake_fresh = STALE_TICKS
            elif k == K_ASLEEP:
                if f32:
                    z32 = np.float32(asleep)
                    delta = float(np.float32(z32 + (np.float32(x) - z32)
                                             * a32)) - asleep
                else:
                    delta = (x - asleep) * alpha
                asleep += delta
                if awake_fresh > 1:
                    awake_fresh -= 1
                else:
                    awake_fresh = 0
                    awake += delta
                asleep_fresh = STALE_TICKS
            else:
                if awake_fresh > 0:
                    awake_fresh -= 1
                if asleep_fresh > 0:
                    asleep_fresh -= 1
            z = awake
            if ff and not np.isnan(v_ref) and not np.isnan(v_now):
                z += ((step_b[s] + step_c[s] * (v_now + v_ref))
                      * (v_now - v_ref))
            out[i] = z
            kind_out[i] = k
            asleep_out[i] = asleep


def _per_sweep(x, n):
    a = np.asarray(x, np.float64)
    return np.full(n, float(a)) if a.ndim == 0 else a.astype(np.float64)


def replay(rule, store, asleep=False):
    """The float tracker over every sweep of the store, state carried: the
    awake zero per row (float counts, after the row's update), the feed
    kind per row, the count of seeds per sweep [, the asleep zero]. State:
    the two zeros, their fresh counters, the last rail sample and the
    rail the awake zero was learned at; one update per row, a row per
    control tick."""
    arr, sw = store.arrays, store.sweeps
    n, ns = len(arr["bridge"]), len(sw)
    out = np.empty(n, np.float64)
    out_s = np.empty(n, np.float64)
    kind = np.empty(n, np.int8)
    n_seed = np.zeros(ns, np.int64)
    _core(arr["bridge"], arr["duty"], arr["trough"], arr["raw"], arr["rail"],
          arr["pre_gap"], sw["i0"].to_numpy(np.int64),
          sw["i1"].to_numpy(np.int64), _gap_ticks(store),
          sw["base_n"].to_numpy(np.int64),
          gate_ticks(rule.gate_pct, store.pwm_arr), 2.0 ** -rule.shift,
          rule.coast, rule.gap == "asleep", float(_seed(store)),
          rule.coast_raw, _per_sweep(rule.drive_off, ns),
          _per_sweep(rule.drive_off_v, ns), float(rule.v_c),
          _per_sweep(rule.step_a, ns), _per_sweep(rule.step_b, ns),
          _per_sweep(rule.step_c, ns),
          rule.seed_boot, rule.seed_stale, float(rule.seed_thr_v),
          rule.shift_only, rule.ff, 2.0 ** -rule.fast_shift,
          int(rule.fast_n), rule.fast_on in ("stale", "both"),
          rule.fast_on in ("seed", "both"), rule.fast_mean, rule.f32,
          int(rule.debounce), rule.no_seed, out, kind, out_s, n_seed)
    return (out, kind, n_seed, out_s) if asleep else (out, kind, n_seed)


def zero_rows(rule, store):
    """The awake zero each row may use (float64 counts per global row): the
    float tracker's state after the row before, as a controller reads it
    before the row's own update; row 0 takes its own. State: replay's."""
    z, _, _ = replay(rule, store)
    return np.r_[z[0], z[:-1]].astype(np.float64)


def score(rule, store, keep=False):
    """Per-drive-run score table of the float tracker: score_int's
    columns, and err_start, the error START_S into the run (past any
    start rule). attrs["n_seed"]: seeds per sweep."""
    sw = store.sweeps
    out, kind, n_seed = replay(rule, store)
    d = _drive_runs(store)
    cs = np.r_[0.0, np.cumsum(out)]
    cf = np.r_[0, np.cumsum(kind == K_AWAKE)]
    i0, i1 = d["i0"].to_numpy(), d["i1"].to_numpy()
    d["z_first"], d["z_last"] = out[i0], out[i1 - 1]
    d["z_mean"] = (cs[i1] - cs[i0]) / (i1 - i0)
    start = int(round(START_S * store.tick_hz))
    d["z_start"] = out[np.minimum(i0 + start, i1 - 1)]
    for k in ("first", "last", "mean", "start"):
        d[f"err_{k}"] = d[f"z_{k}"] - d["truth"]
    d["fed"] = (cf[i1] - cf[i0]) / (i1 - i0)
    for col in SWEEP_COLS:
        d[col] = sw[col].to_numpy()[d["sweep"].to_numpy()]
    d.attrs["n_seed"] = n_seed
    return (d, out, kind) if keep else d


def track(kind, x, awake, asleep, awake_fresh=0, asleep_fresh=0,
          alpha=2.0 ** -ALPHA_SHIFT):
    """The two-zero tracker in float, one tick at a time (the reference
    form of the numba core).

    kind: K_NONE, K_ASLEEP or K_AWAKE per tick; x: the fed sample
    (counts). State: the two zeros and their fresh counters. Returns the
    awake zero per tick and the state.
    """
    out = np.empty(len(kind))
    for i, (k, xi) in enumerate(zip(kind, x)):
        if k == K_AWAKE:
            d = (xi - awake) * alpha
            awake += d
            if asleep_fresh > 1:
                asleep_fresh -= 1
            else:
                asleep_fresh = 0
                asleep += d
            awake_fresh = STALE_TICKS
        elif k == K_ASLEEP:
            d = (xi - asleep) * alpha
            asleep += d
            if awake_fresh > 1:
                awake_fresh -= 1
            else:
                awake_fresh = 0
                awake += d
            asleep_fresh = STALE_TICKS
        else:
            awake_fresh = max(0, awake_fresh - 1)
            asleep_fresh = max(0, asleep_fresh - 1)
        out[i] = awake
    return out, (awake, asleep, awake_fresh, asleep_fresh)


def seed_awake(asleep, rail_v, step_a, step_b):
    """The start rule's seed: the asleep zero plus the step line at the
    rail."""
    return asleep + step_a + step_b * rail_v


def seed_threshold_v(step_b, sd_line, sd_held):
    """The rail move past which seeding from the line beats holding the
    learned step: b x dV > sqrt(sd_line^2 - sd_held^2)."""
    return float(np.sqrt(max(sd_line**2 - sd_held**2, 0.0)) / step_b)


def step_table(sw):
    """Per sweep with a truth: rail, awake step (coast truth less the
    asleep baseline, both in the crest column), day."""
    m = sw["truth"].notna() & sw["rail"].notna() & sw["base_raw"].notna()
    return pd.DataFrame(dict(sweep=np.flatnonzero(m),
                             rail=sw["rail"][m].to_numpy(),
                             step=(sw["truth"] - sw["base_raw"])[m].to_numpy(),
                             day=sw["day"][m].to_numpy()))


def step_fits(sw, by="day", deg=1):
    """Step shape per sweep, fitted with the sweep's own group left out.

    by: 'day', 'set' (experiment/dataset: a capture set), or None (all
    sweeps, in sample). deg 1 = the line, 2 = a + b V + c V^2. Returns
    (a, b, c) per sweep, the fold table, and the per-sweep held-out
    residual table.
    """
    t = step_table(sw)
    grp = None if by is None else (
        sw["day"] if by == "day" else sw["experiment"] + "/" + sw["ds"])
    n = len(sw)
    co_all = np.polyfit(t["rail"], t["step"], deg)[::-1]
    coef = np.zeros((n, 3))
    coef[:, :deg + 1] = co_all
    res = t["step"].to_numpy() - np.polyval(co_all[::-1],
                                            t["rail"].to_numpy())
    rec = [dict(fold="all", n_fit=len(t),
                **{f"c{i}": co_all[i] for i in range(deg + 1)},
                sd_fit=res.std(ddof=deg + 1), n_out=0)]
    if grp is not None:
        tg = grp.to_numpy()[t["sweep"].to_numpy()]
        res = np.full(len(t), np.nan)
        for g in pd.unique(grp):
            fit, out = t[tg != g], t[tg == g]
            if fit["rail"].nunique() <= deg:
                continue
            cf = np.polyfit(fit["rail"], fit["step"], deg)[::-1]
            m = (grp == g).to_numpy()
            coef[m, :] = 0.0
            coef[m, :deg + 1] = cf
            if len(out):
                r = out["step"].to_numpy() - np.polyval(
                    cf[::-1], out["rail"].to_numpy())
                res[tg == g] = r
                rf = fit["step"].to_numpy() - np.polyval(
                    cf[::-1], fit["rail"].to_numpy())
                rec.append(dict(
                    fold=f"without {g}", n_fit=len(fit),
                    **{f"c{i}": cf[i] for i in range(deg + 1)},
                    sd_fit=rf.std(ddof=deg + 1), n_out=len(out),
                    rail_span_fit=np.ptp(fit["rail"]), out_mean=r.mean(),
                    out_sd=r.std(ddof=1) if len(r) > 1 else np.nan))
    t["res_out"] = res
    return (coef[:, 0], coef[:, 1], coef[:, 2]), pd.DataFrame(rec), t


def class_bounds(step_slope, tick_hz):
    """(half_count_v, long_gap_s): the rail move worth half a count of
    the step line (step_slope counts/V) and the rest that makes a sweep
    cold, LONG_GAP_STALES stale periods."""
    return 0.5 / step_slope, LONG_GAP_STALES * STALE_TICKS / tick_hz


def sweep_class(sw, half_count_v, long_gap_s):
    """cold / rail moved / warm per sweep: cold after no sweep or a rest
    over long_gap_s, rail moved when its rail differs from the sweep
    before's by more than half_count_v."""
    cold = (sw["gap_s"].isna() | (sw["gap_s"] > long_gap_s)).to_numpy()
    moved = ((sw["rail"] - sw["rail"].shift(1)).abs().to_numpy()
             > half_count_v)
    return np.where(cold, "cold", np.where(moved, "rail moved", "warm"))


def classes(d, sw, half_count_v, long_gap_s):
    """A score table with its sweep's class (cls) and first_in_sweep."""
    d = d.copy()
    prail = sw["rail"].shift(1).to_numpy()[d["sweep"]]
    rail = sw["rail"].to_numpy()[d["sweep"]]
    cold = (d["gap_s"].isna() | (d["gap_s"] > long_gap_s)).to_numpy()
    moved = np.abs(rail - prail) > half_count_v
    d["cls"] = np.where(cold, "cold", np.where(moved, "rail moved", "warm"))
    d["first_in_sweep"] = d.groupby("sweep")["i0"].transform("min") == d["i0"]
    return d


def stats(e):
    """rms, mean, sd, worst, n of an error series (NaN dropped)."""
    e = np.asarray(e, float)
    e = e[np.isfinite(e)]
    if not len(e):
        return dict(rms=np.nan, mean=np.nan, sd=np.nan, worst=np.nan, n=0)
    return dict(rms=float(np.sqrt((e**2).mean())), mean=float(e.mean()),
                sd=float(e.std(ddof=1)) if len(e) > 1 else np.nan,
                worst=float(e[np.argmax(np.abs(e))]), n=len(e))


def boot_rms(d, col, n=2000, seed=1):
    """95 % bracket of an rms, resampling sweeps."""
    g = d[d[col].notna()].groupby("sweep")[col]
    ss = g.apply(lambda x: (x**2).sum()).to_numpy()
    cnt = g.size().to_numpy()
    if len(ss) < 4:
        return np.nan, np.nan
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(ss), (n, len(ss)))
    return tuple(np.percentile(np.sqrt(ss[idx].sum(1) / cnt[idx].sum(1)),
                               [2.5, 97.5]))
