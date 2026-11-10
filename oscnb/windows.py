"""Stage 3's scoring windows as global row ranges of a pair's row store,
and the duty steps' truth from the ripple tach.

A window is the rows g0..g1 of the global row index (oscnb.sweeps) that
stage 3 took a window mean over, with that window's truth, zero and
probes, so a per-row estimate can be averaged over exactly those rows:
  ladder_windows     the ladder's spins and its reference rungs: the last
                     100 ms of a spin's held duty, the ripple tach seeded
                     by its coast the truth;
  hold_windows       the low holds (kick, hold, coast) and the sag's crawls
                     and cycles: a hold's last 400 ms, a cycle's spin as a
                     ladder spin;
  runup_windows      the 100 % run-ups' turns: one revolution of the order
                     count per window, its speed the truth, and the
                     dropout rows of every run-up;
  lift_windows       the load sittings' lifts: 100 ms windows from 200 ms
                     in, the pot through the table the truth;
  load_hold_windows  the load ladders' held segments, the arm at rest
                     against gravity: the pot's line the truth;
  steps_truth        the duty steps: a first-order response fitted to the
                     phase of the commutation crossings (`fit_step`).

The reductions are stage 3's, the ladder notebook's read of a spin, a hold
and its coast. Nothing of any servo is bound at import: a `Pair` carries
the folder and its constants, and every dataset-to-sitting-manifest map
({dataset: manifest csv under sittings/}) is an argument.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from . import coast, ripple, sweeps, tel

# stage 3's spin read (the ladder notebook)
SPIN_WINDOW_MS = 100    # ms, a spin's window, its held duty's last
SPIN_WINDOW_MIN_MS = 20  # ms, the window at least (half the held rows else)
TAIL_ROWS = 2000        # rows, a coast's last 100 ms: the awake zero
LAST20_ROWS = 400       # rows, the spin's last 20 ms, a second tach count
POT_MIN_ROWS = 20       # rows of the pot inside the table, a pot speed
E0_TARGET_MS = 10       # ms, the coast's early window E0 and Ke are read on
DECEL_FRAC_MAX = 0.35   # the coast's speed lost over the Ke window, at most
KE_MIN_CYCLES = 4       # ripple cycles in the Ke window, at least
# stage 3's hold read (a low cycle: kick, hold, coast)
HOLD_TAIL_ROWS = 8000   # rows, a hold's last 400 ms
SEED_MIN_HZ = 7.5       # Hz, a pot-seeded ripple count under this: none
TURNING_REV_S = 1.0     # rev/s, the pot and the coast's EMF: still turning
VALID_SHARE = 0.99      # of a window's rows with window_valid: valid
# stage 3's run-up turns (compensation q1_runup)
RUNUP_HEAD_ROWS = 40    # rows, the run-up's first, never a dropout
RUNUP_END_ROWS = 20     # rows, the run-up's last, never a dropout
POT_SLOPE_HALF = 40     # rows, the pot line's half width, the rough seed
ROUGH_SCALE_S = (0.015, 0.045)  # s, where the rough speed is scaled
RATE_HALF_MIN_S = 0.025  # s, a turn's dropout rate over +/- this at least
DRAG_REF_REV_S = 100.0  # rev/s, w_ref of stage 2's drag line (its `form`)
# stage 3's lifts (compensation q1_lift) and load holds (q3_position)
LIFT_WINDOW_S = 0.1     # s, a lift window
LIFT_SKIP = 2           # windows, the lift's first 200 ms left out
LIFT_POT_SHARE = 0.9    # of a window's rows with the pot inside the table
POT_BOX_ROWS = 41       # rows, the lift's centered pot boxcar
AT_REST_COUNTS = 4      # counts of pot over the coast's tail: at rest
LOAD_HOLD_MIN_ROWS = 1000   # rows, a held segment at least
STILL_COUNTS = 8        # counts, a held segment's pot range: still
# the duty steps' truth (speed-filters q2_truth, q2_delay)
STEP_PREV_ROWS = 2000   # rows, the segment before: its count is w0
STEP_HOLE_MAX = 200     # ticks no row covers before a step, at most
STEP_MIN_CROSSINGS = 20
STEP_END_CROSSINGS = 40  # the last crossings, the fit's first guess
STEP_TAU_START_S = 0.02
STEP_BOUNDS = ([-50.0, -400.0, 0.002], [50.0, 400.0, 0.3])  # j0, dw, tau s
KEEP_CYCLES = 0.15      # commutation periods rms, the fit's residual under
KEEP_TAU_MS = (5.0, 60.0)   # ms, a kept step's time constant inside


@dataclass(frozen=True, eq=False)
class Pair:
    """A pair's folder and the constants its windows read, all arguments.

    sweeps is the row store's sweep table (folder, i0: the global row
    offsets); chain and motor are oscnb.speed's `chain_of` and `motor_of`;
    servo the oscnb.servos entry (gear ratio, ripple per rev, pot counts
    per motor rev); probes the pair's `oscnb.winding.probes`; pot_table
    stage 3 pot-table's `.table.csv` (counts, angle_deg);
    drive_offset_counts stage 1's in-drive offset (counts)."""
    folder: Path
    sweeps: pd.DataFrame
    board: object
    chain: object
    motor: object
    servo: object
    probes: pd.DataFrame
    pot_table: pd.DataFrame
    drive_offset_counts: float

    @property
    def fs(self):
        return self.chain.fs_hz

    @property
    def per_rev(self):
        return self.servo.motor_entry.ripple_per_rev

    @property
    def deg_s_per_rev_s(self):
        """Horn deg/s per motor rev/s."""
        return 360.0 / self.servo.gear_ratio

    @property
    def v_per_count(self):
        return 1 / self.chain.tap_counts_per_v


def _offsets(pair):
    """{folder: (sweep, its first global row)}."""
    return {f: (s, int(i0)) for s, (f, i0) in
            enumerate(zip(pair.sweeps["folder"], pair.sweeps["i0"]))}


def _span(part, where):
    a, b = int(part.index[0]), int(part.index[-1]) + 1
    if b - a != len(part):
        raise ValueError(f"{where}: the window's rows are not contiguous")
    return a, b


def _probes(pair, manifest_csv):
    """{folder under the pair: (rail V, board NTC C)} of one manifest; a
    folder probed twice keeps its last probe."""
    stem = Path(manifest_csv).stem
    p = pair.probes[pair.probes["manifest"] == stem]
    head = f"telemetry/{pair.folder.name}/"
    return {w[len(head):]: (float(r), float(t))
            for w, r, t in zip(p["where"], p["rail_v"], p["ntc_c"])
            if w.startswith(head)}


def _rail_ntc(probes, rel):
    return probes.get(rel, (np.nan, np.nan))


def pot_deg(pos, table):
    """The pot through the table's inverse, tag degrees, nan outside it:
    stage 3 pot-table's `.table.csv` (counts, angle_deg) on its rising
    counts."""
    c = table["counts"].to_numpy()
    inc = np.r_[True, np.diff(c) > 0]
    return np.interp(np.asarray(pos, float), c[inc],
                     table["angle_deg"].to_numpy()[inc], left=np.nan,
                     right=np.nan)


def _boxcar(x, n):
    """Centered mean of n samples, the ends over what exists (stage 3's
    compensation common.boxcar)."""
    x = np.asarray(x, float)
    k = np.ones(n)
    return (np.convolve(x, k, "same")
            / np.convolve(np.ones_like(x), k, "same"))


def _pot_rev_s(pair, pos, direction):
    """Motor rev/s from a line through the pot through the table, signed
    so the drive reads positive (stage 3's cycles_reduce.pot_rev_s)."""
    deg = pot_deg(pos, pair.pot_table)
    ok = np.isfinite(deg)
    if ok.sum() < POT_MIN_ROWS:
        return np.nan
    t = np.arange(len(deg)) / pair.fs
    slope = np.polyfit(t[ok], deg[ok], 1)[0]
    return slope / pair.deg_s_per_rev_s * direction


def _e0_line(pair, emf, start, f_seed, target_ms=E0_TARGET_MS, order=1):
    """The coast's early line (stage 3's cycles_reduce.e0_line)."""
    f, i0, i1, n = ripple.read_window(emf, start, f_seed,
                                      target_ms=target_ms)
    if not np.isfinite(f) or n == 0:
        return dict(e_cut=np.nan, dedt=np.nan, e_win=np.nan, i0=start,
                    i1=start, f=np.nan, n=0)
    nbox = max(round(pair.fs / f), 1)
    smooth = np.convolve(emf, np.ones(nbox) / nbox, mode="same")
    t = np.arange(len(emf)) / pair.fs
    p = np.polyfit(t[i0:i1], smooth[i0:i1], order)
    return dict(e_cut=float(np.polyval(p, 0.0)),
                dedt=float(np.polyval(np.polyder(p), 0.0)),
                e_win=float(emf[i0:i1].mean()), i0=i0, i1=i1, f=f, n=n)


def _coast_read(pair, co, d, zero_e, zero_i):
    """The coast: turned, E0, the Ke window and its gate, the awake zero,
    the tail tap zero (stage 3's cycles_reduce.coast_read)."""
    row = {"awake_counts": float(co["current_raw"].iloc[-TAIL_ROWS:].mean()
                                 - zero_i)}
    cut = coast.cut_row(co)
    if cut is None:
        return row
    emf_read = coast.emf_from_cut(co, d, cut, zero_e,
                                  v_per_count=pair.v_per_count)
    row["e_first_v"] = float(coast.early_emf(emf_read))
    row["turned"] = bool(row["e_first_v"] >= coast.TURN_V)
    taps = ((co["vmotor_a"] - co["vmotor_b"]) * d).to_numpy(float)
    row["tail_tap_zero_counts"] = float(np.median(taps[-TAIL_ROWS:])
                                        - zero_e)
    tail = co["pos"].iloc[-TAIL_ROWS:]
    row["coast_pos_range"] = float(tail.max() - tail.min())
    if not row["turned"]:
        return row
    bad = coast.open_rows(emf_read)
    emf = coast.bridge(emf_read, bad)
    stop = coast.stop_row(emf)
    f0 = coast.seed_freq(emf)
    m = _e0_line(pair, emf, coast.COAST_FROM_ROWS, f0)
    row.update(f_seed=f0,
               stop_ms=np.nan if stop is None else stop / pair.fs * 1e3,
               e_cut_v=m["e_cut"], dedt_v_s=m["dedt"], e_win_v=m["e_win"],
               coast_cycles=m["n"])
    if np.isfinite(m["f"]):
        w = 2 * np.pi * m["f"] / pair.per_rev
        row["coast_rev_s"] = m["f"] / pair.per_rev
        row["ke_mv_per_rev_s"] = m["e_win"] / w * 2 * np.pi * 1e3
        win_ms = (m["i1"] - m["i0"]) / pair.fs * 1e3
        row["decel_frac_win"] = (-m["dedt"] * win_ms / 1e3 / m["e_win"]
                                 if m["e_win"] else np.nan)
        row["ke_gate"] = bool(row["decel_frac_win"] <= DECEL_FRAC_MAX
                              and m["n"] >= KE_MIN_CYCLES)
    return row


def _spin_window(pair, sp):
    """(window rows, the window before it, row): a spin's last 100 ms of
    its held duty (stage 3's cycles_reduce.spin_window)."""
    ticks = sp["duty_counts"].abs()
    half = float(ticks.iloc[len(sp) // 2:].median())
    held = sp[ticks == half]
    n_held = len(held)
    n = int(min(SPIN_WINDOW_MS * pair.fs / 1e3, n_held // 2))
    n = max(n, int(SPIN_WINDOW_MIN_MS * pair.fs / 1e3))
    n = min(n, n_held)
    win = held.iloc[-n:]
    prev = held.iloc[-2 * n:-n] if n_held >= 2 * n else None
    row = {"half_ticks": half, "duty_pct": 100 * half / pair.board.pwm_arr,
           "held_rows": n_held, "window_ms": n / pair.fs * 1e3,
           "window_valid_share": float(win["window_valid"].mean())}
    return win, prev, row


def _window_signals(pair, win, d, zero_i):
    """The window's mean crest counts over the asleep zero, V_on, its
    dropout rate and trough (stage 3's cycles_reduce.window_signals)."""
    raw = win["current_raw"].to_numpy(float)
    i_over = raw - zero_i
    taps = ((win["vmotor_a"] - win["vmotor_b"]) * d).to_numpy(float)
    v_on = ((taps - pair.chain.tap_offset_counts)
            / pair.chain.tap_counts_per_v)
    drop = tel.dropout_rows(i_over)
    row = {"i_over_counts": float(i_over.mean()),
           "i_sd_counts": float(i_over.std()),
           "v_on_v": float(np.nanmean(v_on)),
           "dropout_rows": int(drop.sum()),
           "dropouts_per_s": float(drop.sum() / (len(win) / pair.fs)),
           "rows": len(win)}
    trough = win.get("current_trough")
    if trough is not None and trough.notna().any():
        row["trough_over_zero_counts"] = float(trough.mean() - zero_i)
    return row


def _spin_tach(pair, win, prev, d, f_seed):
    """The ripple tach over the window, the window before and the last
    20 ms, seeded by the coast; the pot's speed (stage 3's
    cycles_reduce.spin_tach)."""
    row = {}
    raw = win["current_raw"].to_numpy(float)
    if np.isfinite(f_seed):
        f_cnt, ncyc = ripple.count_cycles(raw - raw.mean(), f_seed)
        row["tach_rev_s"], row["tach_cycles"] = f_cnt / pair.per_rev, ncyc
        if prev is not None:
            xp = prev["current_raw"].to_numpy(float)
            fp, _ = ripple.count_cycles(xp - xp.mean(), f_seed)
            row["tach_prev_rev_s"] = fp / pair.per_rev
        x20 = raw[-LAST20_ROWS:]
        f20, _ = ripple.count_cycles(x20 - x20.mean(), f_seed)
        row["tach_last20_rev_s"] = f20 / pair.per_rev
    row["pot_rev_s"] = _pot_rev_s(pair, win["pos"].to_numpy(float), d)
    return row


def _spin_cycle(pair, df, base, rel):
    """[(row, window)]: per direction of a spin + coast cycle, stage 3's
    reduction (cycles_reduce.reduce_spin_cycle) and the window's rows a..b
    of the sweep."""
    zero_i, zero_e = tel.rest_zeros(df)
    out = []
    for d, sp, co in coast.pairs(df):
        crow = _coast_read(pair, co, d, zero_e, zero_i)
        win, prev, srow = _spin_window(pair, sp)
        srow.update(_window_signals(pair, win, d, zero_i))
        srow.update(_spin_tach(pair, win, prev, d,
                               crow.get("f_seed", np.nan)))
        if ("tach_prev_rev_s" in srow
                and np.isfinite(srow.get("tach_rev_s", np.nan))):
            srow["settle_pct"] = (srow["tach_rev_s"]
                                  / srow["tach_prev_rev_s"] - 1) * 100
        row = {**base, "dir": d, "zero_i": zero_i, "zero_e": zero_e, **srow,
               **crow}
        a, b = _span(win, rel)
        tail = float(co["current_raw"].iloc[-TAIL_ROWS:].mean())
        out.append((row, dict(a=a, b=b, run0=int(sp.index[0]),
                              coast_tail=tail)))
    return out


def _low_cycle(pair, df, base, rel):
    """[(row, window)]: per direction of a kick + hold + coast cycle, the
    hold's last 400 ms (stage 3's cycles_reduce.reduce_low_cycle) and its
    rows a..b of the sweep."""
    zero_i, zero_e = tel.rest_zeros(df)
    segs = {int(s): g for s, g in df.groupby("seg")}
    driven = [s for s in sorted(segs) if s > 0]
    out = []
    for first in driven[::3]:
        kick, hold, co = segs[first], segs[first + 1], segs[first + 2]
        d = int(kick["dir"].iloc[0])
        crow = _coast_read(pair, co, d, zero_e, zero_i)
        ticks = hold["duty_counts"].abs()
        half = float(ticks.iloc[-1])
        held = hold[ticks == half]
        tail = held.iloc[-HOLD_TAIL_ROWS:]
        row = {"half_ticks": half,
               "duty_pct": 100 * half / pair.board.pwm_arr,
               "window_ms": len(tail) / pair.fs * 1e3,
               "window_valid_share": float(tail["window_valid"].mean())}
        row.update(_window_signals(pair, tail, d, zero_i))
        pot = _pot_rev_s(pair, tail["pos"].to_numpy(float), d)
        row["pot_rev_s"] = pot
        seed = abs(pot) * pair.per_rev if np.isfinite(pot) else 0.0
        raw = tail["current_raw"].to_numpy(float)
        # a band the filter cannot run reads as no count, as stage 3's
        try:
            f_cnt, n = (ripple.count_cycles(raw - raw.mean(), seed)
                        if seed > SEED_MIN_HZ else (np.nan, 0))
        except ValueError:
            f_cnt, n = np.nan, 0
        row.update(tach_rev_s=f_cnt / pair.per_rev, tach_cycles=n)
        cut = coast.cut_row(co)
        if cut is not None:
            emf_read = coast.emf_from_cut(co, d, cut, zero_e,
                                          v_per_count=pair.v_per_count)
            row["e_first_rev_s"] = (float(coast.early_emf(emf_read))
                                    / pair.motor.ke_v_per_rev_s)
        row["kept_turning"] = bool(
            np.isfinite(pot) and abs(pot) >= TURNING_REV_S
            and row.get("e_first_rev_s", 0) >= TURNING_REV_S)
        a, b = _span(tail, rel)
        out.append(({**base, "dir": d, "zero_i": zero_i, "zero_e": zero_e,
                     **row, **crow}, dict(a=a, b=b, run0=int(kick.index[0]))))
    return out


def _cycle_base(info, meta, rail, ntc, tag, repeat=None):
    base = {"set": tag, "k": int(info["k"]),
            "repeat": (int(info.get("repeat", 0)) if repeat is None
                       else repeat),
            "volts_set": float(info["volts_set"]), "rail_v": rail,
            "ntc_c": ntc, "t_start": float(info["t_start"]),
            "rows_dropped": int(meta.get("rows_dropped", 0))}
    if not np.isfinite(rail):
        base["rail_v"] = float(info.get("vsys_read", np.nan))
    return base


def _placed(off, rel, w):
    """A window's sweep, its global rows g0..g1 and run0."""
    s, i0 = off[rel]
    return dict(w, sweep=s, g0=i0 + w["a"], g1=i0 + w["b"],
                run0=i0 + w["run0"])


def _with_windows(W, wins, cols):
    for c in cols:
        W[c] = [w[c] for w in wins]
    W["zero_s3"] = W["zero_i"] + W["awake_counts"]
    W["truth"] = W.get("tach_rev_s", np.nan)
    W["rate"] = W["dropouts_per_s"]
    return W


def ladder_windows(pair, sittings):
    """The ladder's spins and its reference rungs, one row per direction
    of every cycle of ladder/<dataset> for each {dataset: manifest csv} of
    `sittings`: stage 3's reduction of the spin (the ladder notebook's
    columns, i_crest_ma and i_awake_ma), its window's global rows g0..g1,
    run0 (the spin's first row), coast_tail (the coast's last 100 ms of
    crest counts), zero_s3 (the awake zero, zero_i + awake_counts), truth
    (the ripple tach, rev/s) and rate (dropouts per s)."""
    off, rows, wins = _offsets(pair), [], []
    for name, csv in sittings.items():
        ds = Path("ladder") / name
        cy = pd.read_csv(pair.folder / ds / "cycles.csv")
        pr = _probes(pair, csv)
        for _, info in cy.iterrows():
            rel = str(ds / info["folder"])
            df, meta = tel.load(pair.folder / rel, pair.board)
            base = _cycle_base(info, meta, *_rail_ntc(pr, rel), name)
            for row, w in _spin_cycle(pair, df, base, rel):
                rows.append(row)
                wins.append(_placed(off, rel, w))
    W = pd.DataFrame(rows)
    W["i_crest_ma"] = ((W["i_over_counts"] - pair.drive_offset_counts)
                       * pair.chain.a_per_count * 1e3)
    W["i_awake_ma"] = ((W["i_over_counts"] - W["awake_counts"])
                       * pair.chain.a_per_count * 1e3)
    return _with_windows(W, wins, ("sweep", "g0", "g1", "run0",
                                   "coast_tail"))


def hold_windows(pair, low, sag):
    """The low holds of ladder/<dataset> for each {dataset: manifest csv}
    of `low` (one row per kick, hold, coast: the hold's last 400 ms) and
    the crawls and cycles of sag/<dataset> for `sag` (its levels.csv's
    `what`: a crawl read as a hold, a cycle as a spin), set `low-1` or
    `sag-<what>`: stage 3's reduction, valid (window_valid over
    VALID_SHARE of the rows), turning, v_eff (V), pot_abs (rev/s), the
    window's global rows g0..g1, run0 (the kick's or spin's first row),
    zero_s3, truth (the ripple tach) and rate. Stage 3's estimate on the
    window means (compensation q1_holds' w_* columns) is not kept."""
    off, rows, wins = _offsets(pair), [], []
    for name, csv in low.items():
        ds = Path("ladder") / name
        cy = pd.read_csv(pair.folder / ds / "cycles.csv")
        pr = _probes(pair, csv)
        for _, info in cy.iterrows():
            rel = str(ds / info["folder"])
            df, meta = tel.load(pair.folder / rel, pair.board)
            base = _cycle_base(info, meta, *_rail_ntc(pr, rel), name)
            for row, w in _low_cycle(pair, df, base, rel):
                rows.append(row)
                wins.append(_placed(off, rel, w))
    for name, csv in sag.items():
        ds = Path("sag") / name
        lv = pd.read_csv(pair.folder / ds / "levels.csv")
        pr = _probes(pair, csv)
        for _, info in lv.iterrows():
            if info["what"] not in ("crawl", "cycle"):
                continue
            rel = str(ds / info["folder"])
            df, meta = tel.load(pair.folder / rel, pair.board)
            base = _cycle_base(info, meta, *_rail_ntc(pr, rel),
                               f"sag-{info['what']}", repeat=0)
            red = _low_cycle if info["what"] == "crawl" else _spin_cycle
            for row, w in red(pair, df, base, rel):
                rows.append(row)
                wins.append(_placed(off, rel, w))
    W = pd.DataFrame(rows)
    W["valid"] = W["window_valid_share"] > VALID_SHARE
    W["turning"] = W["kept_turning"].fillna(True).astype(bool)
    W["v_eff"] = W["rail_v"] * W["half_ticks"] / pair.board.pwm_arr
    W["pot_abs"] = W["pot_rev_s"].abs()
    return _with_windows(W, wins, ("sweep", "g0", "g1", "run0"))


def _drag_low_a(drag):
    """A, the drag current at the low end of its speed range: stage 2
    no-load-current's line I0 = a + b ln(w / w_ref), w in rad/s, w_ref
    2 pi x DRAG_REF_REV_S (its `form`), as stage 3's q1_runup seeds the
    order count."""
    return float(drag["a_a"]["value"] + drag["b_a"]["value"] * np.log(
        max(2 * np.pi * drag["speed_range_rev_s"]["lo"], 1e-9)
        / (2 * np.pi * DRAG_REF_REV_S)))


def runup_windows(pair, sittings, r_locked_ohm, drag):
    """(table, dropped): the 100 % run-ups of run-up/<dataset> for each
    {dataset: manifest csv} of `sittings`, recounted as stage 3's q1_runup
    counts them. One row per turn of the order count: the turn's global
    rows g0..g1 (whole rows inside its crossings), run0 (the drive's
    first row), t (s from it), truth (rev/s), ok (`ripple.flag_windows`),
    rate (dropouts per s over +/- half a turn, RATE_HALF_MIN_S at least),
    stage 3's didt_s3 (A/s), i_aw_s3 (A), u_s3 (V) of the turn, zero_s3
    (the coast's awake zero), zero_i, rail_v, ntc_c and sitting (the
    manifest's number). dropped marks the dropout rows of every run-up
    over the global rows. r_locked_ohm is 3-servo resistance's
    `r_term_ohm`; drag stage 2 no-load-current's
    `drive_drag_by_level_meter[<the run-ups' rail>]` of the type
    motor."""
    i_drag0 = _drag_low_a(drag)
    ke_rad = pair.motor.ke_v_per_rev_s / (2 * np.pi)
    l_h = pair.motor.l_h
    cpr = pair.servo.counts_per_motor_rev
    fs, n_r = pair.fs, pair.per_rev
    a_pc, v_pc = pair.chain.a_per_count, pair.v_per_count
    dropped = np.zeros(int(pair.sweeps["i1"].max()), bool)
    off, out = _offsets(pair), []
    for name, csv in sittings.items():
        ds = Path("run-up") / name
        cy = pd.read_csv(pair.folder / ds / "cycles.csv")
        pr = _probes(pair, csv)
        sitting = int(csv.split("-")[1].split(".")[0])
        for _, info in cy.iterrows():
            rel = str(ds / info["folder"])
            df, _ = tel.load(pair.folder / rel, pair.board)
            s, i0 = off[rel]
            zero_i, _ = tel.rest_zeros(df)
            rail, ntc = _rail_ntc(pr, rel)
            for d, spin, co in coast.pairs(df):
                g = spin.iloc[tel.drive_start(spin):]
                g_on = i0 + int(g.index[0])
                counts = g["current_raw"].to_numpy(float)
                awake = float(co["current_raw"].iloc[-TAIL_ROWS:].mean())
                i_aw = (counts - awake) * a_pc
                i_s1 = (counts - zero_i - pair.drive_offset_counts) * a_pc
                u = (((g["vmotor_a"] - g["vmotor_b"]).to_numpy(float) * d
                      - pair.chain.tap_offset_counts) * v_pc)
                op = tel.dropout_rows(i_s1, coast.local_mean(i_s1))
                op[:RUNUP_HEAD_ROWS] = False
                op[-RUNUP_END_ROWS:] = False
                dropped[g_on:g_on + len(op)] = op
                w_pot = (ripple.local_line_slope(g["pos"].to_numpy(float),
                                                 POT_SLOPE_HALF)
                         * fs / cpr * 2 * np.pi * d)
                rough, _ = ripple.rough_speed(i_s1, u, w_pot, r_locked_ohm,
                                              l_h, ke_rad, ROUGH_SCALE_S)
                tc = ripple.count_speed(i_s1, rough, i_drag0, n_r)["tc"]
                rev = ripple.rev_table(tc, n_r, i_aw, {"i_aw": i_aw, "u": u},
                                       op)
                ok = ripple.flag_windows(rev["t"].to_numpy(),
                                         rev["w"].to_numpy())
                for j in range(len(rev)):
                    a = int(np.ceil(tc[j] * fs))
                    b = int(np.floor(tc[j + n_r] * fs))
                    t_, w_ = rev["t"].iloc[j], rev["w"].iloc[j]
                    half = max(np.pi / w_, RATE_HALF_MIN_S)
                    ra = max(int(np.ceil((t_ - half) * fs)), 0)
                    rb = min(int(np.floor((t_ + half) * fs)), len(op))
                    out.append({
                        "set": name, "k": int(info["k"]), "dir": d,
                        "sweep": s, "g0": g_on + a, "g1": g_on + b,
                        "run0": g_on, "t": t_, "truth": w_ / (2 * np.pi),
                        "ok": bool(ok[j]),
                        "rate": (op[ra:rb].sum() / ((rb - ra) / fs)
                                 if rb > ra else np.nan),
                        "didt_s3": rev["didt"].iloc[j],
                        "i_aw_s3": rev["i_aw"].iloc[j],
                        "u_s3": rev["u"].iloc[j], "zero_s3": awake,
                        "zero_i": zero_i, "rail_v": rail, "ntc_c": ntc,
                        "sitting": sitting})
    return pd.DataFrame(out), dropped


def lift_windows(pair, sittings):
    """The load sittings' lifts of load/<dataset> for each {dataset:
    manifest csv} of `sittings`: 100 ms windows every 50 ms from 200 ms
    into the held duty, kept with window_valid on VALID_SHARE of the rows
    and the pot inside the table on LIFT_POT_SHARE, as stage 3's q1_lift
    cuts them. truth is the pot's line through the table (motor rev/s in
    the drive's direction), angle_deg the window's mean angle,
    awake_at_rest the coast's tail still, rate the window's dropouts per
    s, zero_s3 the coast tail's crest mean, rail_v the probe's (the
    cycle's vsys_read without one)."""
    off, out = _offsets(pair), []
    fs = pair.fs
    n = int(LIFT_WINDOW_S * fs)
    for name, csv in sittings.items():
        ds = Path("load") / name
        cy = pd.read_csv(pair.folder / ds / "cycles.csv")
        pr = _probes(pair, csv)
        for _, info in cy.iterrows():
            rel = str(ds / info["folder"])
            df, _ = tel.load(pair.folder / rel, pair.board)
            s, i0 = off[rel]
            zero_i, _ = tel.rest_zeros(df)
            rail, ntc = _rail_ntc(pr, rel)
            if not np.isfinite(rail):
                rail = float(info["vsys_read"])
            for d, sp, co in coast.pairs(df):
                tail = co.iloc[-TAIL_ROWS:]
                g = sp.iloc[tel.drive_start(sp):]
                half = g["duty_counts"].abs().to_numpy(float)
                g = g[half == np.median(half[len(half) // 2:])]
                _span(g, rel)
                i_over = g["current_raw"].to_numpy(float) - zero_i
                valid = g["window_valid"].to_numpy() == 1
                pdeg = pot_deg(_boxcar(g["pos"].to_numpy(float),
                                       POT_BOX_ROWS), pair.pot_table)
                t = np.arange(len(g)) / fs
                for a in range(LIFT_SKIP * n, len(g) - n, n // 2):
                    q = slice(a, a + n)
                    if (valid[q].mean() < VALID_SHARE
                            or np.isfinite(pdeg[q]).sum()
                            < LIFT_POT_SHARE * n):
                        continue
                    okp = np.isfinite(pdeg[q])
                    pot = (np.polyfit(t[q][okp], pdeg[q][okp], 1)[0]
                           / pair.deg_s_per_rev_s * d)
                    g0 = i0 + int(g.index[0]) + a
                    out.append({
                        "set": name, "k": int(info["k"]),
                        "side": int(info["side"]),
                        "duty_pct": float(info["duty_pct"]),
                        "repeat": int(info["repeat"]), "dir": d,
                        "sweep": s, "g0": g0, "g1": g0 + n,
                        "run0": i0 + int(sp.index[0]), "t_ms": a / fs * 1e3,
                        "truth": pot, "angle_deg": float(np.nanmean(pdeg[q])),
                        "awake_at_rest": float(tail["pos"].max()
                                               - tail["pos"].min())
                        <= AT_REST_COUNTS,
                        "rate": float(tel.dropout_rows(i_over[q]).mean()
                                      * fs),
                        "zero_s3": float(tail["current_raw"].mean()),
                        "zero_i": zero_i, "rail_v": rail, "ntc_c": ntc})
    return pd.DataFrame(out)


def load_hold_windows(pair, sittings, lad):
    """The load ladders' held segments of load/<dataset> for each
    {dataset: manifest csv} of `sittings` (its ladder.csv): every streamed
    driven segment at one nonzero duty, window_valid on every row and
    LOAD_HOLD_MIN_ROWS long. truth is the pot's line through the table
    (motor rev/s, nan with the pot outside the table on more than a
    tenth of the rows), still its range within STILL_COUNTS. These sweeps
    have no coast, so zero_s3 is stage 3's model of the awake zero:
    zero_i + `zero_awake_step_per_v_counts` x rail_v +
    `zero_awake_step_intercept_counts` of `lad` (3-servo ladder)."""
    per_v = lad["zero_awake_step_per_v_counts"]["value"]
    c0 = lad["zero_awake_step_intercept_counts"]["value"]
    off, out = _offsets(pair), []
    fs = pair.fs
    for name, csv in sittings.items():
        ds = Path("load") / name
        lt = pd.read_csv(pair.folder / ds / "ladder.csv")
        pr = _probes(pair, csv)
        for _, info in lt.iterrows():
            rel = str(ds / info["folder"])
            df, _ = tel.load(pair.folder / rel, pair.board)
            s, i0 = off[rel]
            zero_i, _ = tel.rest_zeros(df)
            rail, ntc = _rail_ntc(pr, rel)
            for q, g in df[df["seg"] > 0].groupby("seg"):
                half = g["duty_counts"].abs().to_numpy()
                if (half.min() == 0 or half.min() != half.max()
                        or (g["window_valid"] != 1).any()
                        or len(g) < LOAD_HOLD_MIN_ROWS):
                    continue
                d = int(g["dir"].iloc[0])
                pos = g["pos"].to_numpy(float)
                pdeg = pot_deg(pos, pair.pot_table)
                okp = np.isfinite(pdeg)
                pot = (np.polyfit(np.arange(len(g))[okp] / fs, pdeg[okp],
                                  1)[0] / pair.deg_s_per_rev_s * d
                       if okp.sum() > LIFT_POT_SHARE * len(g) else np.nan)
                a, b = _span(g, rel)
                out.append({
                    "set": name, "k": int(info["k"]), "seg": int(q),
                    "side": int(info["side"]),
                    "duty_pct": 100 * float(half[0]) / pair.board.pwm_arr,
                    "dir": d, "sweep": s, "g0": i0 + a, "g1": i0 + b,
                    "run0": i0 + a, "truth": pot,
                    "pos_range": float(np.ptp(pos)),
                    "still": bool(np.ptp(pos) <= STILL_COUNTS), "rate": 0.0,
                    "zero_s3": zero_i + per_v * rail + c0, "zero_i": zero_i,
                    "rail_v": rail, "ntc_c": ntc})
    return pd.DataFrame(out)


def fit_step(tc, k0, w0, per_rev, fs):
    """(j0, dw, tau s, residual): a first-order step fitted to the phase
    of the commutation crossings tc (rows), the duty changing at row k0
    from w0 (rev/s, fixed):

        j = j0 + per_rev (w0 t + dw (t' - tau (1 - exp(-t' / tau))))

    t = (tc - k0) / fs, t' = max(t, 0), j the crossing's number; the
    residual is the rms in commutation periods."""
    t = (np.asarray(tc, float) - k0) / fs
    n = np.arange(len(tc), dtype=float)

    def model(q):
        j0, dw, tau = q
        tt = np.maximum(t, 0.0)
        return j0 + per_rev * (w0 * t + dw * (tt - tau
                                              * (1 - np.exp(-tt / tau))))

    w_end = ripple.speed_of(tc[-STEP_END_CROSSINGS:], per_rev, fs)
    fit = least_squares(lambda q: model(q) - n,
                        [0.0, w_end - w0, STEP_TAU_START_S],
                        bounds=STEP_BOUNDS)
    j0, dw, tau = fit.x
    return j0, dw, tau, float(np.sqrt(np.mean(fit.fun ** 2)))


def steps_truth(store, zero_rows, pair_dir, dataset, per_rev):
    """One row per duty step of the chains of `dataset` (its chains.csv,
    e.g. steps/capture-1) in a pair's RowStore: the drive run after a
    run of the segment before at another duty, no more than STEP_HOLE_MAX
    ticks between them. The crossings of the crest less `zero_rows` (the
    zero each row may use, counts) tracked from the segment before's
    ripple line, the step `fit_step`ped from its count w0; ok with the
    residual under KEEP_CYCLES. Columns: sweep, chain, base_pct,
    step_pct, sign (of the duty's change), dir (the duty's sign), run0,
    g0, g_step (the row the duty changes), g1, hole (ticks), w0, dw
    (rev/s), tau_ms, res_cycles, n_cross, ok."""
    arr, sw, runs = store.arrays, store.sweeps, store.runs
    fs = float(store.tick_hz)
    folders = sw["folder"].to_numpy()
    ch = pd.read_csv(Path(pair_dir) / dataset / "chains.csv")
    out = []
    for c in ch.itertuples():
        f = f"{dataset}/{c.folder}"
        s = int(np.flatnonzero(folders == f)[0])
        sgn = None
        r = runs[(runs.sweep == s)
                 & (runs.kind == sweeps.DRIVE)].reset_index(drop=True)
        for j in range(1, len(r)):
            a, p = r.iloc[j], r.iloc[j - 1]
            ai0, ai1, pi1 = int(a.i0), int(a.i1), int(p.i1)
            if (a.duty_min == a.duty_max or p.seg != a.seg - 1
                    or a.gap_before > STEP_HOLE_MAX):
                continue
            duty = np.asarray(arr["duty"][ai0:ai1]).astype(int)
            k0 = int(np.flatnonzero(np.diff(duty) != 0)[0]) + 1
            x = (np.asarray(arr["raw"][ai0:ai1]).astype(float)
                 - np.asarray(zero_rows[ai0:ai1]))
            xp = np.asarray(arr["raw"][pi1 - STEP_PREV_ROWS:pi1]).astype(
                float)
            f_prev = ripple.ripple_line(xp)
            w0 = ripple.speed_of(ripple.crossings(xp, f_prev), per_rev)
            tc = ripple.tracked_crossings(x, 0, len(x) - 1, f_prev)
            if len(tc) < STEP_MIN_CROSSINGS or not np.isfinite(w0):
                continue
            _, dw, tau, res = fit_step(tc, k0, w0, per_rev, fs)
            if sgn is None:
                rows, _ = tel.load(Path(pair_dir) / f)
                sgn = np.sign(rows["duty_counts"].to_numpy())
            b = max(j - 2, 0)
            out.append(dict(
                sweep=s, chain=int(c.k), base_pct=int(c.base_pct),
                step_pct=int(c.step_pct),
                sign=int(np.sign(duty[k0] - duty[k0 - 1])),
                dir=int(sgn[ai0 + k0 - int(sw["i0"].iat[s])]),
                run0=int(r.i0.iloc[b] if r.seg.iloc[b] >= a.seg - 2
                         else p.i0),
                g0=ai0, g_step=ai0 + k0, g1=ai1, hole=int(a.gap_before),
                w0=float(w0), dw=float(dw), tau_ms=float(tau * 1e3),
                res_cycles=res, n_cross=len(tc), ok=bool(res < KEEP_CYCLES)))
    return pd.DataFrame(out)


def steps_kept(T):
    """The steps a delay is read on: ok, tau inside KEEP_TAU_MS, dw the
    sign of the duty's change (speed-filters q2_delay)."""
    lo, hi = KEEP_TAU_MS
    return (T["ok"] & (T["tau_ms"] > lo) & (T["tau_ms"] < hi)
            & (np.sign(T["dw"]) == T["sign"]))
