"""Winding temperature: the board NTC plus heat nodes driven by the copper
loss, and the stopped-rotor reads and the heat history that grade it.

The estimate is pure and float, nothing of any servo bound at import:
every constant is an argument.

    T_w   = T_ntc + sum_k x_k                       the estimate
    P     = i_w^2 R(T_w)                            copper loss, the
                                                    resistance at T_w
    x_k  += (G_k P - x_k) (1 - exp(-dt / tau_k))    each node, exact over
                                                    a step of constant P
    R(T)  = R0 (1 + alpha (T - T0))                 copper

Nodes are in the parallel (Foster) form: each is a first-order rise of
gain G_k (C/W) and time constant tau_k (s) driven by the same power, and
the rise over the NTC is their sum. No node is the NTC alone. `i_w` is
the winding current (the crest sample less the zero and the reading term,
as oscnb.speed forms it). A step may be any dt: a control tick, a
decimated block (mean of i_w^2 over it), or a gap with i_w = 0.

The grading runs over a pair's folder (oscnb.sweeps), every constant an
argument (oscnb.speed's `Chain` and `Motor`, the board, the stage 1
scales):
  the board NTC in time (`probes`, `ntc_at`): the rows carry no NTC, so
      it comes from the sitting manifests' torque-off probes, one before
      every sweep, interpolated in time;
  the stopped-rotor read (`row_signals`, `r_read`, `dtemp_of_r`): with
      the rotor stopped the back-EMF is zero, R = (V_c - V0) / i_w under
      oscnb.speed's construction;
  the gated reads (`sub_windows`, `gate`, `read_table`): 20 ms
      sub-windows of the streamed driven segments, gated for a stopped,
      quiet rotor, pooled per segment;
  the heat history (`heat`, `heat_sittings`): i_w^2 against time over
      every sweep as pieces, the unstreamed holds and seeks accounted.
"""
import math
from pathlib import Path

import numpy as np
import pandas as pd

from . import speed, sweeps, tel
from .speed import ALPHA_CU

SUB_ROWS = 400          # rows, a 20 ms sub-window at 20 kHz
SETTLE_S = 0.060        # s, a fresh duty's seat wind-up (stage 3's laps)
POS_RANGE_MAX = 8       # counts of pot in a sub-window, the locked gate
CREEP_MAX = 2.0         # counts, a read's pot off its segment's last
SITTING_GAP_S = 600     # s, a rest over this starts a new sitting
STREAM, HOLD, SEEK = 0, 1, 2    # the kinds of heat piece
HOLD_PIECE_S = 0.1      # s, the pieces an unstreamed hold is cut into
CHAIN_GAP_S = 0.1       # s, a drive this close to the one before chains
FILL_MAX_S = 0.05       # s, a host-stamp gap a piece is stretched over
HEAD_ROWS = slice(tel.SETTLE_ROWS, tel.SETTLE_ROWS + SUB_ROWS)
OPEN_ROWS = 2           # rows, a stream's opening current (the seek's)
POS_HEAD_ROWS = 20      # rows, where a stream's pot opens
SEEK_ROWS = 200         # rows, the seek's rail and the pot it left
WATCH_S = 8 * 0.020     # s, a seek returns after 8 still polls of 20 ms
SEEK_MAX_S = 1.5        # s, a seek after a rest: the gap's last part
FREE_SEEK_A = 0.15      # A, a free seek's current, upper bound only
LIFT_S = 0.5            # s, the longest a loaded lift takes to settle
STALL_HEADROOM = 1.10   # a stall's current over the windows', at most
SEEK_DROP_V = 0.45      # V, off the seek duty's volts for its free speed
SEEK_MIN_REV_S = 30.0   # rev/s, the seek's free speed at the least


def r_of_t(t_c, r0_ohm, t0_c, alpha=ALPHA_CU):
    """Copper's resistance at t_c from r0_ohm at t0_c."""
    return r0_ohm * (1.0 + alpha * (t_c - t0_c))


def t_of_r(r_ohm, r0_ohm, t0_c, alpha=ALPHA_CU):
    """The copper temperature a resistance read implies (the thermometer,
    inverted)."""
    return t0_c + (r_ohm / r0_ohm - 1.0) / alpha


def start_state(n_nodes):
    """The state after a power cycle: every node at zero, the winding at
    the NTC. It is the lowest state the history allows, so the estimate
    starts low by whatever heat the winding still held."""
    return [0.0] * n_nodes


def estimate(state, ntc_c):
    """The winding temperature: the NTC plus the nodes."""
    return ntc_c + sum(state)


def step(state, i2_a2, ntc_c, dt_s, gains_c_per_w, taus_s, r0_ohm, t0_c,
         alpha=ALPHA_CU):
    """One step of dt_s under a mean squared winding current i2_a2 (A^2).
    Returns (new state, power W). The power takes the resistance at the
    estimate before the step. State: one rise per node (C)."""
    p = i2_a2 * r_of_t(estimate(state, ntc_c), r0_ohm, t0_c, alpha)
    out = []
    for x, g, tau in zip(state, gains_c_per_w, taus_s):
        out.append(x + (g * p - x) * (1.0 - math.exp(-dt_s / tau)))
    return out, p


def run(t0_s, dt_s, i2_a2, ntc_c, gains_c_per_w, taus_s, r0_ohm, t0_c,
        alpha=ALPHA_CU, state=None):
    """The estimate run down a timeline of pieces (start time, length,
    mean i^2, the NTC at each), in time order; the time between pieces
    passes at zero current. Returns per piece the state at its start (one
    list per node) and its power, so a caller can evaluate inside a piece
    with `inside`. State: one rise per node, carried piece to piece."""
    n = len(gains_c_per_w)
    x = list(state) if state is not None else start_state(n)
    starts = [[0.0] * len(t0_s) for _ in range(n)]
    power = [0.0] * len(t0_s)
    t_prev = None
    for j in range(len(t0_s)):
        t = t0_s[j]
        if t_prev is not None and t > t_prev:
            gap = t - t_prev
            for k in range(n):
                x[k] *= math.exp(-gap / taus_s[k])
        for k in range(n):
            starts[k][j] = x[k]
        x, p = step(x, i2_a2[j], ntc_c[j], dt_s[j], gains_c_per_w, taus_s,
                    r0_ohm, t0_c, alpha)
        power[j] = p
        t_prev = t + dt_s[j]
    return starts, power


def inside(x_start, p_w, since_s, gain_c_per_w, tau_s):
    """A node's value `since_s` into a piece that started at x_start under
    power p_w (also past its end if the caller passes p_w = 0 for the time
    after)."""
    e = math.exp(-since_s / tau_s)
    return x_start * e + gain_c_per_w * p_w * (1.0 - e)


def speed_error_rev_s(r_error_frac, i_w_a, r_ohm, ke_v_per_rev_s):
    """What a relative error in R costs the back-EMF speed estimate:
    R i / Ke times the error."""
    return r_error_frac * r_ohm * i_w_a / ke_v_per_rev_s


def probes(pair_dir, board, scales):
    """Every torque-off probe of every sitting manifest of a pair, in time
    order: manifest, t (host s), where, ntc_raw, vbus_raw, pos, ntc_c
    (`board.ntc_c_beta`), rail_v (`scales.rail_v`, stage 1's line)."""
    rows = []
    for p in sorted((Path(pair_dir) / "sittings").glob("*.csv")):
        m = pd.read_csv(p)
        for _, r in m[m["event"] == "probe"].iterrows():
            where, rest = r["detail"].split(": ", 1)
            v = dict(kv.split(" ") for kv in rest.split(", "))
            rows.append({"manifest": p.stem, "t": float(r["t_host"]),
                         "where": where, "ntc_raw": int(v["ntc_raw"]),
                         "vbus_raw": int(v["vbus_raw"]),
                         "pos": int(v["pos"])})
    out = pd.DataFrame(rows).sort_values("t").reset_index(drop=True)
    out["ntc_c"] = board.ntc_c_beta(out["ntc_raw"].to_numpy(float))
    out["rail_v"] = scales.rail_v(out["vbus_raw"].to_numpy(float))
    return out


def ntc_at(t, probe_table):
    """The board NTC (C) at host times t: the probes' line in time, held
    flat before the first and after the last. Stateless."""
    return np.interp(np.asarray(t, float), probe_table["t"].to_numpy(),
                     probe_table["ntc_c"].to_numpy())


def index(pair_dir, gap_s=SITTING_GAP_S):
    """oscnb.sweeps.index with each sweep's `sitting`: a run of sweeps
    with no rest over gap_s, numbered from 1 in time order."""
    ix = sweeps.index(pair_dir)
    ix["sitting"] = (ix["gap_s"].fillna(1e9) > gap_s).cumsum().to_numpy()
    return ix


def load(pair_dir, folder, board):
    """(rows, meta, segments, zero_i, vb) of one sweep, its sense block
    checked against `board`: zero_i the asleep current zero and vb the
    bias node (counts, the baseline's medians, the second over both
    taps)."""
    df, meta, seg = sweeps.load(pair_dir, folder, board)
    zero_i, _ = tel.rest_zeros(df)
    b = tel.baseline(df)
    vb = float(pd.concat([b["vmotor_a"], b["vmotor_b"]]).median())
    return df, meta, seg, float(zero_i), vb


def row_signals(g, board, chain, zero_i, vb):
    """(d, h, i_over, v_on, rail) per row of a driven segment: the
    direction, the half on-ticks, the crest over the asleep zero (counts),
    V_on (V, oscnb.speed.tap_volts) and the driven terminal at the crest
    (V, `board.term_v` over the bias node). Stateless, per row."""
    duty = g["duty_counts"].to_numpy(float)
    d = np.sign(duty)
    h = np.abs(duty)
    i_over = g["current_raw"].to_numpy(float) - zero_i
    va = g["vmotor_a"].to_numpy(float)
    vb_tap = g["vmotor_b"].to_numpy(float)
    v_on = speed.tap_volts(va, vb_tap, d, chain)
    rail = board.term_v(np.maximum(va, vb_tap), vb)
    return d, h, i_over, v_on, rail


def r_read(h, v_on, i_over, rail, chain, motor, zero_counts):
    """The stopped-rotor read: R = (V_c - V0) / i_w (ohm) under
    oscnb.speed.construct, the crest read against `zero_counts` over the
    asleep zero (stage 3's reads take stage 1's in-drive offset,
    `scales.drive_offset_counts`), the edge lags at `rail`. Returns
    (R, i_w A, V_c V, D). Stateless."""
    i_w, v_c, d_share, _ = speed.construct(i_over, zero_counts, v_on, h,
                                           chain, motor, rail_v=rail)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = (v_c - motor.v0_v) / i_w
    return r, i_w, v_c, d_share


def dtemp_of_r(r, r0_ohm, alpha=ALPHA_CU):
    """Copper's temperature over T0 that R implies, from r0_ohm at T0."""
    return (np.asarray(r, float) / r0_ohm - 1) / alpha


def quiet_sd_of(res):
    """Stage 3's quiet gate (counts of current sd), the upper end of the
    resistance result's `quiet_gate_i_sd_counts`."""
    return res["quiet_gate_i_sd_counts"]["hi"]


def stopped_sweeps(ix, datasets):
    """The sweeps of `ix` with the rotor stopped, in the order of
    `datasets`: (experiment, dataset) pairs, or (experiment, dataset,
    suffix) to keep only the folders ending in suffix."""
    out = []
    for entry in datasets:
        exp, ds = entry[:2]
        sel = ix[(ix["experiment"] == exp) & (ix["dataset"] == ds)]
        if len(entry) > 2:
            sel = sel[sel["folder"].str.endswith(entry[2])]
        out.append(sel)
    return pd.concat(out)


def sub_windows(pair_dir, stopped, board, chain, motor, zero_counts,
                probe_table):
    """Every SUB_ROWS sub-window of every streamed driven segment of the
    `stopped` sweeps (rows of `index`, read in their order), gated or not,
    with its read (`r_read`), the board NTC at its middle and the copper
    rise over T0 its R implies (dT_c, over motor.r_ohm). A segment's drive
    starts at its first nonzero duty; it is `fresh` when the commanded
    duty before it (rest, a seek or a step) differs."""
    fs = chain.fs_hz
    arr = chain.half_period_ticks
    rows = []
    for _, f in stopped.iterrows():
        df, _, seg, zero_i, vb = load(pair_dir, f["folder"], board)
        prev_duty = 0
        for sgi, srow in seg.iterrows():
            if sgi == 0:
                continue
            duty_cmd = int(srow["cmd_duty_q15"])
            if srow["streamed"] == 0 or duty_cmd == 0:
                prev_duty = duty_cmd
                continue
            g = df[df["seg"] == sgi]
            drv = np.flatnonzero(g["duty_counts"].to_numpy() != 0)
            if len(drv) == 0:
                prev_duty = duty_cmd
                continue
            g = g.iloc[drv[0]:]
            fresh = duty_cmd != prev_duty
            _, h, i_over, v_on, rail = row_signals(g, board, chain, zero_i,
                                                   vb)
            tick0 = int(g["tick"].iloc[0])
            for b in range(len(g) // SUB_ROWS):
                sl = slice(b * SUB_ROWS, (b + 1) * SUB_ROWS)
                gg = g.iloc[sl]
                hh = h[sl]
                half = float(np.median(hh))
                rail_s = float(np.median(rail[sl]))
                r, i_w, v_c, _ = r_read(half, float(v_on[sl].mean()),
                                        float(i_over[sl].mean()), rail_s,
                                        chain, motor, zero_counts)
                pos = gg["pos"].to_numpy(float)
                rows.append({
                    "folder": f["folder"], "experiment": f["experiment"],
                    "dataset": f["dataset"], "sitting": int(f["sitting"]),
                    "seg": int(sgi), "sub": b, "fresh": bool(fresh),
                    "dir": int(np.sign(duty_cmd)),
                    "t": float(srow["t_start"])
                    + (tick0 + (b + 0.5) * SUB_ROWS) / fs,
                    "t_in_seg": (b + 0.5) * SUB_ROWS / fs,
                    "t_seg_start": float(srow["t_start"]),
                    "t_seg_end": float(srow["t_end"]),
                    "half_ticks": half, "one_duty": bool((hh == half).all()),
                    "duty_pct": 100 * half / arr,
                    "valid": float(gg["window_valid"].mean()),
                    "i_over_counts": float(i_over[sl].mean()),
                    "i_sd_counts": float(i_over[sl].std()),
                    "v_on_v": float(v_on[sl].mean()), "rail_v": rail_s,
                    "pos": float(pos.mean()),
                    "pos_range": float(pos.max() - pos.min()),
                    "i_w_a": float(i_w), "v_c_v": float(v_c),
                    "r_ohm": float(r), "zero_i": zero_i,
                    "dropouts": int(tel.dropout_rows(i_over[sl]).sum()),
                })
            prev_duty = duty_cmd
    out = pd.DataFrame(rows)
    out["ntc_c"] = ntc_at(out["t"], probe_table)
    out["dT_c"] = dtemp_of_r(out["r_ohm"], motor.r_ohm, motor.alpha_per_c)
    return out


def gate(sub, quiet_sd, pos_range_max=POS_RANGE_MAX, settle_s=SETTLE_S,
         creep_max=CREEP_MAX):
    """The sub-windows that are reads: the pot's range within
    pos_range_max, the current's sd within quiet_sd (counts; stage 3's
    quiet gate, read from its resistance result by `quiet_sd_of`), the
    window valid on every row, one duty, no dropout row, i_w over zero,
    past settle_s of a fresh duty, and the pot's mean within creep_max of
    the segment's last sub-window (the seat has stopped creeping). The
    defaults are rules: the locked-rotor pot gate, stage 3's lap start,
    the creep bound."""
    ok = ((sub["pos_range"] <= pos_range_max)
          & (sub["i_sd_counts"] <= quiet_sd) & (sub["valid"] == 1.0)
          & sub["one_duty"] & (sub["dropouts"] == 0)
          & (~sub["fresh"] | (sub["t_in_seg"] >= settle_s))
          & (sub["i_w_a"] > 0))
    last_pos = sub.groupby(["folder", "seg"])["pos"].transform("last")
    return ok & ((sub["pos"] - last_pos).abs() <= creep_max)


def _sd(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.std(ddof=1)) if len(x) > 1 else np.nan


def read_table(sub, ok):
    """One read per streamed segment with a gated sub-window (`ok`, from
    `gate`): the gated sub-windows' means, R's first and last, its sd
    about a line in time (%, 4 or more) and its slope (% per s, 3 or
    more)."""
    reads = []
    for (folder, sgi), g in sub[ok].groupby(["folder", "seg"], sort=False):
        t = g["t"].to_numpy()
        r = g["r_ohm"].to_numpy()
        slope = (np.polyfit(t - t.mean(), r, 1)[0] / r.mean() * 100
                 if len(g) >= 3 else np.nan)
        sd = (_sd(r - np.polyval(np.polyfit(t, r, 1), t)) / r.mean() * 100
              if len(g) >= 4 else np.nan)
        a = g.iloc[0]
        reads.append({
            "folder": folder, "experiment": a["experiment"],
            "dataset": a["dataset"], "sitting": a["sitting"], "seg": sgi,
            "dir": a["dir"], "fresh": a["fresh"], "n_sub": len(g),
            "t": float(t.mean()), "t_first": float(t[0]),
            "t_last": float(t[-1]), "duty_pct": a["duty_pct"],
            "rail_v": float(g["rail_v"].mean()),
            "v_on_v": float(g["v_on_v"].mean()),
            "i_w_a": float(g["i_w_a"].mean()),
            "i_sd_counts": float(g["i_sd_counts"].mean()),
            "pos": float(g["pos"].mean()), "ntc_c": float(g["ntc_c"].mean()),
            "r_ohm": float(r.mean()), "r_first": float(r[0]),
            "r_last": float(r[-1]), "r_sub_sd_pct": sd,
            "slope_pct_per_s": slope, "dT_c": float(g["dT_c"].mean())})
    return pd.DataFrame(reads)


def _streams(df, seg, segs, board, chain, motor, zero_counts, zero_i, vb,
             add, k):
    """The streamed segments' pieces, SUB_ROWS bins of i_w^2 (driven rows
    under the construction, the rest zero); returns (per segment the
    reads the holds and seeks take, A^2 s, driven rows, invalid rows)."""
    fs = chain.fs_hz
    info = {}
    e = 0.0
    driven = invalid = 0
    for sgi in segs:
        srow = seg.loc[sgi]
        if srow["streamed"] == 0:
            continue
        g = df[df["seg"] == sgi]
        if len(g) == 0:
            continue
        duty = g["duty_counts"].to_numpy(float)
        on = (duty != 0) & (g["dir"].to_numpy() != 0)
        i2 = np.zeros(len(g))
        i_w = np.zeros(len(g))
        if on.any():
            _, h, i_over, v_on, rail = row_signals(g, board, chain, zero_i,
                                                   vb)
            rail_med = float(np.median(rail[on]))
            c_iw = speed.construct(i_over[on], zero_counts, v_on[on], h[on],
                                   chain, motor, rail_v=rail_med)[0]
            i_w[on] = c_iw
            i2[on] = c_iw ** 2
            driven += int(on.sum())
            invalid += int((g["window_valid"].to_numpy()[on] == 0).sum())
        tick = g["tick"].to_numpy()
        t_row0 = float(srow["t_start"])
        pos = g["pos"].to_numpy(float)
        for b in range(int(np.ceil(len(g) / SUB_ROWS))):
            sl = slice(b * SUB_ROWS, min((b + 1) * SUB_ROWS, len(g)))
            n = sl.stop - sl.start
            m = float(i2[sl].mean())
            add(t_row0 + tick[sl.start] / fs, n / fs, m, m, m, STREAM, k)
            e += m * n / fs
        head = HEAD_ROWS if len(g) >= HEAD_ROWS.stop else slice(0, len(g))
        tail = slice(max(len(g) - SUB_ROWS, 0), len(g))
        info[sgi] = {
            "i2_head": float(i2[head].mean()),
            "i2_tail": float(i2[tail].mean()),
            "tail_still": bool(pos[tail].max() - pos[tail].min()
                               <= POS_RANGE_MAX),
            "i_open": float(np.abs(i_w[:OPEN_ROWS]).mean()),
            "duty_open": float(abs(duty[0])),
            "pos_head": float(pos[:POS_HEAD_ROWS].mean())}
    return info, e, driven, invalid


def _holds(seg, segs, info, add, k):
    """The unstreamed holds between streamed windows: i^2 from the windows
    either side, central (a line from the tail before to the head after),
    low and high; returns their A^2 s (central, low, high)."""
    e = e_lo = e_hi = 0.0
    for j, sgi in enumerate(segs):
        srow = seg.loc[sgi]
        if srow["streamed"] != 0 or sgi == 0:
            continue
        dt = float(srow["t_end"] - srow["t_start"])
        a = info.get(segs[j - 1]) if j > 0 else None
        b = info.get(segs[j + 1]) if j + 1 < len(segs) else None
        i2_b = b["i2_head"] if b else (a["i2_tail"] if a else 0.0)
        i2_a = a["i2_tail"] if (a and a["tail_still"]) else i2_b
        # a stall's current only falls as the copper warms, so the later
        # current holds from LIFT_S on; before it the arm may still be
        # lifting at the window before's (lower) current
        lo_head = min(a["i2_tail"] if a else i2_b, i2_b)
        lo = (lo_head * min(LIFT_S, dt)
              + min(i2_a, i2_b) * max(dt - LIFT_S, 0.0)) / dt
        hi = max(i2_a, i2_b) * STALL_HEADROOM ** 2
        n_p = max(round(dt / HOLD_PIECE_S), 1)
        for q in range(n_p):
            fr = (q + 0.5) / n_p
            add(float(srow["t_start"]) + q * dt / n_p, dt / n_p,
                i2_a + (i2_b - i2_a) * fr, lo, hi, HOLD, k)
        e += 0.5 * (i2_a + i2_b) * dt
        e_lo += lo * dt
        e_hi += hi * dt
    return e, e_lo, e_hi


def _seeks(df, meta, seg, segs, info, board, chain, motor, zero_i, vb,
           pot_counts_per_deg, gear_ratio, add, k):
    """The seek before each unchained drive of a `stall` sweep: a press at
    the seek duty before the stream's first row, its current the stream's
    opening rows', its length the gap less the travel at the seek duty's
    free speed, bounded by the watch and the gap. A locked horn has no
    seek; a free seek counts toward the upper bound only. Returns A^2 s
    (central, low, high, free high)."""
    e = e_lo = e_hi = e_free_hi = 0.0
    for j, sgi in enumerate(segs):
        if sgi == 0 or sgi not in info:
            continue
        srow = seg.loc[sgi]
        prev_end = float(seg.loc[segs[j - 1], "t_end"])
        gap = float(srow["t_start"]) - prev_end
        if gap < CHAIN_GAP_S or srow["cmd_duty_q15"] == 0:
            continue
        if meta.get("static_load"):
            continue
        if meta.get("stall"):
            gap_seek = min(gap, SEEK_MAX_S)
            prev = segs[j - 1]
            pos_from = (float(df[df["seg"] == prev]["pos"]
                              .iloc[-SEEK_ROWS:].mean())
                        if prev in info or prev == 0 else np.nan)
            dist = (abs(info[sgi]["pos_head"] - pos_from)
                    if np.isfinite(pos_from) else 0.0)
            rail = float(np.nanmedian(row_signals(
                df[df["seg"] == sgi].iloc[:SEEK_ROWS], board, chain, zero_i,
                vb)[4]))
            w_rev = max((info[sgi]["duty_open"] / chain.half_period_ticks
                         * rail - SEEK_DROP_V) / motor.ke_v_per_rev_s,
                        SEEK_MIN_REV_S)
            travel = dist / (pot_counts_per_deg * 360.0 * w_rev / gear_ratio)
            press = float(np.clip(gap_seek - travel, WATCH_S, gap_seek))
            i2s = info[sgi]["i_open"] ** 2
            add(float(srow["t_start"]) - press, press, i2s,
                i2s * WATCH_S / press, i2s * gap_seek / press, SEEK, k)
            e += i2s * press
            e_lo += i2s * WATCH_S
            e_hi += i2s * gap_seek
        else:
            e_free_hi += FREE_SEEK_A ** 2 * min(gap, SEEK_MAX_S)
    return e, e_lo, e_hi, e_free_hi


def heat(pair_dir, ix, board, chain, motor, zero_counts, pot_counts_per_deg,
         gear_ratio):
    """The heat history of every sweep of `ix` (rows of `index`): i_w^2
    against time as pieces, and the joules per sweep.

    Streamed rows: i_w per row under oscnb.speed.construct (the crest
    against `zero_counts` over the asleep zero, the edge lags at the
    segment's median rail), squared, in SUB_ROWS bins. Unstreamed holds:
    i^2 from the streamed windows either side (`_holds`). Stop seeks
    (`stall` sweeps): `_seeks`, the free speed from Ke, the pot scale
    (counts per output deg) and the gear ratio. Gaps: zero. Chained
    bursts are contiguous on the servo while the host stamps leave a few
    ms between them: a piece is stretched over a gap under FILL_MAX_S and
    trimmed over an overlap.

    Returns (pieces, sweeps): pieces a dict of arrays in time order, t0
    (host s), dt (s), i2 (A^2, central), lo, hi (A^2, the bounds), kind
    (STREAM, HOLD, SEEK), sweep (the index label); sweeps one row per
    sweep with its joules at motor.r_ohm (stream, hold and seek, with
    their bounds, and a free seek's upper bound) and its driven and
    invalid (window_valid 0) row counts."""
    cols = ("t0", "dt", "i2", "lo", "hi", "kind", "sweep")
    acc = {c: [] for c in cols}

    def add(*v):
        for c, x in zip(cols, v):
            acc[c].append(x)

    r0 = motor.r_ohm
    rows = []
    for k, f in ix.iterrows():
        df, meta, seg, zero_i, vb = load(pair_dir, f["folder"], board)
        segs = list(seg.index)
        info, e_stream, driven, invalid = _streams(
            df, seg, segs, board, chain, motor, zero_counts, zero_i, vb,
            add, k)
        e_hold, e_hold_lo, e_hold_hi = _holds(seg, segs, info, add, k)
        e_seek, e_seek_lo, e_seek_hi, e_free_hi = _seeks(
            df, meta, seg, segs, info, board, chain, motor, zero_i, vb,
            pot_counts_per_deg, gear_ratio, add, k)
        rows.append({
            "folder": f["folder"], "experiment": f["experiment"],
            "dataset": f["dataset"], "sitting": int(f["sitting"]),
            "t_start": f["t_start"], "t_end": f["t_end"],
            "gap_s": f["gap_s"], "stall": bool(meta.get("stall")),
            "static": bool(meta.get("static_load")),
            "j_stream": e_stream * r0, "j_hold": e_hold * r0,
            "j_hold_lo": e_hold_lo * r0, "j_hold_hi": e_hold_hi * r0,
            "j_seek": e_seek * r0, "j_seek_lo": e_seek_lo * r0,
            "j_seek_hi": e_seek_hi * r0, "j_free_seek_hi": e_free_hi * r0,
            "driven_rows": driven, "invalid_rows": invalid})
    order = np.argsort(np.array(acc["t0"]), kind="stable")
    pieces = {c: np.array(acc[c])[order] for c in cols}
    t0, dt = pieces["t0"], pieces["dt"]
    gap_next = np.r_[t0[1:] - (t0[:-1] + dt[:-1]), 1e9]
    fill = (gap_next > 0) & (gap_next < FILL_MAX_S)
    dt = np.where(fill, dt + gap_next, dt)
    pieces["dt"] = np.where(gap_next < 0, np.maximum(dt + gap_next, 0.0), dt)
    table = pd.DataFrame(rows)
    table["j_total"] = table["j_stream"] + table["j_hold"] + table["j_seek"]
    return pieces, table


def heat_sittings(table):
    """The heat per sitting from `heat`'s sweep table: the day and first
    dataset, sweeps, minutes, the joules, the mean power (W) and the share
    of the joules no row streamed (%, central, low and high)."""
    by = table.groupby("sitting")
    g = by.agg(
        day=("t_start", lambda x: pd.to_datetime(x.min(), unit="s")
             .strftime("%m-%d %H:%M")),
        first=("dataset", "first"), sweeps=("folder", "size"),
        dur_min=("t_end", lambda x: 0.0), j_stream=("j_stream", "sum"),
        j_hold=("j_hold", "sum"), j_hold_lo=("j_hold_lo", "sum"),
        j_hold_hi=("j_hold_hi", "sum"), j_seek=("j_seek", "sum"),
        j_seek_lo=("j_seek_lo", "sum"), j_seek_hi=("j_seek_hi", "sum"),
        j_free_seek_hi=("j_free_seek_hi", "sum"), j_total=("j_total", "sum"))
    g["dur_min"] = (by["t_end"].max() - by["t_start"].min()) / 60
    g["mean_w"] = g["j_total"] / (g["dur_min"] * 60)
    g["unstreamed_pct"] = 100 * (g["j_hold"] + g["j_seek"]) / g["j_total"]
    g["unstreamed_lo_pct"] = 100 * (g["j_hold_lo"] + g["j_seek_lo"]) / (
        g["j_stream"] + g["j_hold_lo"] + g["j_seek_lo"])
    g["unstreamed_hi_pct"] = 100 * (
        g["j_hold_hi"] + g["j_seek_hi"] + g["j_free_seek_hi"]) / (
        g["j_stream"] + g["j_hold_hi"] + g["j_seek_hi"]
        + g["j_free_seek_hi"])
    return g
