"""motor-model's brush-drop machinery: the running R from the coast cut, and the speed-set voltage it finds.

Only motor-model reads it, so it lives beside the notebook, next to motor_model.py, whose constants and coast
reduction it builds on. Every measured constant comes through the results files, through motor_model.py's module
constants: current-scale, terminal-chain, sample-timing and bridge-states for the board, rest-resistance, bemf-coast,
speed-torque and inductance for the motor. motor-model's own spinning results (the coast's scale to the meter, the
crest term by rail and duty and the 100 % term) are handed in by the notebook from the same run
through `carry`, so the section never reads a result file the notebook is about to rewrite.

The per-spin reduction is motor-model's `reduce_spins` (bemf-coast's coast read) widened: the pre-cut means over
several windows, the EMF at the cut by several reads, the coast's drag three ways, the spin's speed by its current
ripple (seeded by the coast and, independently, by a rough EMF speed), the dropout rate, the dropout rows' raw crest
(the zero a driven bridge sits on) and the cut's phase in the 6/rev ripple.
"""
import contextlib
import io
import json
import os
import warnings

import numpy as np
import pandas as pd

import motor_model as mm
from oscnb import bursts as bu
from oscnb import results, tel

BOARD, MOTOR = mm.BOARD, mm.MOTOR
ARR, P, FS, N_R = mm.ARR, mm.P, mm.FS, mm.N_R
CPA, K_CPV, C_OFF = mm.CPA, mm.K_CPV, mm.C_OFF
KE, J = mm.KE, mm.J
CS, TC, ST, BSR, RR, BEMF, SPT = mm.CS, mm.TC, mm.ST, mm.BSR, mm.RR, mm.BEMF, mm.SPT
IND = results.read("2-motor", "inductance", pair=MOTOR.key)


def half_range(q):
    return (q["hi"] - q["lo"]) / 2


KE_LO, KE_HI = BEMF["ke_v_s_per_rad"]["lo"], BEMF["ke_v_s_per_rad"]["hi"]
DRIVE_OFF = CS["drive_offset_counts"]["value"]                       # counts, the in-drive offset of the crest current
DRIVE_OFF_ERR = half_range(CS["drive_offset_counts"])
CPA_CLASS_PCT = CS["counts_per_a_class_pct"]["value"]
TERM_ERR_PCT = TC["terminal_error_at_nominal_pct"]["value"]
R_BRK = BSR["low_side_loop_ohm"]["value"]
R_BRK_ERR = half_range(BSR["low_side_loop_ohm"])
R_LOCKED = RR["r_term_median_ohm"]["value"]
R_LOCKED_LO, R_LOCKED_HI = RR["r_term_median_ohm"]["lo"], RR["r_term_median_ohm"]["hi"]
R_LOCKED_AMPS = RR["r_term_amps"]["value"]
R_RUNUP = RR["r_runup_ohm"]["value"]
R_RUNUP_AMPS = RR["r_runup_amps"]["value"]
R_OHMS = RR["r_ohms_median_ohm"]["value"]                            # the meter's ohms range at its milliamp, a day later
TAIL_ZERO = {int(k): v["value"] for k, v in BEMF["tail_zero_counts_by_direction"].items()}

# sample-timing's slow-decay tap edges at its two rails; a rung between them takes the line between the two, which is
# what a bridge output slewing at one rate gives (sample-timing's reading of its 5-6 tick split)
RAILS = (3.7, 7.4)
LAG_ON = (ST["tap_on_edge_lag_3v7"]["value"], ST["tap_on_edge_lag_7v4"]["value"])
LAG_OFF = (ST["off_edge_lag_3v7"]["value"], ST["off_edge_lag_7v4"]["value"])

TELE = mm.PAIR
RUNS = {"capture-1": TELE / "bemf-coast" / "capture-1", "meter-1": TELE / "bemf-coast" / "meter-1",
        "high-1": TELE / "bemf-coast" / "high-1", "speed-torque": TELE / "speed-torque" / "capture-1"}
SPUN = TELE / "bemf-coast" / "spinning-1"
PRE_WINDOWS_S = (0.4, 0.2, 0.1, 0.05, 0.02, 0.01)

DELTA_SYS_MV = 5.0            # the edge term's band in the main read: the fold's dead-time dip and 5 V tail are 2-9 mV
ZERO_SYS_COUNTS = 1.2         # the EMF zero's span either side of the center, bemf-coast's none-to-tail (about +/-1.2 counts)
SLOW_BAND = 250.0             # rev/s; the speeds over which the flat-band test runs first
LOW = 160.0                   # rev/s; the band where the speed-set excess is under a few mV

# motor-model's own spinning results, handed in by the notebook from the same run (see `carry`)
CARRIED = {}


def carry(coast_over_meter, crest_less_mean_ma, full_duty_term_ma):
    """motor-model's spinning results in the results files' {value, lo, hi, unit} form, from the notebook's run"""
    CARRIED.update(coast_over_meter=coast_over_meter, crest_less_mean_ma=crest_less_mean_ma, full_duty_term_ma=full_duty_term_ma)


def c_scale():
    """the meter mean over the coast's pinned drag, and its sd (the bounds are +/- one sd of the 30 meter spins)"""
    q = CARRIED["coast_over_meter"]
    return q["value"], q["hi"] - q["value"]


def edge_lags(rail):
    """(ON, OFF) tap lags in ticks at a rail, on the line between sample-timing's 3.7 and 7.4 V values"""
    f = (rail - RAILS[0]) / (RAILS[1] - RAILS[0])
    return LAG_ON[0] + f * (LAG_ON[1] - LAG_ON[0]), LAG_OFF[0] + f * (LAG_OFF[1] - LAG_OFF[0])


def share_of(half_ticks, on_lag, off_lag):
    """the ON share of the period in slow decay, the bridge's edges at the given lags from the commanded ones"""
    return (2 * np.asarray(half_ticks, float) - on_lag + off_lag) / P


def term_ma(level, duty):
    """motor-model's predicted crest sample less the period mean, no meter, with its band's half width"""
    if duty >= 100:
        q = CARRIED["full_duty_term_ma"]
        return q["value"], half_range(q)
    q = CARRIED["crest_less_mean_ma"][f"{level:g} V"][f"{int(duty)} %"]
    return q["value"], half_range(q)


# ---------------------------------------------------------------- the widened per-spin reduction

def line_read(eb, start, target_ms, f_seed, order=1):
    """the EMF projected to the cut: a polynomial through the one-ripple-period boxcar of the bridged EMF over the
    whole-cycle window nearest target_ms from `start` (bemf-coast's E0 read); returns the cut value, the slope, the
    window's mean, its middle, its frequency and cycles, and the line's standard error at the cut"""
    f, i0, i1, n = mm.read_window(eb, start, f_seed, target_ms=target_ms)
    if not np.isfinite(f) or n == 0:
        return dict(e_cut=np.nan, dedt=np.nan, e_win=np.nan, t_mid=np.nan, f=np.nan, n=0, i0=start, i1=start, se=np.nan)
    nbox = max(int(round(FS / f)), 1)
    es = np.convolve(eb, np.ones(nbox) / nbox, mode="same")
    t = np.arange(len(eb)) / FS
    p, cov = np.polyfit(t[i0:i1], es[i0:i1], order, cov=True) if i1 - i0 > order + 3 else (np.polyfit(t[i0:i1], es[i0:i1], order), None)
    # the boxcar's rows are correlated over one ripple period, so the fit's own se is widened by the root of the box
    se = float(np.sqrt(cov[-1, -1]) * np.sqrt(nbox)) if cov is not None else np.nan
    return dict(e_cut=float(np.polyval(p, 0.0)), dedt=float(np.polyval(np.polyder(p), 0.0)), e_win=float(eb[i0:i1].mean()),
                t_mid=(i0 + i1) / 2 / FS, f=f, n=n, i0=i0, i1=i1, se=se)


def first_cycles_read(eb, f, dedt, ncyc, skip=1):
    """the mean of the bridged EMF over the first `ncyc` whole ripple cycles after the cut (from row `skip`), moved
    back to the cut with the line's slope"""
    per = FS / f
    i0, i1 = skip, skip + int(round(ncyc * per))
    if i1 > len(eb):
        return np.nan
    t_mid = (i0 + i1 - 1) / 2 / FS
    return float(eb[i0:i1].mean() - dedt * t_mid)


def first_cycle_fit(eb, f, dedt, skip=1, ncyc=1):
    """the EMF over `ncyc` ripple periods from row `skip` as the constant of a fit with the line's slope held and one
    sinusoid at the ripple frequency free, so a window that is not a whole number of rows leaks no ripple; at the cut"""
    per = FS / f
    i0, i1 = skip, skip + int(np.ceil(ncyc * per)) + 1
    if i1 > len(eb):
        return np.nan
    t = np.arange(i0, i1)
    y = eb[i0:i1] - dedt * t / FS
    X = np.c_[np.ones_like(t, dtype=float), np.cos(2 * np.pi * t / per), np.sin(2 * np.pi * t / per)]
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    return float(b[0])


def cut_phase(eb, f):
    """where in the 6/rev ripple cycle the cut landed: a sinusoid at the ripple frequency (with a line under it) fitted
    to the bridged EMF over rows 1 to 1 + 4 cycles, its phase read at row 0, as a fraction of a cycle"""
    per = FS / f
    i0, i1 = 1, 1 + int(round(4 * per))
    if i1 + 2 > len(eb):
        return np.nan
    t = np.arange(i0, i1)
    X = np.c_[np.ones_like(t, dtype=float), t, np.cos(2 * np.pi * t / per), np.sin(2 * np.pi * t / per)]
    b, *_ = np.linalg.lstsq(X, eb[i0:i1], rcond=None)
    phi = np.arctan2(b[2], b[3])
    return float((-phi / (2 * np.pi)) % 1.0)


def dropout_rate(ic_ma, n_rows):
    """the spin's current dropouts as bemf-coast counts them: rows of the window whose crest sample over the zero sits
    under tel.DROPOUT_FRAC of the window's mean, per second"""
    return float(tel.dropout_rows(ic_ma[-n_rows:]).sum() / (n_rows / FS))


# the crest over the window mean, in bands either side of the dropout threshold, for the threshold's separation check
ROW_BANDS = (("rows_under_0p10", -np.inf, 0.10), ("rows_0p10_to_thr", 0.10, tel.DROPOUT_FRAC),
             ("rows_thr_to_0p40", tel.DROPOUT_FRAC, 0.40), ("rows_0p40_to_0p60", 0.40, 0.60))


def free_peak(x, lo=150.0, hi=6000.0):
    """the strongest line of the window's current in the ripple band, no seed: what a free search reads"""
    return mm.line_freq(x, lo, hi)


def reduce_folder(folder, run):
    """every spin and coast pair of one cycle folder: one dict per spin"""
    df, meta = tel.load(folder, BOARD)
    rest = tel.baseline(df)
    zero_i = float(rest["current_raw"].median())
    zero_e = float((rest["vmotor_a"] - rest["vmotor_b"]).median())
    out = []
    for d, sp, co in mm.spin_coast_pairs(df):
        row = dict(run=run, folder=str(results.rel(folder)), dir=d, zero_i=zero_i, zero_e=zero_e,
                   rail_v=float(BOARD.rail_v(sp["vbus_raw"].iloc[-int(0.2 * FS):].mean())))
        half = float(sp["duty_counts"].abs().iloc[len(sp) // 2:].median())
        row["half_ticks"] = half
        row["duty_applied_pct"] = float(BOARD.duty_pct_of_ticks(half))
        vd = ((sp["vmotor_a"] - sp["vmotor_b"]) * d - zero_e).to_numpy(float) / K_CPV
        ic = (sp["current_raw"].to_numpy(float) - zero_i - DRIVE_OFF) / CPA * 1e3
        for L in PRE_WINDOWS_S:
            n = int(L * FS)
            row[f"v_on_{int(L * 1e3)}"] = float(vd[-n:].mean())
            row[f"i_crest_{int(L * 1e3)}"] = float(ic[-n:].mean())
            row[f"dropouts_{int(L * 1e3)}"] = dropout_rate(ic, n)
        n200 = int(0.2 * FS)
        q = ic[-n200:] / ic[-n200:].mean()
        for name, lo_, hi_ in ROW_BANDS:
            row[name] = int(((q >= lo_) & (q < hi_)).sum())
        # the brush path open for the period, so the raw crest over the torque-off zero is the zero a driven bridge
        # sits on, read with no in-drive offset removed; partial periods read above the floor, hence the 10th percentile
        raw0 = sp["current_raw"].to_numpy(float)[-n200:] - zero_i
        deep = raw0[q < 0.10]
        for name, pct in (("med", 50), ("p10", 10), ("p90", 90)):
            row[f"deep_zero_{name}_counts"] = float(np.percentile(deep, pct)) if len(deep) else np.nan
        drop = raw0[q < tel.DROPOUT_FRAC]
        row["dropout_zero_med_counts"] = float(np.median(drop)) if len(drop) else np.nan
        row["v_on_sd_200"] = float(vd[-n200:].std(ddof=1))
        row["i_crest_sd_200"] = float(ic[-n200:].std(ddof=1))
        # the 200 ms means' own standard errors from 20 blocks of 10 ms, which carries the ripple's correlation
        row["v_on_se_200"] = float(vd[-n200:].reshape(20, -1).mean(axis=1).std(ddof=1) / np.sqrt(20))
        row["i_crest_se_200"] = float(ic[-n200:].reshape(20, -1).mean(axis=1).std(ddof=1) / np.sqrt(20))
        row["spin_rows"] = len(sp)
        row["dropouts_per_s"] = row["dropouts_200"]
        z = mm.cut_row(co)
        row["cut_row"] = z
        ca, cb = co["vmotor_a"].to_numpy(float), co["vmotor_b"].to_numpy(float)
        pre = ((ca - cb) * d - zero_e)[:z] / K_CPV
        row["v_on_tail"] = float(pre.mean()) if z > 0 else np.nan
        row["i_coast_zero_ma"] = float((co["current_raw"].to_numpy(float)[z + 20:z + 400].mean() - zero_i) / CPA * 1e3)
        e = (((ca - cb) * d)[z:] - zero_e) / K_CPV
        if e[2:20].mean() < mm.TURN_V:
            out.append(row)
            continue
        bad = mm.open_rows(e)
        eb = mm.bridge_rows(e, bad)
        row["open_rows_coast"] = int(bad.sum())
        f0 = mm.line_freq(eb[mm.COAST_FROM:mm.COAST_FROM + 200])
        row["f_seed_coast"] = f0
        main = line_read(eb, mm.COAST_FROM, mm.TARGET_MS, f0)
        row.update(e_cut=main["e_cut"], e_cut_se=main["se"], dedt=main["dedt"], e_win=main["e_win"], t_mid=main["t_mid"],
                   f_coast=main["f"], n_cyc=main["n"])
        row["coast_rev_s"] = main["f"] / N_R
        row["e_cut_5ms"] = line_read(eb, mm.COAST_FROM, 5, f0)["e_cut"]
        row["e_cut_20ms"] = line_read(eb, mm.COAST_FROM, 20, f0)["e_cut"]
        row["e_cut_quad20"] = line_read(eb, mm.COAST_FROM, 20, f0, order=2)["e_cut"]
        row["e_cut_from2ms"] = line_read(eb, 2 * mm.COAST_FROM, mm.TARGET_MS, f0)["e_cut"]
        row["e_cut_unbridged"] = line_read(e, mm.COAST_FROM, mm.TARGET_MS, f0)["e_cut"]
        row["e_first1"] = first_cycles_read(eb, main["f"], main["dedt"], 1)
        row["e_first3"] = first_cycles_read(eb, main["f"], main["dedt"], 3)
        row["e_first1_fit"] = first_cycle_fit(eb, main["f"], main["dedt"], 1, 1)
        row["e_first2_fit"] = first_cycle_fit(eb, main["f"], main["dedt"], 1, 2)
        row["e_second1_fit"] = first_cycle_fit(eb, main["f"], main["dedt"], 1 + int(round(FS / main["f"])), 1)
        row["phase_cut"] = cut_phase(eb, main["f"])
        # the spin's speed by its current ripple: seeded by the coast's early line (bemf-coast's spin route), and seeded
        # instead by a rough EMF speed, (duty x V_on - R_locked x crest) / Ke, which needs only to land within the
        # band-pass's +/-35 % to pick the 6/rev line and is independent of the coast; and the free peak beside them
        n_sp = int(0.2 * FS) if run != "speed-torque" else int(0.05 * FS)
        xi = sp["current_raw"].to_numpy(float)[-n_sp:]
        fs_c, nc = mm.count_cycles(xi, f0)
        row["spin_rev_s"] = fs_c / N_R
        row["spin_cycles"] = nc
        dfrac = min(half / ARR, 1.0)
        w_rough = (dfrac * row["v_on_200"] - R_LOCKED * row["i_crest_200"] * 1e-3) / KE
        f_rough = w_rough / (2 * np.pi) * N_R
        row["f_seed_rough"] = f_rough
        fs_r, _ = mm.count_cycles(xi, f_rough) if np.isfinite(f_rough) and f_rough > 100 else (np.nan, 0)
        row["spin_rev_s_rough_once"] = fs_r / N_R
        # the rough seed is off by the excess volts (8 % at 100 %), and where the current drops out hundreds of times a
        # second the band-passed noise crosses at the filter's own center, so a count seeded off the line stays off it.
        # The seed is moved to the strongest line within +/-35 % of the rough speed (the 3/rev and 9/rev lines sit at
        # 0.5 and 1.5 times, outside), then the count is re-seeded from its own result until it stops moving
        if np.isfinite(f_rough) and f_rough > 100:
            f_sp = mm.line_freq(xi, 0.65 * f_rough, 1.35 * f_rough)
            row["f_seed_spec"] = f_sp
            fs_r, _ = mm.count_cycles(xi, f_sp)
        for _ in range(8):
            if not np.isfinite(fs_r):
                break
            fn, _ = mm.count_cycles(xi, fs_r)
            done = abs(fn / fs_r - 1) < 1e-4
            fs_r = fn
            if done:
                break
        row["spin_rev_s_rough"] = fs_r / N_R
        row["f_free"] = free_peak(xi)
        for L in (0.2, 0.05):
            fs_, _n = mm.count_cycles(sp["current_raw"].to_numpy(float)[-int(L * FS):], f0)
            row[f"spin_rev_s_{int(L * 1e3)}"] = fs_ / N_R
        row["alpha_pin"] = 2 * np.pi * (row["spin_rev_s"] - row["coast_rev_s"]) / row["t_mid"]
        row["drag_pin_ma"] = J * row["alpha_pin"] / KE * 1e3
        row["drag_slope_ma"] = J * (-row["dedt"] / KE) / KE * 1e3
        ws, ts_, st = [], [], mm.COAST_FROM
        for _ in range(4):
            fw, a0, a1, nw = mm.read_window(eb, st, main["f"] if not ws else ws[-1] * N_R, target_ms=5)
            if not np.isfinite(fw):
                break
            ws.append(fw / N_R); ts_.append((a0 + a1) / 2 / FS); st = a1
        if len(ws) >= 3:
            q = np.polyfit(ts_[:3], ws[:3], 1)
            row["drag_win_ma"] = J * (-2 * np.pi * q[0]) / KE * 1e3
            row["rev_s_cut_win"] = float(np.polyval(q, 0.0))
        out.append(row)
    return out


def _cycle_job(run, folder, C):
    rows = reduce_folder(folder, run)
    for r in rows:
        r.update(k=int(C["k"]), level=float(C["volts_set"]), duty=int(C["duty_pct"]), repeat=int(C["repeat"]),
                 t_start=float(C["t_start"]))
    return rows


def reduce_jobs(runs=("capture-1", "speed-torque", "meter-1", "high-1")):
    """one job per cycle folder of each run, for motor_model.run_parallel"""
    jobs = []
    for run in runs:
        cyc = pd.read_csv(RUNS[run] / "cycles.csv")
        for _, C in cyc.iterrows():
            jobs.append((_cycle_job, (run, RUNS[run] / C["folder"], C.to_dict()), {}))
    return jobs


def reduce_tables(parts):
    """the per-spin tables by run from the jobs' rows, the meter's reading added where the run logged one"""
    out = {}
    for part in parts:
        for r in part:
            out.setdefault(r["run"], []).append(r)
    T = {}
    for run, rows in out.items():
        D = pd.DataFrame(rows)
        log = RUNS[run] / "dmm.csv"
        if log.exists():
            log = pd.read_csv(log)
            cyc = pd.read_csv(RUNS[run] / "cycles.csv")
            mt = {int(C["k"]): mm.meter_spins(log, C["t_start"], C["t_end"]) for _, C in cyc.iterrows()}
            D["meter_ma"] = [mt.get(k, {}).get(d, np.nan) for k, d in zip(D["k"], D["dir"])]
        T[run] = D
    return T


# ---------------------------------------------------------------- the edge term, measured on the spinning motor

def samples(df):
    """every sample's stream, code and ticks from the window's center (the crest), whole periods only"""
    m, ph, sh, ta, tb = bu.phase(df, ARR)
    k = np.arange(len(df))
    phi0 = m["start_cnt"] if m["start_dir"] == 0 else P - m["start_cnt"]
    tick = phi0 + bu.STEP_TICKS * k + bu.APERTURE_CLK * bu.TICKS_PER_ADC_CLK
    n = np.floor((tick - ARR + ARR) / P).astype(int)
    s = pd.DataFrame({"stream": np.where(sh, "sh", np.where(ta, "ta", "tb")), "code": df["code"].to_numpy(float),
                      "tc": tick - ARR - n * P, "n": n})
    return s[(s["n"] > n.min()) & (s["n"] < n.max())], m


def comb_mean(tc, y, step=16):
    """the period mean of one tap: samples into comb bins (the 208-tick stride on a 2400-tick period lands on a 16-tick
    comb), the mean of the bin means, so each phase weighs alike; and the number of bins filled"""
    b = np.floor((tc + ARR) / step).astype(int)
    bm = pd.Series(y).groupby(b).mean()
    return float(bm.mean()), len(bm)


def profile_at(tc, y, at, w=24):
    """the tap's level at one instant: a line through the samples within +/- w ticks"""
    sel = np.abs(tc - at) <= w
    if sel.sum() < 3:
        return np.nan
    p = np.polyfit(tc[sel], y[sel], 1)
    return float(np.polyval(p, at))


SPUN_POINTS = ((5.0, 30), (7.4, 20), (7.4, 40))


def edge_fold():
    """spinning-1's bursts: the period mean of the tap difference against the share construction, per burst, as
    delta = period mean - share x crest difference - (1 - share) x brake level, mV; both from the same fold, so the
    taps' offset cancels"""
    crest = {"low": ST["crest_sample_ticks"]["low_tap"]["value"], "high": ST["crest_sample_ticks"]["high_tap"]["value"]}
    rows = []
    for lv, du in SPUN_POINTS:
        for b, df in enumerate(bu.load(SPUN / f"{lv:.1f}v-{du}-slow")):
            s, m = samples(df)
            sign = int(np.sign(m["step_q15"]))
            half = int(BOARD.drive_ticks(m["step_q15"]))
            rail = BOARD.rail_v(float(df["vbus_raw"].iloc[0])) if "vbus_raw" in df else lv
            on_lag, off_lag = edge_lags(rail)
            sh_ = share_of(half, on_lag, off_lag)
            a = s[s["stream"] == "ta"]; bb = s[s["stream"] == "tb"]
            hi, lo = (a, bb) if sign > 0 else (bb, a)            # the driven tap is A in direction +1
            mh, nh = comb_mean(hi["tc"].to_numpy(), hi["code"].to_numpy())
            ml, nl = comb_mean(lo["tc"].to_numpy(), lo["code"].to_numpy())
            d_mean = (mh - ml) / K_CPV
            d_crest = (profile_at(hi["tc"].to_numpy(), hi["code"].to_numpy(), crest["high"])
                       - profile_at(lo["tc"].to_numpy(), lo["code"].to_numpy(), crest["low"])) / K_CPV
            on, off = -half + on_lag, half + off_lag

            def brake(x):
                t = x["tc"].to_numpy()
                q = np.where(t < on, t + P, t)                     # brake runs from OFF to the next ON, in one piece
                sel = (q > off + 120) & (q < on + P - 120)
                return float(np.median(x["code"].to_numpy()[sel]))
            d_brk = (brake(hi) - brake(lo)) / K_CPV
            rows.append(dict(level=lv, duty=du, burst=b, sign=sign, half=half, rail_v=rail, share=sh_, bins_hi=nh, bins_lo=nl,
                             d_mean=d_mean, d_crest=d_crest, d_brk_rel=d_brk,
                             delta_mv=(d_mean - sh_ * d_crest - (1 - sh_) * d_brk) * 1e3))
    return pd.DataFrame(rows)


def edge_correction(E):
    """the fold's period-mean volts the share construction misses, per rail level, mV: (mean, se, sd)"""
    out = {}
    for lv, g in E.groupby("level"):
        x = g["delta_mv"]
        out[float(lv)] = (float(x.mean()), float(x.std(ddof=1) / np.sqrt(len(x))), float(x.std(ddof=1)))
    allx = E["delta_mv"]
    out["all"] = (float(allx.mean()), float(allx.std(ddof=1) / np.sqrt(len(allx))), float(allx.std(ddof=1)))
    return out


def edge_shape():
    """the driven tap's own edge shape on the spinning motor, every burst of both directions pooled, as a fraction of
    its brake-to-window swing in 4-tick bins from each construction edge; per point the half crossings, the volt-ticks
    against a step at the construction's edge, and the OFF edge's fall with the capacitance a current-limited fall
    at bemf-coast's OFF-edge current would need"""
    out, profiles = [], {}
    for lv, du in SPUN_POINTS:
        rows = []
        for df in bu.load(SPUN / f"{lv:.1f}v-{du}-slow"):
            s, m = samples(df)
            sign = int(np.sign(m["step_q15"])); half = int(BOARD.drive_ticks(m["step_q15"]))
            on_lag, off_lag = edge_lags(lv); on, off = -half + on_lag, half + off_lag
            x = s[s["stream"] == ("ta" if sign > 0 else "tb")]
            t = x["tc"].to_numpy(); y = x["code"].to_numpy()
            q = np.where(t < -half, t + P, t)
            brk = np.median(y[(q > half + 150) & (q < P - half - 150)]); win = np.median(y[(t > on + 60) & (t < off - 60)])
            rows.append(np.c_[t - on, t - off, (y - brk) / (win - brk), np.full(len(t), (win - brk) / K_CPV)])
        A = np.vstack(rows); swing = A[:, 3].mean()
        r = dict(level=lv, duty=du, swing_v=swing)
        for col, lab in ((0, "on"), (1, "off")):
            e = np.arange(-64, 68, 4); idx = np.digitize(A[:, col], e) - 1
            f = np.array([A[idx == i, 2].mean() if (idx == i).any() else np.nan for i in range(len(e) - 1)])
            c = e[:-1] + 2
            profiles[(lv, du, lab)] = (c, f)
            ok = np.isfinite(f)
            step = (c >= 0).astype(float) if lab == "on" else (c < 0).astype(float)
            vt = np.sum((f - step)[ok]) * 4 * swing * (len(c) / ok.sum())
            r[f"{lab}_half_ticks"] = float(np.interp(0.5, f[ok] if lab == "on" else f[ok][::-1], c[ok] if lab == "on" else c[ok][::-1]))
            r[f"{lab}_mv"] = vt / P * 1e3
            if lab == "on":
                r["on_dip"] = float(np.nanmin(f[(c > -40) & (c < 0)]))
            else:
                t70 = np.interp(0.7, f[ok][::-1], c[ok][::-1]); t10 = np.interp(0.1, f[ok][::-1], c[ok][::-1])
                slope = 0.6 * swing / ((t10 - t70) / 48e6)
                i_off = BEMF["spinning_by_point"][f"{lv:g} V {du} %"]["slow"]["off_edge_ma"]["value"] * 1e-3
                r.update(off_fall_ticks=t10 - t70, off_v_per_us=slope / 1e6, off_edge_ma=i_off * 1e3, off_c_nf=i_off / slope * 1e9)
        out.append(r)
    return pd.DataFrame(out), profiles


# ---------------------------------------------------------------- R_run per spin

def e0_noise(T):
    """E0's own per-spin noise as a fraction: the within-(rung, direction) sd of E0 over Ke x the spin's ripple count,
    whose 200 ms count is good to a few hundredths of a percent, so the scatter is E0's extrapolation to the cut"""
    x = T["e_cut"] / (2 * np.pi * T["spin_rev_s"])
    g = x.groupby([T["level"], T["duty"], T["dir"]])
    v = g.var(ddof=1); n = g.size()
    return float(np.sqrt((v * (n - 1)).sum() / (n - 1).sum()) / x.mean())


def drag_noise(T):
    """the pinned drag's own scatter per spin: the within-rung sd of drag_pin over the crest current less its term,
    whose own per-spin noise is a fraction of a percent; pooled over the rungs, as a fraction"""
    x = T["drag_pin_ma"] / T["i_b"]
    g = x.groupby([T["level"], T["duty"], T["dir"]])
    v = g.var(ddof=1); n = g.size()
    return float(np.sqrt((v * (n - 1)).sum() / (n - 1).sum()) / x.mean())


def prepare(T, pre_ms=200, delta=None):
    """R_run per spin by three currents. delta: None for the main read, or a dict level -> mV added to the
    construction"""
    T = T.copy()
    cs, _ = c_scale()
    T["v_on"] = T[f"v_on_{pre_ms}"]
    T["i_crest"] = T[f"i_crest_{pre_ms}"]
    lags = np.array([edge_lags(lv) for lv in T["level"]])
    T["on_lag"], T["off_lag"] = lags[:, 0], lags[:, 1]
    T["full"] = T["duty"] >= 100
    T["share"] = np.where(T["full"], 1.0, share_of(T["half_ticks"], T["on_lag"], T["off_lag"]))
    T["delta_mv"] = 0.0 if delta is None else [0.0 if f else delta[lv] for lv, f in zip(T["level"], T["full"])]
    tm = [term_ma(lv, du) for lv, du in zip(T["level"], T["duty"])]
    T["term"], T["term_err"] = [t[0] for t in tm], [t[1] for t in tm]
    T["i_a"] = cs * T["drag_pin_ma"]
    T["i_b"] = T["i_crest"] - T["term"]
    T["i_w"] = T["drag_win_ma"]
    for r in ("a", "b", "w"):
        i = T[f"i_{r}"] * 1e-3
        T[f"v_mean_{r}"] = T["share"] * T["v_on"] - (1 - T["share"]) * R_BRK * i + T["delta_mv"] * 1e-3
        T[f"R_{r}"] = (T[f"v_mean_{r}"] - T["e_cut"]) / i
    ib = T["i_b"] * 1e-3
    T["err_E_ohm"] = e0_noise(T) * T["e_cut"] / ib                               # E0's own per-spin noise
    alts = T[["e_cut", "e_cut_5ms", "e_cut_20ms", "e_cut_quad20", "e_cut_from2ms"]].to_numpy()
    T["e_alt_sd"] = alts.std(axis=1, ddof=1)
    T["sys_Ealt_ohm"] = T["e_alt_sd"] / ib                                        # the E0 read's spread over its window choices
    T["err_V_ohm"] = T["share"] * T["v_on_se_200"] / ib
    T["err_ib_ohm"] = T["R_b"].abs() * (T["i_crest_se_200"] / T["i_b"])
    return T


def per_spin(raw):
    """capture-1's R_run per spin with the per-spin budget, the rung-common systematics and the excess over the
    locked R as volts"""
    cs, csd = c_scale()
    C1 = prepare(raw)
    dn = drag_noise(C1)
    C1["err_ia_ohm"] = C1["R_a"].abs() * dn
    C1["err_a_ohm"] = np.sqrt(C1["err_E_ohm"] ** 2 + C1["err_V_ohm"] ** 2 + C1["err_ia_ohm"] ** 2)
    C1["err_b_ohm"] = np.sqrt(C1["err_E_ohm"] ** 2 + C1["err_V_ohm"] ** 2 + C1["err_ib_ohm"] ** 2)
    i_b = C1["i_b"] * 1e-3
    C1["sys_delta_ohm"] = np.where(C1["full"], 0.0, DELTA_SYS_MV * 1e-3 / i_b)
    C1["sys_zero_ohm"] = (1 - C1["share"]) * ZERO_SYS_COUNTS / K_CPV / i_b
    C1["sys_brk_ohm"] = (1 - C1["share"]) * R_BRK_ERR
    C1["sys_term_ohm"] = C1["R_b"].abs() * np.hypot(C1["term_err"], DRIVE_OFF_ERR / CPA * 1e3) / C1["i_b"]
    C1["sys_scale_a_ohm"] = C1["R_a"].abs() * csd / cs / np.sqrt(30)                 # the coast scale's se over its 30 spins
    C1["sys_chain_ohm"] = C1["R_b"].abs() * TERM_ERR_PCT / 100
    C1["edge_ohm_per_tick"] = np.where(C1["full"], 0.0, C1["v_on"] / P / i_b)
    C1["excess_mv_a"] = (C1["v_mean_a"] - C1["e_cut"] - R_LOCKED * C1["i_a"] * 1e-3) * 1e3
    C1["excess_mv_b"] = (C1["v_mean_b"] - C1["e_cut"] - R_LOCKED * C1["i_b"] * 1e-3) * 1e3
    return C1, dn


def rung_table(T, col, err):
    rows = []
    for (lv, du), g in T.groupby(["level", "duty"]):
        rows.append(dict(level=lv, duty=du, n=len(g), rev_s=g["spin_rev_s"].mean(), i_a=g["i_a"].mean(), i_b=g["i_b"].mean(),
                         mean=g[col].mean(), sd=g[col].std(ddof=1), se=g[col].std(ddof=1) / np.sqrt(len(g)),
                         plus=g[g["dir"] > 0][col].mean(), minus=g[g["dir"] < 0][col].mean(),
                         budget=np.sqrt((g[err] ** 2).mean())))
    R = pd.DataFrame(rows)
    R["sd_over_budget"] = R["sd"] / R["budget"]
    return R


def wls(x, y, w=None):
    """y = a + b x by least squares: with weights (1/variance) the covariance is scaled by the reduced chi2 when it
    is over 1; without, by the reduced chi2 itself"""
    x, y = np.asarray(x, float), np.asarray(y, float)
    unweighted = w is None
    w = np.ones_like(x) if w is None else np.asarray(w, float)
    X = np.c_[np.ones_like(x), x]
    cov = np.linalg.inv(X.T @ (w[:, None] * X))
    beta = cov @ X.T @ (w * y)
    res = y - X @ beta
    dof = max(len(y) - 2, 1)
    chi2 = float(res @ (w * res) / dof)
    covs = cov * (chi2 if unweighted else max(chi2, 1.0))
    se = np.sqrt(np.diag(covs))
    return dict(a=beta[0], b=beta[1], se_a=se[0], se_b=se[1], chi2=chi2, res=res, dof=dof)


def scatter_vs_budget(tab):
    """the pooled within-rung sd against the rms per-spin budget"""
    psd = np.sqrt((tab["sd"] ** 2 * (tab["n"] - 1)).sum() / (tab["n"] - 1).sum())
    bud = np.sqrt((tab["budget"] ** 2).mean())
    return float(psd), float(bud), float(psd / bud)


def matched_pairs(T, col):
    """the rung pairs of the two rails that turn at about the same speed: 7.4 V less 5.0 V in R and as mV of V - E at
    the pair's current; the weighted mean over the pairs in mV"""
    out = []
    for (lv1, du1), (lv2, du2) in (((7.4, 20), (5.0, 30)), ((7.4, 30), (5.0, 40)), ((7.4, 40), (5.0, 60))):
        a = T[(T["level"] == lv1) & (T["duty"] == du1)]; b = T[(T["level"] == lv2) & (T["duty"] == du2)]
        d = a[col].mean() - b[col].mean()
        se = np.hypot(a[col].std(ddof=1) / np.sqrt(len(a)), b[col].std(ddof=1) / np.sqrt(len(b)))
        i = 0.5 * (a["i_a"].mean() + b["i_a"].mean()) * 1e-3
        out.append(dict(pair=f"{lv1:g} V {du1} % - {lv2:g} V {du2} %", rev_s_a=a["spin_rev_s"].mean(), rev_s_b=b["spin_rev_s"].mean(),
                        d_ohm=d, se_ohm=se, d_mv=d * i * 1e3, se_mv=se * i * 1e3))
    D = pd.DataFrame(out)
    w = 1 / D["se_mv"] ** 2
    return D, float((w * D["d_mv"]).sum() / w.sum()), float(1 / np.sqrt(w.sum()))


def pooled_low(S, r):
    """the pooled running R under LOW rev/s for route r: weighted by the rungs' se with the scatter folded in, the
    rung-common systematics, and the total"""
    lo = S[S["spin_rev_s"] < LOW]
    _, csd_ = c_scale()
    g = lo.groupby(["level", "duty"])[f"R_{r}"]
    m, se = g.mean(), g.std(ddof=1) / np.sqrt(g.size())
    w = 1 / se ** 2
    pm = float((w * m).sum() / w.sum())
    chi2 = float((w * (m - pm) ** 2).sum() / (len(m) - 1))
    stat = float(1 / np.sqrt(w.sum()) * np.sqrt(max(chi2, 1)))
    sysr = float(np.sqrt(((lo["sys_delta_ohm"] ** 2).mean() + (lo["sys_zero_ohm"] ** 2).mean() + (lo["sys_brk_ohm"] ** 2).mean()
                          + ((lo["sys_term_ohm"] ** 2).mean() if r == "b" else (lo["sys_scale_a_ohm"] ** 2).mean() + (pm * 0.008) ** 2)
                          + (lo["sys_chain_ohm"] ** 2).mean())))
    return dict(rungs=m, pooled=pm, stat=stat, chi2=chi2, sys=sysr, total=float(np.hypot(stat, sysr)))


def slope_low(S, r, lim, with_sys=True):
    """the chord's slope with speed under `lim` rev/s over the rung means, weighted by their se, and with each rung's
    own term (route b) and E0-window systematics added to its error"""
    lo = S[S["spin_rev_s"] < lim]
    g = lo.groupby(["level", "duty"])
    t = pd.DataFrame(dict(w=g["spin_rev_s"].mean(), m=g[f"R_{r}"].mean(), se=g[f"R_{r}"].std(ddof=1) / np.sqrt(g.size()),
                          sysr=g.apply(lambda h: np.sqrt((h["sys_term_ohm"] ** 2).mean() * (r == "b") + (h["sys_Ealt_ohm"] ** 2).mean()),
                                       include_groups=False)))
    err = np.hypot(t["se"], t["sysr"]) if with_sys else t["se"]
    f = wls(t["w"], t["m"], 1 / err ** 2)
    f.update(pct=f["b"] * 100 / t["m"].mean() * 100, pct_se=f["se_b"] * 100 / t["m"].mean() * 100, resid=f["res"] / err, t=t)
    return f


def budget_table(S):
    """one coast reading's budget per rung, route b, as % of the rung's R"""
    rows = []
    for (lv, du), g in S.groupby(["level", "duty"]):
        R = g["R_b"].mean()
        pc = lambda c: float(np.sqrt((g[c] ** 2).mean()) / R * 100)
        rnd = np.sqrt(pc("err_E_ohm") ** 2 + pc("err_V_ohm") ** 2 + pc("err_ib_ohm") ** 2)
        sysb = np.sqrt(sum(pc(c) ** 2 for c in ("sys_delta_ohm", "sys_zero_ohm", "sys_brk_ohm", "sys_term_ohm", "sys_chain_ohm", "sys_Ealt_ohm")))
        rows.append(dict(level=lv, duty=du, rev_s=g["spin_rev_s"].mean(), R_b=R, E0_pct=pc("err_E_ohm"), V_pct=pc("err_V_ohm"), ib_pct=pc("err_ib_ohm"),
                         drag_pct=pc("err_ia_ohm"), random_b_pct=rnd, seen_sd_pct=g["R_b"].std(ddof=1) / R * 100,
                         delta_pct=pc("sys_delta_ohm"), zero_pct=pc("sys_zero_ohm"), brk_pct=pc("sys_brk_ohm"), term_pct=pc("sys_term_ohm"),
                         Ealt_pct=pc("sys_Ealt_ohm"), chain_pct=pc("sys_chain_ohm"), sys_b_pct=sysb))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- the 100 % run-ups

def full_duty(ST_, e0_rel):
    """speed-torque's run-ups at 100 %: (V - E0)/i over the last 50 ms with nothing modeled, its alternative reads,
    the per-run-up budget and the excess over the locked R"""
    T = ST_.copy()
    T["w"] = 2 * np.pi * T["spin_rev_s"]
    for pre in (100, 50, 20, 10):
        T[f"R_{pre}"] = (T[f"v_on_{pre}"] - T["e_cut"]) / (T[f"i_crest_{pre}"] * 1e-3)
    T["i"] = T["i_crest_50"]
    T["v"] = T["v_on_50"]
    T["R"] = T["R_50"]
    T["ke_cut"] = T["e_cut"] / T["w"]
    T["excess_mv"] = (T["v"] - T["e_cut"] - R_LOCKED * T["i"] * 1e-3) * 1e3
    T["R_drag_win"] = (T["v"] - T["e_cut"]) / (T["drag_win_ma"] * 1e-3)
    T["R_quad20"] = (T["v"] - T["e_cut_quad20"]) / (T["i"] * 1e-3)
    T["R_first1"] = (T["v"] - T["e_first1_fit"]) / (T["i"] * 1e-3)
    T["R_ripple"] = (T["v"] - KE * 2 * np.pi * T["spin_rev_s_rough"]) / (T["i"] * 1e-3)
    term_err = half_range(CARRIED["full_duty_term_ma"])
    T["err_E"] = e0_rel * T["e_cut"] / (T["i"] * 1e-3)
    T["err_i"] = T["R"] * np.hypot(T["i_crest_se_200"] * 2, term_err) / T["i"]     # 10 ms blocks: 20 in 200 ms, 5 in 50 ms
    T["err"] = np.hypot(T["err_E"], T["err_i"])
    return T, term_err


def open_share(x_v, r, i):
    """the share f of the time a path must stand open, the drive held, to add x_v over r x i: V - E = R i / (1 - f),
    so x = R i f / (1 - f) and f = q / (1 + q) with q = x / (R i)"""
    q = x_v / (r * i)
    return q / (1 + q)


def arc_mv(l_h, i_a, rate_per_s):
    """the stored energy dumped at each opening, 1/2 L i^2, times the openings per second, as volts at the current i:
    P / i = 1/2 L i rate, in mV"""
    return 0.5 * l_h * i_a * rate_per_s * 1e3


def gap_hz(rev_s, segments=MOTOR.slots):
    """the commutator gaps one brush crosses per second, one per segment per revolution"""
    return segments * np.asarray(rev_s, float)


# ---------------------------------------------------------------- speed against current: the run-ups

ST_NB = results.ROOT / "stages" / "2-motor" / "speed-torque"
# imports and constants, cycles, results carried in, the run-ups, the dropout rows, the estimators, the count, by cell id
ST_CELLS = ("fc44883f", "98a52b0f", "61357cd8", "4c4b548a", "64a4b80c", "71c11876", "1331f6dd")


def runup_windows():
    """speed-torque's turn-window table and its runs, by executing the notebook's own estimator cells unchanged in its
    folder, with its printing, tables and figures kept out of this notebook"""
    import matplotlib.pyplot as plt
    nb = json.loads((ST_NB / "speed-torque.ipynb").read_text())
    here = os.getcwd()
    figs = set(plt.get_fignums())
    show0 = plt.show
    plt.show = lambda *a, **k: None
    g = {"__name__": "speed_torque_cells"}
    try:
        os.chdir(ST_NB)
        with contextlib.redirect_stdout(io.StringIO()), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cells = {c["id"]: c for c in nb["cells"]}
            for c in ST_CELLS:
                exec("".join(cells[c]["source"]), g)
                if c == ST_CELLS[0]:
                    g["show"] = lambda *a, **k: None
    finally:
        os.chdir(here)
        plt.show = show0
        for n in set(plt.get_fignums()) - figs:
            plt.close(n)
    return g["REV"].copy(), g["RUNS"], g["L_H"], g["T_OFFSET"], g["V_PER_COUNT"], g["SETTLED"]


def x_free(C1, ST_):
    """the free spins' excess V - E0 - R_locked i by direction against speed, with their current and V - E0:
    capture-1's rung means (route b) and the 100 % settled ends"""
    rows = []
    for (lv, du, d), g in C1.groupby(["level", "duty", "dir"]):
        rows.append(dict(src=f"{lv:g} V {du} %", dir=d, rev_s=g["spin_rev_s"].mean(), i=g["i_b"].mean() * 1e-3,
                         x=(g["v_mean_b"] - g["e_cut"] - R_LOCKED * g["i_b"] * 1e-3).mean(), ve=(g["v_mean_b"] - g["e_cut"]).mean()))
    for d, g in ST_.groupby("dir"):
        rows.append(dict(src="100 %", dir=d, rev_s=g["spin_rev_s"].mean(), i=g["i_crest_50"].mean() * 1e-3,
                         x=(g["v_on_50"] - g["e_cut"] - R_LOCKED * g["i_crest_50"] * 1e-3).mean(), ve=(g["v_on_50"] - g["e_cut"]).mean()))
    return pd.DataFrame(rows).sort_values(["dir", "rev_s"]).reset_index(drop=True)


def interp_dir(X, d, rev_s, col):
    h = X[X["dir"] == d].sort_values("rev_s")
    return np.interp(rev_s, h["rev_s"], h[col], left=np.nan, right=np.nan)


def runup_y(REV, RUNS, L_H, T_OFF, VPC):
    """speed-torque's y = u - L di/dt - Ke w per turn window, u put on the rest zero so V - E matches the coast's"""
    zero = {(r["k"], r["dir"]): r["zero_e"] for r in RUNS}
    REV = REV.copy()
    REV["u_rest"] = REV["u"] + (T_OFF - np.array([zero[(k, d)] for k, d in zip(REV["k"], REV["dir"])])) * VPC
    REV["y"] = REV["u_rest"] - L_H * REV["didt"] - KE * REV["w"]
    REV["rev_s"] = REV["w"] / (2 * np.pi)
    return REV, float(np.mean(list(zero.values())))


def fit_r(yy, ii):
    """one R through the origin, least squares, and the residuals"""
    r = float(np.sum(yy * ii) / np.sum(ii * ii))
    return r, yy - r * ii


def runup_models(RU):
    """the three readings of the excess in the run-up windows, each with one free R"""
    out = {}
    for lab, ex in (("A", RU["xf"]), ("B", RU["xf"] * RU["i"] / RU["if"]), ("0", 0.0 * RU["xf"])):
        out[lab] = fit_r((RU["y"] - ex).to_numpy(), RU["i"].to_numpy())
    return out


def window_scatter(RU):
    """the windows' own scatter: neighboring windows overlap five turns in six, so the rms of the difference of
    windows six apart (independent turns) over root two, in each run-up"""
    d6 = []
    for _, h in RU.groupby(["k", "dir"]):
        y = h["y"].to_numpy()
        if len(y) > 7:
            d6.append(np.diff(y[::6]))
    d6 = np.concatenate(d6)
    return float(d6.std() / np.sqrt(2))


def runup_bins(RU, R0):
    """y - R0 i in 40 rev/s bins from 200 rev/s, against the free spins' excess and model B's prediction"""
    xr = RU["y"] - R0 * RU["i"]
    rows = []
    for a in np.arange(200, 520, 40):
        sel = (RU["rev_s"] >= a) & (RU["rev_s"] < a + 40)
        h = RU[sel]
        if len(h) < 5:
            continue
        rows.append(dict(rev_s=h["rev_s"].mean(), n=len(h), i_ma=h["i"].mean() * 1e3, runup_excess_mv=xr[sel].mean() * 1e3,
                         sd_mv=xr[sel].std() * 1e3, free_excess_mv=h["xf"].mean() * 1e3, free_i_ma=h["if"].mean() * 1e3,
                         B_predicts_mv=(h["xf"] * h["i"] / h["if"]).mean() * 1e3))
    return pd.DataFrame(rows)


def incremental_r(RU, X):
    """(y_runup - (V - E)_free) / (i_runup - i_free) at the same speed and direction, per run-up and per 40 rev/s band;
    no R is assumed"""
    RU = RU.copy()
    RU["ve_f"] = [interp_dir(X, d, w, "ve") for d, w in zip(RU["dir"], RU["rev_s"])]
    RU["dy"] = RU["y"] - RU["ve_f"]
    RU["di"] = RU["i"] - RU["if"]
    per = pd.DataFrame([dict(k=k, dir=d, n=len(h), r_inc=float(np.sum(h["dy"] * h["di"]) / np.sum(h["di"] ** 2)))
                        for (k, d), h in RU.groupby(["k", "dir"])])
    rows = []
    for a in np.arange(180, 560, 40):
        h = RU[(RU["rev_s"] >= a) & (RU["rev_s"] < a + 40)]
        if len(h) < 10:
            continue
        rr = [float(np.sum(g["dy"] * g["di"]) / np.sum(g["di"] ** 2)) for _, g in h.groupby(["k", "dir"]) if len(g) >= 3]
        rows.append(dict(band=f"{a:.0f}-{a + 40:.0f}", lo=a, n=len(h), i_runup_ma=h["i"].mean() * 1e3, i_free_ma=h["if"].mean() * 1e3,
                         r_inc=float(np.sum(h["dy"] * h["di"]) / np.sum(h["di"] ** 2)), runups=len(rr),
                         se=float(np.std(rr, ddof=1) / np.sqrt(len(rr))) if len(rr) > 1 else np.nan))
    return per, pd.DataFrame(rows), RU


def step_read(l_h, t_s, r=R_LOCKED):
    """what a duty-step read of dV/di at t_s after the step reads in units of R, with neither L di/dt nor the EMF's
    rise taken out: the current approaches dV/R with L/R, the EMF rises with J R / Ke^2 and pulls the current back"""
    te, tm = l_h / r, J * r / KE ** 2
    return 1 / ((1 - np.exp(-t_s / te)) - (1 - np.exp(-t_s / tm)))


# ---------------------------------------------------------------- the excess curve

def excess_by_rung(S, ST_):
    """the excess X = V_mean - E0 - R_locked i per rung and direction (route b, and route a), with the dropout rate
    and the observer's speed bias X / E0; the 100 % ends added"""
    rows = []
    for (lv, du, d), g in S.groupby(["level", "duty", "dir"]):
        rows.append(dict(src=f"{lv:g} V {du} %", dir=d, rev_s=g["spin_rev_s"].mean(), i_ma=g["i_b"].mean(), x_b=g["excess_mv_b"].mean(),
                         x_a=g["excess_mv_a"].mean(), x_se=g["excess_mv_b"].std(ddof=1) / np.sqrt(len(g)), dropouts=g["dropouts_per_s"].mean(),
                         bias_pct=(g["excess_mv_b"] / (g["e_cut"] * 1e3)).mean() * 100))
    for d, g in ST_.groupby("dir"):
        x = (g["v_on_50"] - g["e_cut"] - R_LOCKED * g["i_crest_50"] * 1e-3) * 1e3
        rows.append(dict(src="100 %", dir=d, rev_s=g["spin_rev_s"].mean(), i_ma=g["i_crest_50"].mean(), x_b=x.mean(), x_a=np.nan,
                         x_se=x.std(ddof=1) / np.sqrt(len(x)), dropouts=g["dropouts_per_s"].mean(), bias_pct=(x / (g["e_cut"] * 1e3)).mean() * 100))
    return pd.DataFrame(rows).sort_values(["dir", "rev_s"]).reset_index(drop=True)


def power_law(h):
    """X = A (w / 300 rev/s)^n over rungs above 200 rev/s, a line in logs; A in mV, n with its se, and the speed where
    X reaches 1 % of the EMF"""
    f = wls(np.log(h["rev_s"] / 300), np.log(h["x_b"]))
    A, n = float(np.exp(f["a"])), float(f["b"])
    w = np.linspace(150, 560, 400)
    one = float(np.interp(0.01, (A * (w / 300) ** n) / (KE * 2 * np.pi * w * 1e3), w))
    return dict(mv_300=A, exponent=n, exponent_se=float(f["se_b"]), a_se=float(f["se_a"]), one_pct_rev_s=one)


def v_b(rev_s, mv_300, exponent):
    """the brush drop's power law, mV"""
    return mv_300 * (np.asarray(rev_s, float) / 300.0) ** exponent


# ---------------------------------------------------------------- the meter runs, by differences

def meter_points(T_by_run, Xs):
    """the meter runs per point: the loop R by the meter's own current, and the meter's R by two pairings, the plain
    loop less the locked R, and the loop less the locked R less capture-1's excess at the point's own speed"""
    rows = []
    for run in ("meter-1", "high-1"):
        T = T_by_run[run]
        for (lv, du), g in T.groupby(["level", "duty"]):
            s = 1.0 if du >= 100 else float(share_of(g["half_ticks"].mean(), *edge_lags(lv)))
            i = g["meter_ma"] * 1e-3
            vm = s * g["v_on_200"] - (1 - s) * R_BRK * i
            ve = vm - g["e_cut"]
            r_loop = ve / i
            w = g["spin_rev_s"].mean()
            xc = float(np.interp(w, Xs["rev_s"], Xs["x"]))
            rows.append(dict(run=run, level=lv, duty=du, n=len(g), rev_s=w, meter_ma=i.mean() * 1e3, board_ma=g["i_crest_200"].mean(),
                             ve_v=ve.mean(), r_loop=r_loop.mean(), r_loop_se=r_loop.std(ddof=1) / np.sqrt(len(g)),
                             x_c_at_w_mv=xc * 1e3, r_meter_vb=float(((ve - xc) / i).mean() - R_LOCKED),
                             r_meter_plain=float(r_loop.mean() - R_LOCKED), dropouts=g["dropouts_per_s"].mean()))
    return pd.DataFrame(rows).sort_values("rev_s").reset_index(drop=True)


# ---------------------------------------------------------------- the EMF at the cut

def phase_fit(ph, y):
    """y = c + A cos(2 pi ph) + B sin(2 pi ph): the 1/cycle amplitude, its se, the residual sd"""
    X = np.c_[np.ones_like(ph), np.cos(2 * np.pi * ph), np.sin(2 * np.pi * ph)]
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    res = y - X @ beta
    s2 = res @ res / max(len(y) - 3, 1)
    cov = np.linalg.inv(X.T @ X) * s2
    amp = np.hypot(beta[1], beta[2])
    se = np.sqrt((beta[1] ** 2 * cov[1, 1] + beta[2] ** 2 * cov[2, 2]) / max(amp ** 2, 1e-30))
    return amp, se, np.sqrt(s2)
