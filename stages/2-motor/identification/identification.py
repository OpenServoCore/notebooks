"""identification's machinery: the board-only routes to the constants a BEMF speed estimate needs.

The routes read only what a servo's own board can read, the duty, the terminal taps, the crest current sample, the
rail tap and the board NTC, and are graded against stage 2's bench references. Every measured constant comes through
the results files, mostly through motor-model's motor_model.py and brush_drop.py, whose per-spin reduction this module
widens to the runs it needs. Two notebooks' own reductions are not modules, rest-resistance's lap table and
inductance's burst fits, so `notebook_cells` runs their cells unchanged in their own folder, as brush_drop does for
speed-torque's run-up windows.

The pieces a later notebook will also need live here, so it can import them: the per-spin reductions (`reductions`),
the published reading-term table f(D) and the share construction (`f_table_published`, `signals`), the V_b fit and its
evaluation (`vb_measured`, `rung_means`, `fit_vb`, `vb_eval`), the ripple tach's gate (`tach_gate`), the mechanical
direction (`MECH`, `tag_mech`) and the 100 % anchor (`cut_slopes`, `alpha_reads`, `c_per_cut`, `anchor_delta`,
`f_from`).
"""
import contextlib
import functools
import itertools
import json
import math
import os
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import results, tel

_MM_DIR = results.ROOT / "stages" / "2-motor" / "motor-model"
if str(_MM_DIR) not in sys.path:
    sys.path.insert(0, str(_MM_DIR))
import motor_model as mm  # noqa: E402
import brush_drop as bd  # noqa: E402

BOARD, MOTOR = mm.BOARD, mm.MOTOR
MM = results.read("2-motor", "motor-model", pair=MOTOR.key)
# motor-model's spinning results, handed to brush_drop in the results files' form, as motor-model's notebook does
bd.carry(coast_over_meter=MM["coast_over_meter"], crest_less_mean_ma=MM["crest_less_mean_ma"],
         full_duty_term_ma=MM["full_duty_term_ma"])

RR, BEMF, IND, SPT, TD = mm.RR, mm.BEMF, bd.IND, mm.SPT, mm.TD
KE, J, FS, N_R = mm.KE, mm.J, mm.FS, mm.N_R
KE_REV = 2 * np.pi * KE                     # V per rev/s
R_LOCKED = bd.R_LOCKED                      # rest-resistance's 12-hole median at the stall pass
R_BRK = bd.R_BRK                            # bridge-states' low-side loop, the slow-decay brake
NB2 = results.ROOT / "stages" / "2-motor"
PAIR = mm.PAIR
SWAP = PAIR / "bemf-coast" / "swapped-1"

# A servo's travel at the motor. The gear ratio is counted from the teeth of an MG90 clone's metal train (pinion 9 into
# 48, 12 into 48, 10 into 38, 10 into 38), a parts count like the motor registry's. The horn's travel is taken as
# 184 deg, about what an MG90 clone's end stops allow.
GEAR = (48 * 48 * 38 * 38) / (9 * 12 * 10 * 10)
TRAVEL_DEG = 184.0
TRAVEL_REVS = TRAVEL_DEG / 360.0 * GEAR

ALPHA_CU = 0.0039                           # copper's resistance coefficient, per C, a physical constant

# mechanical direction: +1 is the rotor direction that opens more (capture-1's board +1). capture-1 and speed-torque
# ran black on MOT_A by bemf-coast's inference (moderate confidence), meter-1, high-1 and swapped-1's wired block
# white on MOT_A, swapped-1's swapped block black on MOT_A. MECH maps the board's direction to the mechanical one.
MECH = {"capture-1": 1, "speed-torque": 1, "meter-1": -1, "high-1": -1, "swapped-wired": -1, "swapped-swapped": 1}

# the published reading-term grid's rails and duties, and the V_b speed table's knots
GRID_RAILS = (3.7, 5.0, 7.4, 8.4)
GRID_DUTIES = (10, 15, 20, 30, 40, 50, 60, 70, 80, 90)
VB_KNOTS = np.arange(150.0, 651.0, 50.0)

# the fewest capture-1 rungs that hold the V_b table, both directions each (the analysis' held-out rule)
MIN_RUNGS = [(7.4, 30), (7.4, 40), (7.4, 60)]
# capture-1's four low rungs, the candidates for a reference rung
LOW_RUNGS = [(5.0, 15), (5.0, 20), (7.4, 15), (7.4, 20)]

# the cells of the two notebooks whose reductions are not modules, by cell id: rest-resistance's imports, constants,
# laps, lap rows, lap lines, hole table, held-out checks, direction gap and one-body map (cells 1 to 22), and
# inductance's imports, group order, bursts, samples, periods, edges, fits and summaries (cells 1 to 15)
RR_CELLS = ("3aea8cd3", "0378138d", "752e97fd", "0fc8d828", "8ab61487", "3fd037c5", "00aa4337", "c5dd376b",
            "490b386d", "5c2c5851", "19018c53")
IND_CELLS = ("8d7dd008", "e2f9a5ac", "889d5b45", "28eb3ab7", "22ae159d", "4ef8aaa5", "255fa2d7", "6611214d")


def pct(x, ref):
    """x over ref, less one, in %"""
    return (np.asarray(x, float) / ref - 1) * 100


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.nanmean(x ** 2)))


def in_rungs(T, cells):
    """rows of T whose (level, duty) is one of cells, through a tuple Series' isin"""
    return T[["level", "duty"]].apply(tuple, axis=1).isin(cells)


def kept(label, df, cols):
    """one line of what a row selection kept, by the given columns"""
    g = df.groupby(cols).size()
    return f"[{label}] kept {len(df)} rows: " + ", ".join(f"{k}:{v}" for k, v in g.items())


# ---------------------------------------------------------------- another notebook's cells, run unchanged

def notebook_cells(nb, ids):
    """run the named code cells of stages/2-motor/<nb>/<nb>.ipynb in a fresh namespace, in the notebook's folder,
    with its printing, tables and figures kept out of the caller; returns the namespace. Read-only on the repo."""
    import matplotlib.pyplot as plt
    from IPython.utils.capture import capture_output
    path = NB2 / nb / f"{nb}.ipynb"
    cells = {c.get("id"): c for c in json.loads(path.read_text())["cells"]}
    here = os.getcwd()
    figs = set(plt.get_fignums())
    show0 = plt.show
    plt.show = lambda *a, **k: None
    ns = {"__name__": f"{nb.replace('-', '_')}_cells"}
    try:
        os.chdir(path.parent)
        sys.path.insert(0, str(path.parent))
        with capture_output(), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            for c in ids:
                exec(compile("".join(cells[c]["source"]), f"{nb}.ipynb cell {c}", "exec"), ns)
                for n in set(plt.get_fignums()) - figs:
                    plt.close(n)
    finally:
        os.chdir(here)
        sys.path.remove(str(path.parent))
        plt.show = show0
        for n in set(plt.get_fignums()) - figs:
            plt.close(n)
    return ns


@functools.lru_cache(maxsize=None)
def rest_resistance_cells():
    return notebook_cells("rest-resistance", RR_CELLS)


@functools.lru_cache(maxsize=None)
def inductance_cells():
    return notebook_cells("inductance", IND_CELLS)


@functools.lru_cache(maxsize=None)
def runup_windows():
    """speed-torque's turn-window table, through brush_drop (speed-torque's own cells)"""
    return bd.runup_windows()


# ---------------------------------------------------------------- the per-spin reductions

def drive_slope_folder(folder, run, seeds):
    """per spin of one folder: the ripple tach in successive sub-windows before the cut, a line through them, its slope
    in rev/s per s at the cut and the speed at the cut. seeds: {dir: rev_s} from the spin's own count"""
    df, meta = tel.load(folder, BOARD)
    out = []
    for d, sp, co in mm.spin_coast_pairs(df):
        x = sp["current_raw"].to_numpy(float)
        f0 = seeds.get(d, np.nan) * N_R
        row = dict(run=run, folder=str(results.rel(folder)), dir=d)
        for span_ms, nwin in ((100, 5), (50, 5), (200, 8)):
            n = int(span_ms / 1e3 * FS)
            if not np.isfinite(f0) or len(x) < n:
                row[f"slope_{span_ms}"] = np.nan
                continue
            seg = x[-n:]
            w = n // nwin
            ts, fs = [], []
            for j in range(nwin):
                fj, nc = mm.count_cycles(seg[j * w:(j + 1) * w], f0)
                if np.isfinite(fj):
                    ts.append(((j + 0.5) * w - n) / FS)
                    fs.append(fj / N_R)
            if len(fs) >= 3:
                p = np.polyfit(ts, fs, 1)
                row[f"slope_{span_ms}"] = float(p[0])
                row[f"w_cut_{span_ms}"] = float(np.polyval(p, 0.0))
            else:
                row[f"slope_{span_ms}"] = np.nan
        out.append(row)
    return out


def _swap_jobs():
    jobs = []
    for block in ("wired", "swapped"):
        cyc = pd.read_csv(SWAP / f"{block}.csv")
        for _, C in cyc.iterrows():
            jobs.append((bd._cycle_job, (f"swapped-{block}", SWAP / C["folder"], C.to_dict()), {}))
    return jobs


@functools.lru_cache(maxsize=None)
def reductions():
    """brush_drop's per-spin reduction of capture-1, speed-torque, meter-1 and high-1, and of swapped-1's two blocks
    (TB, by run), and the drive-side tach slopes before every cut (SL)"""
    jobs = bd.reduce_jobs() + _swap_jobs()
    parts = mm.run_parallel(jobs)
    n_main = len(bd.reduce_jobs())
    TB = bd.reduce_tables(parts[:n_main])
    sw = {}
    for part in parts[n_main:]:
        for r in part:
            sw.setdefault(r["run"], []).append(r)
    for k, v in sw.items():
        TB[k] = pd.DataFrame(v)
    sjobs = []
    for run, T in TB.items():
        for folder, g in T.groupby("folder"):
            seeds = {int(r["dir"]): float(r["spin_rev_s"]) for _, r in g.iterrows() if np.isfinite(r.get("spin_rev_s", np.nan))}
            sjobs.append((drive_slope_folder, (results.ROOT / folder, run, seeds), {}))
    SL = pd.DataFrame([r for part in mm.run_parallel(sjobs) for r in part])
    return dict(TB=TB, SL=SL)


def tag_mech(T, run):
    """T with its mechanical direction, the board's direction times the run's lead orientation"""
    T = T.copy()
    T["mech"] = T["dir"] * MECH[run]
    return T


def tach_gate(T):
    """the ripple tach is a truth where its two seeds (the coast's line, a rough EMF speed + spectral step) agree to
    0.3 % and, above 150 rev/s, it sits within 1 % of the coast's own speed at the cut (a line through three 5 ms
    coast windows); returns the mask and a table of what it drops"""
    q = (T["spin_rev_s_rough"] / T["spin_rev_s"] - 1).abs() * 100
    c = (T["spin_rev_s"] / T["rev_s_cut_win"] - 1) * 100
    ok = (q < 0.3) & ((T["spin_rev_s"] < 150) | (c.abs() < 1.0))
    return ok, T.loc[~ok, ["k", "dir", "duty", "spin_rev_s", "spin_rev_s_rough", "rev_s_cut_win"]].assign(
        seed_diff_pct=q[~ok], tach_less_coast_pct=c[~ok])


# ---------------------------------------------------------------- the reading term and the share construction

def f_table_published():
    """f(D) in mA/V by duty: motor-model's published no-meter crest term over its rail, the mean over the four rails,
    held flat below the lowest duty and zero at 100 % (nothing switches there). Returns (duties, values)."""
    grid = MM["crest_less_mean_ma"]
    f = pd.Series({du: np.mean([grid[f"{v:g} V"][f"{du} %"]["value"] / v for v in GRID_RAILS]) for du in GRID_DUTIES})
    d = np.r_[0.0, f.index.to_numpy(float), 100.0]
    v = np.r_[f.iloc[0], f.to_numpy(), 0.0]
    return d, v


def delta_ma(duty_pct, v_on, ftab):
    """the reading term Delta = f(D) x v_window, mA"""
    d, v = ftab
    return np.interp(np.asarray(duty_pct, float), d, v) * np.asarray(v_on, float)


def signals(T, ftab, pre=200, rail_col="rail_v"):
    """the runtime signals and the share construction per spin over the last `pre` ms before the cut: i_hat_d is the
    crest less Delta, v_mean_d the period mean of the terminal volts with it, and the uncompensated v_mean_0 with the
    raw crest"""
    T = T.copy()
    T["v_on"] = T[f"v_on_{pre}"]
    T["i_crest"] = T[f"i_crest_{pre}"]
    lags = np.array([bd.edge_lags(v) for v in T[rail_col]])
    full = T["duty_applied_pct"] >= 100
    T["share"] = np.where(full, 1.0, bd.share_of(T["half_ticks"], lags[:, 0], lags[:, 1]))
    T["delta_ma"] = np.where(full, 0.0, delta_ma(T["duty_applied_pct"], T["v_on"], ftab))
    T["i_hat_d"] = T["i_crest"] - T["delta_ma"]
    T["i_hat_0"] = T["i_crest"]
    for k in ("d", "0"):
        T[f"v_mean_{k}"] = T["share"] * T["v_on"] - (1 - T["share"]) * R_BRK * T[f"i_hat_{k}"] * 1e-3
    T["dropouts"] = T[f"dropouts_{pre}"]
    return T


# ---------------------------------------------------------------- V_b, the speed table by mechanical direction

LOW_SPEED = 160.0       # rev/s; the rungs under it set the table's floor


def vb_measured(T, ftab, r=R_LOCKED, pre=200, ke=KE, emf="tach"):
    """X per spin in mV, X = V_mean - Ke w - R i_hat, i_hat the crest less Delta, V_mean by the share construction and
    w the ripple tach. The EMF is the estimator's own Ke at the tach's speed, so whatever Ke does with speed lands in
    X and the estimator inherits none of it; emf="e0" reads the coast's EMF at the cut instead"""
    T = signals(T, ftab, pre)
    e = T["e_cut"] if emf == "e0" else 2 * np.pi * ke * T["spin_rev_s"]
    return (T["v_mean_d"] - e - np.asarray(r, float) * T["i_hat_d"] * 1e-3) * 1e3


def rung_means(T, x, by=("level", "duty", "mech")):
    g = pd.DataFrame(dict(rev_s=T["spin_rev_s"], x=x, **{k: T[k] for k in by})).groupby(list(by))
    return g.agg(rev_s=("rev_s", "mean"), x=("x", "mean"), sd=("x", "std"), n=("x", "size")).reset_index()


def dropout_term(S, x):
    """k_d, the part of X that follows the dropout rate, in mV per dropout/s, with its se and the fit's dof: least
    squares over the spins of X = S(rung) + k_d x dropouts, one level per (level, duty) shared by both directions, so
    k_d is read from the direction split at one speed and from the spin-to-spin spread inside a rung, never below 0"""
    keys = pd.factorize(S[["level", "duty"]].apply(tuple, axis=1))[0]
    d = S["dropouts"].to_numpy(float)
    if not np.any(d > 0):
        return 0.0, np.nan, 0
    n, nk = len(S), int(keys.max()) + 1
    A = np.zeros((n, nk + 1))
    A[np.arange(n), keys] = 1.0
    A[:, -1] = d
    y = np.asarray(x, float)
    beta, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = y - A @ beta
    dof = n - (nk + 1)
    cov = np.linalg.pinv(A.T @ A) * (res @ res / max(dof, 1))
    return max(float(beta[-1]), 0.0), float(np.sqrt(cov[-1, -1])), int(dof)


def vb_floor(rm, low=LOW_SPEED):
    """the table's value at and below 150 rev/s from the rung means under `low`: their mean when the rails agree within
    2 se, a term of speed the table carries; 0 when they split by rail, since a term that follows the rail is the
    construction's and not the motor's. Returns (floor, the by-rail means, the split in se)"""
    h = rm[rm["rev_s"] < low].copy()
    if len(h) == 0:
        return 0.0, {}, np.nan
    if "n" not in h:
        h["n"], h["sd"] = 1, 0.0                                         # bare rung means, one rail
    if "level" not in h:
        h["level"] = 0.0
    by = {lv: (float(np.average(g["x"], weights=g["n"])), float(np.sqrt(np.nansum(g["sd"].to_numpy() ** 2 / g["n"].to_numpy())) / len(g)))
          for lv, g in h.groupby("level")}
    if len(by) < 2:
        return next(iter(by.values()))[0], by, np.nan
    (x1, s1), (x2, s2) = list(by.values())[:2]
    se = np.hypot(s1, s2)
    # one spin per rung has no se; then the rails split when they differ by more than half the larger of them
    split = (abs(x1 - x2) / se > 2) if se > 0 else (abs(x1 - x2) > 0.5 * max(abs(x1), abs(x2)))
    return (0.0 if split else float(np.average(h["x"], weights=h["n"]))), by, (float(abs(x1 - x2) / se) if se > 0 else np.nan)


def fit_vb(rm, lo=200.0):
    """per mechanical direction: the power law A (w/300)^n over rung means above lo, and the speed table (the floor at
    and below 150 rev/s, the rung means interpolated, extrapolated past the last rung by its last segment, never below 0)"""
    floor, by_rail, split = vb_floor(rm)
    out = {"floor": floor, "floor_by_rail": by_rail, "floor_split_se": split}
    for m, h in rm.groupby("mech"):
        hh = h[(h["rev_s"] > lo) & (h["x"] > 0)].sort_values("rev_s")
        pl = bd.power_law(hh.rename(columns={"x": "x_b"})) if len(hh) >= 2 else dict(mv_300=np.nan, exponent=np.nan, exponent_se=np.nan)
        s = h.sort_values("rev_s")
        xs, ys = s["rev_s"].to_numpy(), s["x"].to_numpy()
        tab = np.interp(VB_KNOTS, xs, ys)
        if len(xs) >= 2:
            sl = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
            beyond = VB_KNOTS > xs[-1]
            tab[beyond] = ys[-1] + sl * (VB_KNOTS[beyond] - xs[-1])
        tab[VB_KNOTS <= 150] = floor
        tab = np.maximum(tab, 0.0)
        out[int(m)] = dict(power=(pl["mv_300"], pl["exponent"], pl.get("exponent_se", np.nan)), table=tab, n_rungs=len(hh),
                           top=float(xs[-1]) if len(xs) else np.nan)
    return out


def vb_fit(T, ftab, r=R_LOCKED, pre=200, ke=KE, emf="tach", dropouts=True):
    """the V_b fit from spins T (mechanical direction tagged) with a given R, Ke and Delta table: k_d from the spins,
    then the speed tables and power laws from the rung means of what k_d leaves; dropouts=False holds k_d at 0, the
    speed-only form"""
    S = signals(T, ftab, pre)
    x = vb_measured(T, ftab, r=r, pre=pre, ke=ke, emf=emf)
    k_d, k_se, dof = dropout_term(S, x) if dropouts else (0.0, np.nan, 0)
    fit = fit_vb(rung_means(T, x - k_d * S["dropouts"]))
    fit.update(k_d=k_d, k_d_se=k_se, k_d_dof=dof)
    return fit


def vb_total(fit, mech, rev_s, dropouts, form="table"):
    """V_b in mV: the speed part by mechanical direction plus k_d times the dropout rate"""
    return vb_eval(fit, mech, rev_s, form) + fit.get("k_d", 0.0) * np.asarray(dropouts, float)


def dropout_separation(T):
    """whether the dropout threshold sits in a gap on this motor: the rows of every spin's 200 ms window by their crest
    over the window mean, in the bands bd.ROW_BANDS names. Clear dropouts sit under 0.10, ordinary troughs over 0.40,
    and the two bands around the threshold are the ambiguous rows; their share of the rows under 0.40 is the figure"""
    s = {name: int(T[name].sum()) for name, _, _ in bd.ROW_BANDS}
    near = s["rows_0p10_to_thr"] + s["rows_thr_to_0p40"]
    low = s["rows_under_0p10"] + near
    return dict(s, threshold=tel.DROPOUT_FRAC, ambiguous_share=(near / low if low else np.nan))


def vb_eval(fit, mech, rev_s, form="table"):
    """V_b in mV at rev_s for each mechanical direction, from the table or the power law; past the table's last knot
    it holds the last value"""
    rev_s = np.maximum(np.asarray(rev_s, float), 0.0)
    mech = np.broadcast_to(np.asarray(mech), rev_s.shape)
    out = np.zeros_like(rev_s)
    for m in (1, -1):
        sel = mech == m
        if not sel.any():
            continue
        f = fit[m]
        if form == "power" or f["table"] is None:
            A, n, _ = f["power"]
            out[sel] = A * (rev_s[sel] / 300.0) ** n
        else:
            out[sel] = np.interp(rev_s[sel], VB_KNOTS, f["table"], right=np.nan)
            out[sel & np.isnan(out)] = f["table"][-1]
    return out


# ---------------------------------------------------------------- the 100 % anchor: c = J/(Kt Ke), and Delta from it

def cut_slopes(REV, ST):
    """the drive-side speed slope over the last 100 ms before each speed-torque cut, from its turn-window tach"""
    rows = []
    for _, r in ST.iterrows():
        h = REV[(REV["k"] == r["k"]) & (REV["dir"] == r["dir"])]
        tcut = h["t"].max()
        hh = h[h["t"] > tcut - 0.1]
        rows.append(dict(k=r["k"], dir=r["dir"], slope_100=np.polyfit(hh["t"], hh["rev_s"], 1)[0]))
    return pd.DataFrame(rows)


def alpha_reads(T):
    """the coast's deceleration at the cut, rad/s^2, by three reads, from the drag columns (drag = J alpha / Ke)"""
    return {x: T[f"drag_{x}_ma"] * 1e-3 * KE / J for x in ("win", "pin", "slope")}


ANCHOR_READS = ("win", "pin")


def c_per_cut(ST):
    """c = i100 / (Ke (alpha_drive + alpha_coast)) at each cut, A s/V, by the coast's 5 ms windows and its pinned read.
    At 100 % nothing switches, so the crest is the mean current, and the same current in the drive and the drag over the
    coast's first milliseconds gives c with no meter. Ke and J cancel in c Ke (alpha_coast + alpha_drive)."""
    a = alpha_reads(ST)
    ad = 2 * np.pi * ST["slope_100"]
    return {x: (ST["i100"] * 1e-3 / (KE * (ad + a[x]))).to_numpy() for x in ANCHOR_READS}


def anchor_delta(T, cvals, SL, run, pre=200):
    """Delta = crest - anchor per spin of T (one run) in mA, with c per read: the anchor is c Ke (alpha + alpha_drive),
    the pinned read under 300 rev/s and the 5 ms windows over it; crest and drive-side slope over `pre` ms before the cut"""
    T = T.merge(SL[SL["run"] == run][["folder", "dir", f"slope_{pre}"]], on=["folder", "dir"], how="left")
    ad = 2 * np.pi * T[f"slope_{pre}"].fillna(0.0)
    a = alpha_reads(T)
    an = {x: cvals[x] * KE * (a[x] + ad) * 1e3 for x in ANCHOR_READS}
    T["anchor"] = np.where(T["spin_rev_s"] < 300, an["pin"], an["win"])
    T["delta_anchor"] = T[f"i_crest_{pre}"] - T["anchor"]
    T["v_on_pre"] = T[f"v_on_{pre}"]
    return T


def f_from(T):
    """f(D) in mA/V per commanded duty from a set of spins with delta_anchor, in the published table's form (flat below
    the lowest duty, 0 at 100 %); returns the table and the per-duty means"""
    T = T.assign(f=T["delta_anchor"] / T["v_on_pre"])
    f = T.groupby("duty")["f"].mean()
    d = np.r_[0.0, f.index.to_numpy(float), 100.0]
    v = np.r_[f.iloc[0], f.to_numpy(), 0.0]
    return (d, v), f


# ---------------------------------------------------------------- locked R: seats and holds

def combos(x, k):
    """every mean of k distinct seats, as a deviation from the mean over all seats, %"""
    x = np.asarray(x, float)
    ref = x.mean()
    return np.array([(np.mean(x[list(c)]) / ref - 1) * 100 for c in itertools.combinations(range(len(x)), k)])


def lap_line(p, t_end, t_from=0.060):
    """rest-resistance's lap read over a shorter hold: a line through V/I against time from t_from to t_end, read back
    to the drive's first row"""
    t, i, u = p["t"], p["amps"], p["volts"]
    m = (t >= t_from) & (t <= t_end)
    return float(np.polyfit(t[m], (u / i)[m], 1)[1])


def lap_mean(p, t_end, t_from=0.002):
    """a plain mean V over mean I from t_from to t_end"""
    t, i, u = p["t"], p["amps"], p["volts"]
    m = (t >= t_from) & (t <= t_end)
    return float(u[m].mean() / i[m].mean())


# ---------------------------------------------------------------- costs and copper joules, per spin

def _drive_current(sp, zero_i):
    on = int(np.flatnonzero(sp["duty_counts"].to_numpy() != 0)[0])
    sp = sp.iloc[on:]
    return sp, (sp["current_raw"].to_numpy(float) - zero_i - bd.DRIVE_OFF) / bd.CPA


def spin_costs(folder, run):
    """per spin: the revs and seconds of the bench spin, and the parts a short spin needs. A rough speed every row from
    the board's own signals, w = (D v_on - R_locked i) / Ke, boxcar 5 ms, integrated, gives the revs the spin used and
    the revs and time to 95 % of its final speed; the coast's EMF over Ke gives the revs from the cut to the stop and
    in the first 15 ms after it"""
    box = int(0.005 * FS)
    df, meta = tel.load(folder, BOARD)
    rest = tel.baseline(df)
    zero_i = float(rest["current_raw"].median())
    zero_e = float((rest["vmotor_a"] - rest["vmotor_b"]).median())
    out = []
    for d, sp, co in mm.spin_coast_pairs(df):
        sp, i = _drive_current(sp, zero_i)
        dfrac = np.minimum(sp["duty_counts"].abs().to_numpy(float) / bd.ARR, 1.0)
        vd = ((sp["vmotor_a"] - sp["vmotor_b"]) * d - zero_e).to_numpy(float) / bd.K_CPV
        w = np.convolve((dfrac * vd - R_LOCKED * i) / KE, np.ones(box) / box, mode="same")
        w = np.maximum(w, 0.0)
        revs = np.cumsum(w) / (2 * np.pi) / FS
        w_end = float(w[-int(0.05 * FS):-box].mean())
        reach = np.flatnonzero(w >= 0.95 * w_end)
        k95 = int(reach[0]) if len(reach) else len(w) - 1
        z = mm.cut_row(co)
        e = (((co["vmotor_a"] - co["vmotor_b"]) * d).to_numpy(float)[z:] - zero_e) / bd.K_CPV
        e = mm.bridge_rows(e, mm.open_rows(e))
        es = np.maximum(np.convolve(e, np.ones(box) / box, mode="same"), 0.0)
        below = np.flatnonzero(es[mm.COAST_FROM:] < mm.TURN_V)
        stop = mm.COAST_FROM + int(below[0]) if len(below) else len(es)
        crev = np.cumsum(es[:stop]) / KE / (2 * np.pi) / FS
        out.append(dict(run=run, folder=str(results.rel(folder)), dir=d, spin_s=len(sp) / FS, spin_revs=float(revs[-1]),
                        rev_s_end=w_end / (2 * np.pi), t95_s=k95 / FS, revs95=float(revs[k95]),
                        coast_stop_s=stop / FS, coast_revs=float(crev[-1]),
                        coast_revs_15ms=float(crev[min(int(0.015 * FS), len(crev) - 1)])))
    return out


def copper_joules(folder):
    """per spin: R_locked i^2 integrated from the drive's start, J, against time (the crest over-reads the mean at
    partial duty, so an upper bound there; at 100 % the crest is the DC current)"""
    df, meta = tel.load(folder, BOARD)
    rest = tel.baseline(df)
    zero_i = float(rest["current_raw"].median())
    out = []
    for d, sp, co in mm.spin_coast_pairs(df):
        sp, i = _drive_current(sp, zero_i)
        out.append(dict(folder=str(results.rel(folder)), dir=d, t=np.arange(len(i)) / FS, e=np.cumsum(R_LOCKED * i ** 2) / FS))
    return out


def folder_ntc(folder):
    """the board NTC at the folder's rest baseline, raw and through the nominal beta"""
    df, meta = tel.load(folder, BOARD)
    raw = float(tel.baseline(df)["ntc_raw"].median())
    return dict(folder=str(results.rel(folder)), ntc_raw=raw, ntc_c=float(BOARD.ntc_c_beta(raw)))


def cycle_folders():
    """(run, cycle row, folder) of every spinning capture this notebook reads: capture-1, speed-torque and swapped-1"""
    out = []
    for run in ("capture-1", "speed-torque"):
        root = bd.RUNS[run]
        for _, C in pd.read_csv(root / "cycles.csv").iterrows():
            out.append((run, C, root / C["folder"]))
    for block in ("wired", "swapped"):
        for _, C in pd.read_csv(SWAP / f"{block}.csv").iterrows():
            out.append((f"swapped-{block}", C, SWAP / C["folder"]))
    return out


@functools.lru_cache(maxsize=None)
def costs():
    """spin_costs over every spinning capture, with each cycle's rail, duty and repeat"""
    F = cycle_folders()
    parts = mm.run_parallel([(spin_costs, (f, run), {}) for run, C, f in F])
    T = pd.DataFrame([r for p in parts for r in p])
    K = pd.DataFrame([dict(run=run, folder=str(results.rel(f)), k=C["k"], volts_set=C["volts_set"], duty_pct=C["duty_pct"],
                           repeat=C["repeat"]) for run, C, f in F])
    return T.merge(K, on=["run", "folder"], how="left")


# ---------------------------------------------------------------- the winding's rise, closed forms

def held_rise_c(joules, c_body):
    """the body's rise, C, from joules held with no cooling credited"""
    return joules / c_body


def block_rise_c(e_spin, n, rest_s, c_cu, c_body, tau_fast):
    """the copper's rise at the end of a block of n equal spins rest_s apart: every spin's joules held by the body, no
    body cooling credited, plus each spin's copper-only rise decayed by the fast time constant to the block's end"""
    return (e_spin / c_body) * n + (e_spin / c_cu) * sum(math.exp(-k * rest_s / tau_fast) for k in range(n))


def spins_for_change(sd_pct, change_pct=0.5):
    """spins per check, half each way, for a change between two checks at one standard error of change_pct, from a
    per-spin sd: the change's variance is 2 sd^2 / n"""
    return int(math.ceil(2 * (sd_pct / change_pct) ** 2))
