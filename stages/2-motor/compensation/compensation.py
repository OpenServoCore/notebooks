"""compensation's machinery: the back-EMF speed estimator with the reading term and the brush drop compensated.

The estimator reads only what a servo's board can read, the applied duty, the terminal volts over the window, the crest
current sample and the rail tap, and turns them into a speed,

    w_hat = (V_mean - R (i_crest - Delta(D) V_window) - k_d dropouts - V_b(w_hat, mech dir)) / Ke

the one root of a monotone equation, found by bisection. V_mean is the share construction, the reading term Delta comes
from a table f(D) per volt of the window, the dropout term is a coefficient on the rate at which the crest sample sits
near zero (off before the solve, since it does not depend on w_hat), and the speed part of V_b is a table or a power law
by mechanical direction. The truth is the ripple tach, which never enters the estimator.

The pieces this notebook shares with identification come from identification.py (the reductions, the published f(D),
the share construction, the V_b fit and its evaluation, the tach gate, the mechanical direction, the anchor), and
motor-model's modules come through it. What is new here is the estimator and its scoring, the dropout rate around a
run-up's turn windows, the reading-term grid as a line in G (motor-model's steady states rebuilt at two values of G, so
a grid at a held-out G is a line), and the pass machinery that scores a set of constants on every held-out capture.
"""
import functools
import sys

import numpy as np
import pandas as pd

from oscnb import results

_ID_DIR = results.ROOT / "stages" / "2-motor" / "identification"
if str(_ID_DIR) not in sys.path:
    sys.path.insert(0, str(_ID_DIR))
import identification as idn  # noqa: E402

mm, bd = idn.mm, idn.bd
BOARD, MOTOR, PAIR, MM = idn.BOARD, idn.MOTOR, idn.PAIR, idn.MM
KE, KE_REV, J, FS = idn.KE, idn.KE_REV, idn.J, idn.FS
KE_LO, KE_HI = bd.KE_LO, bd.KE_HI
R_LOCKED, R_BRK = idn.R_LOCKED, idn.R_BRK
GEAR, MECH, VB_KNOTS = idn.GEAR, idn.MECH, idn.VB_KNOTS
RAILS, DUTIES = idn.GRID_RAILS, idn.GRID_DUTIES
G_SPIN = MM["g_spin_ms"]["value"] * 1e-3            # motor-model's spinning loss, S
R_METER = MM["meter_loop_ohm"]["value"]             # the meter's loop in the voltage path, ohm

VARIANTS = ("none", "delta", "vb", "both")
BANDS = ((0, 100), (100, 160), (160, 250), (250, 350), (350, 450), (450, 600))
BAND_ORDER = ["0-100", "100-160", "160-250", "250-350", "350-450", "450-600", "all"]
CAPS = ["capture-1", "swapped-1 wired", "swapped-1 swapped", "speed-torque run-up", "speed-torque settled"]

# the identification passes' V_b subsets, the same ten as identification's route 6
SUBSETS = {
    "all 10 rungs": [(5.0, 15), (5.0, 20), (5.0, 30), (5.0, 40), (5.0, 60), (7.4, 15), (7.4, 20), (7.4, 30), (7.4, 40), (7.4, 60)],
    "7 rungs over 145 rev/s": [(5.0, 30), (5.0, 40), (5.0, 60), (7.4, 20), (7.4, 30), (7.4, 40), (7.4, 60)],
    "5 V 30/40/60 + 7.4 V 40/60": [(5.0, 30), (5.0, 40), (5.0, 60), (7.4, 40), (7.4, 60)],
    "7.4 V only, 5 rungs": [(7.4, 15), (7.4, 20), (7.4, 30), (7.4, 40), (7.4, 60)],
    "7.4 V 20/30/40/60": [(7.4, 20), (7.4, 30), (7.4, 40), (7.4, 60)],
    "7.4 V 30/40/60": [(7.4, 30), (7.4, 40), (7.4, 60)],
    "5 V 40 + 7.4 V 30/60": [(5.0, 40), (7.4, 30), (7.4, 60)],
    "7.4 V 40/60": [(7.4, 40), (7.4, 60)],
    "7.4 V 60 only": [(7.4, 60)],
    "5 V only, 5 rungs": [(5.0, 15), (5.0, 20), (5.0, 30), (5.0, 40), (5.0, 60)],
}
SUBSET_RULE_PTS = 0.2           # a subset holds if its median pass is within this many points of rms of column A everywhere

rms = idn.rms
in_rungs = idn.in_rungs
tach_gate = idn.tach_gate
signals = idn.signals
vb_measured = idn.vb_measured
rung_means = idn.rung_means
fit_vb = idn.fit_vb
vb_fit = idn.vb_fit


def vb_eval(fit, mech, rev_s, form):
    """the speed part of V_b in mV by mechanical direction, the table or the power law; the form is always named"""
    return idn.vb_eval(fit, mech, rev_s, form)


def vb_total(fit, mech, rev_s, dropouts, form):
    """V_b in mV, the speed part plus k_d times the dropout rate"""
    return idn.vb_total(fit, mech, rev_s, dropouts, form)


# ---------------------------------------------------------------- reductions

@functools.lru_cache(maxsize=None)
def reductions():
    """identification's per-spin reductions (TB by run, SL the drive-side tach slopes) and motor-model's coast
    reduction of the spinning points (CT), the drag at each cut three ways"""
    R = idn.reductions()
    CT = mm.coast_table(mm.run_parallel(mm.coast_jobs()))
    return dict(TB=R["TB"], SL=R["SL"], CT=CT)


@functools.lru_cache(maxsize=None)
def windows():
    """speed-torque's turn windows (its own cells), u on the rest zero: the run-up (t < SETTLED, >= 150 rev/s), the
    settled windows, the period-fit L the windows carry, and the full table"""
    REV, RUNS_ST, L_H, T_OFF, VPC, SETTLED = idn.runup_windows()
    REV, _ = bd.runup_y(REV, RUNS_ST, L_H, T_OFF, VPC)
    REV["dropouts_per_s"] = window_dropouts(REV, RUNS_ST)
    RU = REV[(REV["t"] < SETTLED) & (REV["rev_s"] >= 150)].copy()
    RS = REV[REV["t"] >= SETTLED].copy()
    return dict(REV=REV, RU=RU, RS=RS, L_H=L_H, SETTLED=SETTLED)


DROPOUT_SPAN_S = 0.05   # the shortest stretch a dropout rate is read over: a one-turn window holds 0 to 3 dropout rows


def window_dropouts(REV, RUNS, span_s=DROPOUT_SPAN_S):
    """the dropout rows per second around each turn window, from speed-torque's own open-row mask of its run-up (the
    same quarter, against a 2 ms running mean since the run-up's current moves), over the turn or `span_s` centered on
    it, whichever is longer, clipped to the run"""
    masks = {(r["k"], r["dir"]): np.asarray(r["open"]) for r in RUNS}
    out = np.zeros(len(REV))
    for j, (k, d, t, w) in enumerate(zip(REV["k"], REV["dir"], REV["t"], REV["w"])):
        m = masks[(k, d)]
        half = max(np.pi / w, span_s / 2)
        a, b = max(int(np.ceil((t - half) * FS)), 0), min(int(np.floor((t + half) * FS)), len(m))
        if b <= a:
            continue
        out[j] = m[a:b].sum() / ((b - a) / FS)
    return out


def tag_mech(T, run, mapping="mech"):
    """T with a mechanical direction: the board direction times the run's lead orientation, or the board direction
    itself under the board mapping"""
    T = T.copy()
    T["mech"] = T["dir"] * (MECH[run] if mapping == "mech" else 1)
    return T


# ---------------------------------------------------------------- the reading-term grid as a line in G

def motor_fast_params():
    """motor-model's locked-motor fast-decay fit (the RGb model at hole 11), the L model the crest-term grid holds"""
    r = mm.job_fast("motor", "RGb")
    prm = {n: v for (n, k), v in zip(r["joint"]["names"], r["joint"]["x"])}
    return dict(prm, kc=0.9, mem=2)


def g_fit(gap, a, b, sig, sel):
    """G by weighted least squares on gap = a + b G over the points in sel, its se (scaled by chi2 when over one), chi2/dof"""
    wt = 1 / sig[sel] ** 2
    yy = gap[sel] - a[sel]
    G = np.sum(wt * b[sel] * yy) / np.sum(wt * b[sel] ** 2)
    chi2 = np.sum(wt * (yy - b[sel] * G) ** 2) / max(sel.sum() - 1, 1)
    return G, np.sqrt(1 / np.sum(wt * b[sel] ** 2) * max(chi2, 1)), chi2


def delta_grid(CT):
    """motor-model's 18 spinning points and its crest-term grid (no meter and with the meter, four rails, ten duties)
    rebuilt at G = 0 and 2 mS, so each is a line a + b G; G refitted on capture-1 alone and on the meter runs alone,
    each predicting the other's points; a direct run at G_spin at 5 and 7.4 V checks the line"""
    SP, MO, c_scale, c_sd = mm.spin_points(CT)
    R_M = float(MO["meter_ohm"].mean())
    R_MOT = mm.RR["r_term_median_ohm"]["value"]
    PTS = SP[SP["duty"] < 100].reset_index(drop=True)
    zero_ma = (mm.CS["drive_offset_counts"]["hi"] - mm.CS["drive_offset_counts"]["lo"]) / 2 / mm.CPA * 1e3
    PTS["sig"] = np.sqrt(np.nan_to_num(PTS["mean_err"], nan=0.5) ** 2 + zero_ma ** 2
                         + np.where(PTS["run"] == "capture-1", (c_sd / c_scale * PTS["mean"]) ** 2 / 6, 0.0))
    P0 = motor_fast_params()
    rs_pt = lambda r: R_MOT + (R_M if r["loop"] == "meter" else 0.0)
    mm.simulate(dict(rs=4.0, l0=5e-4), dict(decay="fast", half=300, on_lag=8, off_lag=12, v_on=5.0))   # compiles the simulator once
    jobs, tags = [], []
    for i, r in PTS.iterrows():
        for gg in (0.0, 0.002):
            jobs.append((mm.steady_both, (dict(P0, g=gg, rs=rs_pt(r)), r["duty"], r["v_on"], r["mean"]), {}))
            tags.append(("pt", i, gg))
    # the grid at motor-model's drag-curve operating point, a + b sqrt(w) through capture-1's coasts
    c1 = CT[CT["run"] == "capture-1"]
    w = 2 * np.pi * c1["spin_rev_s"].to_numpy()
    y = c_scale * np.where(c1["spin_rev_s"] < 300, c1["drag_pin_ma"], c1["drag_win_ma"])
    ok = np.isfinite(w) & np.isfinite(y)
    CD = np.linalg.lstsq(np.c_[np.ones(ok.sum()), np.sqrt(w[ok])], y[ok], rcond=None)[0]
    drag = lambda om: CD[0] + CD[1] * np.sqrt(max(om, 0.0))
    cells = []
    for loop, rext in (("no meter", 0.0), ("meter", R_M)):
        for v in RAILS:
            for du in DUTIES:
                sh = mm.share(du)
                lo_, hi_ = 0.0, sh * v / mm.KE
                for _ in range(60):
                    om = 0.5 * (lo_ + hi_)
                    if sh * v - mm.KE * om - drag(om) * 1e-3 * (R_MOT + rext) > 0:
                        lo_ = om
                    else:
                        hi_ = om
                cells.append(dict(loop=loop, rail=v, duty=du, rev_s=om / 2 / np.pi, mean_ma=drag(om)))
                for gg in (0.0, 0.002):
                    jobs.append((mm.steady_both, (dict(P0, g=gg, rs=R_MOT + rext), du, v, drag(om)), {}))
                    tags.append(("grid", len(cells) - 1, gg))
    for i, c in enumerate(cells):
        if c["rail"] in (5.0, 7.4) and c["loop"] == "no meter":
            jobs.append((mm.steady_both, (dict(P0, g=G_SPIN, rs=R_MOT), c["duty"], c["rail"], c["mean_ma"]), {}))
            tags.append(("lincheck", i, G_SPIN))
    OUTS = dict(zip(tags, mm.run_parallel(jobs, chunksize=4)))

    a = np.array([OUTS[("pt", i, 0.0)]["crest_less_mean"] for i in PTS.index])
    b = (np.array([OUTS[("pt", i, 0.002)]["crest_less_mean"] for i in PTS.index]) - a) / 0.002
    gap, sig = PTS["gap"].to_numpy(), PTS["sig"].to_numpy()
    is_c1 = (PTS["run"] == "capture-1").to_numpy()
    GF = {name: g_fit(gap, a, b, sig, sel) for name, sel in
          (("all", np.ones(len(PTS), bool)), ("capture-1 only", is_c1), ("meter runs only", ~is_c1))}
    PTS["a"], PTS["b"] = a, b
    PTS["pred_all"] = a + b * GF["all"][0]
    PTS["pred_c1"] = a + b * GF["capture-1 only"][0]
    PTS["pred_meter"] = a + b * GF["meter runs only"][0]
    PTS["held"] = np.where(is_c1, PTS["pred_meter"], PTS["pred_c1"])
    PTS["resid_held"] = PTS["gap"] - PTS["held"]
    PTS["resid_all"] = PTS["gap"] - PTS["pred_all"]
    PTS["z_held"] = PTS["resid_held"] / PTS["sig"]

    grid_pub, gm_pub = MM["crest_less_mean_ma"], MM["crest_less_meter_ma"]
    GR = pd.DataFrame(cells)
    GR["a"] = [OUTS[("grid", i, 0.0)]["crest_less_mean"] for i in GR.index]
    GR["b"] = [(OUTS[("grid", i, 0.002)]["crest_less_mean"] - OUTS[("grid", i, 0.0)]["crest_less_mean"]) / 0.002 for i in GR.index]
    GR["at_gspin"] = GR["a"] + GR["b"] * G_SPIN
    GR["pub"] = [(grid_pub if r.loop == "no meter" else gm_pub)[f"{r.rail:g} V"][f"{r.duty} %"]["value"] for r in GR.itertuples()]
    GR["lin_less_pub"] = GR["at_gspin"] - GR["pub"]
    lin_err = np.array([GR.loc[i, "at_gspin"] - OUTS[("lincheck", i, G_SPIN)]["crest_less_mean"]
                        for i in GR.index if ("lincheck", i, G_SPIN) in OUTS])
    GR["c1only"] = GR["pub"] + GR["b"] * (GF["capture-1 only"][0] - G_SPIN)
    GR["meteronly"] = GR["pub"] + GR["b"] * (GF["meter runs only"][0] - G_SPIN)
    return dict(PTS=PTS, GF=GF, GR=GR, CD=CD, R_M=R_M, lin_err=lin_err, c_scale=c_scale, c_sd=c_sd, n_jobs=len(jobs))


def f_table(GR, g_shift=0.0):
    """f(D) in mA/V by duty from the no-meter grid moved along its line in G by g_shift (S), the published grid at
    g_shift 0; the rail mean per volt, held flat below 10 %, zero at 100 % where nothing switches"""
    nm = GR[GR["loop"] == "no meter"].copy()
    nm["f"] = (nm["pub"] + nm["b"] * g_shift) / nm["rail"]
    f = nm.groupby("duty")["f"].mean()
    d = np.r_[0.0, f.index.to_numpy(float), 100.0]
    v = np.r_[f.iloc[0], f.to_numpy(), 0.0]
    return d, v


def grid_lookup(GR, loop, rail, duty, col="pub"):
    g = GR[(GR["loop"] == loop) & (GR["rail"] == rail)].sort_values("duty")
    return float(np.interp(duty, g["duty"], g[col]))


def published_vb():
    """motor-model's published power law by direction (capture-1 and the 100 % ends), capture-1's board direction being
    the mechanical one"""
    f = MM["brush_drop_fit"]
    return {m: dict(power=(f[k]["mv_at_300_rev_s"]["value"], f[k]["exponent"]["value"], np.nan), table=None)
            for m, k in ((1, "+1"), (-1, "-1"))} | {"k_d": 0.0}


# ---------------------------------------------------------------- the estimator

def solve_w(u, vb_fn, ke=KE, iters=60):
    """w from 2 pi Ke w + V_b(w)/1000 = u, u = V_mean - R i less any term that does not depend on w, in volts, w in
    rev/s, vb_fn(w) in mV. The left side rises with w wherever V_b does not fall faster than Ke, so with V_b >= 0 the
    root sits between 0 and u/(2 pi Ke) and bisection finds it; a fixed point in w runs away where V_b is steep, which
    a power law is past its fitted range. A u at or under 0 reads as the plain estimate, a speed at or under zero"""
    ke_rev = 2 * np.pi * ke
    u = np.asarray(u, float)
    hi = np.maximum(u, 0.0) / ke_rev
    lo = np.zeros_like(hi)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        over = ke_rev * mid + vb_fn(mid) * 1e-3 > u
        hi = np.where(over, mid, hi)
        lo = np.where(over, lo, mid)
    return np.where(u <= 0, u / ke_rev, 0.5 * (lo + hi))


def estimate(T, mech, fit, r=R_LOCKED, ke=KE, form="power"):
    """w_hat per spin for the four variants: none, Delta only, V_b only, both; T from signals(). The dropout term of
    V_b does not depend on w, so it comes off u before the solve"""
    out = {}
    k_d = fit.get("k_d", 0.0)
    for var in VARIANTS:
        k = "d" if var in ("delta", "both") else "0"
        u = T[f"v_mean_{k}"].to_numpy() - np.asarray(r, float) * T[f"i_hat_{k}"].to_numpy() * 1e-3
        if var in ("vb", "both"):
            u = u - k_d * T["dropouts"].to_numpy(float) * 1e-3
            out[var] = solve_w(u, lambda w: vb_eval(fit, mech, w, form), ke)
        else:
            out[var] = u / (2 * np.pi * ke)
    return out


def band_of(w):
    for a, b in BANDS:
        if a <= w < b:
            return f"{a}-{b}"
    return "other"


def score_spins(T, fit, ftab, run, r=R_LOCKED, ke=KE, form="power", pre=200):
    """the estimator through spins T (mechanical direction tagged), every variant beside the ripple tach (truth) and
    the coast's E0 over the reference Ke (truth_e0)"""
    S = signals(T, ftab, pre)
    W = estimate(S, S["mech"].to_numpy(), fit, r, ke, form)
    for v, w in W.items():
        S[f"w_{v}"] = w
    S["truth"] = S["spin_rev_s"]
    S["truth_e0"] = S["e_cut"] / KE_REV
    S["run"] = run
    S["band"] = [band_of(w) for w in S["truth"]]
    return S


def score_windows(W, fit, form, r=R_LOCKED, ke=KE, l_h=None):
    """speed-torque's turn windows through the estimator at 100 %, w = (u - L di/dt - R i - V_b(w)) / Ke; Delta is 0
    at 100 %, so Delta only is uncompensated and both is V_b only; vb_meas_mv is y - R i, the brush drop the window
    carries, and vb_pred_mv the fit's at the window's tach speed"""
    l_h = windows()["L_H"] if l_h is None else l_h
    W = W.copy()
    mech = W["dir"].to_numpy() * MECH["speed-torque"]
    dropouts = W["dropouts_per_s"].to_numpy(float)
    u = (W["u_rest"] - l_h * W["didt"] - np.asarray(r, float) * W["i"]).to_numpy()
    W["w_none"] = u / (2 * np.pi * ke)
    W["w_delta"] = W["w_none"]
    W["w_vb"] = solve_w(u - fit.get("k_d", 0.0) * dropouts * 1e-3, lambda w: vb_eval(fit, mech, w, form), ke)
    W["w_both"] = W["w_vb"]
    W["truth"] = W["rev_s"]
    W["band"] = [band_of(w) for w in W["truth"]]
    W["vb_meas_mv"] = (W["y"] - np.asarray(r, float) * W["i"]) * 1e3
    W["vb_pred_mv"] = vb_total(fit, mech, W["rev_s"].to_numpy(), dropouts, form)
    return W


def score_table(df, by, truth="truth", variants=VARIANTS):
    """per group: n, the truth's mean speed, and per variant the mean signed error and the rms, in % of the truth and
    in rev/s"""
    rows = []
    for key, g in df.groupby(by):
        r = dict(zip(by, key if isinstance(key, tuple) else (key,)))
        r.update(n=len(g), rev_s=g[truth].mean())
        for v in variants:
            e = g[f"w_{v}"] - g[truth]
            p = e / g[truth] * 100
            r.update({f"{v}_pct": p.mean(), f"{v}_rms_pct": np.sqrt((p ** 2).mean()), f"{v}_revs": e.mean(),
                      f"{v}_rms_revs": np.sqrt((e ** 2).mean())})
        rows.append(r)
    return pd.DataFrame(rows)


def bands(S):
    """both compensations only: mean signed error and rms in % and rev/s by band and over all"""
    rows = []
    for b, g in list(S.groupby("band")) + [("all", S)]:
        e = g["w_both"] - g["truth"]
        p = e / g["truth"] * 100
        rows.append(dict(band=b, n=len(g), rev_s=g["truth"].mean(), mean_pct=p.mean(), rms_pct=rms(p), mean_revs=e.mean(), rms_revs=rms(e)))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- a pass: one set of constants on every held-out set

def score_pass(C1_test, SW, RU, RS, P, form="table"):
    """every held-out set for one pass P (r, ke, l_h, ftab, fit, and optionally ftab_c1 and fit_c1 for capture-1):
    capture-1's spins C1_test, swapped-1's gated blocks SW, speed-torque's run-up and settled windows"""
    out = {"capture-1": bands(score_spins(C1_test, P.get("fit_c1", P["fit"]), P.get("ftab_c1", P["ftab"]), "capture-1",
                                          P["r"], P["ke"], form))}
    for block, T in SW.items():
        out[f"swapped-1 {block}"] = bands(score_spins(T, P["fit"], P["ftab"], f"swapped-{block}", P["r"], P["ke"], form))
    out["speed-torque run-up"] = bands(score_windows(RU, P["fit"], form, P["r"], P["ke"], P["l_h"]))
    out["speed-torque settled"] = bands(score_windows(RS, P["fit"], form, P["r"], P["ke"], P["l_h"]))
    return out


def flat(scores, **tags):
    """a pass's scores as rows, one per capture and band, with the given tags"""
    return [dict(tags, capture=cap, **b.to_dict()) for cap, B in scores.items() for _, b in B.iterrows()]


def summarize(df):
    """per capture and band over a set of passes: the median pass's mean and rms, the range of the rms"""
    return df.groupby(["capture", "band"]).agg(n=("n", "median"), rev_s=("rev_s", "median"), passes=("rms_pct", "size"),
                                               mean_pct=("mean_pct", "median"), rms_pct=("rms_pct", "median"),
                                               rms_lo=("rms_pct", "min"), rms_hi=("rms_pct", "max"),
                                               rms_revs=("rms_revs", "median")).reset_index()


# ---------------------------------------------------------------- the error budget

def quadrature(*terms):
    """independent 1 sigma terms added in quadrature"""
    return float(np.sqrt(np.sum(np.square(terms))))


def unexplained(observed_rms, known):
    """the part of an observed rms the known terms do not cover, zero when they cover it all"""
    return float(np.sqrt(max(observed_rms ** 2 - known ** 2, 0.0)))
