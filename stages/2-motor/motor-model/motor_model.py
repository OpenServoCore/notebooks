"""motor-model's machinery: the folds, the window-current simulator, the fits and the spinning steady state.

Only motor-model reads it, so it lives beside the notebook, which imports it. Every constant comes through
the results files: current-scale, terminal-chain, sample-timing and bridge-states for the board, and from
stage 2 rest-resistance's stall R by hole, bemf-coast's Ke and cycles, speed-torque's J and teardown's lead
zero. The coil runs' series R is the hand ohms in the teardown's hand.csv less that lead zero. No R, L or
offset that inductance or teardown derived is an input here; the notebook reads those for comparison only.

The simulator is compiled with numba. `run_parallel` spreads independent fits over the machine's cores;
each fit is deterministic, so the result does not depend on the order they finish in.
"""
import functools
import itertools
import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import pandas as pd
from numba import njit
from scipy.optimize import least_squares
from scipy.signal import butter, sosfiltfilt

from oscnb import boards, bursts as bu, motors, results, tel

BOARD = boards.BOARDS[boards.DEFAULT]
MOTOR = motors.MOTORS[motors.DEFAULT]
ARR = BOARD.pwm_arr                          # ticks from the trough to the crest
P = 2 * ARR                                  # ticks per PWM period
TPU = bu.TICKS_PER_US
STEP = bu.STEP_TICKS
APER = bu.APERTURE_CLK * bu.TICKS_PER_ADC_CLK

CS = results.read("1-board", "current-scale", pair=BOARD.key)
TC = results.read("1-board", "terminal-chain", pair=BOARD.key)
ST = results.read("1-board", "sample-timing", pair=BOARD.key)
BSR = results.read("1-board", "bridge-states", pair=BOARD.key)
RR = results.read("2-motor", "rest-resistance", pair=MOTOR.key)
BEMF = results.read("2-motor", "bemf-coast", pair=MOTOR.key)
SPT = results.read("2-motor", "speed-torque", pair=MOTOR.key)
TD = results.read("2-motor", "teardown", pair=MOTOR.key)
NLC = results.read("2-motor", "no-load-current", pair=MOTOR.key)

CPA = CS["counts_per_a"]["value"]
K_CPV, C_OFF = TC["terminal_counts_per_v"]["value"], TC["terminal_offset_counts"]["value"]
FAST_ON, FAST_OFF = BSR["fast_edge_lags_ticks"]["on"]["value"], BSR["fast_edge_lags_ticks"]["off"]["value"]
# slow decay's bridge edges: 5.0 V sits between sample-timing's two rails, so the mean of the two
SLOW_ON = np.mean([ST["tap_on_edge_lag_7v4"]["value"], ST["tap_on_edge_lag_3v7"]["value"]])
SLOW_OFF = np.mean([ST["off_edge_lag_7v4"]["value"], ST["off_edge_lag_3v7"]["value"]])
AMP_TAU = BSR["amp_tau_ticks"]["value"]
AMP_GAIN = BSR["amp_gain"]["value"]
KE = BEMF["ke_v_s_per_rad"]["value"]
J = SPT["j_kg_m2"]["value"]
LEAD_ZERO = TD["pair_zero_ohm"]["value"]     # the meter's probe pair, the zero every hand ohm reading carries
CREST_AT = int(ST["crest_sample_ticks"]["shunt"]["value"])   # ticks after the crest at which the crest shunt sample closes

TEL = results.ROOT / "telemetry"
PAIR = TEL / f"{MOTOR.key}__{BOARD.key}"
LV = PAIR / "teardown"
SETS = {"ra": LV / "coil-levels-ra-1", "bare": LV / "coil-levels-bare-1", "can": LV / "coil-levels-can-1"}
OLD = {"motor": PAIR / "inductance" / "fast-1" / "hole-11", "coil-can-1": LV / "coil-can-1",
       "coil-noring-1": LV / "coil-noring-1"}           # 5 V runs, one folder of groups each
IND = PAIR / "inductance"
BS = TEL / BOARD.key / "bridge-states" / "capture-1"
HAND_DIR = TEL / MOTOR.key / "teardown"
HAND = pd.read_csv(HAND_DIR / "hand.csv")
LEVELS = ("3v7", "5v0", "7v4")


def hand(quantity):
    """every value logged under one quantity name in hand.csv, as floats, as teardown reads them"""
    return HAND.loc[HAND["quantity"] == quantity, "value"].astype(float).to_numpy()


# ---------------------------------------------------------------- the bursts and the folds

def burst_arrays(df, decay):
    """One burst: the shunt samples and the terminal difference against ticks from the drive's origin, the first
    trough (fast decay, window on the trough) or crest (slow decay, window on the crest) at or after the step
    sample; t is the tick the aperture closed."""
    m = bu.meta(df)
    half = int(BOARD.drive_ticks(m["step_q15"]))
    k = np.arange(len(df))
    phi0 = m["start_cnt"] if m["start_dir"] == 0 else P - m["start_cnt"]
    tick = phi0 + STEP * k + APER
    code = df["code"].to_numpy(float)
    sh = k % 2 == 0
    ta, tb = k % 4 == 1, k % 4 == 3
    t_step = tick[m["step_index"]]
    c = 0 if decay == "fast" else ARR
    org = P * np.ceil((t_step - c) / P) + c
    z = np.median(code[sh & (k < m["step_index"] - 8)])
    rest_sd = np.std(code[sh & (k < m["step_index"] - 8)][40:], ddof=1)
    sign = 1 if m["step_q15"] > 0 else -1
    t = tick - org
    va = np.interp(tick[ta], tick[tb], code[tb])
    v = (sign * (code[ta] - va) - C_OFF) / K_CPV
    pre = k[ta] < m["step_index"] - 8
    v0 = np.median(v[pre])
    return dict(t=t[sh], i=(code[sh] - z) / CPA * 1e3, tv=t[ta], v=v, v_rest=v0, half=half, sign=sign,
                pct=int(round(abs(m["step_q15"]) * 100 / 32768)), rest_sd_ma=rest_sd / CPA * 1e3,
                t_step=t_step - org, n_after=int(len(df) - m["step_index"]))


def load_folder(folder, decay=None, skip_first=True):
    """every group folder under `folder` -> one dict per burst, each group's meta checked against the board"""
    out = []
    for g in sorted(p for p in Path(folder).iterdir() if (p / "meta.json").exists()):
        meta = json.loads((g / "meta.json").read_text())
        BOARD.check_meta(meta, str(results.rel(g)))
        d = meta.get("decay", "slow")
        if decay is not None and d != decay:
            continue
        for j, df in enumerate(bu.load(g)):
            if skip_first and j == 0:
                continue
            b = burst_arrays(df, d)
            b.update(group=g.name, burst=j, decay=d, rail_mv=meta.get("rail_mv"))
            out.append(b)
    return out


def motor_r(hole=11):
    """rest-resistance's stall R at the hole"""
    return float(RR["r_term_by_hole"][hole])


def hand_r(set_name):
    """the series R held in every coil fit: the hand ohms at the leads' free ends less the lead zero (coil-can-1
    kept coil-noring-1's leads); the motor's is rest-resistance's stall R at hole 11"""
    if set_name == "motor":
        return motor_r(11)
    q = {"ra": "coil levels ra leads ohm", "bare": "coil levels bare leads ohm", "can": "coil levels can leads ohm",
         "coil-noring-1": "coil noring leads ohm", "coil-can-1": "coil noring leads ohm"}[set_name]
    return float(np.median(hand(q))) - LEAD_ZERO


def build_fold(bursts, decay, pct, sign, set_name="", level=""):
    """One fold per (set, level, decay, duty, sign): the raw shunt samples against ticks from the drive origin,
    the bridge's source terms off the taps (V_on and r_on from the terminal difference against the current
    inside the window, V_ret and r_ret inside the fast return), and the float stretch's mean as a check."""
    bb = [b for b in bursts if b["decay"] == decay and b["pct"] == pct and b["sign"] == sign]
    if not bb:
        return None
    half = bb[0]["half"]
    on_lag, off_lag = (FAST_ON, FAST_OFF) if decay == "fast" else (SLOW_ON, SLOW_OFF)
    t = np.concatenate([b["t"] for b in bb])
    i = np.concatenate([b["i"] for b in bb])
    bid = np.concatenate([np.full(len(b["t"]), j) for j, b in enumerate(bb)])
    sd = float(np.mean([b["rest_sd_ma"] for b in bb]))
    x = ((t + ARR) % P) - ARR
    on, off = -half + on_lag, half + off_lag
    vs, ivs, rv, riv = [], [], [], []
    for b in bb:
        xv = ((b["tv"] + ARR) % P) - ARR
        okv = b["tv"] >= ARR
        ii = np.interp(b["tv"], b["t"], b["i"])
        w = okv & (xv > on + 120) & (xv < off - 120)
        vs.append(b["v"][w]); ivs.append(ii[w])
        if decay == "fast":
            r = okv & (xv > off + 120) & (ii < -15)
            rv.append(b["v"][r]); riv.append(-ii[r])
    vs, ivs = np.concatenate(vs), np.concatenate(ivs)
    A = np.vstack([np.ones_like(ivs), -ivs * 1e-3]).T
    (v_on, r_on), *_ = np.linalg.lstsq(A, vs, rcond=None)
    f = dict(set=set_name, level=level, decay=decay, pct=pct, sign=sign, half=half, on_lag=on_lag,
             off_lag=off_lag, t=t, i=i, x=x, bid=bid, sd=sd, v_on=float(v_on), r_on=float(max(r_on, 0.0)),
             v_on_med=float(np.median(vs)), nb=len(bb), on=on, off=off)
    if decay == "fast" and len(rv) and np.concatenate(rv).size > 10:
        rv, riv = np.concatenate(rv), np.concatenate(riv)
        A = np.vstack([np.ones_like(riv), -riv * 1e-3]).T
        (v_ret, r_ret), *_ = np.linalg.lstsq(A, rv, rcond=None)
        f.update(v_ret=float(v_ret), r_ret=float(max(r_ret, 0.05)))
    else:
        f.update(v_ret=-f["v_on"], r_ret=0.3)
    f.update(v_brk=0.0, r_brk=BSR["low_side_loop_ohm"]["value"])
    if decay == "fast":
        fl = (t >= ARR) & (x < on - 60) & (x > -ARR + 30)
        f["float_ma"] = float(np.mean(i[fl])) if fl.sum() > 5 else np.nan
        f["float_se"] = float(np.std(i[fl], ddof=1) / np.sqrt(fl.sum())) if fl.sum() > 5 else np.nan
    return f


def band_mask(f, a, b):
    """the samples inside [ON + a, OFF - b] of every full window, the first partial window left out"""
    keep = f["t"] >= (P - ARR if f["decay"] == "fast" else ARR)
    return keep & (f["x"] >= f["on"] + a) & (f["x"] <= f["off"] - b)


def prep(F):
    """the return's source terms per level from the reverse fast folds (the forward folds' tap regression in the
    return is not usable), used for every fast fold at that level"""
    rt = {}
    for f in F:
        if f["decay"] == "fast" and f["sign"] == -1:
            rt.setdefault(f["level"], []).append((f["v_ret"], f["r_ret"]))
    rt = {k: (float(np.mean([a for a, _ in v])), float(np.mean([b for _, b in v]))) for k, v in rt.items()}
    for f in F:
        if f["decay"] == "fast" and f["level"] in rt:
            f["v_ret"], f["r_ret"] = rt[f["level"]]
    return F


def _bs_bursts(wire, decay):
    """bridge-states' control bursts on one wiring, every burst but the first of each duty and sign"""
    folder = BS / wire / f"bursts-{decay}"
    BOARD.check_meta(json.loads((folder / "meta.json").read_text()), str(results.rel(folder)))
    out, seen = [], set()
    for j, df in enumerate(bu.load(folder)):
        b = burst_arrays(df, decay)
        key = (b["pct"], b["sign"])
        if key not in seen:
            seen.add(key)
            continue
        b.update(group=f"{wire}-{key}", burst=j, decay=decay, rail_mv=None)
        out.append(b)
    return out


def _folds_from(B, set_name, level, decays=("fast", "slow"), pcts=(25, 40)):
    out = []
    for d in decays:
        for pct in pcts:
            for s in (1, -1):
                f = build_fold(B, d, pct, s, set_name, level)
                if f is not None:
                    out.append(f)
    return out


def folds(name):
    """name: ra / bare / can (the levels runs, three rails), motor (fast-1 hole 11), coil-can-1, coil-noring-1
    (5 V), cap-11 (inductance capture-1 at hole 11, slow), rev-11 (reversal-1), bs-ra / bs-rab (bridge-states'
    controls, fast decay)"""
    if name in SETS:
        F = []
        for lv in LEVELS:
            F += _folds_from(load_folder(SETS[name] / lv), name, lv)
    elif name in OLD:
        F = _folds_from(load_folder(OLD[name]), name, "5v0")
    elif name.startswith("cap-"):
        h = int(name[4:])
        F = _folds_from(load_folder(IND / "capture-1" / f"hole-{h}", decay="slow"), name, "5v0", ("slow",))
    elif name == "rev-11":
        F = _folds_from(load_folder(IND / "reversal-1" / "hole-11", decay="slow"), name, "5v0", ("slow",), (25,))
    elif name.startswith("bs-"):
        F = _folds_from(_bs_bursts(name[3:], "fast"), name, "5v0", ("fast",))
    else:
        raise KeyError(name)
    return prep(F)


def rs_of(f):
    """the series R a fold is fitted with"""
    s = f["set"]
    if s.startswith("cap-"):
        return motor_r(int(s[4:]))
    if s == "rev-11":
        return motor_r(11)
    if s.startswith("bs-"):
        return BSR["dc_r_ohm"][s[3:]]["value"]
    return hand_r(s)


def pick(F, decay=None, levels=None, pcts=None, signs=None):
    return [f for f in F if (decay is None or f["decay"] == decay) and (levels is None or f["level"] in levels)
            and (pcts is None or f["pct"] in pcts) and (signs is None or f["sign"] in signs)]


# ---------------------------------------------------------------- the simulator

ON, RET, FLOAT, BRAKE = 0, 1, 2, 3


def schedule(decay, half, on_lag, off_lag, periods=11):
    """per-tick bridge mode from the drive origin: fast decay's first update at the trough (mid window); slow
    decay's first full window centered on the first crest at t = 0"""
    n = P * periods
    if decay == "fast":
        tt = np.arange(n)
        ph = tt % P
        on = (ph < half + off_lag) | (ph >= P - half + on_lag)
        return np.where(on, ON, RET).astype(np.int64), 0
    t0 = -half + int(round(on_lag))
    tt = np.arange(t0, n)
    ph = (tt + P // 2) % P - P // 2
    on = (ph >= -half + on_lag) & (ph < half + off_lag)
    return np.where(on, ON, BRAKE).astype(np.int64), t0


@njit(cache=True)
def _lin(i0, irev, l0, ca, a2, cr, ic, kc):
    # the incremental inductance L0 (1 + a1 s i + a2 i^2 + cr |i - i_rev|) (1 - kc exp(-|i - i_rev| / ic))
    d = i0 - irev
    if d < 0:
        d = -d
    lin = l0 * (1.0 + ca * i0 + a2 * i0 * i0 + cr * d)
    if ic > 0:
        lin = lin * (1.0 - kc * np.exp(-d / ic))
    if lin < 0.02 * l0:
        lin = 0.02 * l0
    return lin


@njit(cache=True)
def _deriv(x, V, rt, lin, g0, lb, gb, mode):
    # x[0] the magnetizing current, x[1] the rung inductor's current; returns dx and the series current
    i0 = x[0]
    dx = np.zeros(2)
    if g0 > 0:
        if mode == 2:
            dx[0] = (-i0 / g0) / lin
            if lb > 0:
                dx[1] = -x[1] / (gb * lb)
            return dx, 0.0
        if lb > 0:
            i_s = (V + i0 / g0 + x[1] / gb) / (rt + 1 / g0 + 1 / gb)
            dx[1] = (i_s - x[1]) / (gb * lb)
        else:
            i_s = (V + i0 / g0) / (rt + 1 / g0)
        dx[0] = (i_s - i0) / g0 / lin
        return dx, i_s
    if mode == 2:
        if lb > 0:
            dx[1] = -x[1] / (gb * lb)
        return dx, 0.0
    vb = 0.0
    if lb > 0:
        vb = (i0 - x[1]) / gb
        dx[1] = vb / lb
    dx[0] = (V - rt * i0 - vb) / lin
    return dx, i0


@njit(cache=True)
def _run(modes, Vm, Rm, rs, l0, ca, a2, cr, ic, kc, g0, lb, gb, mem, nsub, x_init):
    # tick-level RK4 with nsub substeps a tick; the reversal memory of the magnetizing current as a stack of
    # turning points: mem 0 counts from zero at every window start, mem 1 keeps Rayleigh memory with wipe-out,
    # mem 2 the last reversal only
    n = modes.shape[0]
    x = x_init.copy()
    sh = np.zeros(n)
    ser = np.zeros(n)
    h = 1.0 / (48e6 * nsub)
    floating = False
    stack = np.zeros(512)
    ns = 1
    stack[0] = 0.0
    dirn = 1
    prev_mode = -1
    for t in range(n):
        md = modes[t]
        if md == 1 and floating:
            md = 2
        if md == 0 or md == 3:
            floating = False
        if mem == 0:
            if md == 0 and prev_mode != 0:
                ns = 1
                stack[0] = 0.0
        irev = stack[ns - 1]
        prev_mode = md
        V = Vm[md]
        rt = rs + Rm[md]
        xo = x[0]
        for _ in range(nsub):
            l1 = _lin(x[0], irev, l0, ca, a2, cr, ic, kc)
            k1, _a = _deriv(x, V, rt, l1, g0, lb, gb, md)
            xa = x + 0.5 * h * k1
            k2, _b = _deriv(xa, V, rt, _lin(xa[0], irev, l0, ca, a2, cr, ic, kc), g0, lb, gb, md)
            xb = x + 0.5 * h * k2
            k3, _c = _deriv(xb, V, rt, _lin(xb[0], irev, l0, ca, a2, cr, ic, kc), g0, lb, gb, md)
            xc = x + h * k3
            k4, _d = _deriv(xc, V, rt, _lin(xc[0], irev, l0, ca, a2, cr, ic, kc), g0, lb, gb, md)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        _z, i_s = _deriv(x, V, rt, _lin(x[0], irev, l0, ca, a2, cr, ic, kc), g0, lb, gb, md)
        if md == 1 and i_s <= 0:
            floating = True
            md = 2
            if g0 == 0:
                x[0] = 0.0
            i_s = 0.0
        if md == 2:
            i_s = 0.0
        if x[0] < 0 and g0 == 0:
            x[0] = 0.0
        if mem >= 1:
            dxx = x[0] - xo
            if dirn > 0 and dxx < -1e-12:
                if ns < 512:
                    stack[ns] = xo
                    ns += 1
                dirn = -1
            elif dirn < 0 and dxx > 1e-12:
                if ns < 512:
                    stack[ns] = xo
                    ns += 1
                dirn = 1
            if mem == 2:
                if ns > 1:
                    stack[0] = stack[ns - 1]
                    ns = 1
            while mem == 1 and ns >= 3 and ((dirn > 0 and x[0] > stack[ns - 2]) or (dirn < 0 and x[0] < stack[ns - 2])):
                ns -= 2
            if mem == 1 and ns == 2 and dirn < 0 and x[0] <= stack[0]:
                ns = 1
        sh[t] = i_s if md == 0 else (-i_s if md == 1 else 0.0)
        ser[t] = i_s
    return sh, ser, x


@njit(cache=True)
def amp(sh, tau, gain):
    """the amplifier's first-order lag (tau in ticks) and gain"""
    a = np.exp(-1.0 / tau)
    y = np.zeros(sh.shape[0])
    s = 0.0
    for t in range(sh.shape[0]):
        s = a * s + (1 - a) * sh[t]
        y[t] = s * gain
    return y


def simulate(p, fold, periods=11, nsub=4, emf=0.0, x_init=None, modes=None, t0=None):
    """p: rs, l0 and any of a1 a2 cr ic kc g lb gb mem; fold: decay, half, on_lag, off_lag, v_on, r_on, v_ret,
    r_ret, v_brk, r_brk, sign. Returns (t0, the shunt in mA through the amplifier, the series current in mA,
    the final state)."""
    if modes is None:
        modes, t0 = schedule(fold["decay"], fold["half"], fold["on_lag"], fold["off_lag"], periods)
    Vm = np.array([fold.get("v_on", 0.0) - emf, fold.get("v_ret", 0.0) - emf, 0.0, fold.get("v_brk", 0.0) - emf])
    Rm = np.array([fold.get("r_on", 0.3), fold.get("r_ret", 0.3), 0.0, fold.get("r_brk", 0.3)])
    ca = p.get("a1", 0.0) * float(fold.get("sign", 1))
    xi = np.zeros(2) if x_init is None else np.asarray(x_init, float)
    sh, ser, x = _run(modes, Vm, Rm, p["rs"], p["l0"], ca, p.get("a2", 0.0), p.get("cr", 0.0), p.get("ic", 0.0),
                      p.get("kc", 0.0), p.get("g", 0.0), p.get("lb", 0.0), p.get("gb", 1.0), int(p.get("mem", 1)),
                      nsub, xi)
    return t0, amp(sh, AMP_TAU, AMP_GAIN) * 1e3, ser * 1e3, x


# ---------------------------------------------------------------- the fits

class Model:
    """spec {pname: (x0, lo, hi)}, build(p, fold) -> simulator params, split: parameters fitted per sign /
    level / set, fixed: held parameters (mem, kc)"""

    def __init__(self, name, spec, build=None, split=(), fixed=None):
        self.name, self.spec, self.split = name, spec, split
        self.build = build or (lambda p, f: dict(p))
        self.fixed = fixed or {}


def expand(model, F):
    names, x0, lo, hi = [], [], [], []
    by = dict(model.split)
    for p, (a, l, h) in model.spec.items():
        keys = sorted({f[by[p]] for f in F}) if p in by else [None]
        for k in keys:
            names.append((p, k)); x0.append(a); lo.append(l); hi.append(h)
    return names, np.array(x0, float), np.array(lo, float), np.array(hi, float)


def params_for(model, names, x, f):
    p = dict(model.fixed)
    by = dict(model.split)
    for (n, k), v in zip(names, x):
        if k is None or f[by[n]] == k:
            p[n] = v
    return p


def model_y(p, f):
    """the model's shunt at the fold's samples"""
    t0, y, ser, _ = simulate(p, f)
    idx = np.clip((f["t"] - t0).astype(int), 0, len(y) - 1)
    return y[idx]


def fold_resid(model, names, x, f, a, b, rs):
    p = params_for(model, names, x, f)
    if "rs" not in p:
        p["rs"] = rs(f)
    q = model.build(p, f)
    y = model_y(q, f)
    m = band_mask(f, a, b)
    return (f["i"][m] - y[m]) / f["sd"], (f["i"][m] - y[m])


def residuals(model, F, names, x, a, b, rs):
    return np.concatenate([fold_resid(model, names, x, f, a, b, rs)[0] for f in F])


def fit(model, F, a=60, b=30, rs=rs_of, max_nfev=300, x0=None):
    """joint least squares over the folds, weights 1 / the burst's rest sd, band ON + a to OFF - b"""
    names, x0d, lo, hi = expand(model, F)
    if x0 is not None:
        for j, n in enumerate(names):
            if n in x0:
                x0d[j] = x0[n]
    sc = np.where(np.abs(x0d) > 0, np.abs(x0d), 1.0)
    fun = lambda z: residuals(model, F, names, z * sc, a, b, rs)
    r = least_squares(fun, np.clip(x0d, lo, hi) / sc, bounds=(lo / sc, hi / sc), x_scale="jac", max_nfev=max_nfev)
    x = r.x * sc
    res = r.fun
    k = len(x)
    dof = max(len(res) - k, 1)
    chi2 = float(np.sum(res ** 2) / dof)
    sd = np.full(k, np.nan)
    corr = None
    try:
        cov = np.linalg.inv(r.jac.T @ r.jac) * chi2
        sd = np.sqrt(np.diag(cov)) * sc
        dd = np.sqrt(np.diag(cov))
        corr = cov / np.outer(dd, dd)
    except np.linalg.LinAlgError:
        pass
    out = dict(model=model.name, names=names, x=x, sd=sd, chi2=chi2, n=len(res), k=k, corr=corr, a=a, b=b,
               rss=float(np.sum(res ** 2)),
               at_bound=[n for n, v, l_, h_ in zip(names, x, lo, hi)
                         if abs(v - l_) < 1e-9 * max(1, abs(l_)) or abs(v - h_) < 1e-9 * max(1, abs(h_))])
    out["per_fold"] = score(model, names, x, F, a, b, rs)
    return out


def score(model, names, x, F, a=60, b=30, rs=rs_of):
    """chi2/n and mean residual (mA) per fold at fixed parameters; a parameter split by a key the fold's value is
    missing from (held out) takes the mean of the fitted keys"""
    by = dict(model.split)
    rows = []
    for f in F:
        nn, xx = [], []
        seen = {}
        for (n, k), v in zip(names, x):
            seen.setdefault(n, []).append((k, v))
        for n, kv in seen.items():
            keys = [k for k, _ in kv]
            if n in by and f[by[n]] not in keys:
                nn.append((n, None)); xx.append(float(np.mean([v for _, v in kv])))
            else:
                for k, v in kv:
                    nn.append((n, k)); xx.append(v)
        w, r = fold_resid(model, nn, np.array(xx), f, a, b, rs)
        rows.append(dict(set=f["set"], level=f["level"], decay=f["decay"], pct=f["pct"], sign=f["sign"], n=len(w),
                         chi2n=float(np.mean(w ** 2)), mean_ma=float(np.mean(r)), rms_ma=float(np.sqrt(np.mean(r ** 2)))))
    return rows


def pooled(rows):
    """chi2 per point over several folds"""
    n = sum(r["n"] for r in rows)
    return sum(r["chi2n"] * r["n"] for r in rows) / n


def value(res, name, key=None):
    """one fitted parameter of a fit result"""
    return float(res["x"][res["names"].index((name, key))])


def values(res, name):
    """every key of one fitted parameter, {key: value}"""
    return {k: float(v) for (n, k), v in zip(res["names"], res["x"]) if n == name}


# the candidate window models; l in H, g in S, currents in A. kc is held at 0.9 wherever the onset step is used
# (only kc ic is identified, the step's size in A); mem 1 unless a fit names another
L0 = (0.6e-3, 0.1e-3, 3e-3)
G = (1e-3, -0.02, 0.05)
CR = (1.0, -20, 50)
A1 = (0.0, -50, 50)
A2 = (0.0, -200, 200)
IC = (0.002, 1e-5, 0.03)
LB = (2e-3, 1e-5, 0.2)
GB = (1.0, 1e-4, 100.0)
KC = {"kc": 0.9, "mem": 1}
MEM = {"mem": 1}


def suite(kind):
    """kind 'iron' (the bare coil, the motor, the controls: the Rayleigh family) or 'can' (the magnet-biased
    per-sign quadratic family)"""
    if kind == "iron":
        return {
            "RL": Model("R-L", {"l0": L0}, fixed=MEM),
            "LG": Model("L||G", {"l0": L0, "g": G}, fixed=MEM),
            "LGb": Model("L||G + rung", {"l0": L0, "g": G, "lb": LB, "gb": GB}, fixed=MEM),
            "R": Model("Rayleigh", {"l0": L0, "cr": CR, "a1": A1}, fixed=MEM),
            "RG": Model("Rayleigh + G", {"l0": L0, "cr": CR, "a1": A1, "g": G}, fixed=MEM),
            "RS": Model("Rayleigh + step", {"l0": L0, "cr": CR, "a1": A1, "ic": IC}, fixed=KC),
            "RSG": Model("Rayleigh + step + G", {"l0": L0, "cr": CR, "a1": A1, "ic": IC, "g": G}, fixed=KC),
            "RGb": Model("Rayleigh + G + rung", {"l0": L0, "cr": CR, "a1": A1, "g": G, "lb": LB, "gb": GB}, fixed=MEM),
            "RSb": Model("Rayleigh + step + rung", {"l0": L0, "cr": CR, "a1": A1, "ic": IC, "lb": LB, "gb": GB}, fixed=KC),
            "Rb": Model("Rayleigh + rung", {"l0": L0, "cr": CR, "a1": A1, "lb": LB, "gb": GB}, fixed=MEM),
            "Lb": Model("L + rung", {"l0": L0, "lb": LB, "gb": GB}, fixed=MEM),
            "RSGb": Model("Rayleigh + step + G + rung", {"l0": L0, "cr": CR, "a1": A1, "ic": IC, "g": G, "lb": LB, "gb": GB}, fixed=KC),
        }
    ps = (("a1", "sign"), ("a2", "sign"))
    return {
        "RLs": Model("R-L per sign", {"l0": L0}, split=(("l0", "sign"),), fixed=MEM),
        "LGs": Model("L||G per sign", {"l0": L0, "g": G}, split=(("l0", "sign"), ("g", "sign")), fixed=MEM),
        "Q": Model("quad per sign", {"l0": L0, "a1": A1, "a2": A2}, split=ps, fixed=MEM),
        "QG": Model("quad per sign + G", {"l0": L0, "a1": A1, "a2": A2, "g": G}, split=ps, fixed=MEM),
        "QS": Model("quad per sign + step", {"l0": L0, "a1": A1, "a2": A2, "ic": IC}, split=ps, fixed=KC),
        "QSG": Model("quad per sign + step + G", {"l0": L0, "a1": A1, "a2": A2, "ic": IC, "g": G}, split=ps, fixed=KC),
        "QRG": Model("quad per sign + Rayleigh + G", {"l0": L0, "a1": A1, "a2": A2, "cr": CR, "g": G}, split=ps, fixed=MEM),
        "QGb": Model("quad per sign + G + rung", {"l0": L0, "a1": A1, "a2": A2, "g": G, "lb": LB, "gb": GB}, split=ps, fixed=MEM),
        "QSb": Model("quad per sign + step + rung", {"l0": L0, "a1": A1, "a2": A2, "ic": IC, "lb": LB, "gb": GB}, split=ps, fixed=KC),
        "QRGb": Model("quad per sign + Rayleigh + G + rung", {"l0": L0, "a1": A1, "a2": A2, "cr": CR, "g": G, "lb": LB, "gb": GB}, split=ps, fixed=MEM),
        "QRb": Model("quad per sign + Rayleigh + rung", {"l0": L0, "a1": A1, "a2": A2, "cr": CR, "lb": LB, "gb": GB}, split=ps, fixed=MEM),
        "QRSG": Model("quad per sign + Rayleigh + step + G", {"l0": L0, "a1": A1, "a2": A2, "cr": CR, "ic": IC, "g": G}, split=ps, fixed=KC),
    }


KIND = {"ra": "iron", "bs-ra": "iron", "bs-rab": "iron", "bare": "iron", "motor": "iron", "coil-noring-1": "iron",
        "can": "can", "coil-can-1": "can"}
BAND = {"motor": 150}                        # the fast-decay band's start: past the ring's tail on the motor


def band_of(s):
    return BAND.get(s, 60)


def model_of(s, key, mem=None):
    m = suite(KIND[s])[key]
    if mem is None:
        return m
    return Model(m.name, m.spec, m.build, m.split, {**m.fixed, "mem": mem})


def heldout(s, key, FF, SL=(), holdout=True):
    """one model on one set, fast decay: the joint fit over every rail, duty and sign, then (holdout) the fits
    from two rails scored on the third and from one duty scored on the other, starting from the joint fit; the
    joint fit's prediction of the set's slow decay with no refit under each memory rule"""
    a = band_of(s)
    m = model_of(s, key)
    out = {"set": s, "key": key, "band": a}
    j = fit(m, FF, a=a)
    out["joint"] = j
    x0 = {n: v for n, v in zip(j["names"], j["x"])}
    if SL:
        out["slow_pred"] = {mem: score(model_of(s, key, mem), j["names"], j["x"], list(SL), a=60) for mem in (0, 1, 2)}
    ho = {}
    if holdout:
        levels = sorted({f["level"] for f in FF})
        if len(levels) > 1:
            for lv in levels:
                tr = [f for f in FF if f["level"] != lv]; te = [f for f in FF if f["level"] == lv]
                r = fit(m, tr, a=a, x0=x0)
                ho[("rail", lv)] = dict(fit=r, test=score(m, r["names"], r["x"], te, a=a))
        for pct in (25, 40):
            tr = [f for f in FF if f["pct"] == pct]; te = [f for f in FF if f["pct"] != pct]
            r = fit(m, tr, a=a, x0=x0)
            ho[("duty_from", pct)] = dict(fit=r, test=score(m, r["names"], r["x"], te, a=a))
    out["heldout"] = ho
    return out


def slow_rfree(s, key, mem, joint, SL):
    """the fast-decay joint fit's shape held, the series R alone refitted per capture to the slow-decay folds
    (the plateau is R's, the shape the model's), band ON+60 to OFF-30"""
    p = {n: v for (n, k), v in zip(joint["names"], joint["x"]) if k is None}
    per_sign = {(n, k): v for (n, k), v in zip(joint["names"], joint["x"]) if k is not None}

    def build(q, f):
        d = {**p, **{n: v for (n, k), v in per_sign.items() if k == f["sign"]}, "kc": 0.9, "mem": mem}
        d.update(q)
        return d
    mr = Model(f"{key} mem{mem} R free", {"rs": (5.0, 1.0, 10.0)}, build, split=(("rs", "set"),))
    return fit(mr, list(SL), a=60)


def joint_model(s, key, mem):
    base = suite(KIND[s])[key]
    spec = dict(base.spec)
    split = tuple(base.split)
    if s == "motor":
        spec["rs_slow"] = (4.4, 3.0, 6.0)
        split = split + (("rs_slow", "set"),)

    def build(p, f):
        q = dict(p)
        if "rs_slow" in q:
            if f["decay"] == "slow":
                q["rs"] = q["rs_slow"]
            del q["rs_slow"]
        return q
    return Model(f"{base.name} mem{mem} fast+slow", spec, build, split, {**base.fixed, "mem": mem})


def _joint_run(m, a_fast, folds_fast, folds_slow, x0):
    # the fast band and the slow band differ (the ring to ON+150 on the motor; slow decay from ON+60)
    names, x0d, lo, hi = expand(m, folds_fast + folds_slow)
    for j_, n in enumerate(names):
        if n in x0:
            x0d[j_] = x0[n]
    sc = np.where(np.abs(x0d) > 0, np.abs(x0d), 1.0)
    fun = lambda z: np.concatenate([residuals(m, folds_fast, names, z * sc, a_fast, 30, rs_of),
                                    residuals(m, folds_slow, names, z * sc, 60, 30, rs_of)])
    r = least_squares(fun, np.clip(x0d, lo, hi) / sc, bounds=(lo / sc, hi / sc), x_scale="jac", max_nfev=300)
    x = r.x * sc
    chi2 = float(np.sum(r.fun ** 2) / max(len(r.fun) - len(x), 1))
    corr = None
    try:
        cov = np.linalg.inv(r.jac.T @ r.jac) * chi2
        sd = np.sqrt(np.diag(cov)) * sc
        dd = np.sqrt(np.diag(cov))
        corr = cov / np.outer(dd, dd)
    except np.linalg.LinAlgError:
        sd = np.full(len(x), np.nan)
    return dict(names=names, x=x, sd=sd, chi2=chi2, n=len(r.fun), k=len(x), model=m.name, corr=corr,
                per_fold=score(m, names, x, folds_fast, a_fast, 30, rs_of) + score(m, names, x, folds_slow, 60, 30, rs_of))


def joint(s, key, mem, FF, SL, x0):
    """the joint fast + slow fit with the memory rule fixed, the coils' R held, the motor's slow-decay R free per
    capture (its fast decay keeps the stall R, which a discontinuous window barely sees); then each rail held out
    (coils) and each duty held out, scored on the left-out folds of both decays. x0: the fast-decay joint fit."""
    a_fast = band_of(s)
    m = joint_model(s, key, mem)
    out = {"set": s, "key": key, "mem": mem}
    j = _joint_run(m, a_fast, FF, SL, x0)
    out["joint"] = j
    xj = {n: v for n, v in zip(j["names"], j["x"])}
    ho = {}
    levels = sorted({f["level"] for f in FF})
    tests = [("rail", lv) for lv in levels] if len(levels) > 1 else []
    tests += [("duty_from", 25), ("duty_from", 40)]
    for kind, v in tests:
        if kind == "rail":
            tr_f = [f for f in FF if f["level"] != v]; tr_s = [f for f in SL if f["level"] != v]
            te_f = [f for f in FF if f["level"] == v]; te_s = [f for f in SL if f["level"] == v]
        else:
            tr_f = [f for f in FF if f["pct"] == v]; tr_s = [f for f in SL if f["pct"] == v]
            te_f = [f for f in FF if f["pct"] != v]; te_s = [f for f in SL if f["pct"] != v]
        r = _joint_run(m, a_fast, tr_f, tr_s, xj)
        sc = score(m, r["names"], r["x"], te_f, a_fast, 30, rs_of) + score(m, r["names"], r["x"], te_s, 60, 30, rs_of)
        ho[(kind, v)] = dict(fit=r, test=sc)
    out["heldout"] = ho
    return out


def rail_test(s, key, joint_fast, FF):
    """the fast-decay joint fit's L(i) shape held, L0 and an edge term refitted per rail, once as a conductance G
    (the edge current G V) and once as an onset step (a fixed current kc ic); the edge current against the
    window's volts as a line"""
    shape = {(n, k): v for (n, k), v in zip(joint_fast["names"], joint_fast["x"]) if n in ("cr", "a1", "a2")}
    out = {}
    for form in ("G", "step"):
        spec = {"l0": (joint_fast["x"][0], 1e-4, 3e-3), **({"g": (1e-3, -0.02, 0.05)} if form == "G" else {"ic": (0.002, 1e-6, 0.03)})}

        def build(p, f, form=form):
            q = {n: v for (n, k), v in shape.items() if k is None or k == f["sign"]}
            q.update(p, kc=0.9, mem=2)
            return q
        m = Model(f"{key} {form} per rail", spec, build)
        rows = []
        for lv in LEVELS:
            sub = [f for f in FF if f["level"] == lv]
            r = fit(m, sub, a=60)
            v = np.mean([f["v_on"] for f in sub])
            e = r["x"][1] * (v if form == "G" else 0.9) * 1e3
            se = r["sd"][1] * (v if form == "G" else 0.9) * 1e3
            rows.append(dict(level=lv, v_on=float(v), edge_ma=float(e), se_ma=float(se), chi2=r["chi2"], l0=float(r["x"][0])))
        a = np.array([(q["v_on"], q["edge_ma"]) for q in rows])
        X = np.c_[np.ones(3), a[:, 0]]
        c, *_ = np.linalg.lstsq(X, a[:, 1], rcond=None)
        res = a[:, 1] - X @ c
        s2 = max(np.sum(res ** 2) / 1, 1e-6)
        cov = np.linalg.inv(X.T @ X) * s2
        out[form] = dict(rows=rows, intercept=(float(c[0]), float(np.sqrt(cov[0, 0]))), slope=(float(c[1]), float(np.sqrt(cov[1, 1]))))
    return out


def variants(s, key, FF, x0):
    """the fast-decay joint fit again with the band's start moved and with the series R moved (the motor:
    rest-resistance's meter ohms less its lead zero, and 0.95 x the stall R; the coils: the hand ohms +/- 0.1 ohm)"""
    m = model_of(s, key)
    out = {}
    bands = (100, 150, 200, 250) if s == "motor" else (30, 60, 100, 150)
    for a in bands:
        out[("band", a)] = fit(m, FF, a=a, x0=x0)
    if s == "motor":
        r_alt = RR["r_ohms_median_ohm"]["value"] - RR["ohms_lead_zero_ohm"]["value"]
        rvars = {"meter ohms": lambda f: r_alt, "stall R x 0.95": lambda f: 0.95 * rs_of(f)}
    else:
        rvars = {"R - 0.1": lambda f: rs_of(f) - 0.1, "R + 0.1": lambda f: rs_of(f) + 0.1}
    for name, fn in rvars.items():
        out[("R", name)] = fit(m, FF, a=band_of(s), rs=fn, x0=x0)
    return out


@functools.lru_cache(maxsize=None)
def cached_folds(name):
    """folds(name), built once per process; the fits only read them"""
    return folds(name)


def fast_folds(s):
    return pick(cached_folds(s), "fast")


def slow_folds(s):
    """a set's slow-decay folds; the motor's are fast-1's slow groups, capture-1 at hole 11 and reversal-1"""
    SL = pick(cached_folds(s), "slow")
    if s == "motor":
        SL = SL + list(cached_folds("cap-11")) + list(cached_folds("rev-11"))
    return SL


def tests_of(s):
    """the held-out tests a set allows: each rail out where there are several, each duty out"""
    levels = sorted({f["level"] for f in fast_folds(s)})
    return ([("rail", lv) for lv in levels] if len(levels) > 1 else []) + [("duty_from", 25), ("duty_from", 40)]


def _split(F, test):
    kind, v = test
    if kind == "rail":
        return [f for f in F if f["level"] != v], [f for f in F if f["level"] == v]
    return [f for f in F if f["pct"] == v], [f for f in F if f["pct"] != v]


def job_fast(s, key):
    """the fast-decay joint fit of one model on one set, and its prediction of the set's slow decay with no refit
    under memory rules 1 and 2"""
    t = time.perf_counter()
    j = fit(model_of(s, key), fast_folds(s), a=band_of(s))
    seconds = time.perf_counter() - t                  # what the fit cost, to order the later waves by
    SL = slow_folds(s)
    slow = {mem: score(model_of(s, key, mem), j["names"], j["x"], SL, a=60) for mem in (1, 2)} if SL else {}
    return dict(set=s, key=key, joint=j, slow_pred=slow, seconds=seconds)


def job_fast_refit(s, key, test, x0):
    """one held-out refit of the fast-decay fit from x0, scored on the folds left out"""
    m = model_of(s, key)
    tr, te = _split(fast_folds(s), test)
    r = fit(m, tr, a=band_of(s), x0=x0)
    return dict(fit=r, test=score(m, r["names"], r["x"], te, a=band_of(s)))


def job_slow_rfree(s, key, mem, joint_fast):
    return slow_rfree(s, key, mem, joint_fast, slow_folds(s))


def job_joint(s, key, mem, x0):
    """the joint fast + slow fit (joint() without its held-out refits)"""
    return _joint_run(joint_model(s, key, mem), band_of(s), fast_folds(s), slow_folds(s), x0)


def job_joint_refit(s, key, mem, test, xj):
    """one held-out refit of the joint fast + slow fit, scored on the left-out folds of both decays"""
    m = joint_model(s, key, mem)
    a = band_of(s)
    trf, tef = _split(fast_folds(s), test)
    trs, tes = _split(slow_folds(s), test)
    r = _joint_run(m, a, trf, trs, xj)
    return dict(fit=r, test=score(m, r["names"], r["x"], tef, a, 30, rs_of) + score(m, r["names"], r["x"], tes, 60, 30, rs_of))


def job_rail_test(s, key, joint_fast):
    return rail_test(s, key, joint_fast, fast_folds(s))


def job_variant(s, key, which, x0):
    """one variant of the fast-decay fit: ("band", start) or ("R", name)"""
    m = model_of(s, key)
    if which[0] == "band":
        return fit(m, fast_folds(s), a=which[1], x0=x0)
    if s == "motor":
        r_alt = RR["r_ohms_median_ohm"]["value"] - RR["ohms_lead_zero_ohm"]["value"]
        fn = {"meter ohms": lambda f: r_alt, "stall R x 0.95": lambda f: 0.95 * rs_of(f)}[which[1]]
    else:
        fn = {"R - 0.1": lambda f: rs_of(f) - 0.1, "R + 0.1": lambda f: rs_of(f) + 0.1}[which[1]]
    return fit(m, fast_folds(s), a=band_of(s), rs=fn, x0=x0)


def variant_list(s):
    bands = (100, 150, 200, 250) if s == "motor" else (30, 60, 100, 150)
    return [("band", a) for a in bands] + [("R", n) for n in (("meter ohms", "stall R x 0.95") if s == "motor" else ("R - 0.1", "R + 0.1"))]


def job_offset(key, joint_fast, i):
    return offset_shares(key, joint_fast, fast_folds("motor")[i])


def job_ind(s, models, part, nparts):
    """inductance's estimators on every nparts-th burst of one hole-11 run from `part`, data and models"""
    fb = {(f["pct"], f["sign"]): f for f in pick(cached_folds(s), "slow")}
    rows = ind_compare(s, IND_RUNS[s], fb, {k: v[s] for k, v in models.items()}, part, nparts)
    return rows


def _call(job):
    fn, args, kwargs = job
    return fn(*args, **kwargs)


def run_parallel(jobs, workers=None, chunksize=1, cost=None):
    """jobs: (function, args, kwargs) triples of this module's functions, run on separate processes, the costliest
    first when a cost per job is given; results in the order given"""
    workers = workers or os.cpu_count()
    order = list(range(len(jobs))) if cost is None else sorted(range(len(jobs)), key=lambda i: -cost[i])
    with ProcessPoolExecutor(max_workers=workers) as ex:
        res = list(ex.map(_call, [jobs[i] for i in order], chunksize=chunksize))
    out = [None] * len(jobs)
    for i, r in zip(order, res):
        out[i] = r
    return out


# ---------------------------------------------------------------- the spinning steady state

def share(duty):
    """the share of the period the bridge drives in slow decay, its edges at sample-timing's lags"""
    return (2 * duty / 100 * ARR - SLOW_ON + SLOW_OFF) / P


def steady(p, duty, v_on, target_ma, r_on=0.5, r_brk=None, sign=1, per=50, last=16):
    """slow decay to steady state at `duty` %, an EMF solved by a secant so the period mean of the series current
    equals target_ma; returns the crest sample (the shunt through the amplifier at the board's crest instant)
    less that mean, the mean, the EMF, and the window line's ON and OFF values as bemf-coast reads them"""
    r_brk = BSR["low_side_loop_ohm"]["value"] if r_brk is None else r_brk
    q15 = int(round(duty / 100 * 32768))
    half = int(BOARD.drive_ticks(q15))
    fold = dict(decay="slow", half=half, on_lag=SLOW_ON, off_lag=SLOW_OFF, v_on=v_on + r_on * target_ma * 1e-3,
                r_on=r_on, v_brk=0.0, r_brk=r_brk, sign=sign)
    modes, t0 = schedule("slow", half, SLOW_ON, SLOW_OFF, per)
    tt = np.arange(len(modes)) + t0
    lastm = tt >= (per - last) * P - P // 2

    def run(e):
        _t0, sh, ser, _x = simulate(p, fold, emf=e, modes=modes, t0=t0)
        return sh, ser, ser[lastm].mean()
    e0 = max(share(duty) * v_on - p["rs"] * target_ma * 1e-3, 0.0)
    e1 = e0 + 0.05
    _, _, m0 = run(e0)
    _, _, m1 = run(e1)
    for _ in range(8):
        if abs(m1 - m0) < 1e-9:
            break
        e2 = e1 + (target_ma - m1) * (e1 - e0) / (m1 - m0)
        e0, m0 = e1, m1
        e1 = e2
        sh, ser, m1 = run(e1)
        if abs(m1 - target_ma) < 0.005:
            break
    sh, ser, mean = run(e1)
    ks = np.arange(per - last, per)
    crest = np.array([sh[np.searchsorted(tt, k * P + CREST_AT)] for k in ks])
    ph = ((tt + P // 2) % P) - P // 2
    on, off = -half + SLOW_ON, half + SLOW_OFF
    w = lastm & (ph > on + 75) & (ph < off - 20)
    c = np.polyfit(ph[w], sh[w], 1)
    return dict(crest_less_mean=float(crest.mean() - mean), mean=float(mean), emf=float(e1),
                on_edge=float(np.polyval(c, on)), off_edge=float(np.polyval(c, off)), slope_ma_us=float(c[0] * TPU),
                ripple_pp=float(ser[lastm].max() - ser[lastm].min()), sh=sh[lastm], ser=ser[lastm], ph=ph[lastm])


def steady_both(p, duty, v_on, target_ma):
    """steady() for both drive signs, the mean of their scalar outputs"""
    res = [steady(p, duty, v_on, target_ma, sign=s) for s in (1, -1)]
    return {k: float(np.mean([r[k] for r in res])) for k in res[0] if not hasattr(res[0][k], "shape")}


def spin_jobs(p, pts):
    """the crest-less-mean prediction at each spinning point, mean of both signs; pts rows of (duty, v_on, mean, rs)"""
    out = []
    for duty, v_on, mean, rs in pts:
        out.append(steady_both(dict(p, rs=rs), duty, v_on, mean) if duty < 100 else None)
    return out


# ---------------------------------------------------------------- the coast as a meter

FS = BOARD.tick_hz
N_R = MOTOR.ripple_per_rev   # ripple events per revolution, slots x brushes, as bemf-coast counts them
SPIN_MS, COAST_FROM, TARGET_MS, MIN_CYCLES, TURN_V = 200, 20, 10, 3, 0.05
RIPPLE_BAND, NBH, OPEN_FRAC = (150.0, 6000.0), 41, 0.3
BC = PAIR / "bemf-coast"
STQ = PAIR / "speed-torque" / "capture-1"


def spectrum(x):
    x = np.asarray(x, float) - np.mean(x)
    n = len(x)
    return np.fft.rfftfreq(8 * n, 1 / FS), np.abs(np.fft.rfft(x * np.hanning(n), 8 * n))


def line_freq(x, lo=RIPPLE_BAND[0], hi=RIPPLE_BAND[1]):
    f, X = spectrum(x)
    m = (f >= lo) & (f <= hi)
    return float(f[m][int(np.argmax(X[m]))])


def crossings(x, f0, frac=0.35):
    hi = min(f0 * (1 + frac), 0.95 * FS / 2)
    sos = butter(2, [f0 * (1 - frac), hi], btype="band", fs=FS, output="sos")
    y = sosfiltfilt(sos, np.asarray(x, float) - np.mean(x))
    s = np.flatnonzero((y[:-1] < 0) & (y[1:] >= 0))
    return (s + (-y[s]) / (y[s + 1] - y[s]))[1:-1]


def count_cycles(x, f0, frac=0.35):
    tc = crossings(x, f0, frac)
    if len(tc) < 3:
        return np.nan, 0
    return (len(tc) - 1) / ((tc[-1] - tc[0]) / FS), len(tc) - 1


def read_window(e, start, f_seed, target_ms=TARGET_MS, frac=0.35):
    """the window of whole ripple cycles closest to target_ms from `start`: its frequency, its ends, its cycles"""
    f = f_seed
    for _ in range(6):
        n = max(MIN_CYCLES, int(round(target_ms / 1e3 * f)))
        stop = start + int((n + 3) * FS / f * 1.3)
        if stop > len(e):
            return np.nan, 0, 0, 0
        tc = crossings(e[start:stop], f, frac)
        if len(tc) < n + 1:
            return np.nan, 0, 0, 0
        tc = tc[:n + 1]
        f_new = n / ((tc[-1] - tc[0]) / FS)
        done = abs(f_new / f - 1) < 1e-4
        f = f_new
        if done:
            break
    return f, start + int(np.ceil(tc[0])), start + int(np.floor(tc[-1])), n


def open_rows(e):
    ref = np.convolve(e, np.ones(NBH) / NBH, mode="same")
    bad = (e < OPEN_FRAC * ref) & (ref > TURN_V)
    bad[:COAST_FROM] = False
    return bad


def bridge_rows(e, bad):
    if not bad.any():
        return e.copy()
    x = np.arange(len(e))
    out = e.copy()
    out[bad] = np.interp(x[bad], x[~bad], e[~bad])
    return out


def spin_coast_pairs(df):
    out = []
    for seg, co in df[df["seg"] > 0].groupby("seg"):
        if int(co["cmd_duty_q15"].iloc[0]) != 0:
            continue
        out.append((int(co["dir"].iloc[0]), df[df["seg"] == seg - 1], co))
    return out


def cut_row(co):
    z = np.flatnonzero(co["duty_counts"].to_numpy() == 0)
    return int(z[0]) if len(z) else None


def reduce_spins(folder, last_s=SPIN_MS / 1000):
    """one row per spin: the board's crest current over the spin's last `last_s`, its terminal difference, the
    spin's speed by count, and the coast's speed and EMF slope at the cut (bemf-coast's reduction), plus a speed
    pinned at the cut and a line through three 5 ms windows"""
    df, meta = tel.load(folder, BOARD)
    rest = tel.baseline(df)
    zero_i = rest["current_raw"].median()
    zero_e = (rest["vmotor_a"] - rest["vmotor_b"]).median()
    rows = []
    for d, sp, co in spin_coast_pairs(df):
        w = sp.iloc[-int(last_s * FS):]
        x = w["current_raw"].to_numpy(float)
        row = {"dir": d, "i_board_ma": (x.mean() - zero_i - CS["drive_offset_counts"]["value"]) * (1 / CPA) * 1e3,
               "v_on": ((w["vmotor_a"] - w["vmotor_b"]) * d - zero_e).mean() * (1 / K_CPV),
               "duty_applied": float(BOARD.duty_pct_of_ticks(w["duty_counts"].abs().median()))}
        z = cut_row(co)
        e = (((co["vmotor_a"] - co["vmotor_b"]) * d).to_numpy(float)[z:] - zero_e) * (1 / K_CPV)
        if e[2:20].mean() < TURN_V:
            rows.append(row)
            continue
        e = bridge_rows(e, open_rows(e))
        f0 = line_freq(e[COAST_FROM:COAST_FROM + 200])
        f, i0, i1, n = read_window(e, COAST_FROM, f0)
        nbox = max(int(round(FS / f)), 1)
        es = np.convolve(e, np.ones(nbox) / nbox, mode="same")
        t = np.arange(len(e)) / FS
        p = np.polyfit(t[i0:i1], es[i0:i1], 1)
        fs_, _ = count_cycles(x[-int(0.2 * FS):], f0)
        row.update(coast_rev_s=f / N_R, dedt_v_per_s=float(p[0]), emf_v=float(e[i0:i1].mean()), spin_rev_s=fs_ / N_R,
                   t_mid_s=(i0 + i1) / 2 / FS)
        ws, ts_, st = [], [], COAST_FROM
        for _ in range(4):
            fw, a0, a1, nw = read_window(e, st, f if not ws else ws[-1] * N_R, target_ms=5)
            if not np.isfinite(fw):
                break
            ws.append(fw / N_R); ts_.append((a0 + a1) / 2 / FS); st = a1
        if len(ws) >= 3:
            q = np.polyfit(ts_[:3], ws[:3], 1)
            row.update(alpha_win=-2 * np.pi * q[0], rev_s_cut_win=float(np.polyval(q, 0.0)))
        rows.append(row)
    return rows


def meter_spins(log, t0, t1):
    """bemf-coast's meter reader: the spins the meter saw inside one cycle, by direction, packets after the first,
    less any under 85 % of their median, in mA"""
    p = log[(log["t_host"] > t0 - 1) & (log["t_host"] < t1 + 1) & (log["value"].abs() > 2e-3)].copy()
    p["spin"] = (p["t_host"].diff() > 0.8).cumsum()
    out = {}
    for _, g in p.groupby("spin"):
        v = g["value"].to_numpy() * 1e3
        keep = np.abs(v[1:]) > 0.85 * np.median(np.abs(v[1:]))
        out[int(np.sign(v.mean()))] = float(np.abs(v[1:])[keep].mean())
    return out


def _coast_cycle(run, folder, C, last, log_part):
    rows = []
    mt = meter_spins(log_part, C["t_start"], C["t_end"]) if log_part is not None else {}
    for r in reduce_spins(folder, last):
        r.update(run=run, k=int(C["k"]), level=float(C["volts_set"]), duty=int(C["duty_pct"]), meter_ma=mt.get(r["dir"], np.nan))
        rows.append(r)
    return rows


def coast_jobs():
    """one job per cycle of bemf-coast's capture-1, meter-1 and high-1 and speed-torque's run-ups"""
    jobs = []
    for run in ("capture-1", "meter-1", "high-1"):
        root = BC / run
        cyc = pd.read_csv(root / "cycles.csv")
        log = pd.read_csv(root / "dmm.csv") if (root / "dmm.csv").exists() else None
        for _, C in cyc.iterrows():
            part = None if log is None else log[(log["t_host"] > C["t_start"] - 1) & (log["t_host"] < C["t_end"] + 1)]
            jobs.append((_coast_cycle, (run, root / C["folder"], C.to_dict(), 0.2 if run == "capture-1" else 2.0, part), {}))
    cyc = pd.read_csv(STQ / "cycles.csv")
    for _, C in cyc.iterrows():
        jobs.append((_coast_cycle, ("speed-torque", STQ / C["folder"], C.to_dict(), 0.1, None), {}))
    return jobs


def coast_table(rows):
    """the coast's drag at the cut as a current, J alpha / Kt with speed-torque's J and Kt = bemf-coast's Ke, three
    ways: the EMF slope, the spin's count pinned at the cut, and the line through three 5 ms windows"""
    T = pd.DataFrame([r for part in rows for r in part])
    T["alpha"] = -T["dedt_v_per_s"] / KE
    T["drag_cut_ma"] = J * T["alpha"] / KE * 1e3
    T["alpha_pin"] = 2 * np.pi * (T["spin_rev_s"] - T["coast_rev_s"]) / T["t_mid_s"]
    T["drag_pin_ma"] = J * T["alpha_pin"] / KE * 1e3
    T["drag_win_ma"] = J * T["alpha_win"] / KE * 1e3
    return T


def spin_points(T):
    """every spinning point with its measured mean: the meter in the meter runs (meter-1, high-1), and in capture-1
    (no meter) the coast's pinned drag at the cut scaled by the meter runs' own mean-to-drag ratio; then
    spinning-1's three slow-decay points from bemf-coast's result"""
    met = T[T["run"].isin(["meter-1", "high-1"])]
    ratio = (met["meter_ma"] / met["drag_pin_ma"])[met["duty"] < 100]
    c = float(ratio.mean()); c_sd = float(ratio.std())
    rows = []
    for (run, lv, du), g in T.groupby(["run", "level", "duty"]):
        if run == "speed-torque":
            continue
        r = dict(run=run, level=lv, duty=du, v_on=g["v_on"].mean(), board=g["i_board_ma"].mean(), rev_s=g["spin_rev_s"].mean(), n=len(g))
        if run == "capture-1":
            r["mean"] = c * g["drag_pin_ma"].mean()
            r["mean_err"] = np.hypot(c_sd / np.sqrt(len(ratio)) * g["drag_pin_ma"].mean(), c * g["drag_pin_ma"].std() / np.sqrt(len(g)))
            r["loop"] = "motor"
        else:
            r["mean"] = g["meter_ma"].mean()
            r["mean_err"] = g["meter_ma"].std() / np.sqrt(len(g))
            r["loop"] = "meter"
        r["gap"] = r["board"] - r["mean"]
        rows.append(r)
    Pt = pd.DataFrame(rows)
    MO = meter_ohm(Pt)
    for pt, d in BEMF["spinning_by_point"].items():
        s_ = d["slow"]
        lv, du = float(pt.split()[0]), int(pt.split()[2])
        Pt = pd.concat([Pt, pd.DataFrame([dict(run="spinning-1", level=lv, duty=du, v_on=s_["v_on_v"]["value"],
                                                board=s_["crest_sample_board_zero_over_meter_ma"]["value"] + s_["meter_ma"]["value"],
                                                mean=s_["meter_ma"]["value"], mean_err=np.nan, loop="meter", n=6, rev_s=np.nan,
                                                on_edge_meas=s_["on_edge_ma"]["value"], off_edge_meas=s_["off_edge_ma"]["value"],
                                                crest_line_meas=s_["crest_over_meter_ma"]["value"])])], ignore_index=True)
    Pt["gap"] = Pt["board"] - Pt["mean"]
    return Pt, MO, c, c_sd


def meter_ohm(Pt):
    """the meter and its leads: each meter point paired with capture-1 at the same rail and duty, share x V - Ke w
    = R I, the loop R of each, the meter's the difference"""
    out = []
    for _, m in Pt[Pt["loop"] == "meter"].iterrows():
        c = Pt[(Pt["run"] == "capture-1") & (Pt["level"] == m["level"]) & (Pt["duty"] == m["duty"])]
        if not len(c) or m["duty"] >= 100:
            continue
        c = c.iloc[0]
        s = share(m["duty"])
        r_c = (s * c["v_on"] - KE * 2 * np.pi * c["rev_s"]) / (c["mean"] * 1e-3)
        r_m = (s * m["v_on"] - KE * 2 * np.pi * m["rev_s"]) / (m["mean"] * 1e-3)
        out.append(dict(level=m["level"], duty=m["duty"], loop_nometer=r_c, loop_meter=r_m, meter_ohm=r_m - r_c))
    return pd.DataFrame(out)


def nlc_curve(rev_s, level="5 V"):
    """no-load-current's term-out drag curve, a + b ln(w / 100 rev/s), mA"""
    d = NLC["drive_drag_by_level_meter"][level]
    w = 2 * np.pi * np.asarray(rev_s, float)
    return (d["a_a"]["value"] + d["b_a"]["value"] * np.log(w / (2 * np.pi * 100))) * 1e3


# ---------------------------------------------------------------- the window offset taken apart

OFFSET_BAND_STARTS = (100, 150, 200, 250)
OFFSET_BAND_END = 20
PARTS = {"L(i)": ("cr", "a1", "a2"), "loss": ("g", "ic"), "rung": ("lb",)}


def offset_reads(x, y, half):
    """teardown's and inductance's reads: a line from each band start to 20 ticks before the commanded end, read at
    the fast-decay ON edge, and a quadratic from 150; x in ticks from the window center"""
    on = -half + FAST_ON
    out = {}
    for bs in OFFSET_BAND_STARTS:
        w = (x >= -half + bs) & (x <= half - OFFSET_BAND_END)
        out[f"line_{bs}"] = float(np.polyval(np.polyfit(x[w], y[w], 1), on))
    w = (x >= -half + 150) & (x <= half - OFFSET_BAND_END)
    out["quad"] = float(np.polyval(np.polyfit(x[w], y[w], 2), on))
    out["line"] = out["line_150"]
    return out


def _fold_xy(f, y=None):
    keep = f["t"] >= P - ARR
    return f["x"][keep], (f["i"] if y is None else y)[keep]


def _pred(p, f):
    t0, sh, ser, _ = simulate(p, f)
    idx = np.clip((f["t"] - t0).astype(int), 0, len(sh) - 1)
    return sh[idx]


def edge_current(p, f):
    """the model's shunt from ON+10 to ON+60 against a plain R-L of the same L0 and R, mean over the full windows"""
    t0, sh, ser, _ = simulate(p, f)
    _t, sh0, _s, _x = simulate(dict(rs=p["rs"], l0=p["l0"], mem=1), f)
    tt = np.arange(len(sh)) + t0
    ph = ((tt + ARR) % P) - ARR
    on = -f["half"] + f["on_lag"]
    w = (tt >= P - ARR) & (ph >= on + 10) & (ph < on + 60)
    return float(np.mean(sh[w] - sh0[w]))


def _strip(p, parts):
    q = dict(p)
    for part in parts:
        for k in PARTS[part]:
            q[k] = 0.0
    return q


def offset_shares(key, joint_fast, f):
    """the reads on the data fold, on the model at the same samples, and on the model with its parts switched off
    in every order (Shapley shares) for one fold"""
    p = {n: v for (n, k), v in zip(joint_fast["names"], joint_fast["x"]) if k is None or k == f["sign"]}
    p.update(kc=0.9, mem=1, rs=rs_of(f))
    x, yd = _fold_xy(f)
    rd = offset_reads(x, yd, f["half"])
    present = [k for k, ks in PARTS.items() if any(abs(p.get(q, 0.0)) > 0 for q in ks)]
    val = {}
    for r in range(len(present) + 1):
        for sub in itertools.combinations(present, r):
            y = _pred(_strip(p, [q for q in present if q not in sub]), f)
            val[frozenset(sub)] = offset_reads(*_fold_xy(f, y), f["half"])
    row = dict(key=key, pct=f["pct"], sign=f["sign"], v_on=f["v_on"], data_line=rd["line"], data_quad=rd["quad"],
               data_line_lo=min(rd[f"line_{b}"] for b in OFFSET_BAND_STARTS), data_line_hi=max(rd[f"line_{b}"] for b in OFFSET_BAND_STARTS),
               model_line=val[frozenset(present)]["line"], model_quad=val[frozenset(present)]["quad"],
               rl_line=val[frozenset()]["line"], rl_quad=val[frozenset()]["quad"], edge_ma=edge_current(p, f))
    n = len(present)
    for part in present:
        for rd_ in ("line", "quad"):
            sh = 0.0
            others = [q for q in present if q != part]
            for r in range(n):
                for sub in itertools.combinations(others, r):
                    w = math.factorial(r) * math.factorial(n - r - 1) / math.factorial(n)
                    sh += w * (val[frozenset(sub) | {part}][rd_] - val[frozenset(sub)][rd_])
            row[f"{part}_{rd_}"] = sh
    return row


# ---------------------------------------------------------------- inductance's estimators

ON_LAG = np.mean([ST["on_edge_lag_7v4"]["value"], ST["on_edge_lag_3v7"]["value"]])
TAP_ON_LAG = np.mean([ST["tap_on_edge_lag_7v4"]["value"], ST["tap_on_edge_lag_3v7"]["value"]])
OFF_LAG = np.mean([ST["off_edge_lag_7v4"]["value"], ST["off_edge_lag_3v7"]["value"]])
SETTLE = np.mean([ST["shunt_settle_7v4"]["value"], ST["shunt_settle_3v7"]["value"]])
K = ST["window_scale"]["value"]
T_S = P / TPU * 1e-6
IND_RUNS = {"cap-11": IND / "capture-1" / "hole-11", "motor": IND / "fast-1" / "hole-11", "rev-11": IND / "reversal-1" / "hole-11"}


def _ind_samples(df):
    m, ph, sh, ta, tb = bu.phase(df, ARR)
    half = int(BOARD.drive_ticks(m["step_q15"]))
    k = np.arange(len(df))
    phi0 = m["start_cnt"] if m["start_dir"] == 0 else P - m["start_cnt"]
    tick = phi0 + bu.STEP_TICKS * k + bu.APERTURE_CLK * bu.TICKS_PER_ADC_CLK
    trough = P * np.ceil(tick[m["step_index"]] / P)
    c = ARR
    origin = trough + c - half
    n = np.floor((tick - origin) / P).astype(int)
    last = int(np.floor((tick[-1] - origin) / P))
    tc = ((tick - c) % P + P // 2) % P - P // 2
    stream = np.where(sh, "sh", np.where(ta, "ta", "tb"))
    return pd.DataFrame({"stream": stream, "code": df["code"].to_numpy(float), "tick": tick, "tc": tc, "n": n, "full": n < last}), m, half


def ind_periods(df, sign):
    """inductance's reading of a slow-decay burst, the edge volt-ticks left out: per full period the window line at
    the window's middle, the period's volts, the window's volts and slope"""
    s, m, half = _ind_samples(df)
    zero = bu.zero(df, ARR)
    a, b = -half + TAP_ON_LAG, half + OFF_LAG
    on_span = b - a
    s = s[(s["n"] >= 0) & s["full"]].assign(y=lambda d: d["code"] - zero)
    settled = (s["tc"] >= -half + SETTLE) & (s["tc"] <= half)
    clear = (s["tc"] >= b + SETTLE - ON_LAG) | (s["tc"] <= -half - 10)
    rows = []
    for n, g in s.groupby("n"):
        sh = g[(g["stream"] == "sh") & settled[g.index]]
        line = np.polyfit(sh["tc"], sh["y"], 1)
        i_on = np.polyval(line, (a + b) / 2) / CPA
        d_on, d_off = ((g[(g["stream"] == "ta") & w[g.index]]["code"].mean() - g[(g["stream"] == "tb") & w[g.index]]["code"].mean())
                       for w in (settled, clear))
        v_on, v_off = ((sign * d - C_OFF) / K_CPV for d in (d_on, d_off))
        rows.append({"n": n, "i_a": i_on, "v_v": (v_on * on_span + v_off * (P - on_span)) / P, "v_on_v": v_on,
                     "slope_a_per_s": line[0] * TPU * 1e6 / CPA})
    return pd.DataFrame(rows)


def ind_fit(Pd, pairs=None):
    """R and L over the first `pairs` period pairs, each over the window's scale k"""
    i, v = Pd["i_a"].to_numpy(), Pd["v_v"].to_numpy()
    p = len(i) - 1 if pairs is None else pairs
    di, im = i[1:p + 1] - i[:p], (i[1:p + 1] + i[:p]) / 2
    (r, l), *_ = np.linalg.lstsq(np.c_[im, di / T_S], v[:p], rcond=None)
    return r / K, l / K * 1e3


def ind_reads(df, sign):
    Pd = ind_periods(df, sign)
    r, l = ind_fit(Pd)
    rip = (Pd["v_on_v"].to_numpy() - motor_r(11) * K * Pd["i_a"].to_numpy()) / Pd["slope_a_per_s"].to_numpy() / K * 1e3
    scan = {p: ind_fit(Pd, p) for p in range(3, len(Pd))}
    return dict(r=r, l=l, l_rip=float(np.median(rip[-3:])), l_rip_first=float(np.median(rip[:3])), i_last=Pd["i_a"].iloc[-1],
                l3=scan[3][1], r3=scan[3][0], lmax=scan[max(scan)][1], rmax=scan[max(scan)][0])


def synth_burst(df, y_ma):
    """the burst with its shunt codes replaced by the model (mA at the shunt samples), the zero kept, taps as measured"""
    z = bu.zero(df, ARR)
    d = df.copy()
    k = np.arange(len(df))
    sh = np.flatnonzero(k % 2 == 0)
    code = d["code"].to_numpy(float)
    code[sh] = z + y_ma * 1e-3 * CPA
    d["code"] = np.round(code)
    return d


def ind_bursts(folder):
    """every slow-decay burst but the first of its group"""
    out = []
    for g in sorted(p for p in Path(folder).iterdir() if (p / "meta.json").exists()):
        decay = json.loads((g / "meta.json").read_text()).get("decay", "slow")
        for j, df in enumerate(bu.load(g)):
            if j == 0 or decay != "slow":
                continue
            m = bu.meta(df)
            out.append(dict(group=g.name, burst=j, df=df, sign=1 if m["step_q15"] > 0 else -1,
                            pct=int(round(abs(m["step_q15"]) * 100 / 32768))))
    return out


def ind_compare(s, folder, fold_by_key, models, part=0, nparts=1):
    """the estimators on each measured burst and on the same burst with the shunt replaced by each model; models
    {label: simulator params with that capture's R}"""
    rows = []
    for b in ind_bursts(folder)[part::nparts]:
        f = fold_by_key[(b["pct"], b["sign"])]
        arr = burst_arrays(b["df"], "slow")
        rows.append(dict(set=s, group=b["group"], burst=b["burst"], pct=b["pct"], sign=b["sign"], model="data", **ind_reads(b["df"], b["sign"])))
        for label, p in models.items():
            t0, y, ser, _ = simulate(p, f)
            idx = np.clip((arr["t"] - t0).astype(int), 0, len(y) - 1)
            yy = np.where(arr["t"] >= t0, y[idx], 0.0)
            rows.append(dict(set=s, group=b["group"], burst=b["burst"], pct=b["pct"], sign=b["sign"], model=label,
                             **ind_reads(synth_burst(b["df"], yy), b["sign"])))
    return rows


# ---------------------------------------------------------------- the motor from one coil

def z_term(La, Lb, Lc, Ra, Rb_, Rc, k, w):
    """the terminal impedance of three coils in a delta, one across the terminals and two in series, with mutual
    M = -k sqrt(Li Lj) / 2 between every pair"""
    Mab, Mac, Mbc = (-k * np.sqrt(x * y) / 2 for x, y in ((La, Lb), (La, Lc), (Lb, Lc)))
    X = np.array([[La, -(Mab + Mac)], [-(Mab + Mac), Lb + Lc + 2 * Mbc]])
    Rm = np.diag([Ra, Rb_ + Rc])
    Y = np.linalg.inv(Rm + 1j * w * X)
    return 1 / Y.sum(), Y


def delta_cases(states, g_coil, r_c):
    """every combination of the coil states over k in 0..0.95: the terminal L at high and low frequency, the extra R
    at high frequency, and the loss as each coil's G times the square of its share of the terminal volts"""
    w_hf, w_lf = 2 * np.pi * 200e3, 2 * np.pi * 200.0
    rows = []
    for combo in itertools.product(states, repeat=3):
        if combo[1] > combo[2]:
            continue
        L = [states[c] for c in combo]
        Gc = [g_coil[c] for c in combo]
        for k in np.linspace(0, 0.95, 20):
            zh, Yh = z_term(*L, r_c, r_c, r_c, k, w_hf)
            zl, _ = z_term(*L, r_c, r_c, r_c, k, w_lf)
            zd, _ = z_term(*L, r_c, r_c, r_c, k, 1e-3)
            l_hf, l_lf = zh.imag / w_hf, zl.imag / w_lf
            I = Yh @ np.array([1.0, 1.0])
            Mab, Mac, Mbc = (-k * np.sqrt(x * y) / 2 for x, y in ((L[0], L[1]), (L[0], L[2]), (L[1], L[2])))
            vb = abs(1j * w_hf * (L[1] * (-I[1]) + Mab * I[0] + Mbc * (-I[1])) + r_c * (-I[1]))
            vc = abs(1j * w_hf * (L[2] * (-I[1]) + Mac * I[0] + Mbc * (-I[1])) + r_c * (-I[1]))
            rows.append(dict(combo=combo, k=k, l_hf=l_hf * 1e3, l_lf=l_lf * 1e3, rb=zh.real - zd.real, lb=(l_lf - l_hf) * 1e3,
                             r_dc=zd.real, g_term=(Gc[0] + Gc[1] * vb ** 2 + Gc[2] * vc ** 2) * 1e3))
    return pd.DataFrame(rows)


def delta_rungs(states, r_c, combos):
    """the series-rung form R_dc + jw L0 + jw Lb Rb / (Rb + jw Lb) fitted to Z(jw) over 1 to 100 kHz, every case"""
    ws = 2 * np.pi * np.logspace(3, 5, 40)
    fits = []
    for c in combos:
        L = [states[x] for x in c]
        for k in np.linspace(0, 0.95, 20):
            z = np.array([z_term(*L, r_c, r_c, r_c, k, w)[0] for w in ws])
            rdc = z_term(*L, r_c, r_c, r_c, k, 1e-3)[0].real

            def res(q):
                l0, lb, rb = np.exp(q)
                m = rdc + 1j * ws * l0 + (1j * ws * lb * rb) / (rb + 1j * ws * lb)
                return np.r_[(m - z).real / np.abs(z), (m - z).imag / np.abs(z)]
            r = least_squares(res, np.log([4e-4, 1e-4, 2.0]))
            l0, lb, rb = np.exp(r.x)
            fits.append(dict(combo=c, k=k, l0=l0 * 1e3, lb=lb * 1e3, rb=rb, tau_us=lb / rb * 1e6, rms=float(np.sqrt(np.mean(r.fun ** 2)))))
    return pd.DataFrame(fits)
