"""Plumbing of stages/3-servo/pot-side-force: the pot against the horn's
tag, frame by frame, on the hanging arm and on the free horn.

The three blocks of `telemetry/<pair>/pot-side-force/` keep the pot-table
layout with osc-lab track rows in place of sweep rows:

  chains.csv                k, schedule, t_start, t_end, folder
  chains/<k>/meta.json      the track's meta as written, its sense block
                            checked against the board registry
  chains/<k>/segments.csv   one row per segment, its host start t_start
  chains/<k>/track.csv      the TRACK_COLS of every row, as written
  <stem>.corners.csv        `fiducial.track` corners of both tags
  <stem>.stat.csv           the STAT lamp per frame, clapper clips only
  <stem>-clapper.csv        the clapper's host marks, clapper clips only
  camera.json               the lens the corners were tracked through
  manifest.json             the clip's size, rate, sha256 and FOUND_KEYS

A row's host time is its segment's t_start plus its tick over the tick rate,
and a frame's comes from the clip's clapper (`potframes.clock`) or, for a
clip without one, from a shift the notebook fits. The signed winding
current per row takes stage 1's current scale and stage 3's window term.

The pot is compared with stage 3 pot-table's table of record, the counts at
the tag angle on a 1 deg grid fitted on the raw codes, so a row's raw `pos`
is what it is compared with. Each clip's own tag angle maps into that
table's frame as sign x angle + offset (`map_angle`).

`free_frames` builds the free horn's frame table from the pot-table sitting
itself: per camera frame the pot, its 16-row mean, the table at the tag and
the winding current.

The fits, the harmonics and the bootstraps stay in the notebook.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from . import potframes, results, scales, speed
from . import fiducial as fd
from . import position as pe

EXPERIMENT = "pot-side-force"
STEMS = {"plumb-1": "plumb", "holds-1": "holds", "arm-1": "arm"}
TRACK_COLS = ["seg", "tick", "window_valid", "pos", "current_raw",
              "current_trough", "duty_counts", "vmotor_a", "vmotor_b"]
# what a manifest of found data names beside the clip's own entry
FOUND_KEYS = ("setup", "image", "loop", "track_columns", "lens")
CLIP_KEYS = ("original", "frames_tracked", "sha256", "frames", "fps")
FREE_MEAN_ROWS = 201    # rows, a 10 ms centered mean of the current
FREE_MEAN16 = 16        # rows, the pot's 16-row mean, the pattern's period
FREE_SKIP_TICKS = 20    # ticks into a segment before a frame counts


@dataclass
class PotTable:
    """Stage 3 pot-table's table of record on its 1 deg grid."""
    angle_deg: np.ndarray
    counts: np.ndarray
    slope: np.ndarray       # counts/deg, the table's local slope

    @classmethod
    def load(cls, pair):
        p = results.path("3-servo", "pot-table", pair=pair)
        t = pd.read_csv(p.with_suffix(".table.csv"))
        return cls(t["angle_deg"].to_numpy(), t["counts"].to_numpy(),
                   t["local_scale_counts_per_deg"].to_numpy())

    def counts_at(self, theta):
        """T(theta): counts at tag angle theta (deg), linear between the
        grid's points, NaN outside it."""
        return np.interp(theta, self.angle_deg, self.counts,
                         left=np.nan, right=np.nan)

    def angle_at(self, counts):
        """T^-1: the tag angle (deg) at which the table reads `counts`."""
        return np.interp(counts, self.counts, self.angle_deg,
                         left=np.nan, right=np.nan)

    def slope_at(self, theta):
        """The table's local slope at theta, counts/deg."""
        return np.interp(theta, self.angle_deg, self.slope)


def manifest(data, block):
    """A block's manifest.json, refused without the keys found data
    carries."""
    doc = json.loads((Path(data) / block / "manifest.json").read_text())
    missing = [k for k in FOUND_KEYS if k not in doc]
    for name, clip in doc["clips"].items():
        missing += [f"{name}:{k}" for k in CLIP_KEYS if k not in clip]
    if missing:
        raise ValueError(f"{block}: manifest lacks {', '.join(missing)}")
    return doc


def chains(data, block):
    return pd.read_csv(Path(data) / block / "chains.csv")


def winding_current(df, chain, g_s):
    """(signed winding current A per row, zero info). The crest sample less
    the zero, times stage 1's scale, less the window term g (1 - D) V_on,
    signed by the duty; 0 on an undriven row, NaN on a driven row without a
    valid window. The zero is the median current_raw over the chain's
    undriven rows."""
    duty = df["duty_counts"].to_numpy(float)
    off = duty == 0
    zero = float(np.median(df.loc[off, "current_raw"]))
    sgn = np.sign(duty)
    D = np.abs(duty) / chain.half_period_ticks
    v_on = speed.tap_volts(df["vmotor_a"], df["vmotor_b"], sgn, chain)
    mag = ((df["current_raw"].to_numpy(float) - zero) * chain.a_per_count
           - g_s * (1 - D) * v_on)
    ok = (df["window_valid"].to_numpy() == 1) & ~off
    current = np.where(ok, sgn * mag, np.nan)
    current = np.where(off, 0.0, current)
    info = dict(zero=zero, n_undriven=int(off.sum()), n_rows=len(df),
                trough_median=float(np.median(df.loc[~off,
                                                     "current_trough"])),
                a_per_count=chain.a_per_count,
                counts_per_a=1 / chain.a_per_count)
    return current, info


def load_chain(data, block, k, g_s):
    """(rows, meta, segments, board, chain, zero info) of one chain. Rows
    carry `t_seg` (s into the segment), `host` (t_start + t_seg) and the
    signed winding current `I` (A, `winding_current` at window term g_s,
    in S). Refuses a chain whose board does not match or that dropped
    rows."""
    folder = Path(data) / block / "chains" / str(k)
    meta = json.loads((folder / "meta.json").read_text())
    board = potframes.board_of(meta, str(results.rel(folder)))
    if meta["rows_dropped"]:
        raise ValueError(f"{results.rel(folder)}: rows dropped")
    seg = pd.read_csv(folder / "segments.csv").set_index("seg")
    df = pd.read_csv(folder / "track.csv", usecols=TRACK_COLS)
    df["t_seg"] = df["tick"] / meta["tick_hz"]
    df["host"] = df["seg"].map(seg["t_start"]) + df["t_seg"]
    chain = speed.chain_of(board, scales.load(board))
    df["I"], zero = winding_current(df, chain, g_s)
    return df, meta, seg, board, chain, zero


def clock(data, block):
    """`potframes.clock` of the block's clip, None for a clip with no
    clapper."""
    stem = STEMS[block]
    if not (Path(data) / block / f"{stem}-clapper.csv").exists():
        return None
    return potframes.clock(Path(data), block, stem)


def frames(data, block, host_shift=None):
    """(frames, corners, clock). Per frame its container time `t`, `host`
    time (from the clock, or t + host_shift for a clip with no clapper),
    the tag readings, the reference's motion and whether both tags were
    found."""
    c = pd.read_csv(Path(data) / block / f"{STEMS[block]}.corners.csv")
    a = fd.angles(c)
    ck = clock(data, block)
    if ck is not None:
        host = ck["h0"] + (a["t"] - ck["offset"]) / ck["rate"]
    else:
        host = a["t"] + (0.0 if host_shift is None else host_shift)
    out = pd.DataFrame({"frame": a["frame"], "t": a["t"], "host": host,
                        "image_deg": a["image_deg"],
                        "angle_deg": a["angle_deg"],
                        "ref_motion_px": a["ref_motion_px"],
                        "both": np.isfinite(a["image_deg"])})
    return out, c, ck


def map_angle(table, cam_deg, pos, sign=None):
    """(sign, offset deg, residual rms counts): the camera angle into the
    table's frame, sign x cam + offset, chosen so T(sign x cam + offset)
    best matches the pot (raw counts) over the pairs given. The offset is
    scanned on a 0.01 deg grid 5 deg either side of the pot's own median
    angle less the camera's; both signs unless `sign` is given."""
    cam_deg, pos = np.asarray(cam_deg, float), np.asarray(pos, float)
    ok = np.isfinite(cam_deg) & np.isfinite(pos)
    cam_deg, pos = cam_deg[ok], pos[ok]
    best = None
    for s in ((1, -1) if sign is None else (sign,)):
        c0 = np.nanmedian(table.angle_at(pos) - s * cam_deg)
        grid = c0 + np.arange(-5, 5.0001, 0.01)
        cost = [np.nanmean((pos - table.counts_at(s * cam_deg + c)) ** 2)
                for c in grid]
        c = grid[int(np.nanargmin(cost))]
        rms = float(np.sqrt(np.nanmin(cost)))
        if best is None or rms < best[2]:
            best = (s, float(c), rms)
    return best


def at(t_src, x_src, t):
    """x at times t by linear interpolation, NaN outside."""
    return np.interp(t, t_src, x_src, left=np.nan, right=np.nan)


def box(x, n):
    """Centered NaN-aware mean of n samples, NaN where a quarter or less of
    the window is finite."""
    x = np.asarray(x, float)
    ok = np.isfinite(x)
    k = np.ones(n)
    s = np.convolve(np.where(ok, x, 0.0), k, "same")
    c = np.convolve(ok.astype(float), k, "same")
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(c > n // 4, s / c, np.nan)


def coast_zero(df):
    """(the current zero from a chain's coasts or NaN, the mean
    current_trough over its driven rows). A coast is a segment whose
    median duty is 0, its zero the mean current_raw over the second half
    of its undriven rows."""
    coast = []
    for s, g in df[df["seg"] > 0].groupby("seg"):
        if g["duty_counts"].median() == 0:
            # the stream opens before the coast's duty shows
            h = g.iloc[len(g) // 2:]
            coast.append(h.loc[h["duty_counts"] == 0, "current_raw"])
    z_coast = float(pd.concat(coast).mean()) if coast else np.nan
    drv = df[(df["seg"] > 0) & (df["duty_counts"] != 0)]
    return z_coast, float(drv["current_trough"].mean())


def _free_parts(pf, fr, clip, k, zero, chain, g_s):
    """The frames of one pot-table chain, each with the 16-row mean of the
    pot, the duty and the winding current at the row under it."""
    df, meta, _ = pf.rows(clip, k)
    duty = df["duty_counts"].to_numpy(float)
    d = np.sign(duty)
    D = np.abs(duty) / chain.half_period_ticks
    v_on = speed.tap_volts(df["vmotor_a"], df["vmotor_b"], d, chain)
    i_mag = ((df["current_raw"].to_numpy(float) - zero) * chain.a_per_count
             - g_s * (1 - D) * v_on)
    current = np.where(duty != 0, d * i_mag, np.nan)
    segs = df["seg"].to_numpy()
    current_mean = np.full(len(df), np.nan)
    for s in np.unique(segs):
        m = segs == s
        current_mean[m] = box(current[m], FREE_MEAN_ROWS)
    cam_all = df["host_cam"].to_numpy()
    parts = []
    for gd in (1.0, -1.0):
        m = (df["dir"].to_numpy() == gd) & (segs > 0)
        f = fr[(fr["clip"] == clip) & (fr["k"] == k) & (fr["dir"] == gd)]
        if not m.any() or f.empty:
            continue
        cam, sg = cam_all[m], segs[m]
        tick = df["tick"].to_numpy()[m]
        mean16 = pe.boxcar(df["pos"].to_numpy(float)[m], FREE_MEAN16)
        f = f.copy()
        dt = 1.0 / meta["tick_hz"]
        # the row under each frame: the last row stamped at or before it
        i = np.searchsorted(cam, f["host"].to_numpy(), side="right") - 1
        ok = (i >= 0) & (f["host"].to_numpy() - cam[np.maximum(i, 0)]
                         < 2 * dt)
        i = np.maximum(i, 0)
        ok &= (sg[i] == f["seg"].to_numpy()) & (tick[i] >= FREE_SKIP_TICKS)
        f, i = f[ok], i[ok]
        f["box16"] = mean16[i]
        f["duty_ticks"] = duty[m][i]
        f["I"] = current_mean[m][i]
        f["I_inst"] = current[m][i]
        f["kind"], f["gd"] = potframes.kind_of(clip), gd
        parts.append(f)
    return parts


def free_frames(pair_dir, knot_deg, pot_scale, g_s):
    """(frames, PotFrames, info) of the pot-table sitting, the free horn.

    Per camera frame inside a driven chain: `box16` the pot's 16-row mean
    at the row under the frame on the camera-aligned clock
    (`PotFrames.rows` host_cam), `T_counts` stage 3's table refitted at
    `knot_deg` at the tag, `rc` their difference (counts), `I` the winding
    current's 10 ms centered mean (A) and `I_inst` the row's own, `speed`
    the pot's speed over `pot_scale` (deg/s), `duty_pct`, `label` and a
    segment id `sid`. Each chain's zero is its coasts' (`coast_zero`); a
    chain without a coast takes its driven trough mean less the mean
    trough-less-coast offset of the chains that have both."""
    pf = potframes.load(Path(pair_dir), wide_codes=True)
    _, table = pf.fit_table(knot_deg)
    meta0 = next(iter(pf.sweeps.values()))[1]
    board = potframes.board_of(meta0, "first chain")
    chain = speed.chain_of(board, scales.load(board))
    zeros = {ck: coast_zero(pf.sweeps[ck][0]) for ck in pf.sweeps}
    both = [(zc, zt) for zc, zt in zeros.values() if np.isfinite(zc)]
    trough_less_coast = float(np.mean([zt - zc for zc, zt in both]))
    fr = pf.frames[pf.frames["seg"] > 0].dropna(subset=["image_deg"])
    parts, zinfo = [], []
    for (clip, k) in pf.sweeps:
        potframes.board_of(pf.sweeps[(clip, k)][1], f"{clip}/{k}")
        zc, zt = zeros[(clip, k)]
        zero = zc if np.isfinite(zc) else zt - trough_less_coast
        zinfo.append(dict(clip=clip, k=k, zero_coast=zc, zero_trough=zt,
                          zero=zero))
        parts += _free_parts(pf, fr, clip, k, zero, chain, g_s)
    fe = pd.concat(parts, ignore_index=True)
    fe["speed"] = fe["pos_speed"] / pot_scale
    fe["T_counts"] = table(fe["image_deg"].to_numpy())
    fe["rc"] = fe["box16"] - fe["T_counts"]
    fe["duty_pct"] = (fe["duty_ticks"].abs() / board.pwm_arr * 100).round()
    fe["label"] = [potframes.label_of(kd, s, d) for kd, s, d in
                   zip(fe["kind"], fe["seg"], fe["duty_ticks"].abs())]
    fe["sid"] = (fe["clip"] + "/" + fe["k"].astype(str) + "/"
                 + fe["seg"].astype(str))
    info = dict(a_per_count=chain.a_per_count, g_s=g_s, board=board.key,
                trough_less_coast=trough_less_coast,
                zeros=pd.DataFrame(zinfo))
    return fe, pf, info
