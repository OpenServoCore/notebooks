"""Stage 3's pot-table frames: the tag angle and the pot at every camera frame.

The pot-table sitting films the horn's tag against a reference tag while the
servo runs chains of segments. Each clip's container clock maps to the host's
by its clapper flashes, so every frame carries a host time, the tag angle of
record and the pot at that time (a 2 ms centered mean of the rows) with its
speed. Rest points pair the pot and the tag where the servo stands still:
the baseline's and each coast's tail, and the start of every drive that
changes direction. The pot's stamps lead the tag's motion by a few ms, read
per group of segments by correlating the two motions.

The table is fitted on the steady drives (past the kick, short of the end,
moving) with one free offset per segment and the rest points at offset 0:
pos = T(theta) + c_s, T a cubic B-spline in the tag angle.

`load` runs the whole pipeline over one pair's folder; every step is a
function of its inputs, so a notebook can run them one at a time.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.interpolate import BSpline
from scipy.ndimage import uniform_filter1d

from . import boards, results, tel
from . import fiducial as fd

SITTING = "servo-camera-9"  # the sitting's event log under sittings/
EXPERIMENT = "pot-table"    # the sitting's data folder under the pair
BLOCKS = ("pass-open-9", "crawl-9", "reversal-9", "steps-9", "pass-close-9")
ANGLE = "image_deg"     # the tag reading stage 0 graded
FS = 20_000             # rows per second, the control tick
HALF_FLASH_S = 0.050    # s, half the clapper's 100 ms flash, its midpoint
BOX_ROWS = 41           # rows, the pot's 2 ms centered mean
SPEED_ROWS = 201        # rows, the 10 ms window of the pot's speed
TAIL_S = 0.10           # s, the rest at the end of a baseline or a coast
START_ROWS = 40         # rows, a drive's first 2 ms, before it moves
BEFORE_S = 0.15         # s, the tag's frames before a drive starts
MIN_FRAMES = 5          # frames with both tags a rest point needs
# the segments of each lead group, one group per direction
GROUPS = {"pass": ((1, 2), (3, 4)), "crawl": ((1, 2, 3), (4, 5, 6)),
          "reversal": ((1, 2, 3), (4, 5, 6)), "steps": ((1, 2, 3), (4, 5, 6))}
SEARCH_S, STEP_S, GRID_S = 0.06, 0.0005, 0.001   # s
SMOOTH_PTS = 10                                  # grid points, 10 ms
SKIP_S = 0.15           # s, the kick's spin-up out of a hold
END_S = 0.02            # s, the end of a segment out
MOVING_CPS = 50         # counts/s, a frame that moves


def clip_events(manifest):
    """The sitting's clip events, each with its clip name and what it did."""
    events = pd.read_csv(manifest)
    ev = events[events["event"] == "clip"].copy()
    ev["name"] = ev["detail"].str.split().str[0]
    ev["what"] = ev["detail"].str.split().str[1]
    return ev


def guards_and_rail(manifest):
    """((low, high) guard codes in counts, rail V) the sitting logged."""
    events = pd.read_csv(manifest)
    guard_row = events.loc[events["event"] == "guards", "detail"].iloc[0]
    rail_row = events.loc[events["event"] == "rail", "detail"].iloc[0]
    return (tuple(float(x) for x in guard_row.split()[0:3:2]),
            float(rail_row.split()[0]))


def clips_and_chains(data, clip_ev, blocks=BLOCKS):
    """({clip: (block, file stem, [chain k])}, chains table): each chain to
    the clip that was recording when it started."""
    starts = clip_ev[clip_ev["what"] == "started"]["t_host"].to_numpy()
    stops = clip_ev[clip_ev["what"] == "stopped"]["t_host"].to_numpy()
    names = clip_ev[clip_ev["what"] == "started"]["name"].to_numpy()
    chains, clips = [], {}
    for block in blocks:
        ch = pd.read_csv(data / block / "chains.csv")
        for _, r in ch.iterrows():
            i = int(np.flatnonzero((starts <= r["t_start"] + 0.5)
                                   & (stops >= r["t_start"]))[0])
            # the two passes share the file name; the block names them apart
            name = block[:-2] if names[i] == "pass" else names[i]
            clips.setdefault(name, (block, names[i], []))[2].append(r["k"])
            chains.append({"clip": name, "block": block, **r.to_dict()})
    return clips, pd.DataFrame(chains)


def board_of(meta, where=""):
    """The board a sweep's meta names, its sense block checked."""
    board = boards.from_meta(meta)      # identified from the data itself
    if board is None:
        raise ValueError(f"{where}: no board matches {meta['sense']}")
    board.check_meta(meta, where)       # raises on any mismatch
    return board


def load_sweep(data, block, k):
    """(rows with a host time each, meta, segments) of one chain."""
    folder = data / block / "chains" / str(k)
    df, meta = tel.load(folder)
    board_of(meta, str(results.rel(folder)))
    seg = pd.read_csv(folder / "segments.csv").set_index("seg")
    df["host"] = df["seg"].map(seg["t_start"]) + df["tick"] / meta["tick_hz"]
    return df, meta, seg


def clock(data, block, clip):
    """Container time to host time from the STAT flashes: the midpoint fit
    of record and stage 0's onset fit beside it."""
    stat = pd.read_csv(data / block / f"{clip}.stat.csv")
    clap = pd.read_csv(data / block / f"{clip}-clapper.csv")
    b = fd.bursts(stat["t"], stat["level"])
    if len(b) != len(clap):
        raise ValueError(f"{block}/{clip}: {len(b)} flashes, {len(clap)} "
                         "logged")
    h0 = float(clap["t_before"].iloc[0])   # host times from the first flash
    host = clap["t_before"] - h0
    off, rate, rms = fd.fit_clock(b["on"], host)
    mid = (b["on"] + b["off"]) / 2
    off_m, rate_m, rms_m = fd.fit_clock(mid, host + HALF_FLASH_S)
    return {"h0": h0, "offset": off_m, "rate": rate_m, "rms_ms": rms_m * 1e3,
            "onset_rate": rate, "onset_rms_ms": rms * 1e3,
            "onset_late_ms": (off - off_m) * 1e3, "flashes": len(b),
            "flash_ms": float(np.median(b["off"] - b["on"]) * 1e3)}


def clocks_of(data, clips):
    """One clock row per clip, with both fits' rates in ppm."""
    out = pd.DataFrame([{"clip": name, **clock(data, block, clip)}
                        for name, (block, clip, _) in clips.items()])
    out["ppm"] = (out["rate"] - 1) * 1e6
    out["onset_ppm"] = (out["onset_rate"] - 1) * 1e6
    return out


def clip_angles(data, clips, clocks, name):
    """Per frame: container and host time, both readings, the reference's
    motion in px and whether both tags were found."""
    block, clip, _ = clips[name]
    c = pd.read_csv(data / block / f"{clip}.corners.csv")
    a = fd.angles(c)
    ck = clocks.set_index("clip").loc[name]
    return pd.DataFrame({
        "frame": a["frame"], "t": a["t"],
        "host": ck["h0"] + (a["t"] - ck["offset"]) / ck["rate"],
        "image_deg": a["image_deg"], "angle_deg": a["angle_deg"],
        "ref_motion_px": a["ref_motion_px"], "horn_seen": c["hx0"].notna(),
        "both": np.isfinite(a["image_deg"])})


def centered_mean(x, n):
    """Centered mean of n rows (n odd), shrunk at the ends."""
    x = np.asarray(x, float)
    k = np.ones(n)
    return np.convolve(x, k, "same") / np.convolve(np.ones_like(x), k, "same")


def pot_speed(pos, fs=FS):
    """counts/s per row: the slope of the 2 ms centered mean across a
    centered 10 ms window, NaN where the window runs off the segment."""
    p = centered_mean(pos, BOX_ROWS)
    half = SPEED_ROWS // 2
    v = np.full(len(p), np.nan)
    if len(p) > SPEED_ROWS:
        span = (SPEED_ROWS - 1) / fs
        v[half:-half] = (p[SPEED_ROWS - 1:] - p[:-(SPEED_ROWS - 1)]) / span
    return v


def step_of(schedule, s):
    """The schedule step a segment ran: segment 0 is the baseline."""
    steps = schedule.split(",")
    return "baseline" if s == 0 else steps[(s - 1) % len(steps)]


def segment_frames(tags, name, k, s, g, seg, chain):
    """The frames of clip `name` inside segment s of chain k, with the pot at
    each; `g` the segment's rows, `chain` its chains row."""
    a, h = tags[name], g["host"].to_numpy()
    fr = a[(a["host"] >= h[0]) & (a["host"] <= h[-1])]
    hf = fr["host"].to_numpy()
    return pd.DataFrame({
        "clip": name, "k": k, "seg": s,
        "step": step_of(chain["schedule"], s), "dir": int(seg.loc[s, "dir"]),
        "duty_pct": chain.get("duty_pct", np.nan),
        "share": chain.get("start_share", np.nan),
        "frame": fr["frame"].to_numpy(), "host": hf, "t_seg": hf - h[0],
        "seg_len": h[-1] - h[0], "image_deg": fr["image_deg"].to_numpy(),
        "angle_deg": fr["angle_deg"].to_numpy(),
        "pos": np.interp(hf, h, centered_mean(g["pos"], BOX_ROWS)),
        "pos_speed": np.interp(hf, h, pot_speed(g["pos"]))})


def rest_point(tags, name, k, s, kind, approach, host_lo, host_hi, rows):
    """One (pot, tag) pair from the rows and the tag frames given."""
    a = tags[name]
    fa = a[(a["host"] >= host_lo) & (a["host"] < host_hi) & a["both"]]
    if len(fa) < MIN_FRAMES:
        return None
    return {"clip": name, "k": k, "seg": s, "kind": kind,
            "approach": approach, "pos": rows["pos"].mean(),
            "pos_sd": rows["pos"].std(), "image_deg": fa[ANGLE].mean(),
            "image_sd": fa[ANGLE].std(), "angle_deg": fa["angle_deg"].mean(),
            "n_frames": len(fa), "host": host_hi}


def rest_points(tags, name, k, s, g, seg, step):
    """The rest points one segment gives: its tail, its start, or none."""
    h, d, out = g["host"].to_numpy(), int(seg.loc[s, "dir"]), []
    if s == 0 or step.startswith("coast"):
        tail = g[g["host"] >= h[-1] - TAIL_S]
        out.append(rest_point(tags, name, k, s,
                              "baseline" if s == 0 else "coast",
                              0 if s == 0 else d, h[-1] - TAIL_S,
                              h[-1] + 1e-9, tail))
    if s > 0 and (s == 1 or int(seg.loc[s - 1, "dir"]) != d):
        out.append(rest_point(tags, name, k, s, "start", 0, h[0] - BEFORE_S,
                              h[0], g.iloc[:START_ROWS]))
    return [p for p in out if p is not None]


def frames_and_rest(sweeps, chains, tags):
    """(frames, rest points) over every segment of every chain."""
    frame_parts, static = [], []
    for (name, k), (df, meta, seg) in sweeps.items():
        chain = chains[(chains["clip"] == name) & (chains["k"] == k)].iloc[0]
        for s, g in df.groupby("seg"):
            frame_parts.append(segment_frames(tags, name, k, s, g, seg, chain))
            static += rest_points(tags, name, k, s, g, seg,
                                  step_of(chain["schedule"], s))
    return pd.concat(frame_parts, ignore_index=True), pd.DataFrame(static)


def lead_of(tags, rows):
    """(shift s, correlation): the pot stamps' lead over the tag's motion
    by the correlation of 10 ms smoothed first differences."""
    f = tags[tags["both"]]
    t0, t1 = rows["host"].min(), rows["host"].max()
    f = f[(f["host"] >= t0 - 0.3) & (f["host"] <= t1 + 0.3)]
    grid = np.arange(t0 - 0.1, t1 + 0.1, GRID_S)
    va = np.diff(uniform_filter1d(np.interp(grid, f["host"], f[ANGLE]),
                                  SMOOTH_PTS))
    offs = np.arange(-SEARCH_S, SEARCH_S + STEP_S / 2, STEP_S)
    cs = np.array([np.corrcoef(va, np.diff(uniform_filter1d(np.interp(
        grid, rows["host"] + o, rows["pos"]), SMOOTH_PTS)))[0, 1]
        for o in offs])
    i = int(np.argmax(cs))
    if 0 < i < len(cs) - 1:             # a parabola through the peak
        y0, y1, y2 = cs[i - 1:i + 2]
        return offs[i] + 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) * STEP_S, cs[i]
    return offs[i], cs[i]


def lead_table(sweeps, tags):
    """One lead row per (clip, chain, group), with the minutes into the
    sitting its chain started."""
    lead_rows = []
    first_t = min(seg.loc[0, "t_start"] for _, _, seg in sweeps.values())
    for (name, k), (df, meta, seg) in sweeps.items():
        kind = name.split("-")[0]       # pass, crawl, reversal or steps
        for gi, segs in enumerate(GROUPS[kind]):
            r = df[df["seg"].isin(segs)]
            shift, corr = lead_of(tags[name], r)
            lead_rows.append({"clip": name, "k": k, "group": gi,
                              "dir": int(r["dir"].iloc[0]),
                              "lead_ms": shift * 1e3, "corr": corr,
                              "minutes": (seg.loc[0, "t_start"]
                                          - first_t) / 60})
    return pd.DataFrame(lead_rows)


def moving_frames(frames):
    """(moving frames with a segment id `sid`, lo deg, hi deg): the steady
    drives past the kick and short of the end, moving faster than
    MOVING_CPS, with both tags."""
    fr = frames[frames[ANGLE].notna()].copy()
    held = fr["step"].str.startswith("then") | fr["step"].str.startswith("20@")
    moving = fr[held & (fr["t_seg"] > SKIP_S)
                & (fr["t_seg"] < fr["seg_len"] - END_S)
                & (fr["pos_speed"].abs() > MOVING_CPS)].copy()
    moving["sid"] = (moving["clip"] + "/" + moving["k"].astype(str) + "/"
                     + moving["seg"].astype(str))
    return (moving, float(moving[ANGLE].min()), float(moving[ANGLE].max()))


def basis(theta, knot, lo, hi, k=3):
    """The cubic B-spline basis over [lo, hi], knots every `knot` deg."""
    inner = np.arange(lo, hi + 1e-9, knot)
    if inner[-1] < hi:
        inner = np.append(inner, hi)
    t = np.r_[[lo] * k, inner, [hi] * k]
    n = len(t) - k - 1
    x = np.clip(theta, lo, hi)
    B = np.empty((len(x), n))
    for j in range(n):
        c = np.zeros(n)
        c[j] = 1
        B[:, j] = BSpline(t, c, k, extrapolate=False)(x)
    return np.nan_to_num(B)


def table_at(model, theta, lo, hi):
    """T(theta) of a fitted table, NaN outside the range it was fitted on."""
    theta = np.atleast_1d(np.asarray(theta, float))
    y = basis(theta, model["knot"], lo, hi) @ model["c"]
    return np.where((theta >= lo) & (theta <= hi), y, np.nan)


def fit_table(moving, rest, knot, lo, hi):
    """T and one offset per moving segment; the rest points at offset 0.
    Returns (model, {segment: offset}, residuals moving then rest)."""
    sids = sorted(moving["sid"].unique())
    col = {s: i for i, s in enumerate(sids)}
    Bm = basis(moving[ANGLE].to_numpy(), knot, lo, hi)
    S = np.zeros((len(moving), len(sids)))
    S[np.arange(len(moving)), [col[s] for s in moving["sid"]]] = 1.0
    Br = basis(rest["image_deg"].to_numpy(), knot, lo, hi)
    X = np.vstack([np.hstack([Bm, S]),
                   np.hstack([Br, np.zeros((len(rest), len(sids)))])])
    y = np.r_[moving["pos"].to_numpy(), rest["pos"].to_numpy()]
    beta = np.linalg.lstsq(X, y, rcond=None)[0]
    nb = Bm.shape[1]
    model = {"c": beta[:nb], "knot": knot}
    return model, dict(zip(sids, beta[nb:])), y - X @ beta


@dataclass
class PotFrames:
    """The pot-table sitting's tables, as stage 3 built them."""
    clips: dict         # clip -> (block, file stem, [chain k])
    chains: pd.DataFrame
    sweeps: dict        # (clip, k) -> (rows, meta, segments)
    clocks: pd.DataFrame
    tags: dict          # clip -> every frame's angles
    frames: pd.DataFrame
    static: pd.DataFrame
    lead: pd.DataFrame
    moving: pd.DataFrame
    rest: pd.DataFrame
    starts: pd.DataFrame
    lo_deg: float
    hi_deg: float
    guards: tuple       # (low, high) guard codes, counts
    rail_v: float       # V, the rail every plan ran on

    def rows(self, clip, k):
        """(rows, meta, segments) of one chain: one row per tick, with
        `host` (segment t_start + tick / tick rate, the host's stamp) and
        `host_cam` (host + the group's lead, the camera-aligned time; NaN
        in segment 0 and wherever no lead was fitted)."""
        df, meta, seg = self.sweeps[(clip, k)]
        df = df.copy()
        lead = self.lead[(self.lead["clip"] == clip) & (self.lead["k"] == k)]
        by_dir = dict(zip(lead["dir"], lead["lead_ms"] / 1e3))
        # a coast tail follows its drive: take the direction of the drive
        # before
        d = seg["dir"].replace(0, np.nan).ffill()
        df["host_cam"] = df["host"] + df["seg"].map(d).map(by_dir)
        return df, meta, seg

    def fit_table(self, knot_deg):
        """(coefficients, angle deg -> counts) of stage 3's table fitted at
        `knot_deg`, NaN outside lo_deg..hi_deg."""
        model, _, _ = fit_table(self.moving, self.rest, knot_deg,
                                self.lo_deg, self.hi_deg)

        def counts_at(theta):
            return table_at(model, theta, self.lo_deg, self.hi_deg)

        return model["c"], counts_at


def load(pair_dir, manifest=None):
    """The pot-table sitting of one pair's folder: its data under
    EXPERIMENT, its events in sittings/SITTING.csv unless `manifest` names
    another log."""
    pair_dir = Path(pair_dir)
    data = pair_dir / EXPERIMENT
    if manifest is None:
        manifest = pair_dir / "sittings" / f"{SITTING}.csv"
    clips, chains = clips_and_chains(data, clip_events(manifest))
    guards, rail_v = guards_and_rail(manifest)
    sweeps = {(name, k): load_sweep(data, block, k)
              for name, (block, _, ks) in clips.items() for k in ks}
    clocks = clocks_of(data, clips)
    tags = {name: clip_angles(data, clips, clocks, name) for name in clips}
    frames, static = frames_and_rest(sweeps, chains, tags)
    lead = lead_table(sweeps, tags)
    moving, lo, hi = moving_frames(frames)
    rest = static[(static["image_deg"] >= lo)
                  & (static["image_deg"] <= hi)].copy()
    starts = static[static["kind"] == "start"].copy()
    return PotFrames(clips, chains, sweeps, clocks, tags, frames, static,
                     lead, moving, rest, starts, lo, hi, guards, rail_v)
