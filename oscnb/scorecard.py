"""The stage 5 scorecard: a firmware build's closed-loop behavior on one
fixed test set, scored from the pot, so a change is kept or dropped on
numbers.

  uv run python -m oscnb.scorecard run --label 00-baseline --fw <commit> --image-sha <sha1> [--runs 3] [--note TEXT] [--settings FILE] [--pos-table]
  uv run python -m oscnb.scorecard score <label>
  uv run python -m oscnb.scorecard table

`run` records a rung: `--runs` runs of the three captures of the test set
(stages/5-closed-loop/scorecard/testset.py) under
`telemetry/<pair>/scorecard/<label>/run-K/{steps,follow,hold}`, with
`rung.json` naming the build. With `--settings` the rung runs on a file of
CONFIG fields applied volatile (oscnb.settings) and rebooted away at the
end, the fields kept in `rung.json`. With `--pos-table` it runs on the
pair's position table (oscnb.postable) installed volatile after the
fields, one stamp over both, the file and its sha1 kept in `rung.json`
and the same reboot dropping it. `score` writes the rung's scores, the median
over its runs and their range, to
`stages/5-closed-loop/scorecard/results/<pair>/<label>.json`. `table`
prints the ladder, one column per rung.

Every score reads the pot, `pos` in counts against the goal in counts,
turned to degrees by the plan's counts per degree. Under a LIVE position
table the goals are linearized counts, so the rows' raw pot is read
through the table the rung ran on first. The rows' `pos` carries
spikes while the bridge drives, so every peak, slope and settle read goes
through a centered rolling median of MEDIAN_ROWS rows, taken within one
segment; the hold's jitter alone reads the raw pot. A capture's rows go on
one time axis from the servo's own tick: each segment's `tick0` is the
tick counter's low 16 bits, unwrapped against the end of the segment
before, which holds while no gap between segments reaches 65536 ticks
(3.28 s); a goal commits at its segment's first row.
"""
import argparse
import hashlib
import importlib.util
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

from . import bench, boards, osc, postable, results, servos, settings

STAGE = "5-closed-loop"
TESTSET = results.ROOT / "stages" / STAGE / "scorecard" / "testset.py"
TICK_WRAP = 1 << 16
MEDIAN_ROWS = 41          # about 2 ms: wider than a pot spike, far under a move
SETTLE_BAND_DEG = 0.5     # the band a move settles into, about the play of a small servo's train
REST_MS = 500             # the tail a settled position, an arrival and an enable's rest are read over
RIPPLE_WINDOW_MS = 10     # a speed sample on a large move: several counts of travel at the loop's speeds
CRAWL_WINDOW_MS = 100     # a speed sample on the slowest ramp, ten counts and more of travel
WINDOW_MIN_SHARE = 0.5    # a speed window with fewer rows than this share of its span reads nothing
KICK_MS = 200             # the head of the first hold an enable kick is read over
HOLD_HEAD_MS = 500        # the head of a hold that starts at torque-on, left to the enable rows
TAIL_DRIVEN_SHARE = 0.02  # a hunting tail: above the share a single noise wake leaves
FOLLOW_TAIL = 2 / 3       # the share of a ramp, from its end, a follow score reads
RAIL_TOL_V = 0.3          # the rail against the servo's rated voltage before a run

# Other people's measurements of stock smart servos (STS3215, LX-16A),
# shown beside the ladder for scale; never a target or a result here.
OTHERS = {
    "lag_deg": "2.7 to 7",
    "rest_error_deg": "0.9 (dead zone)",
    "speed_ripple_pct": "7",
    "overshoot_deg": "about 3",
}

UNITS = {
    "overshoot_deg": "deg", "settle_ms": "ms", "move_current_rms_ma": "mA",
    "move_reversals_per_s": "1/s", "move_duty_rms_pct": "% of the rail",
    "follow_duty_rms_pct": "% of the rail", "speed_ripple_pct": "%", "rest_error_deg": "deg",
    "two_sided_deg": "deg", "landing_buzz_share": "share", "landing_tail_driven": "landings",
    "landing_drive_ms": "ms", "landing_drive_ms_worst": "ms",
    "hold_jitter_counts": "counts", "hold_buzz_share": "share",
    "hold_current_rms_ma": "mA", "lag_deg": "deg", "follow_error_sd_deg": "deg",
    "follow_current_rms_ma": "mA", "follow_reversals_per_s": "1/s",
    "ramp_speed_deg_s": "deg/s", "crawl_speed_ripple_pct": "%", "enable_kick_deg": "deg",
    "enable_drive_ms": "ms",
}


def base_of(key):
    """A score key's quantity, the per-size or per-speed tag cut off."""
    return max((b for b in UNITS if key == b or key.startswith(b + "_")), key=len)


# --- loading ---

def unwrap_ticks(tick0, last_tick):
    """Each segment's first row on one tick axis. `tick0` is the counter's
    low 16 bits, `last_tick` the segment's last row tick within it; each
    start is put past the end of the segment before by the shortest wrap."""
    starts = []
    end = None
    for t0, last in zip(tick0, last_tick):
        start = int(t0) if end is None else end + ((int(t0) - end) % TICK_WRAP)
        starts.append(start)
        end = start + int(last) + 1
    return np.array(starts, dtype=np.int64)


def median_pos(pos):
    return pd.Series(pos, dtype=float).rolling(MEDIAN_ROWS, center=True, min_periods=1).median().to_numpy()


def current_zero(rows):
    """The shunt's zero for a capture: the median raw current over its rows
    at zero duty, where the bridge drives nothing."""
    idle = rows["duty_counts"] == 0
    return float(rows.loc[idle, "current_raw"].median()) if idle.any() else np.nan


def file_sha1(path):
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def rung_table(folder):
    """The Table a capture's rung names (`rung.json` two levels up), None
    when it names none; refused when the file is not the one recorded."""
    p = Path(folder).parent.parent / "rung.json"
    named = json.loads(p.read_text()).get("pos_table") if p.exists() else None
    if not named:
        return None
    path = results.ROOT / named["file"]
    if not path.exists() or file_sha1(path) != named["sha1"]:
        raise ValueError(f"{named['file']} is not the table the rung ran on (sha1 {named['sha1']})")
    return postable.load_file(path)[0]


def load(folder):
    """A capture's rows on its time axis, with its plan, meta and board.
    Under a LIVE table `pos` is the raw pot read through the rung's table,
    the raw kept as `pos_raw`. Rows gain `t` (s from the first row),
    `pos_f` (the median pot) and `i_ma` (the shunt's read current, NaN
    where the drive window was not read)."""
    folder = Path(folder)
    meta = json.loads((folder / "meta.json").read_text())
    plan = json.loads((folder / "plan.json").read_text())
    lut = meta.get("plant", {}).get("lut_state")
    table = None
    if lut == "LIVE":
        table = rung_table(folder)
        if table is None:
            raise ValueError(f"{folder}: the position table is LIVE and the rung names none: the rows "
                             f"carry the raw pot and the goals linearized counts, and only the table "
                             f"the rung ran on reconciles them")
    elif lut not in (None, "IDENTITY"):
        raise ValueError(f"{folder}: the position table is {lut}: the rows carry the raw pot and the "
                         f"goals linearized counts, which this scorer does not reconcile")
    board = boards.from_meta(meta)
    if board is None:
        raise ValueError(f"{folder}: no board entry matches the sense block {meta.get('sense')}")
    rows = pd.read_csv(folder / "track.csv", usecols=[
        "seg", "goal", "tick", "window_valid", "pos", "current_raw", "duty_counts"])
    if table is not None:
        rows["pos_raw"] = rows["pos"]
        rows["pos"] = postable.interp_q4(rows["pos_raw"].to_numpy(), table.points) / postable.GRID
    segs = pd.read_csv(folder / "segments.csv")
    segs = segs[segs["rows"] > 0]
    last = rows.groupby("seg")["tick"].max().reindex(segs["seg"]).to_numpy()
    start = dict(zip(segs["seg"], unwrap_ticks(segs["tick0"].to_numpy(), last)))
    tick_hz = meta["tick_hz"]
    abs_tick = rows["seg"].map(start).to_numpy() + rows["tick"].to_numpy()
    rows["t"] = (abs_tick - abs_tick.min()) / tick_hz
    rows["pos_f"] = rows.groupby("seg")["pos"].transform(median_pos)
    read = rows["window_valid"] == 1
    rows["i_ma"] = np.where(read, (rows["current_raw"] - current_zero(rows)) * board.a_per_count * 1e3, np.nan)
    return {"rows": rows, "plan": plan, "meta": meta, "cpd": plan["counts_per_deg"],
            "tick_hz": tick_hz, "folder": folder}


def plan_segs(cap, test):
    return [s for s in cap["plan"]["segments"] if s["test"] == test]


def seg_rows(cap, k):
    return cap["rows"][cap["rows"]["seg"] == k]


# --- plain numbers ---

def reversals(duty):
    """Sign changes of the applied duty, zero rows skipped: +, 0, - is one."""
    s = np.sign(np.asarray(duty))
    s = s[s != 0]
    return int((s[1:] != s[:-1]).sum())


def current_rms(i_ma):
    """The rms over rows, a row whose drive window the shunt did not read
    counted as no current: at zero duty nothing is driven, and a window too
    narrow to read is a few percent of the period, so this is a floor."""
    i = np.nan_to_num(np.asarray(i_ma, float))
    return float(np.sqrt(np.mean(i ** 2))) if len(i) else np.nan


def duty_rms_pct(cap, duty):
    """The rms of the applied duty as a share of the rail: the drive's
    effort where the shunt reads nothing, every row counted."""
    d = np.asarray(duty, float)
    return float(np.sqrt(np.mean(d ** 2)) / cap["meta"]["pwm_period_ticks"] * 100) if len(d) else np.nan


def window_speeds(t, y, window_s):
    """|slope| of y over consecutive windows of `window_s` on the time axis,
    a least-squares line each; a window short of rows reads nothing."""
    t = np.asarray(t, float)
    y = np.asarray(y, float)
    if len(t) < 2:
        return np.array([])
    k = np.floor((t - t[0]) / window_s).astype(int)
    full = int(k.max())     # the last window is partial
    need = WINDOW_MIN_SHARE * window_s / np.median(np.diff(t))
    out = []
    for w in range(full):
        m = k == w
        if m.sum() >= max(need, 2):
            out.append(abs(np.polyfit(t[m], y[m], 1)[0]))
    return np.array(out)


def ripple_pct(speeds):
    if len(speeds) < 3 or np.mean(speeds) == 0:
        return np.nan
    return float(np.std(speeds, ddof=1) / np.mean(speeds) * 100)


def tail(g, ms):
    return g[g["t"] >= g["t"].iloc[-1] - ms / 1000]


def head(g, ms):
    return g[g["t"] < g["t"].iloc[0] + ms / 1000]


def drive_end_s(g, tick_hz):
    """s from the first row of `g` to the end of its last driven row, 0
    when none is driven."""
    driven = np.flatnonzero(g["duty_counts"].to_numpy() != 0)
    if len(driven) == 0:
        return 0.0
    t = g["t"].to_numpy()
    return float(t[driven[-1]] - t[0] + 1 / tick_hz)


# --- the steps capture ---

def step_move(cap, seg):
    """One move: overshoot (deg past the goal along the travel, 0 if none),
    settle (s from the commit to the end of the last row out of the band),
    and the rows from the commit to the settle."""
    g = seg_rows(cap, seg["seg"])
    cpd = cap["cpd"]
    goal = seg["goal_counts"]
    past = seg["direction"] * (g["pos_f"].to_numpy() - goal) / cpd
    overshoot = max(0.0, float(past.max()))
    t = g["t"].to_numpy() - g["t"].iloc[0]
    out = np.flatnonzero(np.abs(g["pos_f"].to_numpy() - goal) / cpd > SETTLE_BAND_DEG)
    settle = 0.0 if len(out) == 0 else t[out[-1]] + 1 / cap["tick_hz"]
    return overshoot, settle, g[t < settle]


def step_scores(cap):
    """Per size: the worst overshoot and settle over its four moves, and
    the current rms, the duty reversals per second and the duty rms over
    every move's rows from commit to settle, pooled."""
    out = {}
    for size in sorted({s["size_deg"] for s in plan_segs(cap, "step")}):
        moves = [step_move(cap, s) for s in plan_segs(cap, "step") if s["size_deg"] == size]
        spans = pd.concat([m[2] for m in moves])
        span_s = sum(m[1] for m in moves)
        revs = sum(reversals(m[2]["duty_counts"]) for m in moves)
        out[f"overshoot_deg_{size}deg"] = max(m[0] for m in moves)
        out[f"settle_ms_{size}deg"] = max(m[1] for m in moves) * 1000
        out[f"move_current_rms_ma_{size}deg"] = current_rms(spans["i_ma"])
        out[f"move_reversals_per_s_{size}deg"] = revs / span_s if span_s else np.nan
        out[f"move_duty_rms_pct_{size}deg"] = duty_rms_pct(cap, spans["duty_counts"])
    return out


def speed_ripple_pct(cap, size=None):
    """On the largest moves: sd over mean of the speed over
    RIPPLE_WINDOW_MS windows across the middle half of the travel, from
    the first row past a quarter to the first past three quarters; the
    worst move."""
    steps = plan_segs(cap, "step")
    size = size if size is not None else max(s["size_deg"] for s in steps)
    worst = np.nan
    for seg in (s for s in steps if s["size_deg"] == size):
        g = seg_rows(cap, seg["seg"])
        p = g["pos_f"].to_numpy()
        frac = (p - p[0]) / (seg["goal_counts"] - p[0])
        a = np.argmax(frac >= 0.25)
        b = np.argmax(frac >= 0.75)
        if not (frac >= 0.75).any() or b <= a:
            continue
        r = ripple_pct(window_speeds(g["t"].to_numpy()[a:b], p[a:b], RIPPLE_WINDOW_MS / 1000))
        worst = r if np.isnan(worst) else max(worst, r)
    return worst


def settled_pos(cap, k):
    return float(tail(seg_rows(cap, k), REST_MS)["pos_f"].mean())


def goal_segs(cap):
    return [s for s in cap["plan"]["segments"] if s["goal_counts"] is not None]


def rest_error_deg(cap):
    """The worst |goal - mean pos| over the last REST_MS of every goal of
    the steps capture."""
    return max(abs(s["goal_counts"] - settled_pos(cap, s["seg"])) for s in goal_segs(cap)) / cap["cpd"]


def landing_buzz_share(cap):
    """The share of rows driven over the last REST_MS of every goal of the
    steps capture, pooled: the loop awake on a shaft that has landed."""
    tails = pd.concat([tail(seg_rows(cap, s["seg"]), REST_MS) for s in goal_segs(cap)])
    return float((tails["duty_counts"] != 0).mean())


def landing_tail_driven(cap):
    """The number of goals of the steps capture whose last REST_MS is
    driven over a share above TAIL_DRIVEN_SHARE: the landings that hunt."""
    return sum(int((tail(seg_rows(cap, s["seg"]), REST_MS)["duty_counts"] != 0).mean() > TAIL_DRIVEN_SHARE)
               for s in goal_segs(cap))


def landing_drive_s(cap, seg):
    """One goal: s from the first row of the median pot inside
    SETTLE_BAND_DEG of it to the end of the last row driven from there, 0
    if none is; the whole segment when the pot never enters the band."""
    g = seg_rows(cap, seg["seg"])
    inside = np.flatnonzero(np.abs(g["pos_f"].to_numpy() - seg["goal_counts"]) / cap["cpd"] <= SETTLE_BAND_DEG)
    if len(inside) == 0:
        return float(g["t"].iloc[-1] - g["t"].iloc[0] + 1 / cap["tick_hz"])
    return drive_end_s(g.iloc[inside[0]:], cap["tick_hz"])


def landing_drive_scores(cap):
    """Over every goal of the steps capture, the drive after arrival
    (landing_drive_s): the median goal and the worst."""
    ms = [landing_drive_s(cap, s) * 1000 for s in goal_segs(cap)]
    return {"landing_drive_ms": float(np.median(ms)), "landing_drive_ms_worst": max(ms)}


def two_sided_deg(cap):
    """Per size, the settled center arriving from above against from
    below; the worst size."""
    worst = 0.0
    backs = [s for s in plan_segs(cap, "step") if s["leg"] == "back"]
    for size in {s["size_deg"] for s in backs}:
        above = [settled_pos(cap, s["seg"]) for s in backs if s["size_deg"] == size and s["direction"] < 0]
        below = [settled_pos(cap, s["seg"]) for s in backs if s["size_deg"] == size and s["direction"] > 0]
        worst = max(worst, abs(np.mean(above) - np.mean(below)) / cap["cpd"])
    return worst


def enable_kick_deg(cap):
    """The span of the median pot over the first KICK_MS of the first hold,
    its rest (the mean of the last REST_MS) included."""
    g = seg_rows(cap, plan_segs(cap, "enable")[0]["seg"])
    head = g[g["t"] < g["t"].iloc[0] + KICK_MS / 1000]["pos_f"]
    rest = tail(g, REST_MS)["pos_f"].mean()
    return (max(head.max(), rest) - min(head.min(), rest)) / cap["cpd"]


def enable_drive_ms(cap, test):
    """Over the first HOLD_HEAD_MS of the capture's first `test` segment, a
    hold that starts at torque-on: ms from its first row to the end of the
    last row driven, 0 when none is."""
    g = head(seg_rows(cap, plan_segs(cap, test)[0]["seg"]), HOLD_HEAD_MS)
    return drive_end_s(g, cap["tick_hz"]) * 1000


# --- the hold capture ---

def hold_scores(cap):
    """Over each hold, the worst of the two: the raw pot's sd, the share of
    rows driven, the current rms. The first hold starts at torque-on, so
    its first HOLD_HEAD_MS is cut off; the other is read whole."""
    holds = [seg_rows(cap, s["seg"]) for s in plan_segs(cap, "hold")]
    holds[0] = holds[0][holds[0]["t"] >= holds[0]["t"].iloc[0] + HOLD_HEAD_MS / 1000]
    return {
        "hold_jitter_counts": max(float(g["pos"].std()) for g in holds),
        "hold_buzz_share": max(float((g["duty_counts"] != 0).mean()) for g in holds),
        "hold_current_rms_ma": max(current_rms(g["i_ma"]) for g in holds),
    }


# --- the follow capture ---

def ramps(cap):
    """{ramp: (its plan segments, its rows)} in order."""
    out = {}
    for s in plan_segs(cap, "ramp"):
        out.setdefault(s["ramp"], []).append(s)
    return {k: (segs, cap["rows"][cap["rows"]["seg"].isin([s["seg"] for s in segs])])
            for k, segs in out.items()}


def ramp_tail(g):
    t0, t1 = g["t"].iloc[0], g["t"].iloc[-1]
    return g[g["t"] >= t1 - FOLLOW_TAIL * (t1 - t0)]


def ramp_speed_deg_s(cap, segs, g):
    """The goals' own speed on the real time axis, first commit to last."""
    t = [g[g["seg"] == s["seg"]]["t"].iloc[0] for s in (segs[0], segs[-1])]
    return abs(segs[-1]["goal_counts"] - segs[0]["goal_counts"]) / (t[1] - t[0]) / cap["cpd"]


def follow_scores(cap):
    """Per speed, over the last FOLLOW_TAIL of its up and down ramps pooled:
    the lag (mean of goal - pos along the travel, the goal the one in force
    at each row), its sd, the current rms, the duty reversals per second
    and the duty rms;
    the goals' real speed, the mean of the two; and on the slowest speed
    the crawl ripple over CRAWL_WINDOW_MS windows, the worse ramp."""
    by_pct = {}
    for segs, g in ramps(cap).values():
        by_pct.setdefault(segs[0]["speed_pct"], []).append((segs, g))
    slowest = min(by_pct)
    out = {}
    for pct, rs in by_pct.items():
        tails = [ramp_tail(g) for _, g in rs]
        err = np.concatenate([segs[0]["direction"] * (t["goal"] - t["pos_f"]).to_numpy()
                              for (segs, _), t in zip(rs, tails)]) / cap["cpd"]
        span_s = sum(t["t"].iloc[-1] - t["t"].iloc[0] for t in tails)
        tag = f"{pct}pct"
        out[f"lag_deg_{tag}"] = float(err.mean())
        out[f"follow_error_sd_deg_{tag}"] = float(err.std(ddof=1))
        out[f"follow_current_rms_ma_{tag}"] = current_rms(pd.concat(tails)["i_ma"])
        out[f"follow_reversals_per_s_{tag}"] = sum(reversals(t["duty_counts"]) for t in tails) / span_s
        out[f"follow_duty_rms_pct_{tag}"] = duty_rms_pct(cap, pd.concat(tails)["duty_counts"])
        out[f"ramp_speed_deg_s_{tag}"] = float(np.mean([ramp_speed_deg_s(cap, s, g) for s, g in rs]))
        if pct == slowest:
            out["crawl_speed_ripple_pct"] = max(
                ripple_pct(window_speeds(t["t"].to_numpy(), t["pos_f"].to_numpy(), CRAWL_WINDOW_MS / 1000))
                for t in tails)
    return out


# --- a run and a rung ---

def run_complete(run_dir):
    run_dir = Path(run_dir)
    for c in ("steps", "follow", "hold"):
        m = run_dir / c / "meta.json"
        if not m.exists() or json.loads(m.read_text()).get("ended") != "ok":
            return False
    return True


def score_run(run_dir):
    """Every score of one run, {key: value}."""
    run_dir = Path(run_dir)
    steps = load(run_dir / "steps")
    hold = load(run_dir / "hold")
    out = step_scores(steps)
    out["speed_ripple_pct"] = speed_ripple_pct(steps)
    out["rest_error_deg"] = rest_error_deg(steps)
    out["two_sided_deg"] = two_sided_deg(steps)
    out["landing_buzz_share"] = landing_buzz_share(steps)
    out["landing_tail_driven"] = landing_tail_driven(steps)
    out.update(landing_drive_scores(steps))
    out["enable_kick_deg"] = enable_kick_deg(steps)
    out["enable_drive_ms"] = max(enable_drive_ms(steps, "enable"), enable_drive_ms(hold, "hold"))
    out.update(hold_scores(hold))
    out.update(follow_scores(load(run_dir / "follow")))
    return out


def score_rung(rung_dir):
    """The rung's scores, the median over its complete runs with their
    range, and the runs scored and skipped."""
    rung_dir = Path(rung_dir)
    runs = sorted(p for p in rung_dir.glob("run-*") if p.is_dir())
    done = [p for p in runs if run_complete(p)]
    per = [score_run(p) for p in done]
    keys = list(dict.fromkeys(k for s in per for k in s))
    scores = {k: results.median_range([s.get(k, np.nan) for s in per], UNITS[base_of(k)]) for k in keys}
    return {"scores": scores, "runs": [p.name for p in done],
            "skipped": [p.name for p in runs if p not in done]}


# --- the runner ---

def testset():
    spec = importlib.util.spec_from_file_location("scorecard_testset", TESTSET)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def pair_of(servo, board):
    return f"{servo.key}__{board.key}"


def rung_name(pair, label):
    return f"telemetry/{pair}/scorecard/{label}"


def rung_dir(pair, label):
    return results.ROOT / rung_name(pair, label)


def shown(p):
    """`p` from the repo root when it is under it, as every output names a path."""
    try:
        return results.rel(p)
    except ValueError:
        return p


def result_path(pair, label):
    return results.ROOT / "stages" / STAGE / "scorecard" / "results" / pair / f"{label}.json"


def preflight(servo, board):
    """Refuse unless the bus is at full rate, torque is off and the rail is
    at the servo's rated voltage; nothing is driven or set here."""
    osc.require_3m()
    torque = osc.get("torque_enable")
    if torque not in ("off", 0):
        raise SystemExit(f"torque is {torque}: the scorecard starts from torque off (osc set torque_enable off)")
    vsys = bench.read_vsys(board)
    if abs(vsys - servo.rated_v) > RAIL_TOL_V:
        raise SystemExit(f"the rail reads {vsys:.2f} V, not within {RAIL_TOL_V} V of the servo's rated "
                         f"{servo.rated_v} V: set the supply, the scorecard never does")


def config_hash(config):
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def capture(out, name, segs, center, guard, inp, deadman_check, ts):
    """One capture: its plan.json, then `osc-lab track`, then the check
    that the servo ran the range the plan was built for."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "plan.json").write_text(json.dumps(ts.plan_doc(name, segs, center, guard, inp), indent=2) + "\n")
    osc.track(out, "position", [s["item"] for s in segs],
              current_cap_ma=inp["current_cap_ma"], current_cap_ms=inp["current_cap_ms"],
              deadman_check=deadman_check)
    meta = json.loads((out / "meta.json").read_text())
    if tuple(meta["guard"]) != tuple(guard) or meta["center"] != center:
        raise SystemExit(f"{shown(out)}: the servo ran guard {meta['guard']} center {meta['center']}, "
                         f"the plan was built for {list(guard)} center {center}")
    return meta


def run(label, fw, image_sha, runs=3, note="", settings_file=None, pos_table=False, servo=None, board=None):
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    rung = rung_dir(pair, label)
    if rung.exists():
        raise SystemExit(f"{shown(rung)} already holds a rung: pick a new label or remove it")
    ts = testset()
    inp = ts.inputs(servo, pair)
    fields = settings.load(settings_file)[0] if settings_file else None
    table = None
    if pos_table:
        postable.load(pair)
        path = postable.out_path(pair)
        table = {"file": str(results.rel(path)), "sha1": file_sha1(path)}
    preflight(servo, board)
    guard = (osc.get("pos_min_soft_counts"), osc.get("pos_max_soft_counts"))
    center, sets = ts.build(guard, inp)
    try:
        if fields:
            settings.apply(fields, stamp=not table)
        if table:
            postable.install(pair)
            settings.stamp_live()
        for k in range(1, runs + 1):
            if k > 1:
                preflight(servo, board)
            for i, name in enumerate(ts.CAPTURES):
                meta = capture(rung / f"run-{k}" / name, name, sets[name], center, guard, inp,
                               deadman_check=i == 0, ts=ts)
                if not (rung / "rung.json").exists():
                    (rung / "rung.json").write_text(json.dumps({
                        "label": label, "fw": fw, "image_sha": image_sha, "note": note,
                        "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                        "config_hash": config_hash(meta["config"]), "runs_asked": runs,
                        "settings": fields, "pos_table": table,
                    }, indent=2) + "\n")
            print(f"run-{k} of {runs} done")
    finally:
        # the stored set comes back whether the runs ended or aborted
        if fields or table:
            settings.restore()
    return rung


def score(label, servo=None, board=None):
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    rung = rung_dir(pair, label)
    if not (rung / "rung.json").exists():
        raise SystemExit(f"no rung {label!r} under {shown(rung.parent)}")
    doc = {**json.loads((rung / "rung.json").read_text()), "capture": rung_name(pair, label),
           **score_rung(rung)}
    if not doc["runs"]:
        raise SystemExit(f"rung {label!r} has no complete run to score")
    p = result_path(pair, label)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(results._plain(doc), indent=2) + "\n")
    return shown(p)


def cell(q):
    if q["value"] is None:
        return "-"
    v, lo, hi = q["value"], q["lo"], q["hi"]
    if lo is None or hi is None or lo == hi:
        return f"{v:.3g}"
    return f"{v:.3g} [{lo:.3g}, {hi:.3g}]"


def table(servo=None, board=None, file=None):
    """The ladder: one row per score, one column per rung in label order,
    other people's figures last where a row has one."""
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    files = sorted(result_path(pair, "x").parent.glob("*.json"))
    if not files:
        raise SystemExit(f"no scored rung for {pair}")
    docs = [json.loads(p.read_text()) for p in files]
    keys = list(dict.fromkeys(k for d in docs for k in d["scores"]))
    head = ["score", "unit", *(d["label"] for d in docs), "others (STS3215, LX-16A)"]
    body = []
    for k in keys:
        cells = [cell(d["scores"][k]) if k in d["scores"] else "-" for d in docs]
        body.append([k, UNITS[base_of(k)], *cells, OTHERS.get(base_of(k), "")])
    widths = [max(len(r[c]) for r in [head, *body]) for c in range(len(head))]
    for r in [head, *body]:
        print("  ".join(v.ljust(w) for v, w in zip(r, widths)).rstrip(), file=file or sys.stdout)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.scorecard", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="record a rung on the bench")
    r.add_argument("--label", required=True)
    r.add_argument("--fw", required=True, help="the firmware commit the image was built from")
    r.add_argument("--image-sha", required=True, help="the sha1 of the flashed image")
    r.add_argument("--runs", type=int, default=3)
    r.add_argument("--note", default="")
    r.add_argument("--settings", type=Path, help="a file of CONFIG fields the rung runs on, applied volatile")
    r.add_argument("--pos-table", action="store_true",
                   help="run on the pair's position table, installed volatile")
    s = sub.add_parser("score", help="score a recorded rung")
    s.add_argument("label")
    sub.add_parser("table", help="print the ladder")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        print(shown(run(a.label, a.fw, a.image_sha, a.runs, a.note, a.settings, a.pos_table)))
    elif a.cmd == "score":
        print(score(a.label))
    else:
        table()


if __name__ == "__main__":
    main()
