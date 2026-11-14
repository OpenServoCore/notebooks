"""The stage 5 scorecard: a firmware build's closed-loop behavior on one
fixed test set, scored from the pot, so a change is kept or dropped on
numbers.

  uv run python -m oscnb.scorecard run --label 00-baseline --fw <commit> --image-sha <sha1> [--runs 3] [--note TEXT] [--set PATH] [--deviation FILE] [--protection NAME ...] [--loosen NAME ...] [--max-load N]
  uv run python -m oscnb.scorecard ab --a-label L --a-fw REV --a-image PATH [--a-osc PATH] [--a-set PATH] [--a-deviation FILE] [--a-protection NAME ...] [--a-loosen NAME ...] [--a-note TEXT] --b-label ... [--c-label ...] [--order abc] [--rounds 3] [--max-load N]
  uv run python -m oscnb.scorecard score <label>
  uv run python -m oscnb.scorecard carry
  uv run python -m oscnb.scorecard table

`run` records a rung: `--runs` runs of the captures of the test set
(stages/5-closed-loop/scorecard/testset.py) under
`telemetry/<pair>/scorecard/<label>/run-K/{steps,follow,hold,wide}`, with
`rung.json` naming the build. `wide`, holds at the middle of each ADC
wide code of the board, is run where the board's wide-code result names
a code in reach, on every rung, and `rung.json` names its goals.

A rung runs on a constant set (oscnb.constants): the pair's set of
record, the set of `--set`, or that set with one deviation, the file of
`--deviation`, built over it by the set's own rules. Nothing else puts
a constant on the servo: no list of fields, no table and no slots are
applied beside the set. The test set is planned from that set, its soft
range, and where its position table is LIVE the table's counts at the
table's scale, centered on the raw center read through it (testset.py).
The set is installed before the runs (`constants.install`, volatile,
behind its protection gate, which `--protection` and `--loosen` name
fields for, and stamped there alone) and a reboot brings the stored
state back after them. Before a run's first capture the servo's
constant state is read into `run-K/servo.json`: the bytes of CONFIG and
CALIB with every field there by name and the writable ones named, the
position table's state and its points as fetched back with their sha1,
the wide-code slots, the stamp and the data state. A servo that is not
the rung's set then, or whose data state is not clean, does not start
the run. `rung.json` keeps the hash of the first run's state, and a
later run that reads another is refused, what differs named. It names
the set by its file, its three hashes and its fields by class, the
deviation by its file and every field it moved as [of record,
deviated], and protection as its own entry, the hash and what the
install moved against the servo's values; a copy of the set the rung
ran on lies beside it as `constants.json`, so the rung stays readable
when the set of record moves.
`ab` records two or three such rungs at once, the sides a, b and c
with the same flags each, one run of each per round in `--order`,
flashing a side's image whenever the servo holds another, so all share
the servo's drift through the session. A flash that `osc flash` does
not confirm ends the session; what the image then holds is verified by
the set's install and the state read before the run, as on any rung,
so an image that boots with a stamp from before its covered fields
changed runs once the install has stamped it. Each
side names its own set and deviation, installed on its own image around
its own runs: an A/B is the set against the set with one thing changed,
the one thing named. A side also names its own `osc`, `--a-osc`: the
plant stamp is computed by the tool, so an image whose stamp covers
other fields is flashed, read, installed on and stamped by the tool
built from its own tree. Every `osc` call of the side's turn, from its
flash to the reboot after its run, goes through that tool, and none
outside it. `rung.json` names the tool by the sha1 of its binary and
its `--version` line, never a path. A run that ends because a telemetry
burst came back short is rows lost on the link and no result: it is
moved under the rung's `lost/`, kept in `rung.json` with the capture,
the tool's reason and the host's load, and started again from its first
capture, the servo read again and checked against the set, RETRIES
times at most; any other failure ends the session. With `--max-load` a
run waits while the host's one minute load is over it, LOAD_WAIT_S at
most, then goes on, the wait kept in `rung.json`. `score` writes the rung's scores, the median
over its runs and their range, to
`stages/5-closed-loop/scorecard/results/<pair>/<label>.json`, with the
scale its current rows are at, `current_scale`. `carry` brings the
results that name no such scale, scored at the board's nominal parts
from rows that were not kept, to the board's measured one. `table`
prints the ladder, one column per rung.

Every score reads the pot, `pos` in counts against the goal in counts,
turned to degrees by the plan's counts per degree. Under a LIVE position
table the goals are linearized counts, so the rows' raw pot is read
through the table the rung ran on first. On a rung whose set holds ADC
wide-code slots the servo reads a pot sample at a wide code as the
middle of the span the code stands for, so the rows' raw pot takes the
same step first, with the rung's slots, ahead of the table. The table
and the slots are those of the rung's own record: the set copied into
its folder, the table's points from the run's `servo.json`. A rung
recorded before rungs named a set carries no `set` in its `rung.json`
and names the table file and the wide-code result it ran on there, each
by its sha1; it is read through those files as it always was. The rows' `pos`
carries spikes while the bridge drives, so every peak, slope and settle read goes
through a centered rolling median of MEDIAN_ROWS rows, taken within one
segment; the hold's jitter alone reads the pot unfiltered. A current row
reads the shunt over the capture's own zero through the board's measured
counts per A: the scale of the rung's set, or on a rung recorded before
rungs named one the board's stage 1 current-scale result, never the
registry's nominal parts. A capture's rows go on
one time axis from the servo's own tick: each segment's `tick0` is the
tick counter's low 16 bits, unwrapped against the end of the segment
before, which holds while no gap between segments reaches 65536 ticks
(3.28 s); a goal commits at its segment's first row.
"""
import argparse
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

from . import bench, boards, osc, postable, results, scales, servos, settings, widecodes

STAGE = "5-closed-loop"
SET_COPY = "constants.json"     # the set a rung ran on, beside its rung.json
BOARD_SCALE = "1-board/current-scale"   # where a rung that names no set has its current scale from
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
SHORT_BURST = "the burst came back short"   # how `osc-lab track` words a burst that lost rows on the link
RETRIES = 2               # the times a run that lost a burst is started again
LOAD_WAIT_S = 300         # the longest a run waits for the host's load to fall
LOAD_POLL_S = 5
SIDES = "abc"             # the sides of an alternated session, two or three of them
REPORT_POT_MS = 50        # the pot a position report is read against, its span the quiet gate
REPORT_EDGE_ROWS = 20     # 1 ms at 20 kHz each side of a poll's hole: the pot's level there
CRAWL_SWING_PCTL = 95     # the crawl's widest goal items, short of a lone one
Q16 = 1 << 16

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
    "hold_jitter_deg": "deg", "hold_buzz_share": "share",
    # the hold's jitter as a result scored before the row was in degrees holds it, for the ladder
    "hold_jitter_counts": "counts",
    "hold_current_rms_ma": "mA", "wide_hold_hunting": "holds", "wide_hold_driven_share": "share",
    "wide_hold_current_rms_ma": "mA", "lag_deg": "deg", "lag_ms": "ms", "follow_error_sd_deg": "deg",
    "follow_current_rms_ma": "mA", "follow_reversals_per_s": "1/s",
    "ramp_speed_deg_s": "deg/s", "crawl_speed_ripple_pct": "%", "crawl_swing_deg": "deg",
    "enable_kick_deg": "deg",
    "enable_drive_ms": "ms", "speed_report_error_deg_s": "deg/s", "position_report_error_deg": "deg",
}


def constants():
    """oscnb.constants, imported where it is used: it imports this
    module for the servo's state."""
    from . import constants as cn
    return cn


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


def rung_named(folder, key):
    """What a capture's rung names under `key` (`rung.json` two levels
    up), None when it names nothing there or there is no rung."""
    p = Path(folder).parent.parent / "rung.json"
    return json.loads(p.read_text()).get(key) if p.exists() else None


def rung_set(folder):
    """The constant set a capture's rung ran on, from the copy in the
    rung's folder; None for a rung recorded before rungs named a set,
    which names the files it ran on instead. Refused when the copy is
    not the set `rung.json` names."""
    cn = constants()
    named = rung_named(folder, "set")
    if not named:
        return None
    doc = cn.load(Path(folder).parent.parent / SET_COPY)
    if doc["hash"] != named["hash"]:
        raise ValueError(f"{folder}: the rung's {SET_COPY} is not the set its rung.json names "
                         f"(set hash {named['hash']['set']})")
    return doc


def rung_table(folder):
    """The Table a capture's rung ran on, None when it ran on none. On a
    rung that names its constant set it is the points the run's
    `servo.json` holds, which must be the set's; on a rung recorded
    before, the table file `rung.json` names, refused when the file is
    not the one recorded."""
    cn = constants()
    doc = rung_set(folder)
    if doc is not None:
        if doc["pos_table"]["state"] != "LIVE":
            return None
        state = json.loads((Path(folder).parent / "servo.json").read_text())
        points = bytes.fromhex(state["pos_table"]["points"])
        if hashlib.sha1(points).hexdigest() != doc["pos_table"]["points_sha1"]:
            raise ValueError(f"{folder}: the run's servo.json does not hold the points of the rung's set")
        return postable.from_points(points, *(state["fields"][n] for n in cn.STOPS))
    named = rung_named(folder, "pos_table")
    if not named:
        return None
    path = results.ROOT / named["file"]
    if not path.exists() or file_sha1(path) != named["sha1"]:
        raise ValueError(f"{named['file']} is not the table the rung ran on (sha1 {named['sha1']})")
    return postable.load_file(path)[0]


def rung_wide_slots(folder):
    """The ADC wide-code slots a capture's rung ran on, None when it ran
    on none: the slots of the constant set the rung names, or on a rung
    recorded before, those of the result file `rung.json` names, refused
    when the file is not the one recorded."""
    doc = rung_set(folder)
    if doc is not None:
        slots = doc["wide_codes"]
        return bytes(slots["slots"]) if slots is not None and any(slots["slots"]) else None
    named = rung_named(folder, "wide_codes")
    if not named:
        return None
    path = results.ROOT / named["file"]
    if not path.exists() or file_sha1(path) != named["sha1"]:
        raise ValueError(f"{named['file']} is not the wide-code result the rung ran on (sha1 {named['sha1']})")
    return widecodes.slots_of_file(path)


def rung_current_scale(folder, board):
    """The scale a capture's current is read at, the shunt's counts per
    A and where it is from. On a rung that names its constant set it is
    the set's own scale, the copy in the rung's folder, named by the
    set's scales hash and refused when that set holds none. On a rung
    recorded before, the board's stage 1 current-scale result
    (`scales.load`): the board is the one the rung ran on, and no rung
    is read at the registry's nominal parts."""
    doc = rung_set(folder)
    if doc is None:
        return {"counts_per_a": scales.load(board).counts_per_a, "from": BOARD_SCALE}
    cn = constants()
    entry = (doc.get("scales") or {}).get(cn.CURRENT)
    if entry is None:
        raise ValueError(f"{folder}: the rung's set holds no {cn.CURRENT}: its current has no scale to be read "
                         f"at, the board's stage 1 current-scale result was not there when the set was built")
    return {"counts_per_a": entry["value"], "from": "set", "hash": doc["hash"]["scales"]}


def load(folder):
    """A capture's rows on its time axis, with its plan, meta and board.
    On a rung that names wide-code slots `pos` is the raw pot with each
    stuck code read as the middle of its span (`widecodes.compensated`),
    as the servo read it. Under a LIVE table `pos` is that reading, or
    the raw pot on a rung without slots, read through the rung's table.
    Either way the raw code is kept as `pos_raw`. Rows gain `t` (s from
    the first row), `pos_f` (the median pot) and `i_ma` (the shunt's read
    current, NaN where the drive window was not read): the raw reading
    over the capture's own zero (`current_zero`), which is taken with the
    driver awake as the readings are, so the in-drive offset stage 1 found
    over the torque-off zero is not taken off again, through the board's
    measured counts per A (`rung_current_scale`). `polls` is
    segments.csv by item, each line the poll taken before its item."""
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
    slots = rung_wide_slots(folder)
    if slots is not None or table is not None:
        rows["pos_raw"] = rows["pos"]
    if slots is not None:
        middle = widecodes.compensated(rows["pos_raw"].to_numpy(), slots)
        rows["pos"] = middle if table is None else table.lin(middle)
    elif table is not None:
        rows["pos"] = postable.interp_q4(rows["pos_raw"].to_numpy(), table.points) / postable.GRID
    polls = pd.read_csv(folder / "segments.csv")
    segs = polls[polls["rows"] > 0]
    last = rows.groupby("seg")["tick"].max().reindex(segs["seg"]).to_numpy()
    start = dict(zip(segs["seg"], unwrap_ticks(segs["tick0"].to_numpy(), last)))
    tick_hz = meta["tick_hz"]
    abs_tick = rows["seg"].map(start).to_numpy() + rows["tick"].to_numpy()
    rows["t"] = (abs_tick - abs_tick.min()) / tick_hz
    rows["pos_f"] = rows.groupby("seg")["pos"].transform(median_pos)
    read = rows["window_valid"] == 1
    scale = rung_current_scale(folder, board)
    rows["i_ma"] = np.where(read, (rows["current_raw"] - current_zero(rows)) / scale["counts_per_a"] * 1e3, np.nan)
    return {"rows": rows, "plan": plan, "meta": meta, "cpd": plan["counts_per_deg"], "current_scale": scale,
            "counts_per_a": scale["counts_per_a"], "tick_hz": tick_hz, "folder": folder,
            "polls": polls.set_index("seg"), "table": table}


def plan_segs(cap, test):
    return [s for s in cap["plan"]["segments"] if s["test"] == test]


def seg_rows(cap, k):
    return cap["rows"][cap["rows"]["seg"] == k]


def polls_of(cap, *cols):
    """The capture's polls, None when segments.csv lacks one of `cols`."""
    p = cap["polls"]
    return None if any(c not in p for c in cols) else p


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


def position_report_error_deg(cap):
    """Over the polls of the steps capture that close a goal item (a poll
    is recorded on the next item's line): the reported position less the
    mean median pot over the last REPORT_POT_MS of the item it closes,
    where that pot spans no more than the servo's hold band there; the
    worst |error|. The report is `theta_hat_q16` in the counts the servo
    ran in and the pot as `load` reads it, in the same counts: under a
    LIVE table both are linearized, and the band is the capture's
    deadband times the table's band gain at the median raw code of that
    pot (`postable.Table.band_gain`), as the servo scales it; without a
    table the band is the deadband."""
    p = polls_of(cap, "theta_hat_q16")
    db = cap["meta"].get("config", {}).get("pos_deadband_counts")
    if p is None or db is None:
        return np.nan
    errs = []
    for s in goal_segs(cap):
        g = seg_rows(cap, s["seg"])
        if s["seg"] + 1 not in p.index or g.empty:
            continue
        last = tail(g, REPORT_POT_MS)
        pot = last["pos_f"]
        band = db
        if cap["table"] is not None:
            band = db * float(cap["table"].band_gain(round(last["pos_raw"].median())))
        if pot.max() - pot.min() <= band:
            errs.append(p.loc[s["seg"] + 1, "theta_hat_q16"] / Q16 - pot.mean())
    return float(np.max(np.abs(errs))) / cap["cpd"] if errs else np.nan


# --- the hold capture ---

def hold_scores(cap):
    """Over each hold, the worst of the two: the pot's sd, unfiltered,
    in degrees through the plan's counts per degree as every degree row,
    so it reads alike on a rung in raw counts and on one in a table's;
    the share of rows driven; the current rms. The first hold starts at
    torque-on, so its first HOLD_HEAD_MS is cut off; the other is read
    whole."""
    holds = [seg_rows(cap, s["seg"]) for s in plan_segs(cap, "hold")]
    holds[0] = holds[0][holds[0]["t"] >= holds[0]["t"].iloc[0] + HOLD_HEAD_MS / 1000]
    return {
        "hold_jitter_deg": max(float(g["pos"].std()) for g in holds) / cap["cpd"],
        "hold_buzz_share": max(float((g["duty_counts"] != 0).mean()) for g in holds),
        "hold_current_rms_ma": max(current_rms(g["i_ma"]) for g in holds),
    }


# --- the wide capture ---

def wide_scores(cap):
    """Over the last REST_MS of every hold at the middle of a wide code:
    the holds driven over a share above TAIL_DRIVEN_SHARE, the ones that
    hunt; and over those tails pooled, the share of rows driven and the
    current rms."""
    tails = [tail(seg_rows(cap, s["seg"]), REST_MS) for s in plan_segs(cap, "wide-hold")]
    pooled = pd.concat(tails)
    return {
        "wide_hold_hunting": sum(int((t["duty_counts"] != 0).mean() > TAIL_DRIVEN_SHARE) for t in tails),
        "wide_hold_driven_share": float((pooled["duty_counts"] != 0).mean()),
        "wide_hold_current_rms_ma": current_rms(pooled["i_ma"]),
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


def speed_report_error_deg_s(cap, segs, g):
    """One ramp, over the items in its follow tail: the mean of each poll's
    reported speed less the pot's own speed across the hole the poll was
    taken in, along the travel; negative reads slow. The poll on item k's
    line is taken in the hole between item k-1's rows and k's commit, so
    the pot's speed there is the mean `pos` over k's first
    REPORT_EDGE_ROWS rows less the mean over k-1's last, over the time
    between the two windows' centers on the rows' time axis. The report
    is `omega_hat_cps` in the counts the servo ran in and `pos` the pot
    as `load` reads it, in the same counts: the raw pot, a wide code read
    as the middle of its span, and under a LIVE table the linearized
    counts. The hole holds no rows, so this is the pot's mean speed across
    it, the commit included, against a report taken near its middle. An
    item without REPORT_EDGE_ROWS rows on either side of its hole reads
    nothing."""
    p = polls_of(cap, "omega_hat_cps")
    if p is None:
        return np.nan
    items = dict(tuple(g.groupby("seg")))
    errs = []
    for k in ramp_tail(g)["seg"].unique():
        before, after = items.get(k - 1), items[k]
        if before is None or len(before) < REPORT_EDGE_ROWS or len(after) < REPORT_EDGE_ROWS:
            continue
        a, b = before.iloc[-REPORT_EDGE_ROWS:], after.iloc[:REPORT_EDGE_ROWS]
        pot = (b["pos"].mean() - a["pos"].mean()) / (b["t"].mean() - a["t"].mean())
        errs.append(p.loc[k, "omega_hat_cps"] / Q16 - pot)
    return float(segs[0]["direction"] * np.mean(errs) / cap["cpd"]) if errs else np.nan


def item_swings(g):
    """Per goal item of a ramp's rows, the span of the median pot about
    the item's own line, counts: a shaft that keeps the ramp's pace reads
    the pot's noise, one that stops and jumps or swings back reads the
    travel it does so over."""
    out = []
    for _, q in g.groupby("seg"):
        t, p = q["t"].to_numpy() - q["t"].iloc[0], q["pos_f"].to_numpy()
        if len(p) > 2:
            res = p - np.polyval(np.polyfit(t, p, 1), t)
            out.append(res.max() - res.min())
    return np.array(out)


def follow_scores(cap):
    """Per speed, over the last FOLLOW_TAIL of its up and down ramps pooled:
    the lag (mean of goal - pos along the travel, the goal the one in force
    at each row), its sd, the current rms, the duty reversals per second
    and the duty rms;
    the goals' real speed, the mean of the two, and the lag as a time, the
    lag over that speed: a goal item takes as long as the host's exchanges
    around its burst take, so the same plan runs a few percent faster or
    slower from one sitting to the next, and a lag in degrees is
    comparable only at one ramp speed; the reported speed's error
    (speed_report_error_deg_s), the median ramp; and on the slowest speed
    the crawl ripple over CRAWL_WINDOW_MS windows, the worse ramp, and the
    crawl swing, the CRAWL_SWING_PCTL-th percentile of item_swings over
    every goal item of its ramps."""
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
        speed = float(np.mean([ramp_speed_deg_s(cap, s, g) for s, g in rs]))
        out[f"lag_deg_{tag}"] = float(err.mean())
        out[f"lag_ms_{tag}"] = 1000 * out[f"lag_deg_{tag}"] / speed
        out[f"follow_error_sd_deg_{tag}"] = float(err.std(ddof=1))
        out[f"follow_current_rms_ma_{tag}"] = current_rms(pd.concat(tails)["i_ma"])
        out[f"follow_reversals_per_s_{tag}"] = sum(reversals(t["duty_counts"]) for t in tails) / span_s
        out[f"follow_duty_rms_pct_{tag}"] = duty_rms_pct(cap, pd.concat(tails)["duty_counts"])
        out[f"ramp_speed_deg_s_{tag}"] = speed
        out[f"speed_report_error_deg_s_{tag}"] = float(np.median([speed_report_error_deg_s(cap, s, g)
                                                                  for s, g in rs]))
        if pct == slowest:
            out["crawl_speed_ripple_pct"] = max(
                ripple_pct(window_speeds(t["t"].to_numpy(), t["pos_f"].to_numpy(), CRAWL_WINDOW_MS / 1000))
                for t in tails)
            out["crawl_swing_deg"] = float(np.percentile(
                np.concatenate([item_swings(g) for _, g in rs]), CRAWL_SWING_PCTL)) / cap["cpd"]
    return out


# --- a run and a rung ---

def run_complete(run_dir):
    """Whether every capture of a run ended ok; `wide` counts only where
    the run has one."""
    run_dir = Path(run_dir)
    for c in ("steps", "follow", "hold", "wide"):
        m = run_dir / c / "meta.json"
        if c == "wide" and not m.parent.exists():
            continue
        if not m.exists() or json.loads(m.read_text()).get("ended") != "ok":
            return False
    return True


def score_run(run_dir):
    """Every score of one run, {key: value}; the wide rows only where the
    run has a `wide` capture."""
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
    out["position_report_error_deg"] = position_report_error_deg(steps)
    out["enable_drive_ms"] = max(enable_drive_ms(steps, "enable"), enable_drive_ms(hold, "hold"))
    out.update(hold_scores(hold))
    if (run_dir / "wide").exists():
        out.update(wide_scores(load(run_dir / "wide")))
    out.update(follow_scores(load(run_dir / "follow")))
    return out


def run_current_scale(run_dir):
    """The scale a run's current rows are at (`rung_current_scale`), the
    one every capture of a rung is read at."""
    folder = Path(run_dir) / "steps"
    return rung_current_scale(folder, boards.from_meta(json.loads((folder / "meta.json").read_text())))


def score_rung(rung_dir):
    """The rung's scores, the median over its complete runs with their
    range, the scale its current rows are at, and the runs scored and
    skipped."""
    rung_dir = Path(rung_dir)
    runs = sorted(p for p in rung_dir.glob("run-*") if p.is_dir())
    done = [p for p in runs if run_complete(p)]
    per = [score_run(p) for p in done]
    keys = list(dict.fromkeys(k for s in per for k in s))
    scores = {k: results.median_range([s.get(k, np.nan) for s in per], UNITS[base_of(k)]) for k in keys}
    return {"current_scale": run_current_scale(done[0]) if done else None, "scores": scores,
            "runs": [p.name for p in done], "skipped": [p.name for p in runs if p not in done]}


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


# The first field of CONTROL: CONFIG and CALIB, the regions a SAVE keeps,
# are the table from its first field up to this one's address.
CONTROL_FIRST = "torque_enable"
HASHED = ("addr", "raw", "pos_table", "wide_codes", "stamp", "data_state")


def state_hash(doc):
    """The hash of a state record: all of it but `fields`, which `raw`
    holds, `writable`, which is the descriptor's and not the servo's
    state, and the table's points, which their sha1 stands for."""
    covered = {k: doc[k] for k in HASHED}
    covered["pos_table"] = {k: doc["pos_table"][k] for k in ("state", "points_sha1")}
    return config_hash(covered)


def servo_state():
    """The servo's constant state as it holds it now. `raw` is the bytes
    of CONFIG and CALIB from `addr`, the span taken off the servo's own
    descriptor (`osc dump`): the table's first field to the first field
    of CONTROL. `fields` is every field there by name, as `osc dump`
    decodes it, and `writable` the names of those the descriptor lets a
    host write, in table order. With them the position table's state,
    its 256 points as fetched back, i16 LE in a row, and their sha1, the
    ADC wide-code slots, and the stamp and the data state of `osc
    status`. `hash` is `state_hash` of it."""
    rows = osc.dump()
    lo = min(addr for addr, _, _, _ in rows)
    end = next((addr for addr, name, _, _ in rows if name == CONTROL_FIRST), None)
    if end is None:
        raise SystemExit(f"osc dump names no {CONTROL_FIRST}: where CONFIG and CALIB end is unknown")
    raw = osc.read(lo, end - lo)
    lut_state, points = postable.on_servo()
    slots = widecodes.on_servo()
    status = osc.status()
    doc = {
        "addr": lo, "raw": raw.hex(),
        "fields": {name: v for addr, name, _, v in rows if addr < end},
        "writable": [name for addr, name, access, _ in rows if addr < end and access == "rw"],
        "pos_table": {"state": lut_state, "points_sha1": hashlib.sha1(points).hexdigest(), "points": points.hex()},
        "wide_codes": None if slots is None else slots.hex(),
        "stamp": settings.plant_stamp(status), "data_state": settings.data_state(status),
    }
    return {**doc, "hash": state_hash(doc)}


def differing(first, now):
    """What constant state `now` holds other than `first`, each as
    `name: old -> new`: the fields by name, the table, the slots, the
    stamp and the data state, and failing those the bytes no field
    names."""
    def items(doc):
        return {**doc["fields"], "position table state": doc["pos_table"]["state"],
                "position table points sha1": doc["pos_table"]["points_sha1"],
                "wide-code slots": doc["wide_codes"], "stamp": doc["stamp"], "data state": doc["data_state"]}

    a, b = items(first), items(now)
    named = [f"{n}: {a.get(n)} -> {b.get(n)}" for n in dict.fromkeys([*a, *b]) if a.get(n) != b.get(n)]
    if named:
        return named
    x, y = bytes.fromhex(first["raw"]), bytes.fromhex(now["raw"])
    return ([f"byte 0x{first['addr'] + i:04x}: {p:02x} -> {q:02x}" for i, (p, q) in enumerate(zip(x, y)) if p != q]
            or [f"span: {len(x)} B at 0x{first['addr']:04x} -> {len(y)} B at 0x{now['addr']:04x}"])


def record_constants(r, k):
    """The servo's constant state before run `k` of rung `r`, into the
    run's `servo.json`, with a copy of the rung's set beside `rung.json`
    on the first. A servo that is not the rung's set, or whose data
    state is not clean, is refused before anything is recorded or
    driven: the run does not start. The rung's first state is kept on
    `r`, for `rung.json`'s hash; a later run that reads another is
    refused the same way, what differs named."""
    cn = constants()
    now = servo_state()
    off = cn.differences(r["set"], now)
    if off or now["data_state"] != "clean":
        raise SystemExit(f"{shown(r['dir'])}: run-{k} does not start: the servo is not the rung's set (data state "
                         f"{now['data_state']}{''.join('; ' + d for d in off)})")
    first = r.setdefault("constants", now)
    if now["hash"] != first["hash"]:
        raise SystemExit(f"{shown(r['dir'])}: run-{k} reads a constant state other than the one the rung's "
                         f"first run recorded: {'; '.join(differing(first, now))}")
    out = r["dir"] / f"run-{k}"
    out.mkdir(parents=True, exist_ok=True)
    (out / "servo.json").write_text(json.dumps(now, indent=2) + "\n")
    if not (r["dir"] / SET_COPY).exists():
        cn.write(r["set"], r["dir"] / SET_COPY)


class Lost(Exception):
    """A capture `osc-lab track` aborted because a telemetry burst came
    back short: rows lost on the link, not a result of the servo."""

    def __init__(self, capture, reason):
        super().__init__(f"{capture}: {reason}")
        self.capture, self.reason = capture, reason


class GaveUp(SystemExit):
    """A run that lost a burst on its last try too."""


def short_burst(out):
    """The reason an aborted capture's meta.json ends on, where it is a
    burst that came back short; None for any other end."""
    p = Path(out) / "meta.json"
    ended = json.loads(p.read_text()).get("ended") if p.exists() else None
    reason = ended.get("reason") if isinstance(ended, dict) else None
    return reason if reason and reason.startswith(SHORT_BURST) else None


def capture(out, name, segs, center, guard, inp, deadman_check, ts):
    """One capture: its plan.json, then `osc-lab track`, then the check
    that the servo ran the range the plan was built for: the guard and
    the middle of it `osc-lab track` centers to at the end, which on a
    table rung is not the plan's own center. An abort on a short burst
    raises `Lost`, any other as the tool's own failure."""
    out.mkdir(parents=True, exist_ok=True)
    (out / "plan.json").write_text(json.dumps(ts.plan_doc(name, segs, center, guard, inp), indent=2) + "\n")
    try:
        osc.track(out, "position", [s["item"] for s in segs],
                  current_cap_ma=inp["current_cap_ma"], current_cap_ms=inp["current_cap_ms"],
                  deadman_check=deadman_check)
    except subprocess.CalledProcessError:
        reason = short_burst(out)
        if reason is None:
            raise
        raise Lost(name, reason) from None
    meta = json.loads((out / "meta.json").read_text())
    middle = ts.center_of(guard)
    if tuple(meta["guard"]) != tuple(guard) or meta["center"] != middle:
        raise SystemExit(f"{shown(out)}: the servo ran guard {meta['guard']} center {meta['center']}, "
                         f"the plan was built for {list(guard)} center {middle}")
    return meta


def new_rung(pair, label, fw, image_sha, runs, note="", set_file=None, deviation=None, protection=(), loosen=()):
    """A rung to record, refused when its label already holds one: its
    folder, what `rung.json` names and the constant set it runs on,
    which is the set of `set_file`, the pair's set of record without
    one, or that set with the one deviation of file `deviation`
    (`constants.derive`). Refused offline when the set is not the
    pair's, an input file of it moved, or the deviation does not build.
    `protection` and `loosen` are the names the set's install passes
    its protection gate with."""
    cn = constants()
    rung = rung_dir(pair, label)
    if rung.exists():
        raise SystemExit(f"{shown(rung)} already holds a rung: pick a new label or remove it")
    root = results.ROOT
    try:
        path = Path(set_file) if set_file else cn.set_path(pair, root)
        doc = cn.load(path) if set_file else cn.of_record(pair, root)
        named = cn.in_repo(path, root, "the set a rung runs on").as_posix()
        if doc["pair"] != pair:
            raise ValueError(f"{shown(path)} is the set of {doc['pair']}, the servo at hand is {pair}")
        if deviation:
            doc, _ = cn.derive(doc, deviation, root)
        lines, stale = cn.status(doc, root)
        if stale:
            raise ValueError(f"the set is stale: {'; '.join(x for x in lines if x.startswith('stale: '))}")
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"{label}: {e}")
    return {"dir": rung, "label": label, "fw": fw, "image_sha": image_sha, "note": note, "runs": runs,
            "set": doc, "set_file": named, "protection": tuple(protection), "loosen": tuple(loosen)}


def bench_sets(servo, board, pair, rungs):
    """The test set built for each of `rungs` from the constant set it
    runs on, after one preflight: the module, the rung's inputs, the
    set's soft range, the center and the captures' segments. A rung on
    a set with a LIVE table gets the test set planned in the table's
    counts."""
    ts = testset()
    try:
        inps = [ts.inputs(servo, pair, board.key, r["set"]) for r in rungs]
    except settings.Refused as e:
        raise SystemExit(str(e))
    guards = [ts.guard_of(r["set"]) for r in rungs]
    preflight(servo, board)
    out = []
    for inp, guard in zip(inps, guards):
        center, sets = ts.build(guard, inp)
        out.append({"ts": ts, "inp": inp, "guard": guard, "center": center, "sets": sets})
    return out


@contextmanager
def applied(r):
    """Rung `r`'s constant set on the servo for the body:
    `constants.install`, the one place a state is stamped, behind its
    protection gate, and the protection fields it moves against the
    servo's values kept on `r` for `rung.json`. A reboot brings the
    stored state back when the body ends or aborts, where the install
    wrote anything, and always after a run given up as lost
    (`stored_back`)."""
    cn = constants()
    try:
        state = servo_state()
        r.setdefault("protection_moved", cn.protection_moves(r["set"], state))
        wrote = cn.install(r["set"], r["protection"], r["loosen"], results.ROOT, state=state)
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"{r['label']}: the rung's set does not install: {e}")
    gave_up = False
    try:
        yield
    except GaveUp:
        gave_up = True
        raise
    finally:
        # the stored state comes back whether the runs ended or aborted
        if wrote or gave_up:
            stored_back(state["data_state"])


def stored_back(data_state):
    """Reboot onto the stored state and wait for the bus. The data
    state must then be `data_state`, the one the servo held before the
    install: clean, or on an image whose stored stamp is from before
    its covered fields changed the mismatch every boot of it gives,
    which the install stamps away for the run alone."""
    osc.run("reboot")
    settings.await_boot()
    status = osc.status()
    now = settings.data_state(status)
    if now != data_state:
        raise settings.Refused(f"after the reboot the data state is {now}, not the {data_state} of before the "
                               f"install")
    print(f"rebooted: data state {now}, plant stamp {settings.plant_stamp(status)}")


def record_run(r, k, s):
    """Run `k` of rung `r` on bench set `s`: the servo's constant state
    (`record_constants`), then the captures the set built, in the test
    set's order, and `rung.json` once the rung's first capture lands:
    the `osc` tool the rung's bus calls go through (`osc.identity`),
    the constant set by its file, its three hashes and its fields by
    class, the deviation with every field it moved as [of record,
    deviated], the protection hash with what the install moved against
    the servo's values, the wide holds' goals, and the runs lost and
    the waits for the host's load before it, where there were any
    (`keep`)."""
    cn = constants()
    rung, ts = r["dir"], s["ts"]
    record_constants(r, k)
    wide = [{"stuck_code": g["stuck_code"], "side": g["side"], "goal_counts": g["goal_counts"],
             **({"raw_counts": g["raw_counts"]} if "raw_counts" in g else {})}
            for g in s["sets"].get("wide", []) if g["test"] == "wide-hold"]
    for i, name in enumerate(c for c in ts.CAPTURES if c in s["sets"]):
        meta = capture(rung / f"run-{k}" / name, name, s["sets"][name], s["center"], s["guard"], s["inp"],
                       deadman_check=i == 0, ts=ts)
        if not (rung / "rung.json").exists():
            (rung / "rung.json").write_text(json.dumps({
                "label": r["label"], "fw": r["fw"], "image_sha": r["image_sha"], "tool": r["tool"],
                "note": r["note"],
                "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                "config_hash": config_hash(meta["config"]), "constants_hash": r["constants"]["hash"],
                "runs_asked": r["runs"],
                "set": {"file": r["set_file"], "hash": r["set"]["hash"], "classes": cn.classes(r["set"])},
                "deviation": r["set"].get("deviation"),
                "protection": {"hash": r["set"]["hash"]["protection"], "moved": r["protection_moved"]},
                "wide_goals": wide or None,
                **{key: r[key] for key in KEPT if key in r},
            }, indent=2) + "\n")


KEPT = ("lost", "load_waits")   # what `keep` adds to a rung.json, absent from a rung with none


def keep(r, key, entry):
    """`entry` on rung `r`'s list `key`, and in its `rung.json` where
    the rung has one; a rung with none yet writes the list with it."""
    r.setdefault(key, []).append(entry)
    p = r["dir"] / "rung.json"
    if p.exists():
        p.write_text(json.dumps({**json.loads(p.read_text()), key: r[key]}, indent=2) + "\n")


def host_load():
    """The host's one minute load."""
    return round(os.getloadavg()[0], 2)


def await_load(r, k, max_load):
    """Wait before run `k` of rung `r` while the host's one minute load
    is over `max_load`, LOAD_WAIT_S at most, then go on: the telemetry
    link drops rows on a loaded host. A wait is kept in `rung.json`,
    with the load it began and ended at and whether the run went on
    over the limit. None gates nothing."""
    if max_load is None:
        return
    first = load = host_load()
    waited = 0
    if load > max_load:
        print(f"{r['label']} run-{k} waits for the host: load {load} is over {max_load:g}")
    while load > max_load and waited < LOAD_WAIT_S:
        time.sleep(LOAD_POLL_S)
        waited += LOAD_POLL_S
        load = host_load()
    if waited:
        keep(r, "load_waits", {"run": f"run-{k}", "max_load": max_load, "load_1m_before": first,
                               "load_1m_after": load, "waited_s": waited, "went_on_over": load > max_load})


def recorded(r, k, s, servo, board, max_load=None):
    """`record_run` with the rung's set on the servo. A run that loses a
    capture to a short burst is no result: its folder is moved under
    the rung's `lost/`, the loss is kept in `rung.json` with the
    capture, the tool's reason and the host's load, and the run starts
    again from its first capture after the load gate and the preflight,
    the servo read again and checked against the set, RETRIES times at
    most. A run lost on its last try too ends the session, torque off."""
    for n in range(1, RETRIES + 2):
        if n > 1:
            await_load(r, k, max_load)
            preflight(servo, board)
        try:
            return record_run(r, k, s)
        except Lost as e:
            lost = e
            aside = Path("lost") / f"run-{k}-{n}"
            (r["dir"] / aside.parent).mkdir(exist_ok=True)
            (r["dir"] / f"run-{k}").rename(r["dir"] / aside)
            keep(r, "lost", {"run": f"run-{k}", "capture": e.capture, "reason": e.reason,
                             "load_1m": host_load(), "rows": aside.as_posix()})
            print(f"{r['label']} run-{k} lost in {e.capture}: {e.reason}")
    if osc.get("torque_enable") not in ("off", 0):
        osc.set_("torque_enable", "off")
    raise GaveUp(f"{shown(r['dir'])}: run-{k} lost a burst on each of {RETRIES + 1} tries, the last in "
                 f"{lost.capture}: {lost.reason}")


def run(label, fw, image_sha, runs=3, note="", set_file=None, deviation=None, protection=(), loosen=(),
        servo=None, board=None, max_load=None):
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    r = new_rung(pair, label, fw, image_sha, runs, note, set_file, deviation, protection, loosen)
    r["tool"] = osc.identity()
    s, = bench_sets(servo, board, pair, [r])
    with applied(r):
        for k in range(1, runs + 1):
            await_load(r, k, max_load)
            if k > 1:
                preflight(servo, board)
            recorded(r, k, s, servo, board, max_load)
            print(f"run-{k} of {runs} done")
    return r["dir"]


def ab(*sides, rounds=3, order=None, max_load=None, servo=None, board=None):
    """Two or three rungs recorded alternately, one run of each per
    round in `order`, a permutation of the sides' letters ("ab", "cab"),
    a then b then c without one, so all share the servo's drift through
    the session. A side is {"label", "fw", "image", "note", "set_file",
    "deviation", "protection", "loosen", "osc"}, the last six optional;
    each rung is the one `run` records, its image sha the image file's.
    A side's turn is everything from its flash to the reboot after its
    run, and every `osc` call in it goes through the side's own tool,
    the file of "osc" (`osc.using`), which is the default again outside
    it. In a turn the side's image is flashed and the bus awaited
    whenever the servo holds another image, and before the first run
    always. No stamp is asked of the image as it boots: the side's
    constant set, the set of record or its own derived set, is
    installed around its own run alone, on the image it runs on, and
    the install makes the servo the set field by field, stamps it where
    the data state is not clean and requires clean, so a side whose
    install cannot do that does not start. Its test set is planned from
    the set. A tool that
    is not found, or is another binary at a later turn than at the
    session's start, is refused. Returns the rungs' folders, a first."""
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    letters = SIDES[:len(sides)]
    if len(sides) not in (2, 3):
        raise SystemExit(f"{len(sides)} sides: an alternated session records two rungs or three")
    order = order or letters
    if sorted(order) != sorted(letters):
        raise SystemExit(f"order {order!r} is not the sides {', '.join(letters)} once each")
    labels = [side["label"] for side in sides]
    twice = sorted({x for x in labels if labels.count(x) > 1})
    if twice:
        raise SystemExit(f"two sides are labeled {twice[0]!r}: each side records its own rung")
    rungs = []
    for side in sides:
        r = new_rung(pair, side["label"], side["fw"], file_sha1(side["image"]), rounds, side.get("note", ""),
                     side.get("set_file"), side.get("deviation"), side.get("protection", ()),
                     side.get("loosen", ()))
        r["image"], r["osc"] = Path(side["image"]), side.get("osc")
        with osc.using(r["osc"]):
            r["tool"] = osc.identity()
        if r["osc"] and r["tool"] is None:
            raise SystemExit(f"{r['label']}: no osc tool at {r['osc']}")
        rungs.append(r)
    sets = bench_sets(servo, board, pair, rungs)
    turns = [(rungs[letters.index(x)], sets[letters.index(x)]) for x in order]
    on_servo = None
    flashing = None
    try:
        for k in range(1, rounds + 1):
            for r, s in turns:
                with osc.using(r["osc"]):
                    if osc.identity() != r["tool"]:
                        raise SystemExit(f"{r['label']}: its osc tool is not the binary the session began with")
                    if on_servo is None or on_servo["image_sha"] != r["image_sha"]:
                        on_servo, flashing = None, r
                        osc.flash(r["image"])
                        on_servo, flashing = r, None
                        settings.await_boot()
                    await_load(r, k, max_load)
                    preflight(servo, board)
                    with applied(r):
                        recorded(r, k, s, servo, board, max_load)
                print(f"{r['label']} run-{k} of {rounds} done")
    finally:
        if on_servo:
            print(f"left on the servo: {shown(on_servo['image'])} (sha1 {on_servo['image_sha']})")
        elif flashing:
            print(f"the flash of {shown(flashing['image'])} did not finish: the servo's image is unknown")
    return tuple(r["dir"] for r in rungs)


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


def carry(servo=None, board=None):
    """Carry the pair's results that record no current scale to the
    board's measured one, and return the files carried. Such a result
    was scored at the board's nominal parts, and its rung's rows were
    not kept, so it cannot be scored again: its mA rows, value and
    range, are multiplied by nominal over measured counts per A, which
    is what scoring the same rows at the measured scale gives, and the
    file then records the measured scale and that its rows were carried
    there. No other row is touched, and a file that records a scale is
    left as it is. Refused where a rung's rows are kept: `score` reads
    those."""
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = pair_of(servo, board)
    files = sorted(result_path(pair, "x").parent.glob("*.json"))
    todo = [(p, doc) for p, doc in ((p, json.loads(p.read_text())) for p in files) if "current_scale" not in doc]
    kept = [doc["label"] for _, doc in todo if (rung_dir(pair, doc["label"]) / "rung.json").exists()]
    if kept:
        raise SystemExit(f"the rows of {', '.join(kept)} are kept: score a rung whose rows are there, "
                         f"only a result with no rows is carried")
    if not todo:
        return []
    nominal, measured = board.counts_per_a, scales.load(board).counts_per_a
    scale = {"counts_per_a": measured, "from": BOARD_SCALE,
             "carried": {"counts_per_a": nominal, "from": "the board's nominal parts",
                         "how": "the mA rows times nominal over measured counts per A, not scored again: "
                                "the rung's rows were not kept"}}
    for p, doc in todo:
        for q in doc["scores"].values():
            if q["unit"] == "mA":
                q.update({k: q[k] * nominal / measured for k in ("value", "lo", "hi") if q[k] is not None})
        items = list(doc.items())
        at = list(doc).index("scores")
        p.write_text(json.dumps(results._plain(dict([*items[:at], ("current_scale", scale), *items[at:]])),
                                indent=2) + "\n")
    return [shown(p) for p, _ in todo]


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


def set_args(parser, side):
    """The flags that name the constant set a rung, or a side of an A/B,
    runs on."""
    parser.add_argument(f"--{side}set", type=Path,
                        help="the constant set to run on, the pair's set of record if omitted")
    parser.add_argument(f"--{side}deviation", type=Path,
                        help="one deviation file, built over that set by its rules")
    parser.add_argument(f"--{side}protection", nargs="+", default=[], metavar="NAME",
                        help="the protection fields the set's install may change on the servo")
    parser.add_argument(f"--{side}loosen", nargs="+", default=[], metavar="NAME",
                        help="of those, the ones it may set to protect less than the servo's value")


LOAD_HELP = (f"wait before a run while the host's one minute load is over this, {LOAD_WAIT_S} s at most, "
             f"then go on")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.scorecard", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="record a rung on the bench")
    r.add_argument("--label", required=True)
    r.add_argument("--fw", required=True, help="the firmware commit the image was built from")
    r.add_argument("--image-sha", required=True, help="the sha1 of the flashed image")
    r.add_argument("--runs", type=int, default=3)
    r.add_argument("--note", default="")
    set_args(r, "")
    r.add_argument("--max-load", type=float, help=LOAD_HELP)
    x = sub.add_parser("ab", help="record two or three rungs on the bench, their runs alternated")
    for side in SIDES:
        need = side != SIDES[2]
        x.add_argument(f"--{side}-label", required=need)
        x.add_argument(f"--{side}-fw", required=need, help="the firmware commit the image was built from")
        x.add_argument(f"--{side}-image", required=need, type=Path, help="the raw .bin image to flash")
        set_args(x, f"{side}-")
        x.add_argument(f"--{side}-note", default="")
        x.add_argument(f"--{side}-osc", help="the osc tool every bus call of the side's turn goes through, "
                                              "the one of OSC or the PATH if omitted")
    x.add_argument("--rounds", type=int, default=3)
    x.add_argument("--order", help="the sides' order within a round, their letters once each: abc if omitted")
    x.add_argument("--max-load", type=float, help=LOAD_HELP)
    s = sub.add_parser("score", help="score a recorded rung")
    s.add_argument("label")
    sub.add_parser("carry", help="carry the results of rungs whose rows were not kept to the measured current scale")
    sub.add_parser("table", help="print the ladder")
    a = ap.parse_args(argv)
    if a.cmd == "run":
        print(shown(run(a.label, a.fw, a.image_sha, a.runs, a.note, a.set, a.deviation, a.protection, a.loosen,
                        max_load=a.max_load)))
    elif a.cmd == "ab":
        flags = {"label": "label", "fw": "fw", "image": "image", "note": "note", "set_file": "set",
                 "deviation": "deviation", "protection": "protection", "loosen": "loosen", "osc": "osc"}
        sides = [{key: getattr(a, f"{side}_{flag}") for key, flag in flags.items()} for side in SIDES]
        if not any(sides[2].values()):
            sides.pop()
        elif None in (sides[2]["label"], sides[2]["fw"], sides[2]["image"]):
            ap.error("a third side needs --c-label, --c-fw and --c-image")
        for rung in ab(*sides, rounds=a.rounds, order=a.order, max_load=a.max_load):
            print(shown(rung))
    elif a.cmd == "score":
        print(score(a.label))
    elif a.cmd == "carry":
        for p in carry():
            print(p)
    else:
        table()


if __name__ == "__main__":
    main()
