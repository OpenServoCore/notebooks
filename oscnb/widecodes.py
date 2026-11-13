"""The ADC's wide codes, the board's converter's, found from a slow
closed-loop sweep and written to the servo's `adc_wide_mid_q4` field.

A wide code is one the converter holds while its input rises through it
and the codes above it: the reading sticks at a code just under a
multiple of 256, then resumes at the right value, and the codes between
are never read. The codes are the board's chip, found once per board. The
pot of the servo on the board is only the ramp: swept slowly enough that
every code gets many rows, a code never read inside the swept range is a
code the converter skips, and the code just under a run of them reads
several times the median rows.

`adc_wide_mid_q4` is 16 bytes, one slot per carry: slot k is added, in
1/16 counts, to the position the servo reads when the converter returns
exactly (k << 8) | 0xff on the pot. A stuck code holds the middle of the
span it covers, so a slot holds half the skipped span, 8 x the codes
skipped, in Q4. A run under any other code, one too wide for a byte, one
of fewer than MIN_SKIPPED codes (a step inside the converter's own ripple
from code to code) or one whose code underneath does not read wide (a
thin stretch, not a stuck code) is reported and left out of the slots.

    uv run python -m oscnb.widecodes capture [--settings FILE] [--label L]
    uv run python -m oscnb.widecodes find [--label L]
    uv run python -m oscnb.widecodes install

`capture` sweeps the bench servo's pot closed loop, the soft range less
the scorecard's fit margin, `PASSES` passes at `SPEED_CPS`, into
`telemetry/<board>/wide-codes/<label>/`, under `velocity_limit_cps` and
the file's fields applied volatile and rebooted away at the end;
`sweep.json` there names the servo that served as the ramp. `find` reads
a recorded sweep, prints the ADC's wide codes and writes the board's
result, `stages/1-board/wide-codes/results/<board>.json`. `install`
writes the board's slots to the servo volatile and reads them back;
nothing here sends a SAVE.
"""
import argparse
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pandas as pd

from . import boards, osc, results, servos, settings
from . import scorecard as sc

STAGE, EXPERIMENT = "1-board", "wide-codes"
FIELD = "adc_wide_mid_q4"
ADC_MASK = (1 << 12) - 1
CARRY = 0xFF                # the low byte of a code the field can hold
SLOT_SHIFT = 8
SLOTS = (ADC_MASK + 1) >> SLOT_SHIFT
Q4_PER_SKIPPED = 8          # half a skipped code, in 1/16 counts
SLOT_MAX = 0xFF
# A stuck code stands for itself and the codes it skips, so it reads
# (1 + skipped) x the median rows; under this share of that the code under
# a run is not what holds the reading and the run is a thin stretch.
WIDE_MIN_SHARE = 0.5
# A run shorter than this moves the reading by a count or less at its
# middle, inside the ripple the converter shows from code to code anyway,
# and one or two codes never read cannot be told from a thin stretch.
MIN_SKIPPED = 3
# The codes next to a turnaround are crossed slowly or not at all, so a run
# there says nothing about the converter.
EDGE_MARGIN_COUNTS = 15
# Under this median a code a tenth of the nominal width expects fewer than
# 10 rows, and one never read may be chance rather than a skipped code.
MIN_ROWS_PER_CODE = 100
# The reference sweep, 400 counts/s over four passes at the 20 kHz row rate,
# read a median 234 rows per code.
SPEED_CPS = 400
PASSES = 4
OPENER_COUNTS = 100         # the scorecard tools' drive-direction opener, out then back
OPENER_MS = (400, 600)
TURN_MS = 700               # past the travel at SPEED_CPS: the acceleration and the landing at each end
ITEM_MAX_MS = 3000          # one goal item, under osc-lab track's 3276 ms limit


@dataclass(frozen=True)
class Band:
    """A run of codes never read: the code just under it, the codes it
    skips and how many times the median rows the stuck code reads."""
    stuck: int
    skipped: int
    ratio: float

    @property
    def value(self):
        return Q4_PER_SKIPPED * self.skipped

    @property
    def wide(self):
        return self.skipped >= MIN_SKIPPED and self.ratio >= WIDE_MIN_SHARE * (1 + self.skipped)

    @property
    def representable(self):
        return self.stuck & CARRY == CARRY and self.value <= SLOT_MAX

    @property
    def slot(self):
        return self.stuck >> SLOT_SHIFT if self.wide and self.representable else None


@dataclass(frozen=True)
class Finding:
    """The ADC's wide codes a sweep found: the bands, the median rows per
    code over the codes read from (the swept range less
    EDGE_MARGIN_COUNTS at each end) and the swept range."""
    bands: list
    median_rows: float
    lo: int
    hi: int

    @property
    def inside(self):
        return self.lo + EDGE_MARGIN_COUNTS, self.hi - EDGE_MARGIN_COUNTS


def find(pos, lo, hi):
    """The ADC's wide codes from a sweep from `lo` to `hi` counts, its raw
    pot rows `pos`: every run of codes never read lying wholly inside the range less
    EDGE_MARGIN_COUNTS at each end. Raises when the sweep is too thin to
    tell a skipped code from chance."""
    pos = np.asarray(pos, np.int64)
    h = np.bincount(pos[(pos >= 0) & (pos <= ADC_MASK)], minlength=ADC_MASK + 1)
    a, b = lo + EDGE_MARGIN_COUNTS, hi - EDGE_MARGIN_COUNTS
    if b - a < 2:
        raise ValueError(f"the sweep {lo}..{hi} leaves no codes inside its {EDGE_MARGIN_COUNTS}-count margins")
    med = float(np.median(h[a:b + 1]))
    if med < MIN_ROWS_PER_CODE:
        raise ValueError(f"the sweep reads a median {med:g} rows per code over {a}..{b}, under the "
                         f"{MIN_ROWS_PER_CODE} that tells a skipped code from chance: sweep slower or longer")
    zero = np.r_[False, h == 0, False].astype(np.int8)
    edges = np.diff(zero)
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1) - 1
    bands = [Band(int(s) - 1, int(e - s + 1), float(h[s - 1] / med))
             for s, e in zip(starts, ends) if s >= a and e <= b]
    return Finding(bands, med, lo, hi)


def slots(bands):
    """The 16 bytes of `adc_wide_mid_q4` for `bands`: each wide,
    representable band's value at its slot, zero elsewhere."""
    out = bytearray(SLOTS)
    for band in bands:
        if band.slot is not None:
            out[band.slot] = band.value
    return bytes(out)


# --- the files ---

def captures_dir(board):
    """The board's sweeps, `board` its registry key."""
    return results.ROOT / "telemetry" / board / EXPERIMENT


def result_file(board):
    return results.path(STAGE, EXPERIMENT, board)


def complete(folder):
    m = Path(folder) / "meta.json"
    return m.exists() and json.loads(m.read_text()).get("ended") == "ok"


def label_key(p):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", p.name)]


def latest(board):
    """The last complete sweep's folder by label, refused when none is."""
    done = sorted((p for p in captures_dir(board).glob("*") if p.is_dir() and complete(p)), key=label_key)
    if not done:
        raise SystemExit(f"no complete sweep under {sc.shown(captures_dir(board))}")
    return done[-1]


def next_label(board):
    taken = {p.name for p in captures_dir(board).glob("sweep-*")}
    return next(f"sweep-{k}" for k in range(1, len(taken) + 2) if f"sweep-{k}" not in taken)


def slots_of_record(board):
    """The 16 slot values of the board's result file, refused when it
    holds no 16 bytes."""
    path = result_file(board)
    if not path.exists():
        raise settings.Refused(f"no ADC wide-code result for board {board}: run `find` on a sweep first")
    v = json.loads(path.read_text()).get(FIELD)
    if not isinstance(v, list) or len(v) != SLOTS or not all(isinstance(x, int) and 0 <= x <= SLOT_MAX for x in v):
        raise settings.Refused(f"{sc.shown(path)}: {FIELD} is not {SLOTS} values 0..{SLOT_MAX}")
    return bytes(v)


# --- the capture ---

def leg(goal, start, test, center, cpd, ts, **fields):
    """The items that take the goal from `start` to `goal` at SPEED_CPS, one
    goal re-issued over as many items as keep each under ITEM_MAX_MS."""
    ms = round(abs(goal - start) / SPEED_CPS * 1000) + TURN_MS
    n = math.ceil(ms / ITEM_MAX_MS)
    return [ts._goal(center, cpd, goal, math.ceil(ms / n), test, **fields) for _ in range(n)]


def segments(center, cpd, lo, hi, ts):
    """The sweep's segments: the opener, the approach to `hi`, PASSES
    passes between `hi` and `lo`, and the way home to the center."""
    segs = [ts._goal(center, cpd, center + OPENER_COUNTS, OPENER_MS[0], "opener"),
            ts._goal(center, cpd, center, OPENER_MS[1], "opener")]
    segs += leg(hi, center, "approach", center, cpd, ts)
    at = hi
    for k in range(PASSES):
        to = lo if at == hi else hi
        segs += leg(to, at, "pass", center, cpd, ts, pass_k=k, direction=1 if to > at else -1)
        at = to
    return segs + leg(center, at, "home", center, cpd, ts)


def swept(plan):
    """(lo, hi) of a sweep's plan: the passes' goals."""
    goals = [s["goal_counts"] for s in plan["segments"] if s["test"] == "pass"]
    return min(goals), max(goals)


def capture(label=None, settings_file=None, servo=None, board=None):
    """One sweep on the bench servo's pot into
    `telemetry/<board>/wide-codes/<label>`; returns its folder."""
    servo = servo or servos.selected()
    board = board or boards.BOARDS[boards.DEFAULT]
    pair = sc.pair_of(servo, board)
    label = label or next_label(board.key)
    out = captures_dir(board.key) / label
    if out.exists():
        raise SystemExit(f"{sc.shown(out)} already holds a sweep: pick a new label or remove it")
    fields = dict(settings.load(settings_file)[0]) if settings_file else {}
    fields["velocity_limit_cps"] = SPEED_CPS
    ts = sc.testset()
    inp = ts.inputs(servo, pair)
    sc.preflight(servo, board)
    guard = (osc.get("pos_min_soft_counts"), osc.get("pos_max_soft_counts"))
    cpd = inp["counts_per_deg"]
    margin = ts.FIT_MARGIN_DEG * cpd
    lo, hi = math.ceil(guard[0] + margin), math.floor(guard[1] - margin)
    center = ts.center_of(guard)
    segs = segments(center, cpd, lo, hi, ts)
    ts.check_fits(segs, guard, cpd)
    try:
        settings.apply(fields)
        meta = sc.capture(out, "sweep", segs, center, guard, inp, deadman_check=True, ts=ts)
    finally:
        # the stored set comes back whether the sweep ended or aborted
        settings.restore()
    (out / "sweep.json").write_text(json.dumps({
        "label": label, "utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "board": board.key, "ramp_servo": servo.key,
        "settings_file": str(sc.shown(settings_file)) if settings_file else None, "fields": fields,
        "speed_cps": SPEED_CPS, "passes": PASSES, "swept_counts": [lo, hi],
        "config_hash": sc.config_hash(meta["config"]),
    }, indent=2) + "\n")
    return out


# --- find and install ---

def find_capture(folder):
    """`find` on a recorded sweep's rows and plan."""
    folder = Path(folder)
    lo, hi = swept(json.loads((folder / "plan.json").read_text()))
    pos = pd.read_csv(folder / "track.csv", usecols=["pos"])["pos"].to_numpy()
    return find(pos, lo, hi)


def show(f):
    """The ADC's wide codes as a table, one line a band."""
    a, b = f.inside
    lines = [f"swept {f.lo}..{f.hi} counts, read over {a}..{b}: median {f.median_rows:g} rows per code",
             "stuck  skipped  x median  wide  slot  value  representable"]
    for band in f.bands:
        slot = "-" if band.slot is None else str(band.slot)
        lines.append(f"{band.stuck:5d}  {band.skipped:7d}  {band.ratio:8.1f}  {'yes' if band.wide else 'no':>4}  "
                     f"{slot:>4}  {band.value:5d}  "
                     f"{'yes' if band.representable else 'no'}")
    if not f.bands:
        lines.append("(no code inside the range went unread)")
    lines.append(f"{FIELD}: {' '.join(str(v) for v in slots(f.bands))}")
    return "\n".join(lines)


def ramp_servo(folder):
    """The servo whose pot served as a sweep's ramp, off its `sweep.json`;
    None when the sweep names none."""
    p = Path(folder) / "sweep.json"
    return json.loads(p.read_text()).get("ramp_servo") if p.exists() else None


def write_result(board, folder, f):
    """The board's result file from finding `f` on sweep `folder`."""
    folder = Path(folder)
    return results.write(
        STAGE, EXPERIMENT, folder, pair=board,
        notes=f"the ADC's wide codes from sweep {folder.name}, the pot of the ramp servo as the ramp: "
              f"runs of codes never read, each slot {Q4_PER_SKIPPED} x the codes skipped in 1/16 counts "
              f"at code (k << 8) | 0xff",
        label=folder.name,
        ramp_servo=ramp_servo(folder),
        swept_lo_counts=results.q(f.lo, unit="counts"),
        swept_hi_counts=results.q(f.hi, unit="counts"),
        edge_margin_counts=results.q(EDGE_MARGIN_COUNTS, unit="counts"),
        median_rows_per_code=results.q(f.median_rows, unit="rows"),
        wide_codes=[{"stuck_code": b.stuck, "skipped": b.skipped, "ratio": b.ratio, "wide": b.wide,
                     "representable": b.representable, "slot": b.slot, "value_q4": b.value}
                    for b in f.bands],
        **{FIELD: list(slots(f.bands))},
    )


def install(board):
    """Write the board's slots of record to the servo volatile, at the
    address its own descriptor gives, and read them back; returns (the
    slots, the field's bytes before). No SAVE."""
    data = slots_of_record(board)
    torque = osc.get("torque_enable")
    if not settings.same("off", torque):
        raise settings.Refused(f"torque_enable reads {torque}: the wide-code slots are written with torque off")
    found = osc.field(FIELD)
    if found is None:
        raise settings.Refused(f"the servo's descriptor names no {FIELD}: its image has no wide-code slots")
    addr, width, old = found
    if width != SLOTS:
        raise settings.Refused(f"{FIELD} is {width} B on this servo, the slots are {SLOTS}")
    osc.write(addr, data)
    back = osc.read(addr, SLOTS)
    if back != data:
        raise settings.Refused(f"{FIELD} reads back {back.hex()}, it was written {data.hex()}")
    return data, old


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.widecodes", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="sweep the bench servo's pot slowly, closed loop, as the ADC's ramp")
    c.add_argument("--settings", type=Path, help="a file of CONFIG fields the sweep runs on, applied volatile")
    c.add_argument("--label", help="the sweep's folder name, sweep-K by default")
    f = sub.add_parser("find", help="the ADC's wide codes from a recorded sweep, into the board's result")
    f.add_argument("--label", help="the sweep, the last complete one by default")
    sub.add_parser("install", help="write the board's slots to the servo volatile; no SAVE")
    a = ap.parse_args(argv)
    board = boards.BOARDS[boards.DEFAULT].key
    try:
        if a.cmd == "capture":
            out = capture(a.label, a.settings)
            print(f"{sc.shown(out)}: done; next `python -m oscnb.widecodes find --label {out.name}`")
        elif a.cmd == "find":
            folder = captures_dir(board) / a.label if a.label else latest(board)
            finding = find_capture(folder)
            print(show(finding))
            print(f"wrote {write_result(board, folder, finding)}")
        else:
            data, old = install(board)
            print(f"{FIELD}: {old.hex(' ')} -> {data.hex(' ')}, read back the same; volatile, not saved")
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
