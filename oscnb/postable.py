"""The pot's position table in the servo's own format, from stage 4's lookup.

The format is protocol sec 5.7. 256 i16 calibration points c[k], point k at
raw 16 k over the 12-bit converter, and a 257th at 4096 fixed at 0 and never
written. A raw sample reads, in Q4 counts,

    lin = ((raw + c[i]) << 4) + (c[i + 1] - c[i]) * f,  i = raw >> 4, f = raw & 15

as a u16, so the all-zero table is the identity and point k lands at
raw + c[k]. A table is taken only with torque off, written 32 points a page
over 8 pages, and only when every point at or beyond a stop is zero (k <=
(raw_min + 15) >> 4 and k >= raw_max >> 4) and every interval's Q4 gain
16 + c[k + 1] - c[k] lies in 1..255, ends judged before shape.

Stage 4's lookup maps counts to the tag angle. The table maps raw counts to
counts linear in that angle, pinned so both stops map to themselves:

    L(theta) = raw_min + (theta - theta(raw_min)) * scale,
    scale = (raw_max - raw_min) / (theta(raw_max) - theta(raw_min)),

c[k] = L(theta(16 k)) - 16 k rounded to the count, zero at and beyond the
stops. Past the lookup's knots the angle runs on its end lines, which stage
4 carries as extrapolation.

The servo takes the board's ADC wide-code step in raw counts ahead of the
table (protocol sec 5.7, `oscnb.widecodes`): a pot sample stuck on a wide
code enters the table at the code plus its slot / 16. Stage 4 fits the
lookup on readings that carry that step, so the table holds the pot alone,
and every grade here reads the pot the same way: the sitting's frames
loaded with the step on, and a raw code read through the table at the
compensated raw (`Table.lin` of `widecodes.compensated`).

    uv run python -m oscnb.postable build --stops RAW_MIN RAW_MAX
    uv run python -m oscnb.postable check
    uv run python -m oscnb.postable plan
    uv run python -m oscnb.postable install

`build` writes stages/5-closed-loop/calibration/<pair>.pos_lut.json, `check`
grades it offline, `plan` prints the `osc` commands an install takes and
runs none of them. `install` runs the plan's steps 1 to 4 on the servo,
each read checked and every page read back, and stops short of the stamp:
the table is volatile until a SAVE, which nothing here sends.
"""
import argparse
import hashlib
import json
import re
import time
from dataclasses import dataclass

import numpy as np
import pandas as pd

from . import boards, osc, results, servos, settings, widecodes
from . import position as pe

ADC_BITS = 12
GRID_SHIFT = 4
GRID = 1 << GRID_SHIFT                  # raw counts per interval
INTERVALS = 1 << (ADC_BITS - GRID_SHIFT)  # host-written points
PAGE_POINTS = 32
PAGES = INTERVALS // PAGE_POINTS
GAIN_MAX = 16                           # local gain under 16x nominal
# the servo's clamp on the hold band's gain, Q4: half the deadband in shaft
# angle at the least, four deadbands at the most
BAND_GAIN_MIN_Q4, BAND_GAIN_MAX_Q4 = GRID // 2, 4 * GRID
ADC_MASK = (1 << ADC_BITS) - 1
STATES = ("IDENTITY", "LOADING", "LIVE", "REJECT_TORQUE", "REJECT_ENDS",
          "REJECT_SHAPE")
STORE, FETCH, COMMIT = 1, 2, 3          # pos_lut_cmd
# the window, page then cmd then points then state, protocol sec 5.7
WINDOW_ADDR, POINTS_ADDR = 0x19C, 0x19E
COMMIT_WAIT_S = 0.05    # s between reads of a COMMIT still LOADING
COMMIT_TRIES = 20       # reads before giving up on its verdict

STAGE4 = ("4-estimators", "position")   # the lookup's experiment
OUT_DIR = results.ROOT / "stages" / "5-closed-loop" / "calibration"
MOVE_DEG_S = 2.0        # deg/s, stage 4's scored frame moves at least this
BOX_S = 0.002           # s, the pot's centered mean at a frame
SPEED_FRAMES = 5        # frames either side, a frame's tag speed


def default_pair():
    return f"{servos.DEFAULT}__{boards.DEFAULT}"


def interp_q4(raw, points):
    """The servo's reading of `raw` through `points` (256), Q4 counts, as
    the u16 the format defines. Integers, scalar or array."""
    c = np.r_[np.asarray(points, np.int64), 0]
    raw = np.asarray(raw, np.int64) & ADC_MASK
    i = raw >> GRID_SHIFT
    f = raw & (GRID - 1)
    return (((raw + c[i]) << GRID_SHIFT) + (c[i + 1] - c[i]) * f) & 0xFFFF


def validate(points, raw_min, raw_max):
    """None for a table the servo takes against these stops, else the
    state it lands in and why."""
    k = np.asarray(points, np.int64)
    if len(k) != INTERVALS:
        raise ValueError(f"{len(k)} points, the format holds {INTERVALS}")
    lo, hi = (raw_min + GRID - 1) >> GRID_SHIFT, raw_max >> GRID_SHIFT
    idx = np.arange(INTERVALS)
    bad = np.flatnonzero(((idx <= lo) | (idx >= hi)) & (k != 0))
    if len(bad):
        return (f"REJECT_ENDS: points {', '.join(map(str, bad[:8]))}"
                f"{' ...' if len(bad) > 8 else ''} nonzero at or beyond a stop"
                f" (stops {raw_min} and {raw_max}: points <= {lo} and >= {hi}"
                " must be zero)")
    d = GRID + np.diff(np.r_[k, 0])
    bad = np.flatnonzero((d < 1) | (d >= GAIN_MAX * GRID))
    if len(bad):
        return (f"REJECT_SHAPE: intervals {', '.join(map(str, bad[:8]))}"
                f"{' ...' if len(bad) > 8 else ''} have Q4 gain "
                f"{', '.join(map(str, d[bad[:8]]))}, outside 1..255")
    return None


@dataclass
class Table:
    """A position table and the line it linearizes to: the stops and the
    tag angle at each, deg."""
    points: np.ndarray
    raw_min: int
    raw_max: int
    theta_min: float
    theta_max: float

    @property
    def scale(self):
        """Linearized counts per deg."""
        return (self.raw_max - self.raw_min) / (self.theta_max - self.theta_min)

    def lin(self, raw):
        """Linearized counts at `raw`, float: the format's line between the
        points, so at an integer raw it is interp_q4 / 16."""
        c = np.r_[self.points, 0].astype(float)
        r = np.clip(np.asarray(raw, float), 0, ADC_MASK)
        i = np.minimum((r // GRID).astype(int), INTERVALS - 1)
        return r + c[i] + (c[i + 1] - c[i]) * (r - GRID * i) / GRID

    def band_gain(self, raw):
        """Linearized counts per raw count of the interval raw code `raw`
        falls in, clamped as the servo clamps it for the hold band: the
        servo's band at `raw` is its deadband times this."""
        c = np.r_[self.points, 0].astype(np.int64)
        i = (np.asarray(raw, np.int64) & ADC_MASK) >> GRID_SHIFT
        q4 = np.minimum(np.maximum(GRID + c[i + 1] - c[i], BAND_GAIN_MIN_Q4), BAND_GAIN_MAX_Q4)
        return q4 / GRID

    def angle(self, raw):
        """deg at pot counts `raw` through the table."""
        return self.theta_min + (self.lin(raw) - self.raw_min) / self.scale

    def counts(self, theta):
        """Pot counts the table reads as `theta` deg."""
        r = np.arange(ADC_MASK + 1, dtype=float)
        target = self.raw_min + (np.asarray(theta, float) - self.theta_min) * self.scale
        return np.interp(target, self.lin(r), r)

    def errors(self, pos, theta):
        """(deg, counts) as `position.errors` reads them."""
        return (self.angle(pos) - np.asarray(theta, float),
                np.asarray(pos, float) - self.counts(theta))


def from_lookup(lk, raw_min, raw_max):
    """The Table linearizing lookup `lk` between the stops; raises where
    the servo would refuse it."""
    if not 0 <= raw_min < raw_max <= ADC_MASK:
        raise ValueError(f"stops {raw_min} and {raw_max}: want 0 <= raw_min < raw_max <= {ADC_MASK}")
    th0, th1 = (float(v) for v in lk.angle(np.array([raw_min, raw_max], float)))
    t = Table(np.zeros(INTERVALS, np.int64), raw_min, raw_max, th0, th1)
    raw = GRID * np.arange(INTERVALS, dtype=float)
    c = np.rint(raw_min + (lk.angle(raw) - th0) * t.scale - raw).astype(np.int64)
    lo, hi = (raw_min + GRID - 1) >> GRID_SHIFT, raw_max >> GRID_SHIFT
    c[:lo + 1] = 0
    c[hi:] = 0
    t.points = c
    why = validate(c, raw_min, raw_max)
    if why:
        raise ValueError(f"the servo would refuse this table: {why}")
    return t


def lookup_of_record(pair):
    """(Lookup, its csv path, stage 4's result) of a pair, the csv checked
    against the result's lookup keys and end lines."""
    res = results.read(*STAGE4, pair=pair)
    path = results.path(*STAGE4, pair=pair).with_suffix(".lookup.csv")
    t = pd.read_csv(path)
    c, a = t["counts"].to_numpy(float), t["angle_deg"].to_numpy(float)
    pitch = res["lookup_pitch_counts"]["value"]
    checks = (("lookup_pitch_counts", np.diff(c).max(), np.diff(c).min()),
              ("lookup_knot_first_counts", c[0], c[0]),
              ("lookup_knot_last_counts", c[-1], c[-1]),
              ("lookup_entries_guard_to_guard", len(c), len(c)))
    for key, x, y in checks:
        if not (x == y == res[key]["value"]):
            raise ValueError(f"{results.rel(path)} disagrees with {key}")
    lo, hi = (res[f"end_line_{e}_counts_per_deg"]["value"] for e in ("low", "high"))
    lk = pe.Lookup("counts", c, a, lo, hi)
    for e, want, cell in (("low", lo, (c[1] - c[0]) / (a[1] - a[0])),
                          ("high", hi, (c[-1] - c[-2]) / (a[-1] - a[-2]))):
        if abs(cell - want) > 0.01:
            raise ValueError(f"the lookup's {e} cell runs {cell:.3f} counts/deg,"
                             f" its end line {want:.3f}")
    if not lk.monotone or pitch % GRID:
        raise ValueError(f"lookup not monotone, or its pitch {pitch:g} off the"
                         f" format's grid of {GRID}")
    return lk, path, res


def out_path(pair):
    return OUT_DIR / f"{pair}.pos_lut.json"


def slots_sha1(slots):
    """The sha1 a table file names 16 wide-code slots by."""
    return hashlib.sha1(bytes(slots)).hexdigest()


def fitted_on(res):
    """The sha1 of the wide-code slots stage 4's lookup was fitted on
    (`slots_sha1`), from the wide-code result its result `res` names
    among the results it read; None when it names none, or was written
    before results named what they read. Refused when that file is no
    longer the one the fit read: its slots then are not known."""
    folder = results.rel(results.path(widecodes.STAGE, widecodes.EXPERIMENT, "x").parent).as_posix()
    read = res.get("inputs") or {}
    named = [name for name in read if name.rsplit("/", 1)[0] == folder]
    if not named:
        return None
    if len(named) > 1:
        raise ValueError(f"stage 4's result read more than one wide-code result: {', '.join(named)}")
    path = results.ROOT / named[0]
    now = hashlib.sha1(path.read_bytes()).hexdigest() if path.exists() else "gone"
    if now != read[named[0]]:
        raise ValueError(f"{named[0]} is no longer the wide-code result the lookup was fitted on (sha1 "
                         f"{read[named[0]]} then, {now} now): re-run stage 4 position, then `postable build`")
    return slots_sha1(widecodes.slots_of_file(path))


def fitted_on_said(doc):
    """What a table file's document says of the slots its lookup was
    fitted on, in words: their sha1, or that it records none."""
    sha = doc["inputs"].get("fitted_on_slots_sha1")
    return (f"lookup fitted on wide-code slots of sha1 {sha}" if sha
            else "the wide-code slots its lookup was fitted on are not recorded")


def build(pair, raw_min, raw_max):
    """Write the pair's table against stops `raw_min`, `raw_max`; returns
    (Table, path)."""
    lk, src, res = lookup_of_record(pair)
    t = from_lookup(lk, raw_min, raw_max)
    d = GRID + np.diff(np.r_[t.points, 0])
    inside = slice((raw_min + GRID - 1) >> GRID_SHIFT, raw_max >> GRID_SHIFT)
    quoted = ("table_term_heldout_deg_rms", "table_term_heldout_counts_rms",
              "heldout_lead_applied_deg_rms")
    doc = {
        "format": "protocol sec 5.7: 256 i16 calibration points, point k at "
                  "raw 16 k, point 256 fixed 0 and not written; 8 pages of 32",
        "inputs": {
            "lookup": str(results.rel(src)),
            "result": str(results.rel(results.path(*STAGE4, pair=pair))),
            "lookup_pitch_counts": res["lookup_pitch_counts"]["value"],
            "end_lines_counts_per_deg": [lk.lo_slope, lk.hi_slope],
            "fitted_on_slots_sha1": fitted_on(res),
            "stops": {"raw_min": raw_min, "raw_max": raw_max,
                      "from": "the command line; the install plan reads "
                              "them off the servo first"},
        },
        "target": {
            "theta_at_stops_deg": [t.theta_min, t.theta_max],
            "counts_per_deg": t.scale,
            "line": "L(theta) = raw_min + (theta - theta(raw_min)) * "
                    "counts_per_deg, both stops map to themselves",
        },
        "extrapolated_counts": [[raw_min, float(lk.c[0])],
                                [float(lk.c[-1]), raw_max]],
        "local_gain": [float(d[inside].min()) / GRID, float(d[inside].max()) / GRID],
        "stage4_quoted": {k: res[k]["value"] for k in quoted},
        "points_by_page": [[int(x) for x in t.points[p * PAGE_POINTS:(p + 1) * PAGE_POINTS]]
                           for p in range(PAGES)],
    }
    text = re.sub(r"\[([-\d.,\s]+)\]", lambda m: "[" + " ".join(m.group(1).split()) + "]",
                  json.dumps(doc, indent=1))
    path = out_path(pair)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text + "\n")
    return t, path


def load_file(path):
    """(Table, its document) of a table file, refused where the servo would
    refuse it."""
    doc = json.loads(path.read_text())
    s = doc["inputs"]["stops"]
    th = doc["target"]["theta_at_stops_deg"]
    points = np.concatenate(doc["points_by_page"]).astype(np.int64)
    t = Table(points, s["raw_min"], s["raw_max"], *th)
    why = validate(t.points, t.raw_min, t.raw_max)
    if why:
        raise ValueError(f"{results.rel(path)}: the servo would refuse it: {why}")
    return t, doc


def load(pair):
    """(Table, its document) of a pair's built table."""
    return load_file(out_path(pair))


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def format_cost(t, lk, slots):
    """How far the table reads from the lookup it came from, every raw
    code read as the servo reads it, the board's wide-code `slots` ahead
    of both: (deg rms, deg max, counts rms, counts max) inside the lookup's
    knots, then the same from stop to stop."""
    out = []
    for a, b in ((lk.c[0], lk.c[-1]), (t.raw_min, t.raw_max)):
        r = widecodes.compensated(np.arange(int(np.ceil(a)), int(b) + 1), slots)
        e = t.angle(r) - lk.angle(r)
        out += [rms(e), float(np.abs(e).max()), rms(e * t.scale),
                float(np.abs(e * t.scale).max())]
    return out


def tag_speed(host, deg):
    """deg/s per frame: a line through SPEED_FRAMES frames either side."""
    n = len(host)
    v = np.full(n, np.nan)
    for i in range(n):
        a, b = max(0, i - SPEED_FRAMES), min(n, i + SPEED_FRAMES + 1)
        if b - a >= 4:
            v[i] = np.polyfit(host[a:b] - host[i], deg[a:b], 1)[0]
    return v


def sitting(pair):
    """(frames, slots) of a pair's camera sitting the way stage 4 loads it:
    every chain's pot carrying the board's wide-code step, and the 16
    slots of record its meta names."""
    from .potframes import load as load_frames
    pf = load_frames(results.ROOT / "telemetry" / pair, wide_codes=True)
    return pf, widecodes.slots_of_meta(next(iter(pf.sweeps.values()))[1])


def stage4_sets(pf):
    """(moving, rest, scored) of stage 3's frames the way stage 4 split and
    scored them: a fold per chain by the parity of its rank in its clip
    kind; scored = the frames inside the moving span with tag speed over
    MOVE_DEG_S, the pot re-read on the camera's clock (pos_cam). `pf` is
    `sitting`'s, its pot carrying the board's wide-code step; frames loaded
    on raw codes are refused."""
    from .potframes import kind_of
    fold, seen = {}, {}
    for clip, k in zip(pf.chains["clip"], pf.chains["k"]):
        kind = kind_of(clip)
        fold[(clip, k)] = seen.get(kind, 0) % 2
        seen[kind] = seen.get(kind, 0) + 1

    def folded(t):
        return t.assign(fold=[fold[x] for x in zip(t["clip"], t["k"])])

    fc = folded(pf.frames)
    fc["sid"] = fc["clip"] + "/" + fc["k"].astype(str) + "/" + fc["seg"].astype(str)
    fc["pos_cam"] = np.nan
    fc["tag_speed"] = np.nan
    for (clip, k), g in fc.groupby(["clip", "k"]):
        rows = pf.rows(clip, k)[0]
        if "pos_raw" not in rows:
            raise ValueError(f"{clip}/{k}: its rows carry the raw pot code, the table is "
                             "graded on readings with the board's wide-code step "
                             "(potframes.load with wide_codes)")
        dt = float(np.median(np.diff(rows["host"].to_numpy()[:2000])))
        w = max(1, int(round(BOX_S / dt)))
        for seg, gs in g.groupby("seg"):
            rs = rows[rows["seg"] == seg]
            box = rs["pos"].rolling(w, center=True, min_periods=1).mean()
            cam = rs["host_cam"].to_numpy()
            if np.isfinite(cam).all():
                fc.loc[gs.index, "pos_cam"] = np.interp(gs["host"], cam, box.to_numpy())
            fc.loc[gs.index, "tag_speed"] = tag_speed(gs["host"].to_numpy(),
                                                      gs["image_deg"].to_numpy())
    inside = (fc["image_deg"] > pf.lo_deg) & (fc["image_deg"] < pf.hi_deg)
    scored = fc[(fc["tag_speed"].abs() > MOVE_DEG_S) & inside & fc["pos_cam"].notna()]
    return folded(pf.moving), folded(pf.rest), scored


def heldout(moving, rest, scored, raw_min, raw_max, pitch):
    """Stage 4's two-fold grade through the lookup and through the table
    built from it: per fold the lookup is fitted on the other fold, its
    table built against the stops, and the fold's scored frames inside the
    knots read through both. The sets are `stage4_sets`', their pot
    carrying the board's wide-code step, so the table is read at the
    compensated raw. Returns {source: (deg rms, counts rms) with
    each segment's offset out, then (deg rms, counts rms) with nothing
    out}, pooled over both folds."""
    errs = {"lookup": [], "table": []}
    for q in (0, 1):
        lk, _ = pe.build(moving[moving["fold"] != q], rest[rest["fold"] != q], pitch)
        t = from_lookup(lk, raw_min, raw_max)
        s = scored[scored["fold"] == q]
        s = s[s["pos_cam"].between(lk.c[0], lk.c[-1]) & s["pos"].between(lk.c[0], lk.c[-1])]
        for name, src in (("lookup", lk), ("table", t)):
            ed, ec = (pe.errors(src, s["pos_cam"], s["image_deg"]) if name == "lookup"
                      else src.errors(s["pos_cam"], s["image_deg"]))
            errs[name].append(pd.DataFrame({"sid": s["sid"].to_numpy(),
                                            "deg": ed, "counts": ec}))
    out = {}
    for name, parts in errs.items():
        e = pd.concat(parts, ignore_index=True)
        seg = e.groupby("sid")[["deg", "counts"]].transform("mean")
        out[name] = (rms(e["deg"] - seg["deg"]), rms(e["counts"] - seg["counts"]),
                     rms(e["deg"]), rms(e["counts"]))
    return out


def check(pair):
    """Print the table's verdict, its format cost against the lookup of
    record and stage 4's held-out grade read through it, every reading
    with the board's wide-code step."""
    t, doc = load(pair)
    lk, _, res = lookup_of_record(pair)
    print(f"{results.rel(out_path(pair))}: LIVE against stops {t.raw_min} "
          f"and {t.raw_max}; {t.scale:.3f} linearized counts/deg, local gain "
          f"{doc['local_gain'][0]:.2f} to {doc['local_gain'][1]:.2f}; {fitted_on_said(doc)}")
    pf, slots = sitting(pair)
    fc = format_cost(t, lk, slots)
    print(f"against the lookup of record, every raw code inside its knots, "
          "the board's wide-code step ahead of both: "
          f"{fc[0]:.4f} deg rms, {fc[1]:.4f} max ({fc[2]:.3f} / {fc[3]:.3f} "
          f"counts); stop to stop {fc[4]:.4f} deg rms, {fc[5]:.4f} max")
    for (a, b), (da, db) in zip(doc["extrapolated_counts"],
                                ((t.theta_min, lk.a[0]), (lk.a[-1], t.theta_max))):
        print(f"on an end line, no knot: {a:g} to {b:g} counts, {db - da:.2f} deg")
    moving, rest, scored = stage4_sets(pf)
    g = heldout(moving, rest, scored, t.raw_min, t.raw_max,
                res["lookup_pitch_counts"]["value"])
    q = {k: res[k]["value"] for k in ("table_term_heldout_deg_rms",
                                      "table_term_heldout_counts_rms",
                                      "heldout_lead_applied_deg_rms")}
    print(f"held out, two folds, {len(scored)} scored frames on readings "
          "with the board's wide-code step, each fold's table built against "
          "the same stops:")
    print(f"  stage 4 quotes {q['table_term_heldout_deg_rms']:.4f} deg / "
          f"{q['table_term_heldout_counts_rms']:.3f} counts, segment offsets "
          f"out; {q['heldout_lead_applied_deg_rms']:.4f} deg nothing out")
    for name in ("lookup", "table"):
        a = g[name]
        print(f"  through the {name:6s} {a[0]:.4f} deg / {a[1]:.3f} counts, "
              f"segment offsets out; {a[2]:.4f} deg / {a[3]:.3f} counts "
              "nothing out")
    return g


def page_bytes(points, page):
    k = np.asarray(points[page * PAGE_POINTS:(page + 1) * PAGE_POINTS], "<i2")
    return k.tobytes()


@dataclass(frozen=True)
class Step:
    """One wire step of an install. `get` reads `field` and, with `want`
    set, refuses any other value; `write` puts `data` at the window;
    `poll` reads `field` while it is LOADING and refuses anything but
    `want` after; `fetch` brings `page` back into the window and compares
    its points with `data`."""
    op: str
    field: str = ""
    want: object = None
    data: bytes = b""
    page: int = 0
    note: str = ""


def install_steps(t):
    """The install's steps 1 to 4 for table `t`, section headings as
    strings between them: what `plan` prints and `install` runs."""
    states = ", ".join(f"{i} {s}" for i, s in enumerate(STATES))
    out = [
        "1. preconditions, each read before anything is written",
        Step("get", "torque_enable", "off", note="off; STORE and COMMIT are refused under torque"),
        Step("get", "raw_min", t.raw_min, note=f"{t.raw_min}, the stop the table was built against; else rebuild"),
        Step("get", "raw_max", t.raw_max, note=f"{t.raw_max}"),
        Step("get", "pos_lut_page", note=f"addr 0x{WINDOW_ADDR:04x}: page, cmd, points, state in a row"),
        Step("get", "pos_lut_state", note=f"the state before: {states}"),
        "2. STORE, one 66 B write a page: page, cmd 1, 32 points i16 LE",
    ]
    out += [Step("write", data=bytes([p, STORE]) + page_bytes(t.points, p)) for p in range(PAGES)]
    out += [
        Step("get", "pos_lut_state", "LOADING", note="1 LOADING"),
        "3. COMMIT: page 0 and cmd 3, the write covers the command",
        Step("write", data=bytes([0, COMMIT])),
        Step("poll", "pos_lut_state", "LIVE", note="again while 1 LOADING; 2 LIVE goes on, any REJECT stops here"),
        "4. read back: FETCH a page into the window, read its 64 B, compare with its STORE above",
    ]
    out += [Step("fetch", data=page_bytes(t.points, p), page=p) for p in range(PAGES)]
    return out


def shown_step(s, o):
    if isinstance(s, str):
        return f"# {s}"
    if s.op in ("get", "poll"):
        return f"{o} get {s.field:<16} # {s.note}"
    if s.op == "write":
        return f"{o} write 0x{WINDOW_ADDR:x} {s.data.hex()}"
    return (f"{o} write 0x{WINDOW_ADDR:x} {bytes([s.page, FETCH]).hex()} && "
            f"{o} read 0x{POINTS_ADDR:x} {len(s.data)}")


def plan(pair):
    """The install's wire steps as `osc` commands, one string; runs none."""
    t, doc = load(pair)
    o = f"osc -i {osc.ID}"
    lines = [f"# install {results.rel(out_path(pair))} (protocol sec 5.7); nothing here has run"]
    lines += [shown_step(s, o) for s in install_steps(t)]
    lines += [
        "# 5. stamp the live set, the LIVE points in it; no --save",
        f"{o} stamp                # 'data state clean: closed loop allowed'",
        f"{o} status               # 'data  clean' and 'stamp 0x.... = live set'",
        "# 6. no SAVE: the table and the stamp stay volatile, and osc reboot returns",
        "#    the saved identity table and the saved stamp. A SAVE (osc save, or",
        "#    osc stamp --save) would persist them, and with them every volatile",
        "#    CONFIG write standing at the time (oscnb.settings apply).",
    ]
    return "\n".join(lines)


def state_name(v):
    """A `pos_lut_state` read as its name; refused when it is none."""
    if isinstance(v, int) and 0 <= v < len(STATES):
        return STATES[v]
    if v in STATES:
        return v
    raise settings.Refused(f"pos_lut_state reads {v}, not a state of protocol sec 5.7")


def got(s):
    v = osc.get(s.field)
    return state_name(v) if s.field == "pos_lut_state" else v


def fetch(page, n=2 * PAGE_POINTS):
    """The first `n` bytes of page `page` of the array the servo holds:
    a FETCH brings the page into the window, which is read."""
    osc.write(WINDOW_ADDR, bytes([page, FETCH]))
    return osc.read(POINTS_ADDR, n)


def on_servo():
    """(state, points) as the servo holds them: `pos_lut_state` by name
    and every page fetched, the 256 points i16 LE in a row. A FETCH is
    never gated and moves neither the state nor the stamp (protocol sec
    5.7)."""
    return state_name(osc.get("pos_lut_state")), b"".join(fetch(p) for p in range(PAGES))


def install(pair):
    """Run `plan`'s steps 1 to 4 on the servo: the preconditions, STORE of
    every page, COMMIT, LIVE, every page read back the same. No stamp and
    no SAVE; returns the Table. A refusal past the first STORE leaves the
    array loading: `osc reboot` returns the saved table."""
    return install_table(load(pair)[0])


def install_table(t):
    """`install` for a Table at hand: its steps 1 to 4 run on the servo;
    returns the Table."""
    for s in install_steps(t):
        if isinstance(s, str):
            continue
        if s.op == "get":
            v = got(s)
            if s.want is not None and not settings.same(s.want, v):
                raise settings.Refused(f"{s.field} reads {v}, the install wants {s.want}")
        elif s.op == "write":
            osc.write(WINDOW_ADDR, s.data)
        elif s.op == "poll":
            for _ in range(COMMIT_TRIES):
                v = got(s)
                if v != "LOADING":
                    break
                time.sleep(COMMIT_WAIT_S)
            else:
                raise settings.Refused(f"{s.field} still LOADING {COMMIT_TRIES * COMMIT_WAIT_S:g} s "
                                       "after the COMMIT")
            if v != s.want:
                raise settings.Refused(f"the COMMIT landed {v}, not {s.want}")
        else:
            back = fetch(s.page, len(s.data))
            if back != s.data:
                raise settings.Refused(f"page {s.page} reads back {back.hex()}, it was stored as {s.data.hex()}")
    return t


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.postable",
                                 description="the pot's position table in the servo's format, offline")
    ap.add_argument("--pair", default=default_pair(), help="servo__board, the default pair if omitted")
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="the table from stage 4's lookup, against the servo's stops")
    b.add_argument("--stops", type=int, nargs=2, required=True, metavar=("RAW_MIN", "RAW_MAX"),
                   help="the servo's raw_min and raw_max")
    sub.add_parser("check", help="the verdict and the held-out grade through the table")
    sub.add_parser("plan", help="print the install's osc commands without running them")
    sub.add_parser("install", help="run the plan's steps 1 to 4 on the servo; no stamp, no SAVE")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "build":
            t, path = build(a.pair, *a.stops)
            print(f"wrote {results.rel(path)}: stops {t.raw_min} and {t.raw_max}, "
                  f"{int(np.count_nonzero(t.points))} nonzero points, "
                  f"{t.scale:.3f} counts/deg; {fitted_on_said(load_file(path)[1])}")
        elif a.cmd == "check":
            check(a.pair)
        elif a.cmd == "install":
            install(a.pair)
            print(f"{results.rel(out_path(a.pair))}: LIVE, {PAGES} pages read back the same; not "
                  "stamped and not saved (osc stamp blesses it, osc reboot drops it)")
        else:
            print(plan(a.pair))
    except (ValueError, settings.Refused) as e:
        raise SystemExit(f"refused: {e}")


if __name__ == "__main__":
    main()
