"""The velocity test: the servo in Velocity mode holding a commanded
speed on its own estimate, so the pot's speed over the commanded one is
the estimate's level, read directly, with no position loop to correct it.

  uv run python -m oscnb.velocity_test

The captures are `osc-lab track --mode velocity` runs under
`telemetry/<pair>/velocity-test/r<round>/<image>/<speed>/`, one per
image, speed and round: a hold, then goals of one speed alternating in
sign, a half leg first and last and full legs between them, then a
goal of 0. Beside them lie the constant sets they ran on,
`set-<hash>.json`, and a capture is read through the one whose limits
of motion its config holds: the set's position table, from
the pair's table file, which must hold the set's points, the set's
wide-code slots ahead of it, and its counts per degree and per A.

Each full leg is read over its middle, MIDDLE of the leg's span, which
leaves out the acceleration at its start and the turn at its end. A
full leg is a goal of nonzero speed at least FULL_SHARE of the longest
segment. Over the middle: the pot's slope in deg/s, a least-squares
line, over the commanded speed (`ratio`); the ripple, the sd of the
slopes in windows of a MIDDLE_WINDOWS-th of the middle, no shorter
than WINDOW_MIN_S, over the leg's slope; the duty's reversals a second
(`scorecard.reversals`); and the current rms, the shunt over the
capture's own zero, the median raw reading at zero duty
(`scorecard.current_zero`), a row the shunt did not read counted as no
current (`scorecard.current_rms`). A cell, one image at one speed over
every round, is the median of its legs with their range as the bounds.
The result is written to `stages/5-closed-loop/velocity-test/results/
<pair>.json`, every leg and every cell.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from . import postable, results, scorecard as sc, widecodes

STAGE = "5-closed-loop"
EXPERIMENT = "velocity-test"
MIDDLE = (0.30, 0.85)   # the share of a leg's span its middle starts and ends at
FULL_SHARE = 0.9        # a full leg against the longest segment of its capture
MIDDLE_WINDOWS = 8      # the ripple's windows a middle holds
WINDOW_MIN_S = 0.02     # the shortest ripple window, several counts of travel at 6 deg/s
WINDOW_MIN_ROWS = 4     # a window with fewer rows reads no slope
KEYS = ("ratio", "ripple_pct", "reversals_per_s", "current_rms_ma")
UNITS = {"ratio": "", "ripple_pct": "%", "reversals_per_s": "1/s", "current_rms_ma": "mA"}


def folder(pair):
    return results.ROOT / "telemetry" / pair / EXPERIMENT


def set_of(config, sets):
    """The set among `sets` whose limits of motion the capture's config
    holds; refused when none or more than one does."""
    names = sc.constants().MOTION
    hit = [d for d in sets if all(d["fields"][n]["value"] == config[n] for n in names)]
    if len(hit) != 1:
        raise ValueError(f"{len(hit)} sets of the folder hold the config's "
                         f"{', '.join(f'{n} {config[n]}' for n in names)}")
    return hit[0]


def reading(doc):
    """(pot in deg from raw counts, counts per A) of a set: the slots,
    then the table, then its counts per degree."""
    cn = sc.constants()
    table = cn.live_table(doc, [doc["fields"][n]["value"] for n in cn.STOPS], quiet=True)
    slots = bytes(doc["wide_codes"]["slots"])
    cpd = doc["scale"]["counts_per_deg"]

    def deg(raw):
        return table.lin(widecodes.compensated(np.asarray(raw), slots)) / cpd
    return deg, doc["scales"][cn.CURRENT]["value"], cpd


def leg(t, x, duty, i_ma, commanded):
    """One leg's middle: `t` in s from the leg's first row, `x` the pot
    in deg, `duty` the applied duty, `i_ma` the read current (NaN where
    the shunt read nothing), `commanded` in deg/s."""
    t = np.asarray(t, float)
    a, b = t[0] + MIDDLE[0] * (t[-1] - t[0]), t[0] + MIDDLE[1] * (t[-1] - t[0])
    m = (t >= a) & (t <= b)
    tt, xx = t[m], np.asarray(x, float)[m]
    speed = np.polyfit(tt, xx, 1)[0]
    win = max(WINDOW_MIN_S, (b - a) / MIDDLE_WINDOWS)
    ws = []
    for e in np.arange(tt[0], tt[-1], win):
        w = (tt >= e) & (tt < e + win)
        if w.sum() >= WINDOW_MIN_ROWS:
            ws.append(np.polyfit(tt[w], xx[w], 1)[0])
    return {"commanded_deg_s": commanded, "speed_deg_s": speed, "ratio": speed / commanded,
            "ripple_pct": 100 * np.std(ws) / abs(speed) if len(ws) > 1 else np.nan,
            "reversals_per_s": sc.reversals(np.asarray(duty)[m]) / (b - a),
            "current_rms_ma": sc.current_rms(np.asarray(i_ma, float)[m])}


def capture_legs(rows, tick_hz, deg, cpd, counts_per_a):
    """Every full leg of a capture's rows, in segment order."""
    zero = sc.current_zero(rows)
    i_ma = np.where(rows["window_valid"] == 1, (rows["current_raw"] - zero) / counts_per_a * 1e3, np.nan)
    rows = rows.assign(x=deg(rows["pos"].to_numpy()), i_ma=i_ma)
    span = rows.groupby("seg")["tick"].max()
    out = []
    for k, g in rows.groupby("seg"):
        goal = g["goal"].iloc[0]
        if goal == 0 or span[k] < FULL_SHARE * span.max():
            continue
        out.append({"seg": int(k), **leg(g["tick"].to_numpy() / tick_hz, g["x"], g["duty_counts"], g["i_ma"],
                                         goal / cpd)})
    return out


def read(pair):
    """(every leg, the sets by hash) of the pair's velocity test."""
    root = folder(pair)
    cn = sc.constants()
    sets = [cn.load(p) for p in sorted(root.glob("set-*.json"))]
    readings = {d["hash"]["set"]: reading(d) for d in sets}
    out = []
    for cap in sorted(root.glob("r*/*/*/track.csv")):
        f = cap.parent
        meta = json.loads((f / "meta.json").read_text())
        doc = set_of(meta["config"], sets)
        deg, cpa, cpd = readings[doc["hash"]["set"]]
        rows = pd.read_csv(cap, usecols=["seg", "goal", "tick", "window_valid", "pos", "current_raw",
                                         "duty_counts"])
        for one in capture_legs(rows, meta["tick_hz"], deg, cpd, cpa):
            out.append({"round": f.parent.parent.name, "image": f.parent.name, "speed": f.name,
                        "set": doc["hash"]["set"][:12], **one})
    return out, {d["hash"]["set"][:12]: str(results.rel(p)) for d, p in
                 zip(sets, sorted(root.glob("set-*.json")))}


def cells(legs):
    """{image: {speed: {key: median and range, the commanded speed's
    size, `legs`: count}}}."""
    out = {}
    for (image, speed), g in pd.DataFrame(legs).groupby(["image", "speed"], sort=False):
        out.setdefault(image, {})[speed] = {
            "commanded_deg_s": g["commanded_deg_s"].abs().median(),
            **{k: results.median_range(g[k], UNITS[k]) for k in KEYS}, "legs": len(g)}
    return out


def table(cells_, file=None):
    """The cells, one line a speed and image."""
    images = list(cells_)
    speeds = sorted({s for c in cells_.values() for s in c})
    print(f"{'speed':>5}  {'image':8}  {'legs':>4}  {'ratio [range]':26}  {'ripple %':>8}  {'reversals/s':>11}  "
          f"{'current mA':>10}", file=file or sys.stdout)
    for s in speeds:
        for i in images:
            c = cells_[i].get(s)
            if c is None:
                continue
            r = c["ratio"]
            print(f"{s:>5}  {i:8}  {c['legs']:>4}  {r['value']:.3f} [{r['lo']:.3f}, {r['hi']:.3f}]{'':9}  "
                  f"{c['ripple_pct']['value']:>8.1f}  {c['reversals_per_s']['value']:>11.0f}  "
                  f"{c['current_rms_ma']['value']:>10.0f}", file=file or sys.stdout)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.velocity_test", description=__doc__.split("\n\n")[0])
    ap.add_argument("--pair", default=postable.default_pair(), help="servo__board, the default pair if omitted")
    a = ap.parse_args(argv)
    legs, sets = read(a.pair)
    by_cell = cells(legs)
    path = results.write(STAGE, EXPERIMENT, folder(a.pair), a.pair,
                         notes="Velocity mode, the pot's slope through the position table over the commanded "
                               "speed, per full leg over its middle; a cell is the median of its legs and their "
                               "range.",
                         sets=sets, cells=by_cell, legs=legs)
    table(by_cell)
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
