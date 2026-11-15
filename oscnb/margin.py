"""Loop margins from a margin sweep: in Velocity mode at a cruise the
velocity loop's reference carries a triangle, and each TEL row the
observer's position, the loop's speed and the summed reference. From
those three, the open-loop gain of the speed loop and of the position
loop per frequency, and each loop's gain and phase margin.

  uv run python -m oscnb.margin <sweep folder> [<second sweep folder>]

A sweep is one folder, `telemetry/<pair>/margin/capture-N/`, and a burst
a pair of files in it, numbered from 0 in capture order:

- `burst-K.csv`, one row a TEL row: `tick` (the row's `sample_tick`, low
  16 bits), then the TELEMETRY fields `theta_hat_q16` (cQ16) and
  `omega_hat_cps` and the TELEMETRY-EXT field `omega_ref_cps` (both raw
  csQ16), as the row carries them (protocol sec 5.6). A row is a FAST
  tick and the three hold one medium period's values for PERIOD_TICKS
  rows.
- `burst-K.json`, the burst's facts: the CONTROL fields
  `inject_half_periods` (N, medium periods) and `inject_accel_q88` (the
  slope, Q8.8 c/s a period) as written, `cruise_cps` (the goal's size,
  c/s) and `direction` (+1 or -1, the goal's sign), the CONFIG field
  `p_kp_q88`, and the row stream's losses over the burst, `rows_dropped`
  (the servo's `tel_drop_count`) and `seq_holes` (frames missing from
  `stream_seq`). `reading_cps`, where present, is the cruise of the
  reading the burst belongs to: its own cruise, or the one a two-cruise
  check checks. Other keys are the tool's and are not read here.

The method. A burst's rows become periods, one row a period at a whole
number of periods from its first row once the tick is unwrapped. The
first HEAD_S is dropped, the rest cut into cycles of the triangle, 2N
periods each, and a cycle is dropped where a step of the reference from
one period to the next is not the slope either way (a pause, the clamp)
or a row is missing. Each cycle kept gives the bin of the triangle's
frequency of the reference U and the speed W, both in c/s, and of the
position TH in counts, from its period difference over `1 - e^(-jw)`
since the cruise's ramp does not vanish over whole cycles. Over the
cycles of the bursts of one N at one cruise, by cross spectra a cycle:

  L_v = sum W U* / sum (U - W) U*     L_p = kp sum TH U* / sum |U|^2

with kp = `p_kp_q88` / 256 c/s per count. A pause shifts the triangle
against the cycle grid, so the cycles of one burst differ in phase: each
cycle's spectra carry their own phase into their cross spectrum, where
summed over the burst first they would partly cancel. Negative feedback,
the closed loop L / (1 + L), phase in (-270, 90].

A point's noise is a delete-one-block jackknife over its cycles, the
bursts' in capture order cut into BLOCKS blocks of consecutive cycles
(each cycle its own block where there are fewer), its standard error in
dB and in degrees. A point is unreadable where that is over SE_DB_MAX or
SE_DEG_MAX, or the measured response a cycle (W / U, kp TH / U) is under
RESPONSE_SE_MIN of its own standard errors from zero, or fewer than two
cycles were kept: it stays in the table, marked, and out of the margins
and the repeats.

A point read at a second cruise is checked against the first, and one
that differs by more than APART_SIGMA of the pair's combined standard
error (PAIR_FACTOR times the two jackknifes', each at least SE_FLOOR_DB
and SE_FLOOR_DEG) in dB or deg is flagged and left out of the margins:
an interference fixed in position reads at a frequency that moves with
the cruise. The phase margin is 180 + arg L
where |L| crosses 0 dB, the gain margin -|L| in dB where arg L crosses
-180, both interpolated between the two readable points that straddle it
in log frequency. A loop whose phase reaches -180 only at Nyquist, where
L is real, reads its gain margin there as -20 log10(-Re L) (the speed
loop), or, where its gain there is too far down to read (the position
loop), is reported as over the smallest -|L| of the points above its
crossover whose phase is under BOUND_DEG. A crossing that may sit next
to an unreadable point is not read: a 0 dB or a -180 crossing between
two readable points that straddle it with an unreadable one between
them, or a -180 crossing above the last readable point where it is read
at Nyquist or no bound is found.

A sweep holds one reading a `reading_cps` (one in all where the facts
carry none): the speed band run at a slow and a fast cruise is two
readings, each read on its own, and the position band at the second
cruise is a check of the first. Two sweeps of one servo repeat where
every point of a loop's band (the bursts' `band` facts) that both read
and neither flags differs by at most APART_SIGMA of the pair's combined
standard error in dB and in deg, the phase margins by at most
REPEAT_PM_DEG, and the gain margins by at most REPEAT_DB or both read at
least twice GM_FLOOR_DB: the gain margin decides a GM_FLOOR_DB floor,
and at twice the floor two readings' scatter cannot change that verdict.
A margin only one of the two read is reported and not judged.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

TICK_HZ = 20_000
PERIOD_TICKS = 10           # FAST ticks a medium period
PERIOD_HZ = TICK_HZ / PERIOD_TICKS
TICK_WRAP = 1 << 16
Q16 = 1 << 16
HEAD_S = 0.2                # the loop settling on the stimulus at a burst's start
HALF_PERIODS_MAX = 512      # the firmware reads `inject_half_periods` at most this
BLOCKS = 16                 # jackknife blocks a point, of consecutive cycles: a wander slower than a cycle stays in one
SE_DB_MAX = 1.5             # a point noisier than this cannot place a crossing between its neighbors
SE_DEG_MAX = 10.0
RESPONSE_SE_MIN = 3.0       # a response under 3 of its standard errors is not told from none
# the jackknife's se under the scatter of two sweeps of one setting: the rms of their
# difference over the combined se, 142 differences of the mg90-d margin captures 1 to 14
PAIR_FACTOR = 1.42
APART_SIGMA = 3.0           # two readings of a point apart beyond chance: 0.3 % of normal differences
# a jackknife sees only what varies cycle to cycle, never a reading's fixed errors: a
# point read finer is compared at this
SE_FLOOR_DB = 0.05
SE_FLOOR_DEG = 0.5
BOUND_DEG = -170.0
REPEAT_DB = 1.0             # two sweeps' gain margins
REPEAT_PM_DEG = 5.0         # the phase margins
GM_FLOOR_DB = 6.0           # the gain margin a loop is accepted on
COLUMNS = ("tick", "theta_hat_q16", "omega_hat_cps", "omega_ref_cps")
FACTS = ("inject_half_periods", "inject_accel_q88", "cruise_cps", "direction", "p_kp_q88", "rows_dropped",
         "seq_holes")
LOOPS = {"speed": ("L_v", "read"), "position": ("L_p", "bound")}


def unwrap(tick):
    """The 16-bit tick counter as one rising count from the first row."""
    t = np.asarray(tick, np.int64)
    step = np.diff(t) % TICK_WRAP
    if (step == 0).any():
        raise ValueError("a row repeats the tick of the row before it")
    return t[0] + np.concatenate(([0], np.cumsum(step)))


def half_periods(facts):
    """N as the firmware reads it."""
    n = min(int(facts["inject_half_periods"]), HALF_PERIODS_MAX)
    if n < 1 or int(facts["inject_accel_q88"]) < 1:
        raise ValueError("the burst's facts hold no triangle")
    return n


def periods(rows):
    """(complete, theta, omega_hat, omega_ref) a period from the burst's
    first row, raw: a period's values are its row a whole number of
    periods from the first, and it is complete where all its rows
    arrived. A last period the burst's end cut short is left out."""
    t = unwrap(rows["tick"])
    t = t - t[0]
    slot = t // PERIOD_TICKS
    n = int(slot[-1]) + int(t[-1] % PERIOD_TICKS == PERIOD_TICKS - 1)
    keep = slot < n
    complete = np.bincount(slot[keep], minlength=n) == PERIOD_TICKS
    at = keep & (t % PERIOD_TICKS == 0)
    out = [complete]
    for c in COLUMNS[1:]:
        v = np.zeros(n, np.int64)
        v[slot[at]] = rows[c].to_numpy(np.int64)[at]
        out.append(v)
    return tuple(out)


def spectra(rows, facts):
    """One burst a whole cycle kept: U, W and TH at the triangle's
    frequency, one entry a cycle, kp, and the cycles kept and dropped."""
    n = half_periods(facts)
    size = 2 * n
    step = int(facts["inject_accel_q88"]) << 8
    complete, theta, omega, ref = periods(rows)
    whole = complete & np.concatenate(([False], complete[:-1]))
    on = np.concatenate(([False], np.abs(np.diff(ref)) == step))
    head = round(HEAD_S * PERIOD_HZ)
    cycles = max(0, (len(ref) - head) // size)
    idx = head + np.arange(cycles * size).reshape(cycles, size)
    hole = ~whole[idx].all(axis=1)
    pause = ~hole & ~on[idx].all(axis=1)
    kept = idx[~hole & ~pause]
    w = np.pi / n
    e = np.exp(-1j * w * kept)
    return {"U": (ref[kept] / Q16 * e).sum(axis=1), "W": (omega[kept] / Q16 * e).sum(axis=1),
            "TH": ((theta[kept] - theta[kept - 1]) / Q16 * e).sum(axis=1) / (1 - np.exp(-1j * w)),
            "kp": int(facts["p_kp_q88"]) / 256, "cycles": len(kept),
            "holes": int(hole.sum()), "pauses": int(pause.sum()),
            "rows_dropped": int(facts["rows_dropped"]), "seq_holes": int(facts["seq_holes"])}


def db(x):
    return 20 * np.log10(abs(x))


def deg(x):
    """arg x in (-270, 90]."""
    a = float(np.degrees(np.angle(x)))
    return a - 360 if a > 90 else a


def cross(num, den, response):
    """sum num / sum den over a point's cycles, with its jackknife
    standard error in dB and deg, and how many of its own standard errors
    the mean of `response`, a cycle's, sits from zero."""
    nan = float("nan")
    k = len(num)
    est = {"L": complex(num.sum() / den.sum()) if k else complex(nan, nan), "se_db": nan, "se_deg": nan,
           "response_z": nan}
    if k < 2:
        return est
    groups = np.array_split(np.arange(k), min(BLOCKS, k))
    g = len(groups)
    loo = np.array([(num.sum() - num[i].sum()) / (den.sum() - den[i].sum()) for i in groups])

    def se(x):
        return float(np.sqrt((g - 1) / g * ((x - x.mean()) ** 2).sum()))
    spread = np.sqrt((response.real.var(ddof=1) + response.imag.var(ddof=1)) / k)
    return {**est, "se_db": se(20 * np.log10(np.abs(loo))), "se_deg": se(np.degrees(np.angle(loo / est["L"]))),
            "response_z": float(abs(response.mean()) / spread) if spread > 0 else float("inf")}


def readable(est):
    """A point's reading over its own noise."""
    return bool(est["se_db"] <= SE_DB_MAX and est["se_deg"] <= SE_DEG_MAX
                and est["response_z"] >= RESPONSE_SE_MIN)


def reading(bursts):
    """L_v and L_p of the spectra of one N at one cruise, each with its
    noise, and what went into them."""
    used = [s for s in bursts if s["cycles"]]
    out = {"bursts": len(used), "cycles": sum(s["cycles"] for s in bursts),
           **{k: sum(s[k] for s in bursts) for k in ("holes", "pauses", "rows_dropped", "seq_holes")}}
    out["dropped"] = out["holes"] + out["pauses"]
    u, w, th = (np.concatenate([s[k] for s in used] or [np.zeros(0, complex)]) for k in ("U", "W", "TH"))
    kp = np.concatenate([np.full(s["cycles"], s["kp"]) for s in used] or [np.zeros(0)])
    return {**out, "L_v": cross(w * np.conj(u), (u - w) * np.conj(u), w / u),
            "L_p": cross(kp * th * np.conj(u), np.abs(u) ** 2, kp * th / u)}


def apart(p, q):
    """Two readings of one point: (dB apart, deg apart, the two over the
    pair's combined standard error, both within APART_SIGMA), each
    standard error taken at least at its floor."""
    d_db = abs(p["db"] - q["db"])
    d_deg = abs((p["deg"] - q["deg"] + 180) % 360 - 180)
    z_db = d_db / (PAIR_FACTOR * np.hypot(max(p["se_db"], SE_FLOOR_DB), max(q["se_db"], SE_FLOOR_DB)))
    z_deg = d_deg / (PAIR_FACTOR * np.hypot(max(p["se_deg"], SE_FLOOR_DEG), max(q["se_deg"], SE_FLOOR_DEG)))
    return float(d_db), float(d_deg), float(z_db), float(z_deg), bool(z_db <= APART_SIGMA and z_deg <= APART_SIGMA)


def margins(points, nyquist):
    """The crossover, phase margin and gain margin of a loop's points in
    ascending frequency, the flagged and the unreadable left out. `nyquist`
    "read" takes the gain margin at Nyquist from L there, "bound" reports
    it as over the bound of the points above the crossover.
    `crossover_unread`: no crossover or phase margin is read and the 0 dB
    crossing may sit next to an unreadable point. `gm_unread`: no gain
    margin is read and the -180 crossing may sit next to an unreadable
    point."""
    use = [p for p in points if p["readable"] and p["two_cruise"] is not True]
    blind = [p["hz"] for p in points if not p["readable"]]
    inner = [p for p in use if p["half_periods"] > 1]
    out = {"crossover_hz": None, "pm_deg": None, "gm_db": None, "gm_hz": None, "gm_over": False,
           "crossover_unread": False, "gm_unread": False}

    def at(a, b, s):
        return float(np.exp(np.log(a["hz"]) + s * (np.log(b["hz"]) - np.log(a["hz"]))))
    pairs = list(zip(inner, inner[1:]))
    for a, b in pairs:
        if a["db"] >= 0 > b["db"]:
            if any(a["hz"] < hz < b["hz"] for hz in blind):
                out["crossover_unread"] = True
                break
            s = a["db"] / (a["db"] - b["db"])
            out["crossover_hz"] = at(a, b, s)
            out["pm_deg"] = 180 + a["deg"] + s * (b["deg"] - a["deg"])
            break
    for a, b in pairs:
        if a["deg"] > -180 >= b["deg"]:
            if any(a["hz"] < hz < b["hz"] for hz in blind):
                out["gm_unread"] = True
                return out
            s = (a["deg"] + 180) / (a["deg"] - b["deg"])
            out["gm_hz"] = at(a, b, s)
            out["gm_db"] = -(a["db"] + s * (b["db"] - a["db"]))
            return out
    beyond = any(hz > max((p["hz"] for p in inner), default=0.0) for hz in blind)
    if nyquist == "read":
        if beyond:
            out["gm_unread"] = True
            return out
        top = [p for p in use if p["half_periods"] == 1 and p["L"].real < 0]
        if top:
            out["gm_db"] = float(-db(top[0]["L"].real))
            out["gm_hz"] = top[0]["hz"]
    elif out["crossover_hz"] is not None:
        above = [p for p in inner if p["hz"] > out["crossover_hz"] and p["deg"] < BOUND_DEG]
        if above:
            p = max(above, key=lambda p: p["db"])
            out["gm_db"], out["gm_hz"], out["gm_over"] = -p["db"], p["hz"], True
    out["gm_unread"] = out["gm_db"] is None and beyond
    return out


def point(r, key):
    """A reading's one loop as a point: L, dB and deg, its noise and
    whether it is readable, and what went into it."""
    est = r[key]
    return {"L": est["L"], "db": db(est["L"]), "deg": deg(est["L"]),
            **{k: est[k] for k in ("se_db", "se_deg", "response_z")}, "readable": readable(est),
            **{k: r[k] for k in ("cruise_cps", "bursts", "cycles", "dropped", "holes", "pauses", "rows_dropped",
                                 "seq_holes")}}


def read(bursts):
    """{loop: its points in ascending frequency and its margins} of a
    sweep's bursts, each (rows, facts). A point is read at the lowest
    cruise that read it and checked at the others."""
    kps = {int(f["p_kp_q88"]) for _, f in bursts}
    if len(kps) != 1:
        raise ValueError(f"the sweep's bursts ran on {len(kps)} position gains")
    groups = {}
    for rows, facts in bursts:
        groups.setdefault((half_periods(facts), int(facts["cruise_cps"])), []).append(spectra(rows, facts))
    by_n = {}
    for (n, cruise), ss in sorted(groups.items()):
        by_n.setdefault(n, []).append({"cruise_cps": cruise, **reading(ss)})
    out = {}
    for loop, (key, nyquist) in LOOPS.items():
        points = []
        for n in sorted(by_n, reverse=True):
            first, *rest = (point(r, key) for r in by_n[n])
            checks = [{**{k: c[k] for k in ("cruise_cps", "db", "deg", "se_db", "se_deg", "readable")},
                       "agree": apart(first, c)[4] if first["readable"] and c["readable"] else None}
                      for c in rest if c["bursts"]]
            decided = [c["agree"] for c in checks if c["agree"] is not None]
            points.append({"half_periods": n, "hz": PERIOD_HZ / (2 * n), **first,
                           "two_cruise": not all(decided) if decided else None, "checks": checks})
        out[loop] = {"points": points, **margins(points, nyquist)}
    return out


def readings(bursts):
    """{reading_cps: read of its bursts} of a sweep's bursts, None the key
    of bursts whose facts carry no `reading_cps`."""
    groups = {}
    for rows, facts in bursts:
        groups.setdefault(facts.get("reading_cps"), []).append((rows, facts))
    return {k: read(v) for k, v in groups.items()}


def gm_agree(a, b):
    """Two gain margins, dB, agree: within REPEAT_DB, or both at least
    twice GM_FLOOR_DB."""
    return abs(a - b) <= REPEAT_DB or min(a, b) >= 2 * GM_FLOOR_DB


def bands(bursts):
    """{loop: its band's half-periods} from the bursts' `band` facts, None
    where the facts carry none."""
    out = {}
    for _, facts in bursts:
        if "band" not in facts:
            return None
        out.setdefault(facts["band"], set()).add(half_periods(facts))
    return out


def repeat(one, two, band=None):
    """{loop: the checks} of two reads of one reading: each point of the
    loop's band (`band`, as `bands` gives it; every point without it) both
    read readably and neither flagged, (N, dB apart, deg apart, within
    APART_SIGMA, dB and deg apart each over the pair's combined standard
    error); the phase and gain margins of each and whether they agree,
    None where one of the two read none; and `ok`, nothing that both read
    disagrees."""
    out = {}
    for loop in LOOPS:
        a, b = one[loop], two[loop]
        by_n = {p["half_periods"]: p for p in b["points"]}
        points = []
        for p in a["points"]:
            q = by_n.get(p["half_periods"])
            if band is not None and p["half_periods"] not in band.get(loop, ()):
                continue
            if q is None or True in (p["two_cruise"], q["two_cruise"]) or not (p["readable"] and q["readable"]):
                continue
            d_db, d_deg, z_db, z_deg, ok = apart(p, q)
            points.append((p["half_periods"], d_db, d_deg, ok, z_db, z_deg))
        pm = None if a["pm_deg"] is None or b["pm_deg"] is None else \
            bool(abs(a["pm_deg"] - b["pm_deg"]) <= REPEAT_PM_DEG)
        gm = None if a["gm_db"] is None or b["gm_db"] is None else bool(gm_agree(a["gm_db"], b["gm_db"]))
        out[loop] = {"points": points, "pm": (a["pm_deg"], b["pm_deg"], pm),
                     "gm": (a["gm_db"], b["gm_db"], gm),
                     "ok": all(p[3] for p in points) and pm is not False and gm is not False}
    return out


def load(folder):
    """Every burst of a sweep folder in capture order, each (rows, facts)."""
    out = []
    for k in range(10_000):
        p = Path(folder) / f"burst-{k}.csv"
        if not p.exists():
            break
        facts = json.loads(p.with_suffix(".json").read_text())
        missing = [f for f in FACTS if f not in facts]
        if missing:
            raise ValueError(f"{p.with_suffix('.json').name} lacks {', '.join(missing)}")
        out.append((pd.read_csv(p, usecols=list(COLUMNS)), facts))
    return out


def table(result, file=None):
    """Each loop's points, then its margins."""
    f = file or sys.stdout
    for loop, r in result.items():
        print(f"{loop} loop", file=f)
        print(f"{'N':>4}  {'Hz':>7}  {'dB':>7}  {'+-':>5}  {'deg':>7}  {'+-':>5}  {'bursts':>6}  {'cycles':>6}  "
              f"{'dropped':>7}  two-cruise", file=f)
        for p in r["points"]:
            flag = {None: "-", True: "flagged", False: "agrees"}[p["two_cruise"]]
            line = (f"{p['half_periods']:>4}  {p['hz']:>7.1f}  {p['db']:>7.2f}  {p['se_db']:>5.2f}  {p['deg']:>7.1f}  "
                    f"{p['se_deg']:>5.1f}  {p['bursts']:>6}  {p['cycles']:>6}  {p['dropped']:>7}  {flag:<10}  "
                    f"{'' if p['readable'] else 'unreadable'}")
            print(line.rstrip(), file=f)
        cross = "no crossover read: its 0 dB crossing may sit next to an unreadable point" \
            if r["crossover_unread"] else "no crossover" if r["crossover_hz"] is None else \
            f"crossover {r['crossover_hz']:.1f} Hz, phase margin {r['pm_deg']:.1f} deg"
        gm = "no gain margin read: its -180 crossing may sit next to an unreadable point" if r["gm_unread"] else \
            "no gain margin read" if r["gm_db"] is None else \
            f"gain margin {'over ' if r['gm_over'] else ''}{r['gm_db']:.2f} dB at {r['gm_hz']:.1f} Hz"
        print(f"{cross}; {gm}", file=f)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m oscnb.margin", description=__doc__.split("\n\n")[0])
    ap.add_argument("folder", help="a sweep's folder of burst-K.csv and burst-K.json")
    ap.add_argument("second", nargs="?", help="a second sweep of the same servo, checked against the first")
    a = ap.parse_args(argv)
    first = load(a.folder)
    one = readings(first)
    two = readings(load(a.second)) if a.second else {}
    for cruise, result in one.items():
        if cruise is not None:
            print(f"reading at {cruise} c/s")
        table(result)
        if cruise in two:
            for loop, r in repeat(result, two[cruise], bands(first)).items():
                worst_db = max((p[4] for p in r["points"]), default=0.0)
                worst_deg = max((p[5] for p in r["points"]), default=0.0)
                print(f"{loop} loop against the second sweep: {'repeats' if r['ok'] else 'does not repeat'} "
                      f"({len(r['points'])} points, worst {worst_db:.1f} sigma in dB, {worst_deg:.1f} in deg; "
                      f"phase margins agree {r['pm'][2]}, gain margins agree {r['gm'][2]})")


if __name__ == "__main__":
    main()
