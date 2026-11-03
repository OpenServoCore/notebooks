"""Stage 3, load: the servo turning a known torque, the printed arm with a
mass at a known radius on a horizontal shaft, so gravity's torque is
m g L cos of the arm's angle. The plans of `python -m oscnb.sitting
servo-load`, 5 V, the guards LOAD_INSET_DEG inside the stops so the mass
never meets one. The arm points straight up at mid travel, so below mid
the drive that holds it is forward and above mid reverse.

`holds` stands the arm at seven angles. At each, a short window, an
unstreamed hold of 3 s and a second window, the duty stepped up from 2 %
by 2 % until the arm stops falling between the two windows, which is the
holding duty, and on until it lifts, which is the lifting duty: the band
between them is the train's friction both ways. Every trial is under the
servo's hold current, since a standing arm is a stall.

`lift` runs 30, 40 and 50 % cycles over the guards both ways, lifting the
arm to the top and lowering it past, with a coast at the end.

`steps` steps the duty by 5 % up and down from the holding duty at three of
the angles, after a second's hold there; the hold is the duty the arm stood
at, so only the stepped segments count against the runway. `crawl` crawls
up and down at the ladder's floor plus 2 %.

The datasets are `holds.csv` with `holds/K`, `cycles.csv` with `cycles/K`,
and `chains.csv` with `chains/K`.
"""
import time

from oscnb import osc, results, tel
from oscnb.sitting import Plan, Skip, log_row

LEVEL = 5.0
LOAD_INSET_DEG = 20
ANGLES = 7
HOLD_FROM, HOLD_BY = 2, 2     # the hold's duty grid, percent
HOLD_MS = 3000
WINDOW_MS = 100
STAND_COUNTS = 5              # pos change over a hold that counts as standing
LIFT = (30, 40, 50)
LIFT_MS = 1000
COAST_MS = 300
STEP = 5
STEP_MS = 300
STEADY_MS = 1000              # the hold before a step, the arm standing
STEP_ANGLES = (1, 3, 5)       # of the seven
KICK = 30
KICK_MS = 150
CRAWL_OVER = 2
CRAWL_MS = 3000


def load_guards(s):
    """The sitting's guards narrowed to LOAD_INSET_DEG inside the pot's
    stops, the envelope of every sweep from here on, the pilot's too."""
    inset = round(LOAD_INSET_DEG * s.servo.counts_per_deg_nominal)
    return s.narrow(osc.get("raw_min") + inset, osc.get("raw_max") - inset)


def angles(s):
    lo, hi = load_guards(s)
    return [lo + (i + 1) * (hi - lo) // (ANGLES + 1) for i in range(ANGLES)]


def trial(s, out, k, at, d):
    """One hold at `at` counts, signed duty `d`: the pos change between the
    two windows, in the drive's sense. The chain is cut like a free rung,
    since an arm past its lifting duty climbs through the whole hold."""
    lo, hi = load_guards(s)
    dirs, guards = ("fwd", (at, hi)) if d > 0 else ("rev", (lo, at))
    pct = abs(d)
    segments = [(pct, WINDOW_MS), (pct, HOLD_MS), (pct, WINDOW_MS)]
    first, held, last = s.fit(LEVEL, segments, runway=s.runway(guards))
    steps = [f"{pct}@{first}", f"hold:{pct}@{held}", f"then:{pct}@{last}"]
    sub = f"holds/{k}"
    t0 = time.time()
    s.drive(out / sub, LEVEL, steps, dirs=dirs, guards=guards)
    df, _ = tel.load(out / sub)
    windows = [g["pos"].median() for seg, g in df[df["seg"] > 0].groupby("seg")]
    change = (windows[-1] - windows[0]) * (1 if d > 0 else -1)
    log_row(out / "holds.csv", {"k": k, "angle_counts": at, "duty_pct": d,
                                "hold_ms": held, "change_counts": change,
                                "t_start": f"{t0:.3f}",
                                "t_end": f"{time.time():.3f}", "folder": sub})
    return change


def holds(s, out):
    top = int(s.servo.stop_hold_a * s.r_type * 100 / LEVEL)
    grid = range(HOLD_FROM, top + 1, HOLD_BY)
    s.check(LEVEL, [str(d) for d in grid], seat="hold")
    mid = sum(load_guards(s)) // 2
    found = {}
    k = 0
    for at in angles(s):
        sign = 1 if at <= mid else -1
        for d in grid:
            try:
                change = trial(s, out, k, at, sign * d)
            except Skip as e:
                s.note("skip", f"hold at {at} counts, {d} %: {e}")
                break
            k += 1
            s.rest("the hold done, the copper resting")
            if abs(change) <= STAND_COUNTS:
                found.setdefault(at, sign * d)
            if change > STAND_COUNTS:
                break
    s.found["load-holds"] = found


def lift(s, out):
    load_guards(s)
    s.servo_cycles(out, [(LEVEL, d) for d in LIFT], LIFT_MS, COAST_MS)


def steps(s, out):
    found = s.found.get("load-holds")
    if not found:
        raise Skip("no holding duty from the holds plan in this sitting")
    at = angles(s)
    k = 0
    for i in STEP_ANGLES:
        d = found.get(at[i])
        if d is None:
            s.note("skip", f"no holding duty at {at[i]} counts")
            continue
        pct = abs(d)
        lo, hi = load_guards(s)
        dirs, guards = ("fwd", (at[i], hi)) if d > 0 else ("rev", (lo, at[i]))
        for step in (STEP, -STEP):
            if pct + step <= 0:
                s.note("skip", f"{pct} % less {STEP} % is no drive")
                continue
            s.check(LEVEL, [str(pct), str(pct + step)], seat="hold")
            try:
                stepped, back = s.fit(LEVEL, [(pct + step, STEP_MS),
                                              (pct, STEP_MS)],
                                      runway=s.runway(guards))
            except Skip as e:
                s.note("skip", f"step {step:+} % at {at[i]} counts: {e}")
                continue
            chain = [f"{pct}@{WINDOW_MS}", f"hold:{pct}@{STEADY_MS}",
                     f"then:{pct + step}@{stepped}", f"then:{pct}@{back}"]
            sub = f"chains/{k}"
            t0 = time.time()
            s.drive(out / sub, LEVEL, chain, dirs=dirs, guards=guards)
            log_row(out / "chains.csv", {
                "k": k, "angle_counts": at[i], "duty_pct": d,
                "step_pct": step, "windows_ms": f"{stepped}/{back}",
                "t_start": f"{t0:.3f}",
                "t_end": f"{time.time():.3f}", "folder": sub})
            k += 1
            s.rest("the step done, the copper resting")


def crawl(s, out):
    try:
        doc = results.read("3-servo", "ladder", pair=s.group)
    except FileNotFoundError:
        raise Skip("no ladder result yet: the crawl needs the floor")
    d = round(100 * doc["floor_v_effective"]["value"] / LEVEL) + CRAWL_OVER
    load_guards(s)
    s.servo_cycles(out, [(LEVEL, d)], CRAWL_MS, COAST_MS, kick={LEVEL: KICK},
                   kick_ms=KICK_MS)


PLANS = [
    Plan("holds", "load", "servo-load",
         "the arm stood at seven angles, the duty stepped until it holds "
         "and until it lifts", run=holds, order=10, minutes=25, mb=50),
    Plan("lift", "load", "servo-load",
         "30, 40 and 50 % over the guards both ways, three cycles each",
         run=lift, order=20, minutes=8, mb=40),
    Plan("steps", "load", "servo-load",
         "5 % up and down from the holding duty at three angles",
         run=steps, order=30, minutes=3, mb=5),
    Plan("crawl", "load", "servo-load",
         "kick then 3 s at the floor plus 2 % both ways, three cycles",
         run=crawl, order=40, minutes=3, mb=55),
]
