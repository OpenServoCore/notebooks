"""Stage 5, the scorecard's test set: the closed-loop schedules every rung
runs, written in the servo's own terms so another servo class runs the
same plan.

Three captures per run, each one `osc-lab track` in Position mode:

- `steps`: the first rows after torque-on held, a move to the center, then
  for each size center -> +size -> center -> -size -> center, each goal
  held a time that grows with the size.
- `follow`: goal ramps as a 50 Hz host sends them, one `goal:V@20` item a
  period, at shares of the no-load speed, up from the center and back
  down, a pause between ramps.
- `hold`: two holds at the center.

The inputs come through the chain, none typed: counts per degree from the
stage 3 pot-table result, the no-load speed from the stage 3 run-up result
through the registry's gear ratio, the soft range from the servo, the
center the middle of it as `osc-lab track` takes it. Each capture's
`plan.json` names every segment: its test, size or speed share,
direction and goal in counts and in degrees from the center.

The file is not a `plan.py`: it runs under `python -m oscnb.scorecard`,
never in a sitting.
"""
import math

from oscnb import results

STEP_SIZES_DEG = (2, 20, 60)
STEP_HOLD_MS = (1200, 1500, 2000)   # per size, the settle and the 500 ms rest window
ENABLE_HOLD_MS = 1000
FOLLOW_SPEED_PCT = (1, 3, 10)       # of the no-load speed at the horn
FOLLOW_SPAN_DEG = 30
FOLLOW_SPAN_S = 3                   # a slow ramp is cut to this long
GOAL_ITEM_MS = 20                   # the burst of one 50 Hz goal
# A `goal:V@20` item takes this long on the bus, the burst and the host's
# overhead, as measured on the groundwork ramps; the ramp steps are sized
# from it so the ramp runs at the speed asked.
GOAL_PERIOD_S = 0.0256
PAUSE_MS = 500
HOLD_MS = 3000
HOLDS = 2
# Clear room between a goal and the guard, which aborts the run: an
# overshoot past the far goal must not reach it.
FIT_MARGIN_DEG = 5
CAP_MS = 100                        # the watchdog's current cap holds this long
CAPTURES = ("steps", "follow", "hold")


def inputs(servo, pair):
    """The constants the set is built from, with where each came from."""
    pot = results.read("3-servo", "pot-table", pair)
    runup = results.read("3-servo", "run-up", pair)
    rev_s = runup["no_load_speed_rev_s"]["value"]
    return {
        "counts_per_deg": pot["pot_scale_counts_per_deg"]["value"],
        "no_load_deg_s": rev_s / servo.gear_ratio * 360,
        "current_cap_ma": servo.lock_hold_a * 1000,
        "current_cap_ms": CAP_MS,
        "sources": {
            "counts_per_deg": f"{results.rel(results.path('3-servo', 'pot-table', pair))} pot_scale_counts_per_deg",
            "no_load_deg_s": f"{results.rel(results.path('3-servo', 'run-up', pair))} no_load_speed_rev_s "
                             f"/ servos {servo.key} gear_ratio x 360",
            "current_cap_ma": f"servos {servo.key} lock_hold_a",
        },
    }


def center_of(guard):
    """The middle of the soft range, as `osc-lab track` centers to it."""
    lo, hi = guard
    return lo + (hi - lo) // 2


def _goal(center, cpd, counts, ms, test, **fields):
    return {"item": f"goal:{counts}@{ms}", "test": test, **fields,
            "goal_counts": counts, "goal_deg": (counts - center) / cpd}


def _hold(ms, test):
    return {"item": f"hold@{ms}", "test": test, "goal_counts": None, "goal_deg": None}


def steps(center, cpd):
    """The steps capture's segments, in order."""
    segs = [_hold(ENABLE_HOLD_MS, "enable"),
            _goal(center, cpd, center, STEP_HOLD_MS[0], "lead")]
    for size, ms in zip(STEP_SIZES_DEG, STEP_HOLD_MS):
        out = round(size * cpd)
        for counts, direction, leg in ((center + out, 1, "out"), (center, -1, "back"),
                                       (center - out, -1, "out"), (center, 1, "back")):
            segs.append(_goal(center, cpd, counts, ms, "step", size_deg=size,
                              direction=direction, leg=leg))
    return segs


def ramp_goals(start, direction, counts_s, span_counts):
    """One ramp's goals, one a goal period, from `start` (not included) to
    `span_counts` along `direction`, each rounded from the exact line so the
    mean speed is the one asked."""
    n = math.ceil(span_counts / (counts_s * GOAL_PERIOD_S))
    return [start + direction * round(min(k * counts_s * GOAL_PERIOD_S, span_counts))
            for k in range(1, n + 1)]


def follow(center, cpd, no_load_deg_s):
    """The follow capture's segments, in order: a move to the center, then
    per speed a ramp up, a pause, a ramp back down, a pause."""
    segs = [_goal(center, cpd, center, PAUSE_MS, "lead")]
    ramp = 0
    for pct in FOLLOW_SPEED_PCT:
        deg_s = no_load_deg_s * pct / 100
        span = round(min(FOLLOW_SPAN_DEG, deg_s * FOLLOW_SPAN_S) * cpd)
        for start, direction in ((center, 1), (center + span, -1)):
            for counts in ramp_goals(start, direction, deg_s * cpd, span):
                segs.append(_goal(center, cpd, counts, GOAL_ITEM_MS, "ramp", speed_pct=pct,
                                  direction=direction, ramp=ramp))
            segs.append(_hold(PAUSE_MS, "pause"))
            ramp += 1
    return segs


def hold():
    return [_hold(HOLD_MS, "hold") for _ in range(HOLDS)]


def check_fits(segs, guard, cpd):
    """Refuse a servo whose soft range cannot fit the set, before anything
    moves."""
    lo, hi = guard
    margin = FIT_MARGIN_DEG * cpd
    goals = [s["goal_counts"] for s in segs if s["goal_counts"] is not None]
    low, high = min(goals), max(goals)
    if low - margin < lo or high + margin > hi:
        raise SystemExit(
            f"the soft range {lo}..{hi} counts cannot fit the scorecard: its goals run "
            f"{low}..{high} counts and each needs {FIT_MARGIN_DEG} deg ({margin:.0f} counts) "
            f"clear of the guard; this servo needs a wider soft range or a smaller set")


def build(guard, inp):
    """Every capture's segments for a servo with soft range `guard`,
    refused when one does not fit."""
    cpd = inp["counts_per_deg"]
    center = center_of(guard)
    sets = {"steps": steps(center, cpd),
            "follow": follow(center, cpd, inp["no_load_deg_s"]),
            "hold": hold()}
    for segs in sets.values():
        if any(s["goal_counts"] is not None for s in segs):
            check_fits(segs, guard, cpd)
    return center, sets


def plan_doc(capture, segs, center, guard, inp):
    """A capture's plan.json: the inputs and every segment by index."""
    return {
        "capture": capture,
        "center_counts": center,
        "guard_counts": list(guard),
        "counts_per_deg": inp["counts_per_deg"],
        "no_load_deg_s": inp["no_load_deg_s"],
        "goal_period_s": GOAL_PERIOD_S,
        "sources": inp["sources"],
        "segments": [{"seg": k, **s} for k, s in enumerate(segs)],
    }
