"""Stage 5, the scorecard's test set: the closed-loop schedules every rung
runs, written in the servo's own terms so another servo class runs the
same plan.

Up to four captures per run, each one `osc-lab track` in Position mode:

- `steps`: the first rows after torque-on held, a move to the center, then
  for each size center -> +size -> center -> -size -> center, each goal
  held a time that grows with the size.
- `follow`: goal ramps as a 50 Hz host sends them, one `goal:V@20` item a
  period, at shares of the no-load speed, up from the center and back
  down, a pause between ramps.
- `hold`: two holds at the center.
- `wide`: for each ADC wide code of the board that takes a slot, lowest
  first, two holds at the middle of the codes it skips, the first after a
  goal WIDE_APPROACH_COUNTS above the span the code stands for (itself
  and the codes it skips), the second after one as far below it. Such an
  approach goal is held as long as a step of its travel from the goal
  before it. A code is held when both of those goals fit the soft range
  as every goal of the set must. A board with no wide-code result, or
  with no such code, has no `wide` capture.

The inputs come through the chain, none typed: counts per degree from the
stage 3 pot-table result, the no-load speed from the stage 3 run-up result
through the registry's gear ratio, the wide codes from the board's stage 1
wide-codes result, the soft range from the servo, the center the middle of
it as `osc-lab track` takes it. Each capture's `plan.json` names every
segment: its test, size or speed share, direction and goal in counts and
in degrees from the center, and for a hold at a wide code its stuck code
and the side it is approached from.

The file is not a `plan.py`: it runs under `python -m oscnb.scorecard`,
never in a sitting.
"""
import math

from oscnb import results, scorecard, widecodes

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
WIDE_HOLD_MS = 2000
WIDE_APPROACH_COUNTS = 30           # from the span to the goal held before each hold
# Clear room between a goal and the guard, which aborts the run: an
# overshoot past the far goal must not reach it.
FIT_MARGIN_DEG = 5
CAP_MS = 100                        # the watchdog's current cap holds this long
CAPTURES = ("steps", "follow", "hold", "wide")   # in run order; `wide` only where the set built one


def inputs(servo, pair, board=None):
    """The constants the set is built from, with where each came from.
    `board` is the registry key of the board whose ADC wide codes the
    `wide` capture holds at: the ones its result of record gives a slot.
    Without a board, or for one with no result, there are none."""
    pot = results.read("3-servo", "pot-table", pair)
    runup = results.read("3-servo", "run-up", pair)
    rev_s = runup["no_load_speed_rev_s"]["value"]
    inp = {
        "counts_per_deg": pot["pot_scale_counts_per_deg"]["value"],
        "no_load_deg_s": rev_s / servo.gear_ratio * 360,
        "current_cap_ma": servo.lock_hold_a * 1000,
        "current_cap_ms": CAP_MS,
        "wide_codes": [],
        "sources": {
            "counts_per_deg": f"{results.rel(results.path('3-servo', 'pot-table', pair))} pot_scale_counts_per_deg",
            "no_load_deg_s": f"{results.rel(results.path('3-servo', 'run-up', pair))} no_load_speed_rev_s "
                             f"/ servos {servo.key} gear_ratio x 360",
            "current_cap_ma": f"servos {servo.key} lock_hold_a",
        },
    }
    if board is not None and widecodes.result_file(board).exists():
        slotted = [b for b in widecodes.bands_of_record(board) if b.slot is not None]
        inp["wide_codes"] = [{"stuck_code": b.stuck, "skipped": b.skipped}
                             for b in sorted(slotted, key=lambda b: b.stuck)]
        inp["sources"]["wide_codes"] = (f"{scorecard.shown(widecodes.result_file(board))} wide_codes, "
                                        f"the codes that take a slot")
    return inp


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


def fits(goals, guard, cpd):
    """Whether every goal lies FIT_MARGIN_DEG clear of the guard."""
    lo, hi = guard
    margin = FIT_MARGIN_DEG * cpd
    return min(goals) - margin >= lo and max(goals) + margin <= hi


def wide_goals(code):
    """A wide code's goals: the approach above the span it stands for, the
    middle of the codes it skips, the approach below the span."""
    stuck, skipped = code["stuck_code"], code["skipped"]
    return stuck + skipped + WIDE_APPROACH_COUNTS, stuck + 1 + skipped // 2, stuck - WIDE_APPROACH_COUNTS


def approach_ms(travel_deg):
    """How long a goal `travel_deg` from the one before it is held so the
    shaft is there: the hold of the smallest step size at or above the
    travel, the largest size's when the travel is beyond it."""
    return next((ms for size, ms in zip(STEP_SIZES_DEG, STEP_HOLD_MS) if travel_deg <= size), STEP_HOLD_MS[-1])


def wide(center, cpd, guard, codes):
    """The wide capture's segments, in order: per wide code whose goals
    fit the soft range, lowest first, its middle held after the approach
    from above, then after the one from below, each approach held for
    its travel from the goal before it, the first from the center. Empty
    when no code fits."""
    segs = []
    at = center
    for code in sorted(codes, key=lambda c: c["stuck_code"]):
        above, middle, below = wide_goals(code)
        if not fits((above, below), guard, cpd):
            continue
        for approach, side in ((above, "above"), (below, "below")):
            fields = {"stuck_code": code["stuck_code"], "side": side}
            ms = approach_ms(abs(approach - at) / cpd)
            segs.append(_goal(center, cpd, approach, ms, "wide-approach", **fields))
            segs.append(_goal(center, cpd, middle, WIDE_HOLD_MS, "wide-hold", **fields))
            at = middle
    return segs


def check_fits(segs, guard, cpd):
    """Refuse a servo whose soft range cannot fit the set, before anything
    moves."""
    lo, hi = guard
    margin = FIT_MARGIN_DEG * cpd
    goals = [s["goal_counts"] for s in segs if s["goal_counts"] is not None]
    low, high = min(goals), max(goals)
    if not fits(goals, guard, cpd):
        raise SystemExit(
            f"the soft range {lo}..{hi} counts cannot fit the scorecard: its goals run "
            f"{low}..{high} counts and each needs {FIT_MARGIN_DEG} deg ({margin:.0f} counts) "
            f"clear of the guard; this servo needs a wider soft range or a smaller set")


def build(guard, inp):
    """Every capture's segments for a servo with soft range `guard`,
    refused when one does not fit; `wide` is among them only when a wide
    code of the inputs is in reach."""
    cpd = inp["counts_per_deg"]
    center = center_of(guard)
    sets = {"steps": steps(center, cpd),
            "follow": follow(center, cpd, inp["no_load_deg_s"]),
            "hold": hold()}
    wide_segs = wide(center, cpd, guard, inp.get("wide_codes", ()))
    if wide_segs:
        sets["wide"] = wide_segs
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
