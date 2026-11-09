"""Stage 3, load: the servo lifting a known torque. The printed arm hangs
from the horn with a mass at a known radius, the shaft horizontal, so the
arm is a pendulum: gravity's torque is m g L sin of the angle from the
bottom, nothing where the arm hangs and all of it at horizontal, and it
draws the arm back to the bottom whenever torque is off, never onto a
stop. The plans of `python -m oscnb.sitting servo-load`, 5 V, the current
limit widened to the servo's rated stall for the sitting, since a standing
arm is a stall the train holds; every duty is checked against that rating
before it drives.

`bottom` reads where the arm hangs: the arm hung straight down by hand at
the prompt and read, and that reading is the bottom (two hangs of the
same arm read 2024 and 2025 counts; the drops' rests straddle it by the
static band and their count midpoint sits a degree off it, since the
pot's scale differs on the two sides). `OSC_LOAD_BOTTOM` in the
environment gives the reading instead of the prompt, for a sitting run
with the arm already hung. Then a lift each way to BOTTOM_DUTY, past the
angle the train's static friction holds, and a drop, BOTTOM_DROPS times a
side, the rest read each time: the swing of each drop and where it dies
are the friction of the back-driven train. The sitting refuses to go on
unless the arm can lift LIFT_MIN_DEG either side of the bottom inside the
guards, with which way to re-seat the arm on the spline.

`ladder` is the torque curve: from the bottom, a duty held HOLD_MS between
two short windows, the duty stepped up LADDER with the sides alternated at
each duty so time and direction do not confound, until a side's rest
passes LADDER_TOP_DEG or the rating; a rung whose rest the side's rests
predict past LADDER_REFUSE_DEG is left out, since a hold past horizontal
runs to the end stop. The prediction is the sine of the rests as a line
in duty, which is what a motor torque linear in duty against gravity
gives. The arm comes to rest where the motor's torque
through the train equals gravity plus the friction that resists its rise,
so the rest angle against duty is the torque curve with friction on top.
Three of the duties are then approached from above: a duty up to
ABOVE_OVER_MAX higher first, the overshoot cut back until the side's own
rests predict it under ABOVE_TOP_DEG, then the duty in one step, so the
arm falls onto its rest with friction taken off instead; a rest reached
by a stair of small steps stays where the first step left it, since the
back-driven train's release friction is many times one step's torque.
Under ABOVE_OVER_MIN of overshoot the rung is left out. The two rests
bracket the friction under load; the steps are the measurement of the
band itself.

`lift` drives LIFT from the bottom for LIFT_MS, windows the free servo
would cross the reach in, and coasts COAST_MS: the drive is the train under
load at speed, the coast is the arm falling back and back-driving the
train, both streamed. No pilot sizes these windows: a pilot would seek the
far guard, which a hanging arm cannot reach.

`steps` steps the duty by each of STEPS up and down from a ladder duty
at STEP_AT_DEG, after STEADY_MS held there: the smallest step that moves
the arm brackets the hold's deadband each way, the larger ones its
response.

`push`, optional, is the pot under a side load with no torque: the servo
free at zero duty for PUSH_COASTS coasts while the arm's hub is pressed
straight toward the shaft from each side and along the arm by hand, a
few seconds on and off each. A pot that moves while nothing turns reads
force, not angle; the hand-rock clip of the pot table saw it read half a
degree beyond the horn in the push direction.

The datasets are `bottom.csv`, `ladder.csv` with `ladder/K`, `cycles.csv`
with `cycles/K`, `chains.csv` with `chains/K` and `push/chains/0`. Every sweep leaves the
horn where it is, torque off, so the arm swings to the bottom by itself.
"""
import math
import os
import subprocess
import time

from oscnb import capture, osc, tel
from oscnb.sitting import Plan, Skip, log_row

LEVEL = 5.0
REACH_DEG = 80                # the drives' envelope either side of the bottom
LIFT_MIN_DEG = 70             # the room the arm must have either side of the bottom
WINDOW_MS = 100
HOLD_MS = 3000
BOTTOM_DUTY = 20              # lifts past the static band, short of horizontal
BOTTOM_MS = 1000
DROP_MS = 3000                # the fall and its swing, streamed, one burst
BOTTOM_DROPS = 3
SETTLE_S = 1.5                # after the stream, before the rest is read
LADDER = tuple(range(4, 31, 2)) + tuple(range(34, 101, 4))
LADDER_TOP_DEG = 60           # a side stops once a rest passes this
LADDER_REFUSE_DEG = 75        # a rung predicted past this is left out
ABOVE_OVER_MAX = 20           # the deadband held 16 % off
ABOVE_OVER_MIN = 10
ABOVE_TOP_DEG = 65            # the overshoot's rest stays under this
ABOVE_HOLD_MS = 1000
ABOVE_AT_DEG = (20, 40, 60)
HOT_PCT = 30                  # past this a hold rests HOT_REST_S longer
HOT_REST_S = 10
LIFT = (20, 30, 40)
LIFT_MS = {20: 1000, 30: 700, 40: 500}   # the soft limit cuts a lift that gets there
COAST_MS = 3000
LIFT_REPEATS = 2
STEPS = (10, 15, 20, 25)      # 5 % moved nothing, 16 % off held
STEP_MS = 500
STEADY_MS = 2000
STEP_AT_DEG = (20, 50)
PUSH_MS = 3000
PUSH_COASTS = 12
HANG = ("Hang the arm straight down by hand, the mass pointing at the "
        "floor, and let go.")


def cap_pct(s):
    """The most duty whose stall the rated stall current allows."""
    return int(s.servo.lock_hold_a * s.r_type * 100 / LEVEL)


def limit_counts(s):
    """The rated stall as the shunt counts the servo's limit takes."""
    return round(s.servo.lock_hold_a * s.board.counts_per_a)


def drive(s, out, steps, **kw):
    """`s.drive` on the pendulum, once more after a failed seek: the arm
    swings through the start band at the bottom and the seek catches it
    there most times, not every time."""
    for attempt in (1, 2):
        try:
            return s.drive(out, LEVEL, steps, **kw)
        except subprocess.CalledProcessError:
            if attempt == 2:
                raise
            s.note("retry", f"{out.name}: the sweep failed, the arm settling, once more")
            capture.hold(SETTLE_S * 4)


def bottom_of(s):
    b = s.found.get("load-bottom")
    if b is None:
        raise Skip("no bottom from the bottom plan in this sitting")
    return b


def deg_from(s, pos, bottom):
    return (pos - bottom) / s.servo.counts_per_deg_nominal


def room_check(s, bottom, what):
    """Refuse a bottom that leaves the arm less than LIFT_MIN_DEG of lift
    inside the guards on either side."""
    lo, hi = s.guards()
    down, up = -deg_from(s, lo, bottom), deg_from(s, hi, bottom)
    if min(down, up) < LIFT_MIN_DEG:
        short = "high" if up < down else "low"
        raise SystemExit(f"the bottom {what} reads {bottom} counts: {down:.0f} deg "
                         f"of lift to the low guard, {up:.0f} to the high, under "
                         f"the {LIFT_MIN_DEG} the sitting needs. Re-seat the arm "
                         f"one spline tooth toward the {short} end and run again.")


def side(s, bottom, sign):
    """Direction and guards for a drive away from the bottom on `sign`'s
    side: the start band at the bottom, the envelope REACH_DEG up."""
    lo, hi = s.guards()
    reach = round(REACH_DEG * s.servo.counts_per_deg_nominal)
    if sign > 0:
        return "fwd", (bottom, min(hi, bottom + reach))
    return "rev", (max(lo, bottom - reach), bottom)


def windows(folder):
    """The median pos of every streamed segment after the baseline."""
    df, _ = tel.load(folder)
    return [g["pos"].median() for _, g in df[df["seg"] > 0].groupby("seg")]


def bottom(s, out):
    s.rail(LEVEL)
    s.check(LEVEL, [str(BOTTOM_DUTY)], seat="lock")
    given = os.environ.get("OSC_LOAD_BOTTOM")
    if given:
        hang = int(given)
        s.note("given", f"bottom {hang} counts from OSC_LOAD_BOTTOM, no hang prompt")
    else:
        s.prompt(HANG)
        hang = s.probe("hanging")["pos"]
    room_check(s, hang, "by hand")
    rests = {1: [], -1: []}
    k = 0
    with osc.Limit(limit_counts(s)):
        for sign in (1, -1):
            dirs, guards = side(s, hang, sign)
            for _ in range(BOTTOM_DROPS):
                sub = f"drops/{k}"
                t0 = time.time()
                drive(s, out / sub, [f"{BOTTOM_DUTY}@{BOTTOM_MS}",
                                     f"coast:{DROP_MS}"],
                      dirs=dirs, guards=guards, park=False)
                capture.hold(SETTLE_S)
                rest = osc.get("pos")
                rests[sign].append(rest)
                log_row(out / "bottom.csv", {"k": k, "side": sign,
                                             "rest_pos": rest,
                                             "t_start": f"{t0:.3f}",
                                             "t_end": f"{time.time():.3f}",
                                             "folder": sub})
                k += 1
    up = sum(rests[1]) / len(rests[1])
    down = sum(rests[-1]) / len(rests[-1])
    s.found["load-bottom"] = hang
    s.note("found", f"bottom {hang} counts by hand; the drops rest {up - down:.1f} "
                    f"counts apart about {(up + down) / 2:.0f}")


def trial(s, out, k, bottom, sign, d, approach, over=0):
    """One hold on `sign`'s side at duty `d`, from the bottom or from
    `over` higher. Returns the rest pos, the last window's median."""
    dirs, guards = side(s, bottom, sign)
    if approach == "above":
        steps = [f"{d + over}@{WINDOW_MS}", f"hold:{d + over}@{ABOVE_HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}", f"hold:{d}@{HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}"]
    else:
        steps = [f"{d}@{WINDOW_MS}", f"hold:{d}@{HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}"]
    sub = f"ladder/{k}"
    t0 = time.time()
    drive(s, out / sub, steps, dirs=dirs, guards=guards, park=False)
    w = windows(out / sub)
    rest = w[-1]
    log_row(out / "ladder.csv", {"k": k, "side": sign, "duty_pct": d,
                                 "approach": approach, "hold_ms": HOLD_MS,
                                 "first_pos": w[0], "rest_pos": rest,
                                 "rest_deg": f"{deg_from(s, rest, bottom):.2f}",
                                 "t_start": f"{t0:.3f}",
                                 "t_end": f"{time.time():.3f}", "folder": sub})
    s.rest(f"hold {k} done ({sign * d:+} %), the copper resting")
    if d > HOT_PCT:
        capture.hold(HOT_REST_S, "a hot hold, resting longer")
    return rest


def nearest(rests, s, bottom, want_deg):
    """The ladder duty whose rest is nearest `want_deg` from the bottom."""
    return min(rests, key=lambda d: abs(abs(deg_from(s, rests[d], bottom)) - want_deg))


def deg_at(s, rests, bottom, d):
    """The rest a side's rests so far predict at duty `d`, degrees from
    the bottom. The motor's torque grows with the duty and gravity's with
    the sine of the angle, so the sine of the rests is a line in duty;
    past horizontal the prediction reads 90. None before two rests that
    lifted the arm exist."""
    pts = [(dd, math.sin(math.radians(abs(deg_from(s, r, bottom)))))
           for dd, r in sorted(rests.items()) if abs(deg_from(s, r, bottom)) >= 5]
    if len(pts) < 2:
        return None
    n = len(pts)
    mx = sum(x for x, _ in pts) / n
    my = sum(y for _, y in pts) / n
    sxx = sum((x - mx) ** 2 for x, _ in pts)
    slope = sum((x - mx) * (y - my) for x, y in pts) / sxx
    sine = my + slope * (d - mx)
    return 90.0 if sine >= 1 else math.degrees(math.asin(max(sine, 0.0)))


def overshoot(s, rests, bottom, d):
    """The most overshoot, ABOVE_OVER_MAX down by twos, whose rest the
    side's rests predict under ABOVE_TOP_DEG inside the rating; None
    under ABOVE_OVER_MIN."""
    for over in range(ABOVE_OVER_MAX, ABOVE_OVER_MIN - 1, -2):
        if d + over > cap_pct(s):
            continue
        ahead = deg_at(s, rests, bottom, d + over)
        if ahead is not None and ahead < ABOVE_TOP_DEG:
            return over
    return None


def ladder(s, out):
    s.rail(LEVEL)
    bottom = bottom_of(s)
    duties = [d for d in LADDER if d <= cap_pct(s)]
    s.check(LEVEL, [str(d) for d in duties], seat="lock")
    rests = {1: {}, -1: {}}
    open_sides = [1, -1]
    k = 0
    with osc.Limit(limit_counts(s)):
        for d in duties:
            for sign in list(open_sides):
                ahead = deg_at(s, rests[sign], bottom, d)
                if ahead is not None and ahead >= LADDER_REFUSE_DEG:
                    s.note("skip", f"{sign * d:+} %: the last rests predict "
                                   f"{ahead:.0f} deg, past horizontal")
                    open_sides.remove(sign)
                    continue
                rest = trial(s, out, k, bottom, sign, d, "below")
                rests[sign][d] = rest
                k += 1
                if abs(deg_from(s, rest, bottom)) >= LADDER_TOP_DEG:
                    open_sides.remove(sign)
            if not open_sides:
                break
        for sign in (1, -1):
            for want in ABOVE_AT_DEG:
                d = nearest(rests[sign], s, bottom, want)
                over = overshoot(s, rests[sign], bottom, d)
                if over is None:
                    s.note("skip", f"from above at {sign * d:+} %: no overshoot of "
                                   f"{ABOVE_OVER_MIN} % or more rests under {ABOVE_TOP_DEG} deg")
                    continue
                trial(s, out, k, bottom, sign, d, "above", over)
                k += 1
    s.found["load-ladder"] = rests


def lift(s, out):
    s.rail(LEVEL)
    bottom = bottom_of(s)
    s.check(LEVEL, [str(d) for d in LIFT], seat="lock")
    k = 0
    with osc.Limit(limit_counts(s)):
        for sign in (1, -1):
            dirs, guards = side(s, bottom, sign)
            for d in LIFT:
                ms = LIFT_MS[d]
                for r in range(LIFT_REPEATS):
                    sub = f"cycles/{k}"
                    t0 = time.time()
                    drive(s, out / sub, [f"{d}@{ms}", f"coast:{COAST_MS}"],
                          dirs=dirs, guards=guards, park=False)
                    dps_read, vsys = s.rail_read
                    log_row(out / "cycles.csv", {"k": k, "volts_set": LEVEL,
                                                 "dps_read": f"{dps_read:.3f}",
                                                 "vsys_read": f"{vsys:.3f}",
                                                 "side": sign, "duty_pct": d,
                                                 "repeat": r, "spin_ms": ms,
                                                 "t_start": f"{t0:.3f}",
                                                 "t_end": f"{time.time():.3f}",
                                                 "folder": sub})
                    k += 1
                    s.rest(f"lift {k - 1} done ({sign * d:+} %), resting")


def steps(s, out):
    s.rail(LEVEL)
    bottom = bottom_of(s)
    rests = s.found.get("load-ladder")
    if not rests:
        raise Skip("no ladder rests from the ladder plan in this sitting")
    k = 0
    with osc.Limit(limit_counts(s)):
        for sign in (1, -1):
            dirs, guards = side(s, bottom, sign)
            for want in STEP_AT_DEG:
                d = nearest(rests[sign], s, bottom, want)
                for step in [x * sg for x in STEPS for sg in (1, -1)]:
                    if d + step <= 0 or d + step > cap_pct(s):
                        s.note("skip", f"{d} % stepped {step:+} % is out of range")
                        continue
                    s.check(LEVEL, [str(d), str(d + step)], seat="lock")
                    chain = [f"{d}@{WINDOW_MS}", f"hold:{d}@{STEADY_MS}",
                             f"then:{d + step}@{STEP_MS}", f"then:{d}@{STEP_MS}"]
                    sub = f"chains/{k}"
                    t0 = time.time()
                    drive(s, out / sub, chain, dirs=dirs, guards=guards,
                          park=False)
                    log_row(out / "chains.csv", {"k": k, "side": sign,
                                                 "duty_pct": d, "step_pct": step,
                                                 "schedule": ",".join(chain),
                                                 "t_start": f"{t0:.3f}",
                                                 "t_end": f"{time.time():.3f}",
                                                 "folder": sub})
                    k += 1
                    s.rest(f"step {k - 1} done ({sign * d:+} % {step:+} %), resting")


def push(s, out):
    s.rail(LEVEL)
    s.prompt("Hands ready at the arm's hub, the servo free: when the stream "
             "starts, press the hub straight toward the shaft from the left "
             "for 3 s, off 2 s, then from the right, then push along the arm "
             "toward the shaft, then pull along the arm, and repeat the four "
             "once more. No twisting.")
    t0 = time.time()
    s.drive(out / "chains/0", LEVEL, [f"coast:{PUSH_MS}"] * PUSH_COASTS,
            dirs="fwd", seat="lock")
    log_row(out / "chains.csv", {"k": 0, "schedule": f"coast:{PUSH_MS} x {PUSH_COASTS}",
                                 "t_start": f"{t0:.3f}",
                                 "t_end": f"{time.time():.3f}",
                                 "folder": "chains/0"})


PLANS = [
    Plan("bottom", "load", "servo-load",
         "the arm hung by hand read as the bottom, then a lift past the static band and a drop each way, three times",
         run=bottom, order=0, minutes=2, mb=10),
    Plan("ladder", "load", "servo-load",
         "a duty held 3 s from the bottom, stepped up to 60 deg, sides alternated; three dropped on from above",
         run=ladder, order=10, minutes=20, mb=40),
    Plan("lift", "load", "servo-load",
         "20 to 40 % from the bottom then a 3 s coast, twice each side",
         run=lift, order=20, minutes=8, mb=60),
    Plan("steps", "load", "servo-load",
         "10 to 25 % up and down from a ladder duty at two angles, each side",
         run=steps, order=30, minutes=16, mb=25),
    Plan("push", "load", "servo-load",
         "the pot under a side load by hand with no torque, servo free, 36 s",
         run=push, order=40, optional=True, minutes=2, mb=40),
]
