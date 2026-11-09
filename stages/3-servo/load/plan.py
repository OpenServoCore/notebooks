"""Stage 3, load: the servo lifting a known torque. The printed arm hangs
from the horn with a mass at a known radius, the shaft horizontal, so the
arm is a pendulum: gravity's torque is m g L sin of the angle from the
bottom, nothing where the arm hangs and all of it at horizontal, and it
draws the arm back to the bottom whenever torque is off, never onto a
stop. The plans of `python -m oscnb.sitting servo-load`, 5 V, the current
limit widened to the servo's rated stall for the sitting, since a standing
arm is a stall the train holds; every duty is checked against that rating
before it drives.

`bottom` finds where the arm hangs: the arm hung straight down by hand at
the prompt and read, then a short lift each way and a drop, BOTTOM_DROPS
times a side, the rest read each time. The sitting refuses to go on unless
the arm can lift LIFT_MIN_DEG either side of the bottom inside the guards,
with which way to re-seat the arm on the spline. The two sides' rests
straddle the bottom by the angle the train's static friction holds against
gravity, so the block gives the bottom and that friction in one.

`ladder` is the torque curve: from the bottom, a duty held HOLD_MS between
two short windows, the duty stepped up LADDER until the arm passes
LADDER_TOP_DEG or the rating, each side. The arm comes to rest where the
motor's torque through the train equals gravity plus the friction that
resists its rise, so the rest angle against duty is the torque curve with
friction on top. Three of the duties are then approached from above, a
duty ABOVE_OVER higher first and then the duty, where the arm settles with
friction taken off instead, and the two rests bracket the friction under
load.

`lift` drives LIFT from the bottom for LIFT_MS, windows the free servo
would cross the reach in, and coasts COAST_MS: the drive is the train under
load at speed, the coast is the arm falling back and back-driving the
train, both streamed. No pilot sizes these windows: a pilot would seek the
far guard, which a hanging arm cannot reach.

`steps` steps the duty by STEP up and down from a ladder duty at three
angles, after STEADY_MS held there.

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
import time

from oscnb import capture, osc, tel
from oscnb.sitting import Plan, Skip, log_row

LEVEL = 5.0
REACH_DEG = 80                # the drives' envelope either side of the bottom
LIFT_MIN_DEG = 70             # the room the arm must have either side of the bottom
WINDOW_MS = 100
HOLD_MS = 3000
BOTTOM_DUTY = 12              # a lift well clear of the bottom, short of horizontal
BOTTOM_MS = 1500
DROP_MS = 3000                # the fall and its swing, streamed, one burst
BOTTOM_DROPS = 3
SETTLE_S = 1.5                # after the stream, before the rest is read
LADDER = tuple(range(4, 31, 2)) + tuple(range(34, 101, 4))
LADDER_TOP_DEG = 75           # the ladder stops once a rest passes this
ABOVE_OVER = 6
ABOVE_HOLD_MS = 2000
ABOVE_AT_DEG = (20, 40, 60)
HOT_PCT = 30                  # past this a hold rests HOT_REST_S longer
HOT_REST_S = 10
LIFT = (20, 30, 40)
LIFT_MS = {20: 1000, 30: 700, 40: 500}   # the soft limit cuts a lift that gets there
COAST_MS = 3000
LIFT_REPEATS = 2
STEP = 5
STEP_MS = 300
STEADY_MS = 2000
STEP_AT_DEG = (20, 40, 60)
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
                s.drive(out / sub, LEVEL, [f"{BOTTOM_DUTY}@{BOTTOM_MS}",
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
    found = round((up + down) / 2)
    room_check(s, found, "from the drops")
    s.found["load-bottom"] = found
    s.note("found", f"bottom {found} counts, the drops rest {up - down:.1f} "
                    f"counts apart, hanging it read {hang}")


def trial(s, out, k, bottom, sign, d, approach):
    """One hold on `sign`'s side at duty `d`, from the bottom or from
    ABOVE_OVER higher. Returns the rest pos, the last window's median."""
    dirs, guards = side(s, bottom, sign)
    if approach == "above":
        steps = [f"{d + ABOVE_OVER}@{WINDOW_MS}",
                 f"hold:{d + ABOVE_OVER}@{ABOVE_HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}", f"hold:{d}@{HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}"]
    else:
        steps = [f"{d}@{WINDOW_MS}", f"hold:{d}@{HOLD_MS}",
                 f"then:{d}@{WINDOW_MS}"]
    sub = f"ladder/{k}"
    t0 = time.time()
    s.drive(out / sub, LEVEL, steps, dirs=dirs, guards=guards, park=False)
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


def ladder(s, out):
    s.rail(LEVEL)
    bottom = bottom_of(s)
    duties = [d for d in LADDER if d <= cap_pct(s)]
    s.check(LEVEL, [str(d) for d in duties], seat="lock")
    rests = {1: {}, -1: {}}
    k = 0
    with osc.Limit(limit_counts(s)):
        for sign in (1, -1):
            for d in duties:
                rest = trial(s, out, k, bottom, sign, d, "below")
                rests[sign][d] = rest
                k += 1
                if abs(deg_from(s, rest, bottom)) >= LADDER_TOP_DEG:
                    break
            for want in ABOVE_AT_DEG:
                d = nearest(rests[sign], s, bottom, want)
                if d + ABOVE_OVER > cap_pct(s):
                    s.note("skip", f"from above at {d} %: {d + ABOVE_OVER} % is over the rating")
                    continue
                trial(s, out, k, bottom, sign, d, "above")
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
                    s.drive(out / sub, LEVEL, [f"{d}@{ms}", f"coast:{COAST_MS}"],
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
                for step in (STEP, -STEP):
                    if d + step <= 0 or d + step > cap_pct(s):
                        s.note("skip", f"{d} % stepped {step:+} % is out of range")
                        continue
                    s.check(LEVEL, [str(d), str(d + step)], seat="lock")
                    chain = [f"{d}@{WINDOW_MS}", f"hold:{d}@{STEADY_MS}",
                             f"then:{d + step}@{STEP_MS}", f"then:{d}@{STEP_MS}"]
                    sub = f"chains/{k}"
                    t0 = time.time()
                    s.drive(out / sub, LEVEL, chain, dirs=dirs, guards=guards,
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
         "a short lift and a drop each way, three times: the bottom and the static friction",
         run=bottom, order=0, minutes=2, mb=10),
    Plan("ladder", "load", "servo-load",
         "a duty held 3 s from the bottom, stepped up to the rating, each side; three from above",
         run=ladder, order=10, minutes=22, mb=40),
    Plan("lift", "load", "servo-load",
         "20 to 40 % from the bottom then a 3 s coast, twice each side",
         run=lift, order=20, minutes=8, mb=60),
    Plan("steps", "load", "servo-load",
         "5 % up and down from a ladder duty at three angles, each side",
         run=steps, order=30, minutes=6, mb=10),
    Plan("push", "load", "servo-load",
         "the pot under a side load by hand with no torque, servo free, 36 s",
         run=push, order=40, optional=True, minutes=2, mb=40),
]
