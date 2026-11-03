"""Stage 3, pot-table: the camera sitting. The tag on the horn read against
the reference tag beside it is the angle's truth, for the pot's table over
the travel, the backlash, the travel in degrees, the gear ratio check, the
table's drift between the open and the close, and the crawl. The plans of
`python -m oscnb.sitting servo-camera`, 5 V.

Each block is one phone clip: the recording started at a prompt, `osc
clapper` flashing the STAT lamp at its start for the clock fit, the sweeps,
then the clip copied into the dataset under the block's name. Every
segment's host time is in its `segments.csv`. A clip is kept near a
minute, so the rests inside a clip are a few seconds: a crawl draws tens of
milliamps and warms nothing.

`pass`, the bracket, is one sweep of 20 % each way over the whole runway at
the open and the close of the sitting, the table's pass and its drift.

`crawl` is the slow crawl: a kick of 150 ms at 30 % and a hold of 3 s at
the floor less 1 % up to the floor plus 4 %, both directions, from three
starts, the guards and the guards moved in by a sixth and a third of the
travel. The floor is the ladder's sustaining floor from its result file,
so the camera sitting runs after the ladder notebook.

`reversal` runs `d` then `-d` at the crawl duty, the floor plus 2 %, and at
20 %. `steps` holds a crawl and steps it by 1 % up and down.
"""
import time

from oscnb import capture, results
from oscnb.sitting import Plan, Skip, log_row

LEVEL = 5.0
PASS_DUTY = 20
PASS_MS = 3000             # wanted; the runway cuts it to one end to the other
COAST_MS = 300
KICK = 30
KICK_MS = 150
CRAWL_OFFSETS = ((-1, 0, 1), (2, 4))   # duty over the floor, one clip each
CRAWL_MS = 3000
STARTS = (0.0, 1 / 6, 1 / 3)   # share of the travel the guards move in by
CRAWL_OVER = 2             # the reversal's and the steps' crawl, over the floor
REVERSAL_MS = 1500
STEP = 1
STEP_MS = 1500
CLIP_REST_MS = 2000        # between the directions of one sweep
CLIP_REST_S = 3            # between sweeps of one clip


def floor_duty(s):
    """The ladder's sustaining floor on LEVEL, whole percent."""
    try:
        doc = results.read("3-servo", "ladder", pair=s.group)
    except FileNotFoundError:
        raise Skip("no ladder result yet: the crawl needs the floor")
    return round(100 * doc["floor_v_effective"]["value"] / LEVEL)


def guards_at(s, share):
    lo, hi = s.guards()
    inset = round((hi - lo) * share)
    return (lo + inset, hi - inset), s.runway((lo + inset, hi))


def sweep(s, out, k, steps, guards=None, **extra):
    sub = f"chains/{k}"
    t0 = time.time()
    s.drive(out / sub, LEVEL, steps, guards=guards, rest_ms=CLIP_REST_MS)
    log_row(out / "chains.csv", {"k": k, "schedule": ",".join(steps),
                                 **extra, "t_start": f"{t0:.3f}",
                                 "t_end": f"{time.time():.3f}",
                                 "folder": sub})
    capture.hold(CLIP_REST_S, "between sweeps")


def a_pass(s, out):
    s.check(LEVEL, [str(PASS_DUTY)])
    (ms,) = s.fit(LEVEL, [(PASS_DUTY, PASS_MS)])
    with s.clip(out, "pass"):
        sweep(s, out, 0, [f"{PASS_DUTY}@{ms}", f"coast:{COAST_MS}"])


def crawl(s, out):
    floor = floor_duty(s)
    s.check(LEVEL, [str(KICK), str(floor + max(CRAWL_OFFSETS[-1]))])
    k = 0
    for i, share in enumerate(STARTS):
        guards, runway = guards_at(s, share)
        for j, offsets in enumerate(CRAWL_OFFSETS):
            with s.clip(out, f"crawl-{i}-{j}"):
                for off in offsets:
                    d = max(1, floor + off)
                    (ms,) = s.fit(LEVEL, [(d, CRAWL_MS)], [(KICK, KICK_MS)],
                                  runway)
                    sweep(s, out, k, [f"{KICK}@{KICK_MS}", f"then:{d}@{ms}",
                                      f"coast:{COAST_MS}"], guards,
                          start_share=f"{share:.3f}", duty_pct=d)
                    k += 1


def reversal(s, out):
    duties = (floor_duty(s) + CRAWL_OVER, PASS_DUTY)
    s.check(LEVEL, [str(KICK)] + [str(d) for d in duties])
    with s.clip(out, "reversal"):
        for k, d in enumerate(duties):
            (ms,) = s.fit(LEVEL, [(d, REVERSAL_MS)], [(KICK, KICK_MS)])
            sweep(s, out, k, [f"{KICK}@{KICK_MS}", f"then:{d}@{ms}",
                              f"then:{-d}@{ms}"], duty_pct=d)


def steps(s, out):
    d = floor_duty(s) + CRAWL_OVER
    s.check(LEVEL, [str(KICK), str(d + STEP)])
    with s.clip(out, "steps"):
        for k, step in enumerate((STEP, -STEP)):
            segments = [(d, STEP_MS), (d + step, STEP_MS)]
            hold, stepped = s.fit(LEVEL, segments, [(KICK, KICK_MS)])
            sweep(s, out, k, [f"{KICK}@{KICK_MS}", f"then:{d}@{hold}",
                              f"then:{d + step}@{stepped}"],
                  duty_pct=d, step_pct=step)


PLANS = [
    Plan("pass", "pot-table", "servo-camera",
         "20 % each way over the travel at the open and the close",
         run=a_pass, order=0, bracket=True, minutes=2, mb=5),
    Plan("crawl", "pot-table", "servo-camera",
         "kick then 3 s at the floor -1 to +4 %, three starts, six clips",
         run=crawl, order=10, minutes=10, mb=270),
    Plan("reversal", "pot-table", "servo-camera",
         "d then -d at the crawl and at 20 %, one clip",
         run=reversal, order=20, minutes=2, mb=35),
    Plan("steps", "pot-table", "servo-camera",
         "a crawl stepped by 1 % up and down, one clip",
         run=steps, order=30, minutes=2, mb=35),
]
