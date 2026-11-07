"""Stage 3, inductance: the winding's current rise from duty-step bursts
into the held rotor, the ADC free running on the shunt and the taps. The
same five groups as stage 2 on a 5 V rail, in an order that puts a
different change before each group's first capture. Each group is one
`osc-lab burst` call into its own folder and `marks.csv` holds the host
time each group and rest began. Run by `python -m oscnb.sitting servo`.

`locked` runs the groups with the horn lock on at mid travel, the seat
read before and after, under the servo's hold current.

`wound`, optional, runs them into the lock after a reverse press of the
seat duty has wound the train, the seat read before and after; against
`locked` it shows what the pre-wound train adds to the onset.

`stop`, optional, runs them at the low stop with the train wound, the
seat set by a `--stall` press that skips osc-lab's park, so the horn stays
on the stop, and read back before the first burst and after the last; the
plan centers the horn at the end. A read-back away from the stop ends the
plan rather than record a free rotor as a held one.
"""
from oscnb import capture, osc
from oscnb.sitting import Plan, Skip

LEVEL = 5.0
# (folder, duty %, direction, repeats, rest before it in s): the change
# before each group's first capture is in the comment
GROUPS = [
    ("g1-fwd-25", 25, "fwd", 5, 0),          # the first group, after a rest
    ("g2-fwd-40", 40, "fwd", 5, 0),          # a duty change, the sign kept
    ("g3-rev-40", 40, "rev", 5, 0),          # a reversal, the duty kept
    ("g4-rev-25", 25, "rev", 5, 0),          # a duty change, the sign kept
    ("g5-rev-25-rested", 25, "rev", 3, 20),  # a rest alone, the same step
]
SEAT_PCT = 24          # the stop press, 1.2 V effective on 5 V
SEAT_MS = 300
STOP_TOL = 150         # counts from the pot's low stop a pressed seat reads


def groups(s, out, seat):
    for _, pct, _, _, _ in GROUPS:
        s.check(LEVEL, [str(pct)], seat)
    marks = capture.Marks(out)
    try:
        for group, pct, dirs, repeats, rest in GROUPS:
            if rest:
                marks.mark("rest")
                capture.hold(rest, f"the copper resting before {group}")
            marks.mark(group)
            s.bursts(out / group, LEVEL, [pct], dirs, repeats, seat=seat)
        marks.mark("done")
    finally:
        marks.close()


def locked(s, out):
    s.seat(out, "lock", s.probe("before the bursts")["pos"])
    groups(s, out, "lock")
    s.seat(out, "lock", s.probe("after the bursts")["pos"])


def wound(s, out):
    s.check(LEVEL, [str(SEAT_PCT)] + [str(g[1]) for g in GROUPS], "lock")
    s.seat(out, "lock", s.probe("before the press")["pos"])
    s.drive(out / "seat", LEVEL, [f"{SEAT_PCT}@{SEAT_MS}"], dirs="rev",
            seat="lock")
    s.seat(out, "lock", s.probe("the seat")["pos"])
    groups(s, out, "lock")
    s.seat(out, "lock", s.probe("after the bursts")["pos"])


def stop(s, out):
    s.check(LEVEL, [str(SEAT_PCT)] + [str(g[1]) for g in GROUPS], "stop")
    s.drive(out / "seat", LEVEL, [f"{SEAT_PCT}@{SEAT_MS}"], dirs="rev",
            seat="stop", park=False)
    try:
        pos = s.probe("the seat")["pos"]
        low = osc.get("raw_min")
        if abs(pos - low) > STOP_TOL:
            raise Skip(f"the horn reads {pos} after the seat, {pos - low:+} "
                       "from the low stop: the press did not hold it there")
        s.seat(out, "low stop", pos)
        groups(s, out, "stop")
        s.seat(out, "low stop", s.probe("after the bursts")["pos"])
    finally:
        s.center()


PLANS = [
    Plan("locked", "inductance", "servo",
         "five burst groups at 5 V into the locked horn at mid travel",
         run=locked, order=15, lock=True, minutes=2, mb=1),
    Plan("wound", "inductance", "servo",
         "the same groups into the locked horn, the train wound by a press",
         run=wound, order=16, lock=True, optional=True, minutes=2, mb=2),
    Plan("stop", "inductance", "servo",
         "the same groups at the low stop, the train wound",
         run=stop, order=25, seated=True, optional=True, minutes=2, mb=2),
]
