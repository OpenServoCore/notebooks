"""Stage 3, ladder: the free servo spun at a set duty from one guard toward
the other, then the bridge opened and the rotor left to coast to its
natural end, for Ke, the brush drop, the floor and the breakaway, and the
steady current and speed friction reads. Free-rotor plans of `python -m
oscnb.sitting servo`, the horn lock off; a cycle is one `osc-lab sweep`, a
spin and a coast in both directions, and a dataset is `cycles/K` folders
with one row per cycle in `cycles.csv`.

The travel is about 157 motor revolutions, so a spin is as long as the
travel lets it be: each window is the plan's ceiling for that duty, cut by
the runner so the rung's travel at a bound on its speed, read from the
sitting's pilot spin, ends short of the far guard. A rung the runway cuts
too short to read is left out and noted.

`ref`, the bracket, is three cycles of 30 % on 5 V at the open and the
close of the sitting, the reference a drift through the sitting shows
against.

`capture` is the ladder: the three rails at five duties, three cycles each,
a spin of 300 ms at 60 % up to 1.2 s at 15 %, a coast of 400 ms. Every rung
is inside the rated voltage as rail times duty.

`low` is the slow rungs on the three rails. Each spin kicks the rotor at
about 1.5 V effective for 150 ms and drops to the rung's duty, chained,
held 1.2 s, then coasts; the holds sit at 0.6 down to 0.2 V effective,
across the sustaining floor, which the ripple tach tells from a stop.

`rail`, optional, is the top rung each rail allows that `capture` leaves
out: 80 and 100 % on 3.7 V and 67 % on 7.4 V, the rail term at full drive.
"""
from oscnb.sitting import Plan

RAILS = (3.7, 5.0, 7.4)
DUTIES = (15, 20, 30, 40, 60)
WINDOW_MS = {15: 1200, 20: 1000, 30: 700, 40: 500, 60: 300}   # ceilings
COAST_MS = 400
REF = [(5.0, 30)]
KICK = {3.7: 40, 5.0: 30, 7.4: 20}     # near 1.5 V effective on each rail
KICK_MS = 150
LOW_V = (0.6, 0.5, 0.4, 0.3, 0.25, 0.2)   # the holds, effective volts
LOW_HOLD_MS = 1200
RAIL = [(3.7, 80), (3.7, 100), (7.4, 67)]
RAIL_WINDOW_MS = {67: 300, 80: 300, 100: 300}


def low_rungs():
    """(rail, duty) of the low holds: LOW_V on each rail, whole percent,
    a duty two levels round to the same kept once."""
    out = []
    for v in RAILS:
        duties = sorted({max(1, round(100 * e / v)) for e in LOW_V},
                        reverse=True)
        out += [(v, d) for d in duties]
    return out


def ref(s, out):
    s.servo_cycles(out, REF, WINDOW_MS, COAST_MS)


def capture(s, out):
    s.servo_cycles(out, [(v, d) for v in RAILS for d in DUTIES], WINDOW_MS,
                   COAST_MS)


def low(s, out):
    s.servo_cycles(out, low_rungs(), LOW_HOLD_MS, COAST_MS, kick=KICK,
                   kick_ms=KICK_MS)


def rail(s, out):
    s.servo_cycles(out, RAIL, RAIL_WINDOW_MS, COAST_MS)


PLANS = [
    Plan("ref", "ladder", "servo",
         "three cycles of 30 % on 5 V at the open and the close",
         run=ref, order=5, bracket=True, minutes=3, mb=20),
    Plan("capture", "ladder", "servo",
         "3.7, 5 and 7.4 V at 15 to 60 %, three cycles each",
         run=capture, order=30, minutes=35, mb=270),
    Plan("low", "ladder", "servo",
         "kick then hold at 0.6 to 0.2 V effective on the three rails",
         run=low, order=40, minutes=40, mb=490),
    Plan("rail", "ladder", "servo",
         "3.7 V 80 and 100 %, 7.4 V 67 %, the top of each rail",
         run=rail, order=60, optional=True, minutes=7, mb=35),
]
