"""Stage 2, speed-torque: the free motor started from rest at 100 % on a
fixed rail, so the run-up sweeps it from stall to no load at one voltage.
The rotor's own inertia is the load: the torque at each moment is that
inertia times the rise of the speed, plus the drag at that speed, both from
the no-load-current result. The coast after each spin is a second coast at
full speed for the same notebook. A free-rotor plan, run by
`python -m oscnb.sitting free`.

Kept short for the brushes' sake, since a free motor at 100 % runs at its
top speed: one rail, three cycles, and a spin of 300 ms, which the run-up
needs about half of, so the settled speed at its end is a short stretch.
The coast is long enough for the rotor to stop from full speed. The start
draws near the stall current for a few milliseconds, about an amp at 5 V,
well under the board's overcurrent trip. On the rated 5 V rail, 100 % is
the rated voltage exactly, the top the sitting allows.
"""
from oscnb.sitting import Plan

LEVEL = 5.0                  # rail volts, the lower bemf-coast level, so its partial-duty run-ups sit on the same rail
SPIN_MS = 300
COAST_MS = 600


def runups(s, out):
    s.cycles(out, [(LEVEL, 100)], SPIN_MS, COAST_MS)


PLANS = [Plan("capture", "speed-torque", "free", "three 5 V 100 % run-ups from rest", run=runups, order=20)]
