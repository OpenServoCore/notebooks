"""Stage 3, run-up: the free servo started from rest at 100 % on 5 V, so the
run-up sweeps the motor through the gears from stall toward its top speed
at the rated voltage exactly, the top the runner allows. The rise gives
the inertia of the rotor and the reflected train and the speed-torque
curve through the gears; the stretch at full duty, where nothing switches
and the crest sample is the mean, gives the dropout rate by direction and
the reading term's zero. A free-rotor plan of `python -m oscnb.sitting
servo`, the horn lock off.

`runup` is a bracket, three cycles of a 200 ms spin and a 500 ms coast at
the open and the close of the sitting, `runup-open-N` and `runup-close-N`,
so the brush state the rest of the sitting ran under is read on both sides
of it by the same rung. The window is cut by the runner to the runway like
every free rung, which at full speed leaves less than 200 ms.
"""
from oscnb.sitting import Plan

RUNUP = [(5.0, 100)]
SPIN_MS = 200      # about 90 motor revolutions at a geared top speed
COAST_MS = 500


def runups(s, out):
    s.servo_cycles(out, RUNUP, SPIN_MS, COAST_MS)


PLANS = [
    Plan("runup", "run-up", "servo",
         "three 5 V 100 % run-ups from rest at the open and the close",
         run=runups, order=0, bracket=True, minutes=3, mb=12),
]
