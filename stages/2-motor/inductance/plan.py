"""Stage 2, inductance: the winding's current rise into the locked rotor,
captured by the ADC free running on the shunt and the terminals. A
locked-rotor plan, run by `python -m oscnb.sitting locked`, at the same
twelve holes as the resistance laps of the same sitting, so the bursts see
the winding through every commutator seat the bar can hold, six in each
phase of the 60 degree pattern three segments under two brushes repeat
with. Each hole runs five groups of bursts on a 5 V rail, in an order that
puts a different change before each group's first capture: a duty change
with the sign kept, a reversal, a duty change again, and a 20 s rest with
nothing changed. Each group is one `osc-lab burst` call into its own
folder, `hole-K/<group>`, and `marks.csv` holds the host time each group
and rest began.

`fast`, optional, runs after the last hole with the bar still in it: the
main run's four groups in fast decay, where the current returns through
the diodes and starts each period from zero below 50 %, so the ramp inside
each window gives the winding's L on its own, then two slow groups at the
same seat to tie the set to the main run. The first motor's reversal and
step-size runs at that seat answered their question and are not repeated.
"""
from oscnb import capture, osc
from oscnb.sitting import HOLES, Plan

LEVEL = 5.0
# (folder, duty %, direction, repeats, rest before it in s[, decay]): the change before each group's first capture is in the comment
GROUPS = [
    ("g1-fwd-25", 25, "fwd", 5, 0),              # the hole's first group, after the bar moved and the copper rested
    ("g2-fwd-40", 40, "fwd", 5, 0),              # a duty change, the sign kept
    ("g3-rev-40", 40, "rev", 5, 0),              # a reversal, the duty kept
    ("g4-rev-25", 25, "rev", 5, 0),              # a duty change, the sign kept
    ("g5-rev-25-rested", 25, "rev", 3, 20),      # a rest alone, the same step as before it
]
FAST = [
    ("f1-fwd-25", 25, "fwd", 5, 0, "fast"),      # the main run's four groups in fast decay
    ("f2-fwd-40", 40, "fwd", 5, 0, "fast"),
    ("f3-rev-40", 40, "rev", 5, 0, "fast"),
    ("f4-rev-25", 25, "rev", 5, 0, "fast"),
    ("f5-fwd-25", 25, "fwd", 5, 0, "slow"),      # slow decay at the same seat, its first burst after a reversal
    ("f6-fwd-40", 40, "fwd", 5, 0, "slow"),
]
_MARKS = {}


def setup(s, out):
    _MARKS[out] = capture.Marks(out)


def groups(s, out, k, plan):
    marks = _MARKS[out]
    s.rail(LEVEL)
    for group, pct, dirs, repeats, rest, *decay in plan:
        if rest:
            marks.mark(f"hole-{k}/rest")
            capture.hold(rest, f"hole {k}: the motor resting before {group}")
        marks.mark(f"hole-{k}/{group}")
        osc.bursts(out / f"hole-{k}" / group, [pct], repeats=repeats, chans="diff", dirs=dirs, decay=decay[0] if decay else "slow")
    marks.mark(f"hole-{k}/done")


def at_hole(s, out, k):
    groups(s, out, k, GROUPS)


def fast(s, out):
    groups(s, out, HOLES[-1], FAST)


def finish(s, out):
    _MARKS.pop(out).close()


PLANS = [
    Plan("capture", "inductance", "locked", "five burst groups at 5 V into every hole", setup=setup, at_hole=at_hole, finish=finish, order=20),
    Plan("fast", "inductance", "locked", "fast-decay groups at the last hole, the bar left in", setup=setup, tail=fast, finish=finish, order=30,
         optional=True),
]
