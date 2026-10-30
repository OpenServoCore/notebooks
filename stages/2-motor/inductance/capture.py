"""Stage 2, inductance: the winding's current rise into the locked rotor,
captured by the ADC free running on the shunt and the terminals. Needs the
bare motor in the rig with the wrench bar clamped, the board on the DPS and
the `osc` and `osc-lab` tools; no meter.

  uv run python stages/2-motor/inductance/capture.py [N]             writes telemetry/<motor>__<board>/inductance/capture-N
  uv run python stages/2-motor/inductance/capture.py reversal [N]    writes telemetry/<motor>__<board>/inductance/reversal-N
  uv run python stages/2-motor/inductance/capture.py steps [N]       writes telemetry/<motor>__<board>/inductance/steps-N
  uv run python stages/2-motor/inductance/capture.py fast [N]        writes telemetry/<motor>__<board>/inductance/fast-N

The rotor is held at all twelve holes, 30 degrees apart, so the bursts see
the winding through every commutator seat the bar can hold, six in each
phase of the 60 degree pattern three segments under two brushes repeat
with. Each hole runs five groups of bursts on a 5 V rail, in an order that
puts a different change before each group's first capture: a duty change
with the sign kept, a reversal, a duty change again, and a 20 s rest with
nothing changed. Each group is one `osc-lab burst` call into its own folder,
named in capture order, and `marks.csv` holds the host time each group and
rest began.

`reversal` holds the bar at hole 11, where the main run ends, and nothing is
moved. Every group is 25 % and each one reverses the sign of the one before
it, after a rest that grows from none to 50 s, about what a bar move takes.
The first group keeps the sign of the main run's last group, so it has no
reversal in front of it, and the last repeats the no-rest reversal, so a
drift through the run shows.

`steps` holds the bar at hole 11 and asks whether the first burst after a
reversal grows with the old sign's step. Every forward group is the same
25 % step and each follows a reverse group at 10, 25 or 40 %, with no rest
between. The three old steps run once rising and once falling, 10 and 40
again at the end, so a drift through the run does not line up with the step.
Magnetic history in the iron predicts a jump that grows with the old step,
travel inside the rig's play one that stops growing once the old step moves
the rotor.

`fast` holds the bar at hole 11 and runs the main run's four groups in fast
decay, where the current returns through the diodes and starts each period
from zero below 50 %, so the ramp inside each window gives the winding's L
on its own. Two slow groups after them tie the set to the main run at the
same seat.
"""
import sys

from oscnb import bench, boards, capture, motors, osc

HOLES = list(range(12))
LEVEL = 5.0
# (folder, duty %, direction, repeats, rest before it in s[, decay]): the change before each group's first capture is in the comment
GROUPS = [
    ("g1-fwd-25", 25, "fwd", 5, 0),              # the hole's first group, after the bar moved and the copper rested
    ("g2-fwd-40", 40, "fwd", 5, 0),              # a duty change, the sign kept
    ("g3-rev-40", 40, "rev", 5, 0),              # a reversal, the duty kept
    ("g4-rev-25", 25, "rev", 5, 0),              # a duty change, the sign kept
    ("g5-rev-25-rested", 25, "rev", 3, 20),      # a rest alone, the same step as before it
]
REST_S = 20
REVERSAL = [
    ("r0-rev-25", 25, "rev", 5, 0),              # the main run's last sign kept, no reversal
    ("r1-fwd-25", 25, "fwd", 5, 0),              # a reversal, no rest
    ("r2-rev-25-5s", 25, "rev", 5, 5),           # a reversal after 5 s
    ("r3-fwd-25-20s", 25, "fwd", 5, 20),         # a reversal after 20 s
    ("r4-rev-25-50s", 25, "rev", 5, 50),         # a reversal after 50 s, about a bar move
    ("r5-fwd-25", 25, "fwd", 5, 0),              # a reversal, no rest, again
]
STEPS = [
    ("s00-fwd-25", 25, "fwd", 5, 0),             # sets the forward sign, whatever ran before
    ("s01-rev-10", 10, "rev", 5, 0),             # the old step, 10 %
    ("s02-fwd-25", 25, "fwd", 5, 0),             # a reversal after 10 %
    ("s03-rev-25", 25, "rev", 5, 0),
    ("s04-fwd-25", 25, "fwd", 5, 0),             # a reversal after 25 %
    ("s05-rev-40", 40, "rev", 5, 0),
    ("s06-fwd-25", 25, "fwd", 5, 0),             # a reversal after 40 %
    ("s07-rev-40", 40, "rev", 5, 0),             # the falling order from here
    ("s08-fwd-25", 25, "fwd", 5, 0),             # a reversal after 40 %, again
    ("s09-rev-25", 25, "rev", 5, 0),
    ("s10-fwd-25", 25, "fwd", 5, 0),             # a reversal after 25 %, again
    ("s11-rev-10", 10, "rev", 5, 0),
    ("s12-fwd-25", 25, "fwd", 5, 0),             # a reversal after 10 %, again
]
FAST = [
    ("f1-fwd-25", 25, "fwd", 5, 0, "fast"),      # the main run's four groups in fast decay
    ("f2-fwd-40", 40, "fwd", 5, 0, "fast"),
    ("f3-rev-40", 40, "rev", 5, 0, "fast"),
    ("f4-rev-25", 25, "rev", 5, 0, "fast"),
    ("f5-fwd-25", 25, "fwd", 5, 0, "slow"),      # slow decay at the same seat, its first burst after a reversal
    ("f6-fwd-40", 40, "fwd", 5, 0, "slow"),
]
# subcommand: (dataset name, holes, groups); the bare command is the main run
MODES = {
    None: ("capture", HOLES, GROUPS),
    "reversal": ("reversal", [HOLES[-1]], REVERSAL),
    "steps": ("steps", [HOLES[-1]], STEPS),
    "fast": ("fast", [HOLES[-1]], FAST),
}

mode = next((a for a in sys.argv[1:] if a in MODES), None)
n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
name, holes, groups = MODES[mode]
out = capture.dataset(f"{motor.key}__{board.key}", "inductance", ["", *n], name=name)
print("motor", motor.key, "board", board.key)
capture.prompt(f"Motor leads on the board's motor connector, the board on the DPS. Bar on the pinion at hole {holes[0]}, clamp snug.")
bench.power_up(LEVEL + bench.DIODE_V)
dps_read, vsys = bench.set_vsys(LEVEL)
print(f"VSYS {LEVEL} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
marks = capture.Marks(out)
for k in holes:
    if k != holes[0]:
        capture.prompt(f"Pins out, bar turned to hole {k}, pins in.")
    for group, pct, dirs, repeats, rest, *decay in groups:
        if rest:
            marks.mark(f"hole-{k}/rest")
            capture.hold(rest, f"hole {k}: the motor resting before {group}")
        marks.mark(f"hole-{k}/{group}")
        osc.bursts(out / f"hole-{k}" / group, [pct], repeats=repeats, chans="diff", dirs=dirs,
                   decay=decay[0] if decay else "slow")
    marks.mark(f"hole-{k}/done")
    capture.hold(REST_S, f"hole {k} done, the copper resting")
marks.close()
bench.power_down()
print("wrote", out)
