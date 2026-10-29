"""Stage 2, inductance: the winding's current rise into the locked rotor,
captured by the ADC free running on the shunt and the terminals. Needs the
bare motor in the rig with the wrench bar clamped, the board on the DPS and
the `osc` and `osc-lab` tools; no meter.

  uv run python stages/2-motor/inductance/capture.py [N]    writes telemetry/<motor>__<board>/inductance/capture-N

The rotor is held at four holes a quarter turn apart, so the bursts see the
winding through different commutator seats. Each hole gets the burst ladder
at two duties, both signs, five captures each, on a 5 V rail.
"""
import sys

from oscnb import bench, boards, capture, motors, osc

HOLES = [0, 1, 2, 3]
LEVEL = 5.0
PCT = [25, 40]
REPEATS = 5
REST_S = 20

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "inductance", ["", *n])
print("motor", motor.key, "board", board.key)
capture.prompt(f"Motor leads on the board's motor connector, the board on the DPS. Bar on the pinion at hole {HOLES[0]}, clamp snug.")
bench.power_up(LEVEL + bench.DIODE_V)
dps_read, vsys = bench.set_vsys(LEVEL)
print(f"VSYS {LEVEL} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
for k in HOLES:
    if k != HOLES[0]:
        capture.prompt(f"Pins out, bar turned to hole {k}, pins in.")
    osc.bursts(out / f"hole-{k}", PCT, repeats=REPEATS, chans="diff")
    capture.hold(REST_S, f"hole {k} done, the copper resting")
bench.power_down()
print("wrote", out)
