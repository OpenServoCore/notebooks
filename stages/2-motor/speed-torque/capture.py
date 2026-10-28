"""Stage 2, speed-torque: the free motor started from rest at 100 % on a
fixed rail, so the run-up sweeps it from stall to no load at one voltage.
The rotor's own inertia is the load: the torque at each moment is that
inertia times the rise of the speed, plus the drag at that speed, both from
the no-load-current result. The coast after each spin is a second coast at
full speed for the same notebook. Needs the bare motor in the rig with the
wrench bar OFF, the board on the DPS and the `osc` tool; no meter.

  uv run python stages/2-motor/speed-torque/capture.py [N]    writes telemetry/<motor>__<board>/speed-torque/capture-N

Kept short for the brushes' sake, since a free motor at 100 % runs at its
top speed: one rail, three cycles, and a spin of 300 ms, which the run-up
needs about half of, so the settled speed at its end is a short stretch.
The coast is long enough for the rotor to stop from full speed. The start draws near the stall current for a few milliseconds,
about an amp at 5 V, well under the board's overcurrent trip.
"""
import csv
import sys
import time

from oscnb import bench, boards, capture, motors, osc

LEVEL = 5.0                  # rail volts, the lower bemf-coast level, so its partial-duty run-ups sit on the same rail
REPEATS = 3
SPIN_MS = 300
COAST_MS = 600
REST_S = 20

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "speed-torque", ["", *n])
print("motor", motor.key, "board", board.key)
capture.prompt("Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free.")
bench.power_up(LEVEL + bench.DIODE_V)
with open(out / "cycles.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["k", "volts_set", "dps_read", "vsys_read", "duty_pct", "repeat", "t_start", "t_end", "folder"])
    dps_read, vsys = bench.set_vsys(LEVEL)
    print(f"VSYS {LEVEL} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
    for k in range(REPEATS):
        folder = f"cycles/{k}"
        t0 = time.time()
        osc.sweep(out / folder, [f"100@{SPIN_MS}", f"coast:{COAST_MS}"], baseline_ms=500, rest_ms=REST_S * 1000,
                  dirs="both")
        w.writerow([k, LEVEL, f"{dps_read:.3f}", f"{vsys:.3f}", 100, k, f"{t0:.3f}", f"{time.time():.3f}", folder])
        f.flush()
        if k < REPEATS - 1:
            capture.hold(REST_S, f"cycle {k} done, the motor resting")
bench.power_down()
print("wrote", out)
