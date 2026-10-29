"""Stage 2, bemf-coast: the free motor spun at a set duty, then the bridge
opened, the terminal voltage decaying as the rotor coasts. The spin's tail
is the no-load current, read by the no-load-current notebook from this
same dataset. Needs the bare motor in the rig with the wrench bar OFF, the
board on the DPS and the `osc` and `osc-lab` tools; no meter.

  uv run python stages/2-motor/bemf-coast/capture.py [N]    writes telemetry/<motor>__<board>/bemf-coast/capture-N

One cycle is a spin of a second and a half, long enough for the speed to
settle, then a coast of most of a second, both directions, as one sweep.
Cycles rest twenty seconds apart, the bench rule for a bare motor.
"""
import csv
import sys
import time

from oscnb import bench, boards, capture, motors, osc

LEVELS = [5.0, 7.4]
DUTIES = [15, 20, 30, 40, 60]
REPEATS = 3
SPIN_MS = 1500
COAST_MS = 800
REST_S = 20

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "bemf-coast", ["", *n])
print("motor", motor.key, "board", board.key)
capture.prompt("Motor leads on the board's motor connector, the bar OFF the pinion, the rotor free.")
bench.power_up(LEVELS[0] + bench.DIODE_V)
with open(out / "cycles.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["k", "volts_set", "dps_read", "vsys_read", "duty_pct", "repeat", "t_start", "t_end", "folder"])
    k = 0
    for v in LEVELS:
        dps_read, vsys = bench.set_vsys(v)
        print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
        for d in DUTIES:
            for r in range(REPEATS):
                folder = f"cycles/{k}"
                t0 = time.time()
                osc.sweep(out / folder, [f"{d}@{SPIN_MS}", f"coast:{COAST_MS}"], baseline_ms=500, rest_ms=REST_S * 1000,
                          dirs="both")
                w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", d, r, f"{t0:.3f}", f"{time.time():.3f}", folder])
                f.flush()
                k += 1
                capture.hold(REST_S, f"cycle {k - 1} done ({v} V, {d} %), the motor resting")
bench.power_down()
print("wrote", out)
