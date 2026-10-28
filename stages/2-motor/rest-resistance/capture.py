"""Stage 2, rest-resistance: the motor's terminal resistance at rest, over the
rotor's angle, from the board's own current and terminal taps. Needs the
bare motor in the rig with its leads soldered to the tabs, the wrench bar,
the board on the DPS and the `osc` tool; no meter.

  uv run python stages/2-motor/rest-resistance/capture.py [N]    writes telemetry/<motor>__<board>/rest-resistance/capture-N/stall

The pass steps the bar through the twelve holes of the rig's bolt circle,
prompting at each, so a reading belongs to a rotor angle. At each hole it
drives 100 % laps into the locked rotor at the three 1S rails, both
directions, in laps short enough to leave the copper cold. The taps and the
current chain were graded against the meter in stage 1.
"""
import csv
import sys
import time

from oscnb import bench, boards, capture, motors, osc

ANGLES = 12                  # the rig's bolt circle
LEVELS = [3.3, 3.7, 4.2]     # rail volts, the 1S levels: the stall current stays under an amp
LAP_MS = 300                 # 100 % into the locked rotor, cold: under a joule per lap
REST_S = 20                  # between angles, the bench rule for a bare motor

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "rest-resistance", ["", *n]) / "stall"
out.mkdir(parents=True, exist_ok=True)
print("motor", motor.key, "board", board.key)

capture.prompt("Motor leads on the board's motor connector, the board on the DPS. Bar on the pinion at hole 0, clamp snug.")
bench.power_up(LEVELS[0] + bench.DIODE_V)
with open(out / "laps.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["hole", "volts_set", "dps_read", "vsys_read", "t_start", "t_end", "folder"])
    for k in range(ANGLES):
        if k:
            capture.prompt(f"Pins out, bar turned to hole {k}, pins in.")
        for v in LEVELS:
            dps_read, vsys = bench.set_vsys(v)
            folder = f"hole-{k}/{v:g}V"
            t0 = time.time()
            osc.sweep(out / folder, [f"100@{LAP_MS}"], baseline_ms=500, rest_ms=1000, dirs="both")
            w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", f"{t0:.3f}", f"{time.time():.3f}", folder])
            f.flush()
        capture.hold(REST_S, f"hole {k} done, the copper resting")
bench.power_down()
print("wrote", out)
