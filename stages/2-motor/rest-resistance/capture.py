"""Stage 2, rest-resistance: the motor's terminal resistance at rest, over the
rotor's angle, by two routes. Needs the bare motor in the rig with its leads
soldered to the tabs, the wrench bar, the DMM and, for the stall pass, the
board on the DPS and the `osc` tool.

  uv run python stages/2-motor/rest-resistance/capture.py ohms [N]     the meter in ohms across the tabs, the motor off the board
  uv run python stages/2-motor/rest-resistance/capture.py stall [N]    100 % laps into the locked rotor, the board's own reading

Both passes step the bar through the twelve holes of the rig's bolt circle,
prompting at each, so a reading belongs to a rotor angle. The ohms pass is
the meter's milliamp through the winding and both brush contacts, after a
lead zero with the leads shorted. The stall pass is the board's current and
terminal taps, graded against the meter in stage 1, at the three 1S rails
in laps short enough to leave the copper cold; the meter is not in it, since
its packets come slower than a cold lap lasts.
"""
import csv
import sys
import time

from oscnb import b41t, bench, boards, capture, motors, osc

ANGLES = 12                  # the rig's bolt circle
OHMS_S = 40                  # the meter settles after the rotor moves; the notebook reads the tail
LEVELS = [3.3, 3.7, 4.2]     # rail volts, the 1S levels: the stall current stays under an amp
LAP_MS = 300                 # 100 % into the locked rotor, cold: under a joule per lap
REST_S = 20                  # between angles, the bench rule for a bare motor

route = next((a for a in sys.argv[1:] if a in ("ohms", "stall")), None)
if route is None:
    raise SystemExit(__doc__)
n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]
out = capture.dataset(f"{motor.key}__{board.key}", "rest-resistance", ["", *n]) / route
out.mkdir(parents=True, exist_ok=True)
print("motor", motor.key, "board", board.key, "route", route)

if route == "ohms":
    capture.prompt("Motor leads off the board. DMM probes shorted together, DMM on ohms, manual 220 ohm range, Bluetooth on.")
    with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="ohm"):
        marks = capture.Marks(out)
        marks.mark("leads shorted")
        capture.hold(OHMS_S / 2, "the lead zero")
        capture.prompt("Probes on the motor leads. Bar on the pinion at hole 0, clamp snug.")
        for k in range(ANGLES):
            if k:
                capture.prompt(f"Pins out, bar turned to hole {k}, pins in.")
            marks.mark(f"hole {k}")
            capture.hold(OHMS_S, f"hole {k}: the meter settling")
        marks.mark("end")
        marks.close()
else:
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
