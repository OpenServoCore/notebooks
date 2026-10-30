"""Stage 2, rest-resistance: the motor's terminal resistance at rest, over the
rotor's angle, from the board's own current and terminal taps. Needs the
bare motor in the rig with its leads soldered to the tabs, the wrench bar,
the board on the DPS and the `osc` and `osc-lab` tools; no meter.

  uv run python stages/2-motor/rest-resistance/capture.py [N]         writes telemetry/<motor>__<board>/rest-resistance/capture-N/stall
  uv run python stages/2-motor/rest-resistance/capture.py ohms [N]    writes telemetry/<motor>__<board>/rest-resistance/ohms-N
  uv run python stages/2-motor/rest-resistance/capture.py shake [N]   writes telemetry/<motor>__<board>/rest-resistance/shake-N

The pass steps the bar through the twelve holes of the rig's bolt circle,
prompting at each, so a reading belongs to a rotor angle. At each hole it
drives 100 % laps into the locked rotor at the three 1S rails, both
directions, in laps short enough to leave the copper cold. The taps and the
current chain were graded against the meter in stage 1.

`ohms` is the meter's route, for the teardown's brush path. The motor sits
off the board, unpowered, long enough to be at room, and the meter's
milliamp leaves it there, so a thermocouple taped on the can, read before
and after the pass, is the copper's temperature. Between those it reads the
lead zero with the probes shorted and then the motor's leads at their free
ends, the tabs being out of reach in the rig, at each of the twelve holes, 40 s a hole for the meter to settle after the rotor
moves. The meter has one input, so the dial turns by hand at each prompt,
and the 220 ohm range is held by hand so every window reads at 0.01 ohm.
The teardown reads its copper with the same meter and thermocouple, so
their offsets cancel in the difference. The motor's own leads are in the
reading, as they are in the stall pass's, and the teardown's `terminals`
reads them on their own.

`shake` stays at one hole and reads the meter through several settles. A
knock to the rig moves the brushes' contact, which then settles where it
lands, so a steady window says the contact has settled but not where. The
spread between settles is the meter route's repeatability at one seat.
"""
import csv
import sys
import time

from oscnb import b41t, bench, boards, capture, motors, osc

ANGLES = 12                  # the rig's bolt circle
LEVELS = [3.3, 3.7, 4.2]     # rail volts, the 1S levels: the stall current stays under an amp
LAP_MS = 300                 # 100 % into the locked rotor, cold: under a joule per lap
REST_S = 20                  # between angles, the bench rule for a bare motor
OHMS_S = 40                  # the meter settles after the rotor moves; the notebook reads the tail
SETTLES = 6                  # knocks at one seat, each read once settled
SETTLE_S = 30
ZERO_S = 20
TEMP_S = 60
OHMS = "DMM on ohms, manual 220 ohm range."
TEMP = "DMM on temperature (C)."


def window(meter, marks, out, name, expect, seconds):
    with meter.log(out / f"{name.replace(' ', '-')}.csv", expect=expect):
        marks.mark(name)
        capture.hold(seconds, name)


n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
board = boards.BOARDS[boards.DEFAULT]

if "shake" in sys.argv[1:]:
    out = capture.dataset(f"{motor.key}__{board.key}", "rest-resistance", ["", *n], name="shake")
    print("motor", motor.key, "board", board.key, "route shake")
    capture.prompt(f"Motor leads OFF the board. Probes shorted together. {OHMS} Bluetooth on.")
    marks = capture.Marks(out)
    with b41t.Meter() as meter:
        window(meter, marks, out, "zero", "ohm", ZERO_S)
        capture.prompt("Probe clips on the two motor leads' free ends. Bar on the pinion at any hole, pins in. Hands off.")
        window(meter, marks, out, "settle 0", "ohm", SETTLE_S)
        for k in range(1, SETTLES + 1):
            capture.prompt("Knock the rig's base sideways once, nothing along the shaft, then hands off.")
            window(meter, marks, out, f"settle {k}", "ohm", SETTLE_S)
    marks.mark("end")
    marks.close()
    print("wrote", out)
    sys.exit()

if "ohms" in sys.argv[1:]:
    out = capture.dataset(f"{motor.key}__{board.key}", "rest-resistance", ["", *n], name="ohms")
    print("motor", motor.key, "board", board.key, "route ohms")
    capture.prompt("Motor leads OFF the board, the motor unpowered for at least ten minutes. Thermocouple tip taped "
                   f"on the can. {TEMP} Bluetooth on.")
    marks = capture.Marks(out)
    with b41t.Meter() as meter:
        window(meter, marks, out, "room c1", "C", TEMP_S)
        capture.prompt(f"Probes shorted together. {OHMS}")
        window(meter, marks, out, "zero", "ohm", ZERO_S)
        capture.prompt("Probe clips on the two motor leads' free ends, where they plug into the board. "
                       "Bar on the pinion at hole 0, clamp snug.")
        for k in range(ANGLES):
            if k:
                capture.prompt(f"Pins out, bar turned to hole {k}, pins in.")
            window(meter, marks, out, f"hole {k}", "ohm", OHMS_S)
        capture.prompt(f"Probes off the tabs and shorted together. {OHMS}")
        window(meter, marks, out, "zero end", "ohm", ZERO_S)
        capture.prompt(TEMP)
        window(meter, marks, out, "room c2", "C", TEMP_S)
    marks.mark("end")
    marks.close()
    print("wrote", out)
    sys.exit()

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
