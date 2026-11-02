"""Stage 2, rest-resistance, the meter's routes for the teardown's brush path.
The stall pass itself is the `capture` plan of `python -m oscnb.sitting
locked`, declared in `plan.py` beside this file. These two are hand
procedures with the meter logging over Bluetooth: the motor off the board,
unpowered, the probes on its leads.

  uv run python stages/2-motor/rest-resistance/capture.py ohms [N]    writes telemetry/<motor>__<board>/rest-resistance/ohms-N
  uv run python stages/2-motor/rest-resistance/capture.py shake [N]   writes telemetry/<motor>__<board>/rest-resistance/shake-N

`ohms` reads the brush path with the meter. The motor sits
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
import sys

from oscnb import b41t, boards, capture, motors

ANGLES = 12                  # the rig's bolt circle
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
motor = motors.selected()
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

raise SystemExit("ohms or shake; the stall pass is `python -m oscnb.sitting locked`")
