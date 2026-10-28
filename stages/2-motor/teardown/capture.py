"""Stage 2, teardown: the copper's resistance against its own temperature,
on the rotor out of its can. Needs the rotor with two leads soldered to two
commutator tabs, the thermocouple tip pressed into a coil under tape, both
on a controlled heat source (the printer bed), and the DMM; no board.

  uv run python stages/2-motor/teardown/capture.py [N]    writes telemetry/<motor>/teardown/capture-N

The meter has one input, so at each plateau it reads ohms for a minute and
then the thermocouple for a minute, the dial turned by hand at the prompt.
The heater runs at two settings and then off, like ntc-curve; the masses,
the photos and the counts of the teardown go into the notebook by hand.
"""
import sys

from oscnb import b41t, capture, motors

PLATEAUS = ["room", "warm", "hot", "room again"]
WINDOW_S = 60

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
out = capture.dataset(motor.key, "teardown", ["", *n])
print("motor", motor.key)
capture.prompt("Short the two leads together, DMM on ohms, Bluetooth on: the lead zero is read first.")
marks = capture.Marks(out)
with b41t.Meter() as meter:
    with meter.log(out / "zero.csv", expect="ohm"):
        marks.mark("lead zero")
        capture.hold(WINDOW_S / 2, "the leads shorted")
    capture.prompt("Leads on the commutator tabs, the thermocouple tip in a coil, the rotor on the heater, heater off.")
    for p in PLATEAUS:
        hint = {"room": "At room.",
                "warm": "Set the heater to about 45 C and wait for the thermocouple to settle, then",
                "hot": "Set the heater to about 60 C and wait for it to settle, then",
                "room again": "Switch the heater off and wait for the rotor to come back near room, then"}[p]
        capture.prompt(f"{hint} DMM on ohms.")
        with meter.log(out / f"{p.replace(' ', '-')}-ohm.csv", expect="ohm"):
            marks.mark(f"{p} ohm")
            capture.hold(WINDOW_S, f"{p}: ohms")
        capture.prompt("DMM on temperature (C).")
        with meter.log(out / f"{p.replace(' ', '-')}-c.csv", expect="C"):
            marks.mark(f"{p} C")
            capture.hold(WINDOW_S, f"{p}: the thermocouple")
marks.mark("end")
marks.close()
print("wrote", out)
