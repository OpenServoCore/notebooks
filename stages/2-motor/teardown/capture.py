"""Stage 2, teardown: the copper's resistance against its own temperature,
on the rotor out of its can. Needs the rotor with two leads soldered to two
commutator tabs, the thermocouple tip pressed into a coil under tape, both
on a controlled heat source (the printer bed), and the DMM; no board.

  uv run python stages/2-motor/teardown/capture.py [N]    writes telemetry/<motor>/teardown/capture-N

The meter has one input, so at each plateau it reads the thermocouple for a
minute, then ohms for a minute, then the thermocouple again, the dial turned
by hand at each prompt, so the temperature brackets the resistance. The lead
zero is read with the leads shorted before the run and again after it, since
the leads warm on the bed. The ohms range is set by hand to 220 ohm so every
window reads at the same resolution. The heater runs at two settings and then
off, like ntc-curve; the masses, the photos and the counts of the teardown
go into the notebook by hand.
"""
import sys

from oscnb import b41t, capture, motors

PLATEAUS = ["room", "warm", "hot", "room again"]
WINDOW_S = 60

n = [a for a in sys.argv[1:] if a.isdigit()]
motor = motors.MOTORS[motors.DEFAULT]
out = capture.dataset(motor.key, "teardown", ["", *n])
print("motor", motor.key)
OHMS = "DMM on ohms, manual 220 ohm range."
TEMP = "DMM on temperature (C)."


def window(meter, marks, name, expect, seconds=WINDOW_S):
    with meter.log(out / f"{name.replace(' ', '-')}.csv", expect=expect):
        marks.mark(name)
        capture.hold(seconds, name)


capture.prompt(f"Short the two leads together. {OHMS} Bluetooth on: the lead zero is read first.")
marks = capture.Marks(out)
with b41t.Meter() as meter:
    window(meter, marks, "zero", "ohm", WINDOW_S / 2)
    capture.prompt(f"Leads on the commutator tabs, the thermocouple tip in a coil, the rotor on the heater, heater off. {TEMP}")
    for p in PLATEAUS:
        hint = {"room": "At room.",
                "warm": "Set the heater to about 45 C and wait for the thermocouple to settle, then",
                "hot": "Set the heater to about 60 C and wait for it to settle, then",
                "room again": "Switch the heater off and wait for the rotor to come back near room, then"}[p]
        capture.prompt(f"{hint} {TEMP}")
        window(meter, marks, f"{p} c1", "C")
        capture.prompt(OHMS)
        window(meter, marks, f"{p} ohm", "ohm")
        capture.prompt(TEMP)
        window(meter, marks, f"{p} c2", "C")
    capture.prompt(f"Leads off the tabs and shorted together. {OHMS}")
    window(meter, marks, "zero end", "ohm", WINDOW_S / 2)
marks.mark("end")
marks.close()
print("wrote", out)
