"""Stage 0, thermocouple: offset, slope and time constant of the thermocouple on
the DMM's temperature input. Needs a glass packed with crushed ice and just
enough water to fill the gaps, left to stand about five minutes until the ice
is wet and melting (freezer ice is below 0 C), and boiling water.

  uv run python stages/0-instruments/thermocouple/capture.py [ice] [N]

With no N the run goes into the first telemetry/instruments/thermocouple/capture-N
that holds no DMM log, so an earlier capture is never overwritten and a run that
failed to connect leaves nothing behind. `ice` runs the ice bath alone:
air 30 s, ice 240 s, air 30 s. Without it the run is air, ice, boil, air.
"""
import sys

from oscnb import b41t, capture

ice_only = "ice" in sys.argv[1:]
n = [a for a in sys.argv[1:] if a.isdigit()]
if not n:
    k = 1
    while (capture.ROOT / "telemetry/instruments/thermocouple" / f"capture-{k}" / "dmm.csv").exists():
        k += 1
    n = [str(k)]
out = capture.dataset("instruments", "thermocouple", ["", *n])
if (out / "dmm.csv").exists():
    sys.exit(f"{out} already holds a capture; pass another N")
print("writing", out)

capture.prompt("Thermocouple plugged in, DMM on temperature (C), Bluetooth on, tip in air.")
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="C"):
    marks = capture.Marks(out)
    marks.mark("air")
    capture.hold(30 if ice_only else 60, "logging air")
    capture.prompt("Press Enter at the moment the tip goes in among the ice.")
    marks.mark("ice")
    capture.hold(240 if ice_only else 180, "ice, tip among the ice, not in the water under it")
    if not ice_only:
        capture.prompt("Press Enter at the moment the tip goes into the boiling water.")
        marks.mark("boil")
        capture.hold(180, "boil, tip in the water and off the pot")
    capture.prompt("Press Enter as the tip comes back out into the air.")
    marks.mark("air2")
    capture.hold(30 if ice_only else 60, "air again")
marks.close()
print("wrote", out)
