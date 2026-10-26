"""Stage 0, power-resistors: the value of each load and its drift with power.

  uv run python stages/0-instruments/power-resistors/capture.py <bare|cap|series|leads> [N] [steps]

`steps` after the load skips the ohms readings and redoes only the driven part,
for when that part failed and the rest log is already on disk.

leads: run this first. No load. DMM on DC A, red probe in the A jack (20 A on the B41T+),
  red to DPS+, black to DPS-. The DPS sits in current limit and steps the current from
  0.25 to 2 A, 20 s each; its voltage readback is the drop over the leads and the DMM's
  ammeter at each current, which warms the leads the way the load runs do.
bare, cap, series: one load per run. The script asks for two wirings in turn.
  1. DMM on ohms. Probes shorted first, then across the load with the DPS leads off it.
  2. DMM on DC A, red probe in the A jack. DPS+ -> red probe, black probe -> load -> DPS-.
     The script then steps the DPS.
The script connects to the DMM once and keeps the connection for the whole run, since
the meter goes quiet for a while after every disconnect. It refuses to drive the DPS if
the DMM is on another function than the step needs. The DPS is opened only when it is
about to be driven; a port held open while waiting at a prompt stops answering.
"""
import sys

from oscnb import b41t, capture, dps150

HOLD_S = 25
AMPS = 2.0
LEAD_AMPS = [0.25, 0.5, 1.0, 1.5, 2.0]
VOLTS = {"series": [3, 6, 9, 12, 15, 18], "bare": [1, 2, 3, 4, 5, 6, 7, 8], "cap": [1, 2, 3, 4, 5, 6, 7, 8]}

load = sys.argv[1] if len(sys.argv) > 1 else sys.exit(__doc__)
steps_only = "steps" in sys.argv[2:]
n = [a for a in sys.argv[2:] if a.isdigit()]
out = capture.dataset("instruments", "power-resistors", ["", *n]) / load
out.mkdir(exist_ok=True)
marks = capture.Marks(out, append=steps_only)


def drive(hold_s, amps, volts):
    dps = dps150.open_supply()
    try:
        dps150.steps(dps, out, hold_s, amps, volts)
    finally:
        dps.close()


def drive_current(hold_s, volts, amps):
    dps = dps150.open_supply()
    try:
        dps150.current_steps(dps, out, hold_s, volts, amps)
    finally:
        dps.close()


with b41t.Meter() as meter:
    print("DMM connected")
    if load == "leads":
        capture.prompt("No load. DMM on DC A, red probe in the A jack, red to DPS+, black to DPS-.")
        with meter.log(out / "dmm.csv", expect="DC A"):
            marks.mark("steps")
            drive_current(20, 2.0, LEAD_AMPS)
    else:
        if not steps_only:
            capture.prompt("DMM on ohms, probes shorted together.")
            with meter.log(out / "rest.csv", expect="ohm"):
                marks.mark("shorted")
                capture.hold(20, "logging the shorted probes")
                capture.prompt(f"Probes across the {load} load, DPS leads off it.")
                marks.mark("load")
                capture.hold(30, "logging the load at rest")
        capture.prompt(f"DMM on DC A, red probe in the A jack. DPS+ -> red probe, black probe -> {load} load -> DPS-.")
        with meter.log(out / "dmm.csv", expect="DC A"):
            marks.mark("steps")
            drive(HOLD_S, AMPS, VOLTS[load])
marks.close()
print("wrote", out)
