"""Stage 0, dmm: how late the DMM log lands against the host clock.

Setup: the DMM clipped across the DPS output terminals, set to DC V,
Bluetooth on. Nothing else on the DPS.

  uv run python stages/0-instruments/dmm/capture.py [N]    writes telemetry/instruments/dmm/capture-N
"""
from oscnb import b41t, capture, dps150

HOLD_S = 10
AMPS = 0.5
VOLTS = [3, 6, 3, 9, 3, 12, 3, 6, 3, 9, 3, 12]

out = capture.dataset("instruments", "dmm")
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC V"):
    dps = dps150.open_supply()
    try:
        dps150.steps(dps, out, HOLD_S, AMPS, VOLTS)
    finally:
        dps.close()
print("wrote", out)
