"""Stage 1, sample-timing: where the crest samples land in a short drive
window, and from which duty each channel reads the drive honestly. A duty
ladder on the bare load in both directions and both decay modes, with the
100 % rung as the control, then free-running ADC bursts that draw the
current and the terminals inside one PWM period, then the slow ladder and
the bursts again on a 1S rail. Needs the board on the DPS, the bare load and
the `osc` tool; no meter.

  uv run python stages/1-board/sample-timing/capture.py [N]    writes telemetry/<board>/sample-timing/capture-N
"""
from oscnb import bench, boards, capture, laps, osc

VOLTS = 7.4
VOLTS_1S = 3.7
LADDER = [3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 25, 30, 40, 100]
BURSTS = [10, 15, 25, 38]

out = capture.dataset(boards.DEFAULT, "sample-timing")
capture.prompt("Bare load on the motor terminals, nothing else on them.")
bench.power_up(VOLTS + bench.DIODE_V)
dps_read, vsys = bench.set_vsys(VOLTS)
print(f"VSYS {VOLTS} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
with osc.Limit(laps.LIMIT_COUNTS):
    for decay in ("slow", "fast"):
        osc.sweep(out / f"ladder-{decay}", LADDER, window_ms=150, baseline_ms=1000, rest_ms=500, dirs="both",
                  decay=decay)
    osc.bursts(out / "bursts", BURSTS, repeats=3)
# the servo resets when the DPS steps down to the 1S level and comes back at
# its saved limit, so the wider one is set again after the step
dps_read, vsys = bench.set_vsys(VOLTS_1S)
print(f"VSYS {VOLTS_1S} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
with osc.Limit(laps.LIMIT_COUNTS):
    osc.sweep(out / "ladder-slow-1s", LADDER, window_ms=150, baseline_ms=1000, rest_ms=500, dirs="both", decay="slow")
    osc.bursts(out / "bursts-1s", BURSTS, repeats=3)
bench.power_down()
print("wrote", out)
