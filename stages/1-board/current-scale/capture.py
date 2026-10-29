"""Stage 1, current-scale: the current chain against the meter. DC laps at
100 % duty on a known load at several supply levels, the DMM in series with
the load as the ammeter. Needs the board on the DPS, the load, the DMM and
the `osc` and `osc-lab` tools.

  uv run python stages/1-board/current-scale/capture.py <bare|series|rs1> [N]
                                                          writes telemetry/<board>/current-scale/capture-N/<pass>

bare, series: the DMM on DC A with the red probe in the A jack, the load in
  series with the meter across the board's motor terminals (MOT_A, meter,
  load, MOT_B). The levels run from a 1S pack's 3.3 V through USB to a 2S
  pack's 8 V; the bare 5 ohm load stops at 7.6 V to stay under its 12 W.
rs1: the DMM on DC V across the Rs1 shunt pads on a soldered twisted pair,
  the bare load on the terminals. The shunt's own value drops out of this
  route; it grades the amplifier and the ADC alone.
"""
import sys

from oscnb import b41t, bench, boards, capture, laps, osc

LOADS = {"bare": laps.LEVELS_BARE, "series": laps.LEVELS_ALL, "rs1": laps.LEVELS_BARE}

pas = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in LOADS else sys.exit(__doc__)
n = [a for a in sys.argv[2:] if a.isdigit()]
out = capture.dataset(boards.DEFAULT, "current-scale", ["", *n]) / pas
out.mkdir(exist_ok=True)
levels = LOADS[pas]
if pas == "rs1":
    capture.prompt("Bare load across the motor terminals. DMM on DC V across the Rs1 pads, Bluetooth on.")
    expect = "DC V"
else:
    capture.prompt(f"DMM on DC A, red probe in the A jack. MOT_A -> meter -> {pas} load -> MOT_B. Bluetooth on.")
    expect = "DC A"
bench.power_up(levels[0] + bench.DIODE_V)
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect=expect), osc.Limit(laps.LIMIT_COUNTS):
    laps.run(out, levels, pas)
bench.power_down()
print("wrote", out)
