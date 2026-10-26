"""Stage 1, opamp-temperature: the current chain's gain and zero against
board temperature. DC laps against the meter at each of four plateaus of the
filament dryer, the board NTC riding in every stream for the temperature.
Needs the board in the dryer on the DPS, the bare load, the DMM and the
`osc` tool. Run ntc-curve first, so the NTC reading is a temperature.

  uv run python stages/1-board/opamp-temperature/capture.py [ammeter|rs1] [N]
                                                          writes telemetry/<board>/opamp-temperature/capture-N

ammeter (default): the DMM on DC A in series with the load, outside the dryer.
rs1: the DMM on DC V across the Rs1 pads on a soldered twisted pair, the
  amplifier's own input, so the shunt's value drops out.
"""
import sys

from oscnb import b41t, bench, boards, capture, laps, osc

LEVELS = [3.7, 4.5, 5.5, 7.2, 7.6]
PLATEAUS = ["room", "warm", "hot", "room again"]

route = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ("ammeter", "rs1") else "ammeter"
n = [a for a in sys.argv[1:] if a.isdigit()]
out = capture.dataset(boards.DEFAULT, "opamp-temperature", ["", *n])
if route == "rs1":
    capture.prompt("Bare load on the motor terminals. DMM on DC V across the Rs1 pads, Bluetooth on.")
    expect = "DC V"
else:
    capture.prompt("DMM on DC A, red probe in the A jack. MOT_A -> meter -> bare load -> MOT_B, meter outside "
                   "the dryer. Bluetooth on.")
    expect = "DC A"
(out / "route.txt").write_text(route + "\n")
bench.power_up(LEVELS[0] + bench.DIODE_V)
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect=expect), osc.Limit(laps.LIMIT_COUNTS):
    marks = capture.Marks(out)
    for p in PLATEAUS:
        hint = {"room": "Lid open, dryer off, board at room.",
                "warm": "Dryer at about 45 C, lid closed, settled.",
                "hot": "Dryer at about 60 C, settled.",
                "room again": "Dryer off, lid open, board back near room."}[p]
        capture.prompt(f"{hint} Enter runs the laps for the {p} plateau.")
        marks.mark(p)
        laps.run(out / p.replace(" ", "-"), LEVELS, "bare")
    marks.mark("end")
    marks.close()
bench.power_down()
print("wrote", out)
