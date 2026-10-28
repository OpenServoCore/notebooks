"""Stage 1, opamp-temperature: the current chain's gain and zero against
board temperature. DC laps against the meter at each of four plateaus of a
controlled heat source, the same one ntc-curve used, the board NTC riding in
every stream for the temperature, and a rest stream with the chip's 1.2 V
reference before each plateau's laps, so VDD is read at every temperature. Needs the board on the heater on the DPS,
the bare load and the DMM off the heater, and the `osc` tool. Run ntc-curve first, so the NTC reading is a temperature.

  uv run python stages/1-board/opamp-temperature/capture.py [ammeter|rs1] [N]
                                                          writes telemetry/<board>/opamp-temperature/capture-N

ammeter (default): the DMM on DC A in series with the load, off the heater.
rs1: the DMM on DC V across the Rs1 pads on a soldered twisted pair, the
  amplifier's own input, so the shunt's value drops out.
"""
import sys

from oscnb import b41t, bench, boards, capture, laps, osc

LEVELS = [3.7, 4.5, 5.5, 7.2, 7.6]
REST_V = 5.5          # the rail for the rest stream, well over the regulator's dropout
REST_MS = 2000
PLATEAUS = ["room", "warm", "hot", "room again"]

route = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ("ammeter", "rs1") else "ammeter"
n = [a for a in sys.argv[1:] if a.isdigit()]
out = capture.dataset(boards.DEFAULT, "opamp-temperature", ["", *n])
if route == "rs1":
    capture.prompt("Bare load on the motor terminals. DMM on DC V across the Rs1 pads, Bluetooth on.")
    expect = "DC V"
else:
    capture.prompt("DMM on DC A, red probe in the A jack. MOT_A -> meter -> bare load -> MOT_B, load and meter "
                   "off the heater. Bluetooth on.")
    expect = "DC A"
(out / "route.txt").write_text(route + "\n")
bench.power_up(LEVELS[0] + bench.DIODE_V)
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect=expect), osc.Limit(laps.LIMIT_COUNTS):
    marks = capture.Marks(out)
    for p in PLATEAUS:
        hint = {"room": "Heater off, board at room.",
                "warm": "Heater at about 45 C, settled.",
                "hot": "Heater at about 60 C, settled.",
                "room again": "Heater off, board back near room."}[p]
        capture.prompt(f"{hint} Enter runs the laps for the {p} plateau.")
        marks.mark(p)
        folder = out / p.replace(" ", "-")
        bench.set_vsys(REST_V)
        # the bridge off for two seconds with vref_raw in every row: VDD at this plateau.
        # a 5 % rung of 50 ms is the smallest the sweep records; the rest before it is the capture
        osc.sweep(folder / "rest", ["5@50"], baseline_ms=REST_MS, rest_ms=200, dirs="fwd", tel_chunks=osc.CHUNKS_REST)
        laps.run(folder, LEVELS, "bare")
    marks.mark("end")
    marks.close()
bench.power_down()
print("wrote", out)
