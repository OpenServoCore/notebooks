"""Stage 1, terminal-chain: the terminal taps, the rail tap and the bias
node against the meter. The same DC laps as current-scale, with the DMM on
DC V at one node per pass. Needs the board on the DPS, the bare load, the
DMM and the `osc` tool.

  uv run python stages/1-board/terminal-chain/capture.py <load|tap_a|rail|vdd> [N]
                                                          writes telemetry/<board>/terminal-chain/capture-N/<node>

load: the meter across the load at the board's motor terminals, MOT_A to MOT_B.
tap_a: MOT_A to GND, the absolute terminal voltage one tap reads.
rail: VSYS to GND, the supply rail behind the feed diode, at rest and under the lap.
vdd: the 3V3 test point to GND at rest at every level, no laps: the ADC reference every
  count is a share of, and whether it holds on a 1S rail.
"""
import sys

from oscnb import b41t, bench, boards, capture, laps, osc

NODES = {"load": "across the bare load, MOT_A to MOT_B", "tap_a": "MOT_A to GND",
         "rail": "VSYS to GND", "vdd": "the 3V3 test point to GND"}
LEVELS = laps.LEVELS_BARE

node = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in NODES else sys.exit(__doc__)
n = [a for a in sys.argv[2:] if a.isdigit()]
out = capture.dataset(boards.DEFAULT, "terminal-chain", ["", *n]) / node
out.mkdir(exist_ok=True)
capture.prompt(f"Bare load on the motor terminals. DMM on DC V, {NODES[node]}, Bluetooth on.")
bench.power_up(LEVELS[0] + bench.DIODE_V)
with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC V"):
    if node == "vdd":
        import csv, time
        with open(out / "levels.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["k", "load", "volts_set", "dps_read", "vsys_read", "t_start", "t_end", "folder"])
            for k, v in enumerate(LEVELS):
                dps_read, vsys = bench.set_vsys(v)
                print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
                t0 = time.time()
                osc.sweep(out / f"rest-{v:g}V", ["5@50"], baseline_ms=5000, rest_ms=200, dirs="fwd")
                w.writerow([k, "none", v, f"{dps_read:.3f}", f"{vsys:.3f}", f"{t0:.3f}", f"{time.time():.3f}", f"rest-{v:g}V"])
    else:
        with osc.Limit(laps.LIMIT_COUNTS):
            laps.run(out, LEVELS, "bare")
bench.power_down()
print("wrote", out)
