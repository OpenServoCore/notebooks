"""Stage 1, terminal-chain: the terminal taps, the rail tap and the bias
node against the meter. The same DC laps as current-scale, with the DMM on
DC V at one node per pass. Needs the board on the DPS, the bare load, the
DMM and the `osc` and `osc-lab` tools.

  uv run python stages/1-board/terminal-chain/capture.py <load|tap_a|rail|rail-fine|vdd> [N]
                                                          writes telemetry/<board>/terminal-chain/capture-N/<node>

load: the meter across the load at the board's motor terminals, MOT_A to MOT_B.
tap_a: MOT_A to GND, the absolute terminal voltage one tap reads.
rail: VSYS to GND, the supply rail behind the feed diode, at rest and under the lap.
rail-fine: VSYS to GND, the same probes, no laps. The rail walked up in 20 mV steps
  from 3.7 to 8.0 V with a short rest stream at each, the DPS set and not
  trimmed, so the meter is the x axis and the rail tap the reading. Against
  the meter this is the tap's scale on a fine grid; its residual is the ADC's
  own linearity over the codes the tap crosses, 900 to 1850, which take in
  1024 and 1536 where the current channel reads a step under drive.
vdd: no meter. The chip's own 1.2 V reference streamed at rest at every level, no
  laps: VDD = 1.2 V x 4096 / vref_raw, the ADC reference every count is a share
  of, and whether it holds on a 1S rail.
"""
import sys

from oscnb import b41t, bench, boards, capture, laps, osc

NODES = {"load": "across the bare load, MOT_A to MOT_B", "tap_a": "MOT_A to GND",
         "rail": "VSYS to GND", "rail-fine": "VSYS to GND", "vdd": "no meter"}
LEVELS = laps.LEVELS_BARE
FINE_STEP_V = 0.02          # about 4 counts of the rail tap per step
FINE_RANGE_V = (3.7, 8.0)   # above the regulator's dropout, the whole 2S range; walked up only, a DPS step down resets the board
FINE_REST_MS = 250          # 5000 rows, the tap's mean to a few hundredths of a count
FINE_HOLD_S = 1.5           # the DPS settled and the meter logging at the new level before the stream

node = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in NODES else sys.exit(__doc__)
n = [a for a in sys.argv[2:] if a.isdigit()]
out = capture.dataset(boards.DEFAULT, "terminal-chain", ["", *n]) / node
out.mkdir(exist_ok=True)
if node == "vdd":
    import csv, time
    capture.prompt("Bare load on the motor terminals, no meter.")
    bench.power_up(LEVELS[0] + bench.DIODE_V)
    with open(out / "levels.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "load", "volts_set", "dps_read", "vsys_read", "t_start", "t_end", "folder"])
        for k, v in enumerate(LEVELS):
            dps_read, vsys = bench.set_vsys(v)
            print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
            t0 = time.time()
            osc.sweep(out / f"rest-{v:g}V", ["5@50"], baseline_ms=3000, rest_ms=200, dirs="fwd",
                      tel_chunks=osc.CHUNKS_REST)
            w.writerow([k, "none", v, f"{dps_read:.3f}", f"{vsys:.3f}", f"{t0:.3f}", f"{time.time():.3f}", f"rest-{v:g}V"])
elif node == "rail-fine":
    import csv, time
    capture.prompt(f"Bare load on the motor terminals. DMM on DC V, {NODES[node]}, Bluetooth on.")
    lo, hi = FINE_RANGE_V
    points = [round(lo + k * FINE_STEP_V, 3) for k in range(int(round((hi - lo) / FINE_STEP_V)) + 1)]
    bench.power_up(points[0] + bench.DIODE_V)
    with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC V"), open(out / "levels.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "load", "volts_set", "dps_read", "t_set", "t_start", "t_end", "folder"])
        for k, v in enumerate(points):
            t_set = time.time()
            dps_read = bench.set_volts(v + bench.DIODE_V, hold_s=FINE_HOLD_S)
            t0 = time.time()
            osc.sweep(out / f"rest-{v:.2f}V", ["5@50"], baseline_ms=FINE_REST_MS, rest_ms=200, dirs="fwd",
                      tel_chunks=osc.CHUNKS_REST)
            w.writerow([k, "none", f"{v:.2f}", f"{dps_read:.3f}", f"{t_set:.3f}", f"{t0:.3f}", f"{time.time():.3f}", f"rest-{v:.2f}V"])
            f.flush()
            print(f"{k + 1}/{len(points)}: DPS {dps_read:.3f} V for a rail near {v:.2f} V")
else:
    capture.prompt(f"Bare load on the motor terminals. DMM on DC V, {NODES[node]}, Bluetooth on.")
    bench.power_up(LEVELS[0] + bench.DIODE_V)
    with b41t.Meter() as meter, meter.log(out / "dmm.csv", expect="DC V"), osc.Limit(laps.LIMIT_COUNTS):
        laps.run(out, LEVELS, "bare")
bench.power_down()
print("wrote", out)
