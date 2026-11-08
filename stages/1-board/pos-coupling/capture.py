"""Stage 1, pos-coupling: whether the position, the NTC and the terminal taps
move with the motor current, the duty or the rail when nothing mechanical
can move. The pot is replaced by a fixed divider on the POS connector, 3.3 k
from its 3V3 pin to POS1 and 2.2 k from POS1 to its GND pin, so the position
is a constant voltage from the pot's own source impedance and reads about
1640 counts. The load is the RL dummy, the bare 5 ohm resistor in series
with the two power inductors. Needs the board on the DPS, the divider, the
dummy and the `osc` and `osc-lab` tools. No meter.

  uv run python stages/1-board/pos-coupling/capture.py [N]    writes telemetry/<board>/pos-coupling/capture-N

`sweeps.csv` names each sweep's folder, rail, decay and repeat. Each sweep
drives 10 to 100 % both directions on the dummy, 300 ms a rung with a
torque-off rest between and a torque-off baseline before the grid, and the
row carries pos, the rail, the NTC, the crest shunt and both terminal taps.
The duty from the row's segment. Three rails and both decay modes, three
repeats each, so the current runs from about 70 mA to 1.4 A and each duty
meets it at three currents. A shift that follows the current whatever the
duty and the decay, the same sign both directions and present at 100 %
where the bridge stops switching, is the return current's drop through the
ground copper. One that follows the duty or the switching leg and is gone
at 100 % is coupling from the bridge. The NTC and the taps answer the same
question for their own nodes.

The dummy takes 1.4 A and 10 W on the 100 % rung at 7.4 V, 300 ms at a
time, under the resistor's 12 W and the inductors' saturation.
"""
import csv
import time

from oscnb import bench, boards, capture, laps, osc, results

LEVELS = [3.7, 5.0, 7.4]
DUTIES = [10, 20, 40, 60, 80, 100]
WINDOW_MS = 300
REPEATS = 3
POS_BAND = (1500, 1800)     # the divider's 1640 at rest, with room for 1 % parts
# pos, then vbus_raw, ntc_raw, current_raw, vmotor_a, vmotor_b, 12 B. The duty
# and the trough stay out: the segment names the duty, and the trough reads
# nothing at the crest's node.
ROW = "0x238+2,0x23c+10"

out = capture.dataset(boards.DEFAULT, "pos-coupling")
capture.prompt("The divider on the POS connector, 3.3 k from 3V3 to POS1 and 2.2 k from "
               "POS1 to GND. MOT_A -> the RL dummy -> MOT_B.")
bench.power_up(LEVELS[0] + bench.DIODE_V)
pos = osc.get("pos")
if not POS_BAND[0] <= pos <= POS_BAND[1]:
    bench.power_down()
    raise SystemExit(f"pos reads {pos}, not the divider's {POS_BAND[0]} to {POS_BAND[1]}")
print(f"pos reads {pos} at rest")
with open(out / "sweeps.csv", "w", newline="") as f, osc.Limit(laps.LIMIT_COUNTS):
    w = csv.writer(f, lineterminator="\n")
    w.writerow(["k", "volts_set", "dps_read", "vsys_read", "decay", "repeat", "t_start", "t_end", "folder"])
    k = 0
    for v in LEVELS:
        dps_read, vsys = bench.set_vsys(v)
        print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
        for r in range(REPEATS):
            for decay in ("slow", "fast"):
                folder = f"sweeps/{k}"
                t0 = time.time()
                osc.sweep(out / folder, DUTIES, window_ms=WINDOW_MS, baseline_ms=1000, rest_ms=500,
                          dirs="both", decay=decay, tel_chunks=ROW)
                w.writerow([k, v, f"{dps_read:.3f}", f"{vsys:.3f}", decay, r, f"{t0:.3f}", f"{time.time():.3f}", folder])
                f.flush()
                k += 1
bench.power_down()
print("wrote", results.rel(out))
