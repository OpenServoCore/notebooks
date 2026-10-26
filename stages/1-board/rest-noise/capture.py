"""Stage 1, rest-noise: the raw channels with the bridge off, streamed at the
control rate, at a 1S, a USB and a 2S supply level. Needs the board on the DPS with the load
on its terminals (the bare 5 ohm resistor, which draws nothing at rest) and
the `osc` tool.

  uv run python stages/1-board/rest-noise/capture.py [N]    writes telemetry/<board>/rest-noise/capture-N
"""
from oscnb import bench, boards, capture, osc

LEVELS = [3.7, 5.0, 7.4]
REPEATS = 3
REST_MS = 10_000

out = capture.dataset(boards.DEFAULT, "rest-noise")
bench.power_up(LEVELS[0] + bench.DIODE_V)
for v in LEVELS:
    dps_read, vsys = bench.set_vsys(v)
    print(f"VSYS {v} V: rail reads {vsys:.3f} V, DPS {dps_read:.3f} V")
    for k in range(REPEATS):
        # a 5 % rung of 50 ms is the smallest the sweep records; the rest before it is the capture
        osc.sweep(out / f"rest-{v:g}V-{k}", ["5@50"], baseline_ms=REST_MS, rest_ms=200, dirs="fwd")
    osc.sweep(out / f"rest-{v:g}V-ntc", ["5@50"], baseline_ms=REST_MS, rest_ms=200, dirs="fwd",
              tel_chunks=osc.CHUNKS_NTC)
bench.power_down()
print("wrote", out)
