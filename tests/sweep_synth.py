"""A synthetic `osc sweep` recording on a resistor, for the stage 1 tests."""
import json
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import boards

HEADER = "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,pos_lin"


def write(folder, board=None, rungs=((32767, 1), (32767, -1)), rail_v=7.4, r_load=5.09, zero=1205,
          baseline_rows=2000, rung_rows=3000, climb_rows=0, noise=1.3, ntc=1942, seed=0, drop_at=None):
    """Baseline then rungs at 100 % (or any Q15) on a resistor: the current
    is V/R, the taps split the rail, the rail sags 0.1 V under the lap."""
    board = board or boards.BOARDS[boards.DEFAULT]
    rng = np.random.default_rng(seed)
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    rows = []
    tick = 1000
    vb = board.bias_nom_counts

    def row(seg, cmd, d, duty_counts, amps, va, vbn, rail, n):
        return {"seg": seg, "cmd_duty_q15": cmd, "dir": d, "tick": tick + n, "window_valid": 1, "pos": "",
                "current": "", "current_trough": "", "duty_counts": duty_counts, "vdiff": "", "vbus": "",
                "current_raw": int(round(zero + amps * board.counts_per_a + rng.normal(0, noise))),
                "vmotor_a": int(round(va + rng.normal(0, 0.9))), "vmotor_b": int(round(vbn + rng.normal(0, 0.9))),
                "vbus_raw": int(round(rail / board.v_rail_per_count + rng.normal(0, 0.8))),
                "ntc_raw": int(ntc), "pos_lin": ""}

    for n in range(baseline_rows):
        rows.append(row(0, 0, 0, 0, 0.0, vb, vb, rail_v, n))
    tick += baseline_rows
    for seg, (q15, d) in enumerate(rungs, 1):
        duty = abs(q15) / 32767
        ticks = int(board.drive_ticks(q15))
        amps = rail_v * duty / r_load
        rail = rail_v - 0.1 * duty
        hi = vb + amps * r_load / board.v_term_per_count * 1.0
        va, vbn = (hi, vb + 20) if d > 0 else (vb + 20, hi)
        for n in range(rung_rows):
            if drop_at is not None and seg == drop_at[0] and n == drop_at[1]:
                tick += 16
            k = min(n, climb_rows) / climb_rows if climb_rows else 1.0
            rows.append(row(seg, q15, d, d * int(ticks * k) if climb_rows else d * ticks, amps, va, vbn, rail, n))
        tick += rung_rows
    pd.DataFrame(rows, columns=HEADER.split(",")).to_csv(folder / "sweep.csv", index=False)
    meta = {"model": 257, "fw": 65, "tick_hz": 20000, "pwm_period_ticks": board.pwm_arr, "vbus_counts": 1800,
            "sense": board.sense, "window_ms": 150, "baseline_ms": int(baseline_rows / 20), "static_load": True,
            "host_git_sha": "test"}
    (folder / "meta.json").write_text(json.dumps(meta))
    return folder
