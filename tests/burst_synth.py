"""A synthetic `osc-lab burst` on an R-L load, for the burst tests.

The ADC runs free, shunt and taps alternating in a frame of four, every
STEP_TICKS from the counter's phase at the first sample. Before the step the
bridge rests; from the first trough after it, the bridge drives the rail
across the load for the slow-decay window around each crest, its edges at
the lags given, and shorts the load between windows. The current is the
exact R-L answer tick by tick, the shunt reads it while the bridge drives
and the zero otherwise, and the tap difference reads the rail while it
drives, through a terminal line.
"""
import numpy as np
import pandas as pd

from oscnb import bursts as bu
from oscnb.scales import Scales

ZERO = 1205
BIAS = 1000
COUNTS_PER_A = 1100.0
TERM_OFFSET = 4.0
COUNTS_PER_V = 248.0
LAGS = {"on": 45.0, "tap_on": 40.0, "off": 20.0, "settle": 60.0}


def scales():
    """A Scales whose lines and lags the synthetic burst is written through."""
    timing = {}
    keys = {"on": "on_edge_lag", "tap_on": "tap_on_edge_lag",
            "off": "off_edge_lag", "settle": "shunt_settle"}
    for name, key in keys.items():
        for rail in ("7v4", "3v7"):
            timing[f"{key}_{rail}"] = {"value": LAGS[name]}
    return Scales(
        board="synthetic",
        current_scale={"counts_per_a": {"value": COUNTS_PER_A},
                       "drive_offset_counts": {"value": 0.0}},
        terminal_chain={"terminal_counts_per_v": {"value": COUNTS_PER_V},
                        "terminal_offset_counts": {"value": TERM_OFFSET}},
        sample_timing=timing)


def burst(board, step_q15=8192, r_ohm=5.0, l_h=1e-3, rail_v=5.0,
          n_samples=2400, step_index=400, start_cnt=738, start_dir=0):
    """(frame, current at every tick, drive at every tick, tick of sample 0)."""
    arr = board.pwm_arr
    period = 2 * arr
    half = int(board.drive_ticks(step_q15))
    k = np.arange(n_samples)
    phi0 = start_cnt if start_dir == 0 else period - start_cnt
    tick = phi0 + bu.STEP_TICKS * k + bu.APERTURE_CLK * bu.TICKS_PER_ADC_CLK
    ticks = np.arange(int(np.ceil(tick[-1])) + 1)
    first = period * np.ceil(tick[step_index] / period)
    phase = (ticks - arr) % period
    phase = np.where(phase >= arr, phase - period, phase)
    drive = ((ticks >= first) & (phase >= -half + LAGS["tap_on"])
             & (phase < half + LAGS["off"]))
    dt = 1 / (bu.TICKS_PER_US * 1e6)
    decay = np.exp(-r_ohm * dt / l_h)
    current = np.zeros(len(ticks))
    for j in range(1, len(ticks)):
        target = rail_v / r_ohm if drive[j - 1] else 0.0
        current[j] = current[j - 1] * decay + target * (1 - decay)
    at = np.floor(tick).astype(int)
    i_s, on_s = current[at], drive[at]
    code = np.empty(n_samples)
    shunt = k % 2 == 0
    tap_a = k % 4 == 1
    tap_b = k % 4 == 3
    code[shunt] = ZERO + np.where(on_s, COUNTS_PER_A * i_s, 0.0)[shunt]
    volts = np.where(on_s, rail_v, 0.0)
    code[tap_a] = (BIAS + TERM_OFFSET + COUNTS_PER_V * volts)[tap_a]
    code[tap_b] = BIAS
    frame = pd.DataFrame({"k": k, "code": np.round(code).astype(int)})
    meta = {"pre_q15": 0, "step_q15": step_q15, "step_index": step_index,
            "start_cnt": start_cnt, "pwm_arr": arr, "start_dir": start_dir,
            "chans": 3, "frame_len": 4, "vmotor_bias": BIAS}
    for key, value in meta.items():
        frame[key] = value
    return frame, current, drive, phi0
