import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stages" / "2-motor" / "motor-model"))
import motor_model as mm  # noqa: E402

from oscnb import coast, parallel, ripple  # noqa: E402

TICK_S = 1 / 48e6


def always_on(n):
    return np.zeros(n, dtype=np.int64)


def test_rl_rise_matches_the_closed_form():
    r, l, v = 4.4, 0.5e-3, 5.0
    n = 4 * mm.P
    _t0, _sh, ser, _x = mm.simulate(dict(rs=r, l0=l), dict(v_on=v, r_on=0.0), modes=always_on(n), t0=0)
    t = (np.arange(n) + 1) * TICK_S
    want = v / r * (1 - np.exp(-t * r / l)) * 1e3
    assert ser == pytest.approx(want, rel=1e-6, abs=1e-9)


def test_conductance_carries_g_times_the_volts_across_l():
    # the series current less the inductance's own current is G times the volts left across L after R
    r, g, v = 4.4, 2e-3, 5.0
    for n in (1, 10, 100):
        _t0, _sh, ser, x = mm.simulate(dict(rs=r, l0=0.5e-3, g=g), dict(v_on=v, r_on=0.0), modes=always_on(n), t0=0)
        i_s = ser[-1] * 1e-3
        assert i_s - x[0] == pytest.approx(g * (v - r * i_s), rel=1e-9)
    # and at the edge, with almost nothing in L yet, that is close to V / (R + 1 / G)
    assert ser[0] * 1e-3 == pytest.approx(v / (r + 1 / g), rel=0.05)


def test_amplifier_is_a_first_order_lag():
    tau, gain = 2.32, 1.0034
    y = mm.amp(np.ones(10), tau, gain)
    a = np.exp(-1 / tau)
    assert y == pytest.approx(gain * (1 - a ** (np.arange(10) + 1)), rel=1e-12)


@pytest.mark.parametrize("g", [0.0, 2.3e-3])
def test_memory_rules_agree_in_fast_decay(g):
    # the current returns to zero every period, so counting from the last reversal and keeping Rayleigh memory read
    # the same branch, to well under the shunt's 1 mA rest noise
    fold = dict(decay="fast", half=int(mm.BOARD.drive_ticks(int(0.25 * 32768))), on_lag=mm.FAST_ON, off_lag=mm.FAST_OFF,
                v_on=4.9, r_on=0.5, v_ret=-5.0, r_ret=0.5, sign=1)
    p = dict(rs=4.4, l0=0.5e-3, cr=2.9, a1=0.07, g=g, lb=0.24e-3, gb=0.16)
    _t, sh1, _s, _x = mm.simulate(dict(p, mem=1), fold)
    _t, sh2, _s, _x = mm.simulate(dict(p, mem=2), fold)
    assert np.abs(sh1 - sh2).max() < 0.05
    # slow decay never stops the current, so there the two rules part
    slow = dict(fold, decay="slow", on_lag=mm.SLOW_ON, off_lag=mm.SLOW_OFF, v_brk=0.0, r_brk=0.3)
    _t, sl1, _s, _x = mm.simulate(dict(p, mem=1), slow)
    _t, sl2, _s, _x = mm.simulate(dict(p, mem=2), slow)
    assert np.abs(sl1 - sl2).max() > 1.0


def test_steady_state_meets_the_target_mean():
    p = dict(rs=4.455, l0=0.5e-3, mem=2)
    s = mm.steady(p, 30, 4.9, 60.0)
    assert s["mean"] == pytest.approx(60.0, abs=0.01)
    assert s["emf"] > 0


def test_run_parallel_comes_from_the_library():
    # the second-motor modules call mm._call and set mm.run_parallel
    assert mm.run_parallel is parallel.run_parallel
    assert mm._call is parallel.call


def test_the_coast_reads_come_from_the_library():
    assert mm.spin_coast_pairs is coast.pairs
    assert mm.bridge_rows is coast.bridge
    assert mm.cut_row is coast.cut_row and mm.open_rows is coast.open_rows
    assert mm.read_window is ripple.read_window
    assert mm.line_freq is ripple.line_freq
    assert mm.count_cycles is ripple.count_cycles
    assert (mm.COAST_FROM, mm.TARGET_MS, mm.TURN_V) == (20, 10, 0.05)
