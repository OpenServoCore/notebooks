"""The DMM in series: a synthetic packet log of one cycle, two spins with a
spin-up packet and a packet straddling the cut each, rest packets between."""
import numpy as np
import pandas as pd
import pytest

from oscnb import meter

PACKET_S = 0.51
T_START, T_END = 100.0, 110.0
FORWARD_A, REVERSE_A = 0.050, -0.062
SPIN_UP_A = 0.140                     # the first packet, the rotor still rising
STRADDLE_SHARE = 0.4                  # the last packet, part spin, part coast


def spin(t0, amps, packets=6):
    t = t0 + PACKET_S * np.arange(packets)
    v = np.full(packets, amps)
    v[0] = np.sign(amps) * SPIN_UP_A
    v[-1] = amps * STRADDLE_SHARE
    return t, v


def log():
    t_fwd, v_fwd = spin(101.0, FORWARD_A)
    t_rev, v_rev = spin(106.0, REVERSE_A)
    t_rest = np.r_[np.arange(99.5, 101.0, PACKET_S),
                   np.arange(104.4, 106.0, PACKET_S),
                   np.arange(109.2, 112.0, PACKET_S)]
    v_rest = np.full(len(t_rest), 4e-4)
    outside = [T_START - 1.5, T_END + 1.5]
    frame = pd.DataFrame({"t_host": np.r_[t_fwd, t_rev, t_rest, outside],
                          "value": np.r_[v_fwd, v_rev, v_rest, 0.05, 0.05]})
    return frame.sort_values("t_host", ignore_index=True)


def test_spins_split_on_the_gap_and_drop_the_ends():
    found = meter.spins(log(), T_START, T_END)
    assert [s["dir"] for s in found] == [1, -1]
    assert found[0]["t0"] == pytest.approx(101.0)
    assert found[1]["t0"] == pytest.approx(106.0)
    for s, amps in zip(found, (FORWARD_A, REVERSE_A)):
        assert len(s["values"]) == 4                    # first and last out
        assert s["values"] == pytest.approx(abs(amps) * 1e3)
        assert len(s["t"]) == 4
        assert s["t"][0] == pytest.approx(s["t0"] + PACKET_S)


def test_by_direction_and_scale():
    found = meter.spins(log(), T_START, T_END)
    assert meter.by_direction(found) == pytest.approx(
        {1: FORWARD_A * 1e3, -1: -REVERSE_A * 1e3})
    amps = meter.by_direction(meter.spins(log(), T_START, T_END, scale=1.0))
    assert amps == pytest.approx({1: FORWARD_A, -1: -REVERSE_A})


def test_the_window_reads_a_margin_past_the_cycle():
    packets = meter.window(log(), T_START, T_END)
    assert packets["t_host"].min() > T_START - meter.MARGIN_S
    assert packets["t_host"].max() < T_END + meter.MARGIN_S
    assert len(meter.window(log(), T_START, T_END, margin_s=2.0)) == len(
        log())


def test_a_retried_spin_replaces_the_earlier_or_pools():
    t1, v1 = spin(101.0, FORWARD_A)
    t2, v2 = spin(106.0, 2 * FORWARD_A)
    retry = pd.DataFrame({"t_host": np.r_[t1, t2], "value": np.r_[v1, v2]})
    found = meter.spins(retry, T_START, T_END)
    assert [s["dir"] for s in found] == [1, 1]
    assert meter.by_direction(found) == pytest.approx({1: 2 * FORWARD_A * 1e3})
    pooled = meter.pooled_by_direction(found)
    assert pooled[1] == pytest.approx(1.5 * FORWARD_A * 1e3)
    assert np.isnan(pooled[-1])
