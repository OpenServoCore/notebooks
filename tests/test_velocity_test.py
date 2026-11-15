"""The velocity test's leg reading on synthetic rows."""
import numpy as np
import pandas as pd
import pytest

from oscnb import velocity_test as vt

HZ = 20_000
CPD = 20.0
CPA = 1000.0
ZERO = 1210


def segment(seg, goal, n, speed_share):
    """`n` rows of one goal: the pot on a line at `speed_share` of the
    commanded speed, the duty flipping sign every 100 rows, the shunt
    read on every other row at 100 mA over the zero; a goal of 0 is
    undriven and reads the zero."""
    tick = np.arange(n)
    driven = goal != 0
    return pd.DataFrame({
        "seg": seg, "goal": goal, "tick": tick,
        "window_valid": (tick % 2 == 0).astype(int) if driven else 0,
        "pos": 2000 + speed_share * goal * tick / HZ,
        "current_raw": ZERO + (100 if driven else 0),
        "duty_counts": np.where((tick // 100) % 2 == 0, 300, -300) if driven else 0})


def test_a_capture_reads_its_full_legs_alone_over_their_middle():
    g = 1248
    rows = pd.concat([segment(0, 0, 6000, 0), segment(1, g, 12000, 0.95), segment(2, -g, 24000, 0.95),
                      segment(3, g, 24000, 0.95), segment(4, -g, 24000, 0.95), segment(5, g, 12000, 0.95),
                      segment(6, 0, 6000, 0)], ignore_index=True)
    legs = vt.capture_legs(rows, HZ, lambda raw: np.asarray(raw, float) / CPD, CPD, CPA)
    # the half legs, the hold and the goal of 0 are left out
    assert [leg["seg"] for leg in legs] == [2, 3, 4]
    middle_s = (vt.MIDDLE[1] - vt.MIDDLE[0]) * 23999 / HZ
    for leg, sign in zip(legs, (-1, 1, -1)):
        assert leg["commanded_deg_s"] == pytest.approx(sign * g / CPD)
        assert leg["ratio"] == pytest.approx(0.95)
        # a straight line has no ripple
        assert leg["ripple_pct"] == pytest.approx(0, abs=1e-9)
        # a flip every 100 rows is 200 a second, give or take one flip over the middle
        assert leg["reversals_per_s"] == pytest.approx(HZ / 100, abs=1 / middle_s)
        # 100 counts over the zero of the undriven rows is 100 mA, read on half the rows
        assert leg["current_rms_ma"] == pytest.approx(100 / np.sqrt(2), rel=1e-3)
