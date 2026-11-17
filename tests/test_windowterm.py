import numpy as np
import pandas as pd
import pytest

from oscnb import boards
from oscnb import windowterm as wt

BOARD = boards.BOARDS["dev-v006-2A-D"]
ARR, HZ = 1200, 20_000


def test_the_settle_gain_is_the_boards_word_by_8_tick_band_and_1_past_the_table():
    q15, start = BOARD.i_settle_q15, BOARD.i_settle_start_ticks
    ticks = [0, start, start + 7, start + 8, start + 8 * len(q15) - 1, start + 8 * len(q15), 1200]
    assert BOARD.settle_q15(ticks).tolist() == [q15[0], q15[0], q15[0], q15[1], q15[-1], 1 << 15, 1 << 15]
    assert BOARD.settle_gain([start + 8 * len(q15)]).tolist() == [1.0]
    with pytest.raises(ValueError, match="no settle gain table"):
        boards.BOARDS["dev-v006-2A-E"].settle_q15([64])


def test_a_pulse_train_that_never_switches_is_one_charge_from_rest():
    # every pulse a whole period wide, no edge lag: the winding charges from its first pulse's start,
    # half a period before the first crest, toward 1 / R per volt with L / R
    L, R, sample = np.array([0.8e-3]), np.array([4.0]), 31.0
    h = np.full(12, float(ARR))
    got = wt.pulse_train(h, L, R, 0.0, 0.0, sample, ARR, HZ, 0.3)[:, 0]
    elapsed = (np.arange(12) * 2 * ARR + ARR + sample) * wt.tick_seconds(ARR, HZ)
    assert got == pytest.approx((1 - np.exp(-elapsed * R[0] / L[0])) / R[0], rel=1e-12)
    # a first pulse that starts late starts the charge late by as many ticks
    late = wt.pulse_train(h, L, R, 0.0, 0.0, sample, ARR, HZ, 0.3, first_late=10.0)[:, 0]
    assert late == pytest.approx((1 - np.exp(-(elapsed - 10 * wt.tick_seconds(ARR, HZ)) * R[0] / L[0])) / R[0],
                                 rel=1e-12)


def test_a_pulse_train_fills_a_grid_of_l_and_r_in_one_call_and_a_zero_row_only_decays():
    L, R = np.meshgrid([0.6e-3, 0.9e-3], [3.5, 4.5], indexing="ij")
    h = np.array([68.0, 73.0, 0.0, 0.0])
    grid = wt.pulse_train(h, L, R, 38.7, 13.7, 31.0, ARR, HZ, 0.3)
    assert grid.shape == (4, 2, 2)
    one = wt.pulse_train(h, L[1:2, 0:1], R[1:2, 0:1], 38.7, 13.7, 31.0, ARR, HZ, 0.3)
    assert grid[:, 1, 0] == pytest.approx(one[:, 0, 0], rel=1e-15)
    # with no drive the winding only decays through R and the brake loop, a period apart
    k = 2 * ARR * wt.tick_seconds(ARR, HZ) * (R + 0.3) / L
    assert grid[3] == pytest.approx(grid[2] * np.exp(-k), rel=1e-12)


def test_an_onset_is_gated_on_a_still_entry_one_sign_the_pot_and_the_common_ramp():
    base = {"pre_rows": 3, "pre_vd_max": 2.0, "moved": 1.0, "same_sign": True, "seq": (68, 73)}
    info = pd.DataFrame([base, {**base, "pre_rows": 0}, {**base, "pre_vd_max": 7.0}, {**base, "moved": 7.0},
                         {**base, "same_sign": False}, {**base, "seq": (70, 75)}, base])
    gated, ramp = wt.gate(info)
    assert ramp == (68, 73) and gated.ok.tolist() == [True, False, False, False, False, False, True]
