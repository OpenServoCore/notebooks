"""The two readers of oscnb.screens: the observer's largest still-hold
speed, on rows built with a known largest, and the worst run of pot
steps the sensor screen tests, on built rows and pinned on the rows of
record."""

import numpy as np
import pandas as pd
import pytest

from oscnb import results, screens

Q16 = 1 << 16
SCORECARD = "telemetry/mg90-d__dev-v006-2A-D/scorecard"


def loop_rows(speeds_by_seg):
    """Rows as a Loop capture streams them: each MEDIUM period's speed,
    count/s, as raw csQ16 over that period's ten FAST ticks."""
    seg, speed = [], []
    for s, speeds in speeds_by_seg.items():
        for v in speeds:
            seg += [s] * screens.DECIM_MED
            speed += [round(v * Q16)] * screens.DECIM_MED
    return pd.DataFrame({"seg": seg, "omega_hat_cps": speed})


def test_the_still_speed_is_the_largest_speed_of_the_holds_either_way():
    rows = loop_rows({0: [3.0, -40.5, 12.0], 1: [800.0, -900.0], 2: [-115.0, 60.0, 0.25]})
    # the move between them, segment 1, is no hold, and its speed is not read
    assert screens.still_speed_cps(rows, holds=[0, 2]) == 115.0
    assert screens.still_speed_cps(rows, holds=[0]) == 40.5
    assert screens.still_speed_cps(rows, holds=[1]) == 900.0


def test_a_hold_with_no_speed_is_refused():
    rows = loop_rows({0: [1.0]})
    rows["omega_hat_cps"] = np.nan
    with pytest.raises(ValueError, match="no row of the still holds carries omega_hat_cps"):
        screens.still_speed_cps(rows, holds=[0])
    with pytest.raises(ValueError, match="no row of the still holds"):
        screens.still_speed_cps(loop_rows({0: [1.0]}), holds=[3])


def pot_rows(pot, seg=0, start=0):
    """FAST-tick rows of one segment with the raw pot `pot`."""
    return pd.DataFrame({"seg": seg, "tick": np.arange(start, start + len(pot)), "pos": pot})


def test_the_step_run_is_the_worst_level_successive_rail_period_steps_all_pass():
    # a quiet pot at 2000, and at RAIL phase 3 the samples of five periods in a row read 2009, 1997, 2004,
    # 1994 and 2024
    pot = np.full(200, 2000)
    for k, level in enumerate((2009, 1997, 2004, 1994, 2024)):
        pot[3 + 10 * (5 + k)] = level
    rows = pot_rows(pot)
    # steps of 9, 12, 7, 10, 30 and, back to 2000, 24: the worst single step, the worst level two in a row
    # pass and the worst four in a row pass
    assert screens.step_run(rows, 1) == 30
    assert screens.step_run(rows, 2) == 24
    assert screens.step_run(rows, 4) == 7
    assert screens.step_run(rows) == 7


def test_a_run_never_spans_a_hole_in_the_ticks_or_two_segments():
    pot = np.full(80, 2000)
    pot[0::10] = [2000, 2020, 2040, 2060, 2080, 2100, 2120, 2140]
    rows = pot_rows(pot)
    assert screens.step_run(rows) == 20
    # a hole of one tick in the fourth period leaves three steps of 20 on each side: no run of four passes
    # 20, and the quiet phases read 0
    holed = rows.drop(index=35).reset_index(drop=True)
    assert (screens.step_run(holed), screens.step_run(holed, 3)) == (0, 20)
    # the same rows cut into two segments at the hole
    cut = rows.assign(seg=np.where(rows["tick"] < 35, 0, 1))
    assert (screens.step_run(cut), screens.step_run(cut, 3)) == (0, 20)


def test_too_few_steps_read_nothing():
    assert screens.step_run(pot_rows(np.full(30, 2000))) is None
    assert screens.step_run(pot_rows(np.full(41, 2000))) == 0


@pytest.mark.parametrize("capture, run, worst", [
    # the worst 4-step run over the repo's closed-loop rows of record, 19 rungs and 62.9 M rows, 15 counts on 5 V,
    # the statistic the analysis of the screens read over its 199.4 M rows
    ("12a-pos-table-live/run-1/follow", 4, 15),
    # the worst single step there, 29, and the worst pair, 24 (the analysis: 29 and 25)
    ("00-baseline/run-3/follow", 1, 29),
    ("00-baseline/run-1/follow", 2, 24),
])
def test_the_step_runs_of_record(capture, run, worst):
    rows = pd.read_csv(results.ROOT / SCORECARD / capture / "track.csv", usecols=["seg", "tick", "pos"])
    assert screens.step_run(rows, run) == worst
