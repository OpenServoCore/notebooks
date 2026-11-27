"""The two readers of oscnb.screens: the observer's largest still-hold
speed from 10 ms into each hold, on rows built with a known largest, and
the worst run of pot steps the sensor screen tests outside the pot holes,
on built rows and pinned on the rows of record. No capture of record
streams the observer's speed over a still hold, so the still speed is
pinned on built rows only."""

import numpy as np
import pandas as pd
import pytest

from oscnb import results, screens

Q16 = 1 << 16
SCORECARD = "telemetry/mg90-d__dev-v006-2A-D/scorecard"
# the MEDIUM periods of a hold's first 10 ms, the opening the still speed skips
OPENING = screens.STILL_SKIP_TICKS // screens.DECIM_MED


def loop_rows(speeds_by_seg):
    """Rows as a Loop capture streams them, one tick after another: each
    MEDIUM period's speed, count/s, as raw csQ16 over that period's ten
    FAST ticks."""
    seg, speed = [], []
    for s, speeds in speeds_by_seg.items():
        for v in speeds:
            seg += [s] * screens.DECIM_MED
            speed += [round(v * Q16)] * screens.DECIM_MED
    return pd.DataFrame({"seg": seg, "tick": np.arange(len(seg)), "omega_hat_cps": speed})


def test_the_still_speed_is_the_largest_speed_of_the_holds_either_way():
    quiet = [0.0] * OPENING
    rows = loop_rows({0: quiet + [3.0, -40.5, 12.0], 1: quiet + [800.0, -900.0], 2: quiet + [-115.0, 60.0, 0.25]})
    # the move between them, segment 1, is no hold, and its speed is not read
    assert screens.still_speed_cps(rows, holds=[0, 2]) == 115.0
    assert screens.still_speed_cps(rows, holds=[0]) == 40.5
    assert screens.still_speed_cps(rows, holds=[1]) == 900.0


def test_the_first_10_ms_of_each_hold_are_not_read():
    # a burst start's spike of 180 count/s in a hold's first MEDIUM period, then 60 count/s at most
    assert screens.STILL_SKIP_TICKS == 200
    opened = [180.0] + [5.0] * (OPENING - 1) + [60.0, -42.0, 31.0]
    assert screens.still_speed_cps(loop_rows({0: opened}), holds=[0]) == 60.0
    # the last period of the opening is still skipped, the first one after it is read
    late = [5.0] * (OPENING - 1) + [180.0, 60.0]
    assert screens.still_speed_cps(loop_rows({0: late}), holds=[0]) == 60.0
    read = [5.0] * OPENING + [180.0, 60.0]
    assert screens.still_speed_cps(loop_rows({0: read}), holds=[0]) == 180.0
    # each hold is skipped from its own first row: the second hold's opening spike is not read either
    rows = loop_rows({0: opened, 1: [0.0] * 30, 2: [-195.0] + [5.0] * (OPENING - 1) + [-48.0]})
    assert screens.still_speed_cps(rows, holds=[0, 2]) == 60.0
    assert screens.still_speed_cps(rows, holds=[2]) == 48.0


def test_a_hold_with_no_speed_is_refused():
    rows = loop_rows({0: [1.0] * (OPENING + 1)})
    rows["omega_hat_cps"] = np.nan
    with pytest.raises(ValueError, match="no row of the still holds from 10 ms on carries omega_hat_cps"):
        screens.still_speed_cps(rows, holds=[0])
    with pytest.raises(ValueError, match="no row of the still holds"):
        screens.still_speed_cps(loop_rows({0: [1.0] * (OPENING + 1)}), holds=[3])
    # a hold no longer than its opening has no row read
    with pytest.raises(ValueError, match="no row of the still holds"):
        screens.still_speed_cps(loop_rows({0: [250.0] * OPENING}), holds=[0])


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


def test_the_rows_inside_a_pot_hole_are_left_out():
    # a spin, 7 counts a RAIL period on every phase, and the dead low end of a worn track: the reading glides
    # from 129 to the floor at 4 by 21 or 22 counts a RAIL period, then rests there
    spin = 371 + np.arange(300) * 7 // 10
    glide = np.maximum(4, 129 - np.arange(300) * 215 // 100)
    rows = pd.concat([pot_rows(spin, seg=0), pot_rows(glide, seg=1, start=300)], ignore_index=True)
    assert screens.step_run(rows) == 21
    # with the hole of the dead low end, raw 4 to 121, the glide's samples are left out and the spin reads
    assert screens.step_run(rows, holes=[(4, 121)]) == 7
    # the glide alone: of each phase at most its first sample lies over the hole, and no step is left
    assert screens.step_run(rows[rows["seg"] == 1], holes=[(4, 121)]) is None
    # a hole away from the rows leaves them as they read
    assert screens.step_run(rows, holes=[(2500, 2600)]) == 21


def test_a_sample_in_a_hole_cuts_the_run_of_its_own_phase_alone():
    # steps of 20 on RAIL phase 0 from 2000, eight samples, the other phases quiet at 2500
    pot = np.full(80, 2500)
    pot[0::10] = 2000 + 20 * np.arange(8)
    alone = pot_rows(pot)
    # the hole takes phase 0's fourth sample, 2060, and leaves three steps after it: no run of four passes 20
    assert screens.step_run(alone, holes=[(2060, 2060)]) == 0
    assert screens.step_run(alone, 3, holes=[(2060, 2060)]) == 20
    # a hole's ends are both inside it
    assert screens.step_run(alone, holes=[(2050, 2059)]) == 20
    assert screens.step_run(alone, holes=[(2041, 2060)]) == 0
    # the same steps on phase 5 from 3000, whose samples lie outside the hole, still hold runs of four
    pot[5::10] = 3000 + 20 * np.arange(8)
    assert screens.step_run(pot_rows(pot), holes=[(2060, 2060)]) == 20


def test_a_hole_whose_ends_are_the_wrong_way_round_is_refused():
    with pytest.raises(ValueError, match="a pot hole runs from its lower end to its upper one"):
        screens.step_run(pot_rows(np.full(50, 2000)), holes=[(121, 4)])


@pytest.mark.parametrize("capture, run, holes, worst", [
    # the worst 4-step run over the repo's closed-loop rows of record, 19 rungs and 62.9 M rows, 15 counts on 5 V,
    # the statistic the analysis of the screens read over its 199.4 M rows; it sits on the wide code at 2047, at
    # 2065 / 2047 / 2063 / 2047 / 2062, and no pot hole is recorded for the pair
    ("12a-pos-table-live/run-1/follow", 4, (), 15),
    # a hole of the dead low end's kind, raw 4 to 121, lies off these rows and leaves the 15 as it stands
    ("12a-pos-table-live/run-1/follow", 4, ((4, 121),), 15),
    # the worst single step there, 29, and the worst pair, 24 (the analysis: 29 and 25)
    ("00-baseline/run-3/follow", 1, (), 29),
    ("00-baseline/run-1/follow", 2, (), 24),
])
def test_the_step_runs_of_record(capture, run, holes, worst):
    rows = pd.read_csv(results.ROOT / SCORECARD / capture / "track.csv", usecols=["seg", "tick", "pos"])
    assert screens.step_run(rows, run, holes) == worst
