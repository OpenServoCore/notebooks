"""The sweep plumbing the notebooks share: a sweep's rest zeros, its driven
rows and segment edges on the synthetic sweep, a chain's segments end to
end and its flip rows, and a capture's sweep table asked for a folder by
number."""
import numpy as np
import pandas as pd
import pytest
import sweep_synth as synth

from oscnb import cycles, tel

BASELINE_ROWS = 2000
RUNG_ROWS = 3000
LEAD_ROWS = 30
COAST_ROWS = 400
TAIL_ROWS = 12


@pytest.fixture
def sweep(tmp_path):
    folder = synth.write(tmp_path / "sweep", baseline_rows=BASELINE_ROWS,
                         rung_rows=RUNG_ROWS, lead_rows=LEAD_ROWS,
                         coast_rows=COAST_ROWS, tail_rows=TAIL_ROWS)
    df, _ = tel.load(folder)
    return df


def test_the_synth_sweep_chains_a_coast_to_each_rung(sweep):
    segs = sweep.groupby("seg")
    assert list(segs.groups) == [0, 1, 2, 3, 4]
    coast = segs.get_group(2)
    assert (coast["cmd_duty_q15"] == 0).all()
    assert (coast["dir"] == segs.get_group(1)["dir"].iloc[0]).all()
    idle = np.flatnonzero(coast["duty_counts"].to_numpy() == 0)
    assert idle[0] == TAIL_ROWS
    tap_diff = (coast["vmotor_a"] - coast["vmotor_b"]).to_numpy()
    assert tap_diff[TAIL_ROWS + 1] > tap_diff[-1] + 100


def test_segment_edges(sweep):
    first_coast = BASELINE_ROWS + RUNG_ROWS
    second_rung = first_coast + COAST_ROWS
    expected = [BASELINE_ROWS, first_coast, second_rung,
                second_rung + RUNG_ROWS]
    edges = tel.segment_edges(sweep)
    assert edges.tolist() == expected
    assert edges.tolist() == (
        np.flatnonzero(np.diff(sweep["seg"].to_numpy()) != 0) + 1).tolist()


def test_drive_rows_start_at_the_first_driven_row(sweep):
    rung = sweep[sweep["seg"] == 1]
    assert tel.drive_start(rung) == LEAD_ROWS
    driven = tel.drive_rows(rung)
    assert len(driven) == RUNG_ROWS - LEAD_ROWS
    assert driven["duty_counts"].iloc[0] != 0
    idiom = int(np.flatnonzero(rung["duty_counts"].to_numpy() != 0)[0])
    assert driven.equals(rung.iloc[idiom:])
    assert tel.drive_start(sweep[sweep["seg"] == 2]) == 0
    with pytest.raises(ValueError):
        tel.drive_start(sweep[sweep["seg"] == 0])


def test_rest_zeros_are_the_baseline_medians(sweep):
    zero_i, zero_e = tel.rest_zeros(sweep)
    rest = tel.baseline(sweep)
    assert zero_i == rest["current_raw"].median()
    assert zero_e == (rest["vmotor_a"] - rest["vmotor_b"]).median()
    assert abs(zero_i - 1205) <= 1
    assert abs(zero_e) <= 1


def test_direction_frame_chains_segments_end_to_end(sweep):
    frame, starts = tel.direction_frame(sweep, (3, 4))
    assert starts == {3: 0, 4: RUNG_ROWS}
    assert len(frame) == RUNG_ROWS + COAST_ROWS
    assert frame.index.tolist() == list(range(len(frame)))
    assert frame["seg"].iloc[RUNG_ROWS - 1] == 3
    assert frame["seg"].iloc[RUNG_ROWS] == 4
    assert frame["tick"].equals(pd.concat(
        [sweep[sweep["seg"] == s]["tick"] for s in (3, 4)],
        ignore_index=True))


def test_flip_row_is_the_first_row_showing_the_held_duty():
    duty = [-240, -240, 300, 330, 360, 360, -360, 360, 360, 360]
    row, held = tel.flip_row(pd.DataFrame({"duty_counts": duty}))
    assert (row, held) == (4, 360)


def test_cycles_load_names_each_sweep_folder(tmp_path):
    capture = tmp_path / "capture-1"
    capture.mkdir()
    table = pd.DataFrame({"k": [0, 1, 2], "volts_set": [5.0, 5.0, 7.4],
                          "folder": ["cycles/0", "cycles/1", "cycles/2"]})
    table.to_csv(capture / "cycles.csv", index=False)
    table.to_csv(capture / "levels.csv", index=False)
    sweeps = cycles.load(capture)
    assert sweeps.folder(2) == capture / "cycles/2"
    assert sweeps["folder"].tolist() == table["folder"].tolist()
    high = sweeps[sweeps["volts_set"] == 7.4]
    assert high.folder(2) == capture / "cycles/2"
    sweeps["trim_mv"] = 0.0
    assert sweeps.copy().folder(0) == capture / "cycles/0"
    laps = cycles.load(capture, "levels.csv")
    assert laps.folder(1) == capture / "cycles/1"
    with pytest.raises(IndexError):
        sweeps.folder(9)
