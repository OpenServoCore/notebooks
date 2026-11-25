"""The chip's 1.2 V reference as the servo keeps it, one boot reading with 0
for none: VDD from it, the rail at it, and the one read by name a capture
takes of it, while a recording that streamed it in every row still loads."""
import numpy as np
import pandas as pd

from oscnb import bench, boards, tel

BOARD = boards.BOARDS[boards.DEFAULT]


def test_vdd_is_no_figure_without_a_reading():
    assert BOARD.vdd_v(1491) == BOARD.vref_v * BOARD.adc_counts / 1491
    for none in (0, None, np.nan):
        assert np.isnan(BOARD.vdd_v(none))
    got = BOARD.vdd_v(pd.Series([1491, 0, np.nan]))
    assert got[0] == BOARD.vdd_v(1491) and np.isnan(got[1:]).all()


def test_the_rail_stands_on_the_nominal_vdd_without_a_reading():
    nominal = BOARD.rail_v(1500)
    for none in (None, 0, np.nan):
        assert BOARD.rail_v(1500, none) == nominal
    assert BOARD.rail_v(1500, 1491) == nominal * BOARD.vdd_v(1491) / (BOARD.vdd_mv / 1e3)
    both = BOARD.rail_v(np.array([1500, 1500]), [1491, 0])
    assert both[0] == BOARD.rail_v(1500, 1491) and both[1] == nominal


def test_a_capture_reads_the_reference_once_by_name(monkeypatch, tmp_path):
    reads = []
    regs = {"vbus_raw": 1500, "vref_raw": 0}

    def get(field):
        reads.append(field)
        return regs[field]

    monkeypatch.setattr(bench.osc, "get", get)
    assert bench.read_vsys(BOARD) == BOARD.rail_v(1500)
    regs["vref_raw"] = 1491
    assert bench.read_vsys(BOARD) == BOARD.rail_v(1500, 1491)
    assert reads == ["vbus_raw", "vref_raw"] * 2
    assert bench.record_vref(tmp_path) == 1491
    assert (tmp_path / "vref_raw.txt").read_text() == "1491\n"


def test_a_recording_loads_with_or_without_the_reference_column(tmp_path):
    rows = "seg,cmd_duty_q15,dir,tick,window_valid,pos,current,current_trough,duty_counts,vdiff,vbus," \
           "current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw,vref_raw,pos_lin\n"
    for name, vref in (("old", "1491"), ("new", "")):
        folder = tmp_path / name
        folder.mkdir()
        (folder / "meta.json").write_text("{}")
        (folder / "sweep.csv").write_text(rows + "".join(
            f"{s},0,1,{k + 40 * s},0,,,,0,,,1210,2000,2000,1500,2100,{vref},\n"
            for s in (0, 1) for k in range(40)))
        df, _ = tel.load(folder)
        r = tel.rungs(df, skip=0)
        assert ("vref_raw" in r) == (name == "old")
        if name == "old":
            assert BOARD.rail_v(df["vbus_raw"], df["vref_raw"]).mean() == BOARD.rail_v(1500, 1491)
        else:
            assert (BOARD.rail_v(df["vbus_raw"], df["vref_raw"]) == BOARD.rail_v(1500)).all()
