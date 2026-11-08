"""A sitting's manifest read back: the probes keyed by where they were read,
and a dataset's sweep table with the probe before each sweep."""
import numpy as np
import pandas as pd

from oscnb import manifest

PAIR_DIR = "telemetry/mg90-x__board-x"
LADDER = f"{PAIR_DIR}/ladder/capture-1"
ROWS = [
    (100.0, "start", "servo mg90-x, board board-x, state servo"),
    (101.0, "probe", "center: pos 2014, ntc_raw 2002, vbus_raw 859"),
    (102.0, "dataset", f"capture: {LADDER}"),
    (103.0, "probe", f"{LADDER}/cycles/0: pos -3, ntc_raw 2000, vbus_raw 858"),
    (104.5, "probe", f"{LADDER}/cycles/1: pos 12, ntc_raw 1999, vbus_raw 857"),
    (106.0, "probe",
     f"{PAIR_DIR}/ladder/rail-1/cycles/0: pos 5, ntc_raw 1998, vbus_raw 856"),
    (107.0, "rail", "5.0 V: reads 5.001 V, DPS 5.250 V"),
]


def write(tmp_path):
    path = tmp_path / "servo-1.csv"
    pd.DataFrame(ROWS, columns=["t_host", "event", "detail"]).to_csv(
        path, index=False)
    return path


def test_probes_are_keyed_by_where_they_were_read(tmp_path):
    probes = manifest.manifest_probes(write(tmp_path))
    assert list(probes) == ["center", f"{LADDER}/cycles/0",
                            f"{LADDER}/cycles/1",
                            f"{PAIR_DIR}/ladder/rail-1/cycles/0"]
    assert probes[f"{LADDER}/cycles/0"] == {
        "t_probe": 103.0, "pos": -3, "ntc_raw": 2000, "vbus_raw": 858}
    assert type(probes["center"]["vbus_raw"]) is int


def test_with_probes_merges_the_dataset_s_probes_only(tmp_path):
    probes = manifest.manifest_probes(write(tmp_path))
    cycles = pd.DataFrame({"k": [0, 1, 2],
                           "folder": ["cycles/0", "cycles/1", "cycles/2"]})
    table = manifest.with_probes(cycles, probes, LADDER)
    assert table.columns.tolist() == ["k", "folder", "t_probe", "pos",
                                      "ntc_raw", "vbus_raw"]
    assert table["k"].tolist() == [0, 1, 2]
    assert table["pos"].tolist()[:2] == [-3.0, 12.0]
    assert table["t_probe"].tolist()[:2] == [103.0, 104.5]
    assert table["vbus_raw"].dtype == float
    assert np.isnan(table.loc[2, ["t_probe", "pos", "ntc_raw",
                                  "vbus_raw"]].astype(float)).all()
