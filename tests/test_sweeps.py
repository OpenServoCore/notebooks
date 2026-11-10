"""A pair's sweeps on a hand-made tree: the index's time order and gaps,
the bridge state of a row, the run table and the row store, and the
truth a coast's tail gives."""
import json

import numpy as np
import pandas as pd

from oscnb import boards, sweeps, zero

FS = 20_000
BOARD = boards.BOARDS["dev-v006-2A-D"]
PAIR = "mg90-x__dev-v006-2A-D"
COLS = ("seg,cmd_duty_q15,dir,tick,window_valid,pos,current_trough,"
        "duty_counts,current_raw,vmotor_a,vmotor_b").split(",")


def seg(d, n, cmd=0, duty=0, raw=1200, trough=1200, va=773, vb=773,
        valid=0, pos=2000, after_s=0.0):
    """One segment: `after_s` of clock no row covers before it."""
    return dict(dir=d, n=n, cmd=cmd, duty=duty, raw=raw, trough=trough,
                va=va, vb=vb, valid=valid, pos=pos, after_s=after_s)


def write_sweep(folder, t0, segs, schedule=("30@300", "coast:300")):
    folder.mkdir(parents=True)
    rows, table, t = [], [], t0
    for k, s in enumerate(segs):
        t += s["after_s"]
        table.append(dict(seg=k, dir=s["dir"], cmd_duty_q15=s["cmd"],
                          streamed=1, samples=s["n"], t_start=t,
                          t_end=t + s["n"] / FS))
        t += s["n"] / FS
        for i in range(s["n"]):
            rows.append([k, s["cmd"], s["dir"], i, s["valid"], s["pos"] + i,
                         s["trough"], s["dir"] * s["duty"], s["raw"],
                         s["va"], s["vb"]])
    pd.DataFrame(rows, columns=COLS).to_csv(folder / "sweep.csv",
                                            index=False)
    pd.DataFrame(table).to_csv(folder / "segments.csv", index=False)
    meta = {"tick_hz": FS, "pwm_period_ticks": BOARD.pwm_arr,
            "sense": BOARD.sense, "schedule": list(schedule),
            "rows_dropped": 0, "stall": False, "static_load": True}
    (folder / "meta.json").write_text(json.dumps(meta))
    return t


def tree(tmp_path):
    """Three sweeps, written out of time order; the ladder one has a
    coast long enough for a tail and an unstreamed gap."""
    pair = tmp_path / PAIR
    ends = {}
    ends["ladder"] = write_sweep(pair / "ladder/set-1", 100.0, [
        seg(0, 6000),
        seg(1, 400, cmd=9830, duty=360, raw=1300, trough=1210, va=1500,
            valid=1),
        seg(1, 2100, raw=1240, trough=1235, after_s=0.01),
        seg(-1, 300, cmd=-32767, duty=1200, raw=1100, trough=1190,
            vb=1600, valid=1)])
    ends["c0"] = write_sweep(pair / "run-up/up-1/cycles/0", 10.0, [
        seg(0, 500), seg(1, 300, cmd=32767, duty=1200, valid=1)])
    ends["c1"] = write_sweep(pair / "run-up/up-1/cycles/1", 50.0, [
        seg(0, 500), seg(-1, 200, cmd=-16384, duty=600, valid=0)])
    return pair, ends


def test_index_lists_sweeps_in_time_order_with_rows_and_gaps(tmp_path):
    pair, ends = tree(tmp_path)
    ix = sweeps.index(pair)
    assert list(ix["folder"]) == ["run-up/up-1/cycles/0",
                                  "run-up/up-1/cycles/1", "ladder/set-1"]
    assert list(ix["experiment"]) == ["run-up", "run-up", "ladder"]
    assert list(ix["dataset"]) == ["up-1", "up-1", "set-1"]
    assert list(ix["rows"]) == [800, 700, 8800]
    assert list(ix["segments"]) == [2, 2, 4]
    assert np.isnan(ix["gap_s"][0])
    np.testing.assert_allclose(ix["gap_s"][1:], [50.0 - ends["c0"],
                                                 100.0 - ends["c1"]])
    assert list(ix["duty_pct_max"]) == [100.0, 50.0, 100.0]
    assert ix["schedule"][0] == "30@300 coast:300"


def test_load_gives_each_row_its_bridge_state(tmp_path):
    pair, _ = tree(tmp_path)
    rows, meta, segs = sweeps.load(pair, "ladder/set-1", BOARD)
    by_seg = rows.groupby("seg")["bridge"].unique()
    assert list(by_seg[0]) == [sweeps.ASLEEP]
    assert list(by_seg[1]) == [sweeps.DRIVE]
    assert list(by_seg[2]) == [sweeps.COAST]
    assert list(by_seg[3]) == [sweeps.DRIVE]
    assert meta["tick_hz"] == FS and len(segs) == 4


def test_runs_break_on_bridge_segment_and_gap():
    bridge = np.array([0, 0, 1, 1, 2, 2, 2, 1])
    segn = np.array([0, 0, 1, 1, 1, 1, 2, 2])
    gap = np.array([0, 0, 0, 0, 0, 3, 0, 0])
    st, en = sweeps.run_starts(bridge, segn, gap)
    assert list(st) == [0, 2, 4, 5, 6, 7]
    assert list(en) == [2, 4, 5, 6, 7, 8]


def test_segment_gaps_count_the_clock_no_row_covers():
    segs = pd.DataFrame({"t_start": [0.0, 0.005, 0.02]})
    segn = np.repeat([0, 1, 2], 100)
    gap = sweeps.segment_gaps(segn, segs, FS)
    assert gap[100] == 0 and gap[200] == 200
    assert gap.sum() == 200


def test_row_store_lays_sweeps_end_to_end_with_their_runs(tmp_path):
    pair, _ = tree(tmp_path)
    before = sorted(p for p in tmp_path.rglob("*"))
    st = sweeps.row_store(pair, columns=sweeps.ZERO_COLUMNS + ("va",))
    assert sorted(p for p in tmp_path.rglob("*")) == before
    sw, runs, arr = st.sweeps, st.runs, st.arrays
    assert list(sw["i0"]) == [0, 800, 1500]
    assert list(sw["i1"]) == [800, 1500, 10300]
    assert len(arr["bridge"]) == 10300 and arr["va"].dtype == np.int16
    assert (st.tick_hz, st.pwm_arr) == (FS, BOARD.pwm_arr)
    lad = runs[runs["sweep"] == 2]
    assert list(lad["kind"]) == [sweeps.ASLEEP, sweeps.DRIVE, sweeps.COAST,
                                 sweeps.DRIVE]
    assert list(lad["i0"]) == [1500, 7500, 7900, 10000]
    assert list(lad["gap_before"]) == [0, 0, 200, 0]
    assert arr["pre_gap"][7900] == 200
    assert list(lad["duty"]) == [0, 360, 0, 1200]
    # a valid window: the driven tap through the board; none: the rail
    v = float(BOARD.term_v(1500, 773))
    assert lad["vbus"].iloc[1] == v
    assert sw["rail"][2] == np.median([v, float(BOARD.term_v(1600, 773))])
    c1 = runs[(runs["sweep"] == 1) & (runs["kind"] == sweeps.DRIVE)]
    assert np.isnan(sw["rail"][1]) and np.isnan(c1["vbus"].iloc[0])
    rail = arr["rail"][1500:10300]
    assert np.isnan(rail[:6000]).all() and (rail[6000:6400] == v).all()
    assert sw["base_n"][2] == 6000
    assert sw["base_raw"][2] == 1200.0 and sw["vb_counts"][2] == 773.0
    assert list(sw["ds"]) == ["up-1", "up-1", "set-1"]
    assert list(sw["sub"]) == ["cycles/0", "cycles/1", ""]


def test_row_store_cache_reads_back_what_it_built(tmp_path):
    pair, _ = tree(tmp_path)
    cache = tmp_path / "cache"
    a = sweeps.row_store(pair, cache_dir=cache)
    assert (cache / "store.json").exists()
    b = sweeps.row_store(pair, cache_dir=cache)
    assert isinstance(b.arrays["trough"], np.memmap)
    for k in sweeps.ZERO_COLUMNS:
        assert np.array_equal(a.arrays[k], b.arrays[k], equal_nan=True)
    pd.testing.assert_frame_equal(a.sweeps, b.sweeps)
    pd.testing.assert_frame_equal(a.runs, b.runs)


def test_truth_is_the_nearest_long_coasts_tail(tmp_path):
    pair, _ = tree(tmp_path)
    st = zero.with_truth(sweeps.row_store(pair))
    runs, sw = st.runs, st.sweeps
    coast = runs[(runs["sweep"] == 2) & (runs["kind"] == sweeps.COAST)]
    assert coast["tail_raw"].iloc[0] == 1240.0
    assert coast["tail_trough"].iloc[0] == 1235.0
    drive = runs[runs["kind"] == sweeps.DRIVE]
    assert list(drive[drive["sweep"] == 2]["truth"]) == [1240.0, 1240.0]
    assert drive[drive["sweep"] < 2]["truth"].isna().all()
    assert list(sw["n_tails"]) == [0, 0, 1]
    assert sw["truth"][2] == 1240.0 and np.isnan(sw["truth"][0])
