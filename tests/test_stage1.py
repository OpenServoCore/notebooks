import csv
import json

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, bursts, laps, ntc, tel
import sweep_synth as synth

B = boards.BOARDS[boards.DEFAULT]


def test_board_conversions_match_the_firmware_constants():
    assert B.counts_per_a == pytest.approx(1108.5, abs=0.1)
    assert B.v_term_per_count * 1e3 == pytest.approx(4.028, abs=0.001)
    assert B.i_floor_pct == pytest.approx(64 * 100 / 1200)
    # board D's own NTC reading against the firmware's conversion of it
    assert B.ntc_c_beta(1942) == pytest.approx(27.43, abs=0.1)


def test_board_from_meta_and_check(tmp_path):
    synth.write(tmp_path)
    df, meta = tel.load(tmp_path, B)
    assert boards.from_meta(meta) is B
    meta["sense"]["gain_milli"] = 15000
    with pytest.raises(ValueError):
        B.check_meta(meta, "x")
    assert boards.from_meta(meta) is None


def test_tel_rungs_recover_the_lap_current_and_a_dropped_frame(tmp_path):
    synth.write(tmp_path, rail_v=7.4, r_load=5.09, climb_rows=220, drop_at=(2, 500))
    df, meta = tel.load(tmp_path)
    r = tel.rungs(df)
    assert list(r["dir"]) == [1, -1]
    zero = tel.baseline(df)["current_raw"].median()
    amps = B.amps(r["current_raw"], zero)
    assert amps == pytest.approx([7.4 / 5.09] * 2, rel=0.01)
    assert (r["n"] <= 3000 - 220).all()  # the climb is left out
    g = tel.gaps(df)
    assert g.to_dict() == {2: 16}


def test_laps_table_joins_the_meter(tmp_path):
    out = tmp_path
    (out / "laps").mkdir()
    rows = []
    for k, v in enumerate([4.5, 5.5]):
        synth.write(out / "laps" / str(k), rail_v=v - 0.25)
        rows.append([k, "bare", v, v, 1000 + 10 * k, 1000 + 10 * k + 8, f"laps/{k}"])
    with open(out / "levels.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["k", "load", "volts_set", "volts_read", "t_start", "t_end", "folder"])
        w.writerows(rows)
    # the meter reads 0 at rest, +0.8 while the first direction drives, -0.8 for the second
    t_host = np.arange(1000, 1020, 0.5)
    phase = (t_host - 1000) % 10
    value = np.where((phase >= 1) & (phase < 4), 0.8, np.where((phase >= 5) & (phase < 8), -0.8, 0.0))
    dmm = pd.DataFrame({"t_host": t_host, "value": value})
    t = laps.table(out, B, dmm)
    assert len(t) == 4
    assert (t["meter"].abs() == 0.8).all()
    assert (np.sign(t["meter"]) == t["dir"]).all()
    assert (t["meter_n"] >= 5).all()     # the end margin trims the last reading of the second group
    fit = laps.line(B.amps(t["current_raw"], t["zero"]), t["current_raw"] - t["zero"])
    assert fit["slope"] == pytest.approx(B.counts_per_a, rel=0.01)


def test_driven_groups_split_at_the_sign_flip():
    v = [0, 0.01, 0.8, 0.81, 0.79, 0, -0.8, -0.8, 0.02, 0.5, 0.5]
    g = laps.driven_groups(v)
    assert [list(x) for x in g] == [[0.8, 0.81, 0.79], [-0.8, -0.8], [0.5, 0.5]]
    assert laps.driven_groups([]) == [] and laps.driven_groups([0, 0]) == []


def test_ntc_quadratic_round_trip():
    raw = np.arange(1200, 2800, 50)
    t = B.ntc_c_beta(raw)
    t_ref, k1, k2 = ntc.fit_quadratic(raw, t, 2048)
    back = ntc.firmware_cc(raw, 2048, t_ref, k1, k2) / 100
    assert np.abs(back - t).max() < 0.5
    beta = ntc.fit_beta(B.ntc_ohm(raw), t, B.ntc_r25_ohm)
    assert beta == pytest.approx(B.ntc_beta, rel=1e-3)


def test_burst_fold_places_the_window_on_the_crest(tmp_path):
    # a burst whose shunt reads high inside the drive window and the zero outside it
    arr, n = 1200, 4000
    m = {"pre_q15": 0, "step_q15": 8192, "step_index": 400, "start_cnt": 100, "pwm_arr": arr, "start_dir": 0,
         "restore_dir": 0, "vbus_raw": 1800, "bias": 1205, "chans": 3, "frame_len": 2, "vmotor_bias": 773,
         "pos": 2048, "seated": 1, "drive_polarity": 1}
    k = np.arange(n)
    ph = (m["start_cnt"] + bursts.STEP_TICKS * k) % (2 * arr)
    half = int(B.drive_ticks(m["step_q15"]))
    inside = np.abs(ph + bursts.APERTURE_CLK * bursts.TICKS_PER_ADC_CLK - arr) <= half
    code = np.where((k % 2 == 0) & (k >= m["step_index"]) & inside, 1500, 1205)
    df = pd.DataFrame({"k": k, "code": code, **{c: [v] + [""] * (n - 1) for c, v in m.items()}})
    df.to_csv(tmp_path / "burst-0.csv", index=False)
    b = bursts.load(tmp_path)
    assert len(b) == 1
    t, y, meta = bursts.fold(b[0], arr, "sh")
    x, prof = bursts.profile(t, y, -arr, arr, 8)
    assert prof[np.abs(x) < half - 30].mean() == pytest.approx(1500, abs=5)
    assert prof[np.abs(x) > half + 30].mean() == pytest.approx(1205, abs=5)
    assert bursts.zero(b[0], arr) == 1205
