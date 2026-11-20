"""The pot-side-force plumbing on synthetic inputs: the table and its
inverse, the camera angle mapped into the table's frame, the NaN-aware
centered mean, the signed winding current, a coast's zero and the found
data manifest."""
import json

import numpy as np
import pandas as pd
import pytest

from oscnb import potforce as pf
from oscnb import speed as sp

ARR = 1200
CHAIN = sp.Chain(a_per_count=1e-3, tap_counts_per_v=250.0,
                 tap_offset_counts=2.0, half_period_ticks=ARR,
                 lag_rail_v=(4.0, 8.0), lag_on_ticks=(40.0, 44.0),
                 lag_off_ticks=(12.0, 16.0), fs_hz=20_000)


def table():
    """A bent table on a 1 deg grid, rising everywhere."""
    a = np.arange(-160.0, -29.0, 1.0)
    c = 2000 + 19.0 * (a + 90) + 30 * np.sin(np.radians(3 * a))
    return pf.PotTable(a, c, np.gradient(c, a))


def test_the_tables_inverse_returns_the_angle_and_both_are_nan_outside():
    t = table()
    theta = np.array([-150.3, -90.0, -40.7])
    np.testing.assert_allclose(t.angle_at(t.counts_at(theta)), theta,
                               atol=1e-3)
    assert np.isnan(t.counts_at(-170.0)) and np.isnan(t.counts_at(-20.0))
    assert np.isnan(t.angle_at(t.counts[0] - 1))


@pytest.mark.parametrize("sign", (1, -1))
def test_map_angle_recovers_the_sign_and_the_offset_of_a_clip(sign):
    t = table()
    theta = np.linspace(-130, -50, 400)          # the table's frame
    offset = 93.27
    cam = (theta - offset) * sign                # the clip's own angle
    pos = t.counts_at(theta)
    s, c, rms = pf.map_angle(t, cam, pos)
    assert s == sign
    assert abs(c - offset) < 0.006 and rms < 0.2


def test_box_is_a_centered_mean_that_skips_nan_and_gives_up_on_sparse():
    x = np.arange(20.0)
    np.testing.assert_allclose(pf.box(x, 5)[2:-2], x[2:-2])
    y = x.copy()
    y[10] = np.nan
    assert pf.box(y, 5)[10] == pytest.approx(np.mean([8, 9, 11, 12]))
    z = np.full(20, np.nan)
    z[10] = 1.0                                  # 1 of 5 is a quarter or less
    assert np.isnan(pf.box(z, 5)[10])


def rows(duty, raw, valid=1, taps=(1500, 300)):
    n = len(duty)
    return pd.DataFrame({"duty_counts": duty, "current_raw": raw,
                         "current_trough": raw, "window_valid": valid,
                         "vmotor_a": np.full(n, taps[0]),
                         "vmotor_b": np.full(n, taps[1])})


def test_the_winding_current_is_signed_by_the_duty_and_less_the_window():
    duty = np.array([0, 0, 0, 600, -600, 600])
    raw = np.array([1210, 1212, 1214, 1312, 1312, 1312])
    df = rows(duty, raw, valid=np.array([1, 1, 1, 1, 1, 0]))
    g = 4e-3
    current, info = pf.winding_current(df, CHAIN, g)
    assert info["zero"] == 1212 and info["n_undriven"] == 3
    # 100 counts over the zero at 1 mA a count, less g (1 - D) V_on
    v_fwd = (1200 - 2.0) / 250.0                 # the + drive's taps
    v_rev = (-1200 - 2.0) / 250.0                # read in the - direction
    assert current[3] == pytest.approx(0.1 - g * 0.5 * v_fwd)
    assert current[4] == pytest.approx(-(0.1 - g * 0.5 * v_rev))
    assert np.isnan(current[5])                  # driven, no valid window
    assert (current[:3] == 0).all()


def test_a_coasts_zero_is_its_second_half_and_the_trough_the_drives():
    seg = np.r_[np.zeros(4), np.ones(6), np.full(8, 2)].astype(int)
    duty = np.r_[np.zeros(4), np.full(6, 500), np.zeros(8)]
    raw = np.r_[np.full(4, 900), np.full(6, 1300),
                np.full(4, 1200), np.full(4, 1210)]
    df = pd.DataFrame({"seg": seg, "duty_counts": duty, "current_raw": raw,
                       "current_trough": raw - 90})
    z_coast, trough = pf.coast_zero(df)
    assert z_coast == 1210                        # the baseline is not one
    assert trough == 1210
    no_coast = df[df["seg"] < 2]
    assert np.isnan(pf.coast_zero(no_coast)[0])


def test_a_found_data_manifest_without_its_keys_is_refused(tmp_path):
    doc = {"clips": {"c.mp4": {"original": "IMG.MOV", "sha256": "ab",
                               "frames": 10, "fps": 240.0,
                               "frames_tracked": [0, 9]}},
           "setup": "s", "image": {}, "loop": "l", "track_columns": "t",
           "lens": "x"}
    (tmp_path / "b").mkdir()
    (tmp_path / "b" / "manifest.json").write_text(json.dumps(doc))
    assert pf.manifest(tmp_path, "b")["loop"] == "l"
    del doc["image"], doc["clips"]["c.mp4"]["original"]
    (tmp_path / "b" / "manifest.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="image.*c.mp4:original"):
        pf.manifest(tmp_path, "b")
