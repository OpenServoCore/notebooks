"""Stage 3's frame pipeline on synthetic inputs: the schedule step of a
segment, the pot's centered mean and speed, the stamp lead and the rule that
picks the moving frames."""
import numpy as np
import pandas as pd

from oscnb import potframes as pf

FS = pf.FS


def test_step_of_names_the_baseline_and_wraps_the_schedule():
    schedule = "30@150,then:6@1491,coast:300"
    steps = [pf.step_of(schedule, s) for s in range(7)]
    assert steps == ["baseline", "30@150", "then:6@1491", "coast:300",
                     "30@150", "then:6@1491", "coast:300"]


def test_centered_mean_has_no_delay_on_a_ramp_and_shrinks_at_the_ends():
    x = 3.0 * np.arange(200) + 7
    y = pf.centered_mean(x, 41)
    np.testing.assert_allclose(y[20:-20], x[20:-20], atol=1e-9)
    z = pf.centered_mean(x ** 2, 5)
    assert np.isclose(z[0], np.mean((x ** 2)[:3]))
    assert np.isclose(z[-1], np.mean((x ** 2)[-3:]))


def test_pot_speed_reads_a_ramps_slope_and_is_nan_off_the_segment():
    slope = 400.0                       # counts/s
    x = 1000 + slope * np.arange(2000) / FS
    v = pf.pot_speed(x)
    half = pf.SPEED_ROWS // 2
    assert np.isnan(v[:half]).all() and np.isnan(v[-half:]).all()
    # the centered mean shrinks within BOX_ROWS // 2 of an end, so the
    # window's ends must sit clear of that
    clear = half + pf.BOX_ROWS // 2
    np.testing.assert_allclose(v[clear:-clear], slope, rtol=1e-9)
    assert np.isnan(pf.pot_speed(x[:pf.SPEED_ROWS])).all()


def test_guards_and_rail_read_the_sittings_own_events(tmp_path):
    log = tmp_path / "sitting.csv"
    pd.DataFrame({"t_host": [0.0, 1.0, 2.0],
                  "event": ["rail", "guards", "clip"],
                  "detail": ["5.0 V", "414 to 3460 counts",
                             "pass started"]}).to_csv(log, index=False)
    assert pf.guards_and_rail(log) == ((414.0, 3460.0), 5.0)


def angle_of(t):
    """A group's motion: at rest, a wandering drive, at rest again, as a
    group runs from its first drive to its last coast's tail."""
    w = np.where((t > 0.8) & (t < 2.2), np.sin(np.pi * (t - 0.8) / 1.4), 0)
    return -100 + w ** 2 * (10 * np.sin(2 * np.pi * 1.3 * t)
                            + 4 * np.sin(2 * np.pi * 3.7 * t + 1.0))


def lead_pair(lead_s, counts_per_deg=19.0):
    t = np.arange(0.0, 3.0, 1 / 240)    # the tag's frames, about 4.2 ms
    tags = pd.DataFrame({"host": t, "image_deg": angle_of(t), "both": True})
    tr = np.arange(0.4, 2.6, 1 / FS)
    # each row is stamped lead_s before the instant it read
    rows = pd.DataFrame({"host": tr - lead_s,
                         "pos": 2000 + counts_per_deg * angle_of(tr)})
    return tags, rows


def test_lead_of_recovers_a_known_shift_to_half_a_millisecond():
    for lead_s in (0.0123, -0.0071, 0.0):
        tags, rows = lead_pair(lead_s)
        shift, corr = pf.lead_of(tags, rows)
        assert abs(shift - lead_s) < 0.0005, (lead_s, shift)
        assert corr > 0.99


def frames_table():
    good = {"clip": "crawl-0-0", "k": 1, "seg": 2, "step": "then:6@1491",
            "t_seg": 0.5, "seg_len": 2.0, "pos_speed": 120.0,
            "image_deg": -100.0}
    rows = [
        good,
        {**good, "seg": 5, "pos_speed": -120.0, "image_deg": -90.0},
        {**good, "clip": "pass-open", "k": 0, "seg": 1, "step": "20@915",
         "image_deg": -60.0},
        {**good, "step": "30@150", "image_deg": -150.0},       # the kick
        {**good, "step": "coast:300", "image_deg": -150.0},
        {**good, "t_seg": pf.SKIP_S - 0.01, "image_deg": -150.0},
        {**good, "t_seg": 2.0 - pf.END_S + 0.005, "image_deg": -20.0},
        {**good, "pos_speed": pf.MOVING_CPS - 1, "image_deg": -20.0},
        {**good, "image_deg": np.nan},
    ]
    return pd.DataFrame(rows)


def test_moving_frames_keeps_only_the_steady_drives():
    moving, lo, hi = pf.moving_frames(frames_table())
    assert list(moving.index) == [0, 1, 2]
    assert list(moving["sid"]) == ["crawl-0-0/1/2", "crawl-0-0/1/5",
                                   "pass-open/0/1"]
    assert (lo, hi) == (-100.0, -60.0)
