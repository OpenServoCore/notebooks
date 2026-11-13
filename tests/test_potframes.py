"""Stage 3's frame pipeline on synthetic inputs: the schedule step of a
segment, the pot's centered mean and speed, the stamp lead, the rule that
picks the moving frames, the tag as a truth and the series of a chain."""
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


def quad_tags(t):
    y = 3.0 - 40.0 * t + 25.0 * t ** 2
    return pd.DataFrame({"host": t, "image_deg": y, "both": True})


def test_smooth_tag_returns_a_quadratics_own_values_and_nan_by_a_hole():
    t = np.arange(0.0, 0.5, 1 / 240)
    tf, ys, y, lev = pf.smooth_tag(quad_tags(t))
    assert np.isnan(ys[[0, -1]]).all()
    np.testing.assert_allclose(ys[1:-1], y[1:-1], atol=1e-9)
    assert np.isfinite(lev[1:-1]).all()
    # a 50 ms hole: every frame whose quadratic would span it reads NaN
    th = np.r_[t[t < 0.2], t[t > 0.25]]
    tf, ys, y, _ = pf.smooth_tag(quad_tags(th))
    i = int(np.flatnonzero(np.diff(tf) > pf.MAX_FRAME_GAP_S)[0])
    near = np.arange(i - pf.HALF_FRAMES + 1, i + pf.HALF_FRAMES + 1)
    assert np.isnan(ys[near]).all()
    rest = np.setdiff1d(np.arange(1, len(tf) - 1), near)
    np.testing.assert_allclose(ys[rest], y[rest], atol=1e-9)


def test_tag_at_is_a_line_between_frames_and_nan_across_a_hole():
    tf = np.array([0.0, 0.004, 0.008, 0.060, 0.064])
    yf = np.array([1.0, 2.0, 3.0, 10.0, 11.0])
    tr = np.array([-0.001, 0.002, 0.006, 0.03, 0.062, 0.07, np.nan])
    got = pf.tag_at(tf, yf, tr)
    np.testing.assert_allclose(got[1:3], [1.5, 2.5])
    assert got[4] == 10.5
    assert np.isnan(got[[0, 3, 5, 6]]).all()
    # a frame without a truth is not a frame
    got = pf.tag_at(tf, np.r_[yf[:2], np.nan, yf[3:]], np.array([0.006]))
    assert np.isnan(got).all()


def two_segments(hole_s=0.005):
    """Chain rows of segments 1 (400 rows, pot 1000) and 2 (300 rows, pot
    1100, duty 360 after 2 rows at 0) with hole_s of lost rows between."""
    h1 = 10.0 + np.arange(400) / FS
    h2 = h1[-1] + hole_s + (1 + np.arange(300)) / FS
    duty = np.r_[np.full(400, 200), 0, 0, np.full(298, 360)]
    rows = {"seg": np.repeat([1, 2], [400, 300]),
            "pos": np.repeat([1000, 1100], [400, 300]),
            "host": np.r_[h1, h2], "host_cam": np.r_[h1, h2] + 0.002,
            "duty_counts": duty}
    emf = {k: np.ones(700) for k in pf.EMF_KEYS}
    first = np.zeros(700, bool)
    first[[0, 400]] = True
    emf.update(valid=np.ones(700, bool), first=first)
    return rows, emf


def test_group_fills_the_lost_rows_on_a_line_with_no_valid_emf():
    rows, emf = two_segments(0.005)
    t = np.arange(9.9, 10.1, 1 / 240)
    tag = (t, 5.0 + t)
    g = pf.group(rows, [1, 2], emf=emf, tag=tag)
    fill = g["fill"]
    assert fill.sum() == 100 and len(g["pos"]) == 800
    assert g["starts"] == [0, 500]
    assert not fill[:400].any() and fill[400:500].all()
    assert not fill[500:].any()
    seam = g["pos"][399:501]
    np.testing.assert_allclose(np.diff(seam), 100.0 / 101)
    np.testing.assert_allclose(np.diff(g["host"]), 1 / FS, rtol=1e-6)
    assert (g["src"][fill] == -1).all()
    assert list(g["src"][~fill]) == list(range(700))
    assert np.isnan(g["w_tab"][fill]).all() and not g["valid"][fill].any()
    assert not g["first"][fill].any() and g["first"][500]
    assert (g["duty"][fill] == 360).all() and (g["drive"][fill] == 1).all()
    assert (g["seg"][fill] == 2).all() and g["torque_on"].all()
    np.testing.assert_allclose(g["tag"], 5.0 + g["cam"])


def test_group_without_emf_or_tag_carries_the_rows_only():
    rows, _ = two_segments(0.0)
    g = pf.group(rows, [1, 2])
    assert not g["fill"].any() and len(g["pos"]) == 700
    assert not set(pf.EMF_KEYS) & set(g) and np.isnan(g["tag"]).all()


def test_groups_of_is_the_baseline_then_the_lead_groups():
    assert pf.groups_of("pass", range(5)) == [[0], [1, 2], [3, 4]]
    assert pf.groups_of("crawl", range(7)) == [[0], [1, 2, 3], [4, 5, 6]]
    assert pf.groups_of("steps", [2, 0, 1]) == [[0], [1], [2]]


def test_a_chains_load_passes_the_wide_code_step_through_and_leaves_it_off(
        monkeypatch, tmp_path):
    folder = tmp_path / "block" / "chains" / "3"
    folder.mkdir(parents=True)
    pd.DataFrame({"seg": [0, 1], "t_start": [10.0, 20.0]}).to_csv(
        folder / "segments.csv", index=False)
    seen = []

    def load(f, wide_codes=False):
        seen.append(wide_codes)
        return (pd.DataFrame({"seg": [0, 1], "tick": [0, 20], "pos": [1, 2]}),
                {"tick_hz": FS, "sense": {}})

    monkeypatch.setattr(pf.tel, "load", load)
    monkeypatch.setattr(pf, "board_of", lambda meta, where="": None)
    monkeypatch.setattr(pf.results, "rel", str)
    df, _, _ = pf.load_sweep(tmp_path, "block", 3)
    pf.load_sweep(tmp_path, "block", 3, wide_codes=True)
    assert seen == [False, True]
    np.testing.assert_allclose(df["host"], [10.0, 20.001])
