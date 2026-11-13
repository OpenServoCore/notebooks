"""The position table in the format of protocol sec 5.7, offline: the
interpolation on hand-computed points, the servo's two rules, the build
from a small synthetic lookup, the held-out grade through it and the
install plan's wire steps."""
import json

import numpy as np
import pandas as pd
import pytest

from oscnb import position as pe
from oscnb import postable as pt
from oscnb import results

PAIR = "synth__board"
STOPS = (82, 3792)


def table(**points):
    k = np.zeros(pt.INTERVALS, np.int64)
    for i, c in points.items():
        k[int(i[1:])] = c
    return k


def step(rise):
    """A rise at point 101 tapering back to zero 15 counts a point, so only
    the rise's interval is off nominal."""
    k = np.zeros(pt.INTERVALS, np.int64)
    c, i = rise, 101
    while c > 0:
        k[i] = c
        c -= pt.GRID - 1
        i += 1
    return k


def test_identity_is_raw_shifted_by_four():
    raw = np.arange(4096)
    np.testing.assert_array_equal(pt.interp_q4(raw, np.zeros(256, int)), raw << 4)


def test_interpolation_at_hand_computed_points():
    k = table(k100=5, k101=-3)
    for raw, want in ((1599, 1599 * 16 + 5 * 15), (1600, 1605 * 16),
                      (1608, 1613 * 16 - 8 * 8), (1615, 1620 * 16 - 8 * 15),
                      (1616, 1613 * 16), (1624, 1621 * 16 + 3 * 8)):
        assert pt.interp_q4(raw, k) == want, raw


def test_interpolation_wraps_like_a_u16_and_masks_to_12_bits():
    assert pt.interp_q4(0, table(k0=-1)) == 0xFFF0
    assert pt.interp_q4(4095, table(k255=1)) == (4096 << 4) - 15
    k = table(k100=5)
    assert pt.interp_q4(0x1640, k) == pt.interp_q4(0x640, k)


def test_ends_rule_with_the_stops_off_and_on_a_point():
    for i in (13, 14, 240, 255):
        assert pt.validate(table(**{f"k{i}": 1}), 209, 3849).startswith("REJECT_ENDS")
    for i in (15, 239):
        assert pt.validate(table(**{f"k{i}": 1}), 209, 3849) is None
    for i in (15, 16, 240):
        assert pt.validate(table(**{f"k{i}": 1}), 256, 3840).startswith("REJECT_ENDS")
    for i in (17, 239):
        assert pt.validate(table(**{f"k{i}": 1}), 256, 3840) is None
    assert pt.validate(np.zeros(256, int), 0, 0) is None
    assert pt.validate(table(k100=1), 0, 0).startswith("REJECT_ENDS")


def test_shape_rule_and_ends_judged_first():
    assert "intervals 100" in pt.validate(table(k100=16), 209, 3849)
    assert pt.validate(table(k100=15), 209, 3849) is None
    assert pt.validate(step(240), 209, 3849).startswith("REJECT_SHAPE")
    assert pt.validate(step(239), 209, 3849) is None
    assert pt.validate(table(k0=-100), 209, 3849).startswith("REJECT_ENDS")


def angle_at(counts):
    """A monotone pot read backwards: about 20 counts/deg with a bend."""
    c = np.asarray(counts, float)
    return -180 + (c - 100) / 20.0 + 1.5 * np.sin(c / 300)


def write_lookup(root, pitch=32, first=384, last=3488, angle=angle_at):
    knots = np.arange(first, last + 1, pitch, dtype=float)
    a = angle(knots)
    folder = root / "stages" / "4-estimators" / "position" / "results"
    folder.mkdir(parents=True)
    pd.DataFrame({"counts": knots.astype(int), "angle_deg": a}).to_csv(
        folder / f"{PAIR}.lookup.csv", index=False)
    q = results.quantity
    doc = {"lookup_pitch_counts": q(pitch), "lookup_knot_first_counts": q(first),
           "lookup_knot_last_counts": q(last),
           "lookup_entries_guard_to_guard": q(len(knots)),
           "end_line_low_counts_per_deg": q((knots[1] - knots[0]) / (a[1] - a[0])),
           "end_line_high_counts_per_deg": q((knots[-1] - knots[-2]) / (a[-1] - a[-2])),
           "table_term_heldout_deg_rms": q(0.16),
           "table_term_heldout_counts_rms": q(3.4),
           "heldout_lead_applied_deg_rms": q(0.41)}
    (folder / f"{PAIR}.json").write_text(json.dumps(doc))


@pytest.fixture
def repo(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    monkeypatch.setattr(pt, "OUT_DIR", tmp_path / "stages" / "5-closed-loop" / "calibration")
    return tmp_path


def test_build_linearizes_the_lookup_with_the_stops_fixed(repo):
    write_lookup(repo)
    t, path = pt.build(PAIR, *STOPS)
    assert path == repo / "stages" / "5-closed-loop" / "calibration" / f"{PAIR}.pos_lut.json"
    assert pt.validate(t.points, *STOPS) is None
    assert pt.interp_q4(STOPS[0], t.points) == STOPS[0] << 4
    assert pt.interp_q4(STOPS[1], t.points) == STOPS[1] << 4
    raw = np.arange(4096)
    np.testing.assert_array_equal(t.lin(raw) * 16, pt.interp_q4(raw, t.points))
    lk, _, _ = pt.lookup_of_record(PAIR)
    inside = np.arange(384, 3489)
    err_counts = (t.angle(inside) - lk.angle(inside)) * t.scale
    assert np.abs(err_counts).max() <= 0.5 + 1e-9
    np.testing.assert_allclose(t.counts(lk.angle(inside)), inside, atol=0.6)
    doc = json.loads(path.read_text())
    assert doc["inputs"]["lookup"] == f"stages/4-estimators/position/results/{PAIR}.lookup.csv"
    assert doc["inputs"]["stops"]["raw_min"] == STOPS[0]
    assert len(doc["points_by_page"]) == 8 and all(len(p) == 32 for p in doc["points_by_page"])
    assert "/" + "Users" not in path.read_text() and "/" + "home" not in path.read_text()
    back, _ = pt.load(PAIR)
    np.testing.assert_array_equal(back.points, t.points)


def test_build_refuses_a_lookup_off_the_grid_or_out_of_step_with_its_result(repo):
    write_lookup(repo, pitch=24, first=384, last=3480)
    with pytest.raises(ValueError, match="grid"):
        pt.build(PAIR, *STOPS)
    folder = repo / "stages" / "4-estimators" / "position" / "results"
    for f in folder.iterdir():
        f.unlink()
    folder.rmdir()
    write_lookup(repo)
    doc = json.loads((folder / f"{PAIR}.json").read_text())
    doc["lookup_knot_first_counts"]["value"] = 416
    (folder / f"{PAIR}.json").write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="lookup_knot_first_counts"):
        pt.build(PAIR, *STOPS)


def test_build_refuses_a_table_the_servo_would_reject(repo):
    def flat_cell(c):
        """A cell that barely turns: a near-zero local gain."""
        c = np.asarray(c, float)
        return angle_at(np.where(c <= 1984, c, np.where(
            c >= 2048, c - 63.936, 1984 + (c - 1984) * 0.001)))
    write_lookup(repo, angle=flat_cell)
    with pytest.raises(ValueError, match="REJECT_SHAPE"):
        pt.build(PAIR, *STOPS)
    assert not pt.out_path(PAIR).exists()
    with pytest.raises(ValueError, match="stops"):
        pt.from_lookup(pe.Lookup("counts", [384, 416, 448], [0, 1, 2]), 3792, 82)


def test_load_refuses_a_hand_edited_table(repo):
    write_lookup(repo)
    _, path = pt.build(PAIR, *STOPS)
    doc = json.loads(path.read_text())
    doc["points_by_page"][0][2] = 4
    path.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="REJECT_ENDS: points 2 "):
        pt.load(PAIR)


def test_plan_writes_every_page_then_commits_reads_back_and_stamps_unsaved(repo):
    write_lookup(repo)
    t, _ = pt.build(PAIR, *STOPS)
    text = pt.plan(PAIR)
    cmds = [ln for ln in text.splitlines() if not ln.startswith("#")]
    stores = [c.split()[5] for c in cmds if " write 0x19c " in c and len(c.split()[5]) == 132]
    assert len(stores) == 8
    for p, hexed in enumerate(stores):
        b = bytes.fromhex(hexed)
        assert b[0] == p and b[1] == pt.STORE
        np.testing.assert_array_equal(np.frombuffer(b[2:], "<i2"), t.points[32 * p:32 * (p + 1)])
    order = [next(i for i, c in enumerate(cmds) if key in c)
             for key in ("get torque_enable", "write 0x19c 0001", "write 0x19c 0003",
                         "write 0x19c 0702", " stamp")]
    assert order == sorted(order)
    words = {c.split()[3] for c in cmds}
    assert words <= {"get", "write", "stamp", "status"}
    assert not any("--save" in c or " save" in c for c in cmds)


def synthetic_sets(seed=2):
    """Moving frames, rest points and scored frames in two folds over a
    pot that reads angle_at backwards, each segment with its own offset."""
    rng = np.random.default_rng(seed)
    counts = np.linspace(600, 3200, 3000)
    theta = angle_at(counts)
    moving, rest, scored = [], [], []
    for s in range(8):
        sl = slice(s * 300, s * 300 + 1200)
        off = rng.normal(0, 0.2)
        f = pd.DataFrame({"sid": f"s{s}", "fold": s % 2, "pos": counts[sl],
                          "image_deg": theta[sl] + off + rng.normal(0, 0.05, len(counts[sl]))})
        moving.append(f)
        scored.append(f.assign(pos_cam=f["pos"]))
    for q in (0, 1):
        c = np.linspace(700 + 50 * q, 3100, 20)
        rest.append(pd.DataFrame({"fold": q, "pos": c, "image_deg": angle_at(c)}))
    return (pd.concat(moving, ignore_index=True), pd.concat(rest, ignore_index=True),
            pd.concat(scored, ignore_index=True))


def test_heldout_through_the_table_matches_the_lookup():
    moving, rest, scored = synthetic_sets()
    g = pt.heldout(moving, rest, scored, *STOPS, 32)
    assert g["lookup"][0] < 0.06
    assert abs(g["table"][0] - g["lookup"][0]) < 0.005
    assert abs(g["table"][2] - g["lookup"][2]) < 0.005
    assert g["lookup"][2] > g["lookup"][0]
