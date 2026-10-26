"""`clips.extract` over a synthetic clip: the corners table it writes gives
the same angles as tracking the frames live, and the manifest describes the
file."""
import json
import shutil

import cv2
import numpy as np
import pandas as pd
import pytest

from oscnb import clips, fiducial as fd
import fiducial_synth as sy

TRUTH = np.array([-20.0, 0.0, 33.5, 90.0])


@pytest.fixture(scope="module")
def dataset(tmp_path_factory):
    folder = tmp_path_factory.mktemp("capture-1")
    r = sy.Renderer()
    R, t = sy.pose()
    w, h = r.cam.size
    out = cv2.VideoWriter(str(folder / "turn.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
    for i, d in enumerate(TRUTH):
        g = r.render(sy.tag_scene(d), R, t, seed=i)
        out.write(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    out.release()
    r.cam.camera().save(folder / "camera.json")
    return folder


def test_extract_writes_corners_manifest_and_preview(dataset):
    manifest = clips.extract(dataset, log=lambda *a: None)
    df = pd.read_csv(dataset / "turn.corners.csv")
    assert len(df) == len(TRUTH)
    got = fd.angles(df)["angle_deg"].to_numpy()
    # video compression blurs the edges, so the tolerance is looser than the
    # raw-frame tests, but the truth must still be within a tenth of a degree
    np.testing.assert_allclose(got, TRUTH, atol=0.1)
    entry = manifest["clips"]["turn.mp4"]
    assert entry["frames"] == len(TRUTH) and entry["width"] == sy.SynthCamera().size[0]
    assert len(entry["sha256"]) == 64 and "url" not in entry
    assert manifest["pipeline"]["fiducial_sha256"]
    assert json.loads((dataset / "manifest.json").read_text()) == manifest
    if shutil.which("ffmpeg"):
        assert (dataset / "turn.preview.mp4").stat().st_size > 0


def test_extract_keeps_urls_from_an_earlier_manifest(dataset):
    m = clips.read_manifest(dataset)
    m["clips"]["turn.mp4"]["url"] = "https://example.invalid/turn.mp4"
    clips.write_manifest(dataset, m)
    manifest = clips.extract(dataset, log=lambda *a: None)
    assert manifest["clips"]["turn.mp4"]["url"] == "https://example.invalid/turn.mp4"


def test_release_tag_follows_the_dataset_path():
    assert clips.release_tag(clips.ROOT / "telemetry/instruments/camera-tag/capture-1") == "instruments-camera-tag-capture-1"


@pytest.fixture(scope="module")
def board_dataset(tmp_path_factory):
    folder = tmp_path_factory.mktemp("capture-2")
    r = sy.Renderer()
    w, h = r.cam.size
    poses = [((0, 0), 0, 330), ((22, 0), 5, 340), ((-22, 0), -5, 340), ((0, 24), 90, 330),
             ((0, -24), 85, 330), ((15, 15), 30, 300), ((-15, 18), -25, 310), ((18, -15), 60, 320)]
    board = sy.board_scene()
    rng = np.random.default_rng(7)
    out = cv2.VideoWriter(str(folder / "board.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
    for i, (tilt, roll, dist) in enumerate(poses):
        g = r.render([board], *sy.pose(tilt, roll, dist, look_at=rng.uniform(-30, 30, 2)), seed=i)
        for _ in range(2 * round(30 * clips.BOARD_STEP_S) + 1):
            out.write(cv2.cvtColor(g, cv2.COLOR_GRAY2BGR))
    out.release()
    return folder, len(poses), r.cam.f


def test_extract_calibrates_from_the_board_clip(board_dataset):
    folder, n, f = board_dataset
    manifest = clips.extract(folder, log=lambda *a: None)
    cam = fd.Camera.load(folder / "camera.json")
    assert cam.views == n and cam.rms_px < 0.3
    np.testing.assert_allclose(np.diag(cam.K)[:2], f, rtol=5e-3)
    assert "board.mp4" in manifest["clips"] and not (folder / "board.corners.csv").exists()


def test_board_print_scale_enters_the_calibration(board_dataset):
    folder, n, f = board_dataset
    # a board printed 2 % wider than tall pulls the fitted fx/fy ratio down;
    # the solver trades part of it against the view poses, so the size is
    # checked loosely and the direction strictly
    (folder / "calipers.json").write_text(json.dumps({"board": {"x_mm": 125.0 * 1.02, "y_mm": 125.0}}))
    (folder / "camera.json").unlink()
    clips.extract(folder, log=lambda *a: None)
    cam = fd.Camera.load(folder / "camera.json")
    fx, fy = np.diag(cam.K)[:2]
    assert 0.975 < fx / fy < 0.995
    (folder / "calipers.json").unlink()


def test_board_views_take_one_frame_per_pause(board_dataset):
    folder, n, _ = board_dataset
    views, pauses = clips.board_views(folder / "board.mp4")
    assert pauses == n and len(views) == n


def test_extract_reruns_skip_existing_tables(dataset):
    table = dataset / "turn.corners.csv"
    before = table.stat().st_mtime_ns
    clips.extract(dataset, log=lambda *a: None)
    assert table.stat().st_mtime_ns == before
    clips.extract(dataset, log=lambda *a: None, force=True)
    assert table.stat().st_mtime_ns > before


def test_extract_traces_a_lamp_clip_without_a_lens(tmp_path):
    out = cv2.VideoWriter(str(tmp_path / "lamp.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 30, (160, 120))
    for i in range(60):
        img = np.full((120, 160, 3), 60, np.uint8)
        if i % 20 < 6:
            img[70:78, 100:108] = 230
        out.write(img)
    out.release()
    manifest = clips.extract(tmp_path, log=lambda *a: None, previews=False)
    level = pd.read_csv(tmp_path / "lamp.level.csv")
    assert len(level) == 60 and not (tmp_path / "camera.json").exists()
    assert len(manifest["clips"]["lamp.mp4"]["lamp_roi"]) == 4
    assert len(fd.bursts(level["t"], level["level"], min_gap_s=0.1, min_len_s=0.05)) == 3
