"""Video clips in a dataset: what is committed and what is not.

A phone clip runs to gigabytes, so the originals stay out of git, on the
host that captured them. `extract` runs the tag pipeline over every clip in a
dataset folder once and writes what the notebooks read instead, plus a small
preview of each clip so that someone replicating the experiment can see what
was filmed:

  camera.json          the lens calibration, from one view per pause of
                       board.mp4 (`board_views`), with the board's print scale
                       from calipers.json when that holds a `board` entry
                       (five squares measured along X and Y, nominal 125 mm)
  <clip>.corners.csv   one row per frame: container time and the refined
                       corners of both tags (`fiducial.track`)
  <clip>.level.csv     for a clip named lamp*.mp4, the lamp's grey level per
                       frame instead (`fiducial.lamp_trace`), from the box
                       `fiducial.lamp_roi` finds, recorded in the manifest
  <clip>.preview.mp4   a real-time copy, longest side 960 px, for viewing only
  manifest.json        each original's size, sha256, frame count and rate,
                       and the pipeline version that made the tables

A rerun skips every table and preview that already exists, so adding a
clip or regenerating one file is cheap; `force=True` redoes them all.
`upload` is optional: it attaches the originals to a GitHub release of this
repo (one per dataset, 2 GB per file cap) and writes the URLs into the
manifest. Not used so far; the hashes in the manifest identify the originals
either way.
"""
import hashlib
import json
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

from . import fiducial as fd

ROOT = Path(__file__).resolve().parents[1]
BOARD = "board.mp4"
PREVIEW = ".preview.mp4"
BOARD_STEP_S = 1 / 16   # between sampled frames of the board clip, whatever its rate
BOARD_STILL_PX = 1.0
BOARD_MAX_VIEWS = 60
BOARD_CHECK_MM = 5 * fd.BOARD_SQUARE_MM


def board_scale(folder):
    """The board's print scale from calipers.json, or the nominal one."""
    p = Path(folder) / "calipers.json"
    if not p.exists():
        return fd.PrintScale()
    b = json.loads(p.read_text()).get("board")
    if not b:
        return fd.PrintScale()
    k = fd.CHECK_MM / BOARD_CHECK_MM
    return fd.PrintScale(b["x_mm"] * k, b["y_mm"] * k)


def clips(folder):
    """The original clips in a dataset folder, board first."""
    folder = Path(folder)
    names = sorted(p.name for p in folder.glob("*.mp4") if not p.name.endswith(PREVIEW))
    return [folder / n for n in sorted(names, key=lambda n: (n != BOARD, n))]
    # any other board-*.mp4 is kept and previewed but neither calibrated from nor tracked


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            h.update(chunk)
    return h.hexdigest()


def probe(path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise OSError(f"cannot open {path}")
    try:
        fps = cap.get(cv2.CAP_PROP_FPS)
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        return {"bytes": Path(path).stat().st_size, "frames": frames, "fps": fps,
                "width": int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                "duration_s": frames / fps if fps else None}
    finally:
        cap.release()


def preview(src, dst, side=960, fps=30, crf=30):
    """An H.264 copy at real-time rate with its longest side `side` px: a
    240 fps slow-motion clip plays at the speed it was filmed. About 2 MB per
    minute at the defaults. Needs ffmpeg on the PATH; without it the preview
    is skipped and the caller is told."""
    if not shutil.which("ffmpeg"):
        return False
    scale = f"scale='if(gt(iw,ih),{side},-2)':'if(gt(iw,ih),-2,{side})'"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(src),
                    "-vf", f"{scale},fps={fps}", "-c:v", "libx264", "-crf", str(crf),
                    "-pix_fmt", "yuv420p", "-an", "-movflags", "+faststart", str(dst)], check=True)
    return True


def pipeline_version():
    """What produced the tables: the repo commit and the hash of the pipeline
    module, so a table can be told apart from one made by a later pipeline."""
    def git(*args):
        r = subprocess.run(["git", "-C", str(ROOT), *args], capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else None
    dirty = git("status", "--porcelain", "--", "oscnb")
    return {"commit": git("rev-parse", "--short", "HEAD"), "oscnb_dirty": bool(dirty),
            "fiducial_sha256": sha256(ROOT / "oscnb" / "fiducial.py")[:12], "opencv": cv2.__version__}


def board_views(path, scale=fd.PrintScale(), step_s=BOARD_STEP_S, still_px=BOARD_STILL_PX, max_views=BOARD_MAX_VIEWS):
    """Frames of a board clip to calibrate from, one per pause.

    The phone sits on its mount and the board is moved under it, held still
    at each pose. A frame is read every `step_s` seconds, and a run of frames
    in which the board's corners move under `still_px` between samples is one pause;
    the frame with the most corners in it is its view. A clip with under
    eight pauses (hand-held) falls back to every sampled frame. The views are thinned
    evenly to `max_views`. Returns (views, pauses): the images and how many
    pauses were found (0 for the fallback)."""
    board = fd.charuco_board()
    step = max(1, round(probe(path)["fps"] * step_s))
    # first pass keeps only where the board was, not the frames: a clip holds
    # thousands of samples and a frame is megabytes
    sampled = []
    for i, t, img in fd.video_frames(path, step=step):
        bp = fd.board_points(img, board, scale)
        if bp is not None:
            sampled.append((i, bp[1].mean(axis=0), len(bp[0])))
    runs, run = [], []
    for k, (i, centre, n) in enumerate(sampled):
        if run and np.linalg.norm(centre - sampled[k - 1][1]) >= still_px:
            runs.append(run)
            run = []
        run.append(k)
    if run:
        runs.append(run)
    pauses = [r for r in runs if len(r) >= 2]
    # fewer pauses than this cannot pin a lens: fall back to every sample
    if len(pauses) >= 8:
        chosen = [sampled[max(r, key=lambda k: sampled[k][2])][0] for r in pauses]
    else:
        chosen, pauses = [s[0] for s in sampled], []
    if len(chosen) > max_views:
        chosen = [chosen[int(k)] for k in np.linspace(0, len(chosen) - 1, max_views)]
    wanted = set(chosen)
    views = [img for i, t, img in fd.video_frames(path, stop=max(wanted) + 1) if i in wanted]
    return views, len(pauses)


def read_manifest(folder):
    p = Path(folder) / "manifest.json"
    return json.loads(p.read_text()) if p.exists() else {"clips": {}}


def write_manifest(folder, doc):
    (Path(folder) / "manifest.json").write_text(json.dumps(doc, indent=2) + "\n")


def extract(folder, log=print, force=False, previews=True):
    """Run the pipeline over every clip in `folder`. Reuses camera.json and
    every table and preview that already exist unless `force`. `previews`
    off skips the preview encodes, for a run over clips that are previews
    already. Returns the manifest."""
    folder = Path(folder)
    found = clips(folder)
    if not found:
        raise FileNotFoundError(f"no clips in {folder}")
    cam_path = folder / "camera.json"
    needs_lens = any(not c.name.startswith(("board", "lamp")) for c in found)
    if cam_path.exists() and not (force and (folder / BOARD).exists()):
        cam = fd.Camera.load(cam_path)
    elif (folder / BOARD).exists():
        log(f"calibrating from {BOARD}")
        views, pauses = board_views(folder / BOARD, board_scale(folder))
        cam = fd.calibrate(views, scale=board_scale(folder))
        cam.save(cam_path)
        log(f"  {cam.views} views from {pauses} pauses, {cam.rms_px:.3f} px rms" if pauses
            else f"  {cam.views} views, no pauses found so every sampled frame was used, {cam.rms_px:.3f} px rms")
    elif not needs_lens:
        cam = None
    else:
        raise FileNotFoundError(f"{folder} has neither camera.json nor {BOARD}")
    manifest = read_manifest(folder)
    for clip in found:
        entry = manifest["clips"].get(clip.name, {})
        entry.update(probe(clip))
        entry["sha256"] = sha256(clip)
        table = clip.with_name(clip.stem + ".corners.csv")
        level = clip.with_name(clip.stem + ".level.csv")
        if clip.name.startswith("lamp") and (force or not level.exists()):
            log(f"tracing the lamp in {clip.name}")
            roi = fd.lamp_roi(fd.video_frames(clip, step=4))
            df = fd.lamp_trace(fd.video_frames(clip), roi, channel="grey")
            df.to_csv(level, index=False, float_format="%.3f")
            entry["lamp_roi"] = list(roi)
            log(f"  {len(df)} frames, lamp box {roi}")
        elif not clip.name.startswith(("board", "lamp")) and (force or not table.exists()):
            log(f"tracking {clip.name}")
            df = fd.track(fd.video_frames(clip), cam)
            df.to_csv(table, index=False, float_format="%.4f")
            log(f"  {len(df)} frames, horn seen in {int(df.hx0.notna().sum())}, ref in {int(df.rx0.notna().sum())}")
        pv = clip.with_name(clip.stem + PREVIEW)
        if previews and (force or not pv.exists()) and not preview(clip, pv):
            log("  no ffmpeg on the PATH, preview skipped")
        manifest["clips"][clip.name] = entry
    manifest["pipeline"] = pipeline_version()
    write_manifest(folder, manifest)
    return manifest


def release_tag(folder):
    """One release per dataset, named after its path under telemetry/."""
    rel = Path(folder).resolve().relative_to(ROOT / "telemetry")
    return "-".join(rel.parts)


def upload(folder, log=print):
    """Attach the originals to a GitHub release of this repo and write their
    URLs into the manifest. Needs `gh` logged in. Re-running replaces the
    assets."""
    folder = Path(folder)
    manifest = read_manifest(folder)
    if not manifest["clips"]:
        raise FileNotFoundError(f"{folder} has no manifest; run extract first")
    tag = release_tag(folder)
    repo = subprocess.run(["gh", "repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner"],
                          cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    if subprocess.run(["gh", "release", "view", tag], cwd=ROOT, capture_output=True).returncode != 0:
        subprocess.run(["gh", "release", "create", tag, "--title", tag, "--notes",
                        f"Original clips of the {tag} dataset. Hashes in its manifest.json."],
                       cwd=ROOT, check=True)
    files = [str(folder / name) for name in manifest["clips"]]
    log(f"uploading {len(files)} clips to release {tag}")
    subprocess.run(["gh", "release", "upload", tag, *files, "--clobber"], cwd=ROOT, check=True)
    for name in manifest["clips"]:
        manifest["clips"][name]["url"] = f"https://github.com/{repo}/releases/download/{tag}/{name}"
    write_manifest(folder, manifest)
    return manifest


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    flags = {a for a in argv if a.startswith("--")}
    args = [a for a in argv if not a.startswith("--")]
    if len(args) != 1 or flags - {"--upload", "--force"}:
        sys.exit("usage: extract.py <dataset folder> [--force] [--upload]")
    folder = Path(args[0])
    extract(folder, force="--force" in flags)
    if "--upload" in flags:
        upload(folder)
