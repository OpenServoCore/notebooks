"""Stage 0, camera-tag: the angle error and the noise floor of the tag pipeline
on the bench. The phone is not scripted. This lists the steps, waits for the
files, then extracts the corners tables and previews from the clips (oscnb.clips).
equipment/camera/tags.md says how to generate, print and mount the sheets.

  uv run python stages/0-instruments/camera-tag/capture.py [N]    writes telemetry/instruments/camera-tag/capture-N
"""
import json
import time

from oscnb import capture, clips

out = capture.dataset("instruments", "camera-tag")
if not (out / "calipers.json").exists():
    (out / "calipers.template.json").write_text(json.dumps({
        "phone": "model and video setting, e.g. iPhone 15, 1080p 240 fps slo-mo, 1x",
        "distance_mm": 200.0,
        "horn": {"x_mm": 50.00, "y_mm": 50.00},
        "ref": {"x_mm": 50.00, "y_mm": 50.00},
        "board": {"x_mm": 125.00, "y_mm": 125.00},
    }, indent=2) + "\n")
print(f"""
Generate the sheets with `uv run python equipment/camera/tags.py`, then print
equipment/camera/tags-angle.pdf and tags-calibration.pdf at 100 % (Actual size),
as equipment/camera/tags.md describes.
1. Measure each tag's 50 mm check square along X and Y with calipers, and five squares
   of the board (125 mm) along X and Y. Copy calipers.template.json to calipers.json
   in {out} and fill the numbers and the phone.
2. Glue the HORN tag and the REFERENCE tag flat on one rigid plate, about 60 mm apart,
   both square to the plate's edges. Dry glue or tape, no wet paste.
3. Fix the phone over the desk, lens straight down, about 20 cm above the plate (an overhead
   phone arm, or the phone bridged across two stacks of books). Measure lens to tags with a
   ruler and write distance_mm into calipers.json.
   Phone settings, the same for every clip (step by step in equipment/camera/iphone.md):
   - Slo-Mo mode, 1080p at 240 fps (Settings > Camera > Record Slo-mo)
   - HDR video off (Settings > Camera > Record Video)
   - 1x lens, no automatic lens switching (Macro Control on, macro off), or the
     phone more than about 20 cm from the tags
   - focus and exposure locked on the tags at that distance: long-press until AE/AF Lock shows
4. board.mp4   30 to 60 s, phone on its mount, lock untouched. Board sheet taped to cardboard,
               moved under the camera and held still about 2 s at each pose: flat in the middle,
               slid to each corner and edge of the frame, then propped on a book at 20 to 30
               degrees facing up, down, left and right, 15 to 20 poses in all.
5. still.mp4   the plate at rest under the camera, tags near the frame centre, 10 s.
6. turn.mp4    turn the plate by hand through one full turn in about twelve steps, pausing
               three seconds at each, tags staying near the frame centre.
Copy the three original clips into {out}, as equipment/camera/iphone.md describes.
Rename each .MOV to the name above, no conversion. This script waits for them, then
runs the extraction. The originals stay out of git.
""")
for name in ("calipers.json", "board.mp4", "still.mp4", "turn.mp4"):
    while not (out / name).exists() or (out / name).stat().st_size == 0:
        time.sleep(5)
    print("got", name)
clips.extract(out)
print("wrote", out)
print(f"to put the originals on a GitHub release: uv run python equipment/camera/extract.py {out} --upload")
