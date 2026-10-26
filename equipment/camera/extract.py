"""Turn the clips of a dataset into what the notebooks read: the lens
calibration, a corners table and a real-time preview per clip, and a manifest
with each original's hash. The originals are gigabytes and stay out of git;
the previews are committed so the capture can be seen and replicated.

  uv run python equipment/camera/extract.py telemetry/instruments/camera-tag/capture-1 [--force] [--upload]

A rerun redoes only what is missing; --force redoes everything. --upload
(optional, unused so far) attaches the originals to a GitHub release of this
repo, 2 GB per file. Needs `gh` logged in.
"""
from oscnb import clips

clips.main()
