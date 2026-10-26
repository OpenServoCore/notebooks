"""The result of an experiment: one JSON file next to its notebook.

A result is a dict of named quantities, each `{"value", "lo", "hi", "unit"}`,
plus `capture` (the dataset folder it came from) and `notes`. An experiment
without a servo and board pair writes `results.json` in its folder. One with
a pair writes `results/<servo>__<board>.json` there, one file per pair, and
a new run of the same pair replaces it. A later stage reads these files and
nothing else from the stages before it.
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def quantity(value, lo=None, hi=None, unit=""):
    return {"value": float(value), "lo": None if lo is None else float(lo),
            "hi": None if hi is None else float(hi), "unit": unit}


def path(stage, experiment, pair=None):
    folder = ROOT / "stages" / stage / experiment
    return folder / "results.json" if pair is None else folder / "results" / f"{pair}.json"


def write(stage, experiment, capture, pair=None, notes="", **quantities):
    p = path(stage, experiment, pair)
    p.parent.mkdir(parents=True, exist_ok=True)
    capture = Path(capture)
    doc = {"capture": str(capture.resolve().relative_to(ROOT)) if capture.is_absolute() else str(capture),
           "notes": notes, **quantities}
    p.write_text(json.dumps(doc, indent=2) + "\n")
    return p


def read(stage, experiment, pair=None):
    return json.loads(path(stage, experiment, pair).read_text())
