"""The result of an experiment: one JSON file next to its notebook.

A result is a dict of named quantities, each `{"value", "lo", "hi", "unit"}`,
plus `capture` (the dataset folder it came from) and `notes`. An experiment
without a servo and board pair writes `results.json` in its folder. One with
a pair writes `results/<servo>__<board>.json` there, one file per pair, and
a new run of the same pair replaces it. A later stage takes its constants
from these files; it may also quote the earlier notebooks and re-read their
datasets.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def quantity(value, lo=None, hi=None, unit=""):
    return {"value": float(value), "lo": None if lo is None else float(lo),
            "hi": None if hi is None else float(hi), "unit": unit}


q = quantity


def mean_sd(x, unit, scale=1.0):
    """The mean of `x`, one standard deviation either side as the bounds."""
    x = pd.Series(x).dropna() * scale
    mean = x.mean()
    sd = x.std()
    return quantity(mean, mean - sd, mean + sd, unit)


def median_range(x, unit, value=None):
    """The median of `x`, or `value` when given, the range as the bounds."""
    x = pd.Series(x).dropna()
    if value is None:
        value = x.median()
    return quantity(value, x.min(), x.max(), unit)


def pm(value, err, unit, scale=1.0):
    """`value` with `err` either side as the bounds, all times `scale`."""
    lo = (value - err) * scale
    hi = (value + err) * scale
    return quantity(value * scale, lo, hi, unit)


def line_q(fit, key, unit, scale=1.0):
    """A coefficient of a `laps.line` fit, its standard error as the bounds."""
    return pm(fit[key], fit[key + "_se"], unit, scale)


def _plain(value):
    # JSON knows no numpy type and no NaN: a non-finite float is written as null
    if isinstance(value, dict):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, (bool, np.bool_)):
        return bool(value)
    if isinstance(value, (int, np.integer)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    return value


def rel(p):
    """`p` relative to the repo root, the form every notebook prints and the
    result files carry; an absolute path says where one machine keeps the
    repo and belongs in no output."""
    p = Path(p)
    return p.resolve().relative_to(ROOT) if p.is_absolute() else p


def path(stage, experiment, pair=None):
    folder = ROOT / "stages" / stage / experiment
    return folder / "results.json" if pair is None else folder / "results" / f"{pair}.json"


def write(stage, experiment, capture, pair=None, notes="", **quantities):
    p = path(stage, experiment, pair)
    p.parent.mkdir(parents=True, exist_ok=True)
    capture = Path(capture)
    doc = {"capture": str(rel(capture)), "notes": notes, **quantities}
    p.write_text(json.dumps(_plain(doc), indent=2) + "\n")
    return rel(p)


def read(stage, experiment, pair=None):
    return json.loads(path(stage, experiment, pair).read_text())
