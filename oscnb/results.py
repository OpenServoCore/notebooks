"""The result of an experiment: one JSON file next to its notebook.

A result is a dict of named quantities, each `{"value", "lo", "hi", "unit"}`,
plus `capture` (the dataset folder it came from), `notes` and `inputs`. An
experiment without a servo and board pair writes `results.json` in its
folder. One with a pair writes `results/<servo>__<board>.json` there, one
file per pair, and a new run of the same pair replaces it. A later stage
takes its constants from these files; it may also quote the earlier
notebooks and re-read their datasets.

`inputs` is what the result stands on: every result file the process that
wrote it had read (`read`, `read_file`), by its path from the repo root
with the sha1 of the bytes read, so a result whose input was since re-run
can be told from one that is current. A file written before results named
what they read has no `inputs`: that is "not recorded", not "read
nothing", which is an empty `inputs`.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
_READ = {}      # {path from the repo root: sha1} of every result file this process has read


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


def mean_se(x, unit, scale=1.0):
    """The mean of `x`, one standard error either side as the bounds."""
    x = pd.Series(x).dropna() * scale
    return pm(x.mean(), x.std() / np.sqrt(len(x)), unit)


def wmean(x):
    """(mean, sd, the mean's standard error, count) of x, nan dropped."""
    x = pd.Series(np.asarray(x, float)).dropna()
    if len(x) == 0:
        return np.nan, np.nan, np.nan, 0
    if len(x) == 1:
        return float(x.mean()), np.nan, np.nan, 1
    sd = float(x.std(ddof=1))
    return float(x.mean()), sd, sd / np.sqrt(len(x)), int(len(x))


def fit_line(x, y):
    """Least-squares line of y on x over the finite points: slope,
    intercept, their standard errors, the residuals, rms and count."""
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y)
    x, y = x[ok], y[ok]
    design = np.c_[x, np.ones_like(x)]
    coef = np.linalg.lstsq(design, y, rcond=None)[0]
    resid = y - design @ coef
    s2 = (resid ** 2).sum() / max(len(x) - 2, 1)
    cov = s2 * np.linalg.inv(design.T @ design)
    return {"slope": coef[0], "intercept": coef[1],
            "slope_se": np.sqrt(cov[0, 0]),
            "intercept_se": np.sqrt(cov[1, 1]), "resid": resid,
            "rms": np.sqrt((resid ** 2).mean()), "n": len(x)}


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
    """Write a result; returns its path from the repo root. `inputs`
    names every result file this process has read, the result's own
    earlier file left out."""
    if "inputs" in quantities:
        raise ValueError("`inputs` is where a result names the results it read, not a quantity")
    p = path(stage, experiment, pair)
    p.parent.mkdir(parents=True, exist_ok=True)
    capture = Path(capture)
    own = rel(p).as_posix()
    doc = {"capture": str(rel(capture)), "notes": notes,
           "inputs": {name: _READ[name] for name in sorted(_READ) if name != own}, **quantities}
    p.write_text(json.dumps(_plain(doc), indent=2) + "\n")
    return rel(p)


def read_file(p):
    """A result file's document. The file is noted as read by this
    process, for `write` to name in the results the process writes; a
    file outside the repo is no result of record and is not noted."""
    data = Path(p).read_bytes()
    try:
        _READ[rel(p).as_posix()] = hashlib.sha1(data).hexdigest()
    except ValueError:
        pass
    return json.loads(data)


def read(stage, experiment, pair=None):
    return read_file(path(stage, experiment, pair))
