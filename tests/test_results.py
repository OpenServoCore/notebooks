"""The result files and the paths a notebook shows never carry where one
machine keeps the repo, and the small statistics the notebooks pack
into them."""
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import laps, results


def test_rel_strips_the_repo_root():
    assert results.rel(results.ROOT / "telemetry" / "x" / "capture-1") == Path("telemetry/x/capture-1")
    assert results.rel(Path("telemetry/x/capture-1")) == Path("telemetry/x/capture-1")


def test_write_returns_and_records_repo_relative_paths(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    out = results.write("0-instruments", "x", tmp_path / "telemetry" / "instruments" / "x" / "capture-1",
                        notes="", n=results.quantity(1, unit="count"))
    assert out == Path("stages/0-instruments/x/results.json")
    doc = results.read("0-instruments", "x")
    assert doc["capture"] == "telemetry/instruments/x/capture-1"
    assert "/" + "home" not in str(out) and not str(out).startswith("/")


def _round_trip(tmp_path, monkeypatch, **quantities):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    results.write("1-board", "x", tmp_path / "capture-1", **quantities)
    return results.read("1-board", "x")


def test_each_packer_round_trips_through_json(tmp_path, monkeypatch):
    fit = laps.line([0.0, 1.0, 2.0, 3.0], [1.0, 3.1, 4.9, 7.0])
    packed = {
        "a": results.q(1.5, unit="V"),
        "b": results.mean_sd([1.0, 2.0, 3.0, np.nan], "A"),
        "c": results.median_range([4.0, 1.0, 9.0], "ohm"),
        "d": results.pm(2.0, 0.5, "mV", scale=1e3),
        "e": results.line_q(fit, "slope", "counts/A"),
    }
    doc = _round_trip(tmp_path, monkeypatch, **packed)
    for key, value in packed.items():
        assert doc[key] == value
    assert doc["b"] == {"value": 2.0, "lo": 1.0, "hi": 3.0, "unit": "A"}
    assert doc["c"] == {"value": 4.0, "lo": 1.0, "hi": 9.0, "unit": "ohm"}
    assert doc["d"] == {"value": 2000.0, "lo": 1500.0, "hi": 2500.0,
                        "unit": "mV"}
    assert doc["e"]["lo"] == fit["slope"] - fit["slope_se"]
    assert results.median_range([4.0, 1.0], "", value=7.0)["value"] == 7.0


def test_nan_is_written_as_null(tmp_path, monkeypatch):
    doc = _round_trip(tmp_path, monkeypatch, a=results.q(np.nan, unit="V"),
                      rows=[{"x": np.nan, "y": np.float64(np.inf)}])
    assert doc["a"]["value"] is None
    assert doc["rows"] == [{"x": None, "y": None}]
    text = results.path("1-board", "x").read_text()
    assert "NaN" not in text and "Infinity" not in text


def test_numpy_scalars_and_bools_are_accepted(tmp_path, monkeypatch):
    doc = _round_trip(tmp_path, monkeypatch,
                      cycles=[{"k": np.int64(3), "turned": np.bool_(True),
                               "f": np.float32(0.5), "ok": True}],
                      table=np.array([1, 2]), span=(np.int32(1), 2.5))
    assert doc["cycles"] == [{"k": 3, "turned": True, "f": 0.5, "ok": True}]
    assert type(doc["cycles"][0]["k"]) is int
    assert doc["cycles"][0]["turned"] is True
    assert doc["table"] == [1, 2]
    assert doc["span"] == [1, 2.5]


def test_packers_equal_the_notebook_helpers_they_replace():
    """Bit for bit against the local helpers of bemf-coast and speed-torque
    (spread), rest-resistance (rng), identification (med_range),
    no-load-current (qse), motor-model (pm) and terminal-chain (se_q)."""
    rng = np.random.default_rng(7)
    x = pd.Series(rng.normal(3.0, 0.4, 41))
    x[5] = np.nan
    q = results.quantity

    def spread(x, unit, scale=1.0):
        x = pd.Series(x).dropna() * scale
        return q(x.mean(), x.mean() - x.std(), x.mean() + x.std(), unit)

    def rest_resistance_rng(x, unit, value=None):
        x = pd.Series(x).dropna()
        return q(x.median() if value is None else value, x.min(), x.max(),
                 unit)

    def med_range(x, unit):
        x = np.asarray(x, float)
        return q(np.median(x), x.min(), x.max(), unit)

    def qse(v, se, unit, scale=1.0):
        return q(v * scale, (v - se) * scale, (v + se) * scale, unit)

    def motor_model_pm(v, e, unit):
        return q(v, v - e, v + e, unit)

    def se_q(f, key, unit, scale=1):
        return q(f[key] * scale, (f[key] - f[key + "_se"]) * scale,
                 (f[key] + f[key + "_se"]) * scale, unit)

    fit = laps.line(np.arange(41.0), x.fillna(3.0))
    assert results.mean_sd(x, "V", 1e3) == spread(x, "V", 1e3)
    assert results.median_range(x, "V") == rest_resistance_rng(x, "V")
    clean = x.dropna().to_numpy()
    assert results.median_range(clean, "V") == med_range(clean, "V")
    assert results.pm(0.37, 0.011, "A", 1e3) == qse(0.37, 0.011, "A", 1e3)
    assert results.pm(0.37, 0.011, "A") == motor_model_pm(0.37, 0.011, "A")
    assert results.line_q(fit, "intercept", "V", 1e3) == se_q(
        fit, "intercept", "V", 1e3)


def test_mean_se_and_wmean_carry_the_standard_error():
    x = [1.0, 2.0, 3.0, np.nan]
    assert results.mean_se(x, "A") == results.pm(2.0, 1 / np.sqrt(3), "A")
    assert results.mean_se(x, "mA", 1e3)["value"] == 2000.0
    assert results.wmean(x) == (2.0, 1.0, 1 / np.sqrt(3), 3)
    assert results.wmean([4.0, np.nan])[::3] == (4.0, 1)
    assert np.isnan(results.wmean([4.0])[1:3]).all()
    empty = results.wmean([np.nan])
    assert np.isnan(empty[:3]).all() and empty[3] == 0


def test_fit_line_drops_non_finite_points_and_keeps_its_residuals():
    rng = np.random.default_rng(3)
    x = np.arange(30.0)
    y = 2.5 * x - 4.0 + rng.normal(0, 0.3, 30)
    fit = results.fit_line(np.r_[x, np.nan, 7.0], np.r_[y, 1.0, np.inf])
    ref = laps.line(x, y)
    for key in ("slope", "intercept", "slope_se", "intercept_se", "rms"):
        assert fit[key] == pytest.approx(ref[key], rel=1e-9)
    assert fit["n"] == 30
    assert fit["resid"] == pytest.approx(y - (fit["slope"] * x
                                              + fit["intercept"]))
