"""The result files and the paths a notebook shows never carry where one
machine keeps the repo."""
from pathlib import Path

from oscnb import results


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
