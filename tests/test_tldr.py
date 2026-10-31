"""The TL;DR constants table of every stage README is what the results files
say, so a re-recorded result cannot leave a stale number behind."""
import json

import pytest

from oscnb import results, tldr


@pytest.mark.parametrize("stage", tldr.stages())
def test_readme_table_matches_the_results(stage):
    readme = tldr.stage_dir(stage) / "README.md"
    assert tldr.render(stage) == readme.read_text(), \
        f"stages/{stage}/README.md is stale, run `uv run python -m oscnb.tldr`"


@pytest.mark.parametrize("stage", tldr.stages())
def test_every_row_links_a_notebook_that_exists(stage):
    for row in tldr.spec(stage).get("constant", []):
        nb = row.get("notebook", f"{row['experiment']}/{row['experiment']}.ipynb")
        assert (tldr.stage_dir(stage) / nb).is_file(), nb


def test_stage_zero_has_a_spec():
    assert "0-instruments" in tldr.stages()


@pytest.fixture
def fake(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    folder = tmp_path / "stages" / "9-x" / "e"
    folder.mkdir(parents=True)
    (folder / "results.json").write_text(json.dumps({
        "capture": "telemetry/x", "notes": "",
        "lag_s": results.quantity(0.020198, -0.0813, 0.2023, "s"),
        "plain": results.quantity(5.0916, unit="ohm"),
        "rate": results.quantity(1.0000089, 0.9999162, 1.0001016),
        "acc": {"pct": results.quantity(2.0, unit="% of reading"), "fixed": results.quantity(0.025, unit="A")},
    }))
    readme = tmp_path / "stages" / "9-x" / "README.md"
    readme.write_text(f"# x\n\n{tldr.BEGIN}\nold\n{tldr.END}\n\nafter\n")

    def write_spec(text):
        (tmp_path / "stages" / "9-x" / "tldr.toml").write_text(text)
    return write_spec, readme


def row(**kw):
    base = {"label": "x", "experiment": "e", "confidence": "measured"}
    base.update(kw)
    lines = ["[[constant]]"]
    for k, v in base.items():
        lines.append(f"{k} = {json.dumps(v)}")
    return "\n".join(lines) + "\n"


def test_render_fills_value_bounds_and_unit_from_the_results(fake):
    write_spec, readme = fake
    write_spec(row(key="lag_s", unit="ms", scale=1e3, sig=2, bounds="spread")
               + row(key="plain", decimals=3)
               + row(key="rate", unit="ppm", subtract=1.0, scale=1e6, decimals=0, signed=True, bounds="statistical")
               + row(key="acc", **{"class": True}, unit="mA", scale=1e3, bounds="class"))
    out = tldr.render("9-x")
    assert out.startswith("# x\n\n" + tldr.BEGIN + "\n") and out.endswith(tldr.END + "\n\nafter\n")
    assert "| x | 20 | -81 to 202, spread | ms | measured | [e](e/e.ipynb) |" in out
    assert "| x | 5.092 | - | ohm |" in out
    assert "| x | +9 | -84 to +102, statistical | ppm |" in out
    assert "| x | 2 % of reading + 25 | class | mA |" in out
    assert "old" not in out
    assert tldr.main([]) == 0 and readme.read_text() == out
    assert tldr.main(["--check"]) == 0


def test_check_fails_on_a_stale_readme(fake):
    write_spec, readme = fake
    write_spec(row(key="plain", decimals=3))
    assert tldr.main(["--check"]) == 1


@pytest.mark.parametrize("spec", [
    row(key="missing", sig=2),                       # no such key
    row(key="lag_s", sig=2),                         # bounds in the result, none said
    row(key="plain", sig=2, bounds="spread"),        # bounds said, none in the result
    row(key="lag_s", unit="ms", sig=2, bounds="x"),  # a new unit with no scale
])
def test_spec_errors_are_refused(fake, spec):
    write_spec, _ = fake
    write_spec(spec)
    with pytest.raises((KeyError, ValueError)):
        tldr.render("9-x")
