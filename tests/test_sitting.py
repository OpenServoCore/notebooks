"""The sitting runner against a bench of stubs: the plans it discovers, the
order it runs them in, the rules it enforces and the manifest it writes."""
import csv

import pytest

from oscnb import capture, motors, sitting
from oscnb.sitting import Plan, Sitting


@pytest.fixture
def bench(monkeypatch, tmp_path):
    """Every bench call recorded, no hardware, no prompts, no waiting."""
    log = []
    monkeypatch.setattr(sitting.bench, "power_up", lambda v: log.append(("power_up", round(v, 2))))
    monkeypatch.setattr(sitting.bench, "power_down", lambda: log.append(("power_down",)))
    monkeypatch.setattr(sitting.bench, "set_vsys", lambda v: (log.append(("rail", v)), (v + 0.25, v))[1])
    monkeypatch.setattr(sitting.osc, "sweep", lambda out, duty, **kw: (log.append(("sweep", tuple(duty), kw.get("dirs"))), out.mkdir(parents=True, exist_ok=True))[1])
    monkeypatch.setattr(sitting.capture, "prompt", lambda text: log.append(("prompt", text)))
    monkeypatch.setattr(sitting.capture, "hold", lambda seconds, text="": log.append(("hold", seconds)))
    monkeypatch.setattr(sitting.capture, "ROOT", tmp_path)
    monkeypatch.setattr(sitting.motors, "selected", lambda: motors.MOTORS["mg90-b"])
    return log


def plans():
    return [
        Plan("ref", "x", "free", "", run=lambda s, out: s.cycles(out, [(5.0, 30)], 100, 100, repeats=1), order=0, bracket=True),
        Plan("ladder", "x", "free", "", run=lambda s, out: s.cycles(out, [(5.0, 60)], 100, 100, repeats=1), order=10),
        Plan("extra", "x", "free", "", run=lambda s, out: s.cycles(out, [(7.4, 20)], 100, 100, repeats=1), order=20, optional=True),
        Plan("hot", "x", "free", "", run=lambda s, out: s.cycles(out, [(7.4, 75)], 100, 100, repeats=1, over_rating=True), order=5, over_rating=True),
    ]


def test_select_order_and_flags():
    chosen = sitting.select(plans(), "free")
    assert [p.name for p in chosen] == ["ref", "ladder"]            # optional and over-rating plans wait to be named
    chosen = sitting.select(plans(), "free", include=("extra", "hot"))
    assert [p.name for p in chosen] == ["ref", "ladder", "extra", "hot"]   # the over-rating plan last whatever its order
    assert [p.name for p in sitting.select(plans(), "free", only=("ladder",))] == ["ref", "ladder"]   # the bracket stays
    assert [p.name for p in sitting.select(plans(), "free", skip=("ref",))] == ["ladder"]
    with pytest.raises(SystemExit):
        sitting.select(plans(), "free", only=("nothing",))


def test_free_sitting_brackets_and_manifest(bench, tmp_path):
    sitting.run("free", 3, plans=plans())
    events = [e for e in bench if e[0] in ("prompt", "power_up", "sweep", "power_down")]
    assert events[0][0] == "prompt" and events[1] == ("power_up", 5.25)
    assert [e[1] for e in events if e[0] == "sweep"] == [("30@100", "coast:100"), ("60@100", "coast:100"), ("30@100", "coast:100")]
    assert events[-1] == ("power_down",)
    group = tmp_path / "telemetry" / "mg90-b__dev-v006-2A-D"
    assert sorted(p.name for p in (group / "x").iterdir()) == ["ladder-3", "ref-close-3", "ref-open-3"]
    rows = list(csv.DictReader(open(group / "sittings" / "free-3.csv")))
    assert [r["event"] for r in rows][:2] == ["start", "prompt"]
    assert {r["event"] for r in rows} >= {"dps", "rail", "dataset", "plan", "end"}
    assert rows[-1]["event"] == "end"


def test_rated_voltage_is_enforced(bench):
    over = Plan("over", "x", "free", "", run=lambda s, out: s.cycles(out, [(7.4, 80)], 100, 100, repeats=1), order=10)
    with pytest.raises(SystemExit, match="rated"):
        sitting.run("free", 1, plans=[over])
    assert not [e for e in bench if e[0] == "sweep"]                 # refused before anything ran
    kicked = Plan("kick", "x", "free", "", run=lambda s, out: s.cycles(out, [(7.4, 6)], 100, 100, repeats=1, kick={7.4: 70}), order=10)
    with pytest.raises(SystemExit, match="rated"):                   # the kick counts as the spin's duty
        sitting.run("free", 1, plans=[kicked])


def test_locked_sitting_walks_the_holes(bench, monkeypatch, tmp_path):
    monkeypatch.setattr(sitting, "stall_check", lambda s: s.note("check", "stubbed"))
    seen = []
    p = Plan("laps", "y", "locked", "", setup=lambda s, out: seen.append("setup"), at_hole=lambda s, out, k: seen.append(k),
             tail=lambda s, out: seen.append("tail"), finish=lambda s, out: seen.append("finish"))
    sitting.run("locked", 1, plans=[p])
    assert seen == ["setup", *sitting.HOLES, "tail", "finish"]
    prompts = [e[1] for e in bench if e[0] == "prompt"]
    assert len(prompts) == len(sitting.HOLES)                        # the entry prompt, then one per bar move
    assert "hole 11" in prompts[-1]
    rows = list(csv.DictReader(open(tmp_path / "telemetry" / "mg90-b__dev-v006-2A-D" / "sittings" / "locked-1.csv")))
    assert [r["detail"] for r in rows if r["event"] == "hole"] == [str(k) for k in sitting.HOLES]


def test_polarity_check_refuses_a_reversed_meter(bench, tmp_path, monkeypatch):
    class Log:
        def __init__(self, path, sign):
            self.path, self.sign = path, sign

        def __enter__(self):
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_host", "function", "value", "raw"])
                for i in range(6):
                    w.writerow([i, "DC A", self.sign * (0.05 if i else 0.0), ""])

        def __exit__(self, *exc):
            pass

    class Meter:
        def __init__(self, sign):
            self.sign = sign

        def log(self, path, expect=None):
            return Log(path, self.sign)

    s = Sitting("meter", 1)
    s.open()
    s.meter = Meter(-1)
    with pytest.raises(SystemExit, match="red toward MOT_A"):
        sitting.polarity_check(s)
    s.meter = Meter(+1)
    sitting.polarity_check(s)
    s.close()


def test_discover_finds_the_stage_2_plans():
    found = sitting.discover()
    names = {(p.experiment, p.state, p.name) for p in found}
    assert {("bemf-coast", "free", "ref"), ("bemf-coast", "meter", "meter"), ("speed-torque", "free", "capture"),
            ("rest-resistance", "locked", "capture"), ("inductance", "locked", "capture")} <= names
    assert all(p.bracket for p in found if p.name == "ref")
    assert all(p.over_rating for p in found if p.name.startswith("threshold"))
