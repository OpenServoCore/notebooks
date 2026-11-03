"""The stage 3 plans against a bench of stubs: the servo states discover
their plans and run them in order, the caps refuse before anything moves,
the seat check stops the seated work, the windows stay on the runway, and
the synthetic servo's datasets load through the library."""
import csv
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, capture, results, servos, sitting, tel
from oscnb.sitting import Plan, Sitting

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stages"
                       / "2-motor" / "motor-model"))
import motor_model as mm  # noqa: E402
import servo_synth  # noqa: E402

GROUP = "mg90-d__dev-v006-2A-D"
STAGES = Path(__file__).resolve().parents[1] / "stages"
FAST_MS = 40          # a stubbed sweep's steps cut to this, the pilot's kept
FLOOR_V = 0.4         # the ladder floor a stubbed result file reports


class Bench:
    """Every bench call recorded, the servo answered by the synthetic model;
    a sweep writes rows only where a plan reads them back."""

    def __init__(self, model):
        self.model = model
        self.log = []
        self.rail = None
        mid = round(float(np.mean(model.soft)))
        self.regs = {"pos_min_soft_counts": model.soft[0],
                     "pos_max_soft_counts": model.soft[1],
                     "raw_min": model.stops[0], "raw_max": model.stops[1],
                     "ntc_raw": 2139, "vbus_raw": 1240, "pos": mid}
        self.positions = []           # pos readings to hand out first

    def get(self, field):
        if field == "pos" and self.positions:
            return self.positions.pop(0)
        return self.regs[field]

    def sweep(self, out, duty, **kw):
        seat = ("lock" if kw.get("static_load") else
                "stop" if kw.get("stall") else None)
        self.log.append(("sweep", tuple(duty), kw.get("dirs"), seat))
        out.mkdir(parents=True, exist_ok=True)
        pilot = out.name.endswith("pilot")
        if pilot or seat == "stop" or out.parent.name == "holds":
            steps = duty if pilot else [cut(st) for st in duty]
            guards = (kw["guard_lo"], kw["guard_hi"]) if "guard_lo" in kw \
                else None
            servo_synth.sweep(out, self.model, self.rail, steps,
                              dirs=kw.get("dirs", "both"), seat=seat,
                              guards=guards)
        if kw.get("background"):
            self.log.append(("start",))
            return Proc(self.log)
        return out


class Proc:
    def __init__(self, log):
        self.log, self.returncode = log, 0

    def wait(self):
        self.log.append(("wait",))
        return 0


def cut(step):
    """A schedule entry with its window cut to FAST_MS."""
    head, at, ms = str(step).partition("@")
    if head.startswith(("coast:", "brake:")):
        return f"{head.split(':')[0]}:{min(int(head.split(':')[1]), FAST_MS)}"
    return f"{head}@{min(int(ms), FAST_MS)}" if at else head


@pytest.fixture
def bench(monkeypatch, tmp_path):
    b = Bench(servo_synth.Model())
    real_read = results.read

    def read(stage, experiment, pair=None):
        if stage == "3-servo" and experiment == "ladder":
            return {"floor_v_effective": results.quantity(FLOOR_V, unit="V")}
        return real_read(stage, experiment, pair)

    def set_vsys(v):
        b.rail = v
        b.log.append(("rail", v))
        return v + 0.25, v

    monkeypatch.setattr(sitting.bench, "power_up",
                        lambda v: b.log.append(("power_up", round(v, 2))))
    monkeypatch.setattr(sitting.bench, "power_down",
                        lambda: b.log.append(("power_down",)))
    monkeypatch.setattr(sitting.bench, "set_vsys", set_vsys)
    monkeypatch.setattr(sitting.bench, "set_volts",
                        lambda v, hold_s=2.0: (b.log.append(("dps", v)), v)[1])
    monkeypatch.setattr(sitting.osc, "sweep", b.sweep)
    monkeypatch.setattr(sitting.osc, "bursts", lambda out, pct, **kw: (
        b.log.append(("bursts", tuple(pct), kw.get("dirs"))),
        out.mkdir(parents=True, exist_ok=True))[1])
    monkeypatch.setattr(sitting.osc, "get", b.get)
    monkeypatch.setattr(sitting.osc, "clapper", lambda path, flashes: (
        b.log.append(("clapper", flashes)), path)[1])
    monkeypatch.setattr(sitting.capture, "prompt",
                        lambda text: b.log.append(("prompt", text)))
    monkeypatch.setattr(sitting.capture, "hold",
                        lambda sec, text="": b.log.append(("hold", sec)))
    monkeypatch.setattr(sitting.capture, "ROOT", tmp_path)
    monkeypatch.setattr(sitting.servos, "selected",
                        lambda: servos.SERVOS["mg90-d"])
    monkeypatch.setattr(results, "read", read)
    monkeypatch.setattr("time.sleep", lambda s: None)
    return b


def manifest(tmp_path, state, n=1):
    path = tmp_path / "telemetry" / GROUP / "sittings" / f"{state}-{n}.csv"
    return list(csv.DictReader(open(path)))


def datasets(rows):
    """experiment/name of every dataset a manifest records, in order."""
    out = []
    for r in rows:
        if r["event"] == "dataset":
            path = r["detail"].split(": ")[1]
            out.append("/".join(Path(path).parts[-2:]))
    return out


def test_servo_states_discover_their_plans():
    found = sitting.discover()
    by_state = {s: {(p.experiment, p.name) for p in found if p.state == s}
                for s in sitting.SERVO_STATES}
    assert {("resistance", "locked"), ("resistance", "stop"),
            ("inductance", "locked"), ("ladder", "capture"),
            ("ladder", "low"), ("run-up", "runup"),
            ("steps", "capture"), ("sag", "capture")} <= by_state["servo"]
    assert {("pot-table", "pass"), ("pot-table", "crawl")} <= \
        by_state["servo-camera"]
    assert {("load", "holds"), ("load", "lift")} <= by_state["servo-load"]
    assert all(p.lock for p in found if p.name == "locked")
    assert all(p.bracket for p in found
               if (p.experiment, p.name) in {("run-up", "runup"),
                                             ("ladder", "ref"),
                                             ("pot-table", "pass")})


def test_servo_state_runs_in_order_with_the_lock(bench, tmp_path):
    sitting.run("servo", 1, plans=sitting.discover(STAGES))
    rows = manifest(tmp_path, "servo")
    assert datasets(rows) == [
        "run-up/runup-open-1", "ladder/ref-open-1",
        "resistance/locked-1", "inductance/locked-1",
        "resistance/stop-1", "ladder/capture-1", "ladder/low-1",
        "steps/capture-1", "steps/reversal-1",
        "ladder/ref-close-1", "run-up/runup-close-1"]
    events = [(r["event"], r["detail"]) for r in rows]
    lock = [i for i, e in enumerate(events) if e[0] == "lock"]
    assert [events[i][1] for i in lock] == ["on", "off"]
    at = {d.split(": ")[1]: i for i, (e, d) in enumerate(events)
          if e == "dataset"}
    locked = at[f"telemetry/{GROUP}/resistance/locked-1"]
    stop = at[f"telemetry/{GROUP}/resistance/stop-1"]
    assert lock[0] < locked < lock[1] < stop
    assert ("prompt", sitting.LOCK_ON) in bench.log
    assert sum(1 for e, _ in events if e == "probe") >= sum(
        1 for e in bench.log if e[0] in ("sweep", "bursts"))
    assert bench.log[-1] == ("power_down",)


def test_camera_and_load_states_run(bench, tmp_path):
    sitting.run("servo-camera", 2, plans=sitting.discover(STAGES))
    names = datasets(manifest(tmp_path, "servo-camera", 2))
    assert names == ["pot-table/pass-open-2", "pot-table/crawl-2",
                     "pot-table/reversal-2", "pot-table/steps-2",
                     "pot-table/pass-close-2"]
    clips = [e for e in bench.log if e[0] == "clapper"]
    starts = [e for e in bench.log if e[0] == "prompt"
              and e[1].startswith("Start the phone")]
    assert len(clips) == len(starts) == 10
    sitting.run("servo-load", 3, plans=sitting.discover(STAGES))
    rows = manifest(tmp_path, "servo-load", 3)
    assert datasets(rows) == ["load/holds-3", "load/lift-3", "load/steps-3",
                              "load/crawl-3"]
    holds = pd.read_csv(tmp_path / "telemetry" / GROUP / "load" / "holds-3"
                        / "holds.csv")
    assert len(holds["angle_counts"].unique()) == 7
    assert not [r for r in rows if r["event"] == "skip"
                and "load/steps" in r["detail"]]


def test_a_free_rung_over_the_rating_refuses(bench):
    over = Plan("over", "x", "servo", "", run=lambda s, out: s.servo_cycles(
        out, [(5.0, 30), (7.4, 80)], 300, 300))
    with pytest.raises(SystemExit, match="rated"):
        sitting.run("servo", 1, plans=[over])
    assert not [e for e in bench.log if e[0] == "sweep"]   # not even the pilot


def test_a_locked_lap_over_the_hold_refuses(bench):
    s = Sitting("servo", 1)
    hold = s.servo.lock_hold_a
    assert 4.2 / s.r_type > hold > 3.7 / s.r_type   # stage 2's third rail
    lap = Plan("lap", "x", "servo", "", lock=True, run=lambda s, out: s.drive(
        out, 4.2, ["100@300"], seat="lock"))
    with pytest.raises(SystemExit, match=f"{hold} A"):
        sitting.run("servo", 1, plans=[lap])
    assert not [e for e in bench.log if e[0] == "sweep" and e[3] == "lock"]
    laps = [e for e in bench.log if e[0] == "sweep"]
    sitting.run("servo", 1, plans=sitting.discover(STAGES),
                only=("resistance/locked",), skip=("runup", "ref"))
    locked = [e for e in bench.log[len(laps):] if e[0] == "sweep"
              and e[3] == "lock"]
    assert [e[1] for e in locked] == [("100@300",)] * 6
    stop = Plan("stop", "x", "servo", "", run=lambda s, out: s.drive(
        out, 5.0, ["42@300"], dirs="rev", seat="stop"))
    with pytest.raises(SystemExit, match=f"{s.servo.lap_cap_a} A"):
        sitting.run("servo", 1, plans=[stop])   # 0.45 A of a 0.5 A hold


def test_the_seat_check_flags_a_moved_seat(bench, tmp_path):
    mid = bench.regs["pos"]
    # the centering sweep, centered, the lock on, before the first chain,
    # its sweep, then after it moved
    moved = mid + 3 * sitting.SEAT_SLIP_COUNTS
    bench.positions = [mid] * 5 + [moved]
    sitting.run("servo", 1, plans=sitting.discover(STAGES),
                only=("resistance/locked", "resistance/stop"),
                skip=("runup", "ref"))
    rows = manifest(tmp_path, "servo")
    skips = [r["detail"] for r in rows if r["event"] == "skip"]
    assert any("resistance/locked" in d and "moved" in d for d in skips)
    assert any("resistance/stop" in d for d in skips)        # seated, left out
    assert "resistance/stop-1" not in datasets(rows)
    seats = pd.read_csv(tmp_path / "telemetry" / GROUP / "resistance"
                        / "locked-1" / "seats.csv")
    assert seats["shift_counts"].abs().max() > sitting.SEAT_SLIP_COUNTS


def test_windows_scale_from_the_pilot_and_stay_on_the_runway(bench, tmp_path):
    def windows(model):
        bench.model = model
        s = Sitting("servo", 1)
        s.open()
        out = {}
        for v in (3.7, 5.0, 7.4):
            for d in (15, 30, 60, 67, 100):
                if v * d / 100 > s.rated_v:
                    continue
                try:
                    (ms,) = s.fit(v, [(d, 1200)])
                except sitting.Skip:
                    continue
                out[(v, d)] = ms
                bound = s.speed_bound(v * d / 100)
                travel = bound * (ms / 1000 + sitting.COAST_TAU_S)
                assert travel <= s.runway()
                true = abs(model.steady_rev_s(v * d / 100))
                assert true * model.counts_per_motor_rev <= bound
        s.close()
        return out, s

    base, s = windows(servo_synth.Model())
    slow, _ = windows(servo_synth.Model(coulomb_a=0.1, breakaway_a=0.12,
                                        viscous_a_per_rev_s=8e-4))
    assert all(slow[k] >= base[k] for k in base if k in slow)
    assert any(slow[k] > base[k] for k in base if k in slow)
    assert base[(5.0, 100)] < 200          # full speed leaves less than wanted
    # the model itself driven at the sized window stops short of the far guard
    lo, hi = s.guards()
    folder = servo_synth.sweep(tmp_path / "check", servo_synth.Model(), 7.4,
                               [f"60@{base[(7.4, 60)]}", "coast:400"],
                               dirs="fwd", guards=(lo, hi))
    df, _ = tel.load(folder)
    assert df["pos"].max() < hi


def test_the_runup_bracket_writes_open_and_close(bench, tmp_path):
    sitting.run("servo", 4, plans=sitting.discover(STAGES), only=("runup",),
                skip=("ref",))
    names = datasets(manifest(tmp_path, "servo", 4))
    assert names == ["run-up/runup-open-4", "run-up/runup-close-4"]
    for side in ("open", "close"):
        cyc = pd.read_csv(tmp_path / "telemetry" / GROUP / "run-up"
                          / f"runup-{side}-4" / "cycles.csv")
        assert list(cyc["duty_pct"]) == [100] * 3
        assert (cyc["spin_ms"] <= 200).all()


def test_the_sag_plan_steps_the_dps_with_torque_off(bench):
    sitting.run("servo", 1, plans=sitting.discover(STAGES),
                only=("sag/capture",), skip=("runup", "ref"),
                include=("sag/capture",))
    log = bench.log
    assert not [e for e in log if e[0] in ("start", "dps")]
    sweeps = [e for e in log if e[0] == "sweep"]
    assert not [e for e in sweeps if any("hold:" in st for st in e[1])]
    rails = [e[1] for e in log if e[0] == "rail"]
    sag = [v for v in rails if v >= 6.0]
    assert sag[0] == 8.4 and sag[-1] == 6.0 and sag == sorted(sag,
                                                              reverse=True)
    for i, e in enumerate(log):                 # a rail change between sweeps
        if e[0] == "rail" and i:
            assert log[i - 1][0] != "start"
    bench.log.clear()
    sitting.run("servo", 2, plans=sitting.discover(STAGES),
                only=("sag/held",), skip=("runup", "ref"),
                include=("sag/held",))
    kinds = [e[0] for e in bench.log]
    start, wait = kinds.index("start"), kinds.index("wait")
    steps = [i for i, k in enumerate(kinds) if k == "dps"]
    assert len(steps) == 12 and start < steps[0] and steps[-1] < wait


def test_the_group_is_servo_and_board(monkeypatch, tmp_path):
    monkeypatch.setattr(capture, "ROOT", tmp_path)
    monkeypatch.delenv("OSC_SERVO", raising=False)
    assert Sitting("servo", 1).group == GROUP
    assert Sitting("servo-load", 1).motor.key == "mg90-d"
    assert Sitting("free", 1).group.endswith("__dev-v006-2A-D")
    monkeypatch.setenv("OSC_SERVO", "mg90-x")
    with pytest.raises(SystemExit, match="OSC_SERVO"):
        Sitting("servo", 1)
    with pytest.raises(ValueError, match="horn lock"):
        Plan("x", "x", "servo-load", "", lock=True)


def test_the_synthetic_sitting_loads_through_the_library(tmp_path):
    model = servo_synth.Model()
    group = servo_synth.write_sitting(tmp_path, model)
    board = boards.BOARDS[boards.DEFAULT]
    tables = sorted(group.glob("*/*/cycles.csv"))
    assert {p.parent.parent.name for p in tables} == {"ladder", "run-up"}
    for table in tables:
        cyc = pd.read_csv(table)
        for _, c in cyc.iterrows():
            df, meta = tel.load(table.parent / c["folder"], board)
            assert meta["sense"] == board.sense
            assert set(df["seg"]) >= {0, 1, 2, 3, 4}
    cyc = pd.read_csv(group / "ladder" / "capture-1" / "cycles.csv")
    assert set(cyc["volts_set"]) == set(servo_synth.RAILS)
    c = cyc[(cyc["volts_set"] == 5.0) & (cyc["duty_pct"] == 40)].iloc[0]
    df, _ = tel.load(group / "ladder" / "capture-1" / c["folder"], board)
    spin = df[df["seg"] == 1]
    tail = spin.iloc[len(spin) // 2:]
    rev_s = model.steady_rev_s(2.0)
    f, _ = mm.count_cycles(tail["current_raw"].to_numpy(float),
                           model.ripple_per_rev * rev_s)
    assert f / model.ripple_per_rev == pytest.approx(rev_s, rel=0.01)
    pos_rev_s = sitting.drive_speed(group / "ladder" / "capture-1"
                                    / c["folder"]) / model.counts_per_motor_rev
    assert pos_rev_s == pytest.approx(rev_s, rel=0.03)
    zero = tel.baseline(df)["current_raw"].median()
    assert tel.dropout_rows(tail["current_raw"] - zero).sum() > 0
    coast = df[df["seg"] == 2]
    emf = board.diff_v(coast["vmotor_a"], coast["vmotor_b"])
    assert emf[:20].mean() == pytest.approx(
        model.ke_v_per_rev_s * rev_s, rel=0.05)
    seats = pd.read_csv(group / "resistance" / "stop-1" / "seats.csv")
    stop_pos = sitting.seat_pos(group / "resistance" / "stop-1" / "chains"
                                / "0")
    assert abs(stop_pos[-1] - seats["pos"].iloc[0]) < 5
    rows = list(csv.DictReader(open(group / "sittings" / "servo-1.csv")))
    assert rows[0]["event"] == "start" and rows[-1]["event"] == "end"
    assert (group / "synthetic.json").exists()
