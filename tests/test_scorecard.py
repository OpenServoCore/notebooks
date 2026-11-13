"""The stage 5 scorecard on synthetic captures: a step with a known
overshoot and settle through pot spikes, a ramp with a known lag, the
tick unwrap across a wrap, the reversal count, the test set's schedules
and refusal, and the runner's refusals with `osc` stubbed, then one rung
end to end on a fake servo."""
import io
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from oscnb import boards, results, scorecard as sc, servos

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scorecard_synth as synth  # noqa: E402

CPD = 20.0
CENTER = 2000
GUARD = (300, 3700)
SERVO = servos.SERVOS["mg90-d"]
BOARD = boards.BOARDS[boards.DEFAULT]
TS = sc.testset()


def plan(segs):
    return {"counts_per_deg": CPD, "center_counts": CENTER, "guard_counts": list(GUARD),
            "segments": [{"seg": k, **s} for k, s in enumerate(segs)]}


def step_rows():
    """A move from the center to +20 deg: a rise past the goal by 40 counts
    over 1000 rows, 200 rows flat there, down to the goal one count per 50
    rows, then flat; single-row pot spikes on the rise, the top and the rest."""
    goal = CENTER + 400
    rise = np.linspace(CENTER, goal + 40, 1000)
    fall = goal + 40 - np.floor(np.arange(2000) / 50)
    pos = np.concatenate([rise, np.full(200, goal + 40), fall, np.full(26800, goal)])
    for k in (500, 1100, 1150, 5000):
        pos[k] += 300
    duty = np.zeros(len(pos), int)
    duty[:1000] = 600
    duty[1200:3200] = -600
    return goal, pos, duty


def test_a_step_reads_its_overshoot_and_settle_through_spikes(tmp_path):
    goal, pos, duty = step_rows()
    segs = [{"item": f"goal:{goal}@1500", "test": "step", "size_deg": 20, "direction": 1,
             "leg": "out", "goal_counts": goal, "goal_deg": 20.0}]
    synth.write(tmp_path, [(segs[0]["item"], goal, pos, duty)], CENTER, GUARD, plan=plan(segs))
    cap = sc.load(tmp_path)
    over, settle, span = sc.step_move(cap, cap["plan"]["segments"][0])
    assert over == pytest.approx(2.0)
    # the last row out of the 10-count band is the fall's row 1499
    assert settle == pytest.approx((1000 + 200 + 1500) / 20_000)
    s = sc.step_scores(cap)
    assert s["overshoot_deg_20deg"] == pytest.approx(2.0)
    assert s["settle_ms_20deg"] == pytest.approx(135.0)
    assert s["move_reversals_per_s_20deg"] == pytest.approx(1 / 0.135)
    i_ma = 600 * synth.I_PER_DUTY * BOARD.a_per_count * 1e3
    assert s["move_current_rms_ma_20deg"] == pytest.approx(i_ma * np.sqrt(2500 / 2700))


def test_a_ramp_reads_its_lag_and_its_real_speed(tmp_path):
    """Goals 10 counts apart every 512 ticks (a 400-row burst and a gap),
    the shaft 30 counts behind the goal in force, up and back down."""
    n = 50
    segs, rows = [], []
    for ramp, direction in ((0, 1), (1, -1)):
        base = CENTER if direction > 0 else CENTER + 10 * n
        for k in range(1, n + 1):
            goal = base + direction * 10 * k
            segs.append({"item": f"goal:{goal}@20", "test": "ramp", "speed_pct": 1,
                         "direction": direction, "ramp": ramp, "goal_counts": goal,
                         "goal_deg": (goal - CENTER) / CPD})
            rows.append((segs[-1]["item"], goal, np.full(400, goal - direction * 30), np.full(400, 100 * direction)))
        segs.append({"item": "hold@500", "test": "pause", "goal_counts": None, "goal_deg": None})
        rows.append(("hold@500", goal, np.full(10_000, goal), np.zeros(10_000, int)))
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs))
    f = sc.follow_scores(sc.load(tmp_path))
    assert f["lag_deg_1pct"] == pytest.approx(1.5)
    assert f["follow_error_sd_deg_1pct"] == pytest.approx(0.0, abs=1e-9)
    assert f["ramp_speed_deg_s_1pct"] == pytest.approx(10 / 0.0256 / CPD)
    assert f["follow_reversals_per_s_1pct"] == 0
    assert np.isfinite(f["crawl_speed_ripple_pct"])


def test_ticks_unwrap_across_the_u16_wrap(tmp_path):
    assert sc.unwrap_ticks([65_000, 964, 2464], [999, 999, 999]).tolist() == [65_000, 66_500, 68_000]
    segs = [(f"hold@{50}", CENTER, np.full(1000, CENTER), np.zeros(1000, int))] * 3
    synth.write(tmp_path, segs, CENTER, GUARD, tick0=65_000, gap_ticks=500,
                plan=plan([{"item": "hold@50", "test": "hold", "goal_counts": None}] * 3))
    rows = sc.load(tmp_path)["rows"]
    firsts = rows.groupby("seg")["t"].first().tolist()
    assert firsts == pytest.approx([0.0, 1500 / 20_000, 3000 / 20_000])


def test_reversals_count_sign_changes_across_zeros():
    assert sc.reversals([0, 5, 0, -3, -2, 0, 4, 4, 0]) == 2
    assert sc.reversals([0, 0, 0]) == 0
    assert sc.reversals([3, 0, 0, 3]) == 0


def test_window_speeds_and_ripple():
    t = np.arange(4000) / 20_000
    speeds = np.where(np.floor(t / 0.01) % 2 == 0, 100.0, 60.0)
    y = np.concatenate([[0], np.cumsum(speeds[:-1]) / 20_000])
    got = sc.window_speeds(t, y, 0.01)
    assert len(got) == 19
    assert sc.ripple_pct(got) == pytest.approx(np.std(got, ddof=1) / np.mean(got) * 100)
    assert np.mean(got) == pytest.approx(80, rel=0.02)


INP = {"counts_per_deg": CPD, "no_load_deg_s": 600.0, "current_cap_ma": 850.0,
       "current_cap_ms": 100, "sources": {}}


def test_steps_schedule_in_degrees():
    segs = TS.steps(CENTER, CPD)
    assert [s["item"] for s in segs[:2]] == ["hold@1000", f"goal:{CENTER}@1200"]
    moves = [s for s in segs if s["test"] == "step"]
    assert [s["goal_counts"] for s in moves[:4]] == [CENTER + 40, CENTER, CENTER - 40, CENTER]
    assert [s["goal_counts"] for s in moves[-4:]] == [CENTER + 1200, CENTER, CENTER - 1200, CENTER]
    assert {s["item"].split("@")[1] for s in moves if s["size_deg"] == 60} == {"2000"}
    assert [(s["direction"], s["leg"]) for s in moves[:4]] == [(1, "out"), (-1, "back"), (-1, "out"), (1, "back")]
    assert moves[-2]["goal_deg"] == pytest.approx(-60.0)


def test_follow_schedule_runs_at_the_asked_speed():
    segs = TS.follow(CENTER, CPD, 600.0)
    ramps = {}
    for s in segs:
        if s["test"] == "ramp":
            ramps.setdefault(s["ramp"], []).append(s)
    assert len(ramps) == 6
    up = [s["goal_counts"] for s in ramps[0]]
    # 1 % of 600 deg/s for 3 s is 18 deg, under the 30 deg span
    assert up[-1] == CENTER + 360
    assert len(up) == int(np.ceil(360 / (6 * CPD * TS.GOAL_PERIOD_S)))
    assert all(s["item"].endswith("@20") for s in ramps[0])
    down = [s["goal_counts"] for s in ramps[1]]
    assert down[-1] == CENTER and ramps[1][0]["direction"] == -1
    # 10 % reaches the 30 deg cap
    assert max(s["goal_counts"] for s in ramps[4]) == CENTER + 600
    assert sum(s["test"] == "pause" for s in segs) == 6


def test_the_set_fits_the_soft_range_or_is_refused():
    center, sets = TS.build(GUARD, INP)
    assert center == GUARD[0] + (GUARD[1] - GUARD[0]) // 2
    goals = [s["goal_counts"] for segs in sets.values() for s in segs if s["goal_counts"] is not None]
    assert GUARD[0] < min(goals) and max(goals) < GUARD[1]
    assert [s["item"] for s in sets["hold"]] == ["hold@3000", "hold@3000"]
    with pytest.raises(SystemExit, match="cannot fit"):
        TS.build((900, 3100), INP)


def test_inputs_come_through_the_chain():
    pair = "mg90-d__dev-v006-2A-D"
    inp = TS.inputs(SERVO, pair)
    pot = results.read("3-servo", "pot-table", pair)["pot_scale_counts_per_deg"]["value"]
    rev_s = results.read("3-servo", "run-up", pair)["no_load_speed_rev_s"]["value"]
    assert inp["counts_per_deg"] == pot
    assert inp["no_load_deg_s"] == pytest.approx(rev_s / SERVO.gear_ratio * 360)
    assert inp["current_cap_ma"] == pytest.approx(SERVO.lock_hold_a * 1000)
    assert not any(str(v).startswith("/") for v in inp["sources"].values())


@pytest.fixture
def bus(monkeypatch, tmp_path):
    """A servo at rest on a 3 Mbps bus at 5 V, every call logged; osc-lab
    track answered by the fake servo."""
    state = {"baud": 3_000_000, "torque": "off", "vsys": 5.05, "shaft": 1950.0,
             "lo": 314, "hi": 3560, "log": []}

    def get(field):
        return {"torque_enable": state["torque"], "pos_min_soft_counts": state["lo"],
                "pos_max_soft_counts": state["hi"]}[field]

    def track(out, mode, schedule, **kw):
        state["log"].append((Path(out).name, mode, kw))
        state["shaft"] = synth.fake_track(out, schedule, state["shaft"])
        return out

    monkeypatch.setattr(sc.osc, "bus_baud", lambda: state["baud"])
    monkeypatch.setattr(sc.osc, "get", get)
    monkeypatch.setattr(sc.bench, "read_vsys", lambda board=None: state["vsys"])
    monkeypatch.setattr(sc.osc, "track", track)
    monkeypatch.setattr(sc, "rung_dir", lambda pair, label: tmp_path / "telemetry" / pair / "scorecard" / label)
    monkeypatch.setattr(sc, "result_path", lambda pair, label: tmp_path / "results" / pair / f"{label}.json")
    return state


@pytest.mark.parametrize("change, match", [
    ({"baud": 1_000_000}, "baud"),
    ({"torque": "on"}, "torque is on"),
    ({"vsys": 7.4}, "rail reads 7.40 V"),
    ({"vsys": 4.6}, "rail reads 4.60 V"),
    ({"lo": 1200, "hi": 2800}, "cannot fit"),
])
def test_the_runner_refuses_before_anything_moves(bus, change, match):
    bus.update(change)
    with pytest.raises(SystemExit, match=match):
        sc.run("00-baseline", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert bus["log"] == []


def test_the_runner_refuses_an_existing_rung(bus):
    (sc.rung_dir(sc.pair_of(SERVO, BOARD), "00-baseline")).mkdir(parents=True)
    with pytest.raises(SystemExit, match="already holds a rung"):
        sc.run("00-baseline", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert bus["log"] == []


def test_a_rung_end_to_end(bus):
    rung = sc.run("00-baseline", "abc123", "f" * 40, runs=2, note="fake", servo=SERVO, board=BOARD)
    assert [(name, kw["deadman_check"]) for name, _, kw in bus["log"]] == [
        ("steps", True), ("follow", False), ("hold", False)] * 2
    assert all(mode == "position" and kw["current_cap_ma"] == pytest.approx(850) and kw["current_cap_ms"] == 100
               for _, mode, kw in bus["log"])
    doc = json.loads((rung / "rung.json").read_text())
    assert (doc["label"], doc["fw"], doc["note"]) == ("00-baseline", "abc123", "fake")
    assert doc["config_hash"] == sc.config_hash(synth.meta(0, (0, 0), 0)["config"])
    planned = json.loads((rung / "run-1" / "steps" / "plan.json").read_text())
    assert planned["segments"][2]["goal_deg"] == pytest.approx(2.0, abs=0.05)

    sc.score("00-baseline", servo=SERVO, board=BOARD)
    out = json.loads(sc.result_path(sc.pair_of(SERVO, BOARD), "00-baseline").read_text())
    assert out["runs"] == ["run-1", "run-2"] and out["skipped"] == []
    s = out["scores"]
    expected = {f"{b}_{n}deg" for b in ("overshoot_deg", "settle_ms", "move_current_rms_ma",
                                        "move_reversals_per_s", "move_duty_rms_pct")
                for n in TS.STEP_SIZES_DEG}
    expected |= {f"{b}_{p}pct" for b in ("lag_deg", "follow_error_sd_deg", "follow_current_rms_ma",
                                         "follow_reversals_per_s", "follow_duty_rms_pct",
                                         "ramp_speed_deg_s")
                 for p in TS.FOLLOW_SPEED_PCT}
    expected |= {"speed_ripple_pct", "rest_error_deg", "two_sided_deg", "enable_kick_deg",
                 "hold_jitter_counts", "hold_buzz_share", "hold_current_rms_ma", "crawl_speed_ripple_pct"}
    assert set(s) == expected
    assert all(set(q) == {"value", "lo", "hi", "unit"} for q in s.values())
    # the fake servo is first order: no overshoot, and it rests inside its deadband
    assert s["overshoot_deg_60deg"]["value"] == pytest.approx(0, abs=0.05)
    assert s["rest_error_deg"]["value"] < 4 / planned["counts_per_deg"] + 0.05
    assert s["lag_deg_1pct"]["value"] > 0
    assert s["ramp_speed_deg_s_10pct"]["value"] == pytest.approx(
        0.1 * planned["no_load_deg_s"], rel=0.1)

    buf = io.StringIO()
    sc.table(servo=SERVO, board=BOARD, file=buf)
    lines = buf.getvalue().splitlines()
    assert "00-baseline" in lines[0] and "others" in lines[0]
    lag = next(r for r in lines if r.startswith("lag_deg_10pct"))
    assert lag.rstrip().endswith("2.7 to 7")
    assert out["capture"] == "telemetry/mg90-d__dev-v006-2A-D/scorecard/00-baseline"


def test_a_settings_rung_applies_the_file_and_reboots_it_away(bus, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(sc.settings, "apply", lambda fields, note="": calls.append(("apply", dict(fields))))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append(("restore", len(bus["log"]))))
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}, "note": "softer"}))
    rung = sc.run("01-gains", "abc123", "f" * 40, runs=1, settings_file=f, servo=SERVO, board=BOARD)
    assert calls == [("apply", {"p_kp_q88": 5026}), ("restore", 3)]
    assert json.loads((rung / "rung.json").read_text())["settings"] == {"p_kp_q88": 5026}


def test_a_settings_rung_reboots_after_an_abort(bus, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(sc.settings, "apply", lambda fields, note="": calls.append("apply"))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append("restore"))

    def abort(out, mode, schedule, **kw):
        raise RuntimeError("aborted")

    monkeypatch.setattr(sc.osc, "track", abort)
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    with pytest.raises(RuntimeError, match="aborted"):
        sc.run("01-gains", "abc123", "f" * 40, runs=1, settings_file=f, servo=SERVO, board=BOARD)
    assert calls == ["apply", "restore"]
