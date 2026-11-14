"""The stage 5 scorecard on synthetic captures: a step with a known
overshoot and settle through pot spikes, a ramp with a known lag, the
tick unwrap across a wrap, the reversal count, the test set's schedules
and refusal, the hold rows past a torque-on head, a landing's drive
after arrival and the landings that hunt, the crawl's swing inside a
goal item, the servo's speed and position
reports against the pot, and the runner's refusals with
`osc` stubbed, then one rung end to end on a fake servo; a rung on a
position table, its install's place in the runner, its rows read
through the table, its set planned at the table's scale and center, its
position report gated on the servo's band, an identity rung's plan
rebuilt byte for byte and a table rung of record scored as recorded; an A/B of two rungs, its flashes, settings and
refusals, and the flash's command line; the wide capture the set builds
from a board's wide-code result, its three rows, a rung with and without
it, and a wide-codes rung's rows read through the board's slots."""
import hashlib
import io
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, postable, results, scorecard as sc, servos
from oscnb import widecodes as wc

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


def test_a_landing_reads_the_share_of_its_tail_driven(tmp_path):
    goal, pos, duty = step_rows()
    # the last 500 ms is 10000 rows: 1000 of them driven
    duty[-4000:-3000] = 90
    segs = [{"item": f"goal:{goal}@1500", "test": "step", "size_deg": 20, "direction": 1,
             "leg": "out", "goal_counts": goal, "goal_deg": 20.0}]
    synth.write(tmp_path, [(segs[0]["item"], goal, pos, duty)], CENTER, GUARD, plan=plan(segs))
    assert sc.landing_buzz_share(sc.load(tmp_path)) == pytest.approx(0.1, abs=0.001)


def hold_seg(ms, test="hold"):
    return {"item": f"hold@{ms}", "test": test, "goal_counts": None, "goal_deg": None}


def holds(tmp_path, drives, kick_rows=0):
    """Two 1500 ms holds at the center, a count of jitter on each; `drives`
    are (hold, first row, last row) spans driven, and the first hold's first
    `kick_rows` swing the pot 20 counts either side, driven."""
    rows = []
    for k in range(2):
        pos = CENTER + (np.arange(30_000) % 3 - 1).astype(float)
        duty = np.zeros(30_000, int)
        if k == 0 and kick_rows:
            pos[:kick_rows] = CENTER + 20 * np.sin(np.arange(kick_rows) / 400)
            duty[:kick_rows] = 600
        for h, a, b in drives:
            if h == k:
                duty[a:b] = 600
        rows.append(("hold@1500", CENTER, pos, duty))
    return sc.load(synth.write(tmp_path, rows, CENTER, GUARD, plan=plan([hold_seg(1500)] * 2)))


def test_a_kick_in_the_head_of_the_first_hold_is_left_to_the_enable_rows(tmp_path):
    quiet = holds(tmp_path / "quiet", [])
    kicked = holds(tmp_path / "kicked", [], kick_rows=6000)
    assert sc.hold_scores(kicked) == sc.hold_scores(quiet)
    assert sc.hold_scores(quiet)["hold_buzz_share"] == 0
    assert sc.enable_drive_ms(quiet, "hold") == 0
    # the last driven row is row 5999, read to its end
    assert sc.enable_drive_ms(kicked, "hold") == pytest.approx(300.0)


def test_the_enable_segment_reads_its_drive_over_the_head_only(tmp_path):
    pos = np.full(20_000, CENTER)
    duty = np.zeros(20_000, int)
    duty[1000:4000] = 600
    duty[14_000:15_000] = 600
    cap = sc.load(synth.write(tmp_path, [("hold@1000", CENTER, pos, duty)], CENTER, GUARD,
                              plan=plan([hold_seg(1000, "enable")])))
    assert sc.enable_drive_ms(cap, "enable") == pytest.approx(200.0)


@pytest.mark.parametrize("hold, a, b, scored", [(0, 12_000, 13_000, 20_000), (1, 0, 1000, 30_000)])
def test_a_hold_driven_after_the_head_shows_in_the_hold_rows(tmp_path, hold, a, b, scored):
    cap = holds(tmp_path, [(hold, a, b)])
    s = sc.hold_scores(cap)
    assert s["hold_buzz_share"] == pytest.approx((b - a) / scored)
    i_ma = 600 * synth.I_PER_DUTY * BOARD.a_per_count * 1e3
    assert s["hold_current_rms_ma"] == pytest.approx(i_ma * np.sqrt((b - a) / scored), rel=0.01)
    assert sc.enable_drive_ms(cap, "hold") == 0


def goal_seg(goal):
    return {"item": f"goal:{goal}@1500", "test": "step", "size_deg": 20, "direction": 1,
            "leg": "out", "goal_counts": goal, "goal_deg": (goal - CENTER) / CPD}


@pytest.mark.parametrize("hunting", [0, 1, 2])
def test_landing_tail_driven_counts_the_landings_that_hunt(tmp_path, hunting):
    """Four landings held flat, their tails the last 10000 rows: `hunting`
    of them driven over a tenth of the tail, one woken once for 1 %."""
    goals = [CENTER + 400, CENTER, CENTER - 400, CENTER]
    rows = []
    for k, goal in enumerate(goals):
        duty = np.zeros(30_000, int)
        if k < hunting:
            duty[-5000:-4000] = 90
        elif k == 3:
            duty[-2000:-1900] = 90
        rows.append((goal_seg(goal)["item"], goal, np.full(30_000, goal), duty))
    cap = sc.load(synth.write(tmp_path, rows, CENTER, GUARD, plan=plan([goal_seg(g) for g in goals])))
    assert sc.landing_tail_driven(cap) == hunting


def test_landing_drive_reads_the_drive_after_arrival(tmp_path):
    """A quiet landing, one driven again until 300 ms after it arrives, and
    one that rests 2 deg out and never enters the band."""
    goal = CENTER + 400
    approach = np.concatenate([np.linspace(CENTER, goal, 1000), np.full(29_000, goal)])
    arrive = int(np.argmax(np.abs(np.round(approach) - goal) <= sc.SETTLE_BAND_DEG * CPD))
    quiet = np.where(np.round(approach) < goal - sc.SETTLE_BAND_DEG * CPD, 600, 0)
    late = quiet.copy()
    late[arrive + 5000:arrive + 6000] = 90
    rows = [(goal_seg(goal)["item"], goal, approach, quiet),
            (goal_seg(goal)["item"], goal, approach, late),
            (goal_seg(goal)["item"], goal, np.full(30_000, goal - 40), np.full(30_000, 90))]
    segs = [goal_seg(goal)] * 3
    cap = sc.load(synth.write(tmp_path, rows, CENTER, GUARD, plan=plan(segs)))
    got = [sc.landing_drive_s(cap, s) for s in cap["plan"]["segments"]]
    assert got == pytest.approx([0.0, 0.3, 1.5])
    s = sc.landing_drive_scores(cap)
    assert s["landing_drive_ms"] == pytest.approx(300.0)
    assert s["landing_drive_ms_worst"] == pytest.approx(1500.0)


def ramp_rows(n=50):
    """Goals 10 counts apart every 512 ticks (a 400-row burst and a gap),
    the shaft 30 counts behind the goal in force, up and back down, a
    pause after each: the plan's segments and the rows."""
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
    return segs, rows


def test_a_ramp_reads_its_lag_and_its_real_speed(tmp_path):
    segs, rows = ramp_rows()
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs))
    f = sc.follow_scores(sc.load(tmp_path))
    assert f["lag_deg_1pct"] == pytest.approx(1.5)
    assert f["follow_error_sd_deg_1pct"] == pytest.approx(0.0, abs=1e-9)
    assert f["ramp_speed_deg_s_1pct"] == pytest.approx(10 / 0.0256 / CPD)
    assert f["follow_reversals_per_s_1pct"] == 0
    assert np.isfinite(f["crawl_speed_ripple_pct"])
    assert f["crawl_swing_deg"] == pytest.approx(0.0, abs=1e-9)


def test_the_crawl_swing_reads_the_items_that_swing_about_their_own_line(tmp_path):
    """One item in ten swings 16 counts peak to peak about its place, the
    others sit still: the 95th percentile is a swinging item's span."""
    segs, rows = ramp_rows()
    swing = np.round(8 * np.cos(2 * np.pi * np.arange(400) / 200))
    rows = [(item, goal, pos + swing if k % 10 == 0 and len(pos) == 400 else pos, duty)
            for k, (item, goal, pos, duty) in enumerate(rows)]
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs))
    f = sc.follow_scores(sc.load(tmp_path))
    assert f["crawl_swing_deg"] == pytest.approx(16 / CPD, abs=1 / CPD)


def ramp_paths(path, n=50):
    """ramp_rows' goals, the shaft in item j of each ramp on
    `path(goal, direction, j)`, 400 rows."""
    segs, rows = ramp_rows(n)
    j = 0
    for k, s in enumerate(segs):
        if s["test"] == "ramp":
            rows[k] = (rows[k][0], rows[k][1], path(s["goal_counts"], s["direction"], j), rows[k][3])
            j += 1
        else:
            j = 0
    return segs, rows


def stop_go(goal, direction, j):
    """Still over the item's first and last 40 rows, the item's 10 counts
    travelled between: the shaft stopped across every hole."""
    return goal - direction * 10 * (1 - np.clip((np.arange(400) - 40) / 320, 0, 1))


def steady(goal, direction, j):
    """A count every 4 ticks through the items and the holes alike, 5000
    counts/s, from the ramp's first goal less one step."""
    return goal - direction * 10 * (j + 1) + direction * ((512 * j + np.arange(400)) // 4)


def reports(segs, cps_of):
    """omega_hat_cps a poll an item: `cps_of(direction)` along the travel
    on a ramp item, 0 on a pause."""
    return [round(s["direction"] * cps_of(s["direction"]) * sc.Q16) if s["test"] == "ramp" else 0
            for s in segs]


def test_a_report_of_a_shaft_stopped_at_every_poll_scores_no_error(tmp_path):
    segs, rows = ramp_paths(stop_go)
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs),
                polls={"omega_hat_cps": reports(segs, lambda d: 0.0)})
    f = sc.follow_scores(sc.load(tmp_path))
    assert f["speed_report_error_deg_s_1pct"] == pytest.approx(0.0, abs=1e-6)
    # the same report against the ramp's own speed would read all of it as error
    assert f["ramp_speed_deg_s_1pct"] == pytest.approx(10 / 0.0256 / CPD)


def test_a_report_that_reads_slow_scores_its_shortfall(tmp_path):
    segs, rows = ramp_paths(steady, n=12)
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs),
                polls={"omega_hat_cps": reports(segs, lambda d: 5000 - 4.0 * CPD)})
    f = sc.follow_scores(sc.load(tmp_path))
    assert f["speed_report_error_deg_s_1pct"] == pytest.approx(-4.0, abs=1e-6)


def test_the_speed_report_is_the_mean_of_the_up_and_the_down_ramp(tmp_path):
    segs, rows = ramp_paths(stop_go)
    synth.write(tmp_path, rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs),
                polls={"omega_hat_cps": reports(segs, lambda d: (2.0 if d > 0 else -6.0) * CPD)})
    cap = sc.load(tmp_path)
    assert [sc.speed_report_error_deg_s(cap, s, g) for s, g in sc.ramps(cap).values()] == pytest.approx([2.0, -6.0])
    assert sc.follow_scores(cap)["speed_report_error_deg_s_1pct"] == pytest.approx(-2.0)


def steps_with_reports(tmp_path, offsets):
    """Three goals held flat, the middle one hunting 10 counts either side
    over its last 50 ms, then a hold; `offsets` are each poll's report
    less the goal the item before it held, in counts, the first poll
    taken before any goal."""
    goals = [CENTER + 400, CENTER, CENTER - 400]
    rows, segs = [], [goal_seg(g) for g in goals] + [hold_seg(500)]
    for k, goal in enumerate(goals):
        pos = np.full(30_000, float(goal))
        if k == 1:
            pos[-1000:] += 10 * np.sin(np.arange(1000) / 100)
        rows.append((goal_seg(goal)["item"], goal, pos, np.zeros(30_000, int)))
    rows.append(("hold@500", goals[-1], np.full(10_000, goals[-1]), np.zeros(10_000, int)))
    before = [CENTER, *goals]
    theta = [round((b + o) * sc.Q16) for b, o in zip(before, offsets)]
    return synth.write(tmp_path, rows, CENTER, GUARD, plan=plan(segs), polls={"theta_hat_q16": theta})


def test_a_position_report_reads_against_the_quiet_pot_of_the_item_it_closes(tmp_path):
    # the first poll closes no goal, the third closes the hunting one
    cap = sc.load(steps_with_reports(tmp_path, [300, 1, 50, -0.5]))
    assert sc.position_report_error_deg(cap) == pytest.approx(1 / CPD)


def test_a_capture_without_the_report_columns_scores_nan(tmp_path):
    segs, rows = ramp_rows()
    synth.write(tmp_path / "follow", rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs),
                polls={"omega_hat_cps": reports(segs, lambda d: 0.0)})
    steps_with_reports(tmp_path / "steps", [0, 1, 0, 1])
    for name, col in (("follow", "omega_hat_cps"), ("steps", "theta_hat_q16")):
        f = tmp_path / name / "segments.csv"
        pd.read_csv(f).drop(columns=col).to_csv(f, index=False)
    assert np.isnan(sc.follow_scores(sc.load(tmp_path / "follow"))["speed_report_error_deg_s_1pct"])
    assert np.isnan(sc.position_report_error_deg(sc.load(tmp_path / "steps")))


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
        ("steps", True), ("follow", False), ("hold", False), ("wide", False)] * 2
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
                                         "ramp_speed_deg_s", "speed_report_error_deg_s")
                 for p in TS.FOLLOW_SPEED_PCT}
    expected |= {"speed_ripple_pct", "rest_error_deg", "two_sided_deg", "enable_kick_deg", "enable_drive_ms",
                 "position_report_error_deg",
                 "landing_buzz_share", "landing_tail_driven", "landing_drive_ms", "landing_drive_ms_worst",
                 "hold_jitter_counts", "hold_buzz_share", "hold_current_rms_ma", "crawl_speed_ripple_pct",
                 "crawl_swing_deg", "wide_hold_hunting", "wide_hold_driven_share", "wide_hold_current_rms_ma"}
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
    monkeypatch.setattr(sc.settings, "apply", lambda fields, note="", stamp=True: calls.append(("apply", dict(fields))))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append(("restore", len(bus["log"]))))
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}, "note": "softer"}))
    rung = sc.run("01-gains", "abc123", "f" * 40, runs=1, settings_file=f, servo=SERVO, board=BOARD)
    assert calls == [("apply", {"p_kp_q88": 5026}), ("restore", 4)]
    assert json.loads((rung / "rung.json").read_text())["settings"] == {"p_kp_q88": 5026}


def test_a_settings_rung_reboots_after_an_abort(bus, monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(sc.settings, "apply", lambda fields, note="", stamp=True: calls.append("apply"))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append("restore"))

    def abort(out, mode, schedule, **kw):
        raise RuntimeError("aborted")

    monkeypatch.setattr(sc.osc, "track", abort)
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    with pytest.raises(RuntimeError, match="aborted"):
        sc.run("01-gains", "abc123", "f" * 40, runs=1, settings_file=f, servo=SERVO, board=BOARD)
    assert calls == ["apply", "restore"]


def pos_table_stubs(bus, monkeypatch):
    """settings, the table's install and the stamp stubbed, every call
    logged in order; the reboot logs the captures run before it."""
    calls = []
    monkeypatch.setattr(sc.settings, "apply",
                        lambda fields, note="", stamp=True: calls.append(("apply", dict(fields), stamp)))
    monkeypatch.setattr(sc.postable, "install", lambda pair: calls.append(("install", pair)))
    monkeypatch.setattr(sc.settings, "stamp_live", lambda: calls.append(("stamp",)))
    monkeypatch.setattr(sc.settings, "restore", lambda: calls.append(("restore", len(bus["log"]))))
    return calls


def table_of_record():
    path = postable.out_path(sc.pair_of(SERVO, BOARD))
    return {"file": str(results.rel(path)), "sha1": hashlib.sha1(path.read_bytes()).hexdigest()}


def test_a_pos_table_rung_installs_after_the_settings_and_before_the_one_stamp(bus, monkeypatch, tmp_path):
    calls = pos_table_stubs(bus, monkeypatch)
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    rung = sc.run("03-table", "abc123", "f" * 40, runs=1, settings_file=f, pos_table=True,
                  servo=SERVO, board=BOARD)
    pair = sc.pair_of(SERVO, BOARD)
    assert calls == [("apply", {"p_kp_q88": 5026}, False), ("install", pair), ("stamp",), ("restore", 4)]
    doc = json.loads((rung / "rung.json").read_text())
    assert doc["pos_table"] == table_of_record()
    assert doc["pos_table"]["file"] == f"stages/5-closed-loop/calibration/{pair}.pos_lut.json"


def test_a_pos_table_rung_without_settings_still_installs_and_stamps(bus, monkeypatch):
    calls = pos_table_stubs(bus, monkeypatch)
    rung = sc.run("03-table", "abc123", "f" * 40, runs=1, pos_table=True, servo=SERVO, board=BOARD)
    assert calls == [("install", sc.pair_of(SERVO, BOARD)), ("stamp",), ("restore", 4)]
    doc = json.loads((rung / "rung.json").read_text())
    assert doc["settings"] is None and doc["pos_table"] == table_of_record()


def test_a_rung_without_a_table_names_none_and_stamps_with_its_settings(bus, monkeypatch, tmp_path):
    calls = pos_table_stubs(bus, monkeypatch)
    f = tmp_path / "gains.json"
    f.write_text(json.dumps({"fields": {"p_kp_q88": 5026}}))
    rung = sc.run("01-gains", "abc123", "f" * 40, runs=1, settings_file=f, servo=SERVO, board=BOARD)
    assert calls == [("apply", {"p_kp_q88": 5026}, True), ("restore", 4)]
    assert json.loads((rung / "rung.json").read_text())["pos_table"] is None


def test_a_pos_table_rung_reboots_after_an_abort(bus, monkeypatch):
    calls = pos_table_stubs(bus, monkeypatch)

    def abort(out, mode, schedule, **kw):
        raise RuntimeError("aborted")

    monkeypatch.setattr(sc.osc, "track", abort)
    with pytest.raises(RuntimeError, match="aborted"):
        sc.run("03-table", "abc123", "f" * 40, runs=1, pos_table=True, servo=SERVO, board=BOARD)
    assert [c[0] for c in calls] == ["install", "stamp", "restore"]


def test_a_refused_install_reboots_and_runs_nothing(bus, monkeypatch):
    calls = pos_table_stubs(bus, monkeypatch)

    def refuse(pair):
        calls.append(("install", pair))
        raise sc.settings.Refused("the COMMIT landed REJECT_ENDS, not LIVE")

    monkeypatch.setattr(sc.postable, "install", refuse)
    with pytest.raises(sc.settings.Refused, match="REJECT_ENDS"):
        sc.run("03-table", "abc123", "f" * 40, runs=1, pos_table=True, servo=SERVO, board=BOARD)
    assert [c[0] for c in calls] == ["install", "restore"] and bus["log"] == []


@pytest.fixture
def ab_bench(bus, monkeypatch, tmp_path):
    """The fake servo with two images on disk, a.bin and b.bin: every flash,
    wait for the bus, status read, settings apply and restore, and every
    run's first capture logged in one order."""
    images = {}
    for name in ("a", "b"):
        images[name] = tmp_path / f"{name}.bin"
        images[name].write_bytes(name.encode() * 64)
    events = []
    track = sc.osc.track

    def tracked(out, mode, schedule, **kw):
        out = Path(out)
        if out.name == "steps":
            events.append(("run", out.parent.parent.name, out.parent.name))
        return track(out, mode, schedule, **kw)

    def status():
        events.append(("status",))
        return f"id 1: model 0x0101\n      data  clean\n      stamp {bus['stamp']}\n"

    bus["stamp"] = "0x9c45 = live set"
    monkeypatch.setattr(sc.osc, "track", tracked)
    monkeypatch.setattr(sc.osc, "flash", lambda image: events.append(("flash", Path(image).name)))
    monkeypatch.setattr(sc.osc, "status", status)
    monkeypatch.setattr(sc.settings, "await_boot", lambda: events.append(("boot",)))
    monkeypatch.setattr(sc.settings, "apply",
                        lambda fields, note="", stamp=True: events.append(("apply", dict(fields), stamp)))
    monkeypatch.setattr(sc.settings, "restore", lambda: events.append(("restore",)))
    return {"images": images, "events": events}


def side(bench, label, image, **kw):
    return {"label": label, "fw": "abc123", "image": bench["images"][image], **kw}


def settings_file(tmp_path, name, fields):
    f = tmp_path / f"{name}.json"
    f.write_text(json.dumps({"fields": fields}))
    return f


def flashed(image):
    return [("flash", image), ("boot",), ("status",)]


def test_ab_alternates_the_runs_flashing_before_each_when_the_images_differ(ab_bench):
    a, b = sc.ab(side(ab_bench, "10-a", "a"), side(ab_bench, "10-b", "b"), rounds=3, servo=SERVO, board=BOARD)
    expected = []
    for k in (1, 2, 3):
        expected += [*flashed("a.bin"), ("run", "10-a", f"run-{k}"), *flashed("b.bin"), ("run", "10-b", f"run-{k}")]
    assert ab_bench["events"] == expected
    assert (a.name, b.name) == ("10-a", "10-b")


def test_ab_on_one_image_flashes_once_and_applies_each_sides_settings_around_its_own_runs(ab_bench, tmp_path):
    fa = settings_file(tmp_path, "fa", {"p_kp_q88": 5026})
    fb = settings_file(tmp_path, "fb", {"p_kp_q88": 6000})
    sc.ab(side(ab_bench, "11-a", "a", settings_file=fa), side(ab_bench, "11-b", "a", settings_file=fb),
          rounds=3, servo=SERVO, board=BOARD)
    expected = flashed("a.bin")
    for k in (1, 2, 3):
        expected += [("apply", {"p_kp_q88": 5026}, True), ("run", "11-a", f"run-{k}"), ("restore",),
                     ("apply", {"p_kp_q88": 6000}, True), ("run", "11-b", f"run-{k}"), ("restore",)]
    assert ab_bench["events"] == expected


def test_ab_installs_the_table_on_its_side_alone_and_reboots_it_away_before_the_other(ab_bench, monkeypatch,
                                                                                         tmp_path):
    events = ab_bench["events"]
    monkeypatch.setattr(sc.postable, "install", lambda pair: events.append(("install", pair)))
    monkeypatch.setattr(sc.settings, "stamp_live", lambda: events.append(("stamp",)))
    f = settings_file(tmp_path, "f", {"p_kp_q88": 5026})
    a, b = sc.ab(side(ab_bench, "12-a", "a", settings_file=f),
                 side(ab_bench, "12-b", "a", settings_file=f, pos_table=True), rounds=2, servo=SERVO, board=BOARD)
    pair = sc.pair_of(SERVO, BOARD)
    expected = flashed("a.bin")
    for k in (1, 2):
        expected += [("apply", {"p_kp_q88": 5026}, True), ("run", "12-a", f"run-{k}"), ("restore",),
                     ("apply", {"p_kp_q88": 5026}, False), ("install", pair), ("stamp",),
                     ("run", "12-b", f"run-{k}"), ("restore",)]
    assert events == expected
    assert json.loads((a / "rung.json").read_text())["pos_table"] is None
    assert json.loads((b / "rung.json").read_text())["pos_table"] == table_of_record()


def test_ab_stops_before_any_run_when_the_stamp_is_not_the_live_set(ab_bench, bus, capsys):
    bus["stamp"] = "0x9c45 != live set 0x1234"
    with pytest.raises(SystemExit, match="unverified image"):
        sc.ab(side(ab_bench, "12-a", "a"), side(ab_bench, "12-b", "b"), rounds=3, servo=SERVO, board=BOARD)
    assert ab_bench["events"] == flashed("a.bin") and bus["log"] == []
    pair = sc.pair_of(SERVO, BOARD)
    assert not sc.rung_dir(pair, "12-a").exists() and not sc.rung_dir(pair, "12-b").exists()
    assert "left on the servo: " in capsys.readouterr().out


@pytest.mark.parametrize("labels, change, match", [
    (("13-a", "13-b"), {"torque": "on"}, "torque is on"),
    (("13-a", "13-a"), {}, "both sides are labeled"),
    (("13-a", "00-baseline"), {}, "already holds a rung"),
])
def test_ab_refuses_before_anything_is_flashed(ab_bench, bus, labels, change, match):
    sc.rung_dir(sc.pair_of(SERVO, BOARD), "00-baseline").mkdir(parents=True)
    bus.update(change)
    with pytest.raises(SystemExit, match=match):
        sc.ab(side(ab_bench, labels[0], "a"), side(ab_bench, labels[1], "b"), servo=SERVO, board=BOARD)
    assert ab_bench["events"] == [] and bus["log"] == []


def test_ab_restores_a_sides_settings_after_an_abort_in_its_run(ab_bench, bus, monkeypatch, tmp_path):
    track = sc.osc.track

    def abort_b(out, mode, schedule, **kw):
        if Path(out).parent.parent.name == "14-b":
            raise RuntimeError("aborted")
        return track(out, mode, schedule, **kw)

    monkeypatch.setattr(sc.osc, "track", abort_b)
    fb = settings_file(tmp_path, "fb", {"p_kp_q88": 6000})
    with pytest.raises(RuntimeError, match="aborted"):
        sc.ab(side(ab_bench, "14-a", "a"), side(ab_bench, "14-b", "b", settings_file=fb),
              servo=SERVO, board=BOARD)
    assert ab_bench["events"] == [*flashed("a.bin"), ("run", "14-a", "run-1"), *flashed("b.bin"),
                                  ("apply", {"p_kp_q88": 6000}, True), ("restore",)]
    assert sc.run_complete(sc.rung_dir(sc.pair_of(SERVO, BOARD), "14-a") / "run-1")


FLASH_OUT = ("image: 12288 B, fw 0.4.0; bootloaders at 500000\ninstall\n"
             "  0011aa: 1024/12288 B\r  0011aa: 8192/12288 B\r  0011aa: 12288/12288 B\n"
             "confirm\nuid 0011aa                    id 1    confirmed\n")


@pytest.mark.parametrize("code", [0, 1])
def test_flash_names_the_bus_rate_and_the_id_and_shows_the_last_line_only(monkeypatch, capsys, code):
    calls = []

    def fake_run(cmd, **kw):
        calls.append((cmd, kw))
        return subprocess.CompletedProcess(cmd, code, stdout=FLASH_OUT)

    monkeypatch.setattr(sc.osc, "bus_baud", lambda: 3_000_000)
    monkeypatch.setattr(sc.osc.subprocess, "run", fake_run)
    last = "uid 0011aa                    id 1    confirmed"
    if code:
        with pytest.raises(RuntimeError, match=f"exit 1, {last}"):
            sc.osc.flash(Path("fw.bin"))
    else:
        assert sc.osc.flash(Path("fw.bin")) == last
    (cmd, kw), = calls
    assert cmd == [sc.osc.TOOL, "-b", "3000000", "flash", "fw.bin", "--rate", "500000", sc.osc.ID]
    assert kw["stdout"] == subprocess.PIPE
    assert capsys.readouterr().out == ("" if code else last + "\n")


def test_ab_rungs_are_the_rungs_run_records(ab_bench, tmp_path):
    fb = settings_file(tmp_path, "fb", {"p_kp_q88": 6000})
    rungs = sc.ab(side(ab_bench, "15-a", "a", note="control"),
                  side(ab_bench, "15-b", "b", note="candidate", settings_file=fb),
                  rounds=3, servo=SERVO, board=BOARD)
    refs = [sc.run("ref-a", "abc123", hashlib.sha1(ab_bench["images"]["a"].read_bytes()).hexdigest(),
                   runs=3, note="control", servo=SERVO, board=BOARD),
            sc.run("ref-b", "abc123", hashlib.sha1(ab_bench["images"]["b"].read_bytes()).hexdigest(),
                   runs=3, note="candidate", settings_file=fb, servo=SERVO, board=BOARD)]
    for rung, ref in zip(rungs, refs):
        doc, want = (json.loads((p / "rung.json").read_text()) for p in (rung, ref))
        assert list(doc) == list(want)
        assert {k: v for k, v in doc.items() if k not in ("label", "utc")} == \
            {k: v for k, v in want.items() if k not in ("label", "utc")}
        assert sorted(p.name for p in rung.iterdir()) == ["run-1", "run-2", "run-3", "rung.json"]
        sc.score(rung.name, servo=SERVO, board=BOARD)
        out = json.loads(sc.result_path(sc.pair_of(SERVO, BOARD), rung.name).read_text())
        assert out["runs"] == ["run-1", "run-2", "run-3"] and out["skipped"] == []
    assert json.loads((rungs[1] / "rung.json").read_text())["settings"] == {"p_kp_q88": 6000}


STOPS = (82, 3792)


def bump_points(height):
    """A table bending the pot by up to `height` counts mid travel, zero at
    and beyond the stops."""
    lo, hi = (STOPS[0] + postable.GRID - 1) >> postable.GRID_SHIFT, STOPS[1] >> postable.GRID_SHIFT
    k = np.arange(postable.INTERVALS)
    c = np.where((k > lo) & (k < hi), np.rint(height * np.sin(np.pi * (k - lo) / (hi - lo))), 0)
    return c.astype(np.int64)


def table_rung(root, points, lut_state, segs, plan_doc, named=True, **kw):
    """A rung under `root` with one capture, steps, its table file under
    the repo root `root` and `rung.json` naming it (or not); `kw` go to
    the capture's writer."""
    path = root / "stages" / "5-closed-loop" / "calibration" / "synth.pos_lut.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    assert postable.validate(points, *STOPS) is None
    path.write_text(json.dumps({
        "inputs": {"stops": {"raw_min": STOPS[0], "raw_max": STOPS[1]}},
        "target": {"theta_at_stops_deg": [-90.0, 90.0]},
        "points_by_page": [[int(x) for x in points[p * 32:(p + 1) * 32]] for p in range(postable.PAGES)]}))
    rung = root / "telemetry" / "synth" / "scorecard" / "03-table"
    cap = synth.write(rung / "run-1" / "steps", segs, CENTER, GUARD, plan=plan_doc, lut_state=lut_state, **kw)
    table = {"file": str(path.relative_to(root)), "sha1": hashlib.sha1(path.read_bytes()).hexdigest()}
    (rung / "rung.json").write_text(json.dumps({"label": "03-table", "pos_table": table if named else None}))
    return cap, path


def resting_on(goal, points):
    """The raw pot the table reads nearest `goal` linearized counts, 12000
    rows of it under one goal."""
    raw = np.arange(4096)
    r = int(raw[np.argmin(np.abs(postable.interp_q4(raw, points) / 16 - goal))])
    segs = [{"item": f"goal:{goal}@600", "test": "step", "size_deg": 20, "direction": 1,
             "leg": "out", "goal_counts": goal, "goal_deg": 20.0}]
    return r, [(segs[0]["item"], goal, np.full(12_000, r), np.zeros(12_000, int))], plan(segs)


def test_a_live_rung_reads_the_raw_pot_through_its_table(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    points = bump_points(40)
    goal = CENTER + 400
    r, segs, plan_doc = resting_on(goal, points)
    assert abs(goal - r) > 30
    cap_dir, _ = table_rung(tmp_path, points, "LIVE", segs, plan_doc)
    cap = sc.load(cap_dir)
    assert (cap["rows"]["pos_raw"] == r).all()
    assert (cap["rows"]["pos"] == postable.interp_q4(r, points) / 16).all()
    assert sc.rest_error_deg(cap) < 0.6 / CPD
    # the same rows read raw miss the goal by the table's bend there
    raw_dir = synth.write(tmp_path / "raw", segs, CENTER, GUARD, plan=plan_doc)
    assert sc.rest_error_deg(sc.load(raw_dir)) == pytest.approx(abs(goal - r) / CPD)


def test_a_live_rungs_speed_report_reads_against_the_pot_through_its_table(tmp_path, monkeypatch):
    """A table whose points fall a count a node from 640 to 2816 raw, a
    linearized 15/16 count a raw count there, and a shaft at a steady
    5000 raw counts/s inside it: the servo's report is 4687.5."""
    monkeypatch.setattr(results, "ROOT", tmp_path)
    k = np.arange(postable.INTERVALS)
    points = np.clip(np.minimum(4 * (k - 6), 176 - k), 0, None)
    segs, rows = ramp_paths(steady, n=6)
    polls = {"omega_hat_cps": reports(segs, lambda d: 4687.5)}
    cap_dir, _ = table_rung(tmp_path, points, "LIVE", rows, plan(segs), gap_ticks=112, polls=polls)
    f = sc.follow_scores(sc.load(cap_dir))
    assert f["speed_report_error_deg_s_1pct"] == pytest.approx(0.0, abs=1e-6)
    raw_dir = synth.write(tmp_path / "raw", rows, CENTER, GUARD, gap_ticks=112, plan=plan(segs), polls=polls)
    f = sc.follow_scores(sc.load(raw_dir))
    assert f["speed_report_error_deg_s_1pct"] == pytest.approx((4687.5 - 5000) / CPD, abs=1e-6)


@pytest.mark.parametrize("rung_json", ["null", "absent"])
def test_a_live_capture_without_a_named_table_is_refused(tmp_path, monkeypatch, rung_json):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    _, segs, plan_doc = resting_on(CENTER, bump_points(40))
    cap_dir, _ = table_rung(tmp_path, bump_points(40), "LIVE", segs, plan_doc, named=False)
    if rung_json == "absent":
        (cap_dir.parent.parent / "rung.json").unlink()
    with pytest.raises(ValueError, match="LIVE and the rung names none"):
        sc.load(cap_dir)


def test_a_live_rung_whose_table_changed_since_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    _, segs, plan_doc = resting_on(CENTER, bump_points(40))
    cap_dir, path = table_rung(tmp_path, bump_points(40), "LIVE", segs, plan_doc)
    path.write_text(path.read_text().replace("[-90.0, 90.0]", "[-91.0, 90.0]"))
    with pytest.raises(ValueError, match="not the table the rung ran on"):
        sc.load(cap_dir)


def test_identity_rows_score_bit_for_bit_as_before(tmp_path, monkeypatch):
    """An IDENTITY capture's rows are the file's raw pot untouched, and the
    same rows under a LIVE all-zero table score exactly the same."""
    monkeypatch.setattr(results, "ROOT", tmp_path)
    goal, pos, duty = step_rows()
    segs = [{"item": f"goal:{goal}@1500", "test": "step", "size_deg": 20, "direction": 1,
             "leg": "out", "goal_counts": goal, "goal_deg": 20.0}]
    rows = [(segs[0]["item"], goal, pos, duty)]
    ident = sc.load(synth.write(tmp_path / "ident", rows, CENTER, GUARD, plan=plan(segs)))
    assert "pos_raw" not in ident["rows"]
    assert ident["rows"]["pos"].dtype.kind == "i"
    np.testing.assert_array_equal(ident["rows"]["pos"], np.round(pos).astype(int))
    live_dir, _ = table_rung(tmp_path, np.zeros(postable.INTERVALS, np.int64), "LIVE", rows, plan(segs))
    live = sc.load(live_dir)
    np.testing.assert_array_equal(live["rows"]["pos_f"], ident["rows"]["pos_f"])
    for f in (sc.step_scores, sc.rest_error_deg, sc.landing_buzz_share):
        assert f(live) == f(ident)
    assert float(live["rows"]["pos"].std()) == float(ident["rows"]["pos"].std())


# --- a rung on the table: its plan and its position report ---

def bump_table():
    return postable.Table(bump_points(40), *STOPS, -90.0, 90.0)


def table_inp(t, **kw):
    return {**INP, "counts_per_deg": t.scale, "table": t, **kw}


def test_a_table_set_is_planned_at_the_tables_scale_on_the_raw_center_read_through_it():
    t = bump_table()
    center, sets = TS.build(GUARD, table_inp(t))
    raw_center = TS.center_of(GUARD)
    assert center == round(float(t.lin(raw_center))) and center - raw_center > 30
    moves = [s for s in sets["steps"] if s["test"] == "step"]
    out = [round(size * t.scale) for size in TS.STEP_SIZES_DEG]
    assert [s["goal_counts"] - center for s in moves[::4]] == out
    assert moves[-2]["goal_deg"] == pytest.approx(-60.0, abs=0.5 / t.scale)
    # 1 % of 600 deg/s for 3 s is 18 deg, at the table's counts per degree
    first = [s for s in sets["follow"] if s["test"] == "ramp" and s["ramp"] == 0]
    assert first[-1]["goal_counts"] == center + round(18 * t.scale)
    doc = TS.plan_doc("steps", sets["steps"], center, GUARD, table_inp(t))
    assert (doc["counts_per_deg"], doc["center_counts"]) == (t.scale, center)


def test_a_table_set_fits_the_soft_range_in_the_tables_counts():
    t = bump_table()
    with pytest.raises(SystemExit, match=rf"5 deg \({5 * t.scale:.0f} counts\)"):
        TS.build((900, 3100), table_inp(t))


def test_a_table_sets_wide_goals_are_the_tables_reading_of_the_raw_goals():
    t = bump_table()
    center, sets = TS.build(GUARD, table_inp(t, wide_codes=WIDE_CODES))
    segs = sets["wide"]
    assert [s["raw_counts"] for s in segs[:4]] == [1023 + 10 + 30, 1029, 1023 - 30, 1029]
    assert all(s["goal_counts"] == round(float(t.lin(s["raw_counts"]))) for s in segs)
    assert all(s["goal_counts"] != s["raw_counts"] for s in segs)
    assert segs[1]["goal_deg"] == pytest.approx((segs[1]["goal_counts"] - center) / t.scale)


def test_a_table_rungs_inputs_name_the_tables_scale_and_file():
    pair = sc.pair_of(SERVO, BOARD)
    t = postable.load(pair)[0]
    inp = TS.inputs(SERVO, pair, BOARD.key, pos_table=True)
    assert inp["counts_per_deg"] == t.scale != TS.inputs(SERVO, pair)["counts_per_deg"]
    f = str(results.rel(postable.out_path(pair)))
    assert inp["sources"]["counts_per_deg"] == f"{f} target counts_per_deg, the table's linearized counts per deg"
    assert inp["sources"]["center_counts"].endswith(f)
    center, sets = TS.build((314, 3560), inp)
    doc = json.loads(json.dumps(TS.plan_doc("steps", sets["steps"], center, (314, 3560), inp)))
    assert doc["counts_per_deg"] == t.scale and doc["sources"] == inp["sources"]


def test_an_identity_rungs_plan_is_rebuilt_byte_for_byte():
    """`11a`'s recorded plans, an identity rung, rebuilt from its guard
    and the inputs of record; the `wide` capture is left out, its plan
    following the board's wide-code result, which may gain a code."""
    pair = sc.pair_of(SERVO, BOARD)
    run = results.ROOT / "telemetry" / pair / "scorecard" / "11a-first-reading" / "run-1"
    inp = TS.inputs(SERVO, pair, BOARD.key)
    for name in (c for c in TS.CAPTURES if c != "wide"):
        f = run / name / "plan.json"
        guard = tuple(json.loads(f.read_text())["guard_counts"])
        center, sets = TS.build(guard, inp)
        assert json.dumps(TS.plan_doc(name, sets[name], center, guard, inp), indent=2) + "\n" == f.read_text()


def test_a_table_rung_planned_at_the_pots_scale_scores_as_its_result_holds():
    """`12a` ran on the table on a plan at the stage 3 pot scale: it keeps
    that scale, and every row reads as its result file holds it, the
    position report row among them."""
    pair = sc.pair_of(SERVO, BOARD)
    rung = results.ROOT / "telemetry" / pair / "scorecard" / "12a-pos-table-live"
    planned = json.loads((rung / "run-1" / "steps" / "plan.json").read_text())
    assert planned["counts_per_deg"] == TS.inputs(SERVO, pair)["counts_per_deg"]
    want = json.loads(sc.result_path(pair, "12a-pos-table-live").read_text())["scores"]
    got = results._plain(sc.score_rung(rung))["scores"]
    assert got.keys() == want.keys()
    for row, w in want.items():
        assert got[row]["unit"] == w["unit"], row
        for k in ("value", "lo", "hi"):
            # to the float's last digits, which differ between platforms
            assert got[row][k] == pytest.approx(w[k], rel=1e-9, abs=1e-9, nan_ok=True), (row, k)
    assert want["position_report_error_deg"]["value"] is not None


def gained_points():
    """A table at a local gain of 1.5 from raw 320 to 1920 (linearized 1.5
    raw - 160), then 0.5 to 3520 (0.5 raw + 1760)."""
    k = np.arange(postable.INTERVALS)
    return np.clip(np.minimum(8 * (k - 20), 8 * (220 - k)), 0, None).astype(np.int64)


def square(lo, hi, n=30_000, half=100):
    return np.where((np.arange(n) // half) % 2 == 0, hi, lo)


def test_a_table_rungs_position_report_is_gated_on_the_servos_band_there(tmp_path, monkeypatch):
    """Three goals on a LIVE table, the deadband 4: one at rest, one whose
    pot swings 3 raw counts where the gain is 1.5 (4.5 linearized, inside
    the servo's band of 6 there), one swinging 6 where it is 0.5 (3
    linearized, outside its band of 2); the reports read 1, 3 and 10
    counts off the pot."""
    monkeypatch.setattr(results, "ROOT", tmp_path)
    raws = [np.full(30_000, 1000), square(1199, 1202), square(2800, 2806)]
    pots = [1340.0, 1.5 * 1200.5 - 160, 0.5 * 2803 + 1760]
    segs = [goal_seg(round(p)) for p in pots] + [hold_seg(500)]
    rows = [(s["item"], s["goal_counts"], r, np.zeros(30_000, int)) for s, r in zip(segs, raws)]
    rows.append(("hold@500", segs[2]["goal_counts"], np.full(10_000, 2803), np.zeros(10_000, int)))
    theta = [0] + [round((p + o) * sc.Q16) for p, o in zip(pots, (1, 3, 10))]
    cap_dir, _ = table_rung(tmp_path, gained_points(), "LIVE", rows, plan(segs), polls={"theta_hat_q16": theta})
    cap = sc.load(cap_dir)
    assert cap["table"].band_gain(np.array([1200, 2803])).tolist() == [1.5, 0.5]
    assert sc.position_report_error_deg(cap) == pytest.approx(3 / CPD, abs=0.01 / CPD)


def test_a_table_rungs_side_of_an_ab_is_planned_in_the_tables_counts(ab_bench, monkeypatch):
    monkeypatch.setattr(sc.postable, "install", lambda pair: None)
    monkeypatch.setattr(sc.settings, "stamp_live", lambda: None)
    a, b = sc.ab(side(ab_bench, "16-a", "a"), side(ab_bench, "16-b", "a", pos_table=True), rounds=1,
                 servo=SERVO, board=BOARD)
    pair = sc.pair_of(SERVO, BOARD)
    t = postable.load(pair)[0]
    pa, pb = (json.loads((r / "run-1" / "steps" / "plan.json").read_text()) for r in (a, b))
    middle = TS.center_of((314, 3560))
    assert (pa["counts_per_deg"], pa["center_counts"]) == (TS.inputs(SERVO, pair)["counts_per_deg"], middle)
    assert (pb["counts_per_deg"], pb["center_counts"]) == (t.scale, round(float(t.lin(middle))))
    wide = json.loads((b / "rung.json").read_text())["wide_goals"]
    assert all(g["goal_counts"] == round(float(t.lin(g["raw_counts"]))) for g in wide)
    assert all("raw_counts" not in g for g in json.loads((a / "rung.json").read_text())["wide_goals"])


# --- the wide capture: the set ---

WIDE_CODES = [{"stuck_code": 1023, "skipped": 10}, {"stuck_code": 2047, "skipped": 11}]


def no_board_result(monkeypatch, tmp_path):
    monkeypatch.setattr(wc, "result_file", lambda board: tmp_path / "none" / f"{board}.json")


def board_result(monkeypatch, tmp_path, codes):
    """The board's wide-code result of record replaced by one of `codes`."""
    path = tmp_path / "wide" / f"{BOARD.key}.json"
    synth.wide_result(path, codes)
    monkeypatch.setattr(wc, "result_file", lambda board: path)
    return path


def test_the_wide_capture_holds_each_code_from_above_then_below_lowest_first():
    center, sets = TS.build(GUARD, {**INP, "wide_codes": WIDE_CODES[::-1]})
    assert list(sets) == ["steps", "follow", "hold", "wide"] == list(TS.CAPTURES)
    segs = sets["wide"]
    # 1023 stands for 1023..1033 and 2047 for 2047..2058: 30 counts off each span, then the skipped codes' middle;
    # the approaches travel 46.9, 1.8, 53.0 and 1.8 deg at 20 counts a degree
    assert [s["item"] for s in segs] == [
        "goal:1063@2000", "goal:1029@2000", "goal:993@1200", "goal:1029@2000",
        "goal:2088@2000", "goal:2053@2000", "goal:2017@1200", "goal:2053@2000"]
    assert [s["test"] for s in segs] == ["wide-approach", "wide-hold"] * 4
    assert [(s["stuck_code"], s["side"]) for s in segs] == [
        (1023, "above"), (1023, "above"), (1023, "below"), (1023, "below"),
        (2047, "above"), (2047, "above"), (2047, "below"), (2047, "below")]
    assert segs[1]["goal_deg"] == pytest.approx((1029 - center) / CPD)
    TS.check_fits(segs, GUARD, CPD)


@pytest.mark.parametrize("codes", [None, [], [{"stuck_code": 255, "skipped": 5}, {"stuck_code": 3839, "skipped": 4}]])
def test_a_set_with_no_wide_code_in_reach_has_no_wide_capture(codes):
    inp = INP if codes is None else {**INP, "wide_codes": codes}
    assert list(TS.build(GUARD, inp)[1]) == ["steps", "follow", "hold"]


def test_a_wide_code_is_held_only_when_both_its_approaches_fit_the_soft_range():
    # GUARD less the 100-count fit margin reaches 400..3600
    codes = [{"stuck_code": 255, "skipped": 5}, {"stuck_code": 511, "skipped": 6},
             {"stuck_code": 3327, "skipped": 6}, {"stuck_code": 3583, "skipped": 5},
             {"stuck_code": 3839, "skipped": 4}]
    segs = TS.build(GUARD, {**INP, "wide_codes": codes})[1]["wide"]
    # 3583 stands for 3583..3588, inside the reach, and its approach from above, 3618, is not
    assert sorted({s["stuck_code"] for s in segs}) == [511, 3327]
    TS.check_fits(segs, GUARD, CPD)


@pytest.mark.parametrize("guard, held", [
    ((381, 3700), True),     # the approach below 511 is 481, on the edge of the reach
    ((382, 3700), False),
    ((300, 3463), True),     # the approach above 3327..3333 is 3363, on the edge of the reach
    ((300, 3462), False),
])
def test_a_wide_code_whose_approach_is_on_the_edge_of_the_reach_is_held_and_one_count_past_is_not(guard, held):
    code = {"stuck_code": 511 if guard[0] != 300 else 3327, "skipped": 6}
    sets = TS.build(guard, {**INP, "wide_codes": [code]})[1]
    assert ("wide" in sets) == held
    if held:
        assert len(sets["wide"]) == 4
        TS.check_fits(sets["wide"], guard, CPD)


def test_an_approach_is_held_as_long_as_a_step_of_its_travel():
    assert [TS.approach_ms(deg) for deg in (0, 1.8, 2, 2.1, 20, 20.1, 60, 60.1, 170)] == [
        1200, 1200, 1200, 1500, 1500, 2000, 2000, 2000, 2000]
    assert [TS.approach_ms(size) for size in TS.STEP_SIZES_DEG] == list(TS.STEP_HOLD_MS)
    # from the center at 2000: 72.7 deg down to 511, then 1023 a 27.4 deg travel away, 2047 53 deg, 2303 14.2 deg
    codes = [{"stuck_code": 511, "skipped": 6}, *WIDE_CODES, {"stuck_code": 2303, "skipped": 4}]
    segs = TS.wide(CENTER, CPD, GUARD, codes)
    approaches = [(s["stuck_code"], s["side"], s["item"]) for s in segs if s["test"] == "wide-approach"]
    assert approaches == [
        (511, "above", "goal:547@2000"), (511, "below", "goal:481@1200"),
        (1023, "above", "goal:1063@2000"), (1023, "below", "goal:993@1200"),
        (2047, "above", "goal:2088@2000"), (2047, "below", "goal:2017@1200"),
        (2303, "above", "goal:2337@1500"), (2303, "below", "goal:2273@1200")]
    assert {s["item"].split("@")[1] for s in segs if s["test"] == "wide-hold"} == {"2000"}
    # a code left out for its reach is not travelled from
    far = TS.wide(CENTER, CPD, GUARD, [{"stuck_code": 255, "skipped": 5}, WIDE_CODES[1]])
    assert [s["item"] for s in far[:1]] == ["goal:2088@1500"]


# Over every run of the five kept rungs of mg90-d on dev-v006-2A-D, 60 moves a
# step size: the longest a step took from its commit until the median pot was
# within WIDE_APPROACH_COUNTS of its goal, ms. The slowest are the stored
# settings' (00-baseline); the 60 degree one is a mean pace of 1301 counts/s.
SLOWEST_NEAR_MS = {2: 30, 20: 305, 60: 873}


def test_every_wide_hold_is_reached_from_its_side_at_the_slowest_kept_pace():
    """On the pair the pace was measured on, with its recorded soft range:
    an approach is within WIDE_APPROACH_COUNTS of its goal, so off the
    span on the side its hold names, before its time is up. One no longer
    than a step size is there no later than that step; one beyond the
    largest goes on at that step's mean pace."""
    inp = TS.inputs(SERVO, sc.pair_of(SERVO, BOARD), BOARD.key)
    cpd = inp["counts_per_deg"]
    center, sets = TS.build((314, 3560), inp)
    segs = sets["wide"]
    at, need = center, {}
    for approach, hold in zip(segs[::2], segs[1::2]):
        travel = abs(approach["goal_counts"] - at) / cpd
        size = next((s for s in TS.STEP_SIZES_DEG if travel <= s), None)
        need_ms = SLOWEST_NEAR_MS[size] if size else SLOWEST_NEAR_MS[60] * travel / 60
        ms = int(approach["item"].split("@")[1])
        assert need_ms < ms
        side = 1 if hold["side"] == "above" else -1
        assert side * (approach["goal_counts"] - hold["goal_counts"]) > TS.WIDE_APPROACH_COUNTS
        need[at, approach["goal_counts"]] = (need_ms, ms)
        at = hold["goal_counts"]
    # the long move between two codes, 56 deg: 873 ms of its 2000
    assert need[1029, 2088] == (873, 2000)
    # the first approach, 73.5 deg from the center, and a short one back across a span
    assert need[center, 547] == (pytest.approx(1070, abs=1), 2000)
    assert need[515, 481] == (30, 1200)
    assert len(need) == 8


def test_the_wide_codes_come_from_the_boards_result_the_ones_that_take_a_slot(monkeypatch, tmp_path):
    pair = sc.pair_of(SERVO, BOARD)
    # the board's result of record, through the rule a band takes a slot by
    inp = TS.inputs(SERVO, pair, BOARD.key)
    slotted = [b for b in wc.bands_of_record(BOARD.key) if b.slot is not None]
    assert inp["wide_codes"] == [{"stuck_code": b.stuck, "skipped": b.skipped} for b in slotted] != []
    assert not any(str(v).startswith("/") for v in inp["sources"].values())
    assert inp["sources"]["wide_codes"].startswith(f"stages/1-board/wide-codes/results/{BOARD.key}.json ")
    # no board asked for, and a board with no result
    assert TS.inputs(SERVO, pair)["wide_codes"] == []
    no_board_result(monkeypatch, tmp_path)
    inp = TS.inputs(SERVO, pair, BOARD.key)
    assert inp["wide_codes"] == [] and "wide_codes" not in inp["sources"]
    # a result whose bands take no slot: too short, off a carry, a thin stretch
    board_result(monkeypatch, tmp_path, [(1279, 2), (1500, 5), (1791, 4, 0.5)])
    assert TS.inputs(SERVO, pair, BOARD.key)["wide_codes"] == []
    board_result(monkeypatch, tmp_path, [(2047, 11), (1279, 2), (1023, 10)])
    assert TS.inputs(SERVO, pair, BOARD.key)["wide_codes"] == WIDE_CODES


# --- the wide capture: the rows ---

HUNT = np.tile(np.r_[np.full(400, 300), np.full(400, -300), np.zeros(200, int)], 10)
WAKE = np.r_[np.zeros(5000, int), np.full(100, 300), np.zeros(4900, int)]
REST = np.zeros(10_000, int)


def wide_capture(tmp_path, tails):
    """The wide capture of WIDE_CODES, the reading stuck at each code: every
    approach driven throughout, every hold driven over its first 100 ms and
    then at rest, but for its last 10000 rows, which carry the duty `tails`
    gives it."""
    segs = TS.wide(CENTER, CPD, GUARD, WIDE_CODES)
    rows, tails = [], iter(tails)
    for s in segs:
        duty = np.full(int(s["item"].split("@")[1]) * 20, 300)
        if s["test"] == "wide-hold":
            duty[2000:] = 0
            duty[-10_000:] = next(tails)
        rows.append((s["item"], s["goal_counts"], np.full(len(duty), s["stuck_code"]), duty))
    return sc.load(synth.write(tmp_path, rows, CENTER, GUARD, plan=plan(segs)))


@pytest.mark.parametrize("hunting", [0, 1, 2])
def test_the_wide_rows_read_the_holds_that_hunt_to_the_end(tmp_path, hunting):
    """Four holds, their tails the last 10000 rows: `hunting` of them
    driven on 80 % of the tail, the last one woken once for 1 % of it."""
    s = sc.wide_scores(wide_capture(tmp_path, [HUNT] * hunting + [REST] * (3 - hunting) + [WAKE]))
    assert s["wide_hold_hunting"] == hunting
    share = (8000 * hunting + 100) / 40_000
    # a tail is the rows within 500 ms of the last, 10000 or one more
    assert s["wide_hold_driven_share"] == pytest.approx(share, rel=2e-4)
    i_ma = 300 * synth.I_PER_DUTY * BOARD.a_per_count * 1e3
    assert s["wide_hold_current_rms_ma"] == pytest.approx(i_ma * np.sqrt(share), rel=2e-4)


def test_wide_holds_that_rest_score_zero_whatever_their_approaches_drove(tmp_path):
    assert sc.wide_scores(wide_capture(tmp_path, [REST] * 4)) == {
        "wide_hold_hunting": 0, "wide_hold_driven_share": 0.0, "wide_hold_current_rms_ma": 0.0}
    assert [sc.UNITS[sc.base_of(k)] for k in ("wide_hold_hunting", "wide_hold_driven_share",
                                              "wide_hold_current_rms_ma")] == ["holds", "share", "mA"]


# --- the wide capture: a rung with and without it ---

def test_a_rung_runs_the_wide_capture_last_and_names_its_goals(bus, monkeypatch, tmp_path):
    # 1279 takes no slot, and the approach above 3583..3589 is out of the reach of the soft range
    board_result(monkeypatch, tmp_path, [(1023, 10), (1279, 2), (2047, 11), (3583, 6)])
    rung = sc.run("10-wide", "abc123", "f" * 40, runs=2, servo=SERVO, board=BOARD)
    assert [(name, kw["deadman_check"]) for name, _, kw in bus["log"]] == [
        ("steps", True), ("follow", False), ("hold", False), ("wide", False)] * 2
    planned = json.loads((rung / "run-1" / "wide" / "plan.json").read_text())
    assert planned["capture"] == "wide"
    holds = [s for s in planned["segments"] if s["test"] == "wide-hold"]
    assert [(s["seg"], s["stuck_code"], s["side"], s["goal_counts"]) for s in holds] == [
        (1, 1023, "above", 1029), (3, 1023, "below", 1029), (5, 2047, "above", 2053), (7, 2047, "below", 2053)]
    goals = [{"stuck_code": s["stuck_code"], "side": s["side"], "goal_counts": s["goal_counts"]} for s in holds]
    assert json.loads((rung / "rung.json").read_text())["wide_goals"] == goals

    sc.score("10-wide", servo=SERVO, board=BOARD)
    out = json.loads(sc.result_path(sc.pair_of(SERVO, BOARD), "10-wide").read_text())
    assert out["wide_goals"] == goals and out["runs"] == ["run-1", "run-2"]
    # the fake servo rests inside its deadband
    assert out["scores"]["wide_hold_hunting"] == {"value": 0.0, "lo": 0.0, "hi": 0.0, "unit": "holds"}
    assert out["scores"]["wide_hold_driven_share"] == {"value": 0.0, "lo": 0.0, "hi": 0.0, "unit": "share"}
    assert out["scores"]["wide_hold_current_rms_ma"] == {"value": 0.0, "lo": 0.0, "hi": 0.0, "unit": "mA"}

    # a run whose wide capture did not end is not scored
    meta = rung / "run-2" / "wide" / "meta.json"
    meta.write_text(json.dumps({**json.loads(meta.read_text()), "ended": "aborted"}))
    assert not sc.run_complete(rung / "run-2")
    assert sc.score_rung(rung)["skipped"] == ["run-2"]


def test_a_board_without_a_result_runs_no_wide_capture_and_its_rows_are_absent(bus, monkeypatch, tmp_path):
    no_board_result(monkeypatch, tmp_path)
    rung = sc.run("00-plain", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert [name for name, _, _ in bus["log"]] == ["steps", "follow", "hold"]
    assert json.loads((rung / "rung.json").read_text())["wide_goals"] is None
    assert not (rung / "run-1" / "wide").exists()
    sc.score("00-plain", servo=SERVO, board=BOARD)
    pair = sc.pair_of(SERVO, BOARD)
    plain = json.loads(sc.result_path(pair, "00-plain").read_text())
    assert plain["runs"] == ["run-1"] and not any(k.startswith("wide_") for k in plain["scores"])

    # beside a rung that has the rows, the ladder leaves this one's cells empty
    board_result(monkeypatch, tmp_path, [(1023, 10)])
    sc.run("10-wide", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    sc.score("10-wide", servo=SERVO, board=BOARD)
    buf = io.StringIO()
    sc.table(servo=SERVO, board=BOARD, file=buf)
    lines = buf.getvalue().splitlines()
    assert lines[0].split()[:4] == ["score", "unit", "00-plain", "10-wide"]
    wide = {r.split()[0]: r.split()[1:] for r in lines if r.startswith("wide_hold_")}
    assert wide == {"wide_hold_hunting": ["holds", "-", "0"], "wide_hold_driven_share": ["share", "-", "0"],
                    "wide_hold_current_rms_ma": ["mA", "-", "0"]}
    assert lines[1].startswith("overshoot_deg_2deg")


# --- a wide-codes rung's rows through the board's slots ---

def wide_rung(root, codes, segs, plan_doc, points=None, polls=None):
    """A rung under the repo root `root` with one capture, steps, the
    board's wide-code result file under `root` and `rung.json` naming it;
    with `points`, a LIVE table of them named too."""
    named = synth.wide_result(root / "stages" / "1-board" / "wide-codes" / "results" / "synth.json", codes, root)
    if points is None:
        rung = root / "telemetry" / "synth" / "scorecard" / "09-wide"
        cap = synth.write(rung / "run-1" / "steps", segs, CENTER, GUARD, plan=plan_doc, polls=polls)
        doc = {"label": "09-wide"}
    else:
        cap, _ = table_rung(root, points, "LIVE", segs, plan_doc)
        doc = json.loads((cap.parent.parent / "rung.json").read_text())
    (cap.parent.parent / "rung.json").write_text(json.dumps({**doc, "wide_codes": named}))
    return cap, root / named["file"]


def stuck_rows():
    """A goal at the middle of the codes 1023 skips, the reading stuck at
    1023 but for a few rows on other codes at the start, then a hold: the
    codes, the rows and the plan."""
    codes = np.r_[[1022, 1024, 511, 2047, 2048], np.full(11_995, 1023)]
    goal = 1029
    segs = [{**goal_seg(goal), "item": f"goal:{goal}@600"}, hold_seg(500)]
    rows = [(segs[0]["item"], goal, codes, np.zeros(len(codes), int)),
            ("hold@500", goal, np.full(10_000, 1023), np.zeros(10_000, int))]
    return codes, rows, plan(segs)


def test_a_wide_codes_rung_reads_a_stuck_code_as_the_middle_of_its_span(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    codes, rows, plan_doc = stuck_rows()
    # the servo's report is the reading it used, 1023 + 5
    polls = {"theta_hat_q16": [0, 1028 * sc.Q16]}
    cap_dir, path = wide_rung(tmp_path, [(1023, 10), (2047, 11)], rows, plan_doc, polls=polls)
    cap = sc.load(cap_dir)
    first = cap["rows"][cap["rows"]["seg"] == 0]
    np.testing.assert_array_equal(first["pos_raw"], codes)
    np.testing.assert_array_equal(first["pos"], wc.compensated(codes, wc.slots_of_file(path)))
    assert first["pos"].iloc[:6].tolist() == [1022, 1024, 511, 2047 + 5.5, 2048, 1023 + 5]
    assert sc.rest_error_deg(cap) == pytest.approx(1 / CPD)
    assert sc.position_report_error_deg(cap) == pytest.approx(0)

    # the same rows on a rung that names no slots load as the file holds them
    plain_dir = synth.write(tmp_path / "telemetry" / "synth" / "scorecard" / "08-plain" / "run-1" / "steps",
                            rows, CENTER, GUARD, plan=plan_doc, polls=polls)
    (plain_dir.parent.parent / "rung.json").write_text(json.dumps({"label": "08-plain", "wide_codes": None}))
    plain = sc.load(plain_dir)
    assert "pos_raw" not in plain["rows"] and plain["rows"]["pos"].dtype.kind == "i"
    np.testing.assert_array_equal(plain["rows"]["pos"][plain["rows"]["seg"] == 0], codes)
    assert sc.rest_error_deg(plain) == pytest.approx(6 / CPD)
    assert sc.position_report_error_deg(plain) == pytest.approx(5 / CPD)


def test_a_wide_codes_rung_on_a_live_table_takes_the_step_ahead_of_the_table(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    codes, rows, plan_doc = stuck_rows()
    points = bump_points(40)
    cap_dir, path = wide_rung(tmp_path, [(1023, 10), (2047, 11)], rows, plan_doc, points=points)
    cap = sc.load(cap_dir)
    first = cap["rows"][cap["rows"]["seg"] == 0]
    np.testing.assert_array_equal(first["pos_raw"], codes)
    middle = wc.compensated(codes, wc.slots_of_file(path))
    np.testing.assert_array_equal(first["pos"], cap["table"].lin(middle))
    # off a stuck code the reading is the table's own, and on one it is the table read 5 counts up
    assert first["pos"].iloc[0] == postable.interp_q4(1022, points) / 16
    assert first["pos"].iloc[5] == pytest.approx(postable.interp_q4(1028, points) / 16)
    assert first["pos"].iloc[5] != postable.interp_q4(1023, points) / 16


def test_a_wide_codes_rung_whose_result_changed_since_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(results, "ROOT", tmp_path)
    _, rows, plan_doc = stuck_rows()
    cap_dir, path = wide_rung(tmp_path, [(1023, 10)], rows, plan_doc)
    sc.load(cap_dir)
    synth.wide_result(path, [(1023, 11)])
    with pytest.raises(ValueError, match="not the wide-code result the rung ran on"):
        sc.load(cap_dir)
    path.unlink()
    with pytest.raises(ValueError, match="not the wide-code result the rung ran on"):
        sc.load(cap_dir)
