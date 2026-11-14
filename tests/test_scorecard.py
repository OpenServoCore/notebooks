"""The stage 5 scorecard on synthetic captures: a step with a known
overshoot and settle through pot spikes, a ramp with a known lag, the
tick unwrap across a wrap, the reversal count, the test set's schedules
and refusal, the hold rows past a torque-on head, a landing's drive
after arrival and the landings that hunt, the crawl's swing inside a
goal item, the servo's speed and position reports against the pot, and
the runner's refusals with `osc` stubbed; then rungs end to end on a
fake servo that takes an install: a rung on the pair's constant set of
record, on that set with one deviation, on a set with a live table, the
set and the deviation it records, the test set planned from it, a servo
that is not the rung's set, and the rows read through the table and the
slots of the rung's own record; an A/B of two rungs, each side on its
own derived set and image, its flashes and refusals, and the flash's
command line; the servo's constant state beside every run and a run
that reads another refused by name; rungs recorded before a rung named
a set, read through the files they name, an identity rung's plan and a
table rung's plans rebuilt byte for byte, the last from the set of
record, and a table rung of record scored as recorded; the wide
capture the set builds from a board's wide-code result, its three rows,
and a rung with and without it."""
import copy
import hashlib
import io
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, constants as cn, postable, results, scorecard as sc, servos, settings
from oscnb import widecodes as wc

sys.path.insert(0, str(Path(__file__).resolve().parent))
import scorecard_synth as synth  # noqa: E402

CPD = 20.0
CENTER = 2000
GUARD = (300, 3700)
SERVO = servos.SERVOS["mg90-d"]
BOARD = boards.BOARDS[boards.DEFAULT]
TS = sc.testset()
STAGE1 = [results.path("1-board", name, BOARD.key) for name in ("current-scale", "terminal-chain")]
CPA = sc.scales.load(BOARD).counts_per_a      # the board's measured counts per A, stage 1 current-scale
A_PER_COUNT = 1 / CPA


def repo(monkeypatch, root):
    """The repo at `root`, the board's stage 1 conversions in it."""
    real = results.ROOT
    monkeypatch.setattr(results, "ROOT", root)
    for src in STAGE1:
        dst = root / src.relative_to(real)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(src, dst)


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
    i_ma = 600 * synth.I_PER_DUTY * A_PER_COUNT * 1e3
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
    i_ma = 600 * synth.I_PER_DUTY * A_PER_COUNT * 1e3
    assert s["hold_current_rms_ma"] == pytest.approx(i_ma * np.sqrt((b - a) / scored), rel=0.01)
    assert sc.enable_drive_ms(cap, "hold") == 0


def test_a_capture_recorded_before_rungs_named_a_set_is_read_at_the_boards_stage_1_scale(tmp_path):
    # the board's measured counts per A, not the registry's nominal parts, with or without a rung.json
    assert CPA == results.read("1-board", "current-scale", BOARD.key)["counts_per_a"]["value"]
    assert CPA != pytest.approx(BOARD.counts_per_a, rel=0.01)
    cap = holds(tmp_path / "rung" / "run-1" / "hold", [(1, 10_000, 20_000)])
    assert cap["counts_per_a"] == CPA and cap["current_scale"] == {"counts_per_a": CPA, "from": sc.BOARD_SCALE}
    assert sc.hold_scores(cap)["hold_current_rms_ma"] == pytest.approx(
        600 * synth.I_PER_DUTY / CPA * 1e3 * np.sqrt(1 / 3), rel=0.01)
    (tmp_path / "rung" / "rung.json").write_text(json.dumps({"label": "old", "settings": {"p_kp_q88": 5026}}))
    assert sc.load(tmp_path / "rung" / "run-1" / "hold")["counts_per_a"] == CPA


def test_the_hold_jitter_is_in_degrees_through_the_plans_counts_per_degree(tmp_path):
    # a count of jitter either side of the center, on a plan at 20 counts per degree
    s = sc.hold_scores(holds(tmp_path, []))
    assert s["hold_jitter_deg"] == pytest.approx(np.sqrt(2 / 3) / CPD, rel=1e-3) and "hold_jitter_counts" not in s
    assert sc.UNITS["hold_jitter_deg"] == "deg"


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


PAIR = sc.pair_of(SERVO, BOARD)


@pytest.fixture
def bus(monkeypatch, tmp_path):
    """A servo at rest on a 3 Mbps bus at 5 V, every call logged; osc-lab
    track answered by the fake servo, and `osc` by `servo`, which takes
    an install. The repo is `tmp_path`: the board's stage 1 conversions,
    the pair's stage 3 results, a wide-code result of the board that
    gives the fake servo's own slots, and the pair's constant set of
    record, the fake servo's stored state
    adopted and built, so a rung on it writes nothing."""
    state = {"baud": 3_000_000, "torque": "off", "vsys": 5.05, "shaft": 1950.0, "log": []}

    def track(out, mode, schedule, **kw):
        state["log"].append((Path(out).name, mode, kw))
        state["shaft"] = synth.fake_track(out, schedule, state["shaft"])
        return out

    real = [results.path("3-servo", name, PAIR) for name in ("pot-table", "run-up")]
    repo(monkeypatch, tmp_path)
    for src, name in zip(real, ("pot-table", "run-up")):
        dst = results.path("3-servo", name, PAIR)
        dst.parent.mkdir(parents=True)
        shutil.copy(src, dst)
    synth.wide_result(wc.result_file(BOARD.key), [(1023, 11)])
    # the rules of the registry's gear fields bind nothing: the fake servo has none of them
    monkeypatch.setattr(cn, "registry_of", lambda pair: {})
    state["servo"] = synth.Bench().stub(monkeypatch, lambda: {"torque_enable": state["torque"]})
    monkeypatch.setattr(sc.osc, "bus_baud", lambda: state["baud"])
    monkeypatch.setattr(sc.bench, "read_vsys", lambda board=None: state["vsys"])
    monkeypatch.setattr(sc.osc, "track", track)
    monkeypatch.setattr(sc, "result_path", lambda pair, label: tmp_path / "results" / pair / f"{label}.json")
    record = tmp_path / "servo.json"
    record.write_text(json.dumps(sc.servo_state()))
    adopted, _ = cn.adopt(PAIR, record, root=tmp_path)
    state["set"] = cn.load(cn.write(cn.build(PAIR, adopted, root=tmp_path)[0], cn.set_path(PAIR, tmp_path)))
    state["servo"].log.clear()
    return state


def deviation(root, name="dev", pins=None, policy=None, **values):
    """A deviation file in the repo `root`: pins of `values`, each with
    its why and its rung."""
    pins = pins if pins is not None else {n: {"value": v, "why": "a test's", "rung": name}
                                          for n, v in values.items()}
    path = root / "deviations" / f"{name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({**({"pins": pins} if pins else {}), **({"policy": policy} if policy else {})}))
    return path


@pytest.mark.parametrize("change, match", [
    ({"baud": 1_000_000}, "baud"),
    ({"torque": "on"}, "torque is on"),
    ({"vsys": 7.4}, "rail reads 7.40 V"),
    ({"vsys": 4.6}, "rail reads 4.60 V"),
])
def test_the_runner_refuses_before_anything_moves(bus, change, match):
    bus.update(change)
    with pytest.raises(SystemExit, match=match):
        sc.run("00-baseline", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert bus["log"] == [] and bus["servo"].wrote() == []


def test_the_runner_refuses_an_existing_rung(bus):
    (sc.rung_dir(PAIR, "00-baseline")).mkdir(parents=True)
    with pytest.raises(SystemExit, match="already holds a rung"):
        sc.run("00-baseline", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert bus["log"] == [] and bus["servo"].log == []


def test_a_rung_end_to_end(bus):
    rung = sc.run("00-baseline", "abc123", "f" * 40, runs=2, note="fake", servo=SERVO, board=BOARD)
    assert [(name, kw["deadman_check"]) for name, _, kw in bus["log"]] == [
        ("steps", True), ("follow", False), ("hold", False), ("wide", False)] * 2
    assert all(mode == "position" and kw["current_cap_ma"] == pytest.approx(850) and kw["current_cap_ms"] == 100
               for _, mode, kw in bus["log"])
    doc = json.loads((rung / "rung.json").read_text())
    assert (doc["label"], doc["fw"], doc["note"]) == ("00-baseline", "abc123", "fake")
    # the rung names the set it ran on, the pair's set of record: its file, its hashes, its fields by class
    assert doc["set"] == {"file": f"stages/5-closed-loop/calibration/{PAIR}.constants.json",
                          "hash": bus["set"]["hash"], "classes": cn.classes(bus["set"])}
    assert doc["deviation"] is None
    assert doc["protection"] == {"hash": bus["set"]["hash"]["protection"], "moved": {}}
    assert not any(k in doc for k in ("settings", "pos_table", "wide_codes"))
    assert cn.load(rung / sc.SET_COPY) == bus["set"]
    # the servo held the set already: nothing was written to it, and nothing is rebooted away
    assert bus["servo"].wrote() == []
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
                 "hold_jitter_deg", "hold_buzz_share", "hold_current_rms_ma", "crawl_speed_ripple_pct",
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


def sha1(path):
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def rung_json(rung):
    return json.loads((rung / "rung.json").read_text())


def test_a_rung_records_its_set_and_its_deviation(bus, monkeypatch, tmp_path):
    servo = bus["servo"]
    seen = []
    track = sc.osc.track

    def tracked(out, mode, schedule, **kw):
        seen.append((servo.decoded("p_kp_q88"), servo.decoded("pos_deadband_counts"), servo.data))
        return track(out, mode, schedule, **kw)

    monkeypatch.setattr(sc.osc, "track", tracked)
    f = deviation(tmp_path, "soft-band", p_kp_q88=5026, pos_deadband_counts=8)
    rung = sc.run("01-soft", "abc123", "f" * 40, runs=2, deviation=f, servo=SERVO, board=BOARD)
    doc, derived = rung_json(rung), cn.load(rung / sc.SET_COPY)
    # the set it ran on is the copy in its folder: the set of record with the one deviation, by its rules
    assert doc["set"] == {"file": f"stages/5-closed-loop/calibration/{PAIR}.constants.json",
                          "hash": derived["hash"], "classes": cn.classes(derived)}
    assert doc["set"]["classes"]["pinned"] == 2 and derived["previous"] == bus["set"]["hash"]
    assert doc["deviation"] == derived["deviation"] == {
        "file": "deviations/soft-band.json", "sha1": sha1(f),
        "fields": {"p_kp_q88": [40212, 5026], "pos_deadband_counts": [4, 8]}}
    # a deviation of the plant leaves the protection hash where the set of record has it
    assert doc["protection"] == {"hash": bus["set"]["hash"]["protection"], "moved": {}}
    assert derived["hash"]["plant"] != bus["set"]["hash"]["plant"]
    assert not any(k in doc for k in ("settings", "pos_table", "wide_codes"))
    # the install wrote the two fields and stamped once, every capture ran on them with a clean data
    # state, and the reboot at the end brought the stored state back
    assert seen == [(5026, 8, "clean")] * 8
    assert servo.wrote() == [("set", "p_kp_q88", "5026"), ("set", "pos_deadband_counts", "8"), ("run", "stamp"),
                             ("run", "reboot")]
    assert constants(rung, 1) == constants(rung, 2) and constants(rung, 1)["fields"]["p_kp_q88"] == 5026
    assert doc["constants_hash"] == constants(rung, 1)["hash"]
    assert (servo.decoded("p_kp_q88"), servo.decoded("pos_deadband_counts")) == (40212, 4)
    sc.score("01-soft", servo=SERVO, board=BOARD)
    out = json.loads(sc.result_path(PAIR, "01-soft").read_text())
    assert out["runs"] == ["run-1", "run-2"] and out["deviation"] == doc["deviation"]


def test_a_rung_is_scored_at_the_current_scale_of_its_own_set(bus, tmp_path):
    scale = bus["set"]["scales"][cn.CURRENT]
    assert scale["value"] == CPA and scale["class"] == "measured"
    rung = sc.run("00-scale", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    assert rung_json(rung)["set"]["hash"]["scales"] == bus["set"]["hash"]["scales"]
    assert sc.load(rung / "run-1" / "steps")["counts_per_a"] == CPA
    at_the_boards = sc.score_run(rung / "run-1")

    def rescored(doc):
        cn.write(doc, rung / sc.SET_COPY)
        named = rung_json(rung)
        (rung / "rung.json").write_text(json.dumps({**named, "set": {**named["set"], "hash": doc["hash"]}}))
        return sc.score_run(rung / "run-1")

    # the same rows under a set with twice the counts per A: every mA row at half, and no other row moves
    other = copy.deepcopy(bus["set"])
    other["scales"][cn.CURRENT]["value"] = 2 * CPA
    halved = rescored({**other, "hash": cn.hashes(other)})
    ma = [k for k in at_the_boards if sc.UNITS[sc.base_of(k)] == "mA"]
    assert len(ma) == 8 and all(at_the_boards[k] > 0 for k in ma if not k.startswith(("hold_", "wide_")))
    for k, v in at_the_boards.items():
        assert halved[k] == pytest.approx(v / 2 if k in ma else v, rel=1e-12, nan_ok=True), k
    # the scale is the one hash that tells the two sets apart
    assert [k for k in other["hash"] if cn.hashes(other)[k] != bus["set"]["hash"][k]] == ["scales"]
    # a set that holds no scale gives its rung's current nothing to be read at
    bare = {k: v for k, v in bus["set"].items() if k != "scales"}
    with pytest.raises(ValueError, match="the rung's set holds no current_counts_per_a"):
        rescored({**bare, "hash": cn.hashes(bare)})


def test_a_result_names_its_current_scale_and_one_whose_rows_were_not_kept_is_carried_to_it_once(bus, tmp_path):
    sc.run("00-kept", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    sc.score("00-kept", servo=SERVO, board=BOARD)
    kept = sc.result_path(PAIR, "00-kept")
    scored = json.loads(kept.read_text())
    assert scored["current_scale"] == {"counts_per_a": CPA, "from": "set", "hash": bus["set"]["hash"]["scales"]}
    # the same rung as a result scored at the board's nominal parts, before a result named its scale
    old = {k: v for k, v in copy.deepcopy(scored).items() if k != "current_scale"} | {"label": "00-gone"}
    ma = [k for k, q in old["scores"].items() if q["unit"] == "mA"]
    for k in ma:
        old["scores"][k].update({b: old["scores"][k][b] * CPA / BOARD.counts_per_a for b in ("value", "lo", "hi")})
    old["scores"]["hold_jitter_counts"] = {"value": 1.4, "lo": 1.3, "hi": 1.5, "unit": "counts"}
    gone = sc.result_path(PAIR, "00-gone")
    gone.write_text(json.dumps(old, indent=2) + "\n")

    assert sc.carry(servo=SERVO, board=BOARD) == [sc.shown(gone)]
    carried = json.loads(gone.read_text())
    assert carried["current_scale"] == {
        "counts_per_a": CPA, "from": sc.BOARD_SCALE,
        "carried": {"counts_per_a": BOARD.counts_per_a, "from": "the board's nominal parts",
                    "how": "the mA rows times nominal over measured counts per A, not scored again: "
                           "the rung's rows were not kept"}}
    assert list(carried) == list(scored)
    assert {k: v for k, v in carried.items() if k not in ("current_scale", "scores")} == {
        k: v for k, v in old.items() if k != "scores"}
    # the mA rows are where scoring the rows at the measured scale puts them, and no other row moved
    assert len(ma) == 8 and any(scored["scores"][k]["value"] > 0 for k in ma)
    for k, q in carried["scores"].items():
        if k in ma:
            assert q == pytest.approx(scored["scores"][k], rel=1e-12), k
        else:
            assert q == old["scores"][k], k
    # a second pass finds every result at a scale it names, and changes nothing
    texts = (gone.read_text(), kept.read_text())
    assert sc.carry(servo=SERVO, board=BOARD) == []
    assert (gone.read_text(), kept.read_text()) == texts
    # a result that names no scale and has its rung's rows is scored, never carried
    kept.write_text(json.dumps({k: v for k, v in scored.items() if k != "current_scale"}))
    unnamed = kept.read_text()
    with pytest.raises(SystemExit, match="the rows of 00-kept are kept: score a rung whose rows are there"):
        sc.carry(servo=SERVO, board=BOARD)
    assert kept.read_text() == unnamed


def test_every_result_of_record_names_the_one_current_scale_its_rows_are_at():
    pair = sc.pair_of(SERVO, BOARD)
    docs = [json.loads(p.read_text()) for p in sorted(sc.result_path(pair, "x").parent.glob("*.json"))]
    assert docs and {d["current_scale"]["counts_per_a"] for d in docs} == {CPA}
    for d in docs:
        rows = (results.ROOT / d["capture"] / "rung.json").exists()
        # scored from its rows where they are kept, carried from the nominal figure where they are not
        assert ("carried" in d["current_scale"]) == (not rows), d["label"]
        assert ("hold_jitter_deg" in d["scores"]) == rows and ("hold_jitter_counts" in d["scores"]) != rows, d["label"]


def test_a_protection_deviation_moves_nothing_else_and_is_named_at_the_install(bus, tmp_path):
    servo = bus["servo"]
    f = deviation(tmp_path, "limit", current_limit_counts=500)
    with pytest.raises(SystemExit) as e:
        sc.run("02-limit", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    assert str(e.value) == ("02-limit: the rung's set does not install: the protection gate:\n  "
                            "current_limit_counts: servo 554, set 500, protects more (less is up): name it with "
                            "--protection current_limit_counts")
    assert bus["log"] == [] and servo.wrote() == [] and not sc.rung_dir(PAIR, "02-limit").exists()
    rung = sc.run("02-limit", "abc123", "f" * 40, runs=1, deviation=f, protection=["current_limit_counts"],
                  servo=SERVO, board=BOARD)
    doc, derived = rung_json(rung), cn.load(rung / sc.SET_COPY)
    # protection is its own entry: the hash, and what the install moved against the servo's values
    assert doc["protection"] == {"hash": derived["hash"]["protection"],
                                 "moved": {"current_limit_counts": [554, 500]}}
    assert derived["hash"]["protection"] != bus["set"]["hash"]["protection"]
    assert derived["hash"]["plant"] == bus["set"]["hash"]["plant"]
    assert doc["deviation"]["fields"] == {"current_limit_counts": [554, 500]}
    # with a field of the plant beside it, it is no deviation
    mixed = deviation(tmp_path, "mixed", current_limit_counts=500, p_kp_q88=5026)
    servo.log.clear()
    with pytest.raises(SystemExit, match=r"03-mixed: deviations/mixed.json: a deviation that moves a protection "
                                         r"field \(current_limit_counts\) may move nothing else"):
        sc.run("03-mixed", "abc123", "f" * 40, runs=1, deviation=mixed, protection=["current_limit_counts"],
               servo=SERVO, board=BOARD)
    assert servo.log == []


def other_pair(bus, root):
    doc = copy.deepcopy(bus["set"])
    doc["pair"] = "other__board"
    return {"set_file": cn.write(doc, root / "other.constants.json")}


def stale_set(bus, root):
    synth.wide_result(wc.result_file(BOARD.key), [(1023, 11), (2047, 11)])
    return {}


def two_deviations(bus, root):
    derived, _ = cn.derive(bus["set"], deviation(root, "one", p_kp_q88=5026), root)
    return {"set_file": cn.write(derived, root / "derived.constants.json"),
            "deviation": deviation(root, "two", r_q12=16000)}


def outside(bus, root):
    path = root.parent / f"{root.name}-outside.json"
    path.write_text(json.dumps({"pins": {"p_kp_q88": {"value": 5026, "why": "a test's", "rung": "x"}}}))
    return {"deviation": path}


@pytest.mark.parametrize("make, match", [
    (other_pair, f"other.constants.json is the set of other__board, the servo at hand is {PAIR}"),
    (stale_set, "the set is stale: stale: 1-board/wide-codes .* is no longer the file the set read"),
    (lambda bus, root: {"deviation": deviation(root, "same", p_kp_q88=40212)},
     "deviations/same.json: the deviation moves no field of the set"),
    (lambda bus, root: {"deviation": deviation(root, "raw", pins={"p_kp_q88": {"value": 5026}})},
     "p_kp_q88 lacks its value, its why or its rung"),
    (lambda bus, root: {"deviation": deviation(root, "empty", pins={})}, "a deviation holds policy entries or pins"),
    (two_deviations, "the set already holds the deviation deviations/one.json"),
    (outside, "is not a file of the repo: a deviation is named by its path in it"),
    (lambda bus, root: {"set_file": cn.write(bus["set"], root.parent / f"{root.name}-set.json")},
     "is not a file of the repo: the set a rung runs on is named by its path in it"),
], ids=["the set of another pair", "a set with a moved input", "a deviation that moves nothing",
        "a pin without its why and its rung", "a deviation of nothing", "a second deviation",
        "a deviation outside the repo", "a set outside the repo"])
def test_a_rung_whose_set_does_not_stand_is_refused_with_the_bus_untouched(bus, tmp_path, make, match):
    kw = make(bus, tmp_path)
    with pytest.raises(SystemExit, match=f"00-x: .*{match}"):
        sc.run("00-x", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD, **kw)
    assert bus["log"] == [] and bus["servo"].log == []


def test_the_test_set_is_planned_from_the_rungs_derived_set(bus, tmp_path):
    # the soft range the plan runs in is the derived set's, read off no servo
    f = deviation(tmp_path, "narrow", pos_min_soft_counts=400, pos_max_soft_counts=3500)
    rung = sc.run("04-narrow", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    planned = json.loads((rung / "run-1" / "steps" / "plan.json").read_text())
    assert planned["guard_counts"] == [400, 3500] and planned["center_counts"] == TS.center_of((400, 3500))
    # one that cannot fit the test set is refused before anything is written or driven
    f = deviation(tmp_path, "tight", pos_min_soft_counts=1200, pos_max_soft_counts=2800)
    bus["log"].clear()
    bus["servo"].log.clear()
    with pytest.raises(SystemExit, match="cannot fit"):
        sc.run("05-tight", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    assert bus["log"] == [] and bus["servo"].wrote() == []
    # and a set that names a scale is planned at that scale or not at all
    off = {**bus["set"], "scale": {**bus["set"]["scale"], "counts_per_deg": 19.0}}
    with pytest.raises(SystemExit, match="the constant set's scale is 19.0 counts per deg, the raw one"):
        TS.inputs(SERVO, PAIR, BOARD.key, off)


def test_a_rung_on_a_deviation_reboots_after_an_abort(bus, monkeypatch, tmp_path):
    def abort(out, mode, schedule, **kw):
        raise RuntimeError("aborted")

    monkeypatch.setattr(sc.osc, "track", abort)
    f = deviation(tmp_path, "soft", p_kp_q88=5026)
    with pytest.raises(RuntimeError, match="aborted"):
        sc.run("01-soft", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    servo = bus["servo"]
    assert servo.wrote() == [("set", "p_kp_q88", "5026"), ("run", "stamp"), ("run", "reboot")]
    assert servo.decoded("p_kp_q88") == 40212


def test_a_set_that_does_not_install_runs_nothing(bus, tmp_path):
    servo = bus["servo"]
    servo.drop = {"p_kp_q88"}
    f = deviation(tmp_path, "soft", p_kp_q88=5026)
    with pytest.raises(SystemExit, match="01-soft: the rung's set does not install: p_kp_q88: wrote 5026, read "
                                         "back 40212; rebooted: the servo is back on its saved state"):
        sc.run("01-soft", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    assert bus["log"] == [] and not sc.rung_dir(PAIR, "01-soft").exists()
    assert ("run", "stamp") not in servo.wrote()


def test_a_rung_off_its_set_does_not_start(bus, monkeypatch, tmp_path):
    servo = bus["servo"]
    install = cn.install

    def then_moved(*a, **kw):
        # a state no install left: a field moves after the install's own read-back and stamp
        wrote = install(*a, **kw)
        servo.set("r_q12", 16000)
        return wrote

    monkeypatch.setattr(cn, "install", then_moved)
    f = deviation(tmp_path, "soft", p_kp_q88=5026)
    with pytest.raises(SystemExit) as e:
        sc.run("01-soft", "abc123", "f" * 40, runs=1, deviation=f, servo=SERVO, board=BOARD)
    assert str(e.value).endswith("01-soft: run-1 does not start: the servo is not the rung's set (data state "
                                 "clean; r_q12: servo 16000, set 17100 [adopted])")
    # nothing is recorded, nothing is driven, and the stored state comes back
    assert bus["log"] == [] and not sc.rung_dir(PAIR, "01-soft").exists()
    assert servo.wrote()[-1] == ("run", "reboot") and servo.decoded("r_q12") == 17100
    # the servo on its stored state is not a derived set either, whoever asks
    r = sc.new_rung(PAIR, "01-soft", "abc123", "f" * 40, 1, deviation=f)
    with pytest.raises(SystemExit, match=r"run-1 does not start: the servo is not the rung's set \(data state "
                                         r"clean; p_kp_q88: servo 40212, set 5026 \[pinned\]\)"):
        sc.record_constants(r, 1)
    assert not r["dir"].exists()


def on_table(bus, root):
    """The pair's table file put in the repo and its set of record built
    again on it, LIVE at the table's scale; returns the Table."""
    points = bump_points(40)
    path = cn.set_path(PAIR, root).with_name(postable.out_path(PAIR).name)
    path.write_text(json.dumps({
        "inputs": {"stops": {"raw_min": STOPS[0], "raw_max": STOPS[1]},
                   "fitted_on_slots_sha1": postable.slots_sha1(bus["set"]["wide_codes"]["slots"])},
        "target": {"theta_at_stops_deg": [-90.0, 90.0]},
        "points_by_page": [[int(x) for x in points[p * 32:(p + 1) * 32]] for p in range(postable.PAGES)]}))
    doc, _ = cn.build(PAIR, bus["set"], root=root)
    bus["set"] = cn.load(cn.write(doc, cn.set_path(PAIR, root)))
    assert bus["set"]["pos_table"]["state"] == "LIVE"
    return postable.load_file(path)[0]


def test_a_rung_on_a_set_with_a_live_table_installs_it_and_is_planned_and_read_through_it(bus, tmp_path):
    t = on_table(bus, tmp_path)
    servo = bus["servo"]
    other, _ = cn.derive(bus["set"], deviation(tmp_path, "soft", p_kp_q88=5026), tmp_path)
    rung = sc.run("03-table", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    # the install: the table's eight pages and the COMMIT, one stamp, and the reboot at the end
    wrote = servo.wrote()
    assert [c[0] for c in wrote[:9]] == ["write"] * 9 and wrote[9:] == [("run", "stamp"), ("run", "reboot")]
    assert len(bus["log"]) == 4 and servo.decoded("pos_lut_state") == postable.STATES.index("IDENTITY")
    # the plan: the table's scale, the raw center read through it, the wide goals the table's reading
    planned = json.loads((rung / "run-1" / "steps" / "plan.json").read_text())
    middle = TS.center_of((314, 3560))
    assert (planned["counts_per_deg"], planned["center_counts"]) == (t.scale, round(float(t.lin(middle))))
    f = f"stages/5-closed-loop/calibration/{PAIR}.pos_lut.json"
    assert planned["sources"]["counts_per_deg"] == f"{f} target counts_per_deg, the table's linearized counts per deg"
    doc = rung_json(rung)
    assert doc["wide_goals"] and all(g["goal_counts"] == round(float(t.lin(g["raw_counts"])))
                                     for g in doc["wide_goals"])
    assert doc["set"]["hash"] == bus["set"]["hash"] and constants(rung, 1)["pos_table"]["state"] == "LIVE"
    # the rows: read through the table and the slots of the rung's own record, the table's points those
    # the run's servo.json holds, so the rung reads the same once the pair's table file has moved
    cap_dir = rung / "run-1" / "hold"
    cn.set_path(PAIR, tmp_path).with_name(postable.out_path(PAIR).name).unlink()
    table, slots = sc.rung_table(cap_dir), sc.rung_wide_slots(cap_dir)
    np.testing.assert_array_equal(table.points, t.points)
    assert (table.raw_min, table.raw_max) == STOPS and slots == bytes(bus["set"]["wide_codes"]["slots"])
    meta = json.loads((cap_dir / "meta.json").read_text())
    (cap_dir / "meta.json").write_text(json.dumps({**meta, "plant": {"lut_state": "LIVE"}}))
    cap = sc.load(cap_dir)
    np.testing.assert_array_equal(cap["rows"]["pos"], t.lin(wc.compensated(cap["rows"]["pos_raw"], slots)))
    # the hold's jitter is the sd of that reading in degrees at the table's scale, as on an identity rung
    # it is the sd of the raw pot at the pot's
    held = [cap["rows"][cap["rows"]["seg"] == k] for k in (0, 1)]
    held[0] = held[0][held[0]["t"] >= held[0]["t"].iloc[0] + sc.HOLD_HEAD_MS / 1000]
    assert cap["cpd"] == t.scale
    assert sc.hold_scores(cap)["hold_jitter_deg"] == max(float(g["pos"].std()) for g in held) / t.scale
    # a copy that is not the set the rung names, and points that are not the set's, are refused
    state = constants(rung, 1)
    state["pos_table"]["points"] = bytes(512).hex()
    (rung / "run-1" / "servo.json").write_text(json.dumps(state))
    with pytest.raises(ValueError, match="the run's servo.json does not hold the points of the rung's set"):
        sc.load(cap_dir)
    cn.write(other, rung / sc.SET_COPY)
    with pytest.raises(ValueError, match="constants.json is not the set its rung.json names"):
        sc.load(cap_dir)


@pytest.fixture
def ab_bench(bus, monkeypatch, tmp_path):
    """The fake servo with two images on disk, a.bin and b.bin. A flash
    and a run's first capture, with the position gain the servo holds
    under it, go into the servo's own log, so they stand in one order
    with its writes. A flash reboots the servo onto its stored state;
    `descriptors` gives an image its own fields and stored values."""
    servo = bus["servo"]
    images = {}
    for name in ("a", "b"):
        images[name] = tmp_path / f"{name}.bin"
        images[name].write_bytes(name.encode() * 64)
    descriptors = {}
    track = sc.osc.track

    def tracked(out, mode, schedule, **kw):
        out = Path(out)
        if out.name == "steps":
            servo.log.append(("capture", out.parent.parent.name, out.parent.name, servo.decoded("p_kp_q88")))
        return track(out, mode, schedule, **kw)

    def flash(image):
        servo.log.append(("flash", Path(image).name))
        servo.FIELDS, servo.STORED = descriptors.get(Path(image).name, (type(servo).FIELDS, type(servo).STORED))
        servo.reboot()
        servo.stamp = bus.get("stamp", servo.stamp)

    monkeypatch.setattr(sc.osc, "track", tracked)
    monkeypatch.setattr(sc.osc, "flash", flash)
    return {"images": images, "descriptors": descriptors}


def side(bench, label, image, **kw):
    return {"label": label, "fw": "abc123", "image": bench["images"][image], **kw}


def events(bus):
    """The flashes, the writes to the servo and each run's first capture,
    in order."""
    return [c for c in bus["servo"].log if c[0] in ("flash", "capture", "set", "run")
            or (c[0] == "write" and c in bus["servo"].wrote())]


def test_ab_alternates_the_runs_flashing_before_each_when_the_images_differ(ab_bench, bus):
    a, b = sc.ab(side(ab_bench, "10-a", "a"), side(ab_bench, "10-b", "b"), rounds=3, servo=SERVO, board=BOARD)
    expected = []
    for k in (1, 2, 3):
        expected += [("flash", "a.bin"), ("capture", "10-a", f"run-{k}", 40212),
                     ("flash", "b.bin"), ("capture", "10-b", f"run-{k}", 40212)]
    # both sides on the set of record, which the servo stores: no write, no reboot
    assert events(bus) == expected
    assert (a.name, b.name) == ("10-a", "10-b")


def test_an_ab_runs_each_side_on_its_own_derived_set(ab_bench, bus, tmp_path):
    fa = deviation(tmp_path, "fa", p_kp_q88=5026)
    fb = deviation(tmp_path, "fb", p_kp_q88=6000, pos_min_soft_counts=400)
    rungs = sc.ab(side(ab_bench, "11-a", "a", deviation=fa), side(ab_bench, "11-b", "a", deviation=fb),
                  rounds=2, servo=SERVO, board=BOARD)
    # one image, one flash; a side's set is installed and stamped around its own run and rebooted away
    expected = [("flash", "a.bin")]
    for k in (1, 2):
        expected += [("set", "p_kp_q88", "5026"), ("run", "stamp"), ("capture", "11-a", f"run-{k}", 5026),
                     ("run", "reboot"),
                     ("set", "pos_min_soft_counts", "400"), ("set", "p_kp_q88", "6000"), ("run", "stamp"),
                     ("capture", "11-b", f"run-{k}", 6000), ("run", "reboot")]
    assert events(bus) == expected
    hashes = []
    for rung, fields, guard in zip(rungs, ({"p_kp_q88": [40212, 5026]},
                                           {"pos_min_soft_counts": [314, 400], "p_kp_q88": [40212, 6000]}),
                                   ([314, 3560], [400, 3560])):
        doc, derived = rung_json(rung), cn.load(rung / sc.SET_COPY)
        # the set against the set with one thing changed, and the one thing named
        assert doc["deviation"]["fields"] == fields and derived["previous"] == bus["set"]["hash"]
        assert doc["set"]["hash"] == derived["hash"]
        # each side's own constant state, the same every round, and its own plan
        state = constants(rung, 1)
        assert constants(rung, 2) == state and cn.differences(derived, state) == []
        assert doc["constants_hash"] == state["hash"]
        assert json.loads((rung / "run-1" / "steps" / "plan.json").read_text())["guard_counts"] == guard
        hashes.append((doc["set"]["hash"]["set"], state["hash"]))
    assert hashes[0][0] != hashes[1][0] and hashes[0][1] != hashes[1][1]


def with_field(cls, after, name, addr, stored):
    """A descriptor with one more u16 field than the fake servo's, after
    field `after`, and its stored values."""
    fields = {}
    for n, f in cls.FIELDS.items():
        fields[n] = f
        if n == after:
            fields[name] = (addr, 2, "rw", "uint")
    return fields, {**cls.STORED, name: stored}


def test_a_deviation_naming_a_field_an_image_lacks_is_refused_on_that_side_before_anything_is_written(
        ab_bench, bus, tmp_path):
    """A deviation that pins a field only the newer image has: the side
    on that image runs on it, the side on the older image is refused by
    the install, the field named."""
    ab_bench["descriptors"]["b.bin"] = with_field(synth.Bench, "recip_ke_q", "r_bemf_q12", 0x84, 17000)
    f = deviation(tmp_path, "bemf", recip_ke_q=12000, r_bemf_q12=16500)
    a, b = sc.ab(side(ab_bench, "20-a", "a"), side(ab_bench, "20-b", "b", deviation=f), rounds=1,
                 servo=SERVO, board=BOARD)
    assert events(bus) == [("flash", "a.bin"), ("capture", "20-a", "run-1", 40212), ("flash", "b.bin"),
                           ("set", "recip_ke_q", "12000"), ("set", "r_bemf_q12", "16500"), ("run", "stamp"),
                           ("capture", "20-b", "run-1", 40212), ("run", "reboot")]
    assert rung_json(b)["deviation"]["fields"] == {"recip_ke_q": [11817, 12000], "r_bemf_q12": [None, 16500]}
    assert constants(b, 1)["fields"]["r_bemf_q12"] == 16500 and "r_bemf_q12" not in constants(a, 1)["fields"]
    bus["servo"].log.clear()
    with pytest.raises(SystemExit) as e:
        sc.ab(side(ab_bench, "21-a", "a", deviation=f), side(ab_bench, "21-b", "b", deviation=f), rounds=1,
              servo=SERVO, board=BOARD)
    assert str(e.value) == ("21-a: the rung's set does not install: the servo's descriptor lets a host write no "
                            "r_bemf_q12")
    assert events(bus) == [("flash", "a.bin")] and len(bus["log"]) == 8
    assert not sc.rung_dir(PAIR, "21-a").exists() and not sc.rung_dir(PAIR, "21-b").exists()


def test_a_side_on_the_table_installs_it_around_its_own_runs_and_is_planned_in_its_counts(ab_bench, bus, tmp_path):
    identity = cn.write(bus["set"], tmp_path / "identity.constants.json")
    t = on_table(bus, tmp_path)
    a, b = sc.ab(side(ab_bench, "12-a", "a", set_file=identity), side(ab_bench, "12-b", "a"), rounds=2,
                 servo=SERVO, board=BOARD)
    table = [c for c in bus["servo"].wrote() if c[0] == "write"][:9]
    expected = [("flash", "a.bin")]
    for k in (1, 2):
        expected += [("capture", "12-a", f"run-{k}", 40212), *table, ("run", "stamp"),
                     ("capture", "12-b", f"run-{k}", 40212), ("run", "reboot")]
    assert events(bus) == expected
    pa, pb = (json.loads((r / "run-1" / "steps" / "plan.json").read_text()) for r in (a, b))
    middle = TS.center_of((314, 3560))
    assert (pa["counts_per_deg"], pa["center_counts"]) == (TS.inputs(SERVO, PAIR)["counts_per_deg"], middle)
    assert (pb["counts_per_deg"], pb["center_counts"]) == (t.scale, round(float(t.lin(middle))))
    assert all(g["goal_counts"] == round(float(t.lin(g["raw_counts"]))) for g in rung_json(b)["wide_goals"])
    assert all("raw_counts" not in g for g in rung_json(a)["wide_goals"])
    assert rung_json(a)["set"]["file"] == "identity.constants.json"
    assert (constants(a, 1)["pos_table"]["state"], constants(b, 1)["pos_table"]["state"]) == ("IDENTITY", "LIVE")


def test_ab_stops_before_any_run_when_the_stamp_is_not_the_live_set(ab_bench, bus, capsys):
    bus["stamp"] = "0x9c45 != live set 0x1234"
    with pytest.raises(SystemExit, match="unverified image"):
        sc.ab(side(ab_bench, "12-a", "a"), side(ab_bench, "12-b", "b"), rounds=3, servo=SERVO, board=BOARD)
    assert events(bus) == [("flash", "a.bin")] and bus["log"] == []
    assert not sc.rung_dir(PAIR, "12-a").exists() and not sc.rung_dir(PAIR, "12-b").exists()
    assert "left on the servo: " in capsys.readouterr().out


@pytest.mark.parametrize("labels, change, match", [
    (("13-a", "13-b"), {"torque": "on"}, "torque is on"),
    (("13-a", "13-a"), {}, "both sides are labeled"),
    (("13-a", "00-baseline"), {}, "already holds a rung"),
])
def test_ab_refuses_before_anything_is_flashed(ab_bench, bus, labels, change, match):
    sc.rung_dir(PAIR, "00-baseline").mkdir(parents=True)
    bus.update(change)
    with pytest.raises(SystemExit, match=match):
        sc.ab(side(ab_bench, labels[0], "a"), side(ab_bench, labels[1], "b"), servo=SERVO, board=BOARD)
    assert events(bus) == [] and bus["log"] == []


def test_ab_reboots_a_sides_set_away_after_an_abort_in_its_run(ab_bench, bus, monkeypatch, tmp_path):
    track = sc.osc.track

    def abort_b(out, mode, schedule, **kw):
        if Path(out).parent.parent.name == "14-b":
            raise RuntimeError("aborted")
        return track(out, mode, schedule, **kw)

    monkeypatch.setattr(sc.osc, "track", abort_b)
    fb = deviation(tmp_path, "fb", p_kp_q88=6000)
    with pytest.raises(RuntimeError, match="aborted"):
        sc.ab(side(ab_bench, "14-a", "a"), side(ab_bench, "14-b", "b", deviation=fb), servo=SERVO, board=BOARD)
    assert events(bus) == [("flash", "a.bin"), ("capture", "14-a", "run-1", 40212), ("flash", "b.bin"),
                           ("set", "p_kp_q88", "6000"), ("run", "stamp"), ("run", "reboot")]
    assert sc.run_complete(sc.rung_dir(PAIR, "14-a") / "run-1")


def test_the_cli_names_the_set_and_the_deviation_of_a_run_and_of_each_side(monkeypatch):
    seen = []
    monkeypatch.setattr(sc, "run", lambda *a, **kw: seen.append(a) or Path("x"))
    sc.main(["run", "--label", "l", "--fw", "f", "--image-sha", "s"])
    assert seen[-1] == ("l", "f", "s", 3, "", None, None, [], [])
    sc.main(["run", "--label", "l", "--fw", "f", "--image-sha", "s", "--set", "x.json", "--deviation", "d.json",
             "--protection", "current_limit_counts", "stall_response", "--loosen", "stall_response"])
    assert seen[-1][5:] == (Path("x.json"), Path("d.json"), ["current_limit_counts", "stall_response"],
                            ["stall_response"])
    monkeypatch.setattr(sc, "ab", lambda *sides, rounds: seen.append(sides) or (Path("a"), Path("b")))
    sc.main(["ab", "--a-label", "a", "--a-fw", "f", "--a-image", "i", "--b-label", "b", "--b-fw", "g",
             "--b-image", "j", "--b-set", "x.json", "--b-deviation", "d.json", "--b-protection", "cutoff_cc",
             "--b-loosen", "cutoff_cc"])
    a, b = seen[-1]
    assert a == {"label": "a", "fw": "f", "image": Path("i"), "note": "", "set_file": None, "deviation": None,
                 "protection": [], "loosen": []}
    assert b == {"label": "b", "fw": "g", "image": Path("j"), "note": "", "set_file": Path("x.json"),
                 "deviation": Path("d.json"), "protection": ["cutoff_cc"], "loosen": ["cutoff_cc"]}
    # the flags a rung had before its set are gone
    for gone in ("--settings", "--pos-table", "--wide-codes"):
        with pytest.raises(SystemExit):
            sc.main(["run", "--label", "l", "--fw", "f", "--image-sha", "s", gone])


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
    fb = deviation(tmp_path, "fb", p_kp_q88=6000)
    rungs = sc.ab(side(ab_bench, "15-a", "a", note="control"),
                  side(ab_bench, "15-b", "b", note="candidate", deviation=fb),
                  rounds=3, servo=SERVO, board=BOARD)
    refs = [sc.run("ref-a", "abc123", sha1(ab_bench["images"]["a"]), runs=3, note="control",
                   servo=SERVO, board=BOARD),
            sc.run("ref-b", "abc123", sha1(ab_bench["images"]["b"]), runs=3, note="candidate", deviation=fb,
                   servo=SERVO, board=BOARD)]
    for rung, ref in zip(rungs, refs):
        doc, want = rung_json(rung), rung_json(ref)
        assert list(doc) == list(want)
        assert {k: v for k, v in doc.items() if k not in ("label", "utc")} == \
            {k: v for k, v in want.items() if k not in ("label", "utc")}
        assert sorted(p.name for p in rung.iterdir()) == [sc.SET_COPY, "run-1", "run-2", "run-3", "rung.json"]
        assert (rung / sc.SET_COPY).read_text() == (ref / sc.SET_COPY).read_text()
        sc.score(rung.name, servo=SERVO, board=BOARD)
        out = json.loads(sc.result_path(PAIR, rung.name).read_text())
        assert out["runs"] == ["run-1", "run-2", "run-3"] and out["skipped"] == []
    assert rung_json(rungs[1])["deviation"]["fields"] == {"p_kp_q88": [40212, 6000]}


# --- the servo's constant state beside a run ---

SLOTS = bytes([0, 0, 0, 88] + [0] * 12)     # the fake servo's stored wide-code slots
NO_POINTS = hashlib.sha1(bytes(512)).hexdigest()


def constants(rung, k):
    return json.loads((rung / f"run-{k}" / "servo.json").read_text())


def test_a_run_records_the_servos_constant_state(bus):
    servo = bus["servo"]
    rung = sc.run("00-baseline", "abc123", "f" * 40, runs=2, servo=SERVO, board=BOARD)
    doc = constants(rung, 1)
    assert constants(rung, 2) == doc
    assert list(doc) == ["addr", "raw", "fields", "writable", "pos_table", "wide_codes", "stamp", "data_state",
                         "hash"]
    # CONFIG and CALIB end where the servo's own descriptor places torque_enable, 0xc0 on the fake
    assert doc["addr"] == 0 and bytes.fromhex(doc["raw"]) == bytes(servo.table[:0xC0])
    assert doc["raw"][2 * 0x40:2 * 0x44] == "149d0400" and bytes.fromhex(doc["raw"])[0x91:0xA1] == SLOTS
    below = {n: f for n, f in servo.FIELDS.items() if f[0] < 0xC0}
    assert doc["fields"] == {n: servo.decoded(n) for n in below}
    assert (doc["fields"]["p_kp_q88"], doc["fields"]["drive_polarity"], doc["fields"][wc.FIELD]) == (
        40212, "on", SLOTS.hex(" "))
    # the fields a host may write, by the servo's own descriptor: the read-only ones are not among them
    assert doc["writable"] == [n for n, f in below.items() if f[2] == "rw"]
    assert "model_number" in doc["fields"] and "model_number" not in doc["writable"]
    # the table's 256 points beside their sha1, which alone is in the hash
    assert doc["pos_table"] == {"state": "IDENTITY", "points_sha1": NO_POINTS, "points": bytes(512).hex()}
    assert doc["hash"] == sc.config_hash({**{k: doc[k] for k in sc.HASHED},
                                          "pos_table": {"state": "IDENTITY", "points_sha1": NO_POINTS}})
    assert doc["wide_codes"] == SLOTS.hex()
    assert (doc["stamp"], doc["data_state"]) == ("0x9c45 = live set", "clean")
    assert rung_json(rung)["constants_hash"] == doc["hash"]
    assert sorted(p.name for p in (rung / "run-1").iterdir()) == ["follow", "hold", "servo.json", "steps", "wide"]


def test_the_constant_state_is_read_once_a_run_after_the_set_is_on_and_before_its_first_capture(
        bus, monkeypatch, tmp_path):
    servo = bus["servo"]
    track = sc.osc.track

    def tracked(out, mode, schedule, **kw):
        servo.log.append(("capture", Path(out).parent.name, Path(out).name))
        return track(out, mode, schedule, **kw)

    monkeypatch.setattr(sc.osc, "track", tracked)
    f = deviation(tmp_path, "soft", p_kp_q88=5026)
    sc.run("04-soft", "abc123", "f" * 40, runs=2, deviation=f, servo=SERVO, board=BOARD)
    # the fields and the bytes under them, the table's state and its 8 pages, the slots, the status: 21 calls,
    # the bytes one `osc read` more on a servo whose CONFIG and CALIB outrun a status frame
    read = [("dump",), ("read", 0, 0xC0), ("get", "pos_lut_state")]
    for page in range(postable.PAGES):
        read += [("write", postable.WINDOW_ADDR, bytes([page, postable.FETCH])),
                 ("read", postable.POINTS_ADDR, 64)]
    read += [("field", wc.FIELD), ("status",)]
    assert len(read) == 21
    # the install: the state, the one field with its read-back, the state again, which must be the set, the
    # stamp and the data state; then a state a run; then the reboot and its data state
    expected = [*read, ("set", "p_kp_q88", "5026"), ("get", "p_kp_q88"), *read, ("run", "stamp"), ("status",)]
    for k in (1, 2):
        expected += [*read, *(("capture", f"run-{k}", name) for name in TS.CAPTURES)]
    assert servo.log == [*expected, ("run", "reboot"), ("status",)]


def test_a_constant_changed_between_runs_is_refused_with_the_field_named(bus, monkeypatch):
    servo = bus["servo"]
    track = sc.osc.track

    def tracked(out, mode, schedule, **kw):
        track(out, mode, schedule, **kw)
        if (Path(out).parent.name, Path(out).name) == ("run-1", TS.CAPTURES[-1]):
            servo.set("r_q12", 16000)
        return out

    monkeypatch.setattr(sc.osc, "track", tracked)
    with pytest.raises(SystemExit) as e:
        sc.run("00-baseline", "abc123", "f" * 40, runs=3, servo=SERVO, board=BOARD)
    assert str(e.value).endswith("00-baseline: run-2 does not start: the servo is not the rung's set (data state "
                                 "clean; r_q12: servo 16000, set 17100 [adopted])")
    # nothing of run-2 is recorded or driven, and run-1 stands
    rung = sc.rung_dir(PAIR, "00-baseline")
    assert [name for name, _, _ in bus["log"]] == list(TS.CAPTURES)
    assert sorted(p.name for p in rung.iterdir()) == [sc.SET_COPY, "run-1", "rung.json"]
    assert sc.run_complete(rung / "run-1") and constants(rung, 1)["fields"]["r_q12"] == 17100


PAGE_3 = b"\x01" + bytes(63)


def a_table_point(servo):
    servo.pages[3] = PAGE_3


def an_unnamed_byte(servo):
    servo.table[servo.UNNAMED] = 0xA5


OFF = "does not start: the servo is not the rung's set (data state "
OTHER = "reads a constant state other than the one the rung's first run recorded: "


@pytest.mark.parametrize("change, said", [
    (lambda s: s.set("p_kp_q88", 5026), OFF + "clean; p_kp_q88: servo 5026, set 40212 [adopted])"),
    (lambda s: s.set("drive_polarity", 0), OFF + "clean; drive_polarity: servo off, set on [adopted])"),
    (lambda s: s.set(wc.FIELD, bytes(16)),
     OFF + f"clean; wide-code slots: servo {bytes(16).hex(' ')}, set {SLOTS.hex(' ')} [measured, slots.wide_codes])"),
    (lambda s: s.set("pos_lut_state", 2),
     OFF + "clean; position table state: servo LIVE, set IDENTITY [adopted])"),
    (a_table_point, OFF + "clean; position table points sha1: servo "
                    f"{hashlib.sha1(bytes(3 * 64) + PAGE_3 + bytes(4 * 64)).hexdigest()}, set {NO_POINTS} "
                    "[adopted])"),
    (lambda s: setattr(s, "data", "STAMP_MISMATCH"), OFF + "STAMP_MISMATCH)"),
    # what no set holds is still the state of the rung's first run: the stamp, a byte no field names
    (lambda s: setattr(s, "stamp", "0x9c45 != live set 0x1234"),
     OTHER + "stamp: 0x9c45 = live set -> 0x9c45 != live set 0x1234"),
    (an_unnamed_byte, OTHER + "byte 0x007f: 00 -> a5"),
], ids=["a config field", "a bool", "the slots", "the table's state", "a table point", "the data state",
        "the stamp", "a byte no field names"])
def test_what_a_later_run_reads_differently_is_named(bus, tmp_path, change, said):
    r = {"dir": tmp_path / "rung", "set": bus["set"]}
    sc.record_constants(r, 1)
    sc.record_constants(r, 2)
    change(bus["servo"])
    with pytest.raises(SystemExit) as e:
        sc.record_constants(r, 3)
    assert str(e.value).endswith(f"run-3 {said}")
    assert sorted(p.name for p in r["dir"].iterdir()) == [sc.SET_COPY, "run-1", "run-2"]
    assert constants(r["dir"], 2) == constants(r["dir"], 1) == r["constants"]


def test_a_servo_whose_descriptor_names_no_slots_records_none(bus, monkeypatch, tmp_path):
    monkeypatch.setattr(sc.osc, "field", lambda name: None)
    doc = {**bus["set"], "wide_codes": None}
    r = {"dir": tmp_path / "rung", "set": {**doc, "hash": cn.hashes(doc)}}
    sc.record_constants(r, 1)
    assert constants(r["dir"], 1)["wide_codes"] is None


def test_a_dump_that_names_no_control_field_is_refused(bus, monkeypatch, tmp_path):
    monkeypatch.setattr(sc.osc, "dump", lambda: [(0, "model_number", "ro", 257)])
    with pytest.raises(SystemExit, match="names no torque_enable"):
        sc.record_constants({"dir": tmp_path / "rung", "set": bus["set"]}, 1)
    assert not (tmp_path / "rung").exists()


def test_a_rung_recorded_without_the_constant_state_scores_the_same(bus):
    rung = sc.run("00-baseline", "abc123", "f" * 40, runs=1, servo=SERVO, board=BOARD)
    path = sc.result_path(sc.pair_of(SERVO, BOARD), "00-baseline")
    sc.score("00-baseline", servo=SERVO, board=BOARD)
    with_state = json.loads(path.read_text())
    # the rung as one recorded before: no servo.json beside its captures, no hash in rung.json
    (rung / "run-1" / "servo.json").unlink()
    doc = json.loads((rung / "rung.json").read_text())
    (rung / "rung.json").write_text(json.dumps({k: v for k, v in doc.items() if k != "constants_hash"}))
    sc.score("00-baseline", servo=SERVO, board=BOARD)
    without = json.loads(path.read_text())
    assert "constants_hash" not in without and without["runs"] == ["run-1"]
    assert without == {k: v for k, v in with_state.items() if k != "constants_hash"}
    buf = io.StringIO()
    sc.table(servo=SERVO, board=BOARD, file=buf)
    assert "00-baseline" in buf.getvalue().splitlines()[0]


DUMP = ("note: descriptor osc-servo is fw 0.1, servo reports fw 0.2; fields added since are unnamed\n"
        "0x0000  model_number                 ro  257\n"
        "0x004a  stall_response               rw  Yield (1)\n"
        "0x004b  drive_polarity               rw  on\n"
        "0x0060  derate_start_cc              rw  -200\n"
        "0x00d0  gear_class                   rw  2 (metal)\n"
        "0x00d1  adc_wide_mid_q4              rw  00 00 00 58\n")


def test_osc_dump_reads_a_row_a_field_off_the_tools_lines(monkeypatch):
    seen = []
    monkeypatch.setattr(sc.osc, "run", lambda *a, **kw: seen.append((a, kw)) or DUMP)
    assert sc.osc.dump() == [
        (0x00, "model_number", "ro", 257), (0x4A, "stall_response", "rw", "Yield (1)"),
        (0x4B, "drive_polarity", "rw", "on"), (0x60, "derate_start_cc", "rw", -200),
        (0xD0, "gear_class", "rw", "2 (metal)"), (0xD1, "adc_wide_mid_q4", "rw", "00 00 00 58")]
    assert seen == [(("dump",), {"capture": True})]
    monkeypatch.setattr(sc.osc, "run", lambda *a, **kw: "note: nothing a field line\n")
    with pytest.raises(RuntimeError, match="unexpected output"):
        sc.osc.dump()


def test_osc_read_takes_a_long_span_a_status_frame_at_a_time(monkeypatch):
    table = bytes(i % 251 for i in range(0x180))
    seen = []

    def run(*a, **kw):
        seen.append(a)
        addr, n = int(a[1], 16), a[2]
        return "".join(f"{addr + i:#06x}  {table[addr + i:addr + min(i + 16, n)].hex(' ')}\n"
                       for i in range(0, n, 16))

    monkeypatch.setattr(sc.osc, "run", run)
    assert sc.osc.read(0, 0x180) == table
    assert seen == [("read", "0x0", 252), ("read", "0xfc", 132)]


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
    repo(monkeypatch, tmp_path)
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
    repo(monkeypatch, tmp_path)
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
    repo(monkeypatch, tmp_path)
    _, segs, plan_doc = resting_on(CENTER, bump_points(40))
    cap_dir, _ = table_rung(tmp_path, bump_points(40), "LIVE", segs, plan_doc, named=False)
    if rung_json == "absent":
        (cap_dir.parent.parent / "rung.json").unlink()
    with pytest.raises(ValueError, match="LIVE and the rung names none"):
        sc.load(cap_dir)


def test_a_live_rung_whose_table_changed_since_is_refused(tmp_path, monkeypatch):
    repo(monkeypatch, tmp_path)
    _, segs, plan_doc = resting_on(CENTER, bump_points(40))
    cap_dir, path = table_rung(tmp_path, bump_points(40), "LIVE", segs, plan_doc)
    path.write_text(path.read_text().replace("[-90.0, 90.0]", "[-91.0, 90.0]"))
    with pytest.raises(ValueError, match="not the table the rung ran on"):
        sc.load(cap_dir)


def test_identity_rows_score_bit_for_bit_as_before(tmp_path, monkeypatch):
    """An IDENTITY capture's rows are the file's raw pot untouched, and the
    same rows under a LIVE all-zero table score exactly the same."""
    repo(monkeypatch, tmp_path)
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


def test_a_set_on_the_table_gives_inputs_that_name_the_tables_scale_and_file():
    pair = sc.pair_of(SERVO, BOARD)
    doc = cn.of_record(pair)
    t = postable.load(pair)[0]
    inp = TS.inputs(SERVO, pair, BOARD.key, doc)
    assert inp["counts_per_deg"] == t.scale == doc["scale"]["counts_per_deg"]
    assert t.scale != TS.inputs(SERVO, pair)["counts_per_deg"]
    np.testing.assert_array_equal(inp["table"].points, t.points)
    f = str(results.rel(postable.out_path(pair)))
    assert inp["sources"]["counts_per_deg"] == f"{f} target counts_per_deg, the table's linearized counts per deg"
    assert inp["sources"]["center_counts"].endswith(f)
    guard = TS.guard_of(doc)
    center, sets = TS.build(guard, inp)
    plan_doc = json.loads(json.dumps(TS.plan_doc("steps", sets["steps"], center, guard, inp)))
    assert plan_doc["counts_per_deg"] == t.scale and plan_doc["sources"] == inp["sources"]


def test_the_plans_of_a_rung_on_the_table_rebuild_byte_for_byte_from_the_set_of_record():
    """`13a`'s recorded plans, run-1, every capture: the pair's set of
    record holds the table, the slots and the soft range that rung ran
    on, so a rung on it is given the same goals, the wide holds among
    them. The `wide` plan also reads the board's wide-code result, for
    the codes it skips: the file the set names by its sha1."""
    pair = sc.pair_of(SERVO, BOARD)
    doc = cn.of_record(pair)
    assert cn.status(doc)[1] is False
    run = results.ROOT / "telemetry" / pair / "scorecard" / "13a-wide-step-on-table" / "run-1"
    inp, guard = TS.inputs(SERVO, pair, BOARD.key, doc), TS.guard_of(doc)
    center, sets = TS.build(guard, inp)
    assert list(sets) == list(TS.CAPTURES)
    for name in TS.CAPTURES:
        assert json.dumps(TS.plan_doc(name, sets[name], center, guard, inp), indent=2) + "\n" == \
            (run / name / "plan.json").read_text(), name
    # and the wide holds are the goals that rung's record names
    recorded = json.loads((run.parent / "rung.json").read_text())["wide_goals"]
    assert [{k: g[k] for k in ("stuck_code", "side", "goal_counts", "raw_counts")}
            for g in sets["wide"] if g["test"] == "wide-hold"] == recorded


def test_every_deviation_of_record_derives_from_the_set_of_record_and_moves_its_pins_alone():
    pair = sc.pair_of(SERVO, BOARD)
    base = cn.of_record(pair)
    files = sorted((results.ROOT / "stages" / sc.STAGE / "scorecard" / "deviations").glob("*.json"))
    assert files
    for f in files:
        dev = json.loads(f.read_text())
        doc, _ = cn.derive(base, f)
        assert doc["deviation"]["fields"] == {n: [base["fields"][n]["value"], pin["value"]]
                                              for n, pin in dev["pins"].items()}, f.name
        assert all(doc["fields"][n]["class"] == "pinned" and pin["why"] and pin["rung"]
                   for n, pin in dev["pins"].items())
        assert "/Users/" not in f.read_text() and "/home/" not in f.read_text()


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
    held = json.loads(sc.result_path(pair, "12a-pos-table-live").read_text())
    scored = results._plain(sc.score_rung(rung))
    assert held["current_scale"] == scored["current_scale"] == {"counts_per_a": CPA, "from": sc.BOARD_SCALE}
    want, got = held["scores"], scored["scores"]
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
    repo(monkeypatch, tmp_path)
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
    i_ma = 300 * synth.I_PER_DUTY * A_PER_COUNT * 1e3
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
    repo(monkeypatch, tmp_path)
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
    repo(monkeypatch, tmp_path)
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
    repo(monkeypatch, tmp_path)
    _, rows, plan_doc = stuck_rows()
    cap_dir, path = wide_rung(tmp_path, [(1023, 10)], rows, plan_doc)
    sc.load(cap_dir)
    synth.wide_result(path, [(1023, 11)])
    with pytest.raises(ValueError, match="not the wide-code result the rung ran on"):
        sc.load(cap_dir)
    path.unlink()
    with pytest.raises(ValueError, match="not the wide-code result the rung ran on"):
        sc.load(cap_dir)
