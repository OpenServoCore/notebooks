"""The winding-temperature estimate on synthetic power, and its grading
pipeline (the NTC track, the stopped-rotor read, the gate, the heat
history) on a hand-made sweep."""
import json
import math
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest

from oscnb import boards
from oscnb import speed as sp
from oscnb import winding as wd

FS = 20_000
ARR = 1200
BOARD = boards.BOARDS["dev-v006-2A-D"]
# flat edge lags: the on-share does not move with the rail
CHAIN = sp.Chain(a_per_count=1e-3, tap_counts_per_v=250.0,
                 tap_offset_counts=2.0, half_period_ticks=ARR,
                 lag_rail_v=(4.0, 8.0), lag_on_ticks=(40.0, 40.0),
                 lag_off_ticks=(12.0, 12.0), fs_hz=FS)
MOTOR = sp.Motor(r_ohm=4.0, t0_c=25.0, alpha_per_c=sp.ALPHA_CU, v0_v=0.03,
                 g_s=5e-3, brake_b0_v=0.01, brake_b1_ohm=0.25, l_h=1e-3,
                 ke_v_per_rev_s=7e-3)
ZERO_I = 1200.0         # counts, the asleep zero
ZERO_OVER = 5.0         # counts over it, the zero the read takes
BIAS = 773.0            # counts, the bias node
PAIR = "mg90-x__dev-v006-2A-D"
QUIET_SD = 9.0


def test_one_node_under_constant_power_rises_as_g_p():
    g, tau, i2, r0 = 23.2, 2.38, 0.25, 4.0
    p = i2 * r0
    dt = 0.05
    n = 200
    starts, power = wd.run([j * dt for j in range(n)], [dt] * n, [i2] * n,
                           [25.0] * n, [g], [tau], r0, 25.0, alpha=0.0)
    t = np.arange(n) * dt
    assert starts[0] == pytest.approx(g * p * (1 - np.exp(-t / tau)),
                                      rel=1e-12, abs=1e-12)
    assert power == pytest.approx([p] * n, rel=1e-15)


def test_a_step_is_exact_for_any_dt():
    args = ([23.2], [2.38], 4.0, 25.0)
    one, _ = wd.step([0.5], 0.3, 26.0, 1.0, *args, alpha=0.0)
    x = [0.5]
    for _ in range(1000):
        x, _ = wd.step(x, 0.3, 26.0, 1e-3, *args, alpha=0.0)
    assert x == pytest.approx(one, rel=1e-12)


def test_with_no_power_the_rise_decays_with_tau():
    tau = 2.38
    x, p = wd.step([3.0], 0.0, 25.0, 1.5, [23.2], [tau], 4.0, 25.0)
    assert p == 0.0
    assert x[0] == pytest.approx(3.0 * math.exp(-1.5 / tau), rel=1e-15)
    # the time between pieces passes at zero current
    starts, _ = wd.run([0.0, 4.0], [1.0, 1.0], [0.0, 0.0], [25.0, 25.0],
                       [23.2], [tau], 4.0, 25.0, state=[3.0])
    assert starts[0][1] == pytest.approx(3.0 * math.exp(-4.0 / tau),
                                         rel=1e-15)


def test_two_nodes_rise_as_their_sum():
    gains, taus = [10.0, 13.0], [0.8, 40.0]
    t0 = [0.0, 0.5, 1.0, 3.0]
    dt = [0.5, 0.5, 2.0, 1.0]
    i2 = [0.2, 0.3, 0.1, 0.0]
    ntc = [25.0, 25.1, 25.2, 25.3]
    both, _ = wd.run(t0, dt, i2, ntc, gains, taus, 4.0, 25.0, alpha=0.0)
    for k in range(2):
        alone, _ = wd.run(t0, dt, i2, ntc, gains[k:k + 1], taus[k:k + 1],
                          4.0, 25.0, alpha=0.0)
        assert both[k] == pytest.approx(alone[0], rel=1e-15)
    rise = [both[0][j] + both[1][j] for j in range(len(t0))]
    est = [wd.estimate([both[0][j], both[1][j]], ntc[j])
           for j in range(len(t0))]
    assert est == pytest.approx([n + r for n, r in zip(ntc, rise)])


def test_the_power_takes_the_estimate_s_own_resistance():
    i2, r0, t0 = 0.25, 4.0, 25.0
    _, cold = wd.step([0.0], i2, t0, 0.01, [23.2], [2.38], r0, t0)
    _, hot = wd.step([10.0], i2, t0, 0.01, [23.2], [2.38], r0, t0)
    assert cold == pytest.approx(i2 * r0, rel=1e-15)
    assert hot == pytest.approx(i2 * r0 * (1 + sp.ALPHA_CU * 10.0),
                                rel=1e-15)
    assert hot > cold


def test_t_of_r_inverts_r_of_t_and_one_percent_is_2_56_c():
    for t in (-10.0, 25.0, 61.5):
        r = wd.r_of_t(t, 4.1767, 26.26)
        assert wd.t_of_r(r, 4.1767, 26.26) == pytest.approx(t, abs=1e-12)
    assert wd.t_of_r(1.01 * 4.0, 4.0, 25.0) - 25.0 == pytest.approx(
        2.56, abs=0.005)


def test_inside_is_the_closed_form_of_step_over_a_constant_span():
    g, tau, i2, r0 = 23.2, 2.38, 0.3, 4.0
    starts, power = wd.run([0.0], [2.0], [i2], [25.0], [g], [tau], r0, 25.0,
                           alpha=0.0, state=[1.5])
    for since in (0.0, 0.3, 1.0, 2.0):
        x, _ = wd.step([1.5], i2, 25.0, since, [g], [tau], r0, 25.0,
                       alpha=0.0)
        assert wd.inside(starts[0][0], power[0], since, g, tau) == (
            pytest.approx(x[0], rel=1e-12))


def test_speed_error_of_a_resistance_error():
    # 1 % of 4 ohm at 0.5 A is 20 mV, over 7 mV per rev/s
    assert wd.speed_error_rev_s(0.01, 0.5, 4.0, 7e-3) == pytest.approx(
        0.02 / 7e-3, rel=1e-15)


def write_manifest(pair):
    d = pair / "sittings"
    d.mkdir(parents=True)
    pd.DataFrame([
        {"t_host": 200.0, "event": "probe",
         "detail": "late/x: pos 2000, ntc_raw 1800, vbus_raw 3000"},
        {"t_host": 150.0, "event": "note", "detail": "center"},
        {"t_host": 100.0, "event": "probe",
         "detail": "early/x: pos 1990, ntc_raw 2000, vbus_raw 3100"},
    ]).to_csv(d / "s-1.csv", index=False)


def test_the_ntc_track_interpolates_between_probes_and_holds_the_ends(
        tmp_path):
    pair = tmp_path / PAIR
    write_manifest(pair)
    scales = SimpleNamespace(rail_v=lambda c: np.asarray(c, float) / 400)
    pr = wd.probes(pair, BOARD, scales)
    assert list(pr["where"]) == ["early/x", "late/x"]
    assert list(pr["t"]) == [100.0, 200.0]
    assert list(pr["rail_v"]) == [7.75, 7.5]
    c0, c1 = BOARD.ntc_c_beta(np.array([2000.0, 1800.0]))
    assert list(pr["ntc_c"]) == [c0, c1]
    got = wd.ntc_at([50.0, 100.0, 125.0, 200.0, 900.0], pr)
    assert got == pytest.approx([c0, c0, c0 + 0.25 * (c1 - c0), c1, c1],
                                rel=1e-15)


H = 600.0               # half on-ticks, 50 %
COLS = ["seg", "cmd_duty_q15", "dir", "tick", "window_valid", "pos",
        "duty_counts", "current_raw", "vmotor_a", "vmotor_b"]


def stalled_rows(r_ohm, i_w, n, h=H, motor=MOTOR):
    """(crest, tap_a, tap_b) of n rows at a stopped rotor, drive +1: V_c =
    V0 + R i_w, inverted through oscnb.speed's construction."""
    on, off = sp.edge_lags(6.0, CHAIN)
    s = (2 * h - on + off) / (2 * ARR)
    v_c = motor.v0_v + r_ohm * i_w
    drop = motor.brake_b0_v + motor.brake_b1_ohm * i_w
    v_on = (v_c + (1 - s) * drop) / s
    crest = (ZERO_I + ZERO_OVER
             + (i_w + motor.g_s * (1 - h / ARR) * v_on) / CHAIN.a_per_count)
    diff = v_on * CHAIN.tap_counts_per_v + CHAIN.tap_offset_counts
    return (np.full(n, crest), np.full(n, BIAS + diff), np.full(n, BIAS))


def write_sweep(folder, t0, segs, meta=None):
    """segs: dicts of n (rows; 0 for an unstreamed span of `span_s`),
    dir, cmd, duty, crest, va, vb, pos (scalars or arrays)."""
    folder.mkdir(parents=True)
    rows, table, t = [], [], t0
    for k, s in enumerate(segs):
        n = s["n"]
        dur = s.get("span_s", n / FS)
        table.append(dict(seg=k, dir=s.get("dir", 0),
                          cmd_duty_q15=s.get("cmd", 0),
                          streamed=int(n > 0), samples=n, t_start=t,
                          t_end=t + dur))
        t += dur
        if n == 0:
            continue
        cols = {"seg": k, "cmd_duty_q15": s.get("cmd", 0),
                "dir": s.get("dir", 0), "tick": np.arange(n),
                "window_valid": s.get("valid", 1 if s.get("duty") else 0),
                "pos": s.get("pos", 2000),
                "duty_counts": s.get("dir", 0) * s.get("duty", 0),
                "current_raw": s.get("crest", ZERO_I),
                "vmotor_a": s.get("va", BIAS), "vmotor_b": s.get("vb", BIAS)}
        rows.append(pd.DataFrame({c: np.broadcast_to(cols[c], n)
                                  for c in COLS}))
    pd.concat(rows).to_csv(folder / "sweep.csv", index=False)
    pd.DataFrame(table).to_csv(folder / "segments.csv", index=False)
    m = {"tick_hz": FS, "pwm_period_ticks": ARR, "sense": BOARD.sense,
         "schedule": [], "rows_dropped": 0, "stall": False,
         "static_load": True}
    (folder / "meta.json").write_text(json.dumps({**m, **(meta or {})}))


def drive(r_ohm, i_w, n=4000, **kw):
    crest, va, vb = stalled_rows(r_ohm, i_w, n)
    return dict(n=n, dir=1, cmd=16384, duty=int(H), crest=crest, va=va,
                vb=vb, **kw)


R_TRUE = 4.3
I_TRUE = 0.5


def read_tree(tmp_path):
    """One stopped-rotor sweep: a clean segment (fresh), then segments at
    the same duty whose pot moves, whose current is noisy, and that hold
    one dropout row (in their third sub-window)."""
    pair = tmp_path / PAIR
    clean = drive(R_TRUE, I_TRUE)
    moving = drive(R_TRUE, I_TRUE, pos=2000 + np.arange(4000) // 20)
    noisy = drive(R_TRUE, I_TRUE)
    noisy["crest"] = noisy["crest"] + 15.0 * (-1.0) ** np.arange(4000)
    dropout = drive(R_TRUE, I_TRUE)
    dropout["crest"] = dropout["crest"].copy()
    dropout["crest"][2 * wd.SUB_ROWS + 7] = ZERO_I
    write_sweep(pair / "resistance/stop-1/chains/0", 100.0,
                [dict(n=2000), clean, moving, noisy, dropout])
    return pair


def reads_of(pair, **kw):
    ix = wd.index(pair)
    stopped = wd.stopped_sweeps(ix, [("resistance", "stop-1")])
    probe = pd.DataFrame({"t": [0.0], "ntc_c": [26.0]})
    sub = wd.sub_windows(pair, stopped, BOARD, CHAIN, MOTOR, ZERO_OVER,
                         probe)
    ok = wd.gate(sub, kw.pop("quiet_sd", QUIET_SD), **kw)
    return sub, ok, wd.read_table(sub, ok)


def test_rows_from_a_known_r_at_a_stopped_rotor_come_back_as_that_r(
        tmp_path):
    sub, ok, reads = reads_of(read_tree(tmp_path))
    clean = sub[sub["seg"] == 1]
    assert clean["r_ohm"].to_numpy() == pytest.approx(R_TRUE, rel=1e-9)
    assert clean["i_w_a"].to_numpy() == pytest.approx(I_TRUE, rel=1e-9)
    assert clean["dT_c"].to_numpy() == pytest.approx(
        (R_TRUE / MOTOR.r_ohm - 1) / sp.ALPHA_CU, rel=1e-9)
    assert clean["ntc_c"].to_numpy() == pytest.approx(26.0)
    r = reads.set_index("seg").loc[1]
    assert r["r_ohm"] == pytest.approx(R_TRUE, rel=1e-9)
    assert r["slope_pct_per_s"] == pytest.approx(0.0, abs=1e-9)
    # a fresh duty is read from SETTLE_S on: sub-windows 0 to 2 wind up
    assert list(ok[sub["seg"] == 1]) == [False] * 3 + [True] * 7
    assert r["n_sub"] == 7


def test_the_gate_refuses_a_moving_pot_a_noisy_current_and_a_dropout(
        tmp_path):
    sub, ok, reads = reads_of(read_tree(tmp_path))
    by = {k: list(ok[sub["seg"] == k]) for k in (2, 3, 4)}
    assert sub.loc[sub["seg"] == 2, "pos_range"].min() > wd.POS_RANGE_MAX
    assert not any(by[2])
    assert sub.loc[sub["seg"] == 3, "i_sd_counts"].min() > QUIET_SD
    assert not any(by[3])
    assert by[4] == [True] * 2 + [False] + [True] * 7
    assert sub.loc[sub["seg"] == 4, "dropouts"].tolist() == (
        [0, 0, 1] + [0] * 7)
    assert sorted(reads["seg"]) == [1, 4]
    # each refusal is its own rule: loosen it and the window reads R
    sub2, ok2, reads2 = reads_of(read_tree(tmp_path / "b"), quiet_sd=1e9,
                                 pos_range_max=1e9, creep_max=1e9)
    assert sorted(reads2["seg"]) == [1, 2, 3, 4]
    assert reads2["r_ohm"].to_numpy() == pytest.approx(R_TRUE, rel=1e-9)
    assert not ok2[(sub2["seg"] == 4) & (sub2["sub"] == 2)].any()


def test_the_heat_history_counts_streams_and_unstreamed_holds(tmp_path):
    pair = tmp_path / PAIR
    i1, i3, hold_s = 0.4, 0.6, 2.0
    write_sweep(pair / "load/steps-1/chains/0", 100.0, [
        dict(n=2000), drive(R_TRUE, i1, n=2000),
        dict(n=0, dir=1, cmd=16384, span_s=hold_s),
        drive(R_TRUE, i3, n=2000)])
    ix = wd.index(pair)
    pieces, table = wd.heat(pair, ix, BOARD, CHAIN, MOTOR, ZERO_OVER,
                            10.0, 300.0)
    r0 = MOTOR.r_ohm
    t = table.iloc[0]
    assert t["j_stream"] == pytest.approx((i1 ** 2 + i3 ** 2) * r0 * 0.1,
                                          rel=1e-9)
    assert t["j_hold"] == pytest.approx(
        0.5 * (i1 ** 2 + i3 ** 2) * hold_s * r0, rel=1e-9)
    assert t["j_hold_hi"] == pytest.approx(
        i3 ** 2 * wd.STALL_HEADROOM ** 2 * hold_s * r0, rel=1e-9)
    assert t["j_seek"] == 0.0
    assert t["j_total"] == pytest.approx(
        t["j_stream"] + t["j_hold"], rel=1e-15)
    hold = pieces["kind"] == wd.HOLD
    assert hold.sum() == round(hold_s / wd.HOLD_PIECE_S)
    assert np.all(np.diff(pieces["t0"]) >= 0)
    # the pieces carry the joules the table counts
    e = (pieces["i2"] * pieces["dt"]).sum() * r0
    assert e == pytest.approx(t["j_total"], rel=1e-9)
    s = wd.heat_sittings(table)
    assert s["unstreamed_pct"].iloc[0] == pytest.approx(
        100 * t["j_hold"] / t["j_total"])
