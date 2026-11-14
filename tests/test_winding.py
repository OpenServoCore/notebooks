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


# A two-node chain as earlier work on the family prints it, another MG90:
# name: (R_wc C/W, C_w J/C, R_ca C/W, C_c J/C), then the seconds it
# prints for a hard stall to reach 130 C (7.43 V over 4.74 ohm from 27 C).
CHAINS = {
    "anchored": ((28.00, 1.30, 31.97, 2.22), 17.4),
    "r_wc 40": ((40.00, 1.00, 31.39, 2.34), 12.5),
    "r_wc 25": ((25.00, 0.90, 32.13, 3.12), 12.7),
}


def test_a_chain_s_parallel_terms_are_its_modes_and_sum_to_its_branches():
    (g_fast, tau_fast), (g_slow, tau_slow) = wd.parallel_of(
        *CHAINS["anchored"][0])
    # the chain's source prints its modes at 20.05 and 128.92 s and the
    # winding over the air at 60.0 C/W
    assert tau_fast == pytest.approx(20.0, abs=0.1)
    assert tau_slow == pytest.approx(128.9, abs=0.1)
    assert g_fast + g_slow == pytest.approx(60.0, abs=0.05)
    assert g_fast + g_slow == pytest.approx(28.00 + 31.97, rel=1e-12)
    # the slow term is the winding's, not the can's branch
    assert g_slow == pytest.approx(52.8, abs=0.05)
    assert g_fast == pytest.approx(7.2, abs=0.05)


def test_the_parallel_terms_step_as_the_chain_does():
    r_wc, c_w, r_ca, c_c = CHAINS["anchored"][0]
    terms = wd.parallel_of(r_wc, c_w, r_ca, c_c)
    t_w = t_c = 0.0                     # the chain itself, a watt in
    dt = 1e-3
    for k in range(1, 300_001):
        flow = (t_w - t_c) / r_wc
        t_w += dt * (1.0 - flow) / c_w
        t_c += dt * (flow - t_c / r_ca) / c_c
        if k in (5_000, 60_000, 300_000):
            t = k * dt
            parallel = sum(g * (1 - math.exp(-t / tau)) for g, tau in terms)
            assert t_w == pytest.approx(parallel, rel=2e-4)


def stall_seconds(terms, volts=7.43, r_ohm=4.74, start_c=27.0, limit_c=130.0):
    """Seconds for the winding to reach limit_c at constant volts, the
    current volts / R(T_w), copper's line taken at start_c."""
    gains, taus = [g for g, _ in terms], [tau for _, tau in terms]
    alpha = 1 / (234.5 + start_c)
    x = wd.start_state(len(terms))
    dt, t = 1e-3, 0.0
    while wd.estimate(x, start_c) < limit_c:
        i = volts / wd.r_of_t(wd.estimate(x, start_c), r_ohm, start_c, alpha)
        x, _ = wd.step(x, i * i, start_c, dt, gains, taus, r_ohm, start_c,
                       alpha)
        t += dt
    return t


@pytest.mark.parametrize("name", list(CHAINS))
def test_the_parallel_terms_reproduce_a_chain_s_stall_time(name):
    row, printed_s = CHAINS[name]
    assert stall_seconds(wd.parallel_of(*row)) == pytest.approx(
        printed_s, abs=0.05)


def test_a_written_leak_runs_at_the_nearest_octave_of_the_period():
    # 16 ms periods: 1 s is 62, over 1.5 x 32, so it runs at 64
    assert wd.leak_shift(0) == 0
    assert wd.leak_shift(1) == 6
    assert wd.leak_s(1) == pytest.approx(1.024)
    assert wd.leak_s(2) == wd.leak_s(3) == pytest.approx(2.048)
    assert wd.leak_s(20) == pytest.approx(16.384)
    assert wd.leak_s(115) == wd.leak_s(129) == pytest.approx(131.072)
    assert wd.leak_s(60_000) == pytest.approx(0.016 * 2 ** 15)


def test_a_stored_rise_means_its_gain_back():
    r_ohm, counts_per_a = 4.0, 1100.0
    rise = wd.rise_per_count2(25.0, r_ohm, counts_per_a)
    # 25 C/W x 4 ohm = 100 C per A^2, in centi-C per count squared
    assert rise == pytest.approx(100 * 100.0 / counts_per_a ** 2, rel=1e-15)
    assert wd.gain_of_rise(rise * 65536, r_ohm, counts_per_a) == (
        pytest.approx(25.0, rel=1e-12))


def test_the_stored_integers_follow_the_float_estimate():
    r_ohm, counts_per_a, counts = 4.0, 1100.0, 550
    fields = (2, 450, 129, 700)         # leak s, rise, leak s, rise
    gains = [wd.gain_of_rise(fields[1], r_ohm, counts_per_a),
             wd.gain_of_rise(fields[3], r_ohm, counts_per_a)]
    taus = [wd.leak_s(fields[0]), wd.leak_s(fields[2])]
    stored = wd.stored_rise_c(fields, [(600.0, counts)])
    assert len(stored) == round(600.0 / wd.STORED_PERIOD_S)
    x = wd.start_state(2)
    alpha = wd.ALPHA_Q26 / 2 ** 26 * 100    # the stored copper slope, per C
    for k in range(1, len(stored) + 1):
        x, _ = wd.step(x, (counts / counts_per_a) ** 2, 25.0,
                       wd.STORED_PERIOD_S, gains, taus, r_ohm, 25.0, alpha)
        if k in (63, 188, 625, 3750, 37500):    # 1, 3, 10, 60, 600 s
            assert stored[k - 1] == pytest.approx(sum(x), abs=0.3)
    assert stored[-1] > 60.0            # the warm input is in the plateau


def test_a_second_leak_of_zero_drops_the_second_node():
    one = wd.stored_rise_c((115, 1143, 0, 0), [(60.0, 568), (60.0, 0)])
    two = wd.stored_rise_c((115, 1143, 0, 999), [(60.0, 568), (60.0, 0)])
    assert (one == two).all()
    assert one[3749] > one[-1] > 0.0    # it cools once the current stops


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
