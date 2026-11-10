"""Stage 3's windows on a hand-made tree: a spin's window is its held
duty's last rows, a hold's its last 400 ms, a window's global rows its
sweep's offset plus its own; the load holds' still gate; the steps' fit
recovers a first-order step from its crossings."""
import json
from types import SimpleNamespace

import numpy as np
import pandas as pd
import pytest

from oscnb import boards
from oscnb import speed as sp
from oscnb import sweeps, windows

FS = 20_000
ARR = 1200
BOARD = boards.BOARDS["dev-v006-2A-D"]
PAIR = "mg90-x__dev-v006-2A-D"
CHAIN = sp.Chain(a_per_count=1e-3, tap_counts_per_v=250.0,
                 tap_offset_counts=2.0, half_period_ticks=ARR,
                 lag_rail_v=(4.0, 8.0), lag_on_ticks=(40.0, 40.0),
                 lag_off_ticks=(12.0, 12.0), fs_hz=FS)
MOTOR = sp.Motor(r_ohm=4.0, t0_c=25.0, alpha_per_c=sp.ALPHA_CU, v0_v=0.03,
                 g_s=5e-3, brake_b0_v=0.01, brake_b1_ohm=0.25, l_h=1e-3,
                 ke_v_per_rev_s=7e-3)
SERVO = SimpleNamespace(gear_ratio=300.0, counts_per_motor_rev=10.0,
                        motor_entry=SimpleNamespace(ripple_per_rev=6))
POT = pd.DataFrame({"counts": [0.0, 1000.0, 1000.0, 4000.0],
                    "angle_deg": [-100.0, -50.0, -50.0, 100.0]})
ZERO_I = 1200           # counts, the asleep zero
AWAKE = 1210            # counts, a coast's crest at rest
BIAS = 773
COLS = ["seg", "cmd_duty_q15", "dir", "tick", "window_valid", "pos",
        "duty_counts", "current_raw", "current_trough", "vmotor_a",
        "vmotor_b"]


def seg(n, d=0, cmd=0, duty=0, crest=ZERO_I, pos=2000, va=BIAS):
    """A segment of n rows; duty, crest, pos and va scalars or arrays."""
    return dict(n=n, dir=d, cmd=cmd, duty=duty, crest=crest, pos=pos, va=va)


def write_sweep(folder, t0, segs):
    folder.mkdir(parents=True)
    rows, table, t = [], [], t0
    for k, s in enumerate(segs):
        n = s["n"]
        table.append(dict(seg=k, dir=s["dir"], cmd_duty_q15=s["cmd"],
                          streamed=1, samples=n, t_start=t,
                          t_end=t + n / FS))
        t += n / FS
        duty = np.broadcast_to(s["duty"], n)
        cols = {"seg": k, "cmd_duty_q15": s["cmd"], "dir": s["dir"],
                "tick": np.arange(n), "window_valid": (duty > 0) * 1,
                "pos": s["pos"], "duty_counts": s["dir"] * duty,
                "current_raw": s["crest"], "current_trough": s["crest"],
                "vmotor_a": s["va"], "vmotor_b": BIAS}
        rows.append(pd.DataFrame({c: np.broadcast_to(cols[c], n)
                                  for c in COLS}))
    pd.concat(rows).to_csv(folder / "sweep.csv", index=False)
    pd.DataFrame(table).to_csv(folder / "segments.csv", index=False)
    meta = {"tick_hz": FS, "pwm_period_ticks": ARR, "sense": BOARD.sense,
            "schedule": [], "rows_dropped": 0}
    (folder / "meta.json").write_text(json.dumps(meta))


SPIN_REV_S = 150.0      # rev/s, a turning spin's speed: a 900 Hz ripple


def spin(held_rows, climb_rows=100, turning=False):
    """Baseline, a spin climbing to 30 % then held, a coast. Turning, the
    crest carries the commutation ripple and the coast an EMF that runs
    down with its ripple, then rests; else the shaft never turns."""
    duty = np.r_[np.full(climb_rows, 200), np.full(held_rows, 360)]
    crest, va = 1300, BIAS
    if turning:
        f = SPIN_REV_S * SERVO.motor_entry.ripple_per_rev
        t = np.arange(len(duty)) / FS
        crest = np.round(1300 + 20 * np.sin(2 * np.pi * f * t))
        w = SPIN_REV_S * np.maximum(1 - np.arange(2400) / 4000, 0)
        w[2000:] = 0.0
        phase = 2 * np.pi * SERVO.motor_entry.ripple_per_rev * np.cumsum(w)
        emf = MOTOR.ke_v_per_rev_s * w * (1 + 0.1 * np.sin(phase / FS))
        va = np.round(BIAS + emf * CHAIN.tap_counts_per_v)
    return [seg(2000), seg(len(duty), 1, 9830, duty, crest),
            seg(2400, 1, crest=AWAKE, va=va)]


def low_cycle():
    """Baseline, a kick, a hold that settles to 200 ticks, a coast."""
    duty = np.r_[np.full(50, 300), np.full(10000, 200)]
    return [seg(2000), seg(300, -1, -16000, 600, 1400),
            seg(len(duty), -1, -5000, duty, 1250), seg(2400, -1, crest=AWAKE)]


def write_table(folder, name, folders):
    pd.DataFrame({"folder": folders, "k": range(len(folders)),
                  "repeat": 0, "volts_set": 5.0, "t_start": 0.0,
                  "vsys_read": 5.1, "side": 1}).to_csv(folder / name,
                                                       index=False)


def write_manifest(pair, rels):
    d = pair / "sittings"
    d.mkdir(parents=True)
    pd.DataFrame([{"t_host": 10.0 + j, "event": "probe",
                   "detail": f"telemetry/{PAIR}/{r}: pos 2000, "
                             f"ntc_raw 2000, vbus_raw {3000 + j}"}
                  for j, r in enumerate(rels)]).to_csv(d / "s-1.csv",
                                                       index=False)


def tree(tmp_path):
    """An unrelated sweep first, so every window's sweep has an offset."""
    pair = tmp_path / PAIR
    write_sweep(pair / "other/x/0", 1.0, [seg(700)])
    lad = pair / "ladder/set-1"
    write_sweep(lad / "cycles/0", 10.0, spin(5000, turning=True))
    write_sweep(lad / "cycles/1", 20.0, spin(1000))
    write_table(lad, "cycles.csv", ["cycles/0", "cycles/1"])
    low = pair / "ladder/low-1"
    write_sweep(low / "cycles/0", 30.0, low_cycle())
    write_table(low, "cycles.csv", ["cycles/0"])
    write_manifest(pair, ["ladder/set-1/cycles/0", "ladder/set-1/cycles/1",
                          "ladder/low-1/cycles/0"])
    return pair


def pair_of(pair):
    store = sweeps.row_store(pair)
    probes = pd.DataFrame({
        "manifest": "s-1", "t": [10.0, 11.0, 12.0],
        "where": [f"telemetry/{PAIR}/ladder/set-1/cycles/0",
                  f"telemetry/{PAIR}/ladder/set-1/cycles/1",
                  f"telemetry/{PAIR}/ladder/low-1/cycles/0"],
        "rail_v": [5.0, 5.2, 5.4], "ntc_c": [25.0, 26.0, 27.0]})
    return windows.Pair(pair, store.sweeps, BOARD, CHAIN, MOTOR, SERVO,
                        probes, POT, 3.0), store.sweeps


def offset(sw, folder):
    s = int(np.flatnonzero(sw["folder"] == folder)[0])
    return s, int(sw["i0"].iat[s])


def test_a_spin_window_is_its_held_duty_s_last_100_ms(tmp_path):
    p, sw = pair_of(tree(tmp_path))
    W = windows.ladder_windows(p, {"set-1": "s-1.csv"})
    assert len(W) == 2 and list(W["dir"]) == [1, 1]
    s, i0 = offset(sw, "ladder/set-1/cycles/0")
    assert i0 > 0
    w = W.iloc[0]
    # held from row 2100 to 7100 of the sweep: the last 2000 rows
    assert (w["sweep"], w["g0"], w["g1"]) == (s, i0 + 5100, i0 + 7100)
    assert w["run0"] == i0 + 2000
    assert (w["half_ticks"], w["duty_pct"], w["held_rows"]) == (360, 30, 5000)
    assert w["window_ms"] == 100.0 and w["window_valid_share"] == 1.0
    assert w["i_over_counts"] == pytest.approx(100.0, abs=0.1)
    assert w["coast_tail"] == AWAKE and w["zero_i"] == ZERO_I
    assert w["awake_counts"] == AWAKE - ZERO_I
    assert w["zero_s3"] == AWAKE
    assert w["i_crest_ma"] == pytest.approx(97.0)
    assert w["i_awake_ma"] == pytest.approx(90.0)
    assert (w["rail_v"], w["ntc_c"]) == (5.0, 25.0)
    # the truth: the window's ripple counted on the coast's seed
    assert w["turned"] and w["f_seed"] == pytest.approx(
        SPIN_REV_S * 6, rel=0.1)
    assert w["truth"] == pytest.approx(SPIN_REV_S, rel=1e-3)
    assert w["tach_cycles"] >= 85 and w["rate"] == 0.0
    assert w["pot_rev_s"] == pytest.approx(0.0, abs=1e-9)


def test_a_short_spin_takes_half_its_held_rows(tmp_path):
    p, sw = pair_of(tree(tmp_path))
    w = windows.ladder_windows(p, {"set-1": "s-1.csv"}).iloc[1]
    s, i0 = offset(sw, "ladder/set-1/cycles/1")
    assert (w["sweep"], w["g0"], w["g1"]) == (s, i0 + 2600, i0 + 3100)
    assert (w["held_rows"], w["window_ms"]) == (1000, 25.0)
    assert (w["rail_v"], w["ntc_c"]) == (5.2, 26.0)
    # a shaft that never turns: no coast seed, no truth
    assert not w["turned"] and np.isnan(w["truth"])


def test_a_hold_window_is_its_last_400_ms_at_the_last_duty(tmp_path):
    p, sw = pair_of(tree(tmp_path))
    W = windows.hold_windows(p, {"low-1": "s-1.csv"}, {})
    assert len(W) == 1
    w = W.iloc[0]
    s, i0 = offset(sw, "ladder/low-1/cycles/0")
    # the hold runs rows 2300 to 12350, its duty 200 from row 2350
    assert (w["sweep"], w["g0"], w["g1"]) == (s, i0 + 4350, i0 + 12350)
    assert w["run0"] == i0 + 2000
    assert (w["set"], w["dir"], w["half_ticks"]) == ("low-1", -1, 200)
    assert w["window_ms"] == 400.0 and w["valid"]
    assert w["i_over_counts"] == 50.0 and w["zero_s3"] == AWAKE
    assert not w["kept_turning"] and not w["turning"]
    assert w["v_eff"] == pytest.approx(5.4 * 200 / ARR)
    assert np.isnan(w["truth"]) and w["tach_cycles"] == 0


def test_the_still_gate_and_the_modelled_zero_of_a_load_hold(tmp_path):
    pair = tmp_path / PAIR
    write_sweep(pair / "other/x/0", 1.0, [seg(500)])
    pos_still = 2000 + np.arange(2000) % (windows.STILL_COUNTS + 1)
    pos_moved = 2000 + np.arange(2000) % (windows.STILL_COUNTS + 2)
    stepped = np.r_[np.full(1000, 100), np.full(1000, 120)]
    write_sweep(pair / "load/ladder-1/cycles/0", 10.0, [
        seg(2000), seg(2000, 1, 3000, 100, 1300, pos_still),
        seg(2000, -1, -3000, 100, 1300, pos_moved),
        seg(900, 1, 3000, 100, 1300),       # too short
        seg(2000, 1, 3000, stepped, 1300)])  # two duties
    write_table(pair / "load/ladder-1", "ladder.csv", ["cycles/0"])
    store = sweeps.row_store(pair)
    where = f"telemetry/{PAIR}/load/ladder-1/cycles/0"
    probes = pd.DataFrame({"manifest": ["s-1"], "t": [10.0],
                           "where": [where], "rail_v": [6.0],
                           "ntc_c": [25.0]})
    p = windows.Pair(pair, store.sweeps, BOARD, CHAIN, MOTOR, SERVO, probes,
                     POT, 3.0)
    lad = {"zero_awake_step_per_v_counts": {"value": 1.5},
           "zero_awake_step_intercept_counts": {"value": -2.0}}
    W = windows.load_hold_windows(p, {"ladder-1": "s-1.csv"}, lad)
    s, i0 = offset(store.sweeps, "load/ladder-1/cycles/0")
    assert list(W["seg"]) == [1, 2]
    assert list(W["pos_range"]) == [8.0, 9.0]
    assert list(W["still"]) == [True, False]
    assert list(W["g0"]) == [i0 + 2000, i0 + 4000]
    assert list(W["g1"]) == [i0 + 4000, i0 + 6000]
    assert (W["sweep"] == s).all()
    assert list(W["zero_s3"]) == [ZERO_I + 1.5 * 6.0 - 2.0] * 2
    assert list(W["duty_pct"]) == pytest.approx([100 * 100 / ARR] * 2)


def test_pot_deg_reads_the_table_s_rising_counts_nan_outside():
    got = windows.pot_deg([-1.0, 0.0, 500.0, 1000.0, 2500.0, 4001.0], POT)
    assert np.isnan(got[0]) and np.isnan(got[-1])
    assert list(got[1:-1]) == pytest.approx([-100.0, -75.0, -50.0, 25.0])


def crossings_of(k0, w0, dw, tau, per_rev, j0, t_end_s=0.25):
    """Rows where the phase of a first-order step crosses j0 + n."""
    t = np.arange(-0.03, t_end_s, 1e-6)
    tt = np.maximum(t, 0.0)
    phase = per_rev * (w0 * t + dw * (tt - tau * (1 - np.exp(-tt / tau))))
    n = np.arange(np.ceil(phase[0] - j0), np.floor(phase[-1] - j0) + 1)
    return k0 + np.interp(n + j0, phase, t) * FS


@pytest.mark.parametrize("dw, tau", [(20.0, 0.015), (-12.0, 0.04)])
def test_the_step_fit_recovers_a_first_order_step(dw, tau):
    k0, w0, j0 = 600, 100.0, 0.3
    tc = crossings_of(k0, w0, dw, tau, 6, j0)
    got_j0, dw_fit, tau_fit, res = windows.fit_step(tc, k0, w0, 6, float(FS))
    # the fit numbers the crossings from the first one it is given
    first = j0 + np.ceil(6 * w0 * -0.03 - j0)
    assert got_j0 == pytest.approx(-first, abs=1e-3)
    assert dw_fit == pytest.approx(dw, rel=1e-3)
    assert tau_fit == pytest.approx(tau, rel=1e-3)
    assert res < 1e-3


def test_steps_kept_needs_the_fit_a_sane_tau_and_the_step_s_sign():
    T = pd.DataFrame({"ok": [True, True, True, False, True],
                      "tau_ms": [20.0, 4.0, 20.0, 20.0, 61.0],
                      "dw": [5.0, 5.0, -5.0, 5.0, 5.0],
                      "sign": [1, 1, 1, 1, 1]})
    assert list(windows.steps_kept(T)) == [True, False, False, False, False]
