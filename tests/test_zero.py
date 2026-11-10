"""The current zero's tracker on synthetic rows: a step's closing, the
feed rule, the stale rule, the integer port's floor, the plain form
against the core, the start rule and the drive-feed constant."""
import numpy as np
import pandas as pd
import pytest

from oscnb import boards, zero
from oscnb.sweeps import ASLEEP, COAST, DRIVE, RowStore

FS = 20_000
BOARD = boards.BOARDS["dev-v006-2A-D"]
GATE = zero.gate_ticks(zero.GATE_REC_PCT, BOARD.pwm_arr)
SETTLE = zero.ZERO_SETTLE_TICKS


def store_of(bridge, duty, trough, raw=None, rail=None):
    """One sweep of rows, no gap before it and none inside."""
    n = len(bridge)
    raw = trough if raw is None else raw
    rail = np.full(n, np.nan) if rail is None else rail
    arrays = dict(bridge=np.asarray(bridge, np.int8),
                  duty=np.asarray(duty, np.int16),
                  trough=np.asarray(trough, np.int16),
                  raw=np.asarray(raw, np.int16),
                  rail=np.asarray(rail, np.float32),
                  pre_gap=np.zeros(n, np.int32))
    sw = pd.DataFrame(dict(i0=[0], i1=[n], base_n=[n], gap_s=[np.nan]))
    return RowStore(arrays, sw, pd.DataFrame(), BOARD, FS, BOARD.pwm_arr)


def rows(*parts):
    """(bridge, duty, trough) from (state, duty, trough, n) parts."""
    b = np.concatenate([np.full(n, s) for s, _, _, n in parts])
    d = np.concatenate([np.full(n, x) for _, x, _, n in parts])
    t = np.concatenate([np.full(n, x) for _, _, x, n in parts])
    return b, d, t


@pytest.mark.parametrize("shift", [10, 13])
def test_a_step_in_the_fed_sample_closes_as_exp_minus_t_over_tau(shift):
    tau = 1 << shift
    b, d, t = rows((COAST, 0, 0, SETTLE), (COAST, 0, 100, 3 * tau))
    out, kind, _ = zero.replay(zero.Rule(shift=shift), store_of(b, d, t))
    assert (kind[:SETTLE] == zero.K_NONE).all()
    for k in (1, 2):
        closed = out[SETTLE + k * tau - 1] / 100
        assert abs(closed - (1 - np.exp(-k))) < 1e-3
    assert abs(out[SETTLE + tau - 1] / 100 - 0.632) < 1e-3


def test_a_drive_wider_than_the_gate_feeds_nothing():
    b, d, t = rows((DRIVE, GATE + 1, 1300, 5000))
    out, kind, _ = zero.replay(zero.Rule(), store_of(b, d, t))
    assert (kind == zero.K_NONE).all() and (out == 1300).all()
    b, d, t = rows((ASLEEP, 0, 1200, SETTLE), (DRIVE, GATE, 1300, 5000))
    out, kind, _ = zero.replay(zero.Rule(), store_of(b, d, t))
    assert (kind[SETTLE:] == zero.K_AWAKE).all() and out[-1] > 1240


def test_a_coast_feeds_only_after_the_settle_ticks():
    b, d, t = rows((DRIVE, GATE + 1, 1200, 100), (COAST, 0, 1210, 500))
    _, kind, _ = zero.replay(zero.Rule(), store_of(b, d, t))
    assert (kind[100:100 + SETTLE] == zero.K_NONE).all()
    assert (kind[100 + SETTLE:] == zero.K_AWAKE).all()


def test_torque_off_feeds_the_asleep_zero_only():
    b, d, t = rows((COAST, 0, 1200, 2000), (ASLEEP, 0, 1100, 20000))
    out, kind, _, asleep = zero.replay(zero.Rule(), store_of(b, d, t),
                                       asleep=True)
    assert (kind[2000 + SETTLE:] == zero.K_ASLEEP).all()
    assert (out[2000:] == out[1999]).all()
    assert asleep[-1] < asleep[1999] - 80


def test_a_zero_unfed_for_the_stale_ticks_follows_the_other():
    left = zero.STALE_TICKS - 1000
    b, d, t = rows((COAST, 0, 1200, 2000), (ASLEEP, 0, 1100, SETTLE + left),
                   (ASLEEP, 0, 1000, 6000))
    out, _, _, asleep = zero.replay(zero.Rule(), store_of(b, d, t),
                                    asleep=True)
    a0 = 2000 + SETTLE
    # the awake zero's counter falls one per tick it is not fed, the
    # settle ticks included, so it holds through this many asleep feeds
    held = a0 + zero.STALE_TICKS - SETTLE - 2
    assert (out[a0:held + 1] == out[a0 - 1]).all()
    assert out[held + 1] < out[held]
    np.testing.assert_allclose(np.diff(out[held:]), np.diff(asleep[held:]),
                               rtol=0, atol=1e-12)
    assert out[-1] < out[held] - 10


def test_float_sits_the_ports_floor_above_the_integer_state():
    rng = np.random.default_rng(3)
    n = 300_000
    kind = rng.choice([zero.K_NONE, zero.K_ASLEEP, zero.K_AWAKE], n,
                      p=[0.2, 0.1, 0.7]).astype(np.int8)
    x = np.round(1200 + 40 * np.sin(np.arange(n) / 9000)
                 + rng.normal(0, 3, n)).astype(np.int64)
    tr = zero.Tracker()
    tr.seed(1200)
    qa, _ = tr.run(kind, x)
    zf, _ = zero.track(kind, x.astype(float), 1200.0, 1200.0)
    diff = (zf - zero.q_to_float(qa))[50_000:]
    assert abs(diff.mean() - 0.0625) < 0.005
    assert np.abs(diff).max() < 0.11


def mixed():
    return rows((ASLEEP, 0, 1205, 600), (DRIVE, 200, 1215, 900),
                (COAST, 0, 1212, 700), (DRIVE, GATE + 50, 1250, 400),
                (DRIVE, 300, 1214, 500), (ASLEEP, 0, 1203, 900))


def test_track_equals_the_numba_core():
    b, d, t = mixed()
    st = store_of(b, d, t)
    out, kind, _ = zero.replay(zero.Rule(), st)
    ref, _ = zero.track(kind, t.astype(float), 1205.0, 1205.0)
    np.testing.assert_array_equal(ref, out)


def test_the_integer_core_is_the_port_with_stage_1s_feed_rule():
    b, d, t = mixed()
    st = store_of(b, d, t)
    q, kind = zero.replay_int(zero.IntRule(), st)
    np.testing.assert_array_equal(
        kind, zero.kinds_from_bridge(b, d.astype(np.int64), GATE))
    tr = zero.Tracker()
    tr.seed(1205)
    qa, _ = tr.run(kind, t)
    np.testing.assert_array_equal(q, qa)


def test_the_start_rule_seeds_at_boot_and_on_a_rail_move():
    a, slope, thr = 2.0, 0.5, 0.5
    b, d, t = rows((ASLEEP, 0, 1000, 500), (DRIVE, GATE + 1, 1000, 1000))
    rail = np.full(len(b), np.nan)
    rail[500:600] = 5.0
    rail[600:700] = 5.3          # under the threshold
    rail[700:800] = 6.0          # past it
    rail[800:] = 6.0
    rule = zero.Rule(step_a=a, step_b=slope, seed_boot=True,
                     seed_thr_v=thr)
    out, _, n_seed, asleep = zero.replay(rule, store_of(b, d, t, rail=rail),
                                         asleep=True)
    assert out[499] == 1000.0
    assert out[500] == asleep[500] + zero.seed_awake(0.0, 5.0, a, slope)
    assert (out[500:700 + rule.debounce - 1] == out[500]).all()
    i = 700 + rule.debounce - 1
    assert out[i] == asleep[i] + a + slope * 6.0
    assert n_seed[0] == 2
    # past the threshold for fewer rail rows than the debounce
    rail[800:] = 5.0
    rule = zero.Rule(step_a=a, step_b=slope, seed_boot=True,
                     seed_thr_v=thr, debounce=200)
    _, _, n_seed = zero.replay(rule, store_of(b, d, t, rail=rail))
    assert n_seed[0] == 1
    rail[700:] = 5.3
    rule = zero.Rule(step_a=a, step_b=slope, seed_boot=True,
                     seed_thr_v=thr)
    _, _, n_seed = zero.replay(rule, store_of(b, d, t, rail=rail))
    assert n_seed[0] == 1


def test_seed_threshold_is_the_excess_line_spread_over_the_slope():
    v = zero.seed_threshold_v(1.667, 0.733, 0.505)
    assert v == pytest.approx(np.sqrt(0.733**2 - 0.505**2) / 1.667)
    assert zero.seed_threshold_v(1.667, 0.4, 0.5) == 0.0


def test_the_drive_feed_constant_leaves_the_zero_at_the_crest_column():
    n = 12 << zero.ALPHA_SHIFT
    b, d, t = rows((ASLEEP, 0, 1190, 64), (DRIVE, 200, 1203, n))
    raw = np.where(b == DRIVE, 1200, 1190)
    st = store_of(b, d, t, raw=raw)
    out, _, _ = zero.replay(zero.Rule(), st)
    assert abs(out[-1] - 1203) < 1e-3
    out, _, _ = zero.replay(zero.Rule(drive_off=3.0), st)
    assert abs(out[-1] - 1200) < 1e-3
