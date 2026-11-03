"""Burst placement on a synthetic R-L burst: samples on the window grid, the
tap difference, the binned fold and the period table's step."""
import burst_synth as synth
import numpy as np
import pytest

from oscnb import boards, bursts

BOARD = boards.BOARDS[boards.DEFAULT]
ARR = BOARD.pwm_arr
PERIOD = 2 * ARR
R_OHM, L_H, RAIL_V = 5.0, 1e-3, 5.0


@pytest.fixture(scope="module")
def burst():
    return synth.burst(BOARD, r_ohm=R_OHM, l_h=L_H, rail_v=RAIL_V)


def test_duty_pct_rounds_the_q15_step():
    assert bursts.duty_pct({"step_q15": 8192}) == 25
    assert bursts.duty_pct({"step_q15": -13107}) == 40
    assert bursts.duty_pct({"step_q15": 32767}) == 100


@pytest.mark.parametrize("center", ["crest", "trough"])
def test_samples_cover_whole_periods(burst, center):
    df, _, _, _ = burst
    s, m, half = bursts.samples(df, BOARD, center)
    assert half == int(BOARD.drive_ticks(m["step_q15"]))
    assert s["tc"].min() >= -ARR and s["tc"].max() < ARR
    full = s[(s["n"] >= 0) & s["full"]]
    assert full["n"].min() == 0
    assert np.array_equal(np.unique(full["n"]), np.arange(full["n"].max() + 1))
    # every full period spans its whole grid period, a sample every STEP_TICKS
    span = full["t_us"] * bursts.TICKS_PER_US - full["n"] * PERIOD
    first = span.groupby(full["n"]).min()
    last = span.groupby(full["n"]).max()
    assert (first >= 0).all() and (first < bursts.STEP_TICKS).all()
    assert (last >= PERIOD - bursts.STEP_TICKS).all() and (last < PERIOD).all()
    assert not s[s["n"] == s["n"].max()]["full"].any()
    assert set(s["stream"]) == {"sh", "ta", "tb"}


def test_trough_grid_starts_one_period_later_than_its_crest_window(burst):
    df, _, _, _ = burst
    crest, _, half = bursts.samples(df, BOARD, "crest")
    trough, _, _ = bursts.samples(df, BOARD, "trough")
    shift = (crest["t_us"] - trough["t_us"]) * bursts.TICKS_PER_US
    assert np.allclose(shift, ARR)
    with pytest.raises(ValueError):
        bursts.samples(df, BOARD, "middle")


def test_term_diff_reads_the_rail_inside_the_window(burst):
    df, _, _, _ = burst
    s, _, half = bursts.samples(df, BOARD, "crest")
    tc, n, volts = bursts.term_diff(s, 1, synth.scales())
    assert len(tc) == len(n) == len(volts) == (s["stream"] == "ta").sum()
    driven = (n >= 0) & (tc > -half + synth.LAGS["tap_on"] + 1) & (
        tc < half + synth.LAGS["off"] - 1)
    resting = (n >= 0) & (np.abs(tc) > half + synth.LAGS["off"] + 1)
    assert volts[driven] == pytest.approx(RAIL_V, abs=0.5 / synth.COUNTS_PER_V)
    assert volts[resting] == pytest.approx(0.0, abs=0.5 / synth.COUNTS_PER_V)


def test_binned_median_error_and_count():
    x = np.array([0.5, 1.0, 1.5, 4.2, 4.4, 4.6, 4.8, 9.9, -1.0, 12.0])
    y = np.array([1.0, 3.0, 2.0, 10.0, 14.0, 12.0, 16.0, 7.0, 99.0, 99.0])
    centers, med, se, count = bursts.binned(x, y, 0, 10, 4)
    assert centers.tolist() == [2.0, 6.0, 10.0]
    assert count.tolist() == [3, 4, 1]                  # -1 and 12 outside
    assert med.tolist() == [2.0, 13.0, 7.0]
    assert se[0] == pytest.approx(bursts.MEDIAN_SE * 1.0 / np.sqrt(3))
    assert se[1] == pytest.approx(
        bursts.MEDIAN_SE * np.std([10, 14, 12, 16], ddof=1) / 2)
    assert np.isnan(se[2])                              # under three samples
    _, med_empty, _, count_empty = bursts.binned(x, y, 20, 30, 4)
    assert np.isnan(med_empty).all() and not count_empty.any()


def test_period_table_returns_the_step(burst):
    df, current, _, phi0 = burst
    table = bursts.period_table(df, 1, BOARD, synth.scales())
    s, m, half = bursts.samples(df, BOARD, "crest")
    assert len(table) == s[(s["n"] >= 0) & s["full"]]["n"].nunique()
    a, b = -half + synth.LAGS["tap_on"], half + synth.LAGS["off"]
    step_tick = phi0 + bursts.STEP_TICKS * m["step_index"] + (
        bursts.APERTURE_CLK * bursts.TICKS_PER_ADC_CLK)
    crest = PERIOD * np.ceil(step_tick / PERIOD) + ARR
    middle = crest + table["n"].to_numpy() * PERIOD + (a + b) / 2
    true_a = current[np.round(middle).astype(int)]
    count_a = 1 / synth.COUNTS_PER_A
    assert table["i_a"].to_numpy() == pytest.approx(true_a, abs=0.5e-3)
    assert np.all(np.diff(table["i_a"]) > -count_a)
    share = (b - a) / PERIOD
    assert table["i_a"].iloc[-1] == pytest.approx(RAIL_V * share / R_OHM,
                                                  rel=0.01)
    assert table["v_on_v"].to_numpy() == pytest.approx(RAIL_V)
    assert table["v_off_v"].to_numpy() == pytest.approx(0.0, abs=1e-12)
    assert table["v_v"].to_numpy() == pytest.approx(RAIL_V * share)
    # V_n = R i_mean + L di / T over the period pairs, inductance's fit
    i, v = table["i_a"].to_numpy(), table["v_v"].to_numpy()
    t_s = PERIOD / (bursts.TICKS_PER_US * 1e6)
    pairs = np.c_[(i[1:] + i[:-1]) / 2, (i[1:] - i[:-1]) / t_s]
    (r, l), *_ = np.linalg.lstsq(pairs, v[:-1], rcond=None)
    assert r == pytest.approx(R_OHM, rel=0.01)
    assert l == pytest.approx(L_H, rel=0.01)
    edged = bursts.period_table(df, 1, BOARD, synth.scales(), edge=PERIOD)
    assert edged["v_v"].to_numpy() == pytest.approx(RAIL_V * share + 1)
