"""A spin and its coast: the pairs and the cut on the synthetic sweep, and
the open rows, the bridge, the stop and the deceleration on a synthetic
coast EMF."""
import coast_synth as synth
import numpy as np
import pandas as pd
import pytest
import sweep_synth

from oscnb import coast, ripple, tel

TAIL_ROWS = 12
COAST_ROWS = 400


@pytest.fixture
def sweep(tmp_path):
    folder = sweep_synth.write(tmp_path / "sweep", rung_rows=3000,
                               lead_rows=30, coast_rows=COAST_ROWS,
                               tail_rows=TAIL_ROWS)
    df, _ = tel.load(folder)
    return df


def test_pairs_chain_each_coast_to_the_spin_before_it(sweep):
    found = coast.pairs(sweep)
    assert [d for d, _, _ in found] == [1, -1]
    for (d, spin, co), seg in zip(found, (1, 3)):
        assert spin.equals(sweep[sweep["seg"] == seg])
        assert co.equals(sweep[sweep["seg"] == seg + 1])
        assert (co["cmd_duty_q15"] == 0).all()
        assert (spin["cmd_duty_q15"] != 0).all()


def test_cut_row_is_the_first_row_with_no_drive(sweep):
    for _, _, co in coast.pairs(sweep):
        assert coast.cut_row(co) == TAIL_ROWS
    driven = sweep[sweep["seg"] == 1].iloc[30:]
    assert coast.cut_row(driven) is None


def test_emf_from_the_cut_on_the_synthetic_sweep(sweep):
    _, zero_e = tel.rest_zeros(sweep)
    cpv = 248.0
    for d, _, co in coast.pairs(sweep):
        z = coast.cut_row(co)
        e = coast.emf_from_cut(co, d, z, zero_e, v_per_count=1 / cpv)
        assert len(e) == COAST_ROWS - TAIL_ROWS
        assert coast.early_emf(e) > coast.TURN_V
        assert e[-1] < coast.TURN_V
        assert e[:20].mean() > e[-20:].mean() + 1.0
        by_division = coast.emf_from_cut(co, d, z, zero_e, counts_per_v=cpv)
        assert by_division == pytest.approx(e, rel=1e-15)
    with pytest.raises(ValueError):
        coast.emf_from_cut(co, d, z, zero_e)
    with pytest.raises(ValueError):
        coast.emf_from_cut(co, d, z, zero_e, v_per_count=1.0, counts_per_v=1)


def test_open_rows_flag_exactly_the_dropouts():
    bad = coast.open_rows(synth.emf())
    assert np.flatnonzero(bad).tolist() == list(synth.DROPOUTS)
    assert not coast.open_rows(synth.emf(dropouts=())).any()
    early = synth.emf(dropouts=(5, 300))
    assert np.flatnonzero(coast.open_rows(early)).tolist() == [300]


def test_bridge_restores_the_line():
    line = synth.line()
    dropped = line.copy()
    dropped[list(synth.DROPOUTS)] = 0.0
    bad = coast.open_rows(dropped)
    restored = coast.bridge(dropped, bad)
    assert restored == pytest.approx(line, abs=1e-12)
    assert np.array_equal(restored[~bad], dropped[~bad])
    untouched = coast.bridge(line, np.zeros(len(line), bool))
    assert untouched is not line and np.array_equal(untouched, line)


def test_stop_row_is_where_the_running_mean_falls_under_turn_v():
    e = coast.bridge(synth.emf(), coast.open_rows(synth.emf()))
    under = np.flatnonzero(synth.line() < coast.TURN_V)[0]
    assert abs(coast.stop_row(e) - under) <= 1
    assert coast.stop_row(np.ones(500)) is None


def test_seed_freq_reads_the_early_coast_line():
    e = synth.emf()
    t_mid = (coast.COAST_FROM_ROWS + coast.SEED_ROWS / 2) / synth.FS
    bin_hz = synth.FS / (ripple.ZERO_PAD * coast.SEED_ROWS)
    assert abs(coast.seed_freq(e) - synth.ripple_hz(t_mid)) <= bin_hz


KE = synth.EMF_V / (2 * np.pi * synth.REV_S)   # V s/rad, the coast's own


def test_coast_table_reads_the_line_s_level_slope_and_e0():
    e = synth.emf()
    tab, reads = coast.coast_table(e, KE, synth.PER_REV)
    assert reads["e0"] == pytest.approx(synth.EMF_V, rel=5e-3)
    assert reads["w_cut_emf"] == pytest.approx(synth.REV_S, rel=5e-3)
    assert reads["open_rows_coast"] == len(synth.DROPOUTS)
    assert np.flatnonzero(reads["bad"]).tolist() == list(synth.DROPOUTS)
    assert reads["emf"] == pytest.approx(synth.line(), abs=0.1)
    under = np.flatnonzero(synth.line() < coast.TURN_V)[0]
    assert abs(reads["stop_ms"] - under / synth.FS * 1e3) <= 0.1
    t_mid = (coast.COAST_FROM_ROWS + coast.SEED_ROWS / 2) / synth.FS
    bin_rev_s = synth.FS / (ripple.ZERO_PAD * coast.SEED_ROWS) / 6
    assert abs(reads["coast_line_rev_s"]
               - synth.ripple_hz(t_mid) / synth.PER_REV) <= bin_rev_s
    first = (coast.COAST_FROM_ROWS + coast.DECEL_HALF_S * synth.FS) / synth.FS
    assert tab["t_coast"].iloc[0] == pytest.approx(first)
    assert np.diff(tab["t_coast"]) == pytest.approx(coast.DECEL_STEP_S)
    assert tab["t_coast"].iloc[-1] * 1e3 < reads["stop_ms"]
    level = synth.line()[np.round(tab["t_coast"] * synth.FS).astype(int)]
    assert tab["emf"].to_numpy() == pytest.approx(level, abs=5e-3)
    slope = -synth.EMF_V / synth.STOP_S
    assert tab["dedt"].mean() == pytest.approx(slope, rel=0.01)
    assert tab["dedt"].to_numpy() == pytest.approx(slope, rel=0.1)
    assert tab["alpha"].to_numpy() == pytest.approx(tab["dedt"] / KE)
    assert tab["w"].to_numpy() == pytest.approx(tab["emf"] / KE)
    assert tab["open_rows"].sum() > 0


def test_coast_decel_fn_holds_the_bin_medians_and_the_ends():
    w = np.linspace(0.0, 3000.0, 601)
    table = pd.DataFrame({"w": w[::-1], "alpha": -100.0 - 0.1 * w[::-1]})
    fn, top = coast.coast_decel_fn(table)
    assert top == pytest.approx(3000.0, abs=coast.DECEL_BIN)
    mid = np.linspace(20.0, 2980.0, 50)
    assert fn(mid) == pytest.approx(-100.0 - 0.1 * mid, abs=0.1)
    assert fn(-50.0) == fn(0.0) and fn(1e5) == fn(top)
