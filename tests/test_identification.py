import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "stages" / "2-motor" / "identification"))
import identification as idn  # noqa: E402


def test_travel_is_the_horn_travel_times_the_counted_ratio():
    assert idn.GEAR == pytest.approx(23104 / 75)
    assert idn.TRAVEL_REVS == pytest.approx(184 / 360 * 23104 / 75)
    assert 157 < idn.TRAVEL_REVS < 158


def test_combos_are_every_k_seat_mean_on_the_all_seat_mean():
    x = np.array([4.0, 4.4, 4.6, 5.0])
    one = idn.combos(x, 1)
    assert one == pytest.approx((x / x.mean() - 1) * 100)
    two = idn.combos(x, 2)
    assert len(two) == math.comb(4, 2)
    assert two.mean() == pytest.approx(0.0, abs=1e-12)        # every seat enters equally often
    assert idn.combos(x, 4) == pytest.approx([0.0])


def test_lap_line_reads_a_linear_v_over_i_back_to_the_start():
    t = np.arange(0, 0.3, 5e-5)
    amps = np.full_like(t, 0.8)
    r0, slope = 4.4, 0.05                                     # ohm, ohm per s
    p = dict(t=t, amps=amps, volts=amps * (r0 + slope * t))
    for t_end in (0.1, 0.2, 0.3):
        assert idn.lap_line(p, t_end) == pytest.approx(r0, rel=1e-9)
    # a plain mean over the same stretch reads the mid-point of the line instead
    assert idn.lap_mean(p, 0.1) == pytest.approx(r0 + slope * (0.002 + 0.1) / 2, rel=1e-3)


def test_f_from_is_the_per_duty_mean_held_flat_below_and_zero_at_full_duty():
    T = pd.DataFrame(dict(duty=[15, 15, 30, 30], delta_anchor=[20.0, 22.0, 30.0, 34.0], v_on_pre=[5.0, 5.0, 8.0, 8.0]))
    (d, v), f = idn.f_from(T)
    assert f.loc[15] == pytest.approx(4.2) and f.loc[30] == pytest.approx(4.0)
    assert list(d) == [0.0, 15.0, 30.0, 100.0]
    assert list(v) == pytest.approx([4.2, 4.2, 4.0, 0.0])


def test_published_table_is_the_rail_mean_of_the_grid_per_volt():
    d, v = idn.f_table_published()
    grid = idn.MM["crest_less_mean_ma"]
    for du in idn.GRID_DUTIES:
        want = np.mean([grid[f"{r:g} V"][f"{du} %"]["value"] / r for r in idn.GRID_RAILS])
        assert np.interp(du, d, v) == pytest.approx(want)
    assert v[0] == v[1] and d[-1] == 100.0 and v[-1] == 0.0
    assert idn.delta_ma(40, 7.4, (d, v)) == pytest.approx(np.interp(40, d, v) * 7.4)


def test_anchor_inverts_the_cut_c():
    # at a cut the 100 % current is c Ke (alpha_drive + alpha_coast), so c from it and the anchor from c close the loop
    c_true = 1.02 * idn.J / idn.KE ** 2
    a_coast = np.array([300.0, 320.0, 310.0])                  # rad/s^2
    slope = np.array([40.0, 35.0, 45.0])                       # rev/s per s, the drive side
    drag = a_coast * idn.J / idn.KE * 1e3                      # the drag columns carry J alpha / Ke in mA
    ST = pd.DataFrame(dict(drag_win_ma=drag, drag_pin_ma=drag, drag_slope_ma=drag, slope_100=slope))
    ST["i100"] = c_true * idn.KE * (2 * np.pi * slope + a_coast) * 1e3
    c = idn.c_per_cut(ST)
    for x in idn.ANCHOR_READS:
        assert c[x] == pytest.approx(np.full(3, c_true))


def test_fit_vb_table_is_zero_low_never_negative_and_extends_the_last_segment():
    rm = pd.DataFrame(dict(mech=[1] * 4, rev_s=[100.0, 250.0, 350.0, 450.0], x=[-5.0, 30.0, 150.0, 300.0]))
    fit = idn.fit_vb(rm)[1]
    tab = fit["table"]
    k = idn.VB_KNOTS
    assert np.all(tab[k <= 150] == 0.0) and np.all(tab >= 0.0)
    assert tab[k == 350][0] == pytest.approx(150.0)
    assert tab[k == 300][0] == pytest.approx(90.0)             # interpolated between 250 and 350 rev/s
    assert tab[k == 550][0] == pytest.approx(300.0 + 1.5 * 100)  # the last segment's 1.5 mV per rev/s, carried on
    # the table read back: interpolated inside, held at the last knot past it
    assert idn.vb_eval({1: fit, -1: fit}, 1, 325.0) == pytest.approx(120.0)
    assert idn.vb_eval({1: fit, -1: fit}, 1, 900.0) == pytest.approx(tab[-1])


def test_vb_eval_power_law():
    fit = {m: dict(power=(45.0, 3.5, np.nan), table=None) for m in (1, -1)}
    assert idn.vb_eval(fit, np.array([1, -1]), np.array([300.0, 600.0]), "power") == pytest.approx([45.0, 45.0 * 2 ** 3.5])


def test_signals_at_full_duty_drive_the_whole_period_and_read_no_delta():
    T = pd.DataFrame(dict(v_on_200=[4.8], i_crest_200=[120.0], rail_v=[5.0], duty_applied_pct=[100.0], half_ticks=[idn.mm.ARR]))
    S = idn.signals(T, idn.f_table_published())
    assert S["share"].iloc[0] == 1.0 and S["delta_ma"].iloc[0] == 0.0
    assert S["v_mean_d"].iloc[0] == pytest.approx(4.8) and S["i_hat_d"].iloc[0] == pytest.approx(120.0)


def test_spins_for_a_change_between_two_checks():
    # n per check so that sqrt(2 sd^2 / n) <= 0.5 %
    assert idn.spins_for_change(1.42) == 17
    assert idn.spins_for_change(1.60) == 21
    for sd in (0.7, 1.42, 2.39):
        n = idn.spins_for_change(sd)
        assert math.sqrt(2 * sd ** 2 / n) <= 0.5 < math.sqrt(2 * sd ** 2 / (n - 1))


def test_block_rise_holds_every_spin_and_decays_the_fast_parts():
    e, c_cu, c_body, tau = 0.05, 0.14, 0.54, 2.0
    assert idn.block_rise_c(e, 1, 5.0, c_cu, c_body, tau) == pytest.approx(e / c_body + e / c_cu)
    many = idn.block_rise_c(e, 200, 5.0, c_cu, c_body, tau)
    assert many == pytest.approx(200 * e / c_body + (e / c_cu) / (1 - math.exp(-5.0 / tau)))
    assert idn.held_rise_c(0.54, c_body) == pytest.approx(1.0)
