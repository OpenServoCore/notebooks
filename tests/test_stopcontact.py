"""First contact at an end stop by the ripple: two recorded sittings of one
low stop, one with the pot clean at the stop and one with its low end dead,
and synthetic rungs where the contact and the table are known."""
import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

from oscnb import stopcontact as st

FIXTURES = Path(__file__).parent / "stop_contact"
SOURCES = json.loads((FIXTURES / "sources.json").read_text())
LINE_HZ = 200.0         # the synthetic rung's free ripple line
FREE, SEATED = 1310.0, 1386.0     # raw shunt counts, free and seated
RIPPLE = 6.0            # raw counts, the ripple's amplitude


def per_cycle(name, key="counts_per_cycle"):
    """The counts a cycle its run's table gives a recorded rung."""
    return SOURCES[f"{name}.csv.gz"][key]["value"]


def rung(name):
    """(pot, shunt) of a recorded rung's driven rows."""
    rows = pd.read_csv(FIXTURES / f"{name}.csv.gz")
    a, b = st.driven(rows["duty_counts"].to_numpy())
    return rows["pos"].to_numpy()[a:b], rows["current_raw"].to_numpy()[a:b]


def test_one_low_stop_in_two_sittings_meets_the_ripple_where_the_pot_rests_apart():
    # the pot rests 96 counts apart at one stop, and the ripple puts first contact 0.6 cycles apart
    read = {}
    for name in ("clean-low-stop", "dead-low-stop"):
        pos, current = rung(name)
        c = st.read(current, pos, -1, per_cycle(name))
        assert c.valid and c.rise == pytest.approx(79.6 if name.startswith("clean") else 75.4, abs=0.1)
        read[name] = (st.cycles_past(c, pos, 300), float(np.mean(pos[-200:])), int(pos[c.row]))
    (clean, clean_rest, clean_pot), (dead, dead_rest, dead_pot) = read.values()
    assert [round(clean, 2), round(dead, 2)] == [48.93, 49.53]
    assert abs(clean - dead) <= 1.5 and abs(clean_rest - dead_rest) > 90
    # at the contact row itself the clean pot reads 108 and the dead one 3
    assert (clean_pot, dead_pot) == (108, 3)


def synthetic(rows=13000, contact_row=9000, fs=st.FS):
    """A rung onto a stop: the ripple at LINE_HZ on the free level, then the
    shunt stepping to SEATED with the rotor still at `contact_row`."""
    t = np.arange(rows) / fs
    moving = np.arange(rows) < contact_row
    phase = 2 * np.pi * LINE_HZ * np.minimum(t, contact_row / fs)
    return np.where(moving, FREE + RIPPLE * np.sin(phase), SEATED)


def test_a_rung_reads_its_contact_at_half_the_rise_and_counts_cycles_from_the_spin_up():
    c = st.read(synthetic(), np.arange(13000) * 0.038, +1, 3.8)
    # the centered box crosses half the rise at the step, give or take the ripple inside it
    assert abs(c.row - 9000) <= 5
    assert (c.free, c.seated) == (pytest.approx(FREE, abs=0.5), pytest.approx(SEATED))
    # 200 Hz from the spin-up's end at 2000 rows: 70 cycles at the step
    assert c.cycle == pytest.approx((c.row - 2000) / st.FS * LINE_HZ, abs=0.05)
    assert c.cycle == pytest.approx(70.0, abs=0.1)


def test_a_rung_that_ends_short_of_its_stop_reads_no_contact():
    free = synthetic(contact_row=10**6)
    moving = 470 - np.arange(len(free)) * 0.038
    c = st.read(free, moving, -1, 3.8)
    assert c is not None and not c.valid
    # too few rows to read, and a pot that never moves: nothing to seed the ripple from
    assert st.read(free[:4000], moving[:4000], -1, 3.8) is None
    assert st.read(free, np.full(len(free), 470), -1, 3.8) is None


def test_a_contact_placed_in_a_tables_frame_is_the_rungs_offset_plus_its_cycles():
    # the pot falls 3.8 counts a cycle from 470, through a table whose angle is a cycle per 3.8 counts plus
    # a bow, and the cut keeps the samples off the last CUT_CYCLES
    current = synthetic()
    c = st.read(current, 470 - np.arange(len(current)) * 0.038, -1, 3.8)
    cycle = c.cycle_at(np.arange(len(current)))
    pos = np.round(470 - 3.8 * cycle + 4 * np.sin(cycle / 20)).astype(int)
    angle = lambda x: (np.asarray(x) - 4 * np.sin((470 - np.asarray(x)) / 3.8 / 20)) / 3.8
    p = st.placed(c, pos, angle)
    assert p.angle == pytest.approx((470 - 3.8 * c.cycle) / 3.8, abs=0.1)
    assert p.residual_sd < 0.2
    # a pot that leaves the shaft past the cut moves nothing
    dead = pos.copy()
    dead[c.row - 2 * round(st.FS / LINE_HZ):] = 4
    assert st.placed(c, dead, angle).angle == p.angle
    # samples outside the table's fitted span are left out
    inside = st.placed(c, pos, angle, span=(200, 400))
    assert inside.samples < p.samples and inside.angle == pytest.approx(p.angle, abs=0.1)


def test_a_fast_rung_is_counted_on_the_line_its_free_stretch_predicts():
    # rerun-2's high rung: the pot runs about 1100 counts/s over the free stretch, so at the table's 4.31
    # counts a cycle the line sits near 259 Hz. A fixed 100..300 Hz band held a 131 Hz line on these rows
    # and counted about half the cycles: 33.0 to contact where 67.0 is
    pos, current = rung("fast-high-stop")
    c = st.read(current, pos, +1, per_cycle("fast-high-stop"))
    assert c.valid and c.f_seed == pytest.approx(259.4, abs=0.5)
    assert round(st.cycles_past(c, pos, 3480), 2) == 67.02
    # through a table at the 4.03 counts a cycle it reads over the samples, the track holds
    table = per_cycle("fast-high-stop", "table_counts_per_cycle")
    p = st.placed(c, pos, lambda x: np.asarray(x) / table)
    assert p.tracked and p.counts_per_cycle == pytest.approx(4.09, abs=0.01)
    # seeded where a wrong counts a cycle puts it, the track runs on the 131 Hz line, reads twice the
    # table's counts a cycle, and is named
    wrong = st.read(current, pos, +1, 2 * per_cycle("fast-high-stop"))
    assert wrong.f_seed == pytest.approx(131.2, abs=0.5)
    off = st.placed(wrong, pos, lambda x: np.asarray(x) / table)
    assert not off.tracked and off.counts_per_cycle / table == pytest.approx(2.0, abs=0.15)


def test_a_stop_is_its_rungs_median_with_the_spread_and_a_far_rung_named():
    s = st.stop([27.31, 27.20, 27.35, 32.0, 27.38])
    assert (s.contact, s.n, s.odd, s.left_out) == (27.35, 5, [3], [])
    assert s.spread == pytest.approx(32.0 - 27.20)
    # a rung whose track failed its check is left out of the median and the spread, and named
    s = st.stop([27.31, 27.20, 27.35, 13.0, 27.38], kept=[True, True, True, False, True])
    assert (s.contact, s.n, s.odd, s.left_out) == (pytest.approx(27.33), 4, [], [3])
    assert s.spread == pytest.approx(27.38 - 27.20)
    with pytest.raises(ValueError, match="no rung"):
        st.stop([])
    with pytest.raises(ValueError, match="no rung"):
        st.stop([27.3], kept=[False])


def test_the_travel_and_the_hold_margin_are_in_the_trains_degrees():
    # an MG90's train: 6 events a motor turn, 308.0533 turns an output turn, 0.1948 deg a cycle
    per = st.deg_per_cycle(6, 308.0533)
    assert round(per, 4) == 0.1948
    assert st.travel_deg(27.31, 965.08, 6, 308.0533) == pytest.approx(937.77 * per)
    # K is 2.5 deg where 1.25 x the cut is under it, else 1.25 x the cut
    assert st.hold_margin_deg(per) == 2.5
    assert st.hold_margin_deg(st.deg_per_cycle(6, 30.0)) == pytest.approx(10.0)
