"""The stage 1 conversions read as one object: the lines inverted with their
offsets in place, the edge lags, and the numbers the notebooks' own
carried-in cells compute from the same result files."""
import numpy as np
import pytest

from oscnb import boards, results, scales

BOARD = boards.BOARDS[boards.DEFAULT]
CPA = BOARD.counts_per_a * 1.024           # a measured scale off the nominal
DRIVE_OFF = 5.25
CPV = 1 / BOARD.v_term_per_count * 1.0005
TERM_OFF = 2.375
RAIL_CPV = 1 / BOARD.v_rail_per_count * 0.997
RAIL_OFF = 5.875
LAGS = {"7v4": (47.0, 42.0, 15.0, 75.0), "3v7": (41.0, 37.0, 13.0, 69.0)}


def write_stage1(root, monkeypatch, timing=True, bridge=True, scale=None):
    """The four stage 1 result files of BOARD under a scratch repo root."""
    monkeypatch.setattr(results, "ROOT", root)
    cpa, off, cpv, term, rcpv, roff = scale or (
        CPA, DRIVE_OFF, CPV, TERM_OFF, RAIL_CPV, RAIL_OFF)
    q = results.q
    capture = root / "telemetry" / "x"
    results.write("1-board", "current-scale", capture, pair=BOARD.key,
                  counts_per_a=q(cpa, cpa - 1, cpa + 1, "counts/A"),
                  drive_offset_counts=q(off, unit="counts"),
                  amps_range=q(1.3, 0.23, 1.3, "A"))
    results.write("1-board", "terminal-chain", capture, pair=BOARD.key,
                  terminal_counts_per_v=q(cpv, cpv - 0.2, cpv + 0.2,
                                          "counts/V"),
                  terminal_offset_counts=q(term, unit="counts"),
                  rail_counts_per_v=q(rcpv, unit="counts/V"),
                  rail_offset_counts=q(roff, unit="counts"))
    if timing:
        lags = {}
        for rail, (on, tap_on, off_, settle) in LAGS.items():
            lags[f"on_edge_lag_{rail}"] = q(on, unit="ticks")
            lags[f"tap_on_edge_lag_{rail}"] = q(tap_on, unit="ticks")
            lags[f"off_edge_lag_{rail}"] = q(off_, unit="ticks")
            lags[f"shunt_settle_{rail}"] = q(settle, unit="ticks")
        results.write("1-board", "sample-timing", capture, pair=BOARD.key,
                      **lags)
    if bridge:
        fast = {"on": q(8.0, unit="ticks"), "off": q(12.0, unit="ticks")}
        results.write("1-board", "bridge-states", capture, pair=BOARD.key,
                      fast_edge_lags_ticks=fast)


def test_amps_and_volts_invert_the_lines_with_offsets_in_place(
        tmp_path, monkeypatch):
    write_stage1(tmp_path, monkeypatch)
    s = scales.load(BOARD)
    zero = 1206.0
    amps = np.linspace(-1.5, 1.5, 31)
    counts = zero + DRIVE_OFF + CPA * amps
    assert np.allclose(s.amps(counts, zero), amps, rtol=0, atol=1e-12)
    rest = zero + CPA * amps
    assert np.allclose(s.amps(rest, zero, drive=False), amps, atol=1e-12)
    volts = np.linspace(-8.0, 8.0, 33)
    diff = TERM_OFF + CPV * volts
    assert np.allclose(s.volts(diff, TERM_OFF), volts, atol=1e-12)
    coast_zero = -3.0
    assert np.allclose(s.volts(coast_zero + CPV * volts, coast_zero), volts,
                       atol=1e-12)
    rail = np.linspace(3.3, 8.4, 11)
    assert np.allclose(s.rail_v(RAIL_OFF + RAIL_CPV * rail), rail,
                       atol=1e-12)
    assert s.a_per_count == 1 / CPA and s.v_per_count == 1 / CPV


def test_nominal_lines_invert_the_board_forward_conversions(
        tmp_path, monkeypatch):
    nominal = (BOARD.counts_per_a, 0.0, 1 / BOARD.v_term_per_count, 0.0,
               1 / BOARD.v_rail_per_count, 0.0)
    write_stage1(tmp_path, monkeypatch, scale=nominal)
    s = scales.load(BOARD.key)
    raw = np.arange(1100.0, 2600.0, 37.0)
    zero = 1207.0
    assert np.allclose(s.amps(raw, zero, drive=False), BOARD.amps(raw, zero),
                       rtol=1e-12, atol=0)
    tap_a, tap_b = raw, np.full_like(raw, 773.0)
    assert np.allclose(s.volts(tap_a - tap_b, 0.0),
                       BOARD.diff_v(tap_a, tap_b), rtol=1e-12, atol=0)
    assert np.allclose(s.rail_v(raw), BOARD.rail_v(raw), rtol=1e-12, atol=0)


def test_edge_lags_are_the_mean_of_the_graded_rails(tmp_path, monkeypatch):
    write_stage1(tmp_path, monkeypatch)
    s = scales.load(BOARD)
    assert s.edge_lags() == {"on": 44.0, "tap_on": 39.5, "off": 14.0,
                             "settle": 72.0}
    assert s.edge_lags(rails=("3v7",))["settle"] == 69.0
    assert s.fast_edge_lags() == {"on": 8.0, "off": 12.0}


def test_later_results_are_optional(tmp_path, monkeypatch, capsys):
    write_stage1(tmp_path, monkeypatch, timing=False, bridge=False)
    s = scales.load(BOARD)
    assert s.sample_timing is None and s.bridge_states is None
    with pytest.raises(ValueError):
        s.edge_lags()
    with pytest.raises(ValueError):
        s.fast_edge_lags()
    s.describe()
    out = capsys.readouterr().out
    assert len(out.splitlines()) == 3
    assert f"{DRIVE_OFF:+.1f} + {CPA:.1f} x amps" in out
    assert f"{TERM_OFF:+.1f} + {CPV:.2f} x volts" in out


def test_describe_names_the_edges(tmp_path, monkeypatch, capsys):
    write_stage1(tmp_path, monkeypatch)
    scales.load(BOARD).describe()
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 5
    assert "ON edge 40 ticks" in lines[3] and "settled 72" in lines[3]
    assert "fast decay: ON edge 8" in lines[4]


def test_equals_the_carried_in_cells_on_the_committed_results():
    """The expressions of the carried-in cells of bridge-states [1],
    inductance [1], bemf-coast [10], no-load-current [6], speed-torque [5],
    rest-resistance [3] and teardown [3], on the committed board D files."""
    s = scales.load(BOARD)
    CS = results.read("1-board", "current-scale", pair=BOARD.key)
    TC = results.read("1-board", "terminal-chain", pair=BOARD.key)
    ST = results.read("1-board", "sample-timing", pair=BOARD.key)
    BS = results.read("1-board", "bridge-states", pair=BOARD.key)
    # bridge-states, inductance, sample-timing [13]
    COUNTS_PER_A = CS["counts_per_a"]["value"]
    DRIVE_OFF = CS["drive_offset_counts"]["value"]
    COUNTS_PER_V = TC["terminal_counts_per_v"]["value"]
    TERM_OFF = TC["terminal_offset_counts"]["value"]
    assert (s.counts_per_a, s.drive_offset_counts) == (COUNTS_PER_A,
                                                       DRIVE_OFF)
    assert (s.terminal_counts_per_v, s.terminal_offset_counts) == (
        COUNTS_PER_V, TERM_OFF)
    lags = {
        "on": np.mean([ST["on_edge_lag_7v4"]["value"],
                       ST["on_edge_lag_3v7"]["value"]]),
        "tap_on": np.mean([ST["tap_on_edge_lag_7v4"]["value"],
                           ST["tap_on_edge_lag_3v7"]["value"]]),
        "off": np.mean([ST["off_edge_lag_7v4"]["value"],
                        ST["off_edge_lag_3v7"]["value"]]),
        "settle": np.mean([ST["shunt_settle_7v4"]["value"],
                           ST["shunt_settle_3v7"]["value"]]),
    }
    assert s.edge_lags() == lags
    # bemf-coast, no-load-current, speed-torque
    assert s.v_per_count == 1 / TC["terminal_counts_per_v"]["value"]
    assert s.a_per_count == 1 / CS["counts_per_a"]["value"]

    def rail_v(n):
        return ((np.asarray(n, float) - TC["rail_offset_counts"]["value"])
                / TC["rail_counts_per_v"]["value"])

    probe = np.array([0.0, 917.0, 1234.5, 1980.0, 4095.0])
    assert np.array_equal(s.rail_v(probe), rail_v(probe))
    # bemf-coast [50], inductance [31], teardown [37]
    assert s.fast_edge_lags()["on"] == BS["fast_edge_lags_ticks"]["on"][
        "value"]
    # the inversions, written as bridge-states [11] and rest-resistance [9]
    raw = np.arange(1150.0, 2500.0, 3.5)
    zero = 1206.0
    assert np.array_equal(s.amps(raw, zero),
                          (raw - zero - DRIVE_OFF) / COUNTS_PER_A)
    diff = np.arange(-1800.0, 1800.0, 7.25)
    assert np.array_equal(s.volts(diff, TERM_OFF),
                          (diff - TERM_OFF) / COUNTS_PER_V)
