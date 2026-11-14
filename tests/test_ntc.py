"""The board NTC's conversion as the kernel runs it, held to the
firmware's own function on the same counts, and what a curve reads
against the part's nominal equation."""
import math

import numpy as np
import pytest

from oscnb import boards, ntc

BOARD = boards.BOARDS[boards.DEFAULT]
# core estimator/thermal.rs ntc_cc, as a build of that file printed it: (raw_ref, t_ref_cc, k1_q88, k2_q24) to
# the centi-C it gives each count, None where it gives none
FIRMWARE = {
    # a tangent form about 25 C
    (2048, 2500, 563, 5779): {0: None, 1: 8444, 9: 8416, 500: 6729, 1099: 4897, 1947: 2725, 2047: 2502, 2048: 2500,
                              2049: 2497, 2149: 2280, 3157: 484, 4086: -553, 4094: -559, 4095: None, 5000: None},
    # a quadratic over a wide range
    (2048, 2584, 212, 28463): {0: None, 1: 11387, 9: 11325, 500: 7930, 1099: 4896, 1947: 2684, 2047: 2584,
                               2048: 2584, 2049: 2583, 2149: 2517, 3157: 3751, 4086: 7942, 4094: 7990, 4095: None},
    # a reference past the converter's span: the deviation is held inside it
    (65535, 2500, 563, 5779): {0: None, 1: 17281, 2048: 17281, 4094: 17281, 4095: None},
    # no slope is no curve
    (2048, 2500, 0, 5779): {1: None, 2048: None, 4094: None},
    # a negative slope: the products floor
    (2048, 2500, -563, 0): {1: -2002, 500: -905, 1099: 412, 2047: 2497, 2049: 2502, 2149: 2722, 4094: 6999},
    # a result that leaves the kernel's field reads none
    (2048, 2500, 32767, 65535): {500: None, 1947: 15466, 2047: 2627, 2048: 2500, 2049: 2372, 2149: -10389,
                                 3157: None},
}


@pytest.mark.parametrize("curve", FIRMWARE, ids=["tangent", "quadratic", "far reference", "no slope",
                                                 "negative slope", "out of the field"])
def test_the_conversion_is_the_firmwares_on_the_same_counts(curve):
    assert {raw: ntc.ntc_cc(raw, *curve) for raw in FIRMWARE[curve]} == FIRMWARE[curve]


def test_the_notebooks_quadratic_is_the_kernels_inside_the_span():
    curve = (2048, 2430, 474, 19934)
    raws = np.arange(1, 4095)
    assert list(ntc.quadratic_cc(raws, *curve)) == [ntc.ntc_cc(int(r), *curve) for r in raws]


def test_a_curve_is_read_against_the_parts_nominal_equation():
    exact = (2048, 2500, 563, 5779)
    # the nominal divider reads mid span at 25 C, and the count falls as the board warms
    assert float(ntc.raw_of_c(BOARD, 25.0)) == pytest.approx(BOARD.adc_counts / 2)
    assert float(ntc.raw_of_c(BOARD, 60.0)) < float(ntc.raw_of_c(BOARD, 40.0)) < 2048
    assert float(BOARD.ntc_c_beta(ntc.raw_of_c(BOARD, 73.0))) == pytest.approx(73.0)
    assert ntc.reads_c(BOARD, exact, 25.0) == 25.0
    # the first true temperature a warming board reads a threshold at, from the reference down the counts
    at = ntc.reaches_c(BOARD, exact, 4000)
    assert ntc.reads_c(BOARD, exact, at) == pytest.approx(40.0, abs=0.05)
    assert ntc.reads_c(BOARD, exact, at - 0.5) < 40.0
    # a curve that tops out under a threshold never reads it short of the rail check
    assert ntc.reaches_c(BOARD, exact, 10_000) is None
    assert ntc.reaches_c(BOARD, (2048, 2500, 0, 0), 2500) is None


# the design curve's request on this board: the true temperature a protection limit is to land at, the stretch
# the fit is held over, the most it may read under the part's equation there, and the stretch its over-read is
# made least over
PINNED = {"pin_c": 90, "range_c": (15, 90), "floor_c": -1.0, "low_range_c": (15, 50)}


def residual_c(curve, lo_c, hi_c):
    """The reading less the part's nominal equation at every whole
    count between two true temperatures, as the kernel converts it."""
    raws = range(math.ceil(ntc.raw_of_c(BOARD, hi_c)), math.floor(ntc.raw_of_c(BOARD, lo_c)) + 1)
    return np.array([ntc.ntc_cc(raw, *curve) / 100 - float(BOARD.ntc_c_beta(raw)) for raw in raws])


def test_the_pinned_fit_gives_this_boards_words():
    words = ntc.fit_pinned(BOARD, 2048, **PINNED)
    assert words == (2430, 474, 19934)
    # what the form costs with them: the residual over the stretch, and over the low one
    held, low = residual_c((2048, *words), 15, 90), residual_c((2048, *words), 15, 50)
    assert (round(held.min(), 2), round(held.max(), 2)) == (-1.02, 5.5)
    assert (round(low.min(), 2), round(low.max(), 2)) == (-1.02, 3.27)
    # a stretch that starts lower moves nothing while the floor does not bind there
    assert ntc.fit_pinned(BOARD, 2048, **{**PINNED, "range_c": (0, 90)}) == words


@pytest.mark.parametrize("pin_c", [60, 80, 90, 100])
def test_the_pinned_fit_reads_the_pin_exact_to_the_count_grid(pin_c):
    curve = (2048, *ntc.fit_pinned(BOARD, 2048, **{**PINNED, "pin_c": pin_c, "range_c": (15, pin_c)}))
    # the pin falls between two whole counts, and the pin's number is read between those two
    raw = float(ntc.raw_of_c(BOARD, pin_c))
    cool, warm = math.ceil(raw), math.floor(raw)
    assert cool == warm + 1 and ntc.ntc_cc(cool, *curve) <= 100 * pin_c <= ntc.ntc_cc(warm, *curve)
    # so a warming board first reads it at the count just past the pin, under a count's width from it
    at = ntc.reaches_c(BOARD, curve, 100 * pin_c)
    assert at == float(BOARD.ntc_c_beta(warm)) and 0 <= at - pin_c < at - float(BOARD.ntc_c_beta(cool))


def test_the_pinned_fit_lands_this_boards_pin_within_a_count():
    curve = (2048, *ntc.fit_pinned(BOARD, 2048, **PINNED))
    assert ntc.reads_c(BOARD, curve, 90.0) == 89.98
    assert ntc.reaches_c(BOARD, curve, 9000) == pytest.approx(90.08, abs=0.005)


@pytest.mark.parametrize("pin_c", [60, 80, 90, 100])
def test_the_pinned_fit_holds_its_floor_at_every_whole_count(pin_c):
    request = {**PINNED, "pin_c": pin_c, "range_c": (15, pin_c)}
    held = residual_c((2048, *ntc.fit_pinned(BOARD, 2048, **request)), 15, pin_c)
    # to the rounding of the three words and the floors of the two products, a few centi-C
    assert -1.0 - 0.03 < held.min() < -1.0 + 0.03
    # a tighter floor is held as well, and costs over-read
    tight = residual_c((2048, *ntc.fit_pinned(BOARD, 2048, **{**request, "floor_c": 0.0})), 15, pin_c)
    assert -0.03 < tight.min() and tight.max() > held.max()


def test_the_pinned_fit_keeps_the_curvature_inside_its_field():
    # the stretch from 80 to 100 C alone bends harder than the field holds: the fit stops at the field's top
    steep = {"pin_c": 100, "range_c": (80, 100), "floor_c": -1.0, "low_range_c": (80, 100)}
    assert ntc.fit_pinned(BOARD, 2048, **steep)[2] == ntc.K2_TOP_Q24 == 0xFFFF
    assert ntc.fit_pinned(BOARD, 2048, **steep, k2_top_q24=10 ** 7)[2] > 4 * ntc.K2_TOP_Q24
    # a field narrower than the design curve wants holds it too, at a larger over-read
    words = ntc.fit_pinned(BOARD, 2048, **PINNED, k2_top_q24=15000)
    assert words[2] == 15000
    assert residual_c((2048, *words), 15, 50).max() > residual_c((2048, 2430, 474, 19934), 15, 50).max() + 1


def test_a_request_no_quadratic_meets_is_refused():
    # a floor over zero asks the curve to read over the equation where it is pinned on it
    with pytest.raises(ValueError, match="no quadratic about count 2048 with its curvature inside 0..65535 "
                                         r"\(Q24\) reads the beta equation exactly at 90 C and stays no more than "
                                         "-0.5 C under it from 15 to 90 C"):
        ntc.fit_pinned(BOARD, 2048, **{**PINNED, "floor_c": 0.5})
    # and on this board so does the floor held 10 C past the pin
    with pytest.raises(ValueError, match="stays no more than 1 C under it from 15 to 100 C"):
        ntc.fit_pinned(BOARD, 2048, **{**PINNED, "range_c": (15, 100)})
