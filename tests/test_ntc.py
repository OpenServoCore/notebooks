"""The board NTC's conversion as the kernel runs it, held to the
firmware's own function on the same counts, and what a curve reads
against the part's nominal equation."""
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
    curve = (2048, 2584, 212, 28463)
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
