"""The loop-margin reading on rows of a synthetic loop whose open-loop
gains are known in closed form: a PI speed loop on a first-order plant
that answers a period late, a position gain around it, at the medium
rate, driven as the firmware drives a margin sweep (a cruise and a
triangle summed into the speed reference, the three values published a
period and held over its ten rows)."""
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from scipy.optimize import brentq

from oscnb import margin as mg

T = 1 / mg.PERIOD_HZ
POLE = np.exp(-T / 0.002)       # the plant, unity gain at DC from current counts to c/s
GAIN = 1 - POLE
KP, KI = 1.0, 0.4               # the speed loop's PI, current counts per c/s
P_KP_Q88 = 170 * 256            # the position gain, c/s per count
CRUISE = 800                    # c/s
SECOND = CRUISE * 5 // 4
SPEED_BAND = (1, 2, 3, 4, 5, 6, 7, 8, 10, 12)
POSITION_BAND = (15, 20, 25, 30, 40, 50, 70)
PERIODS = 6000
PREROLL = 200                   # periods of plain cruise before a burst opens
TICK0 = 65_000                  # every burst wraps the 16-bit tick
PUBLISH = 3                     # the row of a period its values first show on
SIGMA_THETA = 1.0               # pot-like noise, white a period: counts on the observer's position
SIGMA_OMEGA = 8.0               # and c/s on the speed the loop closes on
DB, DEG = 0.2, 2.0


def slope_q88(n):
    return min(15 * 256, 2 * 150 * 256 // n)


def closed_form(n):
    """(L_v, L_p) at the frequency of half-period n."""
    z1 = np.exp(-1j * np.pi / n)
    lv = (KP + KI / (1 - z1)) * GAIN * z1 / (1 - POLE * z1)
    return lv, P_KP_Q88 / 256 * T / (1 - z1) * lv / (1 + lv)


def burst(n, cruise, direction, rng=None, pause=None, hole=None, wave=None, amp=0.0):
    """(rows, facts) of one burst. `pause` the period the triangle holds
    off, `hole` the first of 16 rows lost, `wave` and `amp` a current
    disturbance fixed in position, amp x sin(2 pi theta / wave)."""
    goal = direction * cruise
    q = slope_q88(n)
    v, integ = float(goal), float(goal)
    theta = 500.0 if direction > 0 else 4000.0
    p = 0
    rec = np.zeros((PERIODS, 3), np.int64)
    for k in range(-PREROLL, PERIODS):
        ref = goal << 16
        if k >= 0 and k != pause:
            p += 1
            p = 0 if p >= 2 * n else p
            ref += (q * ((n >> 1) - abs(p - n))) << 8
        w = v + (rng.normal(0, SIGMA_OMEGA) if rng else 0.0)
        e = ref / mg.Q16 - w
        integ += KI * e
        i = KP * e + integ
        if k >= 0:
            th = theta + (rng.normal(0, SIGMA_THETA) if rng else 0.0)
            rec[k] = round(th * mg.Q16), round(w * mg.Q16), ref
        if amp:
            i += amp * np.sin(2 * np.pi * theta / wave)
        v = POLE * v + GAIN * i
        theta += T * v
    j = np.arange(PERIODS * mg.PERIOD_TICKS - PUBLISH)
    if hole is not None:
        j = np.concatenate((j[:hole], j[hole + 16:]))
    held = rec[(j + PUBLISH) // mg.PERIOD_TICKS]
    rows = pd.DataFrame({"tick": (TICK0 + j) % mg.TICK_WRAP, "theta_hat_q16": held[:, 0],
                         "omega_hat_cps": held[:, 1], "omega_ref_cps": held[:, 2]})
    facts = {"inject_half_periods": n, "inject_accel_q88": q, "cruise_cps": cruise, "direction": direction,
             "p_kp_q88": P_KP_Q88, "rows_dropped": 0, "seq_holes": 0 if hole is None else 1}
    return rows, facts


def sweep(seed=None, **kw):
    """Every point a burst up and a burst down, the position band again
    at the second cruise."""
    rng = None if seed is None else np.random.default_rng(seed)
    plan = [(n, CRUISE) for n in SPEED_BAND + POSITION_BAND] + [(n, SECOND) for n in POSITION_BAND]
    return [burst(n, c, d, rng, **kw) for n, c in plan for d in (1, -1)]


@pytest.fixture(scope="module")
def quiet():
    return sweep()


def mid_window(n, cycle):
    """A period in the middle of a cycle the reader cuts."""
    return round(mg.HEAD_S * mg.PERIOD_HZ) + cycle * 2 * n + n


def off_closed_form(result):
    """The worst (dB, deg) of each loop's band against the closed form."""
    out = {}
    for loop, band, k in (("speed", SPEED_BAND, 0), ("position", POSITION_BAND, 1)):
        worst = [0.0, 0.0]
        for p in result[loop]["points"]:
            if p["half_periods"] in band:
                ref = closed_form(p["half_periods"])[k]
                worst[0] = max(worst[0], abs(p["db"] - mg.db(ref)))
                worst[1] = max(worst[1], abs(p["deg"] - mg.deg(ref)))
        out[loop] = worst
    return out


def test_the_tick_unwraps_across_its_16_bit_wrap():
    t = mg.unwrap([65_533, 65_534, 65_535, 0, 1, 5])
    assert t.tolist() == [65_533, 65_534, 65_535, 65_536, 65_537, 65_541]
    with pytest.raises(ValueError):
        mg.unwrap([7, 7])


def test_every_point_of_both_bands_reads_the_closed_form_on_quiet_rows(quiet):
    result = mg.read(quiet)
    for loop, (d, a) in off_closed_form(result).items():
        assert d < DB, loop
        assert a < DEG, loop
    for loop in mg.LOOPS:
        assert all(p["dropped"] == 0 and p["bursts"] == 2 for p in result[loop]["points"])


def closed_margins():
    """(speed crossover phase margin, speed gain margin, position phase
    margin, position gain margin bound) of the closed form: the phase
    margins at the crossovers solved on a fine grid, the speed gain margin
    at Nyquist, the position bound by the reader's rule over its points."""
    lv = lambda w: closed_form(np.pi / w)[0]
    lp = lambda w: closed_form(np.pi / w)[1]
    wv = brentq(lambda w: abs(lv(w)) - 1, 1e-3, np.pi - 1e-6)
    wp = brentq(lambda w: abs(lp(w)) - 1, 1e-3, wv)
    fp = wp * mg.PERIOD_HZ / (2 * np.pi)
    above = [mg.db(closed_form(n)[1]) for n in SPEED_BAND + POSITION_BAND
             if n > 1 and mg.PERIOD_HZ / (2 * n) > fp and mg.deg(closed_form(n)[1]) < mg.BOUND_DEG]
    return 180 + mg.deg(lv(wv)), -mg.db(closed_form(1)[0].real), 180 + mg.deg(lp(wp)), -max(above)


def test_the_margins_read_on_quiet_rows_are_the_closed_form(quiet):
    result = mg.read(quiet)
    pm_v, gm_v, pm_p, gm_p = closed_margins()
    sp, po = result["speed"], result["position"]
    assert sp["pm_deg"] == pytest.approx(pm_v, abs=1.0)
    assert sp["gm_db"] == pytest.approx(gm_v, abs=0.2)
    assert sp["gm_hz"] == mg.PERIOD_HZ / 2 and not sp["gm_over"]
    assert po["pm_deg"] == pytest.approx(pm_p, abs=1.0)
    # the position loop's phase reaches -180 only at Nyquist: a bound
    assert po["gm_over"]
    assert po["gm_db"] == pytest.approx(gm_p, abs=0.2)


def test_a_paused_cycle_and_a_hole_are_dropped_and_counted_and_the_reading_holds(quiet):
    bursts = list(quiet)
    plan = [(f["inject_half_periods"], f["cruise_cps"], f["direction"]) for _, f in bursts]
    paused, holed = plan.index((25, CRUISE, 1)), plan.index((8, CRUISE, -1))
    bursts[paused] = burst(25, CRUISE, 1, pause=mid_window(25, 5))
    bursts[holed] = burst(8, CRUISE, -1, hole=mid_window(8, 3) * mg.PERIOD_TICKS + 2)
    result = mg.read(bursts)
    by_n = {p["half_periods"]: p for p in result["position"]["points"]}
    assert (by_n[25]["pauses"], by_n[25]["holes"]) == (1, 0)
    assert (by_n[8]["pauses"], by_n[8]["holes"], by_n[8]["seq_holes"]) == (0, 1, 1)
    assert by_n[8]["dropped"] == by_n[25]["dropped"] == 1
    assert sum(p["dropped"] for p in by_n.values()) == 2
    for loop, (d, a) in off_closed_form(result).items():
        assert d < DB, loop
        assert a < DEG, loop


def test_two_seeds_of_pot_like_noise_repeat_within_the_bounds():
    a, b = mg.read(sweep(seed=1)), mg.read(sweep(seed=2))
    for loop, band in (("speed", SPEED_BAND), ("position", POSITION_BAND)):
        for pa, pb in zip(a[loop]["points"], b[loop]["points"]):
            if pa["half_periods"] in band:
                assert abs(pa["db"] - pb["db"]) < 1.0, (loop, pa["half_periods"])
                assert abs(pa["deg"] - pb["deg"]) < 10.0, (loop, pa["half_periods"])


def test_a_point_on_a_planted_interferer_is_flagged_by_the_two_cruise_check():
    # fixed in position, it reads at 25 Hz (N = 40) at the first cruise and
    # at 31.25 Hz, between the points, at the second
    result = mg.read(sweep(wave=CRUISE / 25, amp=150.0))
    flags = {p["half_periods"]: p["two_cruise"] for p in result["position"]["points"]}
    assert {n for n, f in flags.items() if f} == {40}
    assert all(flags[n] is False for n in POSITION_BAND if n != 40)
    assert all(flags[n] is None for n in SPEED_BAND)


def test_a_sweep_reads_the_same_from_its_folder(tmp_path):
    bursts = [burst(n, CRUISE, d) for n in (4, 20) for d in (1, -1)]
    for k, (rows, facts) in enumerate(bursts):
        rows.to_csv(tmp_path / f"burst-{k}.csv", index=False)
        (tmp_path / f"burst-{k}.json").write_text(pd.Series(facts).to_json())
    a, b = mg.read(bursts), mg.read(mg.load(tmp_path))
    for loop in mg.LOOPS:
        assert [p["L"] for p in a[loop]["points"]] == [p["L"] for p in b[loop]["points"]]


def test_the_half_period_is_read_at_most_512():
    facts = {"inject_half_periods": 600, "inject_accel_q88": 150}
    assert mg.HALF_PERIODS_MAX == 512
    assert mg.half_periods(facts) == 512
    assert mg.half_periods({**facts, "inject_half_periods": 512}) == 512


def test_two_gain_margins_agree_within_1_db_or_both_at_twice_the_floor():
    assert 2 * mg.GM_FLOOR_DB == 12.0
    assert mg.gm_agree(5.0, 5.9)
    assert not mg.gm_agree(5.0, 6.1)
    assert mg.gm_agree(12.0, 25.0)
    assert not mg.gm_agree(11.9, 25.0)


def test_two_seeds_repeat_and_gain_margins_apart_under_twice_the_floor_do_not():
    a, b = mg.read(sweep(seed=1)), mg.read(sweep(seed=2))
    band = {"speed": set(SPEED_BAND), "position": set(POSITION_BAND)}
    r = mg.repeat(a, b, band)
    for loop in mg.LOOPS:
        assert r[loop]["ok"], (loop, r[loop])
        # a point either sweep's two-cruise check flags is left out
        seen = {p[0] for p in r[loop]["points"]}
        assert seen <= band[loop] and len(seen) >= len(band[loop]) - 1, (loop, seen)
    # gain margins 1.5 dB apart under twice the floor do not repeat; 3 dB
    # apart over it do
    def gm(result, db):
        return {**result, "speed": {**result["speed"], "gm_db": db}}
    r = mg.repeat(gm(a, 8.0), gm(b, 9.5), band)
    assert r["speed"]["gm"][2] is False and not r["speed"]["ok"]
    assert r["position"]["ok"]
    assert mg.repeat(gm(a, 13.0), gm(b, 16.0), band)["speed"]["ok"]


def test_two_cruises_of_the_speed_band_are_two_readings_and_the_second_cruise_a_check():
    fast = 2 * CRUISE

    def tagged(bursts, reading):
        return [(rows, {**facts, "reading_cps": reading}) for rows, facts in bursts]
    slow = tagged(sweep(), CRUISE)
    quick = tagged([burst(n, fast, d) for n in SPEED_BAND for d in (1, -1)], fast)
    result = mg.readings(slow + quick)
    assert sorted(result) == [CRUISE, fast]
    assert all(p["two_cruise"] is None and p["cruise_cps"] == fast for p in result[fast]["speed"]["points"])
    flags = {p["half_periods"]: p["two_cruise"] for p in result[CRUISE]["position"]["points"]}
    assert all(flags[n] is False for n in POSITION_BAND)
    for p in result[fast]["speed"]["points"]:
        ref = closed_form(p["half_periods"])[0]
        assert abs(p["db"] - mg.db(ref)) < DB and abs(p["deg"] - mg.deg(ref)) < DEG
    # facts with no reading_cps are one reading, as before
    assert list(mg.readings(sweep())) == [None]


FIXTURE = Path(__file__).parent / "margin_osc_lab"


def test_a_burst_osc_labs_own_writer_wrote_reads_its_rigs_plant():
    """The folder `osc-lab`'s writer wrote for its test rig's plant
    (`margin::tests::the_writer_writes_the_fixture_the_estimator_reads`,
    which fails when the writer drifts from these files): the observer's
    speed half the reference's departure from the cruise a period late and
    its position the speed's sum, so L_v = (z^-1 / 2) / (1 - z^-1 / 2) and
    L_p = kp z^-2 / (2 fm (1 - z^-1))."""
    bursts = mg.load(FIXTURE)
    assert len(bursts) == 1
    rows, facts = bursts[0]
    assert set(mg.FACTS) <= set(facts) and facts["reading_cps"] == facts["cruise_cps"]
    assert mg.bands(bursts) == {"speed": {facts["inject_half_periods"]}}
    result = mg.readings(bursts)
    assert list(result) == [facts["cruise_cps"]]
    n = facts["inject_half_periods"]
    z1 = np.exp(-1j * np.pi / n)
    lv = 0.5 * z1 / (1 - 0.5 * z1)
    lp = facts["p_kp_q88"] / 256 * 0.5 * z1 ** 2 / (mg.PERIOD_HZ * (1 - z1))
    for loop, want in (("speed", lv), ("position", lp)):
        (p,) = result[facts["cruise_cps"]][loop]["points"]
        assert p["half_periods"] == n and p["cycles"] == facts["cycles"] and p["dropped"] == 0
        assert abs(p["db"] - mg.db(want)) < 0.05, loop
        assert abs(p["deg"] - mg.deg(want)) < 0.5, loop
