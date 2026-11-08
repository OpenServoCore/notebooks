"""Stage 3, resistance: the terminal resistance of the motor inside the
servo and the contact offset V0, as the line through laps at two levels,
with the rotor held. Run by `python -m oscnb.sitting servo`.

`locked` is the reference, stage 2's stall pass in situ. The horn lock holds
the output at mid travel, so the torque goes through the train with no
stopper in the path, and each sweep is a 100 % lap of 300 ms in each
direction, three on 3.3 V and three on 3.7 V, 20 s apart. At 100 % nothing
switches, so the crest sample is the mean and no window model enters R.
Both rails stay under the servo's lock hold by the type R, 0.74 and 0.83
A; 4.2 V, stage 2's third rail, would draw 0.94 A and is left out. `pos` is
read before and after each lap; a seat that walks more than the runner's
slip limit from the lock's first reading stops every seated plan of the
sitting.

`stop` is the user's pass, the variant `osc-cal` can run on a servo with
no lock: a chain of 1.2, 1.8 and 1.2 V effective, 300 ms each, on 5 V,
pressed into the end stop by `osc-lab sweep --stall`, three chains at each
stop with the stops alternated, under the servo's lap cap. Its crest
samples need the inductance result's window model to read a mean. The seat
is the median pos the TEL rows read through each chain, against the first
chain at that stop. The notebook grades it against `locked`.

`rails`, optional, is the same pass on 3.7 and 7.4 V at the same two
effective volts, 32 and 49 % and 16 and 24 %: the notebook's three
candidate terms in a partial-duty read, the bridge's edge lags, the brake
interval's drop and a crest over-read per volt, scale as 1/D, (1 - D) and
(1 - D) V_on, so the same two currents on three rails separate them with
no meter.

`meter`, in the `servo-meter` state with the DMM in series on the motor,
holds each of the six levels of `stop` and `rails` into a stop for
METER_HOLD_S with the hold unstreamed between two short windows, the meter
logging into `dmm.csv` through the plan. The meter's mean current through
the hold against the crest sample's in the windows is the as-read current
error at each level with no construction in it: a term proportional to
(1 - D) V_on, a constant offset and no term each predict a different six.

A dataset is `chains.csv`, one row per sweep, `chains/K` sweeps and
`seats.csv`; `meter` writes `holds.csv`, `holds/K` and `dmm.csv`.
"""
import time

from oscnb.sitting import Plan, log_row, seat_pos

LOCKED_RAILS = (3.3, 3.7)  # 100 % laps, 0.74 and 0.83 A under the lock hold
LEVELS_V = (1.2, 1.8)      # the stop chain's effective volts, 0.27 and 0.40 A
STOP_RAIL = 5.0            # the user's supply
RAILS = (3.7, 7.4)         # the pass again at the other two rails
LAP_MS = 300               # each lap or level, the copper cold after
REPEATS = 3                # laps per rail when locked, chains per stop
STOPS = ("rev", "fwd")     # the low stop, then the high, alternated
METER_RAILS = (3.7, 5.0, 7.4)  # the chains' three rails, ascending
METER_HOLD_S = 2.5         # a hold long enough for four of the meter's 0.51 s packets
METER_WINDOW_MS = 300      # the streamed window either side of the hold
METER_REPEATS = 3          # holds per level per stop


def duties(rail):
    """The chain's two duties on `rail`, percent, for LEVELS_V."""
    low, high = (round(100 * v / rail) for v in LEVELS_V)
    return low, high


def chain(rail):
    """The lap chain on `rail`: low, high, low, chained with no release."""
    low, high = duties(rail)
    return [f"{low}@{LAP_MS}", f"then:{high}@{LAP_MS}",
            f"then:{low}@{LAP_MS}"]


def row(k, s, rail, sub, t0, **extra):
    dps_read, vsys = s.rail_read
    return {"k": k, "volts_set": rail, "dps_read": f"{dps_read:.3f}",
            "vsys_read": f"{vsys:.3f}", **extra, "t_start": f"{t0:.3f}",
            "t_end": f"{time.time():.3f}", "folder": sub}


def locked(s, out):
    lap = [f"100@{LAP_MS}"]
    for rail in LOCKED_RAILS:
        s.check(rail, lap, seat="lock")
    k = 0
    for rail in LOCKED_RAILS:
        for _ in range(REPEATS):
            sub = f"chains/{k}"
            before = s.probe("before the lap")["pos"]
            s.seat(out, "lock", before)
            t0 = time.time()
            s.drive(out / sub, rail, lap, dirs="both", seat="lock")
            after = s.probe("after the lap")["pos"]
            log_row(out / "chains.csv", row(k, s, rail, sub, t0,
                                            duty_pct=100, pos_before=before,
                                            pos_after=after))
            s.seat(out, "lock", after)
            k += 1
            s.rest(f"lap {k - 1} done ({rail} V), the copper resting")


def chains_at(s, out, rails):
    for rail in rails:
        s.check(rail, chain(rail), seat="stop")
    k = 0
    for rail in rails:
        low, high = duties(rail)
        for _ in range(REPEATS):
            for side in STOPS:
                sub = f"chains/{k}"
                t0 = time.time()
                s.drive(out / sub, rail, chain(rail), dirs=side, seat="stop")
                (pos,) = seat_pos(out / sub).values()
                log_row(out / "chains.csv", row(k, s, rail, sub, t0,
                                                duty_low_pct=low,
                                                duty_high_pct=high,
                                                stop=side, seat_pos=pos))
                s.seat(out, side, round(pos))
                k += 1
                s.rest(f"chain {k - 1} done ({rail} V, {side} stop), the "
                       "copper resting")


def stop(s, out):
    chains_at(s, out, (STOP_RAIL,))


def held(duty):
    """One level held into the stop: a window, the hold unstreamed, a window."""
    return [f"{duty}@{METER_WINDOW_MS}",
            f"hold:{duty}@{int(METER_HOLD_S * 1000)}",
            f"then:{duty}@{METER_WINDOW_MS}"]


def meter(s, out):
    for rail in METER_RAILS:
        for duty in duties(rail):
            s.check(rail, held(duty), seat="stop")
    k = 0
    with s.meter_log(out):
        for rail in METER_RAILS:
            for level_v, duty in zip(LEVELS_V, duties(rail)):
                for _ in range(METER_REPEATS):
                    for side in STOPS:
                        sub = f"holds/{k}"
                        t0 = time.time()
                        s.drive(out / sub, rail, held(duty), dirs=side,
                                seat="stop")
                        (pos,) = seat_pos(out / sub).values()
                        log_row(out / "holds.csv", row(k, s, rail, sub, t0,
                                                       level_v=level_v,
                                                       duty_pct=duty,
                                                       stop=side,
                                                       seat_pos=pos))
                        s.seat(out, side, round(pos))
                        k += 1
                        s.rest(f"hold {k - 1} done ({rail} V {duty} %, "
                               f"{side} stop), the copper resting")


def rails(s, out):
    chains_at(s, out, RAILS)


PLANS = [
    Plan("locked", "resistance", "servo",
         "100 % laps of 300 ms into the locked horn, 3.3 and 3.7 V",
         run=locked, order=10, lock=True, minutes=4, mb=16),
    Plan("stop", "resistance", "servo",
         "the user's pass: 1.2, 1.8, 1.2 V effective chains on 5 V into "
         "the end stops",
         run=stop, order=20, seated=True, minutes=4, mb=20),
    Plan("rails", "resistance", "servo",
         "the same chains on 3.7 and 7.4 V at the same effective volts",
         run=rails, order=21, seated=True, optional=True, minutes=8, mb=40),
    Plan("meter", "resistance", "servo-meter",
         "the six levels held 2.5 s into the stops with the DMM in series, "
         "the crest sample against the meter",
         run=meter, order=22, seated=True, minutes=29, mb=36),
]
