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

A dataset is `chains.csv`, one row per sweep, `chains/K` sweeps and
`seats.csv`.
"""
import time

from oscnb.sitting import Plan, log_row, seat_pos

LOCKED_RAILS = (3.3, 3.7)  # 100 % laps, 0.74 and 0.83 A under the lock hold
LEVELS_V = (1.2, 1.8)      # the stop chain's effective volts, 0.27 and 0.40 A
STOP_RAIL = 5.0            # the user's supply
LAP_MS = 300               # each lap or level, the copper cold after
REPEATS = 3                # laps per rail when locked, chains per stop
STOPS = ("rev", "fwd")     # the low stop, then the high, alternated


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


def stop(s, out):
    s.check(STOP_RAIL, chain(STOP_RAIL), seat="stop")
    low, high = duties(STOP_RAIL)
    k = 0
    for _ in range(REPEATS):
        for side in STOPS:
            sub = f"chains/{k}"
            t0 = time.time()
            s.drive(out / sub, STOP_RAIL, chain(STOP_RAIL), dirs=side,
                    seat="stop")
            (pos,) = seat_pos(out / sub).values()
            log_row(out / "chains.csv", row(k, s, STOP_RAIL, sub, t0,
                                            duty_low_pct=low,
                                            duty_high_pct=high, stop=side,
                                            seat_pos=pos))
            s.seat(out, side, round(pos))
            k += 1
            s.rest(f"chain {k - 1} done ({side} stop), the copper resting")


PLANS = [
    Plan("locked", "resistance", "servo",
         "100 % laps of 300 ms into the locked horn, 3.3 and 3.7 V",
         run=locked, order=10, lock=True, minutes=4, mb=16),
    Plan("stop", "resistance", "servo",
         "the user's pass: 1.2, 1.8, 1.2 V effective chains on 5 V into "
         "the end stops",
         run=stop, order=20, seated=True, minutes=4, mb=20),
]
