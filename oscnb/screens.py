"""What two protection screens of a servo must clear, read off its rows:
the largest speed the observer reads on a still shaft, which the stall
speed sits over, and the worst run of pot steps the sensor screen
tests, which the pot jump sits over.

The stall detector counts a press only while the observer's speed stays
under the stall speed on every MEDIUM period, so the bound has to sit
over the speed the observer reads with the loop driving a shaft that
does not move. `still_speed_cps` is the largest |omega_hat| over the
rows of the still holds, in count/s on the count the servo ran in. The
rows carry `omega_hat_cps` as the servo streams it, raw csQ16, one
MEDIUM period's value repeated over that period's rows. The still holds
are a closed-loop check's holds at rest, the scorecard's `hold` captures
on the record. A wide-code hold is not one of them. The pot flips
across the skipped span there, and the observer's speed with it.

The reader leaves out the first STILL_SKIP_MS of each hold. A burst of
rows that starts on a still shaft disturbs the pot over its first rows.
In the first 1.5 ms the theta-hat steps by 0.5 to 1.8 counts, and the
observer reads 100 to 200 count/s with no current. From 10 ms on, the
same holds read 59 to 64 count/s at most. The record's holds read a
median of 60 count/s, each hold's largest over its rows.
The record's holds lay inside continuous captures and had no such
opening. The skip applies to every hold all the same, so holds streamed
either way are read alike.

The sensor screen takes the raw pot's step from one RAIL period to the
next, once a MEDIUM period, and faults when a number of steps in a row
each pass its bound. What a healthy pot must clear is therefore the
level that `run` steps in a row all pass: the worst over the rows of
the least of `run` successive steps (`step_run`). The rows are FAST
ticks, so the RAIL samples of every phase of the period are scanned,
and a run never spans a hole in the ticks. The statistic of record
reads a run of 4, the screen's own count.

The rows the statistic reads are the driven rows of an identification
run, its spins, its table's rungs and its closed-loop check, and the
check's wide-code holds. Those are two 2 s holds at the middle of each
wide code that takes a slot, as the scorecard's `wide` capture holds
them (stages/5-closed-loop/scorecard/testset.py). The record's worst run
sits on a wide code. A shaft resting across a code's edge flips the
reading across the span the code skips, so the statistic there is the
span and 2 or 3 counts. Holds off every wide code read 2 to 4 counts, so
a run that holds at no wide code never reaches the level a healthy pot
must clear.

A RAIL sample inside one of the run's pot holes is left out, and no run
spans it, since the bound is what a healthy pot must clear. A pot hole
is a stretch of travel where the pot's counts per ripple cycle on a slow
pass of the table run far over typical. A cycle is far over when its
counts over the pass's median, r, pass 1 + 4 s, s the 99th percentile of
|r - 1| over the inner travel. The dead low end of a worn track is one.
There the reading glides by 21 to 22 counts a RAIL period, where the
shaft at no-load speed moves about 6. The reader takes the holes as raw
ranges.

Both readers are ported from the bringup analysis of the protection
screens on mg90-d with dev-v006-2A-D. It read the still-shaft speed off
a replay of the observer over the scorecard's hold captures, 115
count/s at most over 308 holds, and the 4-step run off the rows of
record, 15 counts on 5 V. The opening skip, the wide-code holds and the
pot holes come from its analysis of a run on the Default policy.
"""

import numpy as np

DECIM_MED = 10          # FAST ticks a MEDIUM period, which the RAIL phase runs once in
Q16 = 1 << 16           # the raw csQ16 of omega_hat_cps over count/s
SCREEN_RUN = 4          # the steps in a row the statistic of record reads
FAST_HZ = 20_000        # FAST ticks a second, one row a tick
STILL_SKIP_MS = 10      # the opening of a still hold, where a burst start disturbs the pot
STILL_SKIP_TICKS = STILL_SKIP_MS * FAST_HZ // 1000


def still_speed_cps(rows, holds):
    """The largest |omega_hat| over the rows of segments `holds`, count/s,
    each hold read from STILL_SKIP_TICKS past its first row: `rows` a
    frame with `seg`, `tick` and `omega_hat_cps` (raw csQ16). Refused
    where no row read carries a speed."""
    held = rows.loc[rows["seg"].isin(list(holds))]
    # the ticks of each row since the first row of its hold
    since = held["tick"] - held.groupby("seg")["tick"].transform("min")
    speed = held.loc[since >= STILL_SKIP_TICKS, "omega_hat_cps"].dropna()
    if speed.empty:
        raise ValueError(f"no row of the still holds from {STILL_SKIP_MS} ms on carries omega_hat_cps")
    # the multiplication form of the csQ16 scale
    return float(np.abs(speed.to_numpy(dtype=np.int64)).max() * (1.0 / Q16))


def runs_of(ticks):
    """Where a segment's ticks break: the start of each run of
    successive ticks."""
    return np.flatnonzero(np.diff(ticks) != 1) + 1


def worst_run(steps, run):
    """The worst level `run` successive steps all pass: the most of the
    least over each window of `run`; None for fewer steps."""
    if len(steps) < run:
        return None
    return int(np.lib.stride_tricks.sliding_window_view(steps, run).min(axis=1).max())


def outside(samples, holes):
    """The pieces of `samples` between the ones inside a hole, in order,
    the ones inside left out."""
    inside = np.zeros(len(samples), dtype=bool)
    for low, high in holes:
        inside |= (samples >= low) & (samples <= high)
    cuts = np.flatnonzero(inside)
    # each piece after the first starts at a sample inside a hole
    return [piece[1:] if k else piece for k, piece in enumerate(np.split(samples, cuts))]


def step_run(rows, run=SCREEN_RUN, holes=()):
    """The worst level `run` successive RAIL-period pot steps all pass,
    raw counts, over every RAIL phase of every run of successive ticks of
    every segment: `rows` a frame with `seg`, `tick` and `pos`, the raw
    pot, and `holes` the run's pot holes, (from, to) raw counts with both
    ends inside. A RAIL sample inside a hole is left out, and no run spans
    it. None where no stretch holds `run` steps."""
    holes = [(low, high) for low, high in holes]
    wrong = [h for h in holes if h[0] > h[1]]
    if wrong:
        raise ValueError(f"a pot hole runs from its lower end to its upper one, and {wrong} does not")
    worst = None
    for _, seg in rows.groupby("seg", sort=False):
        ticks = seg["tick"].to_numpy(dtype=np.int64)
        pot = seg["pos"].to_numpy(dtype=np.int64)
        cuts = runs_of(ticks)
        for stretch in np.split(pot, cuts):
            for phase in range(DECIM_MED):
                # the RAIL samples of one phase, a MEDIUM period apart, the ones in a hole cut out
                for piece in outside(stretch[phase::DECIM_MED], holes):
                    level = worst_run(np.abs(np.diff(piece)), run)
                    if level is not None and (worst is None or level > worst):
                        worst = level
    return worst
