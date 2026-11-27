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
MEDIUM period's value repeated over that period's rows.

The sensor screen takes the raw pot's step from one RAIL period to the
next, once a MEDIUM period, and faults when a number of steps in a row
each pass its bound. What a healthy pot must clear is therefore the
level that `run` steps in a row all pass: the worst over the rows of
the least of `run` successive steps (`step_run`). The rows are FAST
ticks, so the RAIL samples of every phase of the period are scanned,
and a run never spans a hole in the ticks. The statistic of record
reads a run of 4, the screen's own count.

Both readers are ported from the bringup analysis of the protection
screens on mg90-d with dev-v006-2A-D. It read the still-shaft speed off
a replay of the observer over the scorecard's hold captures, 115
count/s at most over 308 holds, and the 4-step run off the rows of
record, 15 counts on 5 V.
"""

import numpy as np

DECIM_MED = 10          # FAST ticks a MEDIUM period, which the RAIL phase runs once in
Q16 = 1 << 16           # the raw csQ16 of omega_hat_cps over count/s
SCREEN_RUN = 4          # the steps in a row the statistic of record reads


def still_speed_cps(rows, holds):
    """The largest |omega_hat| over the rows of segments `holds`, count/s:
    `rows` a frame with `seg` and `omega_hat_cps` (raw csQ16). Refused
    where no row of a hold carries a speed."""
    speed = rows.loc[rows["seg"].isin(list(holds)), "omega_hat_cps"].dropna()
    if speed.empty:
        raise ValueError("no row of the still holds carries omega_hat_cps")
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


def step_run(rows, run=SCREEN_RUN):
    """The worst level `run` successive RAIL-period pot steps all pass,
    raw counts, over every RAIL phase of every run of successive ticks of
    every segment: `rows` a frame with `seg`, `tick` and `pos`, the raw
    pot. None where no stretch holds `run` steps."""
    worst = None
    for _, seg in rows.groupby("seg", sort=False):
        ticks = seg["tick"].to_numpy(dtype=np.int64)
        pot = seg["pos"].to_numpy(dtype=np.int64)
        cuts = runs_of(ticks)
        for stretch in np.split(pot, cuts):
            for phase in range(DECIM_MED):
                # the RAIL samples of one phase, a MEDIUM period apart
                steps = np.abs(np.diff(stretch[phase::DECIM_MED]))
                level = worst_run(steps, run)
                if level is not None and (worst is None or level > worst):
                    worst = level
    return worst
