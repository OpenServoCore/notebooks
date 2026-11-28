"""First contact at an end stop, read off the motor's ripple.

A stop rung drives the shaft onto one end stop open loop, at one duty, with
the rows at the control tick. Until the shaft meets the stop the motor
turns freely and its current carries the commutation ripple, one cycle per
commutation event, so the ripple counts the rotor's turn with no pot. At
the stop the rotor slows and the shunt rises to its seated level. First
contact is the row where the shunt's 5 ms box has made half that rise, and
its place is the rung's ripple cycle at that row.

The pot near an end stop is not always the shaft. Pressed into a stop it
can read anywhere from its seat to its blind end at one rotor angle, so
nothing here reads a stop off the pot. The pot enters only upstream of
the contact, where it is sound: its speed over the free stretch seeds the
ripple, and the rung's samples from its spin-up to G cycles before contact
place the rung's ripple count in the frame of a fitted table, the way one
rung's offset enters a pooled table fit. Every angle a stop is given then
comes from ripple cycles.

The ripple is counted on the line the rung's own free stretch predicts:
the pot's speed there over a table's counts a cycle. A rung's track that
reads counts a cycle off the table's has followed another line, and its
rung is left out of its stop, named.

`read` takes one rung's rows and gives its contact. `cycles_past` counts
the cycles from a pot code to the contact, the reference read on rows
whose pot is sound at that code. `placed` puts a contact in a table's
frame and checks its track. `stop` takes a stop's rungs to its median
and spread, `travel_deg`
the two stops to the physical travel, and `hold_margin_deg` gives K, the
least distance from a soft limit to first contact.

Rows run at 20 kHz unless `fs` says otherwise. Rows are indices into the
arrays a function is given.
"""
from dataclasses import dataclass

import numpy as np

from . import ripple, widecodes

FS = ripple.ROWS_HZ
SPIN_UP_S = 0.1         # s after the drive's edge before a rung's rows are read: the spin-up
FREE_S = 0.04           # s after the spin-up the free shunt level and the ripple's seed are read over
BOX_ROWS = 100          # rows (5 ms), the shunt's box
SEATED_S = 0.1          # s, the rung's last stretch the seated level is read over
TAIL_S = 0.005          # s at the rung's very end left out of the seated level
RISE_SHARE = 0.5        # first contact at this share of the rise from free to seated
MIN_RISE_COUNTS = 30.0  # raw counts, the least rise a contact is read on: a rung that ends short of its stop
# the share either side of the line the free stretch predicts that the
# ripple's seed is looked for in: the pot's speed over the free stretch
# over a table's counts a cycle there. A fixed band can hold another line
# than the fundamental and miss the fundamental itself on a fast rung.
SEED_SHARE = 0.3
# the share a rung's own counts a cycle may stand off the table's over the
# samples that place it: a track on another line than the fundamental reads
# a multiple of the table's, and the rung is left out of its stop
CHECK_SHARE = 0.15
# cycles before contact a rung's samples stop: the rise from its foot to 50 %
# takes about as long as 50 to 90 %, 2.6 +/- 1.1 cycles, and a pot that leaves
# the shaft at the stop starts to fall up to 2.3 cycles before the 50 % row.
# 4 cycles clears both at the mean plus 1 sd.
CUT_CYCLES = 4.0
SAMPLE_ROWS = 16        # rows a placement sample, and the pot's centered mean at it
ODD_CYCLES = 4.0        # a rung this far off its stop's median is named
# K, the least distance from a soft limit to first contact: a landing at
# a soft limit overshoots it by up to 0.75 deg at the default move and
# rests within the hold band, 0.3 deg, and the pot's low-end bumper starts
# up to 1.2 deg above the run's ripple contact, 2.3 deg together, rounded
# up. A K that covers only the contact's drift and the hold band lets every
# landing press the bumper, where a worn pot misreads and the loop answers
# the false error at full duty. On a low-ratio train, where HOLD_CUT_RATIO
# x CUT_CYCLES is wider, K is that, so a soft limit lies on samples a table
# was fitted on.
HOLD_MARGIN_DEG = 2.5
HOLD_CUT_RATIO = 1.25


@dataclass
class Contact:
    """One rung's first contact. `row` is the contact row, `cycle` the
    rung's ripple cycle there, counted from the end of the spin-up.
    `crossings` are the ripple's upward crossings, rows, that the cycles
    are read on, and `lead` the cycle at the spin-up's end."""
    dir: int
    row: int
    cycle: float
    free: float
    seated: float
    f_seed: float
    crossings: np.ndarray
    lead: float
    start: int

    @property
    def rise(self):
        """Raw counts from the free shunt level to the seated one."""
        return self.seated - self.free

    @property
    def valid(self):
        """Whether the rung reached its stop: a rise of MIN_RISE_COUNTS."""
        return self.rise >= MIN_RISE_COUNTS

    def cycle_at(self, row):
        """The rung's ripple cycle at `row`, from the spin-up's end."""
        index = np.arange(len(self.crossings), dtype=float)
        return np.interp(row, self.crossings, index) + self.lead


def driven(duty):
    """(first, last + 1): the rows of a capture the drive is on."""
    on = np.flatnonzero(np.asarray(duty) != 0)
    if len(on) == 0:
        raise ValueError("the drive is never on")
    return int(on[0]), int(on[-1]) + 1


def seed_hz(pos, counts_per_cycle, fs=FS):
    """The ripple line the free stretch predicts, Hz: the pot's speed over
    FREE_S after the spin-up, its centered means at the two ends, over the
    counts a cycle there. `counts_per_cycle` is a number, or a function of
    the stretch's two pot readings that gives a table's counts a cycle
    between them. None for a pot that does not move."""
    p = np.asarray(pos, float)
    start, free_rows = round(SPIN_UP_S * fs), round(FREE_S * fs)
    boxed = centered_mean(p[:start + free_rows + SAMPLE_ROWS], SAMPLE_ROWS)
    a, b = boxed[start], boxed[start + free_rows - 1]
    if a == b:
        return None
    per = counts_per_cycle(a, b) if callable(counts_per_cycle) else counts_per_cycle
    return abs(b - a) / ((free_rows - 1) / fs) / per if per > 0 else None


def table_counts_per_cycle(angle):
    """The `counts_per_cycle` of a table whose angle in cycles at a pot
    reading is `angle`: the counts between two readings over the cycles
    the table puts between them."""
    return lambda a, b: abs(b - a) / abs(float(angle(b)) - float(angle(a)))


def read(current, pos, dir, counts_per_cycle, fs=FS):
    """The Contact of one stop rung toward `dir` (+1 the pot rising, -1
    falling), from its driven rows' raw shunt `current` and pot `pos`.
    Free is the median of the shunt's box over FREE_S after the spin-up,
    seated its median over the last SEATED_S less TAIL_S, and contact the
    first row after the free stretch with the box at or above free +
    RISE_SHARE x (seated - free). The ripple is seeded at the line the
    free stretch's pot speed predicts at `counts_per_cycle` (`seed_hz`),
    looked for within SEED_SHARE of it. None when the rows are too short
    to read, the pot does not move or the spectrum has no line there."""
    x = np.asarray(current, float)
    start = round(SPIN_UP_S * fs)
    free_rows = round(FREE_S * fs)
    seated_rows, tail_rows = round(SEATED_S * fs), round(TAIL_S * fs)
    if len(x) < start + free_rows + seated_rows:
        return None
    box = np.convolve(x, np.ones(BOX_ROWS) / BOX_ROWS, mode="same")
    free = float(np.median(box[start:start + free_rows]))
    seated = float(np.median(box[-seated_rows:-tail_rows]))
    # the seed, the ripple line near the one the free stretch predicts, then the cycles from the spin-up's end on
    predicted = seed_hz(pos, counts_per_cycle, fs)
    if predicted is None:
        return None
    try:
        f_seed = ripple.line_freq(x[start:start + free_rows], (1 - SEED_SHARE) * predicted,
                                  (1 + SEED_SHARE) * predicted, fs=fs)
    except ValueError:
        return None     # no spectrum line at all near the predicted one
    tc = ripple.tracked_crossings(x, start, len(x) - 1, f_seed, fs=fs)
    if len(tc) < 5:
        return None
    after = start + free_rows
    at = np.flatnonzero(box[after:] >= free + RISE_SHARE * (seated - free))
    if len(at) == 0:
        return None
    row = after + int(at[0])
    lead = (tc[0] - start) * f_seed / fs
    c = Contact(dir=dir, row=row, cycle=0.0, free=free, seated=seated, f_seed=f_seed,
                crossings=tc, lead=lead, start=start)
    c.cycle = float(c.cycle_at(row))
    return c


def cycles_past(contact, pos, code):
    """Cycles from the first row the pot `pos` reads past `code` toward
    the stop to the contact: the reference read, on a pot that is sound
    at `code`."""
    p = np.asarray(pos, float)
    past = np.flatnonzero(p <= code if contact.dir < 0 else p >= code)
    if len(past) == 0:
        raise ValueError(f"the pot never reads past {code}")
    return contact.cycle - float(contact.cycle_at(past[0]))


def centered_mean(x, n):
    """The mean of `n` rows centered on each, fewer at the ends."""
    s = np.r_[0.0, np.cumsum(np.asarray(x, float))]
    i = np.arange(len(x))
    a, b = np.maximum(i - n // 2, 0), np.minimum(i + (n + 1) // 2, len(x))
    return (s[b] - s[a]) / (b - a)


@dataclass
class Placed:
    """A contact in a table's frame: `angle`, cycles, the table's angle at
    first contact. `offset` is the rung's ripple count's offset in that
    frame, `samples` how many samples set it and `residual_sd` their
    scatter about it, cycles. `counts_per_cycle` is the rung's own, the
    pot's travel over the samples over the rung's cycles there, and
    `table_counts_per_cycle` the table's over the same travel."""
    angle: float
    offset: float
    samples: int
    residual_sd: float
    counts_per_cycle: float
    table_counts_per_cycle: float

    @property
    def tracked(self):
        """Whether the rung's track is on the table's line: its counts a
        cycle within CHECK_SHARE of the table's."""
        return abs(self.counts_per_cycle / self.table_counts_per_cycle - 1) <= CHECK_SHARE


def placed(contact, pos, angle, slots=None, span=None):
    """`contact` in the frame of a table, `angle` the table's angle in
    cycles at a pot reading, rising with the pot. The rung's samples run
    from its first ripple crossing to CUT_CYCLES before contact, the pot's
    centered mean over SAMPLE_ROWS at each (its wide codes compensated
    with `slots` when given), and only those inside `span`, (low, high)
    counts, the stretch the table was fitted on, when given. Along the
    rung the table's angle is the offset plus dir x the rung's cycle, so
    the offset is the mean of the samples' angle less dir x cycle, the
    least-squares offset of one rung against a fixed table. The rung's
    counts a cycle over the samples are set beside the table's
    (`Placed.tracked`)."""
    p = np.asarray(pos)
    p = widecodes.compensated(p, slots) if slots is not None else p.astype(float)
    boxed = centered_mean(p, SAMPLE_ROWS)
    rows = np.arange(int(np.ceil(contact.crossings[0])), contact.row, SAMPLE_ROWS)
    cycle = contact.cycle_at(rows)
    keep = cycle <= contact.cycle - CUT_CYCLES
    if span is not None:
        keep &= (boxed[rows] >= span[0]) & (boxed[rows] <= span[1])
    if keep.sum() < 2:
        raise ValueError("the rung has no samples before its cut")
    rows, cycle = rows[keep], cycle[keep]
    at = np.asarray(angle(boxed[rows]), float)
    off = at - contact.dir * cycle
    offset = float(np.mean(off))
    travel = abs(boxed[rows[-1]] - boxed[rows[0]])
    return Placed(angle=offset + contact.dir * contact.cycle, offset=offset, samples=int(len(rows)),
                  residual_sd=float(np.std(off)), counts_per_cycle=travel / abs(cycle[-1] - cycle[0]),
                  table_counts_per_cycle=travel / abs(at[-1] - at[0]))


@dataclass
class Stop:
    """One stop from its rungs' contacts, cycles: the median, the spread
    (max - min) and the sd over the rungs kept, the rungs more than
    ODD_CYCLES off the median and the rungs left out, each by its index
    in the list given."""
    contact: float
    spread: float
    sd: float
    n: int
    odd: list
    left_out: list


def stop(angles, kept=None):
    """The Stop of one end from each rung's contact angle, cycles. A rung
    `kept` marks False, one whose track failed its check, is left out of
    the median and named. No other rung is dropped: one far off the
    median is named."""
    a = np.asarray(angles, float)
    keep = np.ones(len(a), bool) if kept is None else np.asarray(kept, bool)
    if not keep.any():
        raise ValueError("no rung read this stop's contact")
    m = float(np.median(a[keep]))
    k = a[keep]
    return Stop(contact=m, spread=float(k.max() - k.min()), sd=float(np.std(k, ddof=1)) if len(k) > 1 else 0.0,
                n=int(len(k)), odd=[int(i) for i in np.flatnonzero(keep & (np.abs(a - m) > ODD_CYCLES))],
                left_out=[int(i) for i in np.flatnonzero(~keep)])


def deg_per_cycle(ripple_per_rev, gear_ratio):
    """Output degrees per ripple cycle: one motor turn is `ripple_per_rev`
    cycles, one output turn `gear_ratio` motor turns."""
    return 360.0 / (ripple_per_rev * gear_ratio)


def travel_deg(low, high, ripple_per_rev, gear_ratio):
    """The physical travel, deg: contact to contact, cycles, at the
    train's degrees a cycle."""
    return (high - low) * deg_per_cycle(ripple_per_rev, gear_ratio)


def hold_margin_deg(per_cycle):
    """K, deg: the least distance from a soft limit to first contact, at
    `per_cycle` output degrees a ripple cycle."""
    return max(HOLD_MARGIN_DEG, HOLD_CUT_RATIO * CUT_CYCLES * per_cycle)
