# Stage 5. Closed loop

The servo's own closed loop, scored. A firmware build runs one fixed set of
tests in Position mode, and the scorecard turns the rows into a short list
of numbers: how far a move overshoots and how long it takes to settle,
where it comes to rest, how closely it follows a slow goal, how still it
holds. A change to the firmware or its settings is kept when the numbers
get better and dropped when they do not. A notebook comes once there is
data.

Nothing here describes what the firmware does. A build is named by its
commit and the sha1 of its image, and its settings by the CONFIG snapshot
every capture's `meta.json` carries.

## The three kinds of data

- **The test set**, [scorecard/testset.py](scorecard/testset.py). Up to
  four captures per run, each one `osc-lab track` call: `steps` (the first
  second after torque-on, then moves of 2, 20 and 60 degrees each way from
  the center and back), `follow` (goal ramps as a 50 Hz host sends them,
  at 1, 3 and 10 % of the no-load speed, up and back down), `hold` (two
  three-second holds at the center) and `wide` (two-second holds at the
  middle of each ADC wide code of the board, described below). It is
  written in the servo's own terms: degrees through the stage 3
  pot-table's counts per degree, speeds as shares of the stage 3 run-up's
  no-load speed, the center the middle of the servo's soft range, the
  wide codes from the board's stage 1 result. A rung that runs on the
  pair's position table is planned in the table's linearized counts, the
  counts its goals and the servo's soft range are in under the table:
  degrees through the table's own counts per degree, which is constant
  over the travel, the center the raw middle of the soft range read
  through the table, and a wide code's goals the table's reading of the
  raw goals. A rung on the table and one on the raw pot then center on
  the same place and move the same angles. A servo whose soft range
  cannot fit the set is refused. Each capture writes a `plan.json` beside
  its rows naming every segment and the counts per degree it was planned
  at, and every score in degrees is taken at that scale.
- **The runs**, under `telemetry/<servo>__<board>/scorecard/<label>/`. A
  rung is one build at one label, `run-1` to `run-K` inside it, and
  `rung.json` with the label, the commit, the image sha, a note, the time,
  a hash of the CONFIG it ran on and the goals of the wide holds. Each
  run holds a `servo.json` beside its captures, the servo's constant
  state read off it before the run's first capture, with the rung's
  settings, table and slots on: the bytes of CONFIG and CALIB with every
  field there by name and the writable ones named, the position table's
  state and its points as the servo holds them with their sha1, the ADC
  wide-code slots, the plant stamp and the data state. `rung.json` keeps
  the hash of the first run's, and a later run that reads another state
  is refused, the fields that differ named.
- **The results**, one file per rung,
  `scorecard/results/<servo>__<board>/<label>.json`: every score as the
  median over the rung's runs with their range as the bounds.

Every score reads the pot. How each one is taken is in the docstrings of
`oscnb/scorecard.py`.

The `wide` capture holds the goals the ADC's wide codes make hard. A wide
code stands for itself and the codes it skips
([stage 1, wide-codes](../1-board/README.md#wide-codes)), and a goal
inside that span is never within the deadband of the reading on a
firmware that takes the code as it comes. For each wide code of the
board's result that takes a slot, lowest first, the capture holds the
middle of the skipped codes for two seconds, twice: once after a goal 30
counts above the span, once after one 30 counts below it. Such an
approach goal is held as long as a step of its travel, by the set's own
rule: the hold of the smallest step size at or above the travel from the
goal before it (the center for the first), 1200, 1500 or 2000 ms, and
2000 ms for a travel beyond 60 degrees. So the long move from one code
to the next gets the time of a 60 degree step and the short one back
across a span that of a 2 degree step. The codes are read from the
board's result, none typed. A code is held when both of those goals fit
the soft range with the set's 5 degree margin. A board with no wide-code
result, or with no such code in reach, has no `wide` capture: its rows
are then absent from the result and empty in the ladder, not zero. The
capture runs on every rung, with or without `--wide-codes`, since a rung
without the slots is the one that shows the defect.

Three rows read it, each over the last 500 ms of the holds.
`wide_hold_hunting` is the number of holds driven on more than 2 % of
those rows, the way `landing_tail_driven` counts landings.
`wide_hold_driven_share` is the share of rows driven and
`wide_hold_current_rms_ma` the current rms, both over the tails of all
the holds pooled. A rung recorded before the capture joined the set has
no such rows.

Two rows read the servo's own reports against the pot,
`position_report_error_deg` and `speed_report_error_deg_s`. A report is
polled once a goal item, in the gap of about 5 ms between two bursts of
rows, 2 to 3 ms after the rows stop. On a goal stream the shaft speeds up
and slows down inside every goal period, and on the 1 % ramp it stands
still through that gap, so the speed row grades each poll against the
pot's own speed across the gap it was taken in: the mean pot over the
first millisecond of rows after the next commit less the mean over the
last millisecond before the gap, over the time between the two, along
the travel. The row is the mean over a ramp's follow tail, then the mean
of the up and the down ramp. The gap holds no rows, so the reference is
the pot's mean speed across it, the next commit included. Report and pot
are in the counts the servo ran in, under a position table the
linearized ones. Over a rung's three runs the row's standard error is
about 1.5, 2 and 3 deg/s at 1, 3 and 10 %, so a change shows on it only
past about 3, 4 and 6 deg/s. The row read the report against the ramp's
mean speed before, which graded the poll's place in the period: a
perfect report scored about -6, -3.5 and -3 deg/s there. A rung whose
rows were not kept stays as scored then. Under a position table the
position row reads report and pot in linearized counts, and a goal's pot
counts as quiet within the deadband times the table's local gain there,
the band the servo holds to.

`crawl_swing_deg` reads how far the shaft strays inside one goal period
of the slowest ramp, the span of the pot about the period's own line, at
the widest twentieth of the periods. A shaft that keeps the ramp's pace
reads the pot's noise there; one that stops and jumps, or swings back
across the play of the train, reads that travel. With `01b`'s gains it
reads 0.4 to 1.0 deg from one sitting to the next, about the play
itself, and 0.25 deg with the hold band at 8 counts: a row for a change
of two times, not of a tenth.

## Running it

The servo on the board and the DPS at its rated voltage, the adapter on
the bus at 3 Mbps, torque off, the production image flashed. The runner
never touches the supply and never prompts; it refuses to start otherwise.

```
uv run python -m oscnb.scorecard run --label 00-baseline --fw <commit> --image-sha <sha1> [--runs 3] [--note TEXT] [--settings FILE] [--pos-table] [--wide-codes]
uv run python -m oscnb.scorecard ab --a-label L --a-fw REV --a-image PATH [--a-settings FILE] [--a-pos-table] [--a-wide-codes] [--a-note TEXT] --b-label L --b-fw REV --b-image PATH [--b-settings FILE] [--b-pos-table] [--b-wide-codes] [--b-note TEXT] [--rounds 3]
uv run python -m oscnb.scorecard score 00-baseline
uv run python -m oscnb.scorecard table
```

A rung of settings runs on a file of CONFIG fields, `{"fields": {...}}`,
passed as `--settings`: the fields are written volatile and stamped
(`oscnb/settings.py`), and a reboot brings the stored set back when the
runs end. The pot's position table in the servo's format is built and
graded offline by `oscnb/postable.py` into [calibration/](calibration).
It is built from the stage 4 lookup, which is fitted on pot readings
that carry the board's ADC wide-code step
([stage 1, wide-codes](../1-board/README.md#wide-codes)), so the table
holds the pot alone. Its grades read the pot the same way: the camera
sitting's frames loaded with the step on, and a raw code read through
the table at the code plus its slot, the step ahead of the table.
`build` writes into the table file the sha1 of the slots the lookup was
fitted on, from the wide-code result the stage 4 result names among the
results it read, and refuses when that file is no longer the one read.
A stage 4 result written before results named what they read gives a
table file that says the slots are not recorded, which `build` and
`check` print.
`uv run python -m oscnb.postable install` writes the table to the servo
volatile and reads every page back. A rung passed `--pos-table` runs on
the pair's table: installed after the settings, one stamp over both, the
file and its sha1 kept in `rung.json`, and the same reboot dropping it at
the end. `score` reads such a rung's raw pot through the table its
`rung.json` names, and refuses a capture with a live table and no table
named.

A rung passed `--wide-codes` (on `ab`, `--a-wide-codes` or
`--b-wide-codes`) runs on the board's ADC wide-code slots, found once per
board ([stage 1, wide-codes](../1-board/README.md#wide-codes)): written
after the settings and the table, outside the stamp, the board's result
file and its sha1 kept in `rung.json`, and the same reboot dropping them
at the end. `score` reads such a rung's pot as its firmware did: a row
at a wide code is read as the middle of the span the code stands for,
with the slots of the result file `rung.json` names, and the raw code is
kept beside it. Under a live position table that step comes first and
the table is read at the stepped value. A result file that is no longer
the one recorded is refused. A rung that names no slots is read as
before.

The same servo drifts within a session, so a candidate and its control
are best recorded together. `ab` records two rungs at once, one run of
each per round, A then B, for `--rounds` rounds. Each side names its
image file, and the image sha in its `rung.json` is that file's sha1.
Before a side's run the servo is flashed with its image whenever it
holds the other side's, and before the first run always, and nothing
runs until `osc status` reads the stamp as the live set; two sides on
one image file, a comparison of settings alone, flash once. A side's
`--settings` are applied around its own runs only. Each side comes out
a rung like any other, for `score` and `table`.

`table` prints the ladder, one column per rung in label order, with a
last column of other people's figures for stock smart servos (STS3215,
LX-16A) where a row has one, for scale only.

## The constant set

The constants a servo is to hold are one generated file per pair,
`calibration/<servo>__<board>.constants.json`, written and read by
`oscnb/constants.py`: every CONFIG and CALIB field a host may write, the
bus id, rate and deadline and the plant stamp left out, each with its
value as `osc dump` prints it, its class and where the value came from;
the ADC wide-code slots; the position table's state and the sha1 of its
points. The file is never typed. The same inputs give the same bytes,
and a file whose hashes are not those of its content is refused.

A field's class says what stands behind its value. The first set of a
servo that already runs holds every field as `adopted`: the value the
servo held before the set existed, taken from a record of its state
(the `servo.json` of a scorecard run) and named by that record's sha1,
so it has no source in the notebooks. Every verb prints that count, the
fields with no source, and the count of adopted fields whose value is
per position count, which were fitted at a scale nothing recorded. The
other classes are in the module's docstring.

Three hashes close the file: `protection` over the values of the
protection fields (the limits, the fault thresholds, the NTC curve and
the thermal model), `plant` over every other value with the table and
the slots, and `set` over the two.

```
uv run python -m oscnb.constants adopt --from SERVO_JSON [--origin TEXT]
uv run python -m oscnb.constants status
uv run python -m oscnb.constants check
uv run python -m oscnb.constants install [--protection NAME ...] [--loosen NAME ...]
```

`adopt` writes the first set of a servo that already runs, and only the
first. `status` is offline: the sha1 of every file the set read against
the file now, a moved one named with the fields that list it. `check`
needs the bus and changes nothing the servo holds: it reads the servo's
state the way a run records it and prints a line for every field of the
set, the table and the slots where the servo holds something else, then
the writable fields the servo has outside the set. It exits 1 on a
difference and refuses a servo whose data state is not clean.

`install` makes the servo equal the set, volatile, torque off, and
writes only what differs: the slots, the stops, the table (a LIVE table
from the pair's table file, which must hold the set's points, be built
against the set's stops and name no other slots than the set's as the
ones its lookup was fitted on; a file that records none is said and
taken), then the other fields in table order,
each read back, a write the servo refuses tried once more after the
rest. It then reads the whole state back, and only when that is the set
does it stamp it, no save, and require the data state clean. A servo
that already holds the set gets no write. Before anything is written it
refuses a set with a moved input file, a field the servo lets no host
write, and a protection change that is not named: a protection field
the install would change is passed as `--protection NAME`, and again as
`--loosen NAME` when the set's value protects less than the one on the
servo. Which way protects less is one table by field name in the
module; a field no one direction says it for (the stall policy, the NTC
curve, the heat time constants, the two where 0 means off) always needs
both. A refusal after the first write reboots the servo, which returns
its saved state. The scorecard's own `--settings`, `--pos-table` and
`--wide-codes` do not go through the set yet.

## The ladder

On mg90-d with board dev-v006-2A-D, a free horn. From
`12a-pos-table-live` on, the rungs run on the pair's position table,
passed `--pos-table`. From `13a-wide-step-on-table` on, the rows of a
rung on the table are in degrees of the table's own scale. A rung is
kept when the
rows it aims at gain by more than the baseline's own spread over its three
runs and no other row loses. A rung that is not kept leaves its result
file, its settings file and one line here, not its rows.

Kept:

- `00-baseline`: the servo's stored settings. Its rows are those of the
  set stored on the servo when it was recorded.
- `01b-gains-3hz-50hz`: softer loop gains, the position gain an eighth and
  the velocity gains a quarter of the stored set. The current of a move
  and of a slow follow falls to between a third and a half, the settle
  times shorten, overshoot and lag do not lose. Open on it: the rest error
  is one count worse (0.25 against 0.21 deg), one hold in three buzzes at
  the edge of the deadband, and its stiffness under a load is not measured
  (free horn only).
- `05a-hold-zone`: `01b`'s settings on firmware fcf2703e, whose hold
  settles on nearness to the goal: a shaft that has stayed within twice
  the deadband of a stopped goal for 64 ms is parked where it is and wakes
  only past that zone. Against the firmware before it, alternated in one
  sitting outside the scorecard: a landing that parks on the band's edge
  and is woken by noise without end, 0 and 1 of 48 small landings against
  5, 2 and 2; a 60 degree landing that oscillates to the end of a 2 s hold
  at a raised speed limit, 0 of 36 against 10 of 36. On the scorecard no
  landing is driven in its last 500 ms (3 of 39 on `zz-control-01b-2`,
  recorded next to it). The cost is the rest error, 0.32 against 0.27
  deg: a landing parked by the zone rests up to 7 counts out. Open on it:
  recorded again an hour later (`zz-control-hold-zone`) 2 landings of 39
  are driven in their tail, one a swing of 17 counts either side of the
  goal, wider than the zone, that runs past a second, and the settle
  times are the control's, so this rung's shorter settle times are not a
  gain. The kick at torque-on that the enable rows read is as common
  on the firmware before it (13 of 48 enables after a move against 9 of
  48). Alternated run by run with the firmware before it
  (`zz-ab-after-hold` against `zz-ab-before-hold`, three rounds): the
  worst drive after an arrival is 218 ms against 1180, no landing is
  driven in its tail against 1 of 13, and the accuracy rows lose, the
  two-sided error 0.37 against 0.23 deg and the rest error 0.36 against
  0.26 deg, each beyond the spread. So the rung trades about 0.1 deg at
  rest for a loop that goes quiet: a trade, held for a decision with the
  test under a load, where the play is taken up.

- `07a-enable-rest`: `01b`'s settings on firmware f5e9ee6f, whose
  torque-on keeps the position estimate and starts the profile on the
  goal when the goal is within the deadband of the estimate and the pot
  sample of the edge within twice the deadband of it. Before it the motor
  was driven for 8 to 90 ms at most torque-ons, the profile running from
  one raw pot sample to a goal a few counts off. Against the firmware
  before it, alternated in one sitting outside the scorecard: 42 of 48
  torque-ons at rest and 46 of 48 after a move are driven, 6 of them
  with duty peaks over 500 counts, against 0 of 96. On the scorecard,
  alternated in two sets of three rounds with the order of the sides
  swapped (`zz-ab-after-enable-rest` against `zz-ab-before-enable-rest`,
  this rung against `zz-ab-before-enable-rest-2`): the drive after
  torque-on is 0 ms in all six runs against 9.7 and 32.9 ms, the kick is
  one count (0.05 deg) against 0.11 and 0.48 deg, and no other row moves
  beyond the spread between the two sets. Not measured: a shaft moved by
  hand with the torque off, the case the change leaves on the old path.
- `09a-wide-codes`: `01b`'s settings on firmware c15722d6, which reads a
  pot sample at one of the ADC's wide codes as the middle of the span
  that code stands for, on the board's slots
  ([stage 1](../1-board/README.md#wide-codes)). Before it a goal inside
  a wide code is never within the deadband of the reading, and the loop
  hunts across the code without end. Against the firmware before it,
  alternated in one sitting outside the scorecard, nine holds of 2 s a
  run at goals inside the codes at 1023, 2047 and 2559: 10 of 27 holds
  are still driven in their last second (8 of 12 at 1023, on 80 % of the
  rows at 250 to 340 mA rms), against 0 of 27. On the scorecard,
  alternated in two sets of three rounds with the order of the sides
  swapped (this rung against `zz-ab-before-wide-codes`,
  `zz-ab-after-wide-codes-2` against `zz-ab-before-wide-codes-2`): the
  speed report on the 1 % ramp reads -5.8 and -5.7 deg/s against -7.2
  and -7.4, and no other row moves the same way beyond the spread in
  both sets. Those speed report figures were read under the row's
  earlier definition, the ramp's mean speed as the reference. The set
  had no hold at a wide-code goal when the rung was recorded, so the
  rung's own gain is not one of its rows. Its rows are
  read through the board's slots, a stuck code as the middle of its
  span, where `zz-ab-after-wide-codes-2`, whose rows were not kept,
  stays as scored on the raw pot.
- `10a-adc-all-channels`: `01b`'s settings on firmware 97f819c9, on the
  board's slots. It reads a sample at one of the ADC's wide codes as the
  middle of the span that code stands for on every channel it consumes,
  not the pot alone: the drive window's shunt sample and both window
  taps, the trough shunt sample the zero is tracked from, the rail and
  the temperature channel. Alternated with firmware 194bf988 in two sets
  of three rounds with the order of the sides swapped (this rung against
  `zz-ab-before-adc-all`, `zz-ab-adc-all-2` against
  `zz-ab-before-adc-all-2`): the crawl's speed ripple reads 22.4 and
  25.2 % against 42.1 and 33.4, the ranges of three runs apart in both
  sets, in the second by a hair (up to 29.91 against 29.93 and up), and
  no other row moves the same way beyond the spread in both sets. The
  `wide` capture is new with these sets: eight holds a run, at the
  middle of the four wide codes of the board that take a slot. As the
  median of three runs and their range, 0 [0, 0] and 0 [0, 2] holds
  hunt against 0 [0, 1] and 1 [0, 1]. Every hold that hunts on the
  slots, on either firmware, is at code 2559: five holds over the twelve
  runs, two on 97f819c9 in one run and three on 194bf988 in three runs,
  three of the five approached from below. That is an open point about
  that code's slot, not a result of the change.
  `zz-ab-before-adc-all-nomap` is firmware 194bf988 with no slots
  installed, recorded right after: 3 [3, 4] of 8 holds hunt, on 0.31 of
  the tails' rows at 114 mA rms, both holds at 1023 in every run, the
  hold at 2559 approached from below in every run and the one from
  above in one, and none at 511 or 2047. It is what the `wide` rows read
  without the correction. Open on it: all four sides of the two sets
  read the 20 degree settle at the length of its hold (1480 to 1500 ms),
  2 or 3 landings of 13 driven in their tail and a worst drive after an
  arrival of 1220 ms, against 438 ms, 0 and 227 ms on `09a`. The four
  `zz-rev-ikp` sides, recorded before them on `09a`'s firmware, read
  1480 to 1500 ms, 1 to 4 landings and 1210 to 1230 ms.
- `11a-first-reading`: `01b`'s settings on firmware 7ef0554f, on the
  board's slots. The shunt is read only on a drive pulse at least as
  wide as the current window's floor, 64 ticks. A commanded pulse too
  narrow for the bridge to put out leaves no pulse, and the first window
  read after one reads far over the winding current, about 145 counts
  against 63 inside a train of pulses, after a command of 1 to 15 ticks.
  On the current loop's proportional gain that one reading steps the
  drive across zero. Firmware 7ef0554f does not feed that reading to its
  current loop, which is fed zero in its place as under the floor. The
  trip, the stall check and the limiter still see it. Alternated with
  firmware 97f819c9 in two sets of three rounds with the order of the
  sides swapped (this rung against `zz-ab-before-first-reading`,
  `zz-ab-first-reading-2` against `zz-ab-before-first-reading-2`), the
  duty reverses less often on every follow and on the 2 and 60 degree
  moves, the ranges of three runs apart in both sets: on the follow 146
  and 156 per second against 278 and 270 at 1 %, 225 and 232 against
  302 and 313 at 3 %, 452 and 435 against 598 and 594 at 10 %, and on
  the moves 335 and 294 against 467 and 537 at 2 degrees, 556 and 582
  against 669 and 632 at 60. In the second set two of them are apart by
  a hair (up to 519 against 520 and up at 10 %, up to 623.3 against
  623.9 and up at 60 degrees). The 20 degree move reads 563 and 485
  against 815 and 535 with the ranges overlapping. No other row moves
  the same way beyond the spread in both sets. Counted on the follow
  captures' rows, outside the scorecard's rows, over all three runs of
  each side: a row of 64 to 79 ticks, where the shunt is read, that
  follows a row of 1 to 15 ticks in the same direction is followed by a
  row driving the other way in 1 of 550 and 1 of 553 cases against 628
  of 668 and 555 of 607 on 97f819c9, and after a row of 16 to 26 ticks
  in 3 of 2349 and 8 of 2323 against 841 of 1786 and 886 of 1757. The
  gain is smaller than the cut the halved current gain showed
  (`zz-rev-ikp-half`, 86 to 116 per second at 1 and 3 %). What remains
  of the reversal count is not the first reading. Readings the loop
  takes at references near zero are a candidate, not measured here.
- `12a-pos-table-live`: `01b`'s settings on firmware 7ef0554f, on the
  board's slots, with the pair's position table live. The pot's code is
  read as the angle the stage 4 lookup gives it
  ([stage 4, position](../4-estimators/README.md#position)), through the
  table built and installed as described under [Running it](#running-it).
  It is the table of `03a-pos-table` rebuilt from a lookup fitted on pot
  readings that carry the board's wide-code step, on a firmware that
  corrects the wide codes. The table gives the shaft's true angle, as
  graded against the camera in stage 4, and the raw pot does not, so the
  rung is kept as the correct reading, not on its rows. Its rows are
  mixed and are recorded as such, the losses as open points. Alternated
  with the same build, settings and slots on the raw pot, with no table
  live, in two sets of three rounds with the order of the sides swapped
  (this rung against `zz-ab-before-pos-table-live`,
  `zz-ab-pos-table-live-2` against `zz-ab-before-pos-table-live-2`), the
  table side reads better in both sets, the ranges of three runs apart,
  on three rows of the 60 degree move: its current 117 and 121 mA
  against 132 and 134, its duty 21.0 and 21.6 % against 22.8 and 23.7,
  and its reversals 499 and 491 per second against 567 and 588, in the
  first set by a hair (up to 531.6 against 532.8 and up). It reads
  better the same way on the reversals of the 3 % follow, 223 and 207
  per second against 244 and 241, and on the current and duty of the
  10 % follow, 137 and 149 mA against 151 and 165, 22.5 and 24.1 %
  against 24.7 and 26.5. It reads worse in both sets, the ranges apart,
  on the follow error sd at 1 %, 0.133 and 0.135 deg against 0.111 and
  0.116. In one set only it reads worse on the 2 degree move's
  overshoot, current and duty in the second (0.658 against 0.529 deg,
  167 against 143 mA, 21.8 against 19.6 %), on the 1 % follow's
  current, duty and reversals in the first (78.7 against 72.1 mA, 9.29
  against 8.88 %, 168 against 157 per second) and on the enable kick in
  the first (0.086 against 0.053 deg). In one set only it reads better
  on the 60 degree settle and the speed ripple of a move in the first
  (842 against 896 ms, 32.3 against 36.3 %) and on the lag of the 10 %
  ramp in the second (1.98 against 2.02 deg). The landing rows read
  worse by the median in both sets with the ranges overlapping: the 20
  degree settle 1092 and 1394 ms against 582 and 668, the landings
  driven in their tail 1 and 1 of 13 against 0 and 0, and the worst
  drive after an arrival 870 and 1168 ms against 423 and 620. No other
  row moves beyond the spread in either set.
  A row in degrees or counts is in the table's counts on the table side
  and in raw pot counts on the other, both taken to degrees through the
  same stage 3 counts per degree. Where the pot's local scale differs
  from its mean, a position error or a jitter is then not the same
  quantity on the two sides. The hold jitter reads 1.04 and 1.04 counts
  against 1.47 and 1.51, the ranges apart, and the hold sits where the
  pot is steep, so that row is not a gain. The follow error sd is a row
  of the same kind. The table side was planned like the other, at the
  stage 3 counts per degree and on the middle of the soft range in
  linearized counts, before a rung on the table was planned at the
  table's own scale on the raw center, so its moves and ramps are not
  the angles and speeds asked, its center is not the raw center and its
  wide holds sit off the codes they name; a later rung on the table runs
  another test, and its rows compare with this rung's only with that in
  mind. The position report error reads 0.057 deg against 0.057 in the
  first set, the ranges overlapping; the table side of the second set
  has no value, its rows not kept. The speed report on the 10 % ramp
  changes sign, -1.09 and -2.70 deg/s against 0.57 and 7.59, the ranges
  apart in both sets, figures read
  under the row's earlier definition, the ramp's mean speed as the
  reference. Open on it: the follow error sd at 1 %, the one-set losses
  on the 2 degree move, the 1 % follow and the enable kick, the landing
  rows.
- `13a-wide-step-on-table`: `01b`'s settings on firmware f933a4a7, on
  the pair's position table and the board's slots. Under a live table
  f933a4a7 reads a pot sample at a stuck code at the middle of the span
  the code stands for, in the table interval that holds that middle,
  where 7ef0554f scaled the step by the stuck code's own interval. The
  rung is also the first planned at the table's own scale on the raw
  center read through the table, as the test set is described above,
  and the first that records the servo's constant state beside each run
  (`servo.json`). A later rung on the table compares with this one and
  not with `12a`, and the rung is kept as that reference, not on its
  rows. Alternated with firmware 7ef0554f on the same settings, table
  and slots in two sets of three rounds with the order of the sides
  swapped (this rung against `zz-ab-before-wide-step`,
  `zz-ab-wide-step-2` against `zz-ab-before-wide-step-2`), no row moves
  the same way beyond the spread in both sets.
  Counted on the follow captures' rows, outside the scorecard's rows:
  the current of the 1 % ramps over the rows whose raw pot reads 2047
  to 2079, stuck code 2047 and the 32 codes above it, against the rows
  at raw 1960 to 2160 outside that stretch, each the rms of the rows
  where the shunt was read, per run, as the mean of three runs and
  their range. The stretch reads 112 [103, 126] mA against 87 on this
  rung and 120 [96, 146] against 94 on `zz-ab-wide-step-2`, and 122
  [114, 134] against 87 and 137 [126, 153] against 90 on the two
  control sides. Its excess over the neighbors is lower on f933a4a7 in
  both sets, 25 and 26 mA against 35 and 47, with the ranges of the
  stretch's current overlapping in both.
  The `wide` capture of these sets holds at the table's reading of the
  raw goals, and `rung.json` keeps both, so its holds sit on the codes
  they name, which `12a`'s did not. As the median of three runs and
  their range, 0 [0, 1] and 2 [1, 2] holds of 8 hunt on f933a4a7
  against 0 [0, 1] and 1 [1, 1]. Every hold that hunts, on either
  firmware, is at code 2559: ten holds over the twelve runs, six on
  f933a4a7 in four runs and four on 7ef0554f in four runs, eight of the
  ten approached from above. One of the ten is driven on 0.05 of its
  tail's rows, the other nine on 0.69 to 0.75.
  Open on it, about one place and not a result of the change: the 20
  degree landing above the center. Its goal, 2319 in the table's
  counts, lies between raw 2308 and 2309, above ADC code 2303, which
  the board's result lists as holding through two skipped codes, 2304
  and 2305, without a slot, so both firmwares read it the same way. The
  landing settles in all three runs of `zz-ab-before-wide-step` and
  hunts in the nine other runs of the four sides, this rung's among
  them. As the median of three runs, the 20 degree overshoot reads 0.121
  deg and the settle 301 ms on that control side, against 0.745, 0.795
  and 0.795 deg and 1271, 1497 and 1492 ms on this rung,
  `zz-ab-wide-step-2` and `zz-ab-before-wide-step-2`. No landing of 13
  is driven in its tail there against 1, 1 and 1, and the worst drive
  after an arrival is 231 ms against 1204, 1204 and 1206. In the nine
  runs that landing is driven on 0.38 to 0.77 of its last 500 ms, the
  raw pot spanning 2285 to 2329 there. One run of this rung has a
  second landing driven in its tail, the 60 degree return to the center
  from above.

Tried, not kept:

- `01a-gains-6hz-100hz`, the position gain a quarter and the velocity
  gains half: over the stored set, under `01b` on every current row, and
  one hold in three buzzes. Recorded before the capture tool turned torque
  on at the servo's own estimate, so its enable kick is not comparable.
- `01c-gains-rule`, the set a gain rule over the measured plant gives,
  between `01a` and `01b`: the slow follow's current lands where the rule
  predicted (101 mA against 80 to 119), and it loses to `01b` on settle,
  overshoot and current.
- `02a-limits-half`, `01b` with the speed limit at half the no-load speed
  and the acceleration limit five times the stored one: the 60 degree move
  settles in 258 ms against 811, the 20 degree landings oscillate and
  never settle, and the currents are back at the stored set's. Recorded
  before the same tool fix.
- `02b-speed-half`, `01b` with the speed limit alone raised: the 60 degree
  move settles in 438 ms against 811, and its landing on the way down
  oscillates about the goal in all three runs (landing buzz 0.06 against
  0.007).
- `02c-speed-half-accel-40`, `02b` with the acceleration limit 2.7 times
  the stored one: 271 ms, the same landing oscillates, and one hold in
  three buzzes.
- `01d-gains-band-8` and `02d-speed-half-band-8`, `01b` and `02b` with the
  hold band at 8 counts in place of 4, over half the play between the
  motor and the pot: the oscillation of `02b`'s landing is gone in all
  three runs (landing buzz 0.001 against 0.06) and its 60 degree move
  settles in 390 ms, and the rest error grows to 0.42 deg against 0.25
  and the two-sided error to 0.5 to 0.6 deg against 0.3. A trade of
  accuracy for speed and quiet, not a gain: held for a decision, and for
  the same test under a load, where the play is taken up.
- `03a-pos-table`, `01b` on the pair's position table as built before
  the lookup carried the board's wide-code step: the rest error
  and the two-sided error fall (0.19 and 0.15 deg against 0.25 and 0.30),
  the speed ripple of a move does not change (46.5 %), and the crawl's
  ripple, the overshoot and the follow's current are worse. Every score
  here reads the pot, through the table once it is live, so what the
  table buys in true angle needs the camera.
- `04a-hold-settle`, firmware 0bf126a7 on `01b`'s settings: against the
  control below, no row gains beyond the spread (2 landings of 45 buzz
  against 3).
- `02e-speed-quarter` and `02f-speed-third`, `01b` with the speed limit
  at a quarter and a third of the no-load speed, on the firmware before
  `05a`: the 60 degree move settles in 466 and 417 ms against 1210 on
  `zz-control-01b-2`, and the landing at one goal oscillates to the end
  of its hold in two runs of three (landing buzz 0.06 against 0.016).
- `06a-hold-zone-speed-third` and `06b-hold-zone-speed-half`, `05a` with
  the speed limit at a third and a half: against `zz-control-hold-zone`,
  recorded between them, the 60 degree move settles in 523 and 514 ms
  against 1290, its current is 226 and 242 mA against 151, its overshoot
  1.16 and 0.53 deg against 0.63, and the 20 degree settle is 294 and 877
  ms against 595; 3 and 2 landings of 39 are driven in their tail against
  2. The landing that blocked `02b` no longer oscillates to the end. A
  trade of current and overshoot for speed, in a setting: held for a
  decision, with the same test under a load.

- `zz-ab-after-pot-speed` and its three siblings, an experiment build
  of f5e9ee6f whose velocity loop always reads the pot observer's speed,
  never the back-EMF estimate, alternated with f5e9ee6f: on `01b`'s
  settings, twice, the order of the sides swapped, the duty reverses a
  fifth to a third less often and the lag of the fast follow falls (1.68
  and 1.76 deg against 1.94 and 2.00), and the landings lose, the 60
  degree overshoot 0.85 and 0.85 deg against 0.63 and 0.16 and the
  median drive after an arrival longer in two sets of three; at `02b`'s
  speed limit (`-half`) the 60 degree move settles in 652 ms against
  453. On `01a`'s stiffer gains (`-01a`) the currents fall by a third or
  more, the 60 degree move 132 mA against 219 and the fast follow 156
  against 253, to the level of `01b` on f5e9ee6f. So the back-EMF
  estimate damps the end of a large move and is what makes a stiffer
  loop cost current: not a rung, a pointer at how that estimate enters
  the loop.
- `08a-friction-measured` and `zz-ab-after-friction-2`, `01b` with the
  Coulomb friction term at the stage 3 running friction (54 counts in
  place of the stored 100), alternated with `zz-ab-before-friction` and
  `zz-ab-before-friction-2`, the order of the sides swapped for the
  second pair: the slow follow it was aimed at does not gain (current
  72 and 98 mA against 77 and 100, error sd 0.13 and 0.18 deg against
  0.14 and 0.20, the crawl's ripple 34 and 45 % against 36 and 27), the
  lag grows on the 1 % and the 10 % ramp in both pairs (0.086 and 0.081
  deg against 0.025 and 0.035; 2.04 and 2.01 against 1.98 and 1.96),
  and the median drive after an arrival is shorter in both (27 and 40 ms
  against 62 and 87) with the worst one longer. The first pair's gain on
  the 2 degree move (overshoot 0.16 deg against 0.42, current 115 mA
  against 146) is gone in the second, where both sides' 2 degree rows
  read worse than either side of the first: those rows move more
  between two sittings an hour apart than between the two settings.

The controls, and what they say about reading the ladder:

- `zz-control-01b` is `01b` recorded again on the same build a few hours
  into the session. Its follow current at 3 % reads 113 mA against 90,
  its 20 degree settle 664 ms against 362, its 60 degree overshoot 0.21
  deg against 0. The spread of three runs taken back to back is narrower
  than the servo's drift over a session, so a candidate is judged against
  a control recorded next to it in time, not against a rung from hours
  before.
- `zz-control-01b-2` is `01b` once more, recorded next to `05a`, and
  `zz-control-hold-zone` is `05a` again an hour on, between `06a` and
  `06b`. The second shows the same drift on a second firmware: its 2, 20
  and 60 degree settle times read 462, 595 and 1290 ms against `05a`'s
  70, 362 and 842. A settle time is the worst of four moves and one slow
  landing sets it, so one recording of three runs does not carry a settle
  claim either way.
- `zz-ab-before-hold` and `zz-ab-after-hold` are `01b` on the firmware
  before `05a` and on `05a`'s, recorded with `ab`: one run of each per
  round, the image flashed between. This is how a firmware rung and its
  control are recorded from here on.
- `zz-ab-before-enable-rest`, `zz-ab-after-enable-rest` and
  `zz-ab-before-enable-rest-2` are the other three sides of `07a`'s two
  sets. The first set reads the rest error as a loss for the change
  (0.27 against 0.23 deg, the ranges of three runs barely touching), the
  second, the order of the sides swapped, as a gain (0.26 against 0.29):
  the worst of 13 landings a run moves by a count or two between sets on
  one firmware, so one alternated set of three rounds does not carry an
  accuracy claim of that size either.
- `zz-ab-before-wide-codes`, `zz-ab-after-wide-codes-2` and
  `zz-ab-before-wide-codes-2` are the other three sides of `09a`'s two
  sets. The first set reads the 60 degree overshoot as a gain for the
  change (0.21 against 0.63 deg), the second as a loss (0.63 against
  0.26), and the slow follow's current as lower by a fifth with the
  ranges apart only in the first. The speed report on the 3 % ramp
  reads -4.8 and -4.9 deg/s against -4.3 and -4.4, the ranges of three
  runs apart in the second set and overlapping by a hair in the first;
  over the twelve ramps a side, nine
  of the change's sit between -3.3 and -5.3 deg/s, inside the control's
  -3.2 to -5.4, and three read -7 to -9. A ramp's speed report is one
  poll per goal item, so the row carries a shift of that size only as a
  pointer. Those speed report figures were read under the row's earlier
  definition, the ramp's mean speed as the reference.
- `zz-ab-before-adc-all`, `zz-ab-adc-all-2` and `zz-ab-before-adc-all-2`
  are the other three sides of `10a`'s two sets. Beside the crawl's
  ripple, every row whose ranges of three runs come apart does so in one
  set only and overlaps in the other. In the first set the change reads
  lower on the 2 degree move's current (136 against 155 mA), the 60
  degree move's duty (22.4 against 23.1 %, apart by a hair), the lag on
  the 3 % ramp (0.313 against 0.316 deg) and the speed of the 10 % ramp
  (62.5 against 63.8 deg/s). In the second, the order of the sides
  swapped, it reads higher on the 20 degree move's current and duty (258
  against 213 mA, 30.0 against 26.3 %) and the 2 degree move's reversals
  (467 against 296 per second), three rows where that control side reads
  lower than any of the other three sides, higher on the hold jitter
  (1.38 against 1.25 counts) and lower on the reversals of the 1 % ramp
  (338 against 362 per second).
- `zz-ab-before-zero-period`, `zz-ab-after-zero-period`,
  `zz-ab-before-zero-period-2` and `zz-ab-after-zero-period-2` are `01b`
  on the board's slots on firmware c15722d6 and on 194bf988, which feeds
  the shunt zero's tracker one sample a control period where c15722d6
  feeds it one every tick. Two alternated sets of three rounds, the
  order of the sides swapped. No row moves the same way beyond the
  spread in both sets. In the first set alone 194bf988 reads worse on
  the 60 degree settle (984 against 829 ms) and the rest error (0.32
  against 0.27 deg) and slower on the 10 % ramp (61.0 against 61.2
  deg/s); in the second alone its duty reverses more often on the 3 %
  and the 10 % ramp (385 against 352 and 629 against 544 per second).
  The landing rows read worse on 194bf988 by the median in both sets,
  short of that rule: the landings driven in their tail 1 [0, 2] and 1
  [1, 1] of 13 against 0 [0, 0] and 0 [0, 1], the worst drive after an
  arrival 769 and 1221 ms against 584 and 628 with the ranges
  overlapping, and the rest error again in the second set, 0.37 against
  0.30 deg with the ranges overlapping. Their telemetry was not kept.
- `zz-rev-ikp-full`, `zz-rev-ikp-half`, `zz-rev-ikp-full-2` and
  `zz-rev-ikp-half-2` are a diagnostic, not a candidate: `01b` on
  firmware c15722d6 on the board's slots, against the same with
  `i_kp_q88` at 137, half of the stored 275. Two alternated sets of
  three rounds, the order of the sides swapped. Every row that counts
  the duty's reversals falls on the half side, the ranges apart in both
  sets: on the follow 86 and 98 per second against 334 and 318 at 1 %,
  104 and 116 against 313 and 387 at 3 %, 187 and 210 against 561 and
  598 at 10 %, and on the moves 121 and 129 against 378 and 328 at 2
  degrees, 188 and 189 against 584 and 460 at 20, 220 and 223 against
  721 and 580 at 60. No other row moves the same way beyond the spread
  in both sets. The first set alone reads the half side slower on the 2
  degree settle (279 against 107 ms, where the second reads 539 against
  1050 with the ranges overlapping) and higher on the 3 % ramp's current
  and speed (86 against 84 mA, 18.32 against 18.26 deg/s). The second
  set alone reads it better on the 60 degree move's overshoot, settle,
  current and duty (0.21 against 0.95 deg, 845 against 1650 ms, 129
  against 159 mA, 22.0 against 23.2 %) and on the landing buzz share
  (0.07 against 0.17), and worse on the two-sided error (0.417 against
  0.228 deg) and the position report error (0.072 against 0.056 deg). So
  most of the reversal count at these gains follows the current loop's
  proportional gain, in both sets; a halved gain is not kept as a
  setting.
- `zz-ab-before-first-reading`, `zz-ab-first-reading-2` and
  `zz-ab-before-first-reading-2` are the other three sides of `11a`'s
  two sets. Beside the reversal counts, every row whose ranges of three
  runs come apart does so in one set only and overlaps in the other. In
  the first set the change reads lower on the position report error
  (0.051 against 0.103 deg) and higher on the follow error sd at 10 %
  (0.451 against 0.430 deg). In the second, the order of the sides
  swapped, it reads lower on the 2 degree overshoot (0.529 against 0.687
  deg) and the follow error sd at 1 % (0.115 against 0.125 deg) and
  higher on the two-sided error (0.427 against 0.368 deg). The landing
  rows read differently from `10a`'s open point, where every side read
  the 20 degree settle at 1480 to 1500 ms. Here, as the median of three
  runs, the 20 degree settle reads 507 and 833 ms on 7ef0554f against
  367 and 1498 on 97f819c9, the landings driven in their tail 0 and 0 of
  13 against 0 and 1, and the worst drive after an arrival 287 and 586
  ms against 464 and 1206. Recorded a few hours after `10a`'s sets, the
  landings settle again on three of the four sides, on both firmwares.
  `zz-ab-before-first-reading`'s 20 degree settle still reaches 1500 ms
  in one run of three. This is an observation next to `10a`'s open
  point, not explained here.
- `zz-ab-before-pos-table-live`, `zz-ab-pos-table-live-2` and
  `zz-ab-before-pos-table-live-2` are the other three sides of `12a`'s
  two sets. The rows whose ranges of three runs come apart in one set
  only are listed under `12a`. The two control sides are `11a`'s build,
  settings and slots on the raw pot, the same set as `11a`'s own two
  sides, recorded about half an hour after them. The first reads inside
  the ranges of `11a`'s sides on every row. The second reads higher than
  both of them, the ranges apart, on the 10 % follow's current, duty and
  error sd (164.8 against 139.9 and 149.7 mA, 26.5 against 23.1 and
  24.4 %, 0.473 against 0.451 and 0.461 deg, the last by a hair), on the
  3 % follow's current, duty and lag (89.3 against 84.9 and 87.7 mA,
  12.8 against 11.6 and 12.3 %, 0.376 against 0.340 and 0.352 deg) and
  on the 60 degree move's duty (23.7 against 22.2 and 23.0 %). The table
  side of that set reads 149.4 mA and 24.1 % on the 10 % follow, inside
  the ranges of `11a`'s sides, so the second set's gain on that follow
  is measured against a control that reads above them. The speed report
  on the 10 % ramp reads 0.57 and 7.59 deg/s on the two controls, -2.71
  and -1.99 on `11a`'s sides with the ranges overlapping, and -1.09 and
  -2.70 on the table sides, read under the row's earlier definition, the
  ramp's mean speed as the reference. This is an observation next to
  `12a`'s rows, not explained here.
- `zz-ab-before-wide-step`, `zz-ab-wide-step-2` and
  `zz-ab-before-wide-step-2` are the other three sides of `13a`'s two
  sets. Every row but one whose ranges of three runs come apart does so
  in one set only and overlaps in the other. In the first set f933a4a7
  reads higher on the 20 degree move's overshoot, settle, current and
  duty (0.745 against 0.121 deg, 1271 against 301 ms, 197 against 124
  mA, 25.3 against 21.5 %) and on the landing rows (the landing buzz
  share 0.055 against 0, the landings driven in their tail 1 of 13
  against 0, the worst drive after an arrival 1204 against 231 ms).
  Each of those rows takes in the landing described under `13a`, which
  hunts on both sides of the second set, where the same rows read 0.795
  against 0.795 deg, 1497 against 1492 ms, 243 against 240 mA, 29.0
  against 28.6 %, 0.058 against 0.057, 1 against 1 and 1204 against
  1206 ms. In the second set, the order of the sides swapped, f933a4a7
  reads lower on the 2 degree move's reversals (270 against 362 per
  second) and on the speed report of the 3 % ramp (-1.83 against -0.61
  deg/s), and higher on the 60 degree settle (906 against 900 ms). The
  one row whose ranges come apart in both sets does so in opposite
  directions: the 20 degree move's reversals read 421 against 610 per
  second in the first set and 459 against 406 in the second.
