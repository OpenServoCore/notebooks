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

- **The test set**, [scorecard/testset.py](scorecard/testset.py). Three
  captures per run, each one `osc-lab track` call: `steps` (the first
  second after torque-on, then moves of 2, 20 and 60 degrees each way from
  the center and back), `follow` (goal ramps as a 50 Hz host sends them,
  at 1, 3 and 10 % of the no-load speed, up and back down) and `hold` (two
  three-second holds at the center). It is written in the servo's own
  terms: degrees through the stage 3 pot-table's counts per degree, speeds
  as shares of the stage 3 run-up's no-load speed, the center the middle
  of the servo's soft range. A servo whose soft range cannot fit it is
  refused. Each capture writes a `plan.json` beside its rows naming every
  segment.
- **The runs**, under `telemetry/<servo>__<board>/scorecard/<label>/`. A
  rung is one build at one label, `run-1` to `run-K` inside it, and
  `rung.json` with the label, the commit, the image sha, a note, the time
  and a hash of the CONFIG it ran on.
- **The results**, one file per rung,
  `scorecard/results/<servo>__<board>/<label>.json`: every score as the
  median over the rung's runs with their range as the bounds.

Every score reads the pot. How each one is taken is in the docstrings of
`oscnb/scorecard.py`.

Two rows read the servo's own reports against the pot,
`position_report_error_deg` and `speed_report_error_deg_s`. A report is
polled once a goal item, in the gap between two bursts of rows, about a
millisecond after the rows stop. On a goal stream the shaft speeds up and
slows down inside every goal period and the poll sits at the slow end of
it, so the speed row reads that swing together with the report's own
error, 5 to 7 deg/s low on the 1 % ramp on every rung with `01b`'s
gains: a row to see a change with, not a grade of the report. The
position row is empty under a position table, where the report is in
linearized counts.

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
uv run python -m oscnb.scorecard ab --a-label L --a-fw REV --a-image PATH [--a-settings FILE] [--a-wide-codes] [--a-note TEXT] --b-label L --b-fw REV --b-image PATH [--b-settings FILE] [--b-wide-codes] [--b-note TEXT] [--rounds 3]
uv run python -m oscnb.scorecard score 00-baseline
uv run python -m oscnb.scorecard table
```

A rung of settings runs on a file of CONFIG fields, `{"fields": {...}}`,
passed as `--settings`: the fields are written volatile and stamped
(`oscnb/settings.py`), and a reboot brings the stored set back when the
runs end. The pot's position table in the servo's format is built and
graded offline by `oscnb/postable.py` into [calibration/](calibration),
and `uv run python -m oscnb.postable install` writes it to the servo
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
at the end.

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

## The ladder

On mg90-d with board dev-v006-2A-D, a free horn. A rung is kept when the
rows it aims at gain by more than the baseline's own spread over its three
runs and no other row loses. A rung that is not kept leaves its result
file, its settings file and one line here, not its rows.

Kept:

- `00-baseline`: the servo's stored settings.
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
- `03a-pos-table`, `01b` on the pair's position table: the rest error
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
