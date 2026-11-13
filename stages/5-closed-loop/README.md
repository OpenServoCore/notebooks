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

## Running it

The servo on the board and the DPS at its rated voltage, the adapter on
the bus at 3 Mbps, torque off, the production image flashed. The runner
never touches the supply and never prompts; it refuses to start otherwise.

```
uv run python -m oscnb.scorecard run --label 00-baseline --fw <commit> --image-sha <sha1> [--runs 3] [--note TEXT] [--settings FILE] [--pos-table]
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

Tried, not kept:

- `01a-gains-6hz-100hz`, the position gain a quarter and the velocity
  gains half: over the stored set, under `01b` on every current row, and
  two holds in three buzz. Recorded before the capture tool turned torque
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
  the stored one: 271 ms, the same landing oscillates, and the holds buzz
  more.
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

One control, and what it says about reading the ladder:

- `zz-control-01b` is `01b` recorded again on the same build a few hours
  into the session. Its follow current at 3 % reads 113 mA against 90,
  its 20 degree settle 664 ms against 362, its 60 degree overshoot 0.21
  deg against 0. The spread of three runs taken back to back is narrower
  than the servo's drift over a session, so a candidate is judged against
  a control recorded next to it in time, not against a rung from hours
  before.
