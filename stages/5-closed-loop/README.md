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
  a hash of the CONFIG it ran on, the constant set it ran on and the
  goals of the wide holds. Each run holds a `servo.json` beside its
  captures, the servo's constant state read off it before the run's
  first capture, with the rung's set installed: the bytes of CONFIG and
  CALIB with every field there by name and the writable ones named, the
  position table's state and its points as the servo holds them with
  their sha1, the ADC wide-code slots, the plant stamp and the data
  state. A servo that is not the rung's set then does not start the
  run. `rung.json` keeps the hash of the first run's state, and a later
  run that reads another state is refused, the fields that differ
  named. A copy of the set lies beside `rung.json` as `constants.json`.
- **The results**, one file per rung,
  `scorecard/results/<servo>__<board>/<label>.json`: every score as the
  median over the rung's runs with their range as the bounds, and the
  scale its current rows are at.

Every score reads the pot. How each one is taken is in the docstrings of
`oscnb/scorecard.py`.

A current row reads the shunt: a row's raw reading over the capture's
own zero, the median reading of its rows at zero duty, through the
board's measured counts per amp, the stage 1 current-scale result
([stage 1](../1-board/README.md)), 1135.0 on this board. A rung that
names a constant set reads that scale from its set, which carries what
it is scored with; a rung recorded before reads it from the board's
result. The zero is read with the driver awake, as the readings are, so
the in-drive offset stage 1 found over the torque-off zero is not taken
off a second time. `hold_jitter_deg` is the sd of the pot over a hold,
unfiltered, in degrees through the plan's counts per degree like every
degree row, so it reads alike on a rung in raw counts and on one in a
table's counts.

Every result file names the scale its current rows are at, as
`current_scale`: the counts per amp and where it is from. The scorer
read the current at the registry's nominal parts before, 2.4 % over the
measured scale, and the jitter in the counts the rung ran in, as
`hold_jitter_counts`. The rungs whose rows are kept (`00-baseline`,
`01b`, `05a`, `07a` and `09a` to `13a`) are scored again from their
rows and hold the rows as described here. The rows of every other rung
recorded before were not kept, so its result cannot be scored again.
`scorecard carry` multiplied its current rows by nominal over measured
counts per amp, which is what scoring the same rows at the measured
scale gives, and its file says so, as `carried` under `current_scale`.
Its jitter row stays `hold_jitter_counts`, in counts: the counts per
degree of its plan went with its rows. So every result of this pair is
at the one measured scale, and every current quoted from a result below
is at it. A hold jitter is quoted in counts where a side of the
comparison has it only in counts, a kept rung's figure there the one
read in counts when it was recorded. The currents counted outside the
scorecard's rows, named as such where they are quoted, were counted at
the nominal parts and are quoted times the same ratio.

The follow rows read a stream of goals, one `goal:V@20` item each. An
item is the 20 ms burst of rows, which the servo times, and the host's
exchanges around it: a poll before and after, the reads ahead of the
arm, the goal and the arm staged, the commit that starts the burst, and
the servo's drop count read on both sides of it. The plan sizes a
ramp's steps for an item of 25.6 ms (`GOAL_PERIOD_S` in the test set).
The burst is 400 rows in every item. The rest is the host's and is not
constant: it moves an item's time by a millisecond or two from run to
run, so over the runs of the rungs whose rows are kept one plan ran the
10 % ramp at 59.5 to 64.7 deg/s for an asked 62.4, moving with the
host's state and not with the build. `ramp_speed_deg_s` is the speed a ramp ran at:
the goals' travel over the time from its first commit to its last, on
the servo's own tick, which a segment's `tick0` gives to one tick and
which agrees with the host's clock to 0.15 % over a capture.

A rung recorded from here on holds the goals of a ramp to a period. The
period is the test set's `GOAL_PERIOD_S`, which the runner passes to
`osc-lab track` as `--goal-period-ms`. It is the 25.6 ms the earlier
rungs' plans were sized for, so the plans and the goal rate stand: on
the bench pair a goal is staged 2.4 to 3.0 ms ahead of its point at the
median, and over three follow captures of 394 held goals the commits
of goals on time are 512.0 ticks apart with an sd of 0.9 to 1.7 ticks,
where the goals of an earlier rung were 28 ticks apart in sd about a
mean that moved from run to run. In those three captures 13, 3 and 55
goals went out late, the last on a loaded host, the latest by a period
less a tenth of a millisecond, none past a whole period, and each
capture's mean spacing over all its goals was 512.0 ticks. The first goal of a ramp goes out
when the host is ready and starts a grid of that period. For every goal
after it the host stages the goal and the arm, waits, and sends the
commit at the goal's own grid point. The host aims by its own clock, and
each goal's `tick0` says where it landed on the servo's tick, so the
host trims its aim by an eighth of what it reads there. The two clocks'
disagreement is taken out as it grows and never adds up, the time a
commit takes to land is in the trim too, and one landing's scatter moves
the next aim by an eighth of itself. An item also reads less than
before: what it read ahead of every arm is read once a run, the drop
count once an item, and inside a ramp the poll after a burst is the poll
before the next goal. So the host's share is shorter, and the wait takes
up what is left of the period.

A goal the host has not staged by its grid point goes out at once, late,
and the grid stands. The goals after it aim at their own points, so a
stall is made up at the rate the period leaves room for, and a ramp
that makes it up ends when it was due to. A goal a whole period late or
more has missed the points it went past, and the goal after it aims at
the first point still ahead. In `segments.csv`, `slot_wait_ms` is how long before its
point a goal was staged, the room the period left it, `slot_late_ms` how
long after its point its commit went out, and `slots_missed` the points
it went past. A capture's `meta.json` names the period as `goal_period`,
with the goals held to it and those of them late, late being more than
one tick. The result's `follow_late_items` is the number of ramp goals
late in a run, 0 on a run that held its period. A result without that
row is of a rung that ran before the period was held, the kept rungs
`00-baseline` to `13a` among them: its goals went out as soon as the
host was ready, as described above, and its `ramp_speed_deg_s` says what
speed that gave.

`lag_deg` is the shaft behind the goal in force. On the 10 % ramp it
grows with the speed the ramp ran at, 0.7 % of lag for 1 % of speed
across the runs of each kept rung, so a lag in degrees there is
comparable only between runs at one ramp speed. `lag_ms` is the lag as
a time, `lag_deg` over the run's own `ramp_speed_deg_s`, and is the row
to compare on that ramp: across the runs of a rung it scatters by 1.7 %
where the degrees scatter by 1.9 %, and between two rungs it takes out
the difference of their speeds, which reaches 6 %. On the 3 % ramp the
lag in degrees does not move with the speed (the plan fixes the step of
each item, and only the time between items moves), so there `lag_deg`
is the row to compare and `lag_ms` carries the speed's scatter, 3.1 %
against 2.2 %. On the 1 % ramp the lag is a few hundredths of a degree
and scatters by 19 % either way. The rungs
whose rows are kept (`00-baseline`, `01b`, `05a`, `07a` and `09a` to
`13a`) are scored again and hold `lag_ms`; every other result recorded
before has the lag in degrees only, and its `ramp_speed_deg_s` beside
it says what speed that was at.

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
capture runs on every rung, whatever slots its set holds, since a rung
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
rows: 2 to 3 ms after the rows stop on a rung that ran at the host's
pace, right after them on one that held its goals to the period. On a
goal stream the shaft speeds up
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
uv run python -m oscnb.scorecard run --label 00-baseline --fw <commit> --image-sha <sha1> [--runs 3] [--note TEXT] [--set PATH] [--deviation FILE] [--protection NAME ...] [--loosen NAME ...] [--max-load N]
uv run python -m oscnb.scorecard ab --a-label L --a-fw REV --a-image PATH [--a-osc PATH] [--a-set PATH] [--a-deviation FILE] [--a-protection NAME ...] [--a-loosen NAME ...] [--a-note TEXT] --b-label ... [--c-label ...] [--order abc] [--rounds 3] [--max-load N]
uv run python -m oscnb.scorecard score 00-baseline
uv run python -m oscnb.scorecard table
```

A rung runs on a constant set ([the constant set](#the-constant-set)):
the pair's set of record, or the set of `--set`, a file of the repo.
Nothing else puts a constant on the servo for a rung. The set is
installed before the runs, volatile, by the set's own `install`: only
what differs from the servo is written, the whole state is read back,
and it is stamped there and nowhere else. A protection field the
install would change on the servo is named with `--protection`, and
with `--loosen` where it protects less, as for `install`. A reboot
brings the stored state back when the runs end, where the install
wrote anything. The test set is planned from the set: its soft range,
and on a set whose position table is LIVE the table's counts at the
table's scale. `rung.json` names the set by its file, its three hashes
and its fields by class, and protection as an entry of its own, the
hash and what the install moved against the servo's values.

A rung that tries one thing runs on the set with one deviation, the
file of `--deviation`: `{"policy": {...}, "pins": {field: {"value",
"why", "rung"}}, "release": [field, ...]}`, policy entries, pins, holds
to release or any of them together, and a `note`. It is not a list of
raw values. A policy entry stands over
[calibration/policy.json](calibration/policy.json) and goes through
the rules, so what it moves is computed at the set's scale; a pin is a
literal, with its reason and the rung that chose it; a release names
fields the policy file holds, each then bound at what its rule gives
([The constant set](#the-constant-set)). The set of record
is built again with the file and must come out as itself with that one
change, so a set of record that is not what a build of its inputs
gives now is refused, as is a deviation that moves nothing. A
deviation that moves a protection field may move nothing else, and
moves the protection fields of one step only
([The constant set](#the-constant-set)).
`rung.json` names the deviation's file and every field it moved as
[of record, deviated]. The deviation files of the ladder are in
[scorecard/deviations/](scorecard/deviations), for example
`--deviation stages/5-closed-loop/scorecard/deviations/hold-band-8.json`.
A deviation that pins a field the servo's image does not have is
refused by the install, the field named, before anything is written.

The pot's position table in the servo's format is built and
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
volatile and reads every page back, without a stamp. A rung gets the
table from its set, and the board's ADC wide-code slots, found once per
board ([stage 1, wide-codes](../1-board/README.md#wide-codes)), the
same way.

`score` reads a rung's pot as its firmware did. Under a live position
table the raw pot is read through the table, and where the rung's set
holds wide-code slots a row at a wide code is first read as the middle
of the span the code stands for, the raw code kept beside it. The
table and the slots are those of the rung's own record: the set copied
into its folder, and the table's points as the run's `servo.json` holds
them, which must be the set's. A rung recorded before rungs named a set
has no `set` in its `rung.json`: it names there the fields, the table
file and the wide-code result file it ran on, the files by their sha1,
and is read through those files as it was, refused when a file is no
longer the one recorded, or when its capture has a live table and it
names none.

The same servo drifts within a session, so a candidate and its control
are best recorded together. `ab` records two rungs at once, or three
with a side c, one run of each per round for `--rounds` rounds. The
sides take the same flags under `--a-`, `--b-` and `--c-`. `--order`
is the order of the sides within a round, their letters once each,
`abc` when it is left out. Each side names its image file, and the
image sha in its `rung.json` is that file's sha1. Before a side's run
the servo is flashed with its image whenever it holds another one, and
before the first run always. A flash counts when `osc flash` ends with
the image installed and its version confirmed, else the session stops.
No stamp is asked of the image as it boots. What the servo then holds
is verified by the side's install and by the state read before the
run. The install makes the servo the set field by field, stamps it
where the data state is not clean and requires clean, and a servo that
is not the rung's set does not start the run. So an image that boots
with a stamp from before its covered fields changed runs once the
install has stamped it, and the reboot after the run brings back the
state that image boots with. Sides on one image file, a comparison of
constants alone, flash once between them. Each side names its own set
and deviation, installed on its own image around its own runs only, so
an A/B is the set against the set with one thing changed, the one
thing named in the record. Each side comes out a rung like any other,
for `score` and `table`.

A side can name its own `osc` tool with `--a-osc PATH`. The plant
stamp is computed by the tool and not by the servo, so an image whose
stamp covers other fields needs the `osc` built from its own tree, and
the tool of one image cannot bless a set on the other. A side's turn
runs from its flash to the reboot after its run. Every `osc` call in
the turn goes through the side's tool: the flash and the wait for
boot, the state read, the install with its stamp, the reboot. Outside the turn the tool is the one of `OSC` or the PATH
again, and a side that names none uses that one. `osc-lab` records the
captures on every side. `rung.json` names the tool under `tool`, by
the sha1 of its binary and its `--version` line where the tool has
one, and never by a path. A tool that is not found is refused before
anything is flashed, and so is a turn whose tool is no longer the
binary the session began with.

A firmware gate is three arms: the control, the candidate, and the
candidate with one constant changed.

```
uv run python -m oscnb.scorecard ab --rounds 3 --order abc --max-load 4 \
  --a-label 30-main --a-fw <main rev> --a-image <main.bin> --a-osc <main's osc> --a-note "control" \
  --b-label 30-new --b-fw <new rev> --b-image <new.bin> --b-osc <the new tree's osc> \
  --c-label 30-new-one --c-fw <new rev> --c-image <new.bin> --c-osc <the new tree's osc> \
  --c-deviation stages/5-closed-loop/scorecard/deviations/<one>.json
```

Sides b and c share an image, so a round flashes twice. A second set
with the order swapped is a second call with `--order cba` and new
labels.

The telemetry link drops rows on a loaded host, and `osc-lab track`
then aborts the capture with "the burst came back short". That is rows
lost on the link and not a result of the servo. The runner moves the
run's folder under the rung's `lost/` and starts the run again from
its first capture, after the preflight, with the servo read again and
checked against the set. It does this twice at most. Each loss is kept
in `rung.json` under `lost`, with the capture, the tool's reason line
and the host's one minute load. A run that is lost a third time ends
the session with torque off and the servo rebooted onto its stored
state. Any other failure of a capture ends the session at once, as
before. With `--max-load N` a run waits while the host's one minute
load is over N, five minutes at most, then goes on. A wait is kept in
`rung.json` under `load_waits`, with the load before and after and
whether the run went on over the limit. `run` takes `--max-load` and
runs a lost run again the same way.

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

A field's class says what stands behind its value: `measured` a rule
over results of record, `policy` a rule over a general choice, `pinned`
a literal typed for the pair, `held` a value carried while its rule may
not bind. The first set of a servo that already runs holds every field
as `adopted`: the value the servo held before the set existed, taken
from a record of its state (the `servo.json` of a scorecard run) and
named by that record's sha1, so it has no source in the notebooks.
Every verb prints that count, the fields with no source, and the count
of adopted fields whose value is per position count, which were fitted
at a scale nothing recorded.

Three hashes close the file: `protection` over the values of the
protection fields (the limits, the fault thresholds, the NTC curve and
the thermal model), `plant` over every other value with the table and
the slots, and `set` over the two.

The set also carries what a capture of the pair is scored with, under
`scales`: the shunt's counts per amp, the board's stage 1 current-scale
result with its band, read by a rule like a field. It is on no servo,
so it has a hash of its own, `scales`, outside `set`: a scale that
moves leaves what the servo is to hold, and the name a save is kept
under, where they were.

```
uv run python -m oscnb.constants adopt --from SERVO_JSON [--origin TEXT]
uv run python -m oscnb.constants build [--out PATH [--deviation FILE]] [--protection NAME ...]
uv run python -m oscnb.constants status
uv run python -m oscnb.constants check
uv run python -m oscnb.constants install [--protection NAME ...] [--loosen NAME ...]
uv run python -m oscnb.constants save [--protection NAME ...] [--loosen NAME ...]
uv run python -m oscnb.constants restore --from STATE_JSON [--protection NAME ...] [--loosen NAME ...]
```

`status`, `check` and `install` read the pair's set of record, or a
derived set named with `--set PATH` ahead of the verb.

`adopt` writes the first set of a servo that already runs, and only the
first.

`build` is offline. It writes the set from the results of record on top
of the pair's set of record, which it replaces, or to `--out` as a
derived set, and prints what it changes. A field is bound by a rule,
a function in the module, which reads results, the registry
and [calibration/policy.json](calibration/policy.json) through one
reader that records each read: a field's `inputs` are what its rule
read, and the set names every file by its sha1. The first rules are
the ones that define a count and what follows from it directly. The
slots are the board's wide-code result. The table is the pair's table
file, LIVE, which must be built against the set's stops and fitted on
the set's slots. The scale is the table's counts per degree. The angle
map puts the clockwise stop at the policy's angle and the other stop
the travel past it, the travel being the stops' span at that scale, so
a host that reads the map gets the table's counts per degree. The gear
ratio and the gear class are the registry's. The motor block, the
feedforwards and the observer gains have the rules of the table below.
Every other field is carried as it stands, the stops among them, which
have no notebook source yet. `policy.json` holds the choices that are no measurement, in
physical units, one file for every pair; a `<pair>.pins.json` beside
it, where there is one, holds literals typed for one pair, each with
its reason and the rung that chose it, and shows as `pinned`. A rule
whose input is missing does not guess: the field is `held` at the value
the set had, with the missing input named. One build moves one hash,
the plant by default: a protection field is carried, with what its rule
would give beside it, until a build names it with `--protection`, which
then moves nothing of the plant. A held field stands as held in either
build: its value is the one that stood.

The kernel works in counts: tap counts, shunt counts, position counts
and its MEDIUM tick, a tenth of the tick rate. A rule of the motor
block takes a stage 3 quantity there through the board's measured
stage 1 scales (tap counts per volt, shunt counts per amp), the gear
ratio and the scale of the count, so a field per position count is
computed for the set's table and names it. The power of counts per
degree a field carries is declared with its rule, and one test holds
every rule to it.

| field | what its rule gives | from | per count |
|---|---|---|---|
| `r_q12` | the winding resistance, R x tap counts per V / shunt counts per A | the resistance as the policy names it, below | no |
| `recip_ke_q` | 1 / Ke, count/s per tap count | Ke x tap counts per V / counts per motor rad, stage 3 ladder | times |
| `ke_vpc_q` | Ke, tap counts per count/s | the same | over |
| `b_i_q313` | B, the count/s a shunt count adds in a MEDIUM tick | Ke / shunt counts per A / J x counts per motor rad / MEDIUM rate, J the stage 3 run-up's | times |
| `fric_fc_counts` | the Coulomb friction as a current, Tc / Ke x shunt counts per A | the friction result's pair for the firmware's model, pooled | no |
| `fric_fv_q016` | the viscous friction, shunt counts per count/s, b / Ke x shunt counts per A / counts per motor rad | the same | over |
| `j_ff_q88` | the inertia feedforward, 1 / B | B | over |
| `l1_q016` | the observer's position gain, 1 - p^3 | p = exp(-2 pi f / MEDIUM rate), f the policy's `observer_hz` | no |
| `l2_q88` | its speed gain, 1.5 (1 - p)^2 (1 + p) x MEDIUM rate | the same | no |
| `l3_q88` | its disturbance gain, (1 - p)^3 x MEDIUM rate / (2 B) | p and B | over |

Each rule names in the module the firmware function that uses its
field. The feedforward and the three observer gains are the firmware
repo's own synthesis (`ident/src/gains.rs`), and one test holds the
rules to the fields that file's code gives on its own test plant. The
three observer gains are of class `policy`: the bandwidth is a choice.

The resistance is on record four ways in stage 3, each a rule:
`rails_fit`, the fit over three rails with the edge lags at stage 1's
measured ones; `locked_laps`, the laps of the locked horn;
`small_step`, the volts over the current across a small duty step of
a spinning motor, the volts built the firmware's way; `take_back`,
the current such a step takes back as the speed follows, with no
volts in it. `policy.json` names under `rule` the one that binds
`r_q12`, the rails fit, which is the winding's. A field with several rules and none named
is held: the choice is not the tool's. The same four are the rules of
`r_bemf_q12`, the resistance the back-EMF boxcar subtracts on an image
that gives it a field of its own, for which the policy names the small
step's.

`policy.json` holds a field back under `hold`, with the reason. A held
field stays at the value the set had, class `held`, with what its rule
gives beside it as `would_be` and the results the rule read still
listed, so the set shows what the record gives next to what the servo
holds and moves nothing. A held field the set never held stays out of
it, and `build` says so with what its rule gives. The fields of the
motor block whose rule gives another value than the servo holds are
held, each to move on a rung of its own: the back-EMF pair, `r_q12`
and `recip_ke_q`, which a replay reads worse on an estimator without
the on-time term; the Coulomb term, benched alone as
`08a-friction-measured` and not kept; and `b_i_q313`, `j_ff_q88` and
`l3_q88`, whose stored values sit inside their rules' bands.
`r_bemf_q12` is held because no image has the field yet. `l1_q016` and
`l2_q88` bind: their rules give what the servo holds. `fric_fv_q016`
and `ke_vpc_q` bind at what their rules give, 812 and 315 where the
servo held 289 and 355, the two stored values of the block outside
their rule's band: their release was benched on the saved set and
kept (below). A value held from before the
set existed is still counted as at an unknown scale where it is per
count.

A rung releases a hold with a deviation: `"release": [field, ...]`
names held fields, and the derived set binds each at what its rule
gives, `rung.json` showing it as [of record, deviated] like any field
a deviation moves. Nothing else of the policy is restated, and the
other holds stand. A release of a field that is not held is refused,
as is one whose rule has no value to bind. The releases of record are
[motor-model.json](scorecard/deviations/motor-model.json), the model
gain, the inertia feedforward and the observer's disturbance gain, and
[motor-r.json](scorecard/deviations/motor-r.json), `r_q12` alone.

The first release benched was the block outside the back-EMF pair and
the Coulomb term, five fields together, against the saved set of
record: two alternated sets of three runs, the sides' order swapped
for the second, a row moved when the two sides' ranges are apart in
both sets, the fields to bind unless a row was worse on them in both
(`zz-ab-before-motor-rest` against `zz-ab-motor-rest`, and the same
with `-2`). One row was: the 1 % ramp's duty rms, 9.08 [8.97, 9.17]
against 9.47 [9.18, 9.51] % and 10.48 [8.82, 10.55] against 11.65
[11.04, 12.81] %. The five stayed held and the step was split in two
and run three-sided by the same rule (`zz-ab-before-motor-split`,
`zz-ab-motor-feedforward` and `zz-ab-motor-model`, and the same with
`-2`): `fric_fv_q016` and `ke_vpc_q`, the two stored values outside
their rule's band, against `b_i_q313`, `j_ff_q88` and `l3_q88`, the
three inside it. No row was worse on either half in both sets, the
1 % ramp's duty rms among them (9.49 against 9.30 and 9.32 %, 9.65
against 9.74 and 9.14 %), so the first step's row did not repeat on
either half. The first half binds. The second stays held: its values
are inside their bands, so the record does not tell them from the
stored ones, and its reversal rows leaned up, each apart in one set
(the 60 degree move's 498 against 528 a second in the first set, the
1 % ramp's 156 against 186 and the 10 % ramp's 416 against 514 in the
second). The 20 degree landing ran its hold out hunting, a settle
over 1000 ms, in 3 of 6 runs of the first step's released side and
none of its control's, and in the split in 2 of 6 runs of the control
itself, 2 of 6 of the first half and 1 of 6 of the second: it lands on
a stuck code's gap and flips from run to run on the set of record
alone.

Two groups of protection fields have rules, both held, so the set
shows what the record gives beside what protects the servo now and no
limit moves.

The board NTC's curve is the design quadratic of the board's stage 1
`ntc-curve` result ([stage 1](../1-board/README.md)): `ntc_t_ref_cc`,
`ntc_k1_q88` and `ntc_k2_q24` as that result holds them, the
fixed-point quadratic in the count fitted through the part's nominal
beta equation over 20 to 100 C. The thermocouple run graded that
equation and found it inside the meter's class once the rig's two
effects are taken out. The result's `refit_*` curve and its
fitted beta are fits against a thermocouple shown to read low and are
not design constants. `ntc_raw_ref` has no rule and stays as the servo
holds it: the result names a reference count for its refit and none
for its design curve, which the notebook fits about the same count,
2048, and has to publish. The servo holds another curve, the host's
tangent form about 25 C. What each reads on the bench board, through
the kernel's own conversion and with the part's nominal equation as
the truth:

| board at a true | 25 C | 40 C | 60 C | 80 C | 100 C |
|---|---|---|---|---|---|
| the stored curve reads | 25.0 | 40.2 | 57.3 | 68.5 | 75.1 |
| the curve of record reads | 25.8 | 37.8 | 61.8 | 81.6 | 94.4 |

| reaches | the derate at 80 C | the cutoff at 100 C |
|---|---|---|
| the stored curve | at a true 130.3 C | never: it tops out at 84.2 C |
| the curve of record | at a true 78.0 C | at a true 113.8 C |

So the board term of the thermal protection is far looser than its
numbers on the stored curve, and the curve of record brings the derate
to within 2 C of its number and leaves the cutoff 14 C late, which is
the form's own error: the quadratic reads 5.6 C low at a true 100 C,
the end of the range it was fitted over, and falls further behind past
it. Two more things about the curve of
record. Between a true 26.5 and 49.4 C it reads under the stored curve,
by up to 2.6 C at 37.4 C, so over that stretch it protects less. And
it turns at a true 20 C: below that it reads warmer as the board
cools, 37.5 C at a true 0 C where the stored curve reads 4.8 C.

The current limits are each a policy entry in mA, taken to shunt
counts by the board's measured counts per amp, the scale the set
carries. The kernel compares its own current reading against them,
and that reading is over the awake zero, which already holds the
driver's own current, so the in-drive offset of stage 1 is not added.
The entries are the mA the stored counts were typed for: the whole mA
the board's nominal parts take to each stored count.

| field | policy | the servo holds | which is | the rule gives |
|---|---|---|---|---|
| `current_limit_counts` | 500 mA | 554 | 488.1 mA | 568 |
| `i_cont_counts` | 250 mA | 277 | 244.0 mA | 284 |
| `stall_release_counts` | 90 mA | 100 | 88.1 mA | 102 |
| `oc_trip_counts` | 2200 mA | 2439 | 2148.9 mA | 2497 |

Each stored limit is 2.3 % under the current it was typed for, and the
rule's value puts more counts on the same mA, which protects less than
what the servo holds. `stall_tau_trip_counts` is off on the servo and
has no rule.

A deviation moves the protection fields of one step: the words of the
NTC curve are one step and the four current limits another, a
protection field in neither is a step alone, and a release of one
field of a step releases every held field of it, so no curve of two
fits and no limits at two scales are built. The steps of record are
[protection-ntc-curve.json](scorecard/deviations/protection-ntc-curve.json)
and
[protection-current-scale.json](scorecard/deviations/protection-current-scale.json).
The install of either names every field it moves as `--protection`
and again as `--loosen`: the curve's words are judged, as is the
continuous current, and the other limits move up.

The fields in counts of position were typed when the servo ran on the
raw pot, 18.913 counts per degree at mid travel, and are read by a
kernel that runs on the set's count, the table's 20.127 counts per
degree. A count typed then is 6.4 % less speed or angle now. Each has
a policy entry in the unit a person sets it in and a rule that takes
it to counts at the set's scale: a speed in deg/s and an angle in deg
times the counts per degree, an acceleration in deg/s^2 times the
counts per degree over the MEDIUM rate, since the kernel holds it as
the speed one MEDIUM tick adds. The entries are what the stored counts
were typed for at the raw pot's mid travel scale, to four figures.
None is a round number: the stored counts are the firmware's boot
defaults, which are round in counts (1500 count/s, reached in 50 ms;
500 count/s; 400 counts).

| field | the kernel's use | policy | the servo holds | typed for | on the table | the rule gives |
|---|---|---|---|---|---|---|
| `velocity_limit_cps` | the cap of the profile's speed and of the position loop's command, count/s | 79.31 deg/s | 1500 | 79.31 deg/s | 74.53 deg/s | 1596 |
| `accel_limit_q88` | the speed the profile may add in a MEDIUM tick, count/s, Q8.8 | 1586 deg/s^2 | 3840 | 1586 deg/s^2 | 1491 deg/s^2 | 4086 |
| `stall_omega_max_cps` | a press over the continuous current counts toward a stall while the estimate's speed is under this, count/s | 26.44 deg/s | 500 | 26.44 deg/s | 24.84 deg/s | 532 |
| `pos_error_counts` | the profile less the estimate, over this for the error time, faults; counts | 21.15 deg | 400 | 21.15 deg | 19.87 deg | 426 |

All four are held, so no value moves. The two limits of motion are of
the plant and move together on a rung,
[motion-limits.json](scorecard/deviations/motion-limits.json): the
acceleration limit was typed as the speed limit reached in 50 ms. The
two screens are protection fields, each a step alone.
[protection-position-error.json](scorecard/deviations/protection-position-error.json)
puts the screen at 426 counts, which trips later than 400 and protects
less: its install names `pos_error_counts` as `--protection` and as
`--loosen`.
[protection-stall-speed.json](scorecard/deviations/protection-stall-speed.json)
puts the stall speed at 532, so a press is taken as a stall up to a
higher speed, which protects more: its install names
`stall_omega_max_cps` as `--protection` alone.

Three more fields are counts of position and move with no table. The
soft limits are positions: `osc cal` typed them a sixteenth of the
travel inside each stop on a straight line between the stops, and the
table's count is that line, reading the stops as themselves. Their
rule is the policy's `soft_margin_share` of the stops' span inside
each stop, 314 and 3560, the values the servo holds, so they bind and
are of class `policy`. On the raw pot the same counts sat 10.4 and
10.9 deg inside the stops; on the table they are 11.5 deg, what they
were typed for. They keep the train off its stops, the wall that zeroes
the current into a stop and the clamp of a goal, and are not among the
protection fields yet: naming them there moves two hashes of every
set with no value moved, which is a step of its own.

The hold band, `pos_deadband_counts`, and the sensor screen,
`sensor_delta_max`, are in the raw counts of the pot whatever the
table: the kernel takes the band through the table's local gain
itself, and screens the pot sample before any table. So neither was
ever at another scale. The band's rule is the firmware repo's own
(`ident/src/gains.rs`): 2.5 times the pot's noise at rest
([stage 3, pot-table](../3-servo/README.md#pot-table)) and no less than 4
counts, the policy's `hold_band_noise_multiple` and
`hold_band_floor_counts`. This pot's noise is under a count, so the
band sits on the floor, 4, the value the servo holds. The band at 8
counts, over half the train's play, stays a pin of its own deviation.
The sensor screen is the policy's `sensor_delta_max_counts`, 256 raw
counts between two MEDIUM samples, the value the servo holds. Both
bind.

Of the loop gains, two carry a position unit: `v_kp_q88` and
`v_ki_q412` are shunt counts per count/s, so they fall as the counts
per degree rise. They have no rule yet and are the two fields left
with no source at a scale nothing recorded. `p_kp_q88` is count/s per
count, a rate, and carries none, nor do `v_kaw_q412` and the three
gains of the current loop.

`status` is offline: the sha1 of every file the set read against
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
taken; an IDENTITY table as all-zero points, whose COMMIT must land
IDENTITY, a servo whose firmware lands it LIVE being refused as that),
then the other fields in table order,
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
its saved state. No rung is stamped any other way: a scorecard rung runs
on a set installed this way ([Running it](#running-it)), and
`build --deviation FILE --out PATH` writes the derived set a rung with
that deviation runs on, which names the file and the fields it moved
and is never saved.

`save` makes the set what a reboot returns, from the state the servo
has stored. It is a verb of its own and saves the pair's set of record
alone. With the bus untouched it refuses a derived set, a set with a
moved input file and a set file that is not committed as it stands (a
read only `git status` of that one file); then torque must be off. It
reboots the servo, waits for the bus and writes the state the servo
boots with, its table's points in it, to
`telemetry/<pair>/constants/<set hash, 12 characters>/stored.json`; a
later save of the same set keeps its files in a numbered folder inside
that one, so no such record is written over. From there it installs the
set, the same install behind the same gate (`--protection` and
`--loosen` are passed through, so a set that differs from the stored
state in a protection field is refused unless named), and the `check`
after it must be clean. Then the SAVE, a reboot, the wait, the state to
`after.json` beside the record, and its comparison with the set. The
SAVE keeps the servo's CONFIG and CALIB whole, with the slots, the stamp
and the table's points, so what those regions hold outside the set, the
bus id and rate among it, is saved with it. A refusal before the SAVE
leaves flash as it was, and a reboot returns it.

A save is undone from its own record. `restore --from stored.json`
installs the state the servo had stored, volatile, and it is made
permanent again by adopting that record as the pair's set of record
(the present set's file out of the way first, since `adopt` writes a
first set only), committing it and saving it. A servo that does not
come back from the SAVE as the set is reported as that, with what flash
then holds and these steps.

`restore` puts a recorded state back, volatile, from a save's
`stored.json` or any `servo.json` that holds its table's points: it is
the install of the set adopted from the record, the table written from
the record's own points and not from a file, behind the same protection
gate.

## The ladder

On mg90-d with board dev-v006-2A-D, a free horn. From
`12a-pos-table-live` on, the rungs run on the pair's position table.
From `13a-wide-step-on-table` on, the rows of a
rung on the table are in degrees of the table's own scale. A rung is
kept when the
rows it aims at gain by more than the baseline's own spread over its three
runs and no other row loses. A rung that is not kept leaves its result
file and one line here, not its rows. The rungs up to `13a` were
recorded before a rung named a constant set: each ran on a file of
fields, kept in its result file under `settings`, and where its
`rung.json` names them on the table file and the wide-code result. The
pair's set of record holds what `13a` ran on, `01b`'s gains, the table
and the slots, so a rung on it is given `13a`'s goals. Of the fields
those rungs tried, the ones that mean on the set what they meant when
typed are kept as deviation files, the hold band at 8 counts and the
measured friction term; a speed limit, an acceleration limit or a
velocity gain typed in counts at the raw pot's scale is not, since on
the table's count it is another speed or gain.

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
  rows at 244 to 332 mA rms), against 0 of 27. On the scorecard,
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
  the tails' rows at 112 mA rms, both holds at 1023 in every run, the
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
  on three rows of the 60 degree move: its current 114 and 118 mA
  against 129 and 131, its duty 21.0 and 21.6 % against 22.8 and 23.7,
  and its reversals 499 and 491 per second against 567 and 588, in the
  first set by a hair (up to 531.6 against 532.8 and up). It reads
  better the same way on the reversals of the 3 % follow, 223 and 207
  per second against 244 and 241, and on the current and duty of the
  10 % follow, 133 and 146 mA against 147 and 161, 22.5 and 24.1 %
  against 24.7 and 26.5. It reads worse in both sets, the ranges apart,
  on the follow error sd at 1 %, 0.133 and 0.135 deg against 0.111 and
  0.116. In one set only it reads worse on the 2 degree move's
  overshoot, current and duty in the second (0.658 against 0.529 deg,
  163 against 140 mA, 21.8 against 19.6 %), on the 1 % follow's
  current, duty and reversals in the first (76.9 against 70.4 mA, 9.29
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
  their range. The stretch reads 109 [101, 123] mA against 85 on this
  rung and 117 [94, 143] against 92 on `zz-ab-wide-step-2`, and 119
  [111, 131] against 85 and 134 [123, 149] against 88 on the two
  control sides. Its excess over the neighbors is lower on f933a4a7 in
  both sets, 24 and 25 mA against 34 and 46, with the ranges of the
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
  predicted (99 mA against 80 to 119), and it loses to `01b` on settle,
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
  against 1290, its current is 221 and 237 mA against 147, its overshoot
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
  more, the 60 degree move 129 mA against 214 and the fast follow 152
  against 247, to the level of `01b` on f5e9ee6f. So the back-EMF
  estimate damps the end of a large move and is what makes a stiffer
  loop cost current: not a rung, a pointer at how that estimate enters
  the loop.
- `08a-friction-measured` and `zz-ab-after-friction-2`, `01b` with the
  Coulomb friction term at the stage 3 running friction (54 counts in
  place of the stored 100), alternated with `zz-ab-before-friction` and
  `zz-ab-before-friction-2`, the order of the sides swapped for the
  second pair: the slow follow it was aimed at does not gain (current
  70 and 96 mA against 75 and 98, error sd 0.13 and 0.18 deg against
  0.14 and 0.20, the crawl's ripple 34 and 45 % against 36 and 27), the
  lag grows on the 1 % and the 10 % ramp in both pairs (0.086 and 0.081
  deg against 0.025 and 0.035; 2.04 and 2.01 against 1.98 and 1.96),
  and the median drive after an arrival is shorter in both (27 and 40 ms
  against 62 and 87) with the worst one longer. The first pair's gain on
  the 2 degree move (overshoot 0.16 deg against 0.42, current 112 mA
  against 143) is gone in the second, where both sides' 2 degree rows
  read worse than either side of the first: those rows move more
  between two sittings an hour apart than between the two settings.

The controls, and what they say about reading the ladder:

- `zz-control-01b` is `01b` recorded again on the same build a few hours
  into the session. Its follow current at 3 % reads 110 mA against 88,
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
  lower on the 2 degree move's current (133 against 151 mA), the 60
  degree move's duty (22.4 against 23.1 %, apart by a hair), the lag on
  the 3 % ramp (0.313 against 0.316 deg) and the speed of the 10 % ramp
  (62.5 against 63.8 deg/s). In the second, the order of the sides
  swapped, it reads higher on the 20 degree move's current and duty (252
  against 208 mA, 30.0 against 26.3 %) and the 2 degree move's reversals
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
  and speed (84 against 82 mA, 18.32 against 18.26 deg/s). The second
  set alone reads it better on the 60 degree move's overshoot, settle,
  current and duty (0.21 against 0.95 deg, 845 against 1650 ms, 126
  against 156 mA, 22.0 against 23.2 %) and on the landing buzz share
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
  error sd (160.9 against 136.6 and 146.2 mA, 26.5 against 23.1 and
  24.4 %, 0.473 against 0.451 and 0.461 deg, the last by a hair), on the
  3 % follow's current, duty and lag (87.2 against 82.9 and 85.6 mA,
  12.8 against 11.6 and 12.3 %, 0.376 against 0.340 and 0.352 deg) and
  on the 60 degree move's duty (23.7 against 22.2 and 23.0 %). The table
  side of that set reads 145.9 mA and 24.1 % on the 10 % follow, inside
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
  duty (0.745 against 0.121 deg, 1271 against 301 ms, 193 against 121
  mA, 25.3 against 21.5 %) and on the landing rows (the landing buzz
  share 0.055 against 0, the landings driven in their tail 1 of 13
  against 0, the worst drive after an arrival 1204 against 231 ms).
  Each of those rows takes in the landing described under `13a`, which
  hunts on both sides of the second set, where the same rows read 0.795
  against 0.795 deg, 1497 against 1492 ms, 237 against 235 mA, 29.0
  against 28.6 %, 0.058 against 0.057, 1 against 1 and 1204 against
  1206 ms. In the second set, the order of the sides swapped, f933a4a7
  reads lower on the 2 degree move's reversals (270 against 362 per
  second) and on the speed report of the 3 % ramp (-1.83 against -0.61
  deg/s), and higher on the 60 degree settle (906 against 900 ms). The
  one row whose ranges come apart in both sets does so in opposite
  directions: the 20 degree move's reversals read 421 against 610 per
  second in the first set and 459 against 406 in the second.
