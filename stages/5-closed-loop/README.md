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
  at, and every score in degrees is taken at that scale. Every move runs
  at the test set's own limits of motion, a speed limit of 74.53 deg/s
  and the acceleration that reaches it in 50 ms, 1490.6 deg/s^2
  (`SPEED_DEG_S`, `ACCEL_DEG_S2`), and not at the set's default (below).
- **The runs**, under `telemetry/<servo>__<board>/scorecard/<label>/`. A
  rung is one build at one label, `run-1` to `run-K` inside it, and
  `rung.json` with the label, the commit, the image sha, a note, the time,
  a hash of the CONFIG it ran on, the constant set it ran on, the limits
  of motion its moves ran at and the
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
  median over the rung's runs with their range as the bounds, the
  scale its current rows are at and the limits of motion its moves ran
  at.

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
count once an item, and inside a ramp there is one poll between two
bursts. That poll is taken once the next goal and its arm are staged,
ahead of the wait, so it sits mid-gap, where the poll before a goal sat
on the rungs before, and the rows that read the servo's reports keep
their meaning. So the host's share is shorter, and the wait takes up
what is left of the period.

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
pace, and behind the next goal's staged writes, mid-gap as well, on one
that holds its goals to the period. On a
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

The moves run at the test's own limits of motion, not at the set's
default speed unless the rung asks for it (below). A step row measures the speed the move ran at as much as
the loop, so a rung at another speed does not compare with the ladder,
and the default speed is the owner's to raise
([the constant set](#the-constant-set)). On the set of record's count,
the table's, the test's 74.53 deg/s and 1490.6 deg/s^2 are the 1500
count/s of `velocity_limit_cps` and the 3840 of `accel_limit_q88`, the
counts the servo held for every rung before the test set named them
that did not move them itself. A rung that ran on the raw pot ran the
same counts at 79.31 deg/s, on the pot's count. The runner takes the
test's two to counts on the rung's set's count by the set's own rules,
installs them over the set with it for the runs and checks the servo
against the set at those counts before each run; the reboot after the
runs brings the stored ones back. A set on another count runs the same
speed at other counts. A rung on a deviation that moves either limit of
motion runs at its set's two, the one it does not move at the set's
value: the limits are what such a side tests. `--motion set` on `run`,
or `--a-motion set` on a side of `ab`, runs the rung at the set's own
default instead, the set installed as it is, to score the speed a user
gets; its step rows are not comparable with the ladder's, and its
record says so with `from` `set`. A deviation that moves either limit
still runs at its own with the flag given, and its record says
`deviation`. `rung.json` and the result name them under `motion`:
`from` (`test set`, `set` or `deviation`), the two as the fields hold
them, and what they are on the set's count in deg/s and deg/s^2, to
five figures.

A result recorded before rungs named their limits of motion has no
`motion`, and its file is not rewritten. The tool reads it at the limits
of its day: the ones its deviation moved (`zz-sweep-motion-200` and
`-400` and the three `zz-ab-400-` sides), else the ones its settings
named (the `02a` to `02f` speed
rungs, `06a`, `06b` and the two `pot-speed-half` A/B rungs), else 1500
count/s and 3840: the counts every set held while the two were held,
and the ones the config in every capture of every kept rung holds. `table` prints
the two first, one row each; `carry` touches only the current rows and
leaves them as they are.

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
A file there that the set of record has overtaken is history. Its
holds are released in the set of record, or the literal it pins is
what the set's rule binds now, so `build --deviation` refuses it, the
field named as not held or the file as moving nothing: `motor-r.json`,
`motor-model.json`, `friction-measured.json` and the seven
`protection-*.json` files, each the release of one protection step
as it was written before the set of record took the step, and the
three files of the limits of motion, `motion-limits.json`,
`motion-200.json` and `motion-400.json`, each a release of the two
holds the set of record let go with the owner's default move speed
([the constant set](#the-constant-set)). Each stays as it was written,
and no rung runs on one again. Five are files a rung of record ran,
`motor-model.json` two rungs, `motor-r.json` `zz-ab-r-measured`,
`protection-ntc-curve.json` `zz-ab-ntc-pinned`, `motion-200.json`
`zz-sweep-motion-200` and `motion-400.json` `zz-sweep-motion-400` and
the three sides of the speed blend's session at 400 deg/s
(`zz-ab-400-`, [the ladder](#the-ladder)), and a result that names one
keeps its sha1 and the fields it moved. What
`motion-200.json` moved is the set of record's own default now, so
that rung ran the set of record itself, every hash alike; the 400
deg/s point of `motion-400.json` is no value of the set, and its rungs
keep it as the sides they ran. Moving a speed with a deviation now takes
a file of policy entries alone, with no hold to release:
`motion-400-policy.json` was the 400 deg/s point on the set of record;
the ceiling's count rounds down now, 8050, and the point's 8051 is one
over it, so its build is refused, the field named.
One test names
the fourteen and holds every other file to a build.
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

A field's class says where its value comes from. There are four
sources: `measured`, a rule over results of record, a measurement on
this pair; `bench`, a recorded bench result on this servo, a scorecard
rung's results file or a dated reading, read from the record and never
typed; `datasheet`, a value the maker of the servo or of a part
states, a rating or a nominal and no measurement, read from where the
repo records it with the document named; and `policy`, a rule over a
choice that is no measurement, whose reason `policy.json` gives under
`why`. The choice is a decision, the owner's, or a target a bench
sweep picked: a bandwidth is a target and no quantity of the servo,
the sweep is its record, its rungs named in the `why`, and the pick
was made under a rule written before the sweep's session and is not
the owner's decision. `pinned` is a
literal typed for the pair with the rung that chose it. Every other
field is of class `none`, no source: what the servo held before its
set existed, which may be another servo's since a board is moved
between servos, an old tool's output, a boot default or a zero; a
field held back from its rule, while it is held; and a field whose
rule reads a policy entry that restates a default nobody decided,
listed under `undecided` in `policy.json`. A field with no source says
why (`reason`) and what would give it one (`needs`), and names a
maker's value the repo records that could supply it, as a candidate
and not as its source. `<pair>.sources.json`, beside the set, holds
what no rule can say of one pair: the bench and datasheet records of
its fields, and the reason and the need of the ones with none.

Every verb prints the count per class, and the count of fields with no
source whose value is per position count, which were fitted at a scale
nothing recorded. On mg90-d with board D the set holds 62 fields: 16
measured, 10 bench, no datasheet, 34 policy, one pinned and one with no
source. The pin is the board's nominal supply word, `vdd_mv` (below).
Six of the policy's are the current loop's three gains and the
observer's three, each bound by its rule at a bandwidth a bench sweep
picked ([the ladder](#the-ladder), the constants session): they had no
source while the bandwidths were an old tool's default targets nobody
had decided. Two are the limits of motion, the owner's default move
speed and its acceleration (below): they had no source while they
restated the boot default's counts. The one with no source is the
breakaway record, held at a zero never written because no result on
record is its quantity (below). Every other field whose rule gives
another value than the servo held is bound at the rule's value: a
constant with a source replaces one the board happened to hold, on
principle, and a scorecard row that loses by it is an open item on
top of it, not a reason to keep the old value. The
bench fields are the four loop gains of rung `01b-gains-3hz-50hz`,
the stops, the drive's sign and the gear class. The stops are a hand
reading of 2026-11-05, kept as a dated file beside the set that says of
itself that it is a weak record, to be replaced by three recorded reads
a stop. The drive's sign is the state the reference rung ran closed
loop on, since no calibration run found it on this servo. The gear
class is read off the part, in the registry. The registry says of this
servo that it has no datasheet worth the name, and no field is of
class `datasheet` yet.

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

Two protection fields are read off a maker's sheet. The first is the
speed ceiling, `speed_ceiling_cps`, the most the output is ever to turn
at. It protects the brushes, and it is a speed, not an effective
voltage. Its source is a maker's sheet. mg90-d is a Vorpal MG90, and
its own maker states it: Vorpal Robotics' page for the Vorpal MG90
Micro Servo, which links no PDF sheet. What the page states is a
record beside the set,
[calibration/vorpal-mg90.datasheet.json](calibration/vorpal-mg90.datasheet.json),
with the page's address, the day it was last edited and the day it was
read: a stall torque and a no-load time over 60 deg, each at 4.8 V and
at 6.0 V, and an operating voltage of 4.8 V to 6.6 V, and no current,
travel or temperature range. The page also says that Vorpal
contracted with Tower Pro to make the servos, the gears and the
plastic housing identical to the standard MG90S's: the record quotes
that as the maker's claim, and no value is taken on its strength.
TowerPro's own MG90S page stays the record of
its class,
[calibration/mg90s.datasheet.json](calibration/mg90s.datasheet.json),
which no servo of the registry names; it states the same two speeds.
The registry names the class a servo is of, so a table of classes can
grow from such records later. A value of the sheet is not taken whole:
`policy.json` holds this servo's derates, two thirds for a speed and
0.8 for any other value, decisions for this servo and not a rule for
every servo. The page states a range of operating voltage, so the
sheet's operating voltage names no one speed, and the policy says
which stated voltage the speed is read at: the one nearest the rail
the servo runs on, the registry's 5.0 V, which is the 4.8 V row. That
is the owner's decision: the 6.0 V row, 750 deg/s, would give
500 deg/s and is not taken, 400 deg/s being a speed measured not to
damage the brushes. The ceiling is the no-load speed at that voltage times the
speed derate, 600 deg/s x 2/3 = 400 deg/s, which is
8050.65 counts per second at the table's 20.13 counts per degree,
rounded down to 8050: a cap derated from a maker's value never rounds
past its derate. The set
refuses a `velocity_limit_cps` over it and `status` prints it with its
source. It is a protection field under the protection hash, moving the
way that protects less as it rises. A set written while the control
table had no field for it held it under `limits`, a bound of the set
with a hash of its own outside `set`; such a set loads as it was
written, and a build carries the limit until a build names the field
with `--protection`. The second is the current limit, below: the
current the sheet's stall voltage drives through this motor's winding,
at the 0.8 the policy takes any other value of the sheet at.

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
A field with no rule, no pin and no record in the pair's sources file
is carried at its value, with no source. `policy.json` holds the
choices that are no measurement, in
physical units, one file for every pair; a `<pair>.pins.json` beside
it, where there is one, holds literals typed for one pair, each with
its reason and the rung that chose it, and shows as `pinned`. A pin
stands ahead of a record, and a record ahead of a rule. A rule
whose input is missing does not guess: the field is held at the value
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
| `i_kp_q88` | the current loop's proportional gain, w x L x tap counts per V / shunt counts per A | w = 2 pi f, f the policy's `current_loop_hz`; L the stage 3 inductance of the locked bursts | no |
| `i_ki_q412` | its integral gain, w x R a FAST tick | w, and R the rails fit's, as `r_q12` takes it | no |
| `i_kaw_q412` | its anti-windup gain, twice the integral gain | the same | no |
| `r0_q12` | the cold resistance, the rails fit taken to 25 C along copper's slope | the stage 3 resistance and the board temperature that result states | no |
| `ke_uvs_per_rad` | Ke, uV s/rad | the stage 3 ladder | no |
| `fric_breakaway_counts` | the static friction as a current, Ts / Ke x shunt counts per A | the stage 3 static friction, back-driven with the load on the horn, which is not the field's quantity: held (below) | no |

Each rule names in the module the firmware function that uses its
field. The feedforward and the three observer gains are the firmware
repo's own synthesis (`ident/src/gains.rs`), and one test holds the
rules to the fields that file's code gives on its own test plant. The
three observer gains are the policy's by their rule: the bandwidth is a
target, not a quantity of the servo. 15 Hz was an old tool's default
target; it is now the pick of a bench sweep, which is its record, and
the gains bound at it are of class `policy`. A deviation's
`observer_hz` moves them: 10 Hz gives 5894,
1446 and 25 for `l1_q016`, `l2_q88` and `l3_q88`, 15 Hz 8640, 3180 and
84, 25 Hz 13757, 8433 and 371. The two ends are deviation files,
[observer-10hz.json](scorecard/deviations/observer-10hz.json) and
[observer-25hz.json](scorecard/deviations/observer-25hz.json), the
sides of the sweep that ran on the bench. The pick was made under a
rule written before the session, which keeps 15 Hz
([the ladder](#the-ladder), the constants session), and `policy.json`
says so under `why` with the three rungs named.

The current loop's three gains are that file's placement too. The
proportional gain cancels the winding's electrical pole, w L, the
integral gain sets the crossover, w R a FAST tick, and the anti-windup
gain is twice the integral one, with L and R in counts through the
measured stage 1 scales. The bandwidth is the policy's
`current_loop_hz`, 1 kHz, a target as the observer's is: the old
tool's default, and now the pick of a bench sweep, its record. On
this motor's 0.82 mH and
4.18 ohm the rules give 289, 1176 and 2352, where the servo held 275,
1580 and 3160, an old identification fit of another servo. The three
are bound at the placement, of class `policy`. A deviation's
`current_loop_hz` moves them in
proportion: 144, 588 and 1176 at 500 Hz, 433, 1764 and 3528 at 1.5 kHz,
the deviation files
[current-loop-500hz.json](scorecard/deviations/current-loop-500hz.json)
and
[current-loop-1500hz.json](scorecard/deviations/current-loop-1500hz.json),
the sides of the sweep that ran on the bench. The pick was made under
a rule written before the session, which keeps 1 kHz, and
`policy.json` says so under `why` with the three rungs named.
[current-gains-as-held.json](scorecard/deviations/current-gains-as-held.json)
pins the three at the old fit, for the A/B of the placement against
it ([the ladder](#the-ladder), the constants session).

Three fields are records a host keeps on the servo, which the kernel
never reads: the cold resistance `r0_q12`, the back-EMF constant
`ke_uvs_per_rad` and the static friction `fric_breakaway_counts`.
Their rules give 3725, 1159 and 97, where the servo held three zeros
never written. The cold resistance is the rails fit, 4.177 ohm at the
26.26 C its result states, taken to the 25 C the field is stated at.
Two of the three are bound at what their rules give. The breakaway is
not: the
field means the current at which a driven ramp first moves the horn,
and the static friction on record is the back-driven one, measured
with the load arm on the horn. So the field stays held at its zero,
with no source, the 97 beside it as what the rule would give, until a
driven-ramp breakaway capture gives a result it can bind to.

The supply word, `vdd_mv`, is not such a record. It is a word of the
sense block every capture carries in its `meta.json`, read from the
servo, and a capture is matched to its board by that block
(`oscnb/boards.py`). So the field holds the board entry's nominal
supply, 3300 mV, typed for the pair as a pin in
`<pair>.pins.json`, of class `pinned`. The supply as measured, 3.293
V, is the stage 1 terminal chain's `vdd_v` and stays in that result.
A rule bound it to the field once, 3293: the first rungs on that set
ran clean on the servo and could not be scored, since their captures
matched no board entry; those runs are not in the repo. The rule is
removed, so a field left without
its pin is carried as it stands and is not bound at the measured
supply again, and one test holds the set's `vdd_mv` to the board
entry's sense word and a capture of the reference rung to the board.

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
field stays at the value the set had, with no source, the hold's reason
under `held`, what its rule gives beside it as `would_be` and the
results the rule read still listed, so the set shows what the record
gives next to what the servo holds and moves nothing. A held field the
set never held stays out of
it, and `build` says so with what its rule gives. Two holds are left:
the breakaway record, and `r_bemf_q12`, held because no image has the
field yet. A value held from before the set
existed is still counted as at an unknown scale where it is per count.

The fields of the motor block whose rule gives another value than the
servo held were each held to move on a rung of its own: the back-EMF
pair, `r_q12` and `recip_ke_q`, which a replay reads worse on an
estimator without the on-time term; the Coulomb term, benched alone as
`08a-friction-measured` and not kept; and `b_i_q313`, `j_ff_q88` and
`l3_q88`, whose stored values sit inside their rules' bands. Those
holds are released, and with the current loop's gains and the host's
records the set binds each at what its rule gives:

| field | the servo held | the set binds | class |
|---|---|---|---|
| `r_q12` | 4541 | 3744 | measured |
| `recip_ke_q` | 11817 | 13315 | measured |
| `b_i_q313` | 2657 | 2434 | measured |
| `j_ff_q88` | 789 | 862 | measured |
| `l3_q88` | 77 | 84 | policy: the observer's bandwidth, the pick of its sweep |
| `fric_fc_counts` | 100 | 54 | measured |
| `i_kp_q88`, `i_ki_q412`, `i_kaw_q412` | 275, 1580, 3160 | 289, 1176, 2352 | policy: the loop's bandwidth, the pick of its sweep |
| `r0_q12`, `ke_uvs_per_rad` | 0, 0 | 3725, 1159 | measured |

`l1_q016` and `l2_q88` bind at the values the servo held, which their
rules give. `fric_fv_q016` and `ke_vpc_q` bind at what their rules
give, 812 and 315 where the servo held 289 and 355, the two stored
values of the block outside their rule's band: their release was
benched on the saved set and kept (below).
[plant-as-held.json](scorecard/deviations/plant-as-held.json) pins
the eleven fields of the table at what the servo held, the values the
reference rung `14a-set-of-record` ran on, so one deviation is the
plant as it was: the A/B of the constants of record against the ones
they replaced. Five more files pin a part of it, the arms of the
bisect of that A/B: [r-as-held.json](scorecard/deviations/r-as-held.json),
[ke-as-held.json](scorecard/deviations/ke-as-held.json),
[emf-pair-as-held.json](scorecard/deviations/emf-pair-as-held.json),
[model-block-as-held.json](scorecard/deviations/model-block-as-held.json)
and [friction-as-held.json](scorecard/deviations/friction-as-held.json).

A rung releases a hold with a deviation: `"release": [field, ...]`
names held fields, and the derived set binds each at what its rule
gives, `rung.json` showing it as [of record, deviated] like any field
a deviation moves. Nothing else of the policy is restated, and the
other holds stand. A release of a field that is not held is refused,
as is one whose rule has no value to bind. The releases benched so far
were [motor-model.json](scorecard/deviations/motor-model.json), the
model gain, the inertia feedforward and the observer's disturbance
gain, and [motor-r.json](scorecard/deviations/motor-r.json), `r_q12`
alone. The set of record has released those holds itself, so the two
files no longer build ([Running it](#running-it)).

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
either half. The first half was bound on that result. The second
stayed held then: its values are inside their bands, so the record
does not tell them from the stored ones, and its reversal rows leaned
up, each apart in one set (the 60 degree move's 498 against 528 a
second in the first set, the 1 % ramp's 156 against 186 and the 10 %
ramp's 416 against 514 in the second). It is bound now with the rest
of the block, on the principle above, and those reversal rows are an
open item on top of it. The 20 degree landing ran its hold out hunting, a settle
over 1000 ms, in 3 of 6 runs of the first step's released side and
none of its control's, and in the split in 2 of 6 runs of the control
itself, 2 of 6 of the first half and 1 of 6 of the second: it lands on
a stuck code's gap and flips from run to run on the set of record
alone.

Every protection field has a rule, and every one is bound. The
current limits, the thermal limit and the decisions the servo already
held were bound first. The rest came
in seven protection steps, one build a step with `--protection`
naming the step's fields and nothing of the plant moving: the NTC
curve's three words, the winding model's four fields, the
position-error screen's angle and time, the sensor screen, the
undervolt floor, the rail fold and the stall response. Each is said
below with what the servo held before it. A servo that still stores
the set from before these steps, set `297f3cb4a544`, differs from the
set of record in those thirteen fields and in the eleven of the plant
above, and an install on it names the thirteen (below, after the
undervolt floor).

The board NTC's curve is the design quadratic of the board's stage 1
`ntc-curve` result ([stage 1](../1-board/README.md)): `ntc_t_ref_cc`,
`ntc_k1_q88` and `ntc_k2_q24` as that result holds them, the
fixed-point quadratic in the count fitted through the part's nominal
beta equation and pinned to read it exactly at a true 90 C, no more
than 1.0 C under it from 15 to 90 C. The thermocouple run graded that
equation and found it inside the meter's class once the rig's two
effects are taken out. The result's `refit_*` curve and its
fitted beta are fits against a thermocouple shown to read low and are
not design constants. `ntc_raw_ref` is the count that result names for
its design curve (`design_raw_ref`), 2048, which is also the count the
servo held, so it bound as it stood. The other three words are bound
as one protection step, in place of the curve the servo held and
stored, the host's tangent form about 25 C. What each reads on the
bench board, through
the kernel's own conversion and with the part's nominal equation as
the truth:

| board at a true | 25 C | 40 C | 60 C | 80 C | 100 C |
|---|---|---|---|---|---|
| the stored curve reads | 25.0 | 40.2 | 57.3 | 68.5 | 75.1 |
| the curve of record reads | 24.3 | 40.6 | 65.1 | 83.5 | 95.0 |

| reaches | the cutoff at 90 C |
|---|---|
| the stored curve | never: it tops out at 84.2 C |
| the curve of record | at a true 90.1 C |

So on the stored curve the board term of the thermal protection never
reads its limit, and the curve of record, pinned at a true 90 C, puts
the limit on its number: its reading passes 90.00 C at a true 90.1 C.
Below its pin it reads over the part's equation, 3.5 C at a true 80 C,
which is the protective side, and past its pin under it, 5 C at a true
100 C and further behind past that. Two more things about the curve of
record. Between a true 21.8 and 38.2 C it reads under the stored curve,
by up to 1.1 C at 29.9 C, so over that stretch it protects less. And it
turns at a true 8 C: below that it reads warmer as the board cools,
18.4 C at a true 0 C where the stored curve reads 4.8 C.

The thermal protection is designed to one limit: the winding estimate
at or over `cutoff_cc` is an overtemperature fault, and nothing folds
the current limit under it. The field's rule is the policy's
`cutoff_c` in centi-C, 90 C, of class `policy`, and `policy.json` says
under `why` whose choice the number is: the owner's. The servo held a
cutoff at 100 C behind a derate from 80 C. The set's 90 C is under that
cutoff, which protects more, so its install names `cutoff_cc` as
`--protection` and not as `--loosen`. `derate_start_cc` has left the
set with the table: the module lists it as retired, with the way it
moved to protect less, so a set written while it stood, the copy a
rung of record ran on, still loads as it was written. A build carries a
retired field until one names it with `--protection`, which leaves it
out and moves the protection hash alone.

The winding estimate's four fields, `thermal_tau_s`, `th_rise_q016`,
`thermal_tau2_s` and `th_rise2_q016`, are one model of the winding's
heat, a leak and a rise for each of two nodes, and one protection step.
Their rules read the stage 4 result
([stage 4, winding-temperature](../4-estimators/README.md#winding-temperature)):
the fast node is this pair's own, of class `measured`, and the slow
node the family's, of class `policy` until a lock hold measures this
motor's. The policy's `winding_plateau_reading` names the reading of
the plateau the two rises take, and it is `high`, the owner's decision:
of the two readings it alone reads at least as hot as what the servo
held in every case computed. The rules give 2 / 434 / 129 / 1274, and
the kernel runs the two leaks at 2.048 and 131.072 s (the rounding as
firmware 08c7beaf has it, mirrored in `oscnb.winding`). The four are
bound as one protection step, in place of the one node on the slow
leak the servo held, 115 / 1143 / 0 / 0.
No direction field by field says what that step does, so an install
on a servo that holds another four prints what the estimate reads
over the board NTC under the servo's four and under the set's, by the
kernel's integers: a hold
from rest at the continuous current and at the limit, the rest after
it, four duty cycles, and whether the wanted four read at least as hot
in every case. Over the four the servo held they do, in every case.
The install still names all four as `--protection`, and
the two leaks, which are judged, and the fast rise, which falls, as
`--loosen` too.

The current limits are taken to shunt counts by the board's measured
counts per amp, the scale the set carries. The kernel compares its own
current reading against them, and that reading is over the awake zero,
which already holds the driver's own current, so the in-drive offset
of stage 1 is not added. The three under the cap are each a policy
entry in mA, the mA the stored counts were typed for: the whole mA the
board's nominal parts take to each stored count. The cap was such an
entry too, 500 mA, which the owner judged conservative. It is now read
off the maker's sheet: the voltage the page states the stall torque at
nearest the rail this servo runs on, 4.8 V on 5.0 V, over the
winding's resistance of the stage 3 rails fit, 4.177 ohm, at the 0.8
the policy takes a maker's value at, 0.919 A. At the measured
1135.01 counts per amp that is 1043.5 counts, rounded down to 1043: a
cap derated from a maker's value never rounds past its derate, where
every other field rounds half away from zero.

| field | policy | the servo held | which was | the rule gives |
|---|---|---|---|---|
| `current_limit_counts` | 0.8 x 4.8 V / 4.177 ohm, 919.4 mA | 554 | 488.1 mA | 1043 |
| `i_cont_counts` | 250 mA | 277 | 244.0 mA | 284 |
| `stall_release_counts` | 90 mA | 100 | 88.1 mA | 102 |
| `oc_trip_counts` | 2200 mA | 2439 | 2148.9 mA | 2497 |

Each limit the servo held was 2.3 % under the current it was typed
for, and the rule's value puts more counts on the same mA, which
protects less than what the servo held. The four were bound at what
their rules gave then, the cap at 568 for its 500 mA: one rung of three runs on the set with the four moved
(`zz-current-limits-true-ma`) raised no fault and left every hold at
rest. Set beside `14a-set-of-record`, recorded three hours before it
and not alternated with it, its 60 degree move settles in 883 [874,
893] ms against 910 [906, 912] and its lag at the 10 % ramp reads
1.941 [1.934, 1.948] deg against 1.983 [1.948, 2.000]; 28 to 68 of its
ramps' goals went out late on a busy host, so its follow rows are not
a comparison.

The four mA are decisions. The continuous current stays
250 mA, interim, until a lock hold gives this motor's plateau, and the
release moves with it; `policy.json` says so under `why`. The overcurrent
trip stays 2200 mA: it is a detector of a wiring or bridge fault, above
the locked-rotor current on a full 2S pack, about 2.0 A, so only a
short reaches it. The collision trip stays off, a decision:
the policy's `collision_trip` is `off`, and the rule of
`stall_tau_trip_counts` gives the field's largest value, which no
disturbance estimate passes. A level in counts has no rule yet.

A deviation moves the protection fields of one step: the words of the
NTC curve are one step, the four current limits another, the winding
model's four a third and the position-error screen's angle and time a
fourth, a protection field in none is a step alone, and a
release of one field of a step releases every held field of it, so no
curve of two fits, no limits at two scales, no model of two halves and
no screen of one decision and one default are built. A build of the
set of record takes a step the same way, its fields named together
with `--protection`. An install of the NTC curve's step on a servo
that holds the stored curve names the three words it moves as
`--protection` and again as `--loosen`: the curve's words are judged.
The current limits' step
was installed the same way, the continuous current judged and the
other three moving up.

The fields in counts of position were typed when the servo ran on the
raw pot, 18.913 counts per degree at mid travel, and are read by a
kernel that runs on the set's count, the table's 20.127 counts per
degree. A count typed then is 6.4 % less speed or angle now. Each has
a policy entry in the unit a person sets it in and a rule that takes
it to counts at the set's scale: a speed in deg/s and an angle in deg
times the counts per degree, an acceleration in deg/s^2 times the
counts per degree over the MEDIUM rate, since the kernel holds it as
the speed one MEDIUM tick adds. The two limits of motion are the
owner's default move speed and its acceleration, and the two screens
are decided.

| field | the kernel's use | policy | the servo held | which is, on the table | the rule gives |
|---|---|---|---|---|---|
| `speed_ceiling_cps` | the most the output turns at, the clamp of the profile's speed and of the position loop's command, count/s; a protection field | 400 deg/s, the sheet's 600 x 2/3 | no such field | - | 8050 |
| `velocity_limit_cps` | the move speed, the profile's speed, at most the ceiling, 0 being the ceiling, count/s | 200 deg/s | 1500 | 74.53 deg/s | 4025 |
| `accel_limit_q88` | the speed the profile may add in a MEDIUM tick, count/s, Q8.8 | 4000 deg/s^2 | 3840 | 1491 deg/s^2 | 10305 |
| `stall_omega_max_cps` | a press over the continuous current counts toward a stall while the estimate's speed is under this, count/s | 24.84 deg/s | 500 | 24.84 deg/s | 500 |
| `pos_error_counts` | the profile less the estimate, over this for the error time, faults; counts | 9.9 deg | 400 | 19.87 deg | 199 |

The servo has two speeds, the owner's decision. One is the speed
ceiling, 400 deg/s, the most the output ever turns at, which protects
the brushes: the protection field `speed_ceiling_cps` (above). The
other is the move speed, which a user sets from 0 to the
ceiling, held in `velocity_limit_cps` with its acceleration in
`accel_limit_q88`, the two of the plant and moving together. Its
default is 200 deg/s, half the ceiling, with the acceleration that
reaches it in 50 ms, 4000 deg/s^2: 4025 and 10305 as the fields hold
them on the set's count. It is interim until the load arm, and meant to
rise toward the ceiling. The bench sweep under the ceiling ran the
1500 count/s and 3840 the servo held, 74.5 deg/s and 1491 deg/s^2 on
the table, against 200 and 400 deg/s, each side at the speed reached
in 50 ms, and its rule, written before the session, keeps 200 deg/s
([the ladder](#the-ladder), the constants session). 400 deg/s is the
ceiling's speed, but the move speed rounds to the nearest count, 8051,
and the ceiling down, 8050, so 400 deg/s is refused as one count over
it, as is a deg/s more; a speed at the ceiling's count passes. The default does not move what the scorecard
measures: a rung runs at the test set's own limits of motion, the
counts the servo held before, whatever the default
([running it](#running-it)), so the ladder stays comparable. A rung
at another speed is a deviation of policy entries; a rung at the
default itself is none the tool runs yet, since a deviation that
states the default moves nothing over the set. The
stall speed is decided at the speed its counts are on the set's count,
24.84 deg/s, four times the speed noise of a still shaft. Its rule
then gives the 500 the servo held, so it bound as it stood, of class
`policy`, at a scale that is on record, and had no step. The
position-error screen
is decided at 9.9 deg, twice the worst healthy following error on
record, 4.4 deg on a free horn, interim until the load arm confirms
it. That is 199 counts on the set's count where the servo held 400,
so the field moved, with the screen's time, as one protection step
(below).

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
band sits on the floor, 4, the value the servo holds. The band of 4
counts is a decision, said under `why`. The band at 8
counts, over half the train's play, stays a pin of its own deviation.
The sensor screen is the policy's `sensor_delta_max_counts`, raw
counts between two MEDIUM samples. It is decided at 64, twice the worst
healthy step of the pot on 5 V, 32 counts, where the servo held the
boot default's 256: the band bound as it stood, and the screen is
bound at 64 by a protection step of its own.

The other screens, the supply terms and the open loop's two choices
each have a policy entry in the unit a person sets it in, its reason
under `why`, and a rule: a time in ms is that many kernel ticks, a
count of samples is the field, and a choice between named behaviors is
the policy's word, written as `osc dump` prints the field. Where the
decision is the value the servo holds, the field binds, of class
`policy`, and nothing moves:

| field | policy | the rule gives | why |
|---|---|---|---|
| `duty_max_q15` | `duty_max_share` 1, the whole rail | 32767 | no voltage cap: the speed ceiling and the current limit protect the motor |
| `stall_time_ticks` | `stall_time_ms` 500 | 10000 | on a free horn the stall conditions never last past 12 ms, and the Feetech class allows 2 s; interim until the load arm |
| `oc_trip_ticks` | `oc_trip_samples` 8 | 8 | it rides over single-sample spikes, 0.4 ms |
| `sensor_bad_count` | `sensor_bad_samples` 4 | 4 | the default stands: no capture shows a failing pot, and a capture of one would revisit it |
| `openloop_decay` | `openloop_decay` slow | Slow | the off-time brake: smooth current, torque in proportion to duty |
| `openloop_zero_brake` | `openloop_zero_brake` off | off | zero duty coasts in open loop, which calibration relies on |

Where the decision is another value than the servo held, a protection
step of its own bound the field at the decision's, one build a step.
The last column is what an install names on a servo that still holds
the old value:

| field | policy | the servo held | the set binds | its install names |
|---|---|---|---|---|
| `stall_response` | `stall_response` yield | Fault | Yield | `--protection` and `--loosen`: judged |
| `pos_error_counts`, `pos_error_time_ticks` | `pos_error_deg` 9.9, `pos_error_time_ms` 100 | 400, 10000 | 199, 2000 | `--protection`: both protect more |
| `sensor_delta_max` | `sensor_delta_max_counts` 64 | 256 | 64 | `--protection`: it protects more |
| `v_undervolt_counts` | `v_undervolt_v` 3.70 | 720 | 922 | `--protection`: it protects more |
| `rail_fold_q8` | `rail_source_ohm` 0.8 | 1430 | 1467 | `--protection` and `--loosen`: judged |

A blocked horn now yields: the current limit folds to the continuous
current and the servo recovers on the next command, as the Feetech
class does, with the 90 C cutoff as the backstop. The position error's
100 ms is a time a healthy error never holds, and it catches a reversed
motor before it reaches a stop; it is interim with the angle, and the
two are one step. The rail fold's slope is the shunt counts a rail
count of sag gives through the source the policy designs for, 0.8 ohm,
a soft supply such as a battery on a long lead; a stiffer setting
protects less, and it never acts on the 5 V bench supply. The 1430 the
servo held is that source through the board's nominal parts. Through
the measured counts per amp and rail counts per volt the same source
is 1467, a steeper slope, which folds later, so its install is judged
and named to loosen.

The undervolt floor stands on a bench record, the rail walk of
2026-11-14, kept as a dated file beside the set with its rows under
`telemetry/<pair>/rail-walk/`: the supply stepped down and back up with
torque off, the rail tap and the chip's supply read at each level. Down
to a supply of 3.70 V the chip's supply, which is the ADC's reference,
holds, and the rail reading follows the rail. Under it the chip's
supply follows the rail down and the reading stops: 860 to 861 counts
at the edge, 3.45 V at the rail on stage 1's rail line, and 857 to 862
at every level down to 2.70 V. The 720 counts the servo held are under
that floor, so the undervolt fault could not trip on a rail the chip
survives. The policy's floor is 3.70 V at the rail, 0.25 V over the
edge, for the 5 V bench supply, and its rule takes it through the
measured rail line, offset and scale, to 922 counts, on the stretch
that line is graded over, which is what the set binds. A 2S pack's
floor waits for the pack on the
board. The rail fold stands on the floor, so its band moves up with it.

A servo that stores the set from before the releases, set
`297f3cb4a544`, holds the plant the reference rung `14a-set-of-record`
ran on and the thirteen protection fields at the values the tables
above give as held. The install of the set of record on it writes the
eleven plant fields with no name asked, and the gate asks for the
thirteen, in the set's order:

```
--protection stall_response thermal_tau_s v_undervolt_counts thermal_tau2_s pos_error_time_ticks pos_error_counts sensor_delta_max rail_fold_q8 th_rise_q016 th_rise2_q016 ntc_t_ref_cc ntc_k1_q88 ntc_k2_q24
--loosen stall_response thermal_tau_s thermal_tau2_s rail_fold_q8 th_rise_q016 ntc_t_ref_cc ntc_k1_q88 ntc_k2_q24
```

Eight are named to loosen: the seven that are judged, and the fast
rise, which falls. The other five protect more. A deviation of the
plant over the set asks for the same names on such a servo, since it
moves no protection field. One test holds both lists to the gate.

Of the loop gains, two carry a position unit: `v_kp_q88` and
`v_ki_q412` are shunt counts per count/s, so they fall as the counts
per degree rise. The four gains of the position and velocity loops are
of class `bench`: rung `01b-gains-3hz-50hz` chose them, and the set
reads them from that rung's results file. The rung ran on the raw pot,
and the same counts have run on the table since `12a-pos-table-live`.
They stay as benched, in counts, which is a decision `policy.json`
records under `why`, so neither is at a scale nothing recorded.
`p_kp_q88` is count/s per count, a rate, and carries no position unit,
nor do `v_kaw_q412` and the three gains of the current loop, which are
bound at what their rule gives (above).

`status` is offline: the sha1 of every file the set read against
the file now, a moved one named with the fields that list it. It then
prints the debt: the count per class and every field with no source, a
line each, with its reason, what would give it one and, where a
maker's value on record could supply it, that candidate; then the
fields whose record is a weak one. A test holds the count of
fields with no source, so a change that gives a field a source shows
it going down. `check`
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

The bench servo stores set `324e326794b5`, the set of record, since
the speed blend's session, and `18a-speed-blend` and
`19a-lifecycle-heat-mean` are the rungs that ran on it; that save's
record, under
`telemetry/<pair>/constants/324e326794b5/`, is not kept in the repo,
as no save's record is.

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
rung on the table are in degrees of the table's own scale. From
`14a-set-of-record` on, a rung runs on the pair's set of record, or on
that set with one named deviation, and holds the goals of a ramp to the
period. A rung is
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
- `14a-set-of-record`: firmware 624e29d3 on the pair's set of record,
  the first rung that names a constant set and the first whose ramps
  hold their goals to the period. The set is what `13a` ran on with the
  viscous friction term and the feedforward Ke at what their rules give
  ([The constant set](#the-constant-set)), and the servo has it saved,
  so a run installs nothing a reboot would not return. 624e29d3 differs
  from `13a`'s f933a4a7 in how a commit of an all-zero position table
  lands and in where the drive check starts a leg, neither of which is
  in what the loops command. The rung is kept as the reference for
  rungs on the set, not on its rows: a later rung compares with this
  one and not with `13a`.
  Its ramps ran at 6.204, 18.59 and 61.67 deg/s, each the same in the
  three runs to 0.02 %, with no goal late, where `13a`'s ran 6.28 to
  6.59, 18.76 to 19.47 and 62.28 to 64.48. The 10 % ramp's speed is its
  goals' 62.1 deg/s less the shorter last step of each ramp.
  As the median of three runs and their range, against `13a`: the lag
  at the 10 % ramp reads 1.983 [1.948, 2.000] deg against 2.033 [2.019,
  2.119], and as a time 32.16 [31.59, 32.43] ms against 32.41 [31.85,
  32.87], the ranges of the time overlapping; the follow error's sd at
  the 1 % and the 3 % ramp is lower, 0.107 [0.102, 0.108] deg against
  0.118 [0.114, 0.129] and 0.151 [0.147, 0.153] against 0.172 [0.163,
  0.174]; the 2 degree overshoot is lower, 0.323 [0.267, 0.332] deg
  against 0.373 [0.373, 0.435]. Two sittings and four changes lie
  between the two rungs (the firmware, the two constants, the saved
  state, the held period), and no row of this comparison is given to
  one of them: the two constants were alternated against the set
  without them, as told under the constant set, and moved no row.
  The 20 degree landing above the center, open under `13a`, settles in
  all three runs here, 302 [302, 303] ms with an overshoot of 0.106
  [0.016, 0.174] deg, where it hunted in all three of `13a`. It flipped
  the same way between `13a`'s own control sides, and on the control
  sides of the two motor steps that preceded this rung
  (`zz-ab-before-motor-rest`, `zz-ab-before-motor-split` and their
  `-2`) it hunted to the end of its hold in 2 of 12 runs. The rows that
  take that landing in move with it: the 20 degree move's current, 113
  [113, 118] mA against 193 [190, 211], its duty, the landing rows.
- `15a-set-of-record`: firmware 3dcca8ff on the set of record as it
  then stood, set `297f3cb4a544`: `14a`'s plant, with the current
  limits at the mA they were typed for and the one thermal limit at
  90 C ([The constant set](#the-constant-set)). Four commits lie
  between 624e29d3 and 3dcca8ff: the run-time drive check is removed,
  the reference's volts are commanded through the winding under the
  shunt floor, the winding estimate's cutoff is the one thermal limit,
  and the winding resistance follows the estimate's temperature. The
  rung is kept as the reference on that firmware, not on its rows. It
  is one side of three alternated in the order abc, and was not
  alternated with `14a`, recorded six hours before it. Every run
  started over the host's load limit after the full wait, and 18 to
  91 of a run's ramp goals went out late where `14a` had none, so its
  follow rows are not a comparison with `14a`'s.
  As the median of three runs and their range, against `14a`, on the
  moves: the duty reverses a quarter to a half as often, 114 [70, 189]
  a second against 267 [244, 304] on the 2 degree move, 134 [101, 161]
  against 523 [511, 616] on the 20 degree and 117 [114, 120] against
  446 [435, 450] on the 60 degree; the landings lose, the overshoot
  0.453 [0.413, 1.286] deg against 0.323 [0.267, 0.332], 0.646 [0.273,
  0.745] against 0.106 [0.016, 0.174] and 0.183 [0.124, 0.242] against
  0.009 [0, 0.065], the worst drive after an arrival 1044 [449, 1211]
  ms against 345 [156, 348], the 20 degree settle 634 [293, 1494] ms
  against 302 [302, 303]; the rest error is lower, 0.227 [0.211,
  0.247] deg against 0.305 [0.304, 0.339], and the 60 degree move
  settles sooner, 852 [847, 858] ms against 910 [906, 912]. Six
  changes lie between the two rungs, the four commits, the current
  limits and the thermal limit, and no row is given to one of them.
  Its two other sides are `zz-ab-r-measured` and `zz-ab-ntc-pinned`.
  The first is the set with `r_q12` alone at the stage 3 rails fit,
  3744 in place of 4541: the duty reverses 1.6 to 3.0 times as often
  on the moves and the two faster ramps (340 [309, 411] a second
  against 114 [70, 189] on the 2 degree move, 229 [213, 278] against
  134 [101, 161] on the 20 degree, 219 [186, 227] against 117 [114,
  120] on the 60 degree, 129 [125, 143] against 82 [79, 85] at 3 % and
  229 [221, 243] against 125 [101, 162] at 10 %), the 60 degree
  overshoot falls to 0 [0, 0.031] deg from 0.183 [0.124, 0.242], and
  the lag at 3 % and 10 % and the hold jitter grow (19.75 [19.72,
  19.94] ms against 17.73 [17.35, 17.98], 33.05 [32.82, 33.35] against
  30.35 [30.16, 30.48], 0.064 [0.060, 0.064] deg against 0.053 [0.052,
  0.056]). The second is the set with the NTC curve's protection step
  alone: no row's ranges come apart from this rung's.
- `16a-plant-as-held`: firmware 08c7beaf on the set of record, set
  `b300d5e71e38`, with
  [plant-as-held.json](scorecard/deviations/plant-as-held.json): the
  eleven plant fields the release moved pinned at what the board held,
  and the seven protection steps installed. It is the plant of `15a`
  on the next firmware commit, in which the back-EMF boxcar closes on
  its own phase, two ticks ahead of the observer, and it is the first
  side of the session that scored the constants of record (`16b`),
  kept as that side. No fault on any run. Against `15a`, four hours
  earlier and not alternated, with the commit and the protection steps
  between them, as the median of three runs and their range: the 60
  degree landing loses, its overshoot 0.714 [0.301, 0.714] deg against
  0.183 [0.124, 0.242], its current 175 [123, 190] mA against 113
  [109, 117], with the median drive after an arrival 84.15 [84.05,
  90.55] ms against 51.15 [49.65, 67.30]; the 1 % ramp costs more, 91.5 [82.2,
  98.1] mA against 72.1 [71.3, 77.1] with an error sd of 0.159 [0.153,
  0.167] deg against 0.129 [0.128, 0.129]; the rest error is lower,
  0.159 [0.132, 0.187] deg against 0.227 [0.211, 0.247], and so is
  the lag at 3 % and 10 %, 16.92 [16.73, 17.10] ms against 17.73
  [17.35, 17.98] and 29.75 [29.25, 30.10] against 30.35 [30.16,
  30.48]. The duty's reversals overlap on every move and ramp. 26 to
  34 of a run's ramp goals went out late, 18 to 91 on `15a`.
- `16b-constants-with-sources`: firmware 08c7beaf on the set of
  record, set `b300d5e71e38`: every constant of the plant that has a
  source at what this motor's results give, the current loop at the
  rule's 1 kHz, and the seven protection steps
  ([The constant set](#the-constant-set)). Alternated with `16a` in
  three rounds, the order abc, `zz-ab-current-gains-as-held` the
  third side, so it differs from `16a` in the plant alone. No fault on
  any run, and 24 to 35 of a run's ramp goals went out late, 26 to 34
  on `16a`.
  The rung is kept, and not by the ladder's rule: rows lose on it. It
  is kept by the owner's decision that the constants measured on this
  motor are the canonical ones, and what it loses is open on top of
  it. Both halves, as the median of three runs and their range against
  `16a`, every row named having its ranges apart.
  The landings are better: the overshoot reads 0.152 [0.112, 0.233]
  deg against 0.413 [0.354, 0.655] on the 2 degree move, 0.161 [0,
  0.161] against 0.543 [0.373, 0.814] on the 20 degree and 0.009 [0,
  0.031] against 0.714 [0.301, 0.714] on the 60 degree; the 20 degree
  move settles in 362 [356, 379] ms against 564 [422, 671]; the median
  drive after an arrival is 38.65 [25.40, 44.90] ms against 84.15
  [84.05, 90.55]; the crawl swings 0.362 [0.334, 0.384] deg against 0.691
  [0.477, 0.738]; the 1 % ramp takes 77.3 [74.8, 77.7] mA against 91.5
  [82.2, 98.1] with an error sd of 0.130 [0.129, 0.138] deg against
  0.159 [0.153, 0.167], and the 2 degree move 119 [117, 121] mA
  against 149 [133, 163]. The 60 degree move settles in 970 [954, 983]
  ms where `16a` reads 1997 [840, 2000], out of its window in two runs
  of three, the ranges overlapping.
  The duty reverses 2 to 3 times as often: 187 [137, 222] a second
  against 99 [76, 112] on the 2 degree move, 293 [246, 332] against
  116 [114, 132] on the 20 degree, 316 [289, 331] against 131 [102,
  145] on the 60 degree, 186 [167, 192] against 85 [82, 95] at 3 % and
  320 [269, 327] against 106 [87, 118] at 10 %; at 1 % it is 108 [104,
  114] against 93 [83, 94].
  The rest error and the lag are worse: the rest error reads 0.367
  [0.305, 0.380] deg against 0.159 [0.132, 0.187] and the two-sided
  error 0.289 [0.229, 0.337] against 0.190 [0.180, 0.209]; the lag is
  15.5 [14.3, 15.8] ms against 4.4 [4.3, 5.6] at 1 %, 20.2 [19.6,
  21.6] against 16.9 [16.7, 17.1] at 3 % and 37.44 [36.91, 38.65]
  against 29.75 [29.25, 30.10] at 10 %. With them the duty at 3 % and 10 % is
  higher, 12.7 [11.5, 12.8] % against 10.8 [10.6, 11.1] and 24.7
  [22.7, 24.7] against 22.2 [21.1, 22.3], the current at 3 % is 96.1
  [89.3, 96.9] mA against 83.8 [82.5, 84.9], and the speed report at
  1 % reads -3.0 [-3.3, -1.5] deg/s against -1.0 [-1.3, -0.6].
  What carries each half is bisected below (the constants session).
- `17a-constants-of-record`: firmware 08c7beaf on the set of record,
  set `b300d5e71e38`, three runs alone after ten sweeps of the travel:
  the reference for rungs on the constants of record, kept as that and
  not on its rows. It is `16b`'s set and firmware 70 minutes later and
  not as one side of three, after the sweeps of the session kept the
  current loop at 1 kHz and the observer at 15 Hz and with the limits
  of motion as held. No fault. Against `16b`, as the median of three
  runs and their range, six rows' ranges come apart: the crawl swings
  0.281 [0.265, 0.301] deg against 0.362 [0.334, 0.384], the 1 % ramp
  reads an error sd of 0.116 [0.114, 0.121] deg against 0.130 [0.129,
  0.138] and 99 [96, 101] reversals a second against 108 [104, 114],
  the 2 degree move takes 111 [106, 116] mA against 119 [117, 121],
  and the 60 degree move reads 339 [332, 373] reversals a second
  against 316 [289, 331] and a duty of 21.8 [21.7, 22.5] % against
  21.4 [20.7, 21.5], both by a hair. Every other row's ranges
  overlap, the rest error at 0.304 [0.304, 0.346] deg and the lag at
  10 % at 38.3 [37.1, 38.4] ms among them. 26 to 52 of a run's ramp
  goals went out late.
- `18a-speed-blend`: firmware 2e9cb678 on the set of record, set
  `324e326794b5`, which the servo has saved, three runs alone after
  ten sweeps of the travel: the reference for rungs on the speed
  blend, kept as that and not on its rows. Two commits lie between
  08c7beaf and 2e9cb678: a7d3b377 holds a back-EMF boxcar off through
  the start-up half and voids a half that holds a drive flip, and
  2e9cb678 feeds the velocity loop the pot observer's speed plus the
  back-EMF's fast part in place of the source switch. Its image is the
  one the blend's sides of the sessions below ran, the same sha1. The
  set differs from `17a`'s, `b300d5e71e38`, in the default limits of
  motion alone, and this is the first rung recorded at the test set's
  own limits ([Running it](#running-it)): its `rung.json` names them
  under `motion`, from `test set`, 1500 count/s and 3840 as the fields
  hold them, the counts `17a` ran at as held, so its moves ran as
  `17a`'s did. No fault. Recorded two hours after `17a` and not
  alternated with it. 21 to 33 of a run's ramp goals went out late, 26
  to 52 on `17a`.
  Both halves, as the median of three runs and their range against
  `17a`, every row named having its ranges apart unless said. The duty
  reverses far less often: 38.9 [26.6, 40.6] a second against 249
  [198, 252] on the 2 degree move, 21.6 [20.7, 31.8] against 312 [303,
  326] on the 20 degree and 13.5 [12.1, 16.4] against 339 [332, 373]
  on the 60 degree, 51.7 [50.7, 59.9] against 99 [96, 101] at 1 %,
  58.4 [57.5, 69.7] against 177 [159, 191] at 3 % and 28.1 [20.3,
  34.3] against 310 [265, 337] at 10 %. The current falls on every
  move and ramp: 93.8 [93.7, 96.7] mA against 111 [106, 116], 85.8
  [85.0, 85.9] against 134 [131, 139] and 87.1 [86.6, 88.1] against
  133 [131, 138] on the 2, 20 and 60 degree moves, 71.6 [71.1, 72.1]
  against 75.5 [73.5, 75.6], 82.6 [80.0, 82.9] against 92.1 [91.3,
  92.9] and 96.3 [93.4, 96.8] against 154 [147, 159] at 1, 3 and 10 %,
  and the duty with it. The 60 degree move settles in 897 [893, 899]
  ms against 976 [971, 977]. The lag is shorter at 3 % and 10 %, 18.5
  [18.5, 18.8] ms against 20.8 [20.0, 21.0] and 34.7 [34.0, 34.7]
  against 38.3 [37.1, 38.4]; at 1 % it overlaps, 16.3 [15.9, 17.8]
  against 15.6 [15.2, 16.5]. The speed report at 10 % reads 2.57
  [2.09, 2.68] deg/s against 9.80 [1.05, 11.65], the ranges
  overlapping. With them the crawl swings 0.197 [0.188, 0.200] deg
  against 0.281 [0.265, 0.301], the speed ripple of a move is 27.0
  [26.9, 28.9] % against 35.5 [33.6, 36.7] and the position report
  error 0.038 [0.033, 0.049] deg against 0.060 [0.050, 0.073].
  Against that: the 20 degree move overshoots, 0.161 [0.019, 0.171]
  deg against 0 [0, 0]; the follow error's sd at 10 % is wider, 0.495
  [0.462, 0.507] deg against 0.445 [0.430, 0.452]; the worst drive
  after an arrival (`landing_drive_ms_worst`) is 626.5 [176.4, 726.8]
  ms against 173.3 [91.8, 185.4], the ranges overlapping by a hair;
  and a wide-code hold hunts, 1 [0, 1] of 8 against 0 [0, 0]. The 60
  degree overshoot leans up, 0.130 [0.006, 0.211] deg against 0.006
  [0, 0.211], and the rest error overlaps, 0.313 [0.294, 0.346] deg
  against 0.304 [0.304, 0.346]. Those four losses are open on it.
- `19a-lifecycle-heat-mean`: firmware 32ea2f31 on the set of record,
  set `324e326794b5`, which the servo has saved, three runs alone after
  ten sweeps of the travel, at the test set's own limits of motion: the
  reference for rungs on 32ea2f31, kept as that and not on its rows.
  Two of the commits between 2e9cb678 and 32ea2f31 change the image:
  dca48f5e steps the winding heats on the slow cycle's mean of the held
  current squared, not on one sample, and 32ea2f31 holds the control
  path's lifecycle in one state, so a mode change under torque enters
  each mode on the loop states that physically hold it. cc7cf782,
  bedb7c33 and dce52cea change tests only, bfc71a69 a document and a
  comment. No fault. Recorded about three hours after `18a` and not
  alternated with it, with the winding estimate 1.4 C cooler, 23.5 to
  24.2 C over the segment polls against 24.9 to 25.7 C. 3 to 33 of a
  run's ramp goals went out late, 21 to 33 on `18a`.
  Both halves, as the median of three runs and their range against
  `18a`, every row named having its ranges apart. One row reads worse:
  the 60 degree move takes 90.2 [89.2, 91.5] mA against 87.1 [86.6,
  88.1]. Ten read better, the lag at 1 % in both its rows. The duty
  falls on the 2 degree move, 12.82 [12.74, 12.91] % against 13.49
  [13.31, 13.58], and on the ramps, 8.21 [8.17, 8.22] against 8.45
  [8.37, 8.47] at 1 %, 10.90 [10.86, 10.95] against 11.22 [10.99,
  11.45] at 3 % and 18.46 [18.33, 18.54] against 18.82 [18.64, 18.90]
  at 10 %. The 20 degree move settles in 350 [347, 354] ms against 362
  [357, 373] and reverses 14.3 [13.5, 19.4] times a second against
  21.6 [20.7, 31.8]. The 60 degree move settles in 891 [884, 892] ms
  against 897 [893, 899], by a hair. The lag at 1 % is 0.0944 [0.0900,
  0.0982] deg against 0.101 [0.0987, 0.110] and 15.2 [14.5, 15.8] ms
  against 16.3 [15.9, 17.8], both by a hair, and the follow error's sd
  at 3 % is 0.147 [0.144, 0.147] deg against 0.152 [0.148, 0.152], by
  a hair.
  No row is given to the two commits. The two rungs were not recorded
  together, so the image was run back to back with 2e9cb678 in two
  sessions (the lifecycle head's session, below). There the 60 degree
  current is not apart for the worse in either, and the one row apart
  the same way in both, the 2 degree move's reversals, is not apart in
  a count over forty such moves. Of the four losses open on `18a`, the
  20 degree overshoot reads 0.019 [0, 0.078] deg, the follow error's sd
  at 10 % 0.477 [0.465, 0.489] deg, the worst drive after an arrival
  596 [364, 1086] ms and the wide-code holds that hunt 0 [0, 1] of 8,
  each overlapping `18a`'s range. They stay open on this rung, and so
  do the 2 degree move's settle and reversals, rows whose range over
  three runs understates their scatter between sessions.

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

The constants session, its bisect and its sweeps:

Every rung here is firmware 08c7beaf on the set of record, set
`b300d5e71e38`, or on that set with one deviation, the seven
protection steps installed, and no run of any raised a fault. Their
result files are kept and their rows are not. The session has no
notebook: the numbers below are the medians and ranges of the result
files. The host limits what the follow rows of this session can say:
24 to 52 of a run's ramp goals went out late on `16a`, `16b` and
`17a` and 3 to 115 on a side of the bisect or of a sweep, and on the
reference rung `15a` 18 to 91, each of its three runs after the 300 s
wait for the host's load ran out with the load still over its limit,
so a lag or a follow row here carries the host's late goals with the
servo's following.

- `zz-ab-current-gains-as-held`, the third side of `16a` and `16b`,
  three rounds: the set with the current loop's three gains as the
  board held them, 275, 1580 and 3160 in place of the placement's
  289, 1176 and 2352. Against `16b` thirteen rows' ranges come apart,
  the lag at 10 % in both its rows, and they do not point one way. Lower on the held gains: the 2
  degree move's current (110 [102, 116] mA against 119 [117, 121]),
  the 20 degree settle (350.8 [343.5, 353.0] ms against 362.4 [355.5,
  378.7]),
  the crawl swing (0.267 [0.257, 0.310] deg against 0.362 [0.334,
  0.384]), the 3 % ramp's reversals (164 [154, 166] a second against
  186 [167, 192]), the lag at 10 % (36.1 [35.9, 36.5] ms against 37.4
  [36.9, 38.7]), the hold jitter and the 1 % ramp's error sd. Higher:
  the duty of the 20 and the 60 degree move (24.1 [22.2, 24.2] %
  against 21.1 [20.2, 21.9], 23.9 [21.9, 24.3] against 21.4 [20.7,
  21.5]), the 60 degree move's current (147 [135, 150] mA against 131
  [126, 133]) and the 10 % ramp's current and duty (167 [164, 172] mA
  against 157 [145, 160], 26.3 [25.7, 27.0] % against 24.7 [22.7,
  24.7]). The reversals of the moves and of the 10 % ramp overlap.
  The set keeps the placement, a rule over this motor's inductance
  and resistance.
- The bisect of `16b` against `16a`, five arms of two rounds each,
  recorded in the half hour after those two. An arm is the set of
  record with one part of the plant pinned as the board held it:
  `zz-bisect-r-as-held`, `r_q12` at 4541 in place of 3744;
  `zz-bisect-ke-as-held`, `recip_ke_q` at 11817 in place of 13315;
  `zz-bisect-emf-pair-as-held`, those two together, the pair the
  back-EMF speed is formed with; `zz-bisect-model-block-as-held`,
  `b_i_q313`, `j_ff_q88` and `l3_q88` at 2657, 789 and 77 in place of
  2434, 862 and 84; `zz-bisect-friction-as-held`, `fric_fc_counts` at
  100 in place of 54. The four single arms and the current gains' side
  above cut `16a`'s eleven pins in groups, the two host records left
  out, which no loop reads. As medians, the two ends over three runs
  and an arm over two, the ranges in the result files:

  | row | `16b` | Ke | model block | friction | R | R and Ke | `16a` |
  |---|---|---|---|---|---|---|---|
  | reversals a second, 2 deg move | 187 | 169 | 164 | 325 | 62 | 66 | 99 |
  | reversals a second, 20 deg move | 293 | 216 | 336 | 396 | 165 | 132 | 116 |
  | reversals a second, 60 deg move | 316 | 177 | 332 | 358 | 110 | 86 | 131 |
  | reversals a second, 3 % ramp | 186 | 161 | 162 | 196 | 110 | 105 | 85 |
  | reversals a second, 10 % ramp | 320 | 252 | 319 | 345 | 167 | 122 | 106 |
  | overshoot, 2 deg move, deg | 0.15 | 0.14 | 0.04 | 0.32 | 0.11 | 0.24 | 0.41 |
  | overshoot, 20 deg move, deg | 0.16 | 0.13 | 0.12 | 0.02 | 0.50 | 0.59 | 0.54 |
  | overshoot, 60 deg move, deg | 0.01 | 0.02 | 0.00 | 0.05 | 0.11 | 0.41 | 0.71 |
  | settle, 20 deg move, ms | 362 | 335 | 358 | 345 | 907 | 536 | 564 |
  | rest error, deg | 0.37 | 0.34 | 0.30 | 0.32 | 0.36 | 0.24 | 0.16 |
  | lag, 1 % ramp, ms | 15.5 | 14.0 | 15.1 | 6.8 | 13.1 | 13.3 | 4.4 |
  | lag, 10 % ramp, ms | 37.4 | 34.6 | 37.9 | 36.6 | 32.6 | 30.5 | 29.7 |
  | crawl swing, deg | 0.36 | 0.32 | 0.33 | 0.40 | 0.47 | 0.61 | 0.69 |
  | speed report, 10 % ramp, deg/s | +2.8 | -4.1 | 0.0 | -3.2 | -10.3 | -12.0 | -4.1 |

  The back-EMF pair carries both halves of `16b` against `16a`. With
  the two as the board held them the reversals are near `16a`'s on
  every move and ramp, and so are the 20 degree overshoot, the crawl
  swing and the lag at 10 %. With the two as the record gives them,
  on `16b` and on the model block's and the friction's arms, the
  reversals are 1.7 to 3.4 times `16a`'s and the 20 and 60 degree
  landings are clean. `r_q12` is the larger share of
  both and `recip_ke_q` the smaller: the resistance alone as held
  takes the reversals down to 62 to 167 a second and brings the 20
  degree overshoot back, 0.50 deg, with a settle of 907 [313, 1500]
  ms over its two runs, and the back-EMF constant alone as held takes
  them to 161 to 252 with the landings as on `16b`. The friction term
  carries the lag of the crawl: as held, the lag at 1 % is 6.8 ms
  against 15.5 on `16b` and 4.4 on `16a`, and the 2 degree move pays
  for it, an overshoot of 0.32 deg and 325 reversals a second. The
  model block moves the 2 degree overshoot, 0.04 [0.03, 0.05] deg as
  held against 0.15 [0.11, 0.23], and little else. The rest error
  follows no single arm: 0.30 to 0.36 deg on each, 0.24 [0.18, 0.30]
  with the pair as held, 0.37 on `16b` and 0.16 on `16a`.
  A hypothesis that these rows fit and do not test: with the record's
  pair the back-EMF speed reads higher against the pot in the driven
  loop, so the speed loop is fed more speed than there is, which
  damps a landing more and puts more gain on the reading's noise, and
  the two halves are one effect.
- The current loop's bandwidth, two rounds in the order abc:
  `zz-sweep-current-loop-500hz`, `zz-sweep-current-loop-1000hz`, the
  set itself, and `zz-sweep-current-loop-1500hz`, the gains at 144,
  588 and 1176, at 289, 1176 and 2352 and at 433, 1764 and 3528. The
  rule was written before the session: the side with no landing, hold
  or lag row worse than the middle side in both rounds and the fewest
  reversals, a tie keeping the middle. The reversals scale with the
  bandwidth. At 500 Hz they are 0.65 to 0.70 of the middle's on the
  moves and the 10 % ramp (168 a second against 242 on the 2 degree
  move, 218 against 335, 243 against 366, 238 against 363), 0.88 at
  3 % (164 against 186) and the same at 1 %; at 1500 Hz they are 1.4
  to 1.7 times it on every move and ramp (365, 547 and 611 on the
  moves, 149, 254 and 556 on the ramps). 500 Hz loses the lag at 1 %
  and 3 %, 18.2 [17.9, 18.5] ms against 16.5 [15.9, 17.1] and 22.2
  [22.2, 22.3] against 21.4 [21.3, 21.5], and the median drive after
  an arrival, 62.98 [59.30, 66.65] ms against 51.43 [50.00, 52.85].
  1500 Hz
  gains the lag, 14.5 [13.8, 15.2], 20.6 [20.4, 20.7] and 35.3 [34.9,
  35.6] ms against 16.5, 21.4 and 38.7 [38.6, 38.8], and the drive
  after an arrival, 42.03 [39.00, 45.05] ms. By the rule 500 Hz is out,
  and of the other two the middle has the fewer reversals: 1 kHz
  stays.
- The observer's bandwidth, two rounds in the order abc:
  `zz-sweep-observer-10hz`, `zz-sweep-observer-15hz`, the set itself,
  and `zz-sweep-observer-25hz`, the gains at 5894, 1446 and 25, at
  8640, 3180 and 84 and at 13757, 8433 and 371, under the same rule.
  10 Hz has fewer reversals (153 a second against 293 on the 2 degree
  move, 304 against 390 on the 20 degree, 92 against 122 at 1 %, 138
  against 196 at 3 %), 9 to 22 % less current on every move and ramp
  and a crawl swing of 0.311 [0.251, 0.371] deg against 0.536 [0.528,
  0.544]. It loses the 20 degree landing, an overshoot of 0.151
  [0.130, 0.171] deg against 0 and a settle of 511 [386, 637] ms
  against 365 [363, 367], with the median drive after an arrival 68.63
  [65.55, 71.70] ms against 36.25 [30.25, 42.25], and one wide-code hold
  hunts in both rounds where none does at 15 Hz. 25 Hz reads 15 to
  84 % more reversals and 20 to 46 % more current on every move and
  ramp, a 2 degree overshoot of 0.253 [0.233, 0.273] deg against 0.151
  [0.109, 0.193] and a lag at 3 % of 22.6 [22.3, 23.0] ms against 21.6
  [21.4, 21.7]; its rest error is lower, 0.243 [0.195, 0.291] deg
  against 0.337 [0.329, 0.345], and so by a hair is the 3 % ramp's
  error sd. By the rule both ends are out: 15 Hz stays.
- The limits of motion, two rounds in the order abc:
  `zz-sweep-motion-75`, the set itself at the held 1500 count/s and
  3840, 74.5 deg/s and 1491 deg/s^2 on the table,
  `zz-sweep-motion-200`, 200 deg/s and 4000 deg/s^2, and
  `zz-sweep-motion-400`, the speed ceiling and 8000 deg/s^2, the
  position-error screen at 9.9 deg for 100 ms on every side. As
  medians over the two rounds and their range:

  | row | 75 deg/s | 200 deg/s | 400 deg/s |
  |---|---|---|---|
  | settle, 60 deg move, ms | 971 [967, 975] | 421 [417, 425] | 266 [262, 270] |
  | settle, 20 deg move, ms | 355 [348, 363] | 185 [171, 199] | 136 [135, 137] |
  | overshoot, 2 deg move, deg | 0.172 [0.071, 0.273] | 0.340 [0.267, 0.413] | 0.343 [0.314, 0.373] |
  | overshoot, 20 deg move, deg | 0.061 [0, 0.121] | 0 | 0.068 [0.016, 0.121] |
  | overshoot, 60 deg move, deg | 0 | 0 | 0.095 [0.065, 0.124] |
  | reversals a second, 60 deg move | 340 [313, 367] | 638 [629, 647] | 659 [627, 691] |
  | current rms, 60 deg move, mA | 135 [128, 142] | 211 [208, 214] | 290 [288, 292] |
  | worst drive after an arrival, ms | 95 [88, 102] | 150 [109, 191] | 254 [176, 332] |
  | wide-code holds that hunt | 0 | 0.5 [0, 1] | 0 |

  The rule, written before the session: the fastest side with no
  fault, a peak following error at most two thirds of the screen, and
  no overshoot or settle row worse than the 75 deg/s side in both
  rounds. The peak following error is no row of the result files, and
  the session's own reading of it is not carried here. 400 deg/s is
  out on the rows: its 2 and 60 degree overshoot are worse than the
  slow side's in both rounds, and its 60 degree move's current, 290
  mA rms, is over the 250 mA continuous current on a free horn. 200
  deg/s has no overshoot or settle row worse in both rounds, the 2
  degree overshoot's ranges overlapping by a hair, settles the 60
  degree move in under half the time, and runs it at 211 mA. So the
  session recommends 200 deg/s and 4000 deg/s^2, interim until the
  load arm. Against it: the reversals are 1.8 to 1.9 times the slow
  side's on the moves, the worst drive after an arrival
  (`landing_drive_ms_worst`) is longer in both rounds, 150 [109, 191]
  ms against 95 [88, 102], a landing row the rule's overshoot and
  settle do not count, the 2 degree overshoot leans up, and one wide-code
  hold hunted in one round of two. The ramps' rows move with the
  limits too, the lag at 1 % 17.4 [17.4, 17.5] ms at the held limits,
  9.8 [7.4, 12.2] at 200 deg/s and 4.7 [3.3, 6.2] at 400, with the
  ramp's current at 82, 109 and 166 mA, so a rung at other limits
  does not compare with the rungs at these. The owner took 200 deg/s
  and 4000 deg/s^2 as the default move speed, interim until the load
  arm, and a rung runs the test set's own limits whatever the default
  ([The constant set](#the-constant-set)).

The speed blend's sessions:

Three firmwares: main a7d3b377; 06059d1e, the
speed blend as a commit on a7d3b377, whose velocity loop reads the pot
observer's speed plus the back-EMF's fast part, merged since as
2e9cb678 with the same image; and e9b33fd6, a bench experiment on
06059d1e whose velocity loop reads the pot observer's speed alone
("pot only") and which never merges. Every side ran on set
`b300d5e71e38`, or on it with one deviation, on a free horn, and no
run raised a fault. Their result
files are kept and their rows are not. The host limits what their
follow rows can say: a run's ramp goals that went out late
(`follow_late_items`) are given with each session, and a lag or a
follow row carries them with the servo's following. As the median of
the runs and their range, a row named having its ranges apart unless
said.

- `zz-ab-main-a7d3b377` and `zz-ab-speed-blend`, three rounds in the
  order ab, after ten 120 degree sweeps. 28 [22, 32] and 25 [22, 31]
  of a run's ramp goals went out late. The blend's duty reverses far
  less often, 13.5 [8.9, 30.4] a second against
  233 [204, 240] on the 2 degree move, 24.9 [21.8, 35.8] against 221
  [217, 245] on the 20 degree, 17.2 [15.4, 17.8] against 253 [245,
  267] on the 60 degree, 61.6 [58.9, 79.9] against 138 [113, 146] at
  3 % and 65.5 [46.5, 70.0] against 240 [238, 245] at 10 %, and 71.4
  [58.0, 77.8] against 88.0 [84.8, 88.5] at 1 %. Its current is lower
  on every move and ramp, 94.2 [93.4, 97.7], 88.9 [88.4, 95.2] and
  90.9 [89.7, 91.8] mA against 128 [125, 135], 132 [131, 139] and 131
  [130, 131] on the moves, 71.8 [71.1, 72.5], 79.9 [79.1, 81.7] and
  106 [99.3, 113] against 76.9 [75.8, 77.1], 92.7 [88.6, 96.0] and 165
  [151, 180] at 1, 3 and 10 %. The 60 degree move settles in 893 [887,
  896] ms against 951 [950, 962], the median drive after an arrival is
  36.2 [33.3, 37.0] ms against 52.1 [47.6, 79.5] and the worst one 83.6
  [81.1, 88.0] against 154 [111, 1010]; the lag is 18.7 [18.5, 19.0] ms
  against 20.3 [19.6, 20.4] at 3 % and 33.7 [33.5, 34.5] against 36.2
  [36.0, 36.3] at 10 %; the speed report at 10 % reads 2.57 [0.20,
  3.91] deg/s against 14.7 [10.3, 15.2]. One row loses: the follow
  error's sd at 10 %, 0.488 [0.479, 0.498] deg against 0.454 [0.426,
  0.463]. The overshoots overlap, 0.112 [0.009, 0.112], 0.078 [0,
  0.090] and 0.006 [0, 0.009] deg against 0.155 [0.050, 0.252], 0.130
  [0.050, 0.130] and 0 [0, 0.211], and so does the rest error, 0.309
  [0.304, 0.361] deg against 0.287 [0.268, 0.304]. Main's side against
  `17a`, firmware 08c7beaf under an hour before it and not alternated,
  reverses less often, 221 [217, 245] a second against 312 [303, 326]
  on the 20 degree move, 253 [245, 267] against 339 [332, 373] on the
  60 degree and 240 [238, 245] against 310 [265, 337] at 10 %, settles
  the 60 degree move in 951 [950, 962] ms against 976 [971, 977], and
  overshoots the 20 degree move, 0.130 [0.050, 0.130] deg against 0
  [0, 0]: another session, so a soft read.
- `zz-ab3-main-a7d3b377`, `zz-ab3-speed-blend` and `zz-ab3-pot-only`,
  three rounds in the order abc. 2 [0, 15], 3 [0, 47] and 64 [22, 98]
  of a run's ramp goals went out late, so pot only's follow rows carry
  far more of the host than the other two. The blend against main
  reads as in the first session: the duty reverses 63.5 [41.6, 67.5],
  46.2 [32.7, 50.4] and 25.3 [25.1, 36.0] times a second against 212
  [208, 229], 256 [237, 257] and 289 [256, 301] on the 2, 20 and 60
  degree moves, and 79.8 [79.6, 84.8], 83.7 [79.8, 92.6] and 67.0
  [56.1, 73.3] against 101 [99.7, 111], 160 [156, 167] and 259 [253,
  273] at 1, 3 and 10 %; the current of the 20 and 60 degree moves is
  101 [99.3, 125] and 94.7 [94.1, 96.7] mA against 147 [143, 162] and
  142 [136, 147], with the 2 degree move's overlapping, 102 [99.8,
  130] against 127 [126, 141]; the ramps take 73.3 [71.9, 73.4], 83.2
  [83.0, 87.5] and 123 [115, 125] mA against 76.4 [75.9, 91.3], 97.2
  [94.1, 99.3] and 174 [170, 174]; the 60 degree move settles in 902
  [896, 914] ms against 964 [946, 965]; the lag is 18.3 [18.0, 18.6]
  ms against 20.3 [19.8, 20.7] at 3 % and 34.7 [34.6, 35.0] against
  36.4 [36.4, 36.9] at 10 %; and the follow error's sd at 10 % loses
  again, 0.503 [0.486, 0.521] deg against 0.455 [0.449, 0.462]. The 2
  degree overshoot leans up, 0.394 [0.193, 0.615] deg against 0.193
  [0.109, 0.233], and the speed report at 10 % overlaps, -1.9 [-13.8,
  3.8] deg/s against 6.7 [-1.1, 12.1].
  The blend's two sides differ from each other, the second against
  the first: the 2 degree overshoot, 0.394 [0.193, 0.615] deg against
  0.112 [0.009, 0.112]; the reversals of the 2 and the 60 degree move,
  63.5 [41.6, 67.5] and 25.3 [25.1, 36.0] a second against 13.5 [8.9,
  30.4] and 17.2 [15.4, 17.8]; the current of every move, 102 [99.8,
  130], 101 [99.3, 125] and 94.7 [94.1, 96.7] mA against 94.2 [93.4,
  97.7], 88.9 [88.4, 95.2] and 90.9 [89.7, 91.8], and of the 10 % ramp,
  123 [115, 125] against 106 [99.3, 113]; the worst drive after an
  arrival, 200 [133, 296] ms against 83.6 [81.1, 88.0]; the crawl
  swing, 0.486 [0.483, 0.521] deg against 0.238 [0.231, 0.246]; and the
  rest error, lower, 0.248 [0.216, 0.303] deg against 0.309 [0.304,
  0.361]. Main's two sides moved the same way on the current of the 20
  and 60 degree moves, 147 [143, 162] and 142 [136, 147] mA against
  132 [131, 139] and 131 [130, 131], and on the crawl swing, 0.369
  [0.363, 0.571] deg against 0.217 [0.207, 0.250].
  Pot only against the blend keeps the low reversals: 40.2 [36.2,
  63.2], 53.3 [52.0, 55.6] and 36.4 [25.5, 37.0] a second on the
  moves, the 20 degree's alone apart and higher, and 79.1 [79.0, 91.0],
  80.9 [72.5, 85.1] and 69.2 [59.3, 95.1] on the ramps, overlapping.
  It loses the landings: the 20 and 60 degree overshoot, 0.795 [0.795,
  0.845] and 0.773 [0.515, 0.832] deg against 0.068 [0.068, 0.556] and
  0.009 [0, 0.071]; the 20 degree settle, 1149 [1013, 1468] ms against
  357 [339, 496], and the 60 degree's 1995 [904, 2000] against 902
  [896, 914], the ranges overlapping; 2 [1, 2] landings of 13 driven in their tail against 0
  [0, 0]; the worst drive after an arrival, 1156 [863, 1199] ms
  against 200 [133, 296]. Its current is higher, 218 [210, 239] and
  181 [116, 184] mA against 101 [99.3, 125] and 94.7 [94.1, 96.7] on
  the 20 and 60 degree moves, 81.7 [79.3, 156], 97.9 [95.7, 136] and
  145 [144, 173] against 73.3 [71.9, 73.4], 83.2 [83.0, 87.5] and 123
  [115, 125] at 1, 3 and 10 %, and so is the crawl swing, 0.540
  [0.535, 1.035] deg against 0.486 [0.483, 0.521]. Its lag is shorter,
  11.9 [11.2, 12.7] ms against 15.0 [12.9, 15.0] at 1 %, 17.4 [17.0,
  17.7] against 18.3 [18.0, 18.6] at 3 % and 30.7 [30.6, 30.8] against
  34.7 [34.6, 35.0] at 10 %.
- `zz-ab-400-main-a7d3b377`, `zz-ab-400-speed-blend` and
  `zz-ab-400-pot-only`, two rounds in the order abc, each side on the
  set with `motion-400.json`, 400 deg/s and 8000 deg/s^2, the side
  `zz-sweep-motion-400` ran. The file is history now, and the three
  results name it by its sha1 and the two fields it moved, 8051 and
  20610 as they held them, a point the set of record does not hold
  ([Running it](#running-it)): they stay the sides they ran. 31.5 [18,
  45], 19.5 [16, 23] and 17.5 [16, 19] of a run's ramp goals went out
  late. The blend against main reverses less often on the 20 and 60
  degree moves, 100 [95.3, 105] and 49.6 [46.4, 52.8] a second against
  476 [428, 525] and 569 [563, 574], and overlaps on the 2 degree, 327
  [210, 445] against 406 [364, 449]; its current is lower on every
  move, 195 [193, 198], 242 [241, 243] and 210 [201, 220] mA against
  268 [256, 280], 278 [260, 296] and 284 [281, 287], and so is the worst
  drive after an arrival, 94.7 [86.2, 103.2] ms against 211 [164,
  257], and the speed ripple of a move, 13.2 [12.8, 13.6] % against
  17.9 [17.4, 18.3]. Its overshoot is HIGHER than main's: 0.231 [0.211,
  0.252] deg against 0.008 [0, 0.016] on the 20 degree move and 0.456
  [0.397, 0.515] against 0.213 [0.183, 0.242] on the 60 degree. The
  settle times overlap, 137 [134, 140] ms against 137 [122, 151] on
  the 20 degree move and 264 [240, 287] against 254 [250, 259] on the
  60 degree. Its lag at 1 % and 3 % is about twice main's, 22.8 [22.8,
  22.8] ms against 10.4 [9.4, 11.4] and 21.7 [21.6, 21.8] against 9.3
  [9.2, 9.4]. Pot only against the blend reverses 4 to 5 times as
  often on the 20 and 60 degree moves, 505 [503, 508] and 220 [210,
  229] a second against 100 [95.3, 105] and 49.6 [46.4, 52.8], and less
  often on the 2 degree, 96.1 [82.1, 110] against 327 [210, 445]; its
  60 degree overshoot is twice the blend's, 0.911 [0.814, 1.009] deg
  against 0.456 [0.397, 0.515]; its current is higher on every move,
  288 [278, 299], 286 [278, 293] and 251 [246, 255] mA; its worst drive
  after an arrival is 218 [188, 248] ms against 94.7 [86.2, 103.2] and
  its crawl swing 0.708 [0.658, 0.757] deg against 0.314 [0.281,
  0.346].
- The velocity test, two rounds of the three images, outside the
  scorecard. In Velocity mode the servo holds a commanded speed on its
  own estimate, with no position loop to correct it, so the pot's
  speed over the commanded one is the estimate's level, read directly.
  A capture per image, speed and round: a hold of 300 ms, then goals
  of one speed alternating in sign, a half leg first and last and three
  full legs between, then a goal of 0. The full legs are 3000 ms at 6
  and 19 deg/s, 1200 ms at 62, 500 ms at 200 and 300 ms at 400; the
  three slower speeds ran on the set as it stands, the two faster on
  the set with `motion-400.json`. `uv run python -m
  oscnb.velocity_test` reads every full leg over the middle of its
  span, 30 to 85 %: the pot's slope
  through the position table over the commanded speed, the ripple of
  the slopes in windows of an eighth of the middle, the duty's reversals a
  second, and the current rms over the capture's own zero, read the
  way a current row of the scorecard is. A cell is the median of its
  six legs and their range. Its result,
  [velocity-test/results/](velocity-test/results), is read here; the
  captures, with the two sets they ran on, lie under
  `telemetry/<pair>/velocity-test/` and are not kept, as a `zz-`
  rung's rows are not. No fault on any capture. The pot's speed over
  the commanded:

  | deg/s | a7d3b377, main | 06059d1e, the blend | e9b33fd6, pot only |
  |---|---|---|---|
  | 6.01 | 0.990 [0.963, 1.029] | 0.993 [0.956, 1.023] | 0.994 [0.980, 1.003] |
  | 18.98 | 0.977 [0.961, 0.993] | 0.996 [0.994, 0.997] | 1.000 [0.999, 1.001] |
  | 62.01 | 0.913 [0.910, 0.916] | 1.003 [1.000, 1.006] | 0.998 [0.998, 1.000] |
  | 199.98 | 0.939 [0.934, 0.945] | 0.995 [0.985, 0.997] | 0.995 [0.994, 0.998] |
  | 400.02 | 0.935 [0.915, 0.940] | 0.984 [0.963, 0.989] | 0.971 [0.970, 0.978] |

  The duty's reversals a second, the speed ripple in % and the current
  in mA, as medians:

  | deg/s | a7d3b377, main | 06059d1e, the blend | e9b33fd6, pot only |
  |---|---|---|---|
  | 6.01 | 16, 44.2, 74 | 4, 34.2, 75 | 7, 51.0, 72 |
  | 18.98 | 31, 4.3, 67 | 8, 2.0, 61 | 9, 2.8, 62 |
  | 62.01 | 264, 3.9, 128 | 9, 3.7, 89 | 8, 2.5, 87 |
  | 199.98 | 498, 5.7, 209 | 0, 5.3, 100 | 51, 3.2, 151 |
  | 400.02 | 803, 9.0, 306 | 0, 5.3, 147 | 24, 6.4, 191 |

  Pot only's reversals at 200 deg/s range from 0 to 385 a second over
  its six legs.

What was written before the sessions, and how each held:

- The blend's reversals on the 20 and 60 degree moves and the 10 %
  ramp fall by a third or more in all three rounds: held, the blend's
  highest run under a third of main's lowest on each, in both
  sessions.
- The blend's cruise within 3 % of the commanded speed, on the 60
  degree move, read off the raw pot: not shown. That read is no row of
  the result files, and the raw pot's slope is not the table's count;
  the velocity test reads the speed through the table.
- The landings, at risk on the blend, with no sign predicted: no
  overshoot or settle row of the blend's is worse than main's in all
  three rounds of the first session, and none comes apart worse in the
  second; the 60 degree move settles sooner in both. By the rule
  written before the first session the blend passes.
- Pot only keeps the low reversals and loses where the back-EMF's fast
  part matters, overshoot or settle of the 20 and 60 degree moves,
  the hold jitter or the follow error's spread: held, on the overshoot
  and settle.
- At 400 deg/s the blend has fewer reversals than main: held on the 20
  and 60 degree moves, not on the 2 degree, where the ranges overlap.
  No sign predicted for the overshoot: the blend's is higher.
- At 400 deg/s blend and pot only read alike: did not hold. Pot only
  reverses 4 to 5 times as often on the 20 and 60 degree moves and
  overshoots the 60 degree move twice as far.
- In Velocity mode main runs 5 % or more under the commanded speed at
  62 deg/s and above: held, 8.7, 6.1 and 6.5 % under by the median.
- Blend and pot only run within 2 % of the commanded at every speed,
  and alike: held by the median, 0.984 to 1.003 on the blend and 0.994
  to 1.000 on pot only, but for pot only at 400 deg/s, 2.9 % under,
  where a full leg is 300 ms and its middle 165 ms. At 6 deg/s, where
  no sign was predicted, the lowest leg reads 3.7, 4.4 and 2.0 % under
  the commanded on main, the blend and pot only.
- Main's reversals in Velocity mode are well above the other two:
  held, 264, 498 and 803 a second at 62, 200 and 400 deg/s against 0
  to 51.

The lifecycle head's session:

Two dead-man images, 2e9cb678, `18a`'s, and 32ea2f31, `19a`'s. Every
side ran on set `324e326794b5` as the servo has it saved, on a free
horn, and no run raised a fault. The scorecard's sides ran at the test
set's own limits of motion, after ten 120 degree sweeps. Their result
files are kept and their rows are not. As the median of three runs and
their range, a row named having its ranges apart unless said.

- `zz-ab-blend-head` (2e9cb678) and `zz-ab-lifecycle-head`
  (32ea2f31), three rounds in the order ab, recorded in the half hour
  after `19a` because `19a` and `18a` were not recorded together. 16
  [2, 41] and 21 [11, 22] of a run's ramp goals went out late. The 60
  degree move's current overlaps, 92.6 [90.6, 92.8] mA on 32ea2f31
  against 90.9 [90.8, 94.2], so 2e9cb678 reads about 91 mA in this
  session too. 32ea2f31 reads worse on the 2 degree move, its settle
  66.0 [65.5, 67.0] ms against 62.3 [61.2, 64.3] and its reversals
  42.1 [41.6, 43.8] a second against 31.7 [8.3, 32.0], and on the
  speed report at 3 %, -1.74 [-2.78, -1.39] deg/s against -0.53
  [-1.17, -0.10]. It reads better on the lag at 1 %, 15.1 [14.5, 15.4]
  ms against 16.2 [15.5, 16.8], in degrees too, and on the 1 % ramp's
  current and duty, 68.8 [67.3, 69.2] mA against 69.8 [69.5, 72.1] and
  8.02 [7.88, 8.04] % against 8.12 [8.08, 8.40].
- `zz-ab2-lifecycle-head` and `zz-ab2-blend-head`, the same three
  rounds again in the order ba, recorded next. 15 [0, 22] and 10 [1,
  67] of a run's ramp goals went out late. 32ea2f31's 2 degree move
  reverses more often again, 39.9 [31.9, 74.9] a second against 21.4
  [8.7, 29.1], and its 2 degree settle overlaps, 67.0 [59.1, 67.6] ms
  against 66.9 [66.6, 67.7]. Its 60 degree move takes less current,
  88.9 [88.7, 89.7] mA against 90.7 [89.7, 91.7], by a hair. It reads
  worse on the 3 % ramp's current and duty, 80.2 [79.6, 82.9] mA
  against 78.7 [78.2, 79.3] and 11.17 [11.14, 11.31] % against 10.97
  [10.85, 11.02], and on the speed report at 1 %, -3.20 [-4.39, -2.96]
  deg/s against -2.53 [-2.73, -1.23]. Across the two sessions
  2e9cb678 alone reads the 2 degree settle at 62.3 and 66.9 ms. The 2
  degree move's reversals read 31.7 and 21.4 a second on 2e9cb678 and
  38.9 on `18a`, 42.1 and 39.9 on 32ea2f31 and 27.4 on `19a`. Of the
  rows apart in either session, the 2 degree move's reversals alone
  are apart the same way in both.
- The 2 degree move's reversals counted, outside the scorecard. A
  scorecard run holds four 2 degree moves of about 60 ms, so the row
  stands on a few reversals a run. Each capture here holds forty 2
  degree moves, `goal:1956@300,goal:1916@300` twenty times from the
  scorecard's center after a first goal of 1916 held 600 ms, in
  Position mode at the set's default limits of motion, 4025 count/s and
  10305 as the fields hold them. Six captures, the two images
  alternated three times, 2e9cb678 first. The count is the duty's sign
  reversals by the scorecard's own definition, zero rows skipped, over
  the first 100 ms
  after each goal, summed over the forty moves. In the order recorded
  it reads 318, 278 and 355 on 2e9cb678 against 239, 291 and 384 on
  32ea2f31, the means 317 and 305, the two sets of three overlapping.
  Over the whole 300 ms of each goal it reads 324, 285 and 364 against
  242, 300 and 395. 32ea2f31's three rise in the order recorded, and
  each image's last capture reads the highest of its three. No fault
  on any capture. The captures lie under
  `telemetry/<pair>/reversals-2deg/`, one folder a round and an image
  in it, and are not kept, as the velocity test's are not.
- Before the scorecard each image ran ten 120 degree sweeps from rest,
  the winding estimate within 0.05 C of the board sensor at the start,
  the estimate polled at the end of every leg and the sensor read
  before the sweeps and after the 12 s of holds that followed them.
  Against the sensor taken as a line between those two reads, the
  estimate peaked 0.46 C over it on 2e9cb678 and 0.47 C on 32ea2f31,
  and its mean over the twenty legs was 0.32 C against 0.30, 6 %
  apart. The plan written before the session asked for the two within
  15 % of each other.
- The kernel's `budgets` probe on 32ea2f31's bench image passed three
  times.

What the session leaves on record, for reading the ladder:

- A rung recorded hours from its reference carries no verdict on its
  own rows. `19a`'s one loss against `18a`, the 60 degree current, is
  not apart for the worse in either back to back session, where
  2e9cb678 reads it at 90.9 and 90.7 mA against `18a`'s 87.1.
- A row that stands on a few events per run has a range over three
  runs narrower than its scatter between sessions. The 2 degree move's
  settle on 2e9cb678 alone reads 62.3 and 66.9 ms in two sessions
  recorded one after the other, and its reversals, apart the same way
  in both sessions, are not apart when counted over forty moves.

The loop-margin sessions:

The servo ran the dead-man image of firmware ec3051ea, which carries
the loop-margin stimulus. The captures carry `fw` 65 and not the
commit. Servo mg90-d on a free horn, on the set the servo has saved,
`324e326794b5`: every capture's `meta.json` holds that set's CONFIG,
and the captures on the saved set carry the plant stamp 0x6f9a with
the verdict `match`. A sweep is one `osc-lab margin` call: in Velocity
mode at a cruise, a triangle on the velocity loop's reference, one
burst up and one down the travel a point, each point a half period N
of the triangle, read at 1000 / N Hz. `uv run python -m oscnb.margin
<capture> [<second>]` reads a sweep: each loop's gain and phase a
point with a jackknife standard error, a point too noisy to place a
crossing marked `unreadable` and left out of the margins and the
repeats, the crossover with its phase margin, and the gain margin
where one is read. Given a second sweep it says whether the two
repeat by its own rule: each point both read within three of the
pair's combined standard errors in dB and in deg, the phase margins
within 5 deg, the gain margins within 1 dB or both at least 12 dB.
Every number below is its reading of the captures,
`telemetry/<pair>/margin/capture-1` to `capture-20`. Every sweep
ended `ok`, with no row dropped, no frame missing and the stimulus
fields at 0 after it.

The numbers written before the sessions and in their running record
were read by the estimator before it formed the cross spectrum a
cycle and carried a noise a point. Where the two differ, the reading
here stands. Three readings changed in kind: the two-cruise check
flags none of the position points, where it flagged 4 of 8; the pair
at `r_q12` 5470 reads no crossover and repeats, where it read one at
61.8 Hz and did not repeat; and the 1 kHz point is unreadable in both
sweeps of the method.

- The method, `capture-1` and `capture-2`, one after the other: the
  default plan, the speed band (N = 40, 30, 25, 20, 15, 12, 10, 8 and
  1, 25 Hz to 1 kHz) and the position band (N = 512 to 50, 2 to 20
  Hz) at 742 c/s, and the position band again at 928 c/s, the
  two-cruise check. The speed loop crosses over at 52.9 and 52.7 Hz
  with a phase margin of 52.9 and 53.6 deg, the position loop at 2.9
  and 2.8 Hz with 87.3 and 85.9 deg. Neither loop's gain margin is
  read, its -180 crossing possibly next to an unreadable point. Both
  loops repeat, on 8 points each, the worst 0.6 of the pair's combined
  standard errors in dB and 0.7 in deg. Every position point agrees
  between the two cruises in both sweeps. The 1 kHz point reads
  -17.80 +/- 1.62 dB and -44.03 +/- 43.75 dB, unreadable in both; its
  burst's peak is 15 c/s by the plan. The plan's source check
  (`source_checks` in `meta.json`) reads `omega_hat_src` 1, the source that carries the back-EMF's fast part,
  on 33.7 and 36.9 % of 1000 periods at 742 c/s and 30.2 and 30.3 % at
  928.
- The fast cruise, `capture-3` and `capture-4`: the speed band alone at
  2500 c/s, `omega_hat_src` 1 on 100 % of periods. The band holds no
  crossover: the gain reads 18.50 and 18.63 dB at 25 Hz and 6.06 and
  6.95 dB at 125 Hz, the phase -96 to -110 deg from 25 to 125 Hz. The
  1 kHz point is unreadable in both. The pair repeats on 8 points.
- The speed loop by cruise, on the extended band (N = 40, 25, 15, 10,
  8, 7, 6, 5, 4, 3 and 2, 25 to 500 Hz), two sweeps a cruise on the
  saved set. The share is that of `omega_hat_src` 1 in the source
  check:

  | capture | c/s | share | crossover, Hz | phase margin, deg | gain margin | 125 Hz, dB | 333 Hz, dB |
  |---|---|---|---|---|---|---|---|
  | 9 | 742 | 38.9 % | 53.9 | 55.5 | not read | -6.89 | -13.24 |
  | 10 | 742 | 38.0 % | 53.8 | 57.4 | not read | -7.83 | -12.89 |
  | 7 | 1500 | 63.8 % | 52.9 | 66.6 | not read | -7.47 | -5.81 |
  | 8 | 1500 | 66.8 % | 51.7 | 60.9 | not read | -7.04 | unreadable |
  | 5 | 2500 | 100 % | 283.7 | 43.6 | 4.00 dB at 457.6 Hz | 6.18 | -1.45 |
  | 6 | 2500 | 100 % | 281.8 | 44.8 | not read | 6.32 | -1.40 |
  | 15 | 3000 | 100 % | 265.9 | 50.0 | not read | 6.29 | -1.73 |
  | 16 | 3000 | 100 % | 275.9 | 44.1 | 2.79 dB at 444.6 Hz | 5.12 | -1.56 |
  | 17 | 3500 | 100 % | 283.5 | 41.0 | not read | 6.52 | -1.34 |
  | 18 | 3500 | 100 % | 295.1 | 38.9 | not read | 6.73 | -1.14 |

  The pairs at 742, 2500 and 3500 c/s repeat. The pair at 1500 c/s
  does not: its 500 Hz point is 34.6 deg apart, 5.2 standard errors,
  and its phase margins 5.7 deg. The pair at 3000 c/s does not: every
  point repeats, and the phase margins are 5.8 deg apart. At 742 and
  1500 c/s the crossover reads 51.7 to 53.9 Hz. Above it the gain
  falls at 742 c/s, from -5.12 and -6.27 dB at 100 Hz to -13.24 and
  -12.89 dB at 333 Hz, and at 1500 c/s it reads -4.5 to -7.6 dB at
  every readable point from 100 to 500 Hz. At 2500 to 3500 c/s, over
  the six sweeps, the crossover reads 266 to 295 Hz with a phase
  margin of 39 to 50 deg, and a gain margin of 2.8 to 4.0 dB where one
  is read, in two sweeps of six. A gain rule that holds a loop to 45
  deg of phase margin and 6 dB of gain margin would find five of those
  six under 45 deg, all but `capture-15`, and both gain margins read
  under 6 dB.
- The resistance test at 2500 c/s: the extended sweep with `r_q12`
  installed volatile at 4290 (`capture-13` and `-14`), 4837 (`-11`
  and `-12`) and 5470 (`-19` and `-20`), against the saved set's 3744
  (`capture-5` and `-6`). Each install is the one pin of a deviation
  file beside the captures, `telemetry/<pair>/margin/margin-r-4p8-ohm.json`,
  `margin-r-5p4-ohm.json` and `margin-r-6p1-ohm.json` in that order. The
  captures do not carry `r_q12`; they carry the plant stamp of the
  install, 0x75c3, 0xa1a2 and 0x1fdb, where the saved set's is 0x6f9a.
  `capture-15` to `-18`, recorded between the 4290 and the 5470 pairs,
  carry 0x6f9a again.

  | `r_q12` | captures | 125 Hz, dB | 250 Hz, dB | crossover, Hz | phase margin, deg | repeat |
  |---|---|---|---|---|---|---|
  | 3744 | 5, 6 | 6.18, 6.32 | 1.13, 0.99 | 283.7, 281.8 | 43.6, 44.8 | repeats |
  | 4290 | 13, 14 | 4.21, 4.80 | 0.28, 0.10 | 257.8, 252.7 | 54.9, 58.2 | does not repeat |
  | 4837 | 11, 12 | 1.71, 1.75 | -0.38, -2.70 | 193.3, 193.3 | 84.3, 84.6 | does not repeat |
  | 5470 | 19, 20 | -1.55, -2.11 | -1.37, -2.68 | not read | not read | repeats, on 6 points |

  At 4290 the 500 Hz point's phase is 19.8 deg apart, 6.4 standard
  errors, and at 4837 the 250 Hz point's gain 2.32 dB, 3.4. At 5470
  the estimator reads no crossover in either sweep, its 0 dB crossing
  possibly next to an unreadable point, with 3 of 11 points unreadable
  in each; the phase at 125 Hz reads -61.1 and -62.7 deg. On those two
  sweeps the estimator drops cycles as pauses, a step of the
  reference from one period to the next that is not the slope: 2 to
  21 cycles a point on `capture-19` and 0 to 16 on `capture-20`, none
  for a missing row. `capture-1` to `-18` drop none.
- Two more records of the same night, on the same image and the saved
  set, outside the margin sweeps. `openloop-legs` is three chains of
  `osc-lab sweep --duty-pct 18@600,then:12@600,then:8@600`, open-loop
  duty of 18, 12 and 8 % for 600 ms each in both directions, recorded
  for the volts balance with no loop. `mode-edge` is nine `osc-lab
  track` captures with a `mode:` item this repo's `osc-lab` does not
  have, three rounds of Velocity mode at 800 c/s switched to Position
  under torque at mid travel in each direction and of OpenLoop duty at
  12 % switched to Current mode at 75 counts. Their readers are not in
  this repo, so no number of theirs is carried here. The captures lie
  under `telemetry/<pair>/openloop-legs/` and
  `telemetry/<pair>/mode-edge/` and are not kept, as the velocity
  test's are not.

What was written before the sessions, and how each held:

- The method on silicon: every point of both bands that the two-cruise
  check does not flag within 1 dB and 10 deg between the two sweeps,
  the phase margins within 5 deg, the gain margins within 1 dB or both
  at least 12 dB, at most one flagged point a band, no row dropped, no
  fault and the stimulus fields at 0 after each sweep. Held but for the
  1 kHz point, which reads unreadable in both sweeps and is left out:
  the other points of both bands are within 0.90 dB and 5.6 deg, no
  point is flagged, and no gain margin is read to compare.
- The source check at 742 c/s, recorded with no verdict: `omega_hat_src`
  0 was expected on at least 99 % of periods. It reads 1 on 33.7 and
  36.9 %, and on 38.9 and 38.0 % in the extended band's sweeps, so the
  slow cruise is a mixed regime on this servo, not the pot observer
  alone.
- Predictions for the loops as the set holds them, from the sim and a
  linear model: the speed loop at 742 c/s at 56 Hz, 42 to 45 deg and a
  gain margin of 24 to 27 dB at Nyquist, the position loop at 3.0 Hz
  and 88 deg, the speed loop at 2500 c/s at 46 Hz, 31 deg and 13 dB. A
  crossover within 25 % and a phase margin within 10 deg was to say the
  model holds. At 742 c/s every crossover is within 25 % and the
  position loop's phase margins within 10 deg; the speed loop's, 52.9
  and 53.6 deg, are 7.9 and 8.6 deg over the top of its range; no gain
  margin is read. At 2500 c/s it did not hold: no crossover in the
  default band, then 283.7 and 281.8 Hz on the extended band.
- The fast cruise: `omega_hat_src` 1 on at least 99 % of periods, held
  at 100 % at 2500, 3000 and 3500 c/s; the two sweeps at 2500 c/s to
  repeat, held on the default and on the extended band.
- The extended band at 742, 1500 and 2500 c/s: no prediction, a map.
- The resistance test: at 4837 the gain from 125 to 250 Hz to fall by
  at least 6 dB from the saved set's with the crossover under 150 Hz,
  and at 4290 to land between; under 2 dB at 4837 to rule the
  resistance out, 2 to 6 dB to give it a share. The 125 Hz gain fell
  by 4.5 dB, from 6.18 and 6.32 to 1.71 and 1.75, the 250 Hz gain by
  1.5 and 3.7 dB, and the crossover read 193.3 Hz in both: a share by
  that reading. 4290 lands between, 4.21 and 4.80 dB at 125 Hz.
- Two tests of an analysis outside this repo. At 3000 and 3500 c/s on
  the saved set, a 125 Hz gain at 3500 c/s within 1 dB of 2500's was
  to say that what lifts the gain at speed is fixed in speed, at or
  over 8 dB that it grows with it: 6.52 and 6.73 dB against 6.18 and
  6.32, fixed by that reading. At `r_q12` 5470 at 2500 c/s, -2.3 dB and
  -70 deg at 125 Hz and a crossover at 57 Hz were predicted; a 125 Hz
  gain within 1.5 dB of -2.3 with a crossover under 100 Hz was to
  confirm the analysis, a 125 Hz gain over 0 dB or a crossover over 150
  Hz to falsify it. The gain reads -1.55 and -2.11 dB and no crossover
  is read: neither confirmed nor falsified.

What the sessions leave open:

- What in the winding or the drive lifts the speed loop's gain at 2500
  c/s and over. Raising `r_q12` moves it by a share. An
  analysis outside this repo fits it, and its numbers are not carried
  here; a notebook over these captures would be where it is tested.
- The fix for the speed loop at those cruises.
- What drops the cycles of the 5470 pair as pauses, which no other
  sweep shows.
- The speed loop's gain margin at Nyquist at 742 c/s: the 1 kHz point
  is unreadable at its burst's peak of 15 c/s.
