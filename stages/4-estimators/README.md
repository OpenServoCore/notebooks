# Stage 4. Estimators

The real-time estimators, designed offline. Stage 3 scored the speed and
the position as means over windows of 100 ms and longer, with a current
zero taken from each sweep's own coast. A loop cannot wait for a window or
look ahead to a coast. This stage runs the same quantities the way a
controller has to, one row at a time, and says what each one costs in
bias, noise and delay.

There are no captures. Every stage 3 sweep already holds one row per PWM
period at 20 kHz, which is the control tick, so an estimator is a function
run down the rows of a sweep and graded against what stage 3 graded
against: the ripple tach, the camera's tag, and the sweep's own coast. The
servo is `mg90-d` on `dev-v006-2A-D`, the data is
`telemetry/mg90-d__dev-v006-2A-D/`, and each notebook writes
`results/mg90-d__dev-v006-2A-D.json` in its folder.

## The rules of the desk

- An estimator is causal. At a row it reads that row and earlier ones. The
  truth it is graded against is offline and may look ahead.
- The arithmetic is float. A notebook states the state an estimator keeps,
  the constants it reads and the rate it runs at, which is what a
  fixed-point port needs. The port itself is not this stage's work.
- A grade is held out. Whatever is fitted is fitted on part of the data and
  scored on the rest, and the split is named.
- Constants come through the chain: `meta.json`, the registries in `oscnb`,
  and the result files of stages 0 to 3. None is typed.
- A threshold comes from the servo's own noise or its own constants, so the
  same rule works on a servo this stage never saw.
- The back-EMF speed is never trimmed against the pot. The two are graded
  against each other, and against the tach and the tag.
- No estimator asks for a coast. One may use a coast, a rest or a narrow
  drive that happens, and it says how it degrades when none does.
- Each notebook ends with what stage 5 reads: the bias by band, the noise
  per tick and its spectrum, the delay of each candidate filter in ticks,
  the valid flag with the share of time it holds, and the start-up time.
- Each estimator is kept as a pure function of the rows in `oscnb`, with
  its tests, so the notebook after it imports it.

## current-zero

Targets: the zero of the current channel while the bridge is awake, tracked
at the tick, on every rail and through a rail step; what may feed it on a
servo; and how long a held zero stays good with no feed.

Needs: stage 1's two-zero tracker and its awake step, every stage 3 sweep's
baseline, trough and coast rows, and the sag's levels.

How: the tracker run over every sweep in time order with its state
carried, scored against each sweep's coast tail by rail, duty and day. The
trough sample against the coast as a function of the duty and the winding
current, which picks the feed rule. The convergence after a rail step and
from a cold start, and the drift of an unfed zero over 1, 10 and 60 s.

Run: no capture of its own.

## speed

Targets: the back-EMF speed per row with its valid flag, its noise and its
delay, and the two forms of the brush term graded side by side.

Needs: `current-zero`, and stage 3's identification and compensation
results.

How: the stage 3 estimate form run per row with the tracked zero in place
of the coast's, the reading term, the volts construction, R(T) from the
NTC, and the brush term as the speed-times-current product and as the
signed speed table, each solved in closed form or by a fixed number of
steps. The gate is that window means of the per-row estimate reproduce
stage 3's scores per band. Then the noise per tick, the candidate filters,
and the valid flag: the tap floor, the dropout rate, and the first rows
after a reversal.

Run: no capture of its own.

## position

Targets: the horn angle from the pot per row, as a lookup indexed by
counts, with a held-out grade against the tag, the converter's carry
codes, the filter, and the lash offset.

Needs: stage 3's pot-table result and its clips, and the load ladder's
holds.

How: the table refitted as counts to angle at candidate pitches and graded
by chain, by clip kind and by pass, with and without each segment's offset.
The pot's noise at rest and in motion against candidate filters, as noise
against delay. What a causal estimator does at the carry codes. The
direction-signed offset with a causal direction source, scored through the
reversals where the train crosses the play.

Run: no capture of its own.

## velocity

Targets: the velocity a loop reads from a crawl to full speed, and a rest
verdict. The smooth crawl is the first target.

Needs: `speed` and `position`.

How: the back-EMF speed over short windows and the pot's slope over long
ones, each against the tag, and the rule that picks or blends them by speed
and by the valid flags. Stage 3's crossover of 77 to 146 ms is the starting
point. The crawl's angle-locked ripple and what a filter does to it. The
stopped rotor, which the back-EMF reads as a speed on some levels, and the
verdict that calls it at rest.

Run: no capture of its own.

## winding-temperature

Targets: the winding's temperature from the NTC and the dissipated power,
graded on the resistance reads stage 3 already holds.

Needs: stage 3's resistance and sag results, and the thermal constants of
earlier work on the family.

How: the I2t model over the board NTC run down each sitting's rows, scored
where a stall or a locked read gives the winding's own resistance. What the
stage 3 data can and cannot grade is stated, since no sitting was a thermal
one.

Run: no capture of its own.

## compensation

Targets: the closing table of the stage: one row per estimator with its
bias, noise, delay, valid flag and start-up time, the term it corrects, how
it corrects it, and the bound it leaves.

Needs: every result above.

How: the five result files read into one table, with stage 3's closing
table beside it row by row, so every term stage 3 handed on is either
compensated here, designed around, or stated as a bound.

Run: no capture of its own.

## What stages 5 to 7 read from here

| Deliverable | From | Gap |
|---|---|---|
| current loop's feedback | `current-zero`: the zero's error and its start-up time | none |
| velocity loop's feedback | `velocity`: noise against delay, the rest verdict, the crawl | the brush term's form under load |
| position loop's feedback | `position`: noise against delay, the held-out residual, the lash offset | the grade at speed is bounded by the stamp lead |
| thermal derate and cutoff | `winding-temperature` | no thermal capture |
| valid flags for the faults | `speed`, `velocity` | none |

The gaps are stage 3's and are carried, not closed here. The brush term's
form under load needs holds at one speed and two currents. The position
grade at speed needs the segment stamps fixed in the capture tool. Chirps
and a closed-loop hold are stage 5's bench.
