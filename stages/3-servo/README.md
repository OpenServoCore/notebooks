# Stage 3. Servo

The assembled servo, meaning the motor plus its gear train, its pot and its
case, run open loop. This is the last stage that captures anything open
loop. Stages 4 to 6 design the estimators, the loops and the limits over its
data, and stage 7 holds the closed loop to the numbers it sets, so the data
bank here has to be complete in one pass.

The servo is `mg90-d`, an MG90 clone of the same brand as the two stage 2
motors: a new, unused motor from one new servo, in the new, unused body
that came with mg90-c's motor, with the train cleaned of its stale factory
grease and regreased by hand with Super Lube (synthetic PTFE, NLGI 2)
before the first sitting. Its motor is never opened. One key names the servo and the
motor inside it, so its datasets land in
`telemetry/mg90-d__dev-v006-2A-D/<experiment>/<name>-N/` and each notebook
writes `results/mg90-d__dev-v006-2A-D.json` in its folder. The registry
entry in `oscnb/servos.py` holds what can be counted or read off the part,
the limits the bench holds it to, and a few numbers from earlier work on
another unit of the family (the gear ratio from counted teeth, about 19.8
pot counts per degree, about 184 degrees of travel), kept for comparison
and for sizing a capture. `OSC_SERVO` picks the servo the way `OSC_MOTOR`
picks a bare motor. mg90-b's stage 2 results stand as the type values every
electrical constant is graded against, with mg90-c's beside them as the
spread.

The board is `dev-v006-2A-D` with its stage 1 results, on the DPS, running
the experiment image, with the adapter on its bus at 3 Mbps and the `osc`
and `osc-lab` tools on the host.

## The sittings

The hands set the bench once per state and `uv run python -m oscnb.sitting
<state>` runs every plan that wants it. There are three states.

- `servo` is the servo on the board and the DPS with no instrument. The
  horn lock is a printed part of the rig that holds the horn at mid travel.
  It is a prompt inside this state, not a state of its own. Before the
  first plan that wants the rotor held, the runner centers the horn and asks
  for the lock, and before the next free plan it asks for it off.
- `servo-camera` is the servo in the printed rig with the tag horn, the
  reference pad beside it and the phone overhead, the STAT lamp in view.
- `servo-load` is the rig on its side with the shaft horizontal, the printed
  arm on the horn with a known mass at a known radius, the hub tag on top.

`servo` runs before the rig exists, and the camera and load sittings come
after it is built. `uv run python -m oscnb.sitting list` prints every plan
with its time and size.

## The rules of the bench

Nothing puts more than the rated 5 V across the motor as rail times duty.
On mg90-c, 7.4 V at 70 % changed the brushes within seconds and 67 % did
not, so on the 7.4 V rail no rung goes past 67 %, and the 5 V 100 % run-ups
sit at the rated voltage exactly. The runner refuses a rung over the line
before anything moves.

Three currents are limits too, all as rail times duty over the type
resistance. A lap into the horn lock draws at most 0.85 A, the stock
servo's own stall rating. The MG90 is sold at 1.8 kg cm of stall torque at
4.8 V, and through the 308:1 train at about 0.6 efficiency and the type Kt
of 1.16 mN m per A that comes to 0.75 to 0.85 A at the motor. The bolted
horn puts that torque through the train with no stopper in the path, so the
servo's rating is the right limit there. Any other held stall, like the arm
stood against gravity, draws at most the 0.5 A the metal train holds at a
stop. A lap pressed into an end stop draws at most 0.45 A, lower again,
since the output stopper is plastic and took a permanent set at about 0.32
A on another unit. So the locked laps run at 100 % on 3.3 and 3.7 V, 0.74
and 0.83 A, and the laps at the stops run at 1.2 and 1.8 V effective.

The motor rests 20 s torque off between cycles. Every free drive starts at
one of the guards, 100 counts inside the soft limits, and `osc-lab` ends
every drive with a park at mid travel and torque off. A rung runs unpolled,
so nothing stops a rung that outruns its window but the end stop. The runner
sizes every window from a pilot spin of 30 % on 5 V at the start of the
sitting. It bounds the speed of each rung from the pilot's speed and the
drag-free motor's, and cuts the window so the travel at that bound, the
coast after the cut included, ends 300 counts short of the far guard. On
fast rungs that leaves less than the plan asked for, and a rung the runway
cuts under 30 ms is left out and noted.

Before every sweep or burst set, the runner reads `pos`, `ntc_raw` and
`vbus_raw` into the sitting's manifest. A held seat is checked by its `pos`
before and after each lap. When it moves more than 10 counts, the runner
leaves out every plan of the sitting that presses a seat, since a seat that
walks is a lock that slipped or a stopper taking a set.

There is no thermocouple in this stage. The winding is its own thermometer.
R0 is read at rest with the motor at room, T0 from the board NTC at that
rest, and the copper's temperature after that is T0 + (R / R0 - 1) / 0.0039.
The two-node thermal model and the I2t constants were measured in earlier
work and are quoted from there, conclusions and key numbers only. Ke against
temperature stays unmeasured unless a later stage needs it.

## resistance

Targets: the terminal resistance of the motor inside the servo and the
contact offset V0, as the line through laps at two levels, tagged with the
board NTC's temperature, how far the seat moves per lap, and the gap to the
type value. Then the same read the way a user's pass runs it, at the end
stops with no lock, graded against the locked one.

Needs: the `servo` state with the horn lock; the stage 1 current-scale,
terminal-chain and sample-timing results; for `stop`, the inductance
result's window model for the crest correction at partial duty; mg90-b's
rest-resistance result for the grade; for `meter`, the `servo-meter` state,
the stage 0 dmm result for the meter's class and stage 1's current-scale
series laps as the DC reference.

How: `locked` is stage 2's stall pass in situ. The lock holds the horn at
mid travel and each sweep drives a 100 % lap of 300 ms in each direction,
three on 3.3 V and three on 3.7 V. At 100 % nothing switches, so the crest
sample is the mean and no window model enters R. Stage 2's third rail, 4.2
V, would draw 0.94 A, over the lock hold, and is left out. R and V0 come
from the line through the two rails. `stop` is the user's pass, a chain of
1.2, then 1.8, then 1.2 V effective, 300 ms each with no release, on 5 V,
pressed into the end stop by `osc-lab sweep --stall`, three chains at each
stop with the stops alternated. Its R and V0 come from the line through
the two levels, the crest sample corrected to the period mean by the
inductance result's window model at each duty. The seat is `pos` before
and after each locked lap, and the median `pos` of the TEL rows through
each stop chain. `rails`, optional, runs the same chains on 3.7 and 7.4 V
at the same two effective volts, so the terms a partial-duty read may
carry, the bridge's edge lags, the brake interval's drop and a crest
over-read per volt, separate by how each scales with the duty and the
rail. `meter` holds the same six levels into a stop for 2.5 s with the DMM
in series on the motor, so the crest sample's error against the true mean
current is read at each level with no construction in it.

Run: the `locked` and `stop` plans of `uv run python -m oscnb.sitting
servo`, `--include resistance/rails` for the other rails, `uv run python
-m oscnb.sitting servo-meter` with the meter in the loop, then the
notebook.

## inductance

Targets: L and the current's time constant from duty-step bursts into the
held rotor, and the window model, the loss conductance and the shape of
L with current that turns a crest sample into a period mean at any duty and
rail. Also whether the locked seat at mid travel and the seat at the stop
agree.

Needs: the `servo` state with the horn lock; the stage 1 sample-timing and
bridge-states results; mg90-b's inductance and motor-model results for the
grade.

How: stage 2's five groups on 5 V, forward 25 then 40 %, reverse 40 then
25 %, five captures each, then three more at reverse 25 % after a 20 s rest,
each change before a group's first capture different. `locked` runs them at
mid travel with the lock on. `wound`, optional, runs them into the lock
after a reverse press has wound the train. `stop`, optional, runs them at the low stop
with the train wound by a `--stall` press that skips osc-lab's park, the
seat read back before and after the groups and the horn centered at the
end; a read-back away from the stop ends the plan rather than record a free
rotor as a held one.

Run: the `locked` plan of `uv run python -m oscnb.sitting servo`, `--include
inductance/wound` with the lock on and `--include inductance/stop` with the
horn free, then the notebook.

## ladder

Targets: Ke from each rung's natural coast, the EMF at the cut over the
ripple tach, the brush drop as a speed table per direction with its
coefficient on the dropout rate, the breakaway duty against the sustaining
floor, the steady current and speed per rung for friction, and the dropout
rate per rung. All on the three rails a user's servo sees, 3.7, 5 and 7.4 V.

Needs: the `servo` state with the horn free; the stage 1 results; the
resistance and inductance results; mg90-b's bemf-coast, identification and
compensation results.

How: cycles of a spin and a 400 ms coast with the bridge open, both
directions, each spin from a seek to the near guard. `ref`, the bracket, is
three cycles of 30 % on 5 V at the open and the close of the sitting.
`capture` is 15, 20, 30, 40 and 60 % on each rail, three cycles a rung, with
windows of 1.2 s at 15 % down to 300 ms at 60 % as the ceiling the runway
cuts from. `low` kicks the rotor at about 1.5 V effective for 150 ms and
drops to a hold of 1.2 s at 0.6 down to 0.2 V effective on each rail, then
coasts. The ripple tach tells a turning hold from a stopped one. `rail`,
optional, adds the top rung each rail allows that the ladder leaves out, 80
and 100 % on 3.7 V and 67 % on 7.4 V.

Run: the `ref`, `capture` and `low` plans of `uv run python -m oscnb.sitting
servo`, `--include rail` for the top rungs, then the notebook.

## run-up

Targets: the inertia of the rotor plus the train reflected to it, from 5 V
100 % run-ups from rest, the speed-torque curve through the gears, the
dropout rate at full duty by direction as the brush state of the sitting,
and the crest less the coast's mean current at 100 %, where nothing switches,
as the meterless check of the inductance result's reading term.

Needs: the `servo` state with the horn free; the resistance, inductance and
ladder results; mg90-b's speed-torque result.

How: `runup` is a bracket, three cycles of a 100 % spin and a 500 ms coast
at the open and the close of the sitting, `runup-open-N` and
`runup-close-N`, so the brushes are read on both sides of everything else by
the same rung. The spin asks for 200 ms and the runway cuts it shorter.

Run: the `runup` plan of `uv run python -m oscnb.sitting servo`, then the
notebook.

## steps

Targets: the current's and the speed's response to duty steps at several
sizes and operating points, the delay from the command's edge, and the
reversals for the backlash's dynamics, the plant stage 5 designs the
current and velocity loops on.

Needs: the `servo` state with the horn free; the inductance and ladder
results.

How: `capture` runs chains of a base duty, a step up and back, 400, 300 and
300 ms asked for, at bases of 20, 30 and 40 % and steps of 2, 5 and 10 % on
5 V, both directions, from three starts, the guards and the guards moved in
by a sixth and by a third of the travel. `reversal` runs a duty then its
negative, chained, at 10, 20 and 30 %. Excitation inside one armed burst,
like PRBS or a chirp, needs an osc-lab verb that writes duty on a host clock
and is not here yet.

Run: the `capture` and `reversal` plans of `uv run python -m oscnb.sitting
servo`, then the notebook.

## sag

Targets: the current zero, R, a crawl and a held stall as the rail falls
from a full 2S pack at 8.4 V to its floor at 6.0 V, for the rail-tracking
duty cap, the zero tracker's step with the rail, and the undervolt behavior
stage 6 designs.

Needs: the `servo` state with the horn free, the DPS scripted.

How: `capture` steps the rail down 0.2 V at a time, and at each rail runs a
1.5 V effective chain into the low stop, a crawl both ways, kicked and then
held at 0.5 V effective, and one 30 % cycle. The DPS steps only between
sweeps with torque off, since a DPS step down has reset the servo once.
`held` runs only when the first pass came through clean. It steps the DPS
down the same rails while osc-lab holds a low stall into the horn lock.

Run: `uv run python -m oscnb.sitting servo --include sag/capture`, then
`--include sag/held`, then the notebook.

## pot-table

Targets: the pot's table over the whole travel graded by the tag, the
backlash through reversals, the travel in degrees, the gear ratio checked as
tag degrees per ripple cycle, the table's drift between the open and the
close, and the crawl truth: the speed at 2 to 5 degrees per second at the
horn, where neither the pot nor the back EMF reads well.

Needs: the `servo-camera` state; the stage 0 camera-tag and clock-sync
results; the ladder result for the sustaining floor.

How: each block is one phone clip, the STAT lamp flashed by `osc clapper` at
its start, every segment's host time in its `segments.csv`. `pass`, the
bracket, is 20 % each way over the whole runway at the open and the close.
`crawl` kicks at 30 % for 150 ms and holds 3 s at the floor less 1 % up to
the floor plus 4 %, both directions, from the same three starts as steps,
two clips per start. `reversal` runs the crawl duty and 20 % each followed by
its negative, and `steps` steps a crawl by 1 % up and down.

Run: `uv run python -m oscnb.sitting servo-camera`, then
`equipment/camera/extract.py` on the dataset, then the notebook.

## load

Targets: the holding current against gravity's torque over the angle, so Kt
through the gears and the train's efficiency both ways, the friction line
under load, the step response with a known load, and crawls lifting and
lowering.

Needs: the `servo-load` state, 120 g at 55 mm on the arm, about 66 mN m at
horizontal and 0.3 A at the motor, under the hold with margin; the
resistance, ladder and run-up results.

How: the guards sit 20 degrees inside the stops so the mass never meets
one, and the arm points straight up at mid travel. `holds` stands the arm at
seven angles, a short window, a hold of 3 s and a second window, the duty
stepped up by 2 % until the arm stops falling, which is the holding duty,
and on until it lifts, which is the lifting duty. `lift` runs 30, 40 and 50
% cycles over the guards both ways. `steps` steps 5 % up and down from the
holding duty at three of the angles, and `crawl` crawls both ways at the
floor plus 2 %. The holds are cut like free rungs, since an arm past its
lifting duty climbs through the whole hold.

Run: `uv run python -m oscnb.sitting servo-load`, then the notebook.

## friction

Targets: the Coulomb and viscous terms at the motor through the gears, both
ways, the two routes to them compared, and the train's efficiency.

Needs: the ladder, run-up and load datasets and results, the resistance
result.

How: the steady current of each ladder rung against its speed from the
ripple tach and Ke, in motor-side units, the run-ups' torque against their
speed, and the load holds' band between holding and lifting.

Run: no capture of its own, the notebook reads the datasets above.

## identification

Targets: the one-pass identification of the next section run on this
stage's own datasets, its cost in time and motor revolutions, and its
constants graded against the long-form results of the experiments above.

Needs: every result above.

How: the pass's steps picked out of the resistance, inductance and ladder
datasets, each reduced the way a user's tool would reduce it, against the
same quantity from its own notebook.

Run: no capture of its own.

## compensation

Targets: the four-term speed estimate scored by speed band against the
ripple tach and the tag, the pot against the tag by window length (the
crossover curve that picks the velocity source at a crawl), the threshold
of the dropout-rate flag, and the closing table of what to compensate and
how.

Needs: every result above, the pot-table crawl in particular.

How: the estimate built from the identification's constants and run over
the ladder, run-up and pot-table datasets, residual per band against the
tach and the tag, and pot velocity over windows from 50 to 500 ms against
the tag.

Run: no capture of its own.

## The one-pass identification

What a user runs on their own servo, with one supply, no meter and the stops
known, inside the rated voltage:

0. A rest of 2 s torque off: the zeros, the rail and the NTC's T0.
1. Laps at the stops: seek the low stop and chain 1.2, 1.8 and 1.2 V
   effective, 300 ms each, twice, then the high stop the same, the sides
   alternated so the seat's walk cancels. This gives R0 at T0, V0 and a seat
   check, in about a minute with the rests.
2. Bursts at a held seat, 25 and 40 % both signs, three each. This gives L
   and the window model, about 30 s.
3. Coast rungs: kick then hold at five duties from the floor plus 1 % to the
   rated cap, both directions, each ending in a natural coast. This gives Ke
   from the tach against the EMF at the cut, the brush table per direction
   and its dropout coefficient, the floor and the breakaway, and the dropout
   reference, in about 3 minutes and 1500 motor revolutions.

The current zero is identified by none of it, a live tracker learns it.
The whole pass repeats at the open and the close of the `servo` sitting for
its own repeatability, and the identification notebook grades it.

## What stages 4 to 7 read from here

| Deliverable | Dataset | Gap |
|---|---|---|
| speed estimator, four terms | ladder capture, low and rail, run-up, pot-table crawl, with the resistance and inductance results | none |
| position estimator, pot table | pot-table pass, crawl and reversal: pos against the tag and the ripple angle | the tag's 0.16 deg error, 3 counts, is the grade's floor |
| current zero tracker | every dataset's baseline and trough rows, sag | none |
| winding temperature | resistance at rest against the board NTC | earlier work holds the thermal constants |
| current loop | steps, the inductance window model | PRBS and chirps wait on an osc-lab verb |
| velocity loop and the crawl | pot-table crawl, friction, load lift | none |
| position loop and the hold | load holds, pot-table reversal | a closed-loop hold is stage 5's bench |
| current ceiling, stall fault | resistance chains, load holds | none |
| position-error fault | load steps and lift | none |
| sag and undervolt | sag | a real pack's sag curve, the DPS only |

Stage 7 validates the closed loop mode by mode on this servo and then on
servos it was not designed on, so every grade here is written as a bound it
can be held to, with the dataset that carries the baseline. The speed
against the tag per band, expected at 1 to 3 rev/s absolute at a crawl and 2
to 4 % above 100 rev/s. The crawl's speed ripple over 0.2 to 30 Hz, open
loop, as the ceiling a closed loop has to stay under. The table's residual
in counts over the travel and the backlash in degrees. The hold's pos spread
and drift under a known torque. Kt through the gears per direction. The
dropout rate above which the brush table no longer holds. The lowest rail
the crawl and the stall chain stay in bounds at. The seat shift per lap at
the lap cap. And the thermal estimate against the winding's own resistance
read at rest after a hold.

The fixtures serve both stages. The rig and the tag horn hold the same body
pocket, the board pocket and the reference pad, with the second orientation
for the arm, and the sitting states take a closed-loop plan the same way
they take an open-loop one, so stage 7 adds plans, not states. A second
servo needs only a registry entry and the one-pass identification.
