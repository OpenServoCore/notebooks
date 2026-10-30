# Stage 2. Motor

The motor on its own, out of the servo, with nothing between it and the
board: no gear train, no pot, no load. Every electrical constant stage 3
reads in situ is graded against the numbers measured here. The motor is
`mg90-b`, the motor of a new MG90 clone opened for this stage; its registry
entry in `oscnb/motors.py` carries only what can be counted, the slots, the
brushes and the pinion's teeth, since a motor has no datasheet. The board is
`dev-v006-2A-D` with its stage 1 results.

The sitting needs the bare motor in the [motor rig](../../rigs/motorrig/__init__.py)
with two leads soldered to its tabs, the rig's wrench bar for the experiments
that hold the rotor, the board on the DPS with the adapter on its bus at
3 Mbps, the DMM, and the `osc` and `osc-lab` tools. The board runs the experiment image, as
in stage 1. Datasets land in `telemetry/<motor>__<board>/<experiment>/capture-N/`,
and each notebook writes `results/<motor>.json` in its folder. The teardown's
`terminals` run has no board in it and lands in `telemetry/<motor>/teardown/`,
and its `cool` runs land beside the others.

Two rules of the bench. The bare motor runs at most thirty seconds at a time
with twenty seconds of rest after, so the copper stays within a degree or
two of the can, and every stall lap is short enough to leave it cold: a
reading of the winding at an unknown temperature is not a reading. And
nothing pushes or pulls along the shaft: the bar is clamped on the pinion
and lifted off by its screw, since a shaft pressed past its end play rides
the brushes wrong and the motor is not the same motor after.

One motor goes through every experiment in the order below and is torn
down last.

## [rest-resistance](rest-resistance/rest-resistance.ipynb)

Targets: the terminal resistance at rest and how it moves with the rotor's
angle, the brush drop and the contact resistance as the line through the
stall laps gives them, and how much of the total the brushes own against
the copper the teardown read.

Needs: motor in the rig, bar, board, DPS, host; for `ohms` the DMM on
ohms and temperature with the thermocouple, no board; the stage 0 dmm
result, the stage 1 current-scale and terminal-chain results, the bemf-coast result, and the speed-torque dataset
for its run-ups' first milliseconds.

How: the bar steps the rotor through the twelve holes. At each hole the
stall pass drives 100 % laps of 300 ms into the locked rotor at the three
1S levels, both directions, the board's current and terminal taps as stage
1 graded them. `ohms` is the meter's route, for the teardown's brush path.
The motor sits unpowered until it is at room, a thermocouple on the can
reads that temperature before and after, and between them the meter reads
the lead zero and then the motor's leads at their free ends at each of the
twelve holes, 40 s a hole. The meter's milliamp leaves the copper at room, so the can's
temperature is the copper's.

Run: `uv run python stages/2-motor/rest-resistance/capture.py`, and
`capture.py ohms` before the teardown, then the notebook.

## [inductance](inductance/inductance.ipynb)

Targets: the winding's inductance and the time constant of its current rise,
from the ADC free running through a duty step into the locked rotor, and
whether the rise is one pole on the burst's time scale.

Needs: motor in the rig, bar, board, DPS, host; the stage 1 current-scale,
terminal-chain and sample-timing results, the sample-timing resistor bursts
and current-scale laps for the window's scale, the rest-resistance result
for the comparison and the inertia bound, the bemf-coast result for the
leftover volts, and the bridge-states result and its fast-decay bursts on
one inductor for the method check.

How: at each of the twelve holes, five groups of bursts on a 5 V rail in
an order that puts a different change before each group's first capture:
forward 25 then 40 %, reverse 40 then 25 %, five captures each, then three
more at reverse 25 % after a 20 s rest. A second run, `reversal`, stays
at hole 11 with nothing moved and reverses the sign at 25 % after rests of
none, 5, 20 and 50 s. A third, `steps`, at the same hole puts a 25 % step
after a reversal from 10, 25 or 40 %, rising then falling, and a fourth,
`fast`, runs the four main groups in fast decay and then two slow groups at
the same seat.

Run: `uv run python stages/2-motor/inductance/capture.py`, then
`capture.py reversal`, `capture.py steps` and `capture.py fast` after it
with the bar left at hole 11, then the notebook.

## [bemf-coast](bemf-coast/bemf-coast.ipynb)

Targets: the back-EMF constant, from the terminal voltage of the coasting
rotor over its speed, the speed counted in the commutation ripple of the
same samples, and whether the spin's own route agrees.

Needs: motor in the rig, bar off, board, DPS, host; for `meter` and
`spinning` the DMM on its 220.00 mA range in series with one motor lead,
logging over Bluetooth; the stage 0 dmm result, the stage 1 current-scale,
terminal-chain, rest-noise and sample-timing results, the rest-resistance
result for the meter run's loop, and the inductance result for the locked
rotor's step.

How: cycles of a 1.5 s spin at 15 to 60 % and a 0.8 s coast with the bridge
open, both directions, three per duty, at 5.0 and 7.4 V, twenty seconds
apart. `meter` runs three cycles each of 30 % on 5.0 V and 20 and 40 % on
7.4 V with 3 s spins, the meter in series, for the board's crest sample
against the true mean current. `high` runs the same at 60, 80 and 100 % on
5.0 V, for how the reading term falls toward full duty, with 100 %, where
nothing switches, as the zero of a driven bridge. `swapped` runs the 40 and 60 % cycles at
7.4 V with the leads as wired and then swapped at the board's connector, to
tell a direction split of the motor from one of the board. `spinning` keeps
the meter in series and records bursts of the rotor already turning at the
meter run's three points, slow and fast decay, both directions, for the
current inside the PWM period.

Run: `uv run python stages/2-motor/bemf-coast/capture.py`, then
`capture.py meter`, `capture.py high`, `capture.py swapped` and `capture.py spinning`, then the
notebook.

## [no-load-current](no-load-current/no-load-current.ipynb)

Targets: the current the free motor draws against its speed, the drag it
stands for, and the inertia that ties the spin's current to the coast's
deceleration.

Needs: the bemf-coast dataset and result, its reading term and the spins'
corrected currents among it, and the speed-torque result for its run-up's
inertia as a cross-check.

How: the last stretch of every bemf-coast spin, where the speed has settled,
against the coast that follows it, then again with bemf-coast's reading
term taken out of the spin's current.

Run: no capture of its own; the notebook reads bemf-coast's dataset.

## [speed-torque](speed-torque/speed-torque.ipynb)

Targets: the speed, the current, the output power and the efficiency
against the shaft torque at one rail, the motor's performance curve, with
the measured run-up over the line the model draws from the constants, and
the rotor's inertia from the same run-ups.

Needs: motor in the rig, bar off, board, DPS, host; the bemf-coast,
no-load-current, rest-resistance and inductance results.

How: three cycles of a 300 ms spin at 100 % from rest and a 0.6 s coast,
both directions, on a 5.0 V rail, twenty seconds apart. The rotor's
inertia is the load, so the run-up sweeps the curve from stall to no load.
The shaft torque is the inertia times the rise of the speed, and with the
drag at that speed added it is the motor's whole torque, which the torque
constant times the current checks. The ripple counts the speed from about
200 rev/s up, and the first milliseconds read the stall end below that.
The bemf-coast spins at partial duty are left out, since there the current
switches and the curve is another one. The spin is short because a free
motor at 100 % runs at its top speed, which wears the brushes.

Run: `uv run python stages/2-motor/speed-torque/capture.py`; the notebook
comes after rest-resistance and inductance, its last inputs.

## [teardown](teardown/teardown.ipynb)

Targets: the masses, dimensions and counts of every part, and the rotor's
inertia bounded from its mass and size. Any path across the motor's
terminals other than the winding. The winding's own resistance with no
brushes in the path, and with the leads read on their own, the share of
the terminal resistance the brushes own at the meter's milliamp and at the
stall pass's current. The gray disc behind the commutator, held against the
spike and the drive-only current inductance and bemf-coast left open. And
whether the copper's resistance through the board, over the textbook
copper coefficient, reads the copper's temperature, and how the heated
copper cools. The cooling's shape is the part a brushed motor of this
class shares. Its numbers belong to this rotor in open air and feed no
calibration.

The coefficient itself comes from earlier work, which soaked a whole MG90 of
the same model in a dryer box until it sat at one temperature, a K
thermocouple on the can, and read the DC resistance across its terminals
at rest on the meter at each plateau, 4.52 ohm at 28.6 C and 5.09 ohm at
61.5 C. The fitted slope there is 0.396 +/- 0.010 % per C, 101 % of
copper's 0.393, with a fixed term of -0.04 +/- 0.11 ohm, so the loop is
copper with nothing else in series at rest and the textbook coefficient
holds to about 2.5 %. That motor is a different unit than mg90-b, but the
same model wound from the same wire in the same process should give a
similar coefficient, so the teardown takes it as settled. Its cool run
checks that the bare copper through the board reads its own temperature
over that coefficient, and the shape of the heating and cooling, which a
plateau soak never sees.

Needs: a 1 mg scale, a micrometer and calipers. For `terminals` the end
cap with its brush card, the DMM on ohms, capacitance and DC mA, and the
DPS. For `cool winding` the rotor with leads on commutator tabs 1 and 2,
held by its shaft in still air, the thermocouple taped in the winding, the
DMM on ohms and on temperature, and the board on the DPS. The stage 0
thermocouple result, the stage 1 current-scale and terminal-chain results,
and the rest-resistance, inductance, bemf-coast, speed-torque and
no-load-current results.

How: the motor comes apart by hand. Each part is weighed and photographed,
the can and the rotor measured, and the poles, segments, magnets, brushes
and brush fingers counted. `terminals` reads the end cap seated back on
the empty can with the rotor out, the brushes touching nothing. The meter
reads ohms and capacitance across the two terminal tabs and ohms from each
tab to the can, then the DPS puts 3.7, 5.0 and 7.4 V across the tabs
with the meter on DC mA in series, since a part that conducts only above a
volt or two reads open on the meter's ohms. Then a lead zero and the motor's two leads in series.
The rotor's three tab pairs are read by hand on ohms, each one coil in
parallel with the other two in series. `cool winding` reads the room and
the leads on the meter, then puts the leads on the board. The board probes
the copper at room with 5 ms windows at 100 %, heats it at 100 % on the
3.3 V rail for 5 s, a 100 ms window each second between held drives, and
runs on with no pause into probes 0.2 s apart for six seconds, then once a
second to a minute and every 5 s to about eleven minutes, the meter
logging the thermocouple throughout. The first heat reached the 40 C
target, so the second heat did not run. Last, the coil wires are cut
loose at one pad of the gray disc and its pad pairs read by hand on ohms
and capacitance.

`cool core`, with the tip on the iron, was not run. The tip in the winding
lagged the copper by tens of seconds, too slow to resolve the copper's 2 s
exchange with the iron. The plateau soak and the coil unwind were not done
either. The coefficient comes from earlier work, and the copper's mass and
the coils' turns stay open.

Run: `uv run python stages/2-motor/teardown/capture.py terminals` once the
rotor is out, then `capture.py cool winding` with the rotor at room, then
the notebook.
