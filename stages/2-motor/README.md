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
3 Mbps, the DMM, and the `osc` tool. The board runs the experiment image, as
in stage 1. Datasets land in `telemetry/<motor>__<board>/<experiment>/capture-N/`,
and each notebook writes `results/<motor>.json` in its folder. The teardown
has no board in it and lands in `telemetry/<motor>/teardown/`.

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
stall laps gives them, and how much of the total the brushes own once the
teardown has read the copper.

Needs: motor in the rig, bar, board, DPS, host; the stage 1 current-scale
and terminal-chain results.

How: the bar steps the rotor through the twelve holes. At each hole the
stall pass drives 100 % laps of 300 ms into the locked rotor at the three
1S levels, both directions, the board's current and terminal taps as stage
1 graded them.

Run: `uv run python stages/2-motor/rest-resistance/capture.py`, then the
notebook.

## [inductance](inductance/inductance.ipynb)

Targets: the winding's inductance and the time constant of its current rise,
from the ADC free running through a duty step into the locked rotor, and
whether a slower component sits under it.

Needs: motor in the rig, bar, board, DPS, host.

How: the burst ladder at 25 and 40 % on a 5 V rail, both signs, five
captures each, at four holes a quarter turn apart.

Run: `uv run python stages/2-motor/inductance/capture.py`, then the notebook.

## [bemf-coast](bemf-coast/bemf-coast.ipynb)

Targets: the back-EMF constant, from the terminal voltage of the coasting
rotor over its speed, the speed counted in the commutation ripple of the
same samples, and whether the spin's own route agrees.

Needs: motor in the rig, bar off, board, DPS, host.

How: cycles of a 1.5 s spin at 15 to 60 % and a 0.8 s coast with the bridge
open, both directions, three per duty, at 5.0 and 7.4 V, twenty seconds
apart.

Run: `uv run python stages/2-motor/bemf-coast/capture.py`, then the notebook.

## [no-load-current](no-load-current/no-load-current.ipynb)

Targets: the current the free motor draws against its speed, the drag it
stands for, and the inertia that ties the spin's current to the coast's
deceleration.

Needs: the bemf-coast dataset and result.

How: the last stretch of every bemf-coast spin, where the speed has settled,
against the coast that follows it.

Run: no capture of its own; the notebook reads bemf-coast's dataset.

## speed-torque

Targets: the speed, the current, the output power and the efficiency
against the torque at one rail, the motor's performance curve, with the
measured run-up over the line the model draws from the constants.

Needs: motor in the rig, bar off, board, DPS, host; the bemf-coast,
no-load-current and rest-resistance results.

How: three cycles of a 300 ms spin at 100 % from rest and a 0.6 s coast,
both directions, on a 5.0 V rail, twenty seconds apart. The rotor's
inertia is the load, so the run-up sweeps the curve from stall to no load:
the torque is the inertia times the rise of the speed plus the drag at that
speed. The bemf-coast spins add the same at their partial duties. The spin
is short because a free motor at 100 % runs at its top speed, which wears
the brushes.

Run: `uv run python stages/2-motor/speed-torque/capture.py`; the notebook
comes after rest-resistance, its last input.

## [teardown](teardown/teardown.ipynb)

Targets: the copper's temperature coefficient on the rotor out of its can,
the winding's own resistance with no brushes in the path, the masses and
the counts of every part, and with rest-resistance the share of the terminal
resistance the brushes own.

Needs: the rotor with leads on two commutator tabs, the thermocouple in a
coil, a controlled heat source, DMM on ohms and on temperature by turns,
host; the rest-resistance result.

How: the rotor on the heater through room, about 45 C, about 60 C and room
again, the meter reading ohms for a minute and then the thermocouple for a
minute at each plateau, the lead zero read first with the leads shorted.
The masses, the photos and the counts go into the notebook by hand.

Run: `uv run python stages/2-motor/teardown/capture.py`, then the notebook.
