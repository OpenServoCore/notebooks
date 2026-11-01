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
terminal-chain and sample-timing results, sample-timing's window scale and
its resistor bursts for the window's own slope, the rest-resistance result
for the comparison, the ripple and the inertia bound, the bemf-coast result
for the leftover volts, the bridge-states result for fast decay's edges and,
with its fast-decay bursts on one inductor, for the method check, and the
teardown result for comparison.

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
result for the meter run's loop, the inductance result for the locked
rotor's step, and the bridge-states result for fast decay's edges.

How: cycles of a 1.5 s spin at 15 to 60 % and a 0.8 s coast with the bridge
open, both directions, three per duty, at 5.0 and 7.4 V, twenty seconds
apart. `meter` runs three cycles each of 30 % on 5.0 V and 20 and 40 % on
7.4 V with 3 s spins, the meter in series, for the board's crest sample
against the true mean current. `high` runs the same at 60, 80 and 100 % on
5.0 V, for how the reading term falls toward full duty, with 100 %, where
nothing switches, as the zero of a driven bridge. `swapped` runs the 40 and 60 % cycles at
7.4 V with the leads as wired and then swapped at the board's connector, to
tell a direction split of the motor from one of the board. `spinning` keeps
the meter in series and records bursts after 1.5 s at the burst's own duty
at the meter run's three points, slow and fast decay, both directions, for
the current inside the PWM period. Under fast decay the rotor stays still
at 5.0 V 30 % and 7.4 V 20 %, below fast decay's onset, and turns only at
7.4 V 40 %.

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
drag at that speed added it is the motor's whole torque, the torque
constant times the current. The inertia is read from those two together,
so the torque constant is taken equal to the back-EMF constant and not
checked, and the drag is no-load-current's with bemf-coast's reading term
taken out, since nothing switches at 100 %. The ripple counts the speed from about
200 rev/s up, and the first milliseconds read the stall end below that.
The bemf-coast spins at partial duty are left out, since there the current
switches and the curve is another one. The spin is short because a free
motor at 100 % runs at its top speed, which wears the brushes.

Run: `uv run python stages/2-motor/speed-torque/capture.py`; the notebook
comes after rest-resistance and inductance, its last inputs.

## [teardown](teardown/teardown.ipynb)

Targets: the masses, dimensions and counts of every part, and the rotor's
inertia as a uniform cylinder of its mass and size, a reference beside the
earlier readings. Any path across the motor's
terminals other than the winding. The winding's own resistance with no
brushes in the path, and with the leads read on their own, the share of
the terminal resistance the brushes own at the meter's milliamp and at the
stall pass's current. The gray disc behind the commutator, held against the
spike and the drive-only current inductance and bemf-coast left open. The
disc sweep reads the disc as a varistor ring, about 7.8 V at 1 mA per pad
pair, and the ring holds the spike, taking a third to a half more charge
per volt at a PWM edge than the meter's slow test reads. The coil runs put
the drive-only current on one coil with no brushes and no ring, a few
milliamps on the bare rotor that do not scale with the rail and more with
the can and the magnets, beside an inductance that moves with the current,
and leave which network carries it to motor-model. And how the heated
copper cools, read by its resistance through the board. The cooling's
shape is the part a brushed motor of this class shares. Its numbers
belong to this rotor in open air and feed no calibration.

The coefficient itself comes from earlier work, which soaked a whole MG90 of
the same model in a dryer box until it sat at one temperature, a K
thermocouple on the can, and read the DC resistance across its terminals
at rest on the meter at each plateau, 4.52 ohm at 28.6 C and 5.09 ohm at
61.5 C. The fitted slope there is 0.396 +/- 0.010 % per C, 101 % of
copper's 0.393, with a fixed term of -0.04 +/- 0.11 ohm, so the loop is
copper with nothing else in series at rest and the textbook coefficient
holds to about 2.5 %. That motor is a different unit than mg90-b, but the
same model wound from the same wire in the same process should give a
similar coefficient, so the teardown takes it from there and does not
test it, since its cool run has no reference that read the copper hot. The
cool run reads the shape of the heating and cooling, which a plateau soak
never sees.

Needs: a 1 mg scale, a micrometer and calipers. For `terminals` the end
cap with its brush card, the DMM on ohms, capacitance and DC mA, and the
DPS. For `cool winding` the rotor with leads on commutator tabs 1 and 2,
held by its shaft in still air, the thermocouple taped in the winding, the
DMM on ohms and on temperature, and the board on the DPS. For `coil` the
bare rotor with leads on one coil, and the board on the DPS, and for
`coil levels ra` bridge-states' bare load and inductor A. For `disc iv`
the disc off the rotor with two leads on its pads, the DMM on capacitance,
ohms and DC current, and the DPS. The stage 0
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

`coil` then asks whether the drive-only current needs the brushes, the
magnets or the can. It runs inductance's `fast` groups, fast decay at 25
and 40 % both ways and slow decay forward, five bursts each on a 5 V rail,
into one coil of the bare rotor, which keeps only its laminations. `ring`
has the leads on the disc's two pads at the coil's ends, so the ring sits
across the coil, and `noring` moves one lead to the coil's free wire end
cut from the third pad, so it does not. `can` keeps those leads and puts
the rotor back in the can with its magnets, no end cap or brushes, the
shaft clamped, so the can's steel and the magnets return without the
brushes. `coil levels` then runs the same groups at 3.7, 5.0 and 7.4 V in
one sitting with the disc off, on bridge-states' resistor and inductor A
as the control, on one coil bare and on the same coil back in the can, so
a current that follows the volts and one that does not read apart.

`disc iv` takes the gray disc off the rotor, the segments still fused to
its pads and two leads on two pads, and asks whether it is a varistor or a
capacitor ring with a leak. The meter reads ohms and capacitance across the
leads, then the DPS steps them from 0 to 20 V and back with 2 mA as its
limit, both ways round, the meter on DC current in series. A leak follows
the volts in a straight line, and a varistor turns up at its knee.

`cool core`, with the tip on the iron, was not run. The tip in the winding
lagged the copper by tens of seconds, too slow to resolve the copper's 2 s
exchange with the iron. The plateau soak and the coil unwind were not done
either. The coefficient comes from earlier work, and the copper's mass and
the coils' turns stay open.

Run: `uv run python stages/2-motor/teardown/capture.py terminals` once the
rotor is out, then `capture.py cool winding` with the rotor at room, then
`capture.py coil ring`, `capture.py coil noring` and `capture.py coil can`
once the disc's pads are read, `capture.py disc iv` once the disc is off the
rotor, then `capture.py coil levels ra`, `capture.py coil levels bare` and
`capture.py coil levels can` with the disc off, then the notebook.

## [motor-model](motor-model/motor-model.ipynb)

Targets: one model of the current inside the PWM window that draws every
locked-rotor burst of stage 2, the one coil bare and in the can and the
motor at one seat, in fast and slow decay, and the board's crest sample
against the motor's mean current that the model predicts on the free motor,
by duty and rail. On the motor at hole 11 the model that holds up is an
inductance that rises with the current from its last reversal, the Rayleigh
law of soft iron, a loss across it, and a slow series rung. The rung is
what makes inductance's period fit read more than the window does, and what
makes it slide with the fit's length. The window offset at the ON edge is
mostly a loss at the edge, about 12 mA at 4.95 V, and the rest is the
ramp's shape. On the free motor the crest sample reads a loss in the window
that a meter averages away, falling with one less the duty to zero at
100 %, about 19 mA over the mean at 5.0 V and 29 mA at 7.4 V up to 30 %,
high confidence. Whether that loss sits across the inductance or in the
rung is moderate at most, and the spinning motor needs a third or more loss
than the locked one, for a reason these data leave open. The 20 mA
speed-torque found at 100 % is drag, high confidence. On the turning motor
the terminal volts carry a brush drop beyond R i and the EMF, a voltage set
by the speed and the direction and not by the current, within a few
millivolts below 160 rev/s, about 45 mV at 300 rev/s and 0.37 V at 550
rev/s, high confidence that it is in the motor and does not scale with the
current. Below 160 rev/s the running resistance is the resistance at rest
within 3 to 4 %.

Needs: no bench. The teardown's levels and coil runs and hand readings,
inductance's fast-1, capture-1 and reversal-1 at hole 11, bridge-states'
fast-decay controls, bemf-coast's capture-1, meter-1 and high-1 and its
result for spinning-1 and its bursts, speed-torque's run-ups, and the
teardown's photo of the brushes. The stage 1 current-scale, terminal-chain,
sample-timing and bridge-states results, and the rest-resistance,
bemf-coast, speed-torque and teardown results. The inductance, teardown,
bemf-coast and no-load-current results for comparison only.

How: a tick-level simulator of one branch through the bridge, a series
resistance into an inductance that depends on the current with a
conductance across it and an optional rung, through the bridge's four modes
and the amplifier's lag. Each candidate network is fitted to a set's fast
decay and held out by rail, by duty and by decay mode, then the motor's
model is run to steady state on the spinning motor with its loss refitted
to the meter runs and to capture-1, whose mean current comes from the
coast's deceleration through speed-torque's inertia. The brush drop is the
voltage before each coast's cut less the EMF at the cut, over the free
spins and the 100 % run-ups, set against speed-torque's run-ups, which pass
the same speeds at up to eleven times the current.

Run: no capture of its own, the notebook reads the datasets above. Its fits
run on every core, and the simulator needs numba, which `uv sync` installs.

## [identification](identification/identification.ipynb)

Targets: whether the board alone recovers each motor constant a back-EMF
speed estimate needs, and how close each board-only route lands to this
stage's bench references, at what cost in maneuvers, seconds and motor
revolutions against a servo's travel of about 157 motor revolutions. The
routes are $R$ from one, two and three locked seats, $R$ against the board
NTC with the winding's self-heat bounded and a low-speed reference rung as
a relative thermometer, $K_e$ from the coasts of one repeat, $L$ from a few
bursts at one seat, the reading term $\Delta$ and $c = J/(K_t K_e)$ from the
100 % anchor, and the brush drop's two speed tables by mechanical direction
from three rungs. It scores no speed. The compensation notebook takes these
constants into the estimator.

Needs: no bench. rest-resistance's stall pass and inductance's capture-1
through their own cells, bemf-coast's capture-1, meter-1, high-1 and
swapped-1, and speed-torque's run-ups through its own cells. The stage 1
current-scale, terminal-chain, sample-timing, bridge-states and ntc-curve
results, and the rest-resistance, inductance, bemf-coast, speed-torque,
teardown and motor-model results.

How: each route reads the captures as a servo's board could, one seat for
an end stop, one repeat of capture-1's twenty spins for an ident run, the
last 50 ms before each cut and the first 15 ms of coast for a short spin,
and one speed-torque run-up each way for the anchor. Every route is graded
against the bench reference, held out where the data allow, and costed by
the board's own rough speed. The notebook ends with what the next motor's
capture must record so every route runs on it.

Run: no capture of its own. `identification.py` beside the notebook holds
the routes and the pieces the compensation notebook also reads, and its
reductions run on every core.

## [compensation](compensation/compensation.ipynb)

Targets: how accurate a back-EMF speed estimate from the board's own signals
is against the ripple tach, with the reading term $\Delta$ and the brush drop
$V_b$ compensated. The signals are the duty, the terminal volts over the
window, the crest current sample and the rail. It is asked twice, with this stage's
reference constants and with the constants identification recovered
board-only, and the gap between the two is the price of identifying without a
meter. Beside it, the reading term's table per volt and whether it holds
across rails, the brush drop as a speed table against a power law, whether it
follows the motor's rotation or the board's drive sign, how it moved between
two sittings, the estimate's error budget by speed band, and how few rungs an
ident needs for $V_b$.

Needs: no bench. bemf-coast's capture-1, swapped-1, meter-1 and high-1,
speed-torque's run-ups through its own cells, and for the identified constants
rest-resistance's stall pass and inductance's capture-1 through their own
cells. The stage 1 current-scale, terminal-chain, sample-timing, bridge-states
and ntc-curve results, and the rest-resistance, inductance, bemf-coast,
speed-torque, motor-model and identification results.

How: the estimate is solved by fixed point per spin over the last 200 ms
before the cut, four ways, with neither term, either one alone and both.
capture-1 is scored by the fold that left its repeat out of $V_b$, with
$\Delta$ at a $G$ fitted on the meter runs alone, and swapped-1 and
speed-torque's one-turn windows were never in a fit. The same estimate then
runs on identification's 36 board-only passes, and on each identified
constant alone in the reference set, to split the price by constant.

Run: no capture of its own. `compensation.py` beside the notebook holds the
estimator and its scoring on top of identification's module, and the grid's
steady states run on every core.
