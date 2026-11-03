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
down last. A second motor of the same clone, `mg90-c`, went through the
free-rotor and locked-rotor captures afterward and was not opened; each
notebook closes with a second-motor section that runs its method on
mg90-c's own captures and result files, with mg90-b's numbers beside them as
the comparison only, and writes `results/mg90-c.json`. That section is the
check that the constants and the methods carry to a motor they were not
built on.

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

Run: the `capture` plan of `uv run python -m oscnb.sitting locked`, and
`stages/2-motor/rest-resistance/capture.py ohms` before the teardown, then the notebook.

Second motor: mg90-c's stall pass through the same cells reads 4.40 ohm in the
median over its twelve holes in direction +1 at 0.79 A, 4.28 to 4.44, against
mg90-b's 4.46 at a board 3 C warmer, a gap the size of 3 C of copper with the
copper's temperature measured on neither pass. Direction -1 reads 1.3 % over
it, the laps of a hole repeat to 0.24 % as on mg90-b, and the direction gap
keeps half the first lap's rise with an offset mg90-b's did not have. Hole 1
sets the low end: its 3.7 V lap is not on a line and warms twice as fast as
the rest. The early fall is near zero on this motor. The meter's route and the
run-up read were not run on mg90-c, so its result file carries the stall
pass's keys alone.

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

Run: the `capture` plan of `uv run python -m oscnb.sitting locked`, with
`--include fast` for the fast-decay groups at the last hole, then the
notebook. The reversal and step-size runs were the first motor's alone.

Second motor: mg90-c's `capture-1` and `fast-1`, the main run at twelve holes
and the fast-decay run at hole 11, on the same bar seat as its stall pass,
read by the same cells with the motor's own edge pools and its own rest-
resistance result at each hole, into `results/mg90-c.json` with mg90-b's keys
where the runs allow (the reversal, step-size and travel-bound keys are left
out). L agrees with mg90-b within 2 % on every model and duty, 0.84 against
0.83 mH over both duties, tau 199 against 194 us, the window's L and the fast
ramp within 1 to 5 %, the ripple-to-fit ratio within 0.003, and the first
burst's jump after a reversal within a point, 10.3 against 11.1 %. The burst R
spreads 2.6 % over the holes against mg90-b's 13 %, and the low triple of L
sits at holes 1, 5 and 9 on this seat against 3, 7 and 11 on mg90-b's.

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

Run: `uv run python -m oscnb.sitting free` for the reference, the ladder
and the slow rungs, `python -m oscnb.sitting meter` for the meter runs,
`--include swapped` or `spinning` for those, then the notebook.

Second motor: mg90-c's ladder, meter and swapped runs through the same cells,
every number re-derived from its own captures. The back-EMF constant reads
7.316 mV per rev/s against mg90-b's 7.341, 0.34 % apart with bounds about 1 %
wide on each, the two directions equal to 0.02 %, the spin route -0.19 % as
before, and the trend down the coast again the zero's, flat under no zero. The
reading term reads 4.12 to 4.25 mA/V at 20 to 40 % and falls to nothing at 100
% as on mg90-b, within its systematics, while the metered rungs, run after the
brush change, draw 5 to 15 % more current than mg90-b's. Its 7.4 V block does
not read under its 5.0 V block at matched speed, its direction split under
drive is there at most rungs, and the 3.7 V 15 % rung sits under the window's
floor and is left out. `results/mg90-c.json` carries every key but the
spinning run's.

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

Second motor: mg90-c's ladder, meter-1 and high-1 through the same cells,
every input re-derived from its own captures, since no mg90-c bemf-coast
result existed. $K_e$ 1.1652 mV s/rad against 1.1684, the reading term 4.18
mA/V against 4.29 with the same fall by duty, and the 7.4 V rungs on the 5.0 V
curve with the term out within -3 to +2 mA. The routes to $J$ through the
coast land within 1 to 3 % of mg90-b's, the spin-up's ratio at 5.93 against
6.30, and only the spin-up's line does not transfer, 3.88 against 5.44, which
sets the bracket's low end. As delivered the motor drew 3 to 9 % less than
mg90-b at the same rungs and ran faster; its meter runs after the brush change
read 3 to 8 mA more. Its 3.7 V level, which ran first, sits 3 to 8 mA under
the 5.0 V curve, and its free-rotor drag is about 5 % lower at 100 rev/s.

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

Run: the `capture` plan of `uv run python -m oscnb.sitting free`; the notebook
comes after rest-resistance and inductance, its last inputs.

Second motor: mg90-c's `speed-torque/capture-1`, state A as delivered, goes
through the same cells with every constant from mg90-c's own result files of
bemf-coast, no-load-current, rest-resistance and inductance, and writes
`results/mg90-c.json` with the same keys. The curve comes out the same within
the method's spread: $K_e$ within 0.3 %, the stall current within 1.5 %, the
stall torque within 1.5 % by the first millisecond, the peak power within 0.5
% at the same torque, and $J$ 6.14 against 6.26 x $10^{-9}$ kg m$^2$ with
overlapping brackets. The 20 mA the 100 % drive draws over the term-out drag
curve shows on both motors. What differs is the settled end, where mg90-c runs
3 to 4 % faster at no load, drops 0.13 V less besides its EMF and drops out
half as often, a state the record shows lasting only until its first spins
over the rated voltage. The section also reads all eleven mg90-c run-up sets
and the eight slow references as the brush-state record: the fast run-up's
dropout rate steps up after each exposure over the rated voltage as rail times
duty, including 0.7 s of 7.4 V pulses that never passed 300 rev/s, does not
rise after 7.4 V held under it, and the slow reference sees none of it;
whether the run-ups themselves add to it was not separated.

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

Second motor: mg90-c is not torn down. The question the teardown answers was
settled on mg90-b, and mg90-c stays whole as the motor the other notebooks
grade their methods on a second time and as the motor whose brushes were
changed on purpose, so a later teardown against mg90-b's photographs is the
way to see what changed. The notebook quotes the terminals' record from the
dataset's README, the run-up's dropout rate stepping after every exposure to
7.4 V, with or without speed, and the crest falling a few milliamps each time
while the slow reference held, and lists three candidates, eroded finger tips,
deposits at the segment edges and a lubricant film, each with the observation
on an opened mg90-c that separates it. It writes no `results/mg90-c.json`.

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
100 %, high confidence, about 19 mA over the mean at 5.0 V and 29 mA at
7.4 V up to 30 % by the board's current zero, and about 25 to 26.5 mA at
7.4 V by the zero a driven bridge sits on, which the current dropouts read
as the open bridge's. Whether that loss sits across the inductance or in
the rung is moderate at most. By the board's zero the spinning motor needs
about a third more loss than the locked one, and by the driven zero its
meter points need none and all its points 12 % more, so the gap is mostly
the zero, high confidence at 7.4 V and moderate at 5 V, and a DMM in series
at 7.4 V confirms it. The 20 mA speed-torque found at 100 %
is not a window current, high confidence, and a drag of the run-up's first
few hundred milliseconds rather than of the settled drive, moderate
confidence. On the turning motor the terminal volts carry a
brush drop beyond R i and the EMF, a voltage set by the speed and the
direction, within about 10 mV of zero below 160 rev/s at 5 V beside a 10
to 25 mV rail term of the construction at 7.4 V, about 45 mV at 300 rev/s
and 0.37 V at 550 rev/s. High confidence that it is in the motor, moderate
that it does not scale with the current, since a part of up to about 0.3
ohm that does is not excluded. Below 160 rev/s the running resistance is
the resistance at rest within 3 to 4 %.

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

Second motor: the same code, run through `second_motor.py` on mg90-c's own
result files and captures. The locked model holds: the Rayleigh, G and rung
form fits hole 11 at 1.94 per point as on mg90-b, with L0 0.516 mH, G 2.49 mS
in fast decay and the rung at 0.264 mH and 38 us beside mg90-b's 0.505, 2.49
and 0.240 and 39, and only the sign term of L(i) changes sign. The spinning
loss equals the locked loss once the current is read on the zero a driven
bridge sits on, 2.43 to 2.54 mS against 2.49; by the board's constant zero it
reads 3.03 mS at 3.5 per point, because the new 3.7 V rail's zero sits under
that constant, and the dropout rows read 2.5, 6.0 and 10.2 counts at 3.7, 5
and 7.4 V. The coast still reads as a meter at 0.990, the 100 % run-ups sit 20
mA over the term-out curve on both motors, and $V_b$ has the same shape and
direction split, 52 mV at 300 rev/s with exponent 2.88 against 45 mV and 3.55,
smaller at full speed with half the dropouts. The 3.7 V 15 % cycles are left
out as bemf-coast left them, and the teardown-side keys are not written.

## [identification](identification/identification.ipynb)

Targets: whether the board alone recovers each motor constant a back-EMF
speed estimate needs, and how close each board-only route lands to this
stage's bench references, at what cost in maneuvers, seconds and motor
revolutions against a servo's travel of about 157 motor revolutions. The
routes are $R$ from one, two and three locked seats, $R$ against the board
NTC with the winding's self-heat bounded and a low-speed reference rung as
a relative thermometer, $K_e$ from the coasts of one repeat, $L$ from a few
bursts at one seat, the reading term $\Delta$ and $c = J/(K_t K_e)$ from the
100 % anchor, and $V_b$ from three rungs, read as a coefficient on the rate at
which the motor's current drops out, the board's own count, and a speed table
by mechanical direction beside it. It scores no speed. The compensation
notebook takes these constants into the estimator.

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

Second motor: the same passes run on mg90-c's own datasets through
`second_motor_ident.py`, each graded against mg90-c's own result files, with
mg90-b as the comparison only. The locked routes and the coasts carry. R from
one seat lands within 0.92 % sd against 0.91 %, L from one seat within 2.7 %
against 2.8 %, and Ke from one repeat within 0.13 % held out against 0.05 %.
k_d reads 0.44 against 0.52. Two rules fail on this motor and are reported,
not worked around. The pinned coast read gives a negative c on one cut in six,
which takes pass 1's c and $\Delta$ down (+68.5 mA on the grid, -0.31 of the
DMM), while the window read of the same cuts sits at 0.92 to 1.01. The ripple
tach's gate drops two 7.4 V 60 % spins, which leaves the minimal V_b table in
direction +1 64 mV rms from the full-data one. `results/mg90-c.json` carries
mg90-b's keys plus `pinned_c_negative_cuts` and `tach_gate_dropped_spins`.

## [compensation](compensation/compensation.ipynb)

Targets: how accurate a back-EMF speed estimate from the board's own signals
is against the ripple tach, with the reading term $\Delta$ and $V_b$
compensated. The signals are the duty, the terminal volts over the window, the
crest current sample, the rail and the dropout rate. It is asked twice, with
this stage's reference constants and with the constants identification
recovered board-only, and the gap between the two is the price of identifying
without a meter. Beside it, the reading term's table per volt and whether it
holds across rails, $V_b$ in three forms, a power law, a speed table, and the
speed table with a coefficient on the dropout rate, whether its direction
split is the rotation or the dropouts, how it moved between two sittings, the
rail term the construction leaves under 160 rev/s, the estimate's error budget
by speed band, and how few rungs an ident needs for $V_b$.

Needs: no bench. bemf-coast's capture-1, swapped-1, meter-1 and high-1,
speed-torque's run-ups through its own cells, and for the identified constants
rest-resistance's stall pass and inductance's capture-1 through their own
cells. The stage 1 current-scale, terminal-chain, sample-timing, bridge-states
and ntc-curve results, and the rest-resistance, inductance, bemf-coast,
speed-torque, motor-model and identification results.

How: the estimate is the root of the motor's equation per spin over the last
200 ms before the cut, by bisection, four ways, with neither term, either one
alone and both, and $V_b$ fitted against the ripple tach so the estimator
inherits nothing of $K_e$'s speed trend. capture-1 is scored by the fold that
left its repeat out of $V_b$, with $\Delta$ at a $G$ fitted on the meter runs
alone, and swapped-1 and speed-torque's one-turn windows were never in a fit.
The same estimate then runs on identification's 36 board-only passes, and on
each identified constant alone in the reference set, to split the price by
constant.

Run: no capture of its own. `compensation.py` beside the notebook holds the
estimator and its scoring on top of identification's module, and the grid's
steady states run on every core.

Second motor: the same scoring runs on mg90-c through `second_motor_comp.py`,
with mg90-c's own reference constants and identification's three mg90-c
passes, and mg90-b as the comparison only. Under 350 rev/s the free shaft
reads 0.41 to 2.64 % rms by band against 0.45 to 2.44 %, the run-ups 2.35
against 2.54 %, and the 3.7 V rungs, which no fit saw, 1.50 %. swapped-1, run
after the brushes changed, reads +4.1 % at 469 rev/s with the speed table and
+1.3 to +1.7 % with k_d. R, Ke and L from the board cost nothing visible.
$\Delta$ by the anchor costs 3.6 points on capture-1 against 0.6 on mg90-b,
and the pass with the negative pinned c reads 28.7 %. No V_b subset holds the
0.2 point rule once the gate takes a repeat's top rung, and the one complete
repeat of 7.4 V 30, 40 and 60 % does. `results/mg90-c.json` carries mg90-b's
keys plus the 3.7 V set.

## What the stage concludes: four terms, one pass
Four terms stand between the terminal volts and the motor's speed. The stage
measured each on mg90-b, checked each on mg90-c, and ends with how a user's
bench identifies it with one supply and no meter, and how the firmware
compensates it. The identification is one pass on the servo, inside the rated
voltage as rail times duty: a rest, locked bursts, then coast rungs.
The current zero. The shunt amplifier reads one zero with the bridge asleep
and a higher one awake, the step growing with the rail, 2 to 8 mA from 3.7 to
7.4 V on this board, and it wanders a count or two over a sitting. It is the
board's, not the motor's. Nothing is identified: the install's rest seeds it
and a live two-zero tracker learns the rest, fed by coasts, brakes and the
slow-decay trough under low duties, which stage 1's compensation notebook
grades at 0.3 to 0.4 rev/s steady and up to 2 rev/s for the first second after
a rail change.
The reading term. The board samples the shunt once per period near the crest,
and the period mean sits under it at partial duty by 3.4 to 3.9 mA per volt of
the window at 10 to 40 %, tens of milliamps on the free shaft at 7.4 V,
nothing at 100 %, set by the winding's L and a loss conductance of about 2.5
mS that came out the same on both motors in the locked fit. The locked bursts
already run for R and L give both constants. The firmware turns each crest
reading into a mean through that model at the live duty and rail before it
enters the estimate; what the model leaves at speed lands in the brush table,
fitted on the same construction. About a milliamp is left, held out, on either
motor.
The brush drop. 100 to 300 mV across the brushes at speed, different by
direction, plus a share that follows the dropout rate at about half a
millivolt per dropout per second, and it moves with the state of the brushes.
Rungs over the speed range in both directions, each ending in a natural coast,
give Ke and the true speed at the cut from the coast and the leftover voltage
from the drive, so one pass yields Ke, the speed table per direction and the
per-dropout coefficient. The estimate is the root of the motor's equation with
the table inside it. The dropout rate over a window is also the flag: when it
sits well above the identified rate, the tables are stale and the estimate is
marked, never trimmed live; re-identification is the user's act. One to two
percent is left above 300 rev/s, a fixed voltage of a few rev/s at a crawl.
R and Ke with temperature. Copper rises 0.39 % per degree. The identified R is
tagged with its temperature, the copper coefficient is physics, and the
magnets' coefficient on Ke, a fraction of a percent per degree and one
constant per motor family, is not measured in this stage; the bare motors ran
within a few degrees of room. Both scale from the identified values by the
winding temperature the thermal model carries. No coast is injected at runtime
to read them.
Two rules frame the chain. Nothing puts more than the rated voltage across the
motor as rail times duty: 7.4 V at 70 % changed mg90-c's brushes within
seconds, 67 % did not, and so did 0.7 s of 7.4 V pulses from rest that never
passed 300 rev/s, so the line is electrical and the firmware's duty cap tracks
the rail. And the dropout rate is the flag that the brushes have moved anyway.
What the second motor added: the constants carry, Ke within 0.3 %, R within
1.2 % at a board 3 C cooler, L within 1 %, the loss conductance the same; the
identification carries with two fragilities for the next stage's requirements,
a pinned read of the drag coefficient that one cut in six inverted, and a
speed gate that excluded real spins; and the mechanism behind the brush
change, eroded tips, edge deposits or a film, stays open, with mg90-c kept
whole for the teardown that would tell.
