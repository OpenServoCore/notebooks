# Stage 1. Board

The controller board on its own, with known loads on its motor terminals
and no motor. Every current, voltage and temperature the later stages read
comes through this board's chain, so its conversions are graded here, against
the stage 0 instruments, before anything with a motor is measured. The board
is `dev-v006-2A-D`, the rev 2A board D; its registry entry in
`oscnb/boards.py` carries the parts' nominal values, and the results here
carry the measured conversions, which is where the firmware takes its
constants from.

The sitting needs the board on the DPS, the adapter on its bus at 3 Mbps,
the stage 0 loads (the bare 5 ohm and the series 15 ohm power resistors),
the DMM, the thermocouple and a controlled heat source for the two thermal
experiments: a 3D printer's heated bed, a hot plate with a thermostat or a
filament dryer, anything that holds the whole board at a set temperature.
The `osc` and `osc-lab` tools on the PATH. The board runs the experiment
image (`scripts/build-image.sh --experiment <rev>` in the firmware repo,
then `osc flash` over the bus): open-loop duty, raw streaming and register
access, with the raw current trip and the bus dead-man as its only
protections, so every other rule of a capture is the script's. Datasets land in
`telemetry/<board>/<experiment>/capture-N/`, and each notebook writes
`results/<board>.json` in its folder.

Supply levels are rail volts, VSYS, from a 1S pack's 3.3 V through USB to a
2S pack's 8 V. A swap board has no feed diode and its rail is the battery, so
the levels name the rail, and each capture trims the DPS until the board's
own rail tap reads the level. The DPS output comes up at its previous
setting for a moment when enabled, which the board's rail tolerates, so the
scripts turn it on with the leads connected.

Run in this order. The lap experiments share one wiring and the heated
experiments share another, opamp-temperature reads the ntc-curve result, and
bridge-states comes last on the RL dummy, the bare load in series with two
power inductors.

## [rest-noise](rest-noise/rest-noise.ipynb)

Targets: the noise floor of each raw channel with the bridge off, and any
structure in it.

Needs: board, DPS, bare load, host.

How: torque-off streams at the control rate, three seconds each, the longest burst the servo arms, three per
level at a 1S, a USB and a 2S level, the board NTC in every row.

Run: `uv run python stages/1-board/rest-noise/capture.py`, then the notebook.

## [current-scale](current-scale/current-scale.ipynb)

Targets: counts per amp and the zero under drive, against the meter.

Needs: board, DPS, bare and series loads, DMM on DC A, host.

How: DC laps at 100 % duty, the DMM in series with the load, at every level.
A line through counts against the meter's amps over both loads. An optional
pass with the DMM across the shunt pads separates the amplifier from the shunt.

Run: `capture.py bare`, `capture.py series`, optionally `capture.py rs1`, then the notebook.

## [terminal-chain](terminal-chain/terminal-chain.ipynb)

Targets: the terminal taps, the rail tap and the bias node against the meter,
and the ADC reference against the chip's own 1.2 V reference, and whether it
holds on a 1S rail.

Needs: board, DPS, bare load, DMM on DC V for three of the four passes, host.

How: the same laps with the meter at one node per pass: across the load,
MOT_A to GND, VSYS to GND. A fine pass with the probes still on VSYS walks
the rail up in 20 mV steps at rest, the DPS set and not trimmed, so the
meter is the x axis: the rail tap's scale on a fine grid, and in its residual
the ADC's own linearity over the codes the tap crosses. The last pass streams
the chip's 1.2 V reference at rest at every level with no meter: VDD = 1.2 V
x 4096 / vref_raw.

Run: `capture.py load`, `capture.py tap_a`, `capture.py rail`, `capture.py rail-fine`, `capture.py vdd`, then the notebook.

## [sample-timing](sample-timing/sample-timing.ipynb)

Targets: the duty from which each crest sample reads the drive honestly,
against the window floors the board registry carries, and the shunt's settle inside a window.

Needs: board, DPS, bare load, host.

How: duty ladders from 3 to 40 % with the 100 % rung as the control, both
directions and both decays at 7.4 V and slow decay again at 3.7 V, with
free-running ADC bursts folded onto the PWM period at each rail.

Run: `uv run python stages/1-board/sample-timing/capture.py`, then the notebook.

## [ntc-curve](ntc-curve/ntc-curve.ipynb)

Targets: the board NTC against the thermocouple, and the count to
temperature conversion the firmware takes from it.

Needs: board on the heater, DPS, thermocouple taped to the board, DMM on temperature, host.

How: the NTC polled while the heater takes the board to about 45 C, then 60 C,
then cools it toward room; the thermocouple logged throughout, and the heater's
own log kept as a side note on the rig, never as a reference.

Run: `uv run python stages/1-board/ntc-curve/capture.py [board]`, once per board of the
revision, then the notebook.

## [opamp-temperature](opamp-temperature/opamp-temperature.ipynb)

Targets: the current chain's gain and zero against board temperature, how
much of that is the shunt's and how much the amplifier's, VDD at each
plateau, and whether the board's own heat under drive moves the NTC.

Needs: board on the heater, DPS, bare load, DMM on DC A off the heater for
the ammeter route or on DC V across the Rs1 pads for the rs1 route, host; the
ntc-curve result.

How: the DC laps at four plateaus of the heater, the NTC in every stream and
a rest stream with the chip's 1.2 V reference before each plateau's laps; a
line per plateau gives the gain and offset, and their slope with temperature
is the drift. The ammeter route reads the chain as a whole, the rs1 route
reads the amplifier alone, and the rest streams read VDD at each plateau.

Run: `capture.py ammeter`, `capture.py rs1`, then the notebook.

## [bridge-states](bridge-states/bridge-states.ipynb)

Targets: the burst inductance method on a load with no moving parts, the
ripple L against the period fit, the fitted R against the DC R, and whether
two inductors add. What the shunt and the taps read in each bridge state
with a real inductive current, in slow and fast decay. The dummy's DC
resistance.

Needs: board, DPS, bare load, two 1 mH power inductors (Bourns 2300LL-102-H-RC), A with orange leads and B with brown leads, Dupont pigtails, host.

How: four wirings, each change adding or removing one part. The bare
resistor alone is the control, then come A, A and B in series, and B.
Each wiring gets DC laps at 100 % on a 3.7 and a 4.2 V rail for the DC
resistance, then on a 5 V rail `osc-lab burst` at 25 and 40 %, both signs,
five repeats each, once in slow decay and once in fast, and duty sweeps at
25 and 40 % with the 100 % rung as the control in both decays, the trough
shunt in the row beside the crest samples as the steady-state check. One
inductor's time constant is about 0.19 ms, four PWM periods, and both in
series about 0.37 ms, seven periods. A slow-decay burst runs about 9 periods
past the step, so it ends near 90 % of the final current on one inductor and
near 70 % on both, and fits L from the ramp inside each window and L/R from
the rise across periods. The bare resistor's current settles with the
amplifier, so on it the method should find next to no inductance. Under
slow decay the current circulates through the low-side switches out of the
shunt's path, which the slow bursts see through the whole period. Under
fast decay it returns to the rail through the body diodes and reaches zero
before the next drive below 50 %, so the fast bursts are periodic from the
first full period and show the turn-off, the return on the taps, the amplifier's
tail and the discontinuous current inside one period.

Run: `uv run python stages/1-board/bridge-states/capture.py`, then the notebook.
