# notebooks

This is where I measure how a servo behaves, and where the firmware in
[open-servo-core](https://github.com/OpenServoCore/open-servo-core) gets
its reasons from. A loop, a limit or a default over there should have a
notebook in here showing the data behind it.

## The stages

The work is split into stages. Each stage reads the results of the stages
before it, and each stage is also one bench setup, so the reading order and
the bench order are the same.

### 0. Instruments

Targets the rulers. The DMM, the power resistors, the camera tag, the
thermocouple thermocouple and the clock that lines their logs up.

The later stages compare their results against these, so each one gets its
own error bar first. No board and no servo on the bench.

### 1. Board

Targets the controller board, one type at a time (dev board, swap board),
latest revision only.

Known loads only, power resistors and an RL dummy, with the rail swept by
the DPS. This gives the conversions from ADC counts to volts and
amps, the sample timing, the noise floor and the board NTC curve. Shares a
sitting with stage 0.

### 2. Motor

Targets the motor on its own, by type. Today that is a small brushed DC
motor.

The bare motor is clamped on a plate with the DMM across its tabs.
Resistance, inductance, the back-EMF constant and the no-load current are
measured here with nothing else in the way. Stage 3 is graded against these
numbers. Same sitting as stage 1.

### 3. Servo

Targets the assembled servo, meaning the motor plus a gear train, a pot and
a case.

Open loop throughout, with the camera tag on the horn and the thermocouple inside
the case. Three groups of experiments live here. Electrical repeats the
stage 2 measurements in situ, the way `osc ident` has to do them. Mechanical
covers friction, inertia, the gear train and the pot. Thermal
covers the heating holds and the winding temperature constants. This is the
long sitting.

### 4. Estimators

Targets the real-time estimators the firmware runs. Speed, position,
current zero and winding temperature.

No new captures. Each estimator is designed offline over the stage 3 data
and graded against the stage 0 instruments.

### 5. Control design

Targets the control loops. Current, velocity, position and trajectory,
including how a hold behaves.

The loops are designed from the measured plant and the estimators' noise
and delay, then validated closed loop on the bench. Each notebook ends with
what the firmware does today next to what the data supports.

### 6. Safety design

Targets the limits and the faults. The current ceiling, the stall and
position-error faults, the thermal derate and cutoff, the stop-hold limits.

Derived from stages 3 to 5 and validated on the bench, including on a
sagging supply.

## How an experiment is laid out

One folder with two files. A `capture.sh` that drives the bench through
`osc` and writes a dataset, and one notebook that fits, grades and writes
its result to `results/`. Each stage folder has a README listing its
experiments and how they chain.

## Purpose

### Derive the firmware's design

Which loops exist and how they are shaped, which limits and faults exist,
and the defaults shared by all servos come out of the measurements here.
The board constants are the only numbers copied into the firmware. Servo
numbers like resistance, back-EMF constant, friction and thermal gains are
reproduced by the tools on the user's own servo.

### Build the methods behind the tools

The characterization, identification and calibration steps that `osc ident`,
`osc cal` and the osc-web calibration flow run on a user's servo start here
as experiments, in long form, with the model and its limits worked out on
real data. An experiment is written for a type of thing, like a brushed DC
motor or a pot, not for one servo model. The MG90 goes through first, the
SG90 second to check the methods carry over.

### Grade the tools

The notebooks hold the outside truth, a DMM, a camera, a thermocouple
thermocouple. The tools are scored against it, and the range where an experiment
holds becomes the range where the tool stops.

### Teach

A hobbyist without an EE degree should be able to follow along. The
notebooks are committed with their outputs baked in, so they read straight
from GitHub.

## Equipment

What the experiments use, by the names the stage READMEs refer to. The
scripts that drive the controlled ones live in `equipment/`.

### DMM

An OWON B41T+ handheld multimeter, 4.5 digits, true RMS, with Bluetooth LE
streaming and a K thermocouple input. It reads DC volts, ohms at about a
milliamp, DC amps and temperature. [equipment/b41t.py](equipment/b41t.py)
connects and logs every packet with the host time of arrival. A review with
the measured accuracy is at
[lygte-info.dk](https://lygte-info.dk/review/DMMOwon%20B41T%20UK.html).
Amazon link: to add.

### DPS

A FNIRSI DPS-150 bench supply, 0 to 30 V, 0 to 5 A, 150 W, fed from a
100 W USB-C PD adapter, scripted over USB serial through the
[fnirsi-dps150](https://pypi.org/project/fnirsi-dps150/) package.
[equipment/dps150.py](equipment/dps150.py) reads it, sets a voltage and a
current limit, and switches the output. The voltage readback agrees with
the DMM within millivolts. The current readback reads well under the
true current and is not used as a measurement. Product page at
[Elektor](https://elektor.com/products/fnirsi-dps150-dc-power-supply-150-w).
Amazon link: to add.

### loads

Three uxcell aluminium-housed wirewound power resistors, 5 ohm 50 W 5 %.
One is used bare, one with a capacitor across it, and the three in series
make about 15 ohm. Their values come from the power-resistors experiment in
stage 0, not from the label. A power inductor of about 1 mH rated 2 A or
more, in series with one resistor, makes the RL dummy for the bridge-state
experiment. Amazon links: to add.

### thermocouple

A welded K-type thermocouple on twisted leads, read on the DMM's
temperature input. Its offset, slope and time constant come from the thermocouple
experiment in stage 0. Amazon link: to add.

### camera

A phone filming two printed ArUco tags, one on the horn and one fixed
beside it, plus a printed ChArUco board for the lens calibration. The
prints come from `oscnb.fiducial_print` and the pipeline lives in
`oscnb.fiducial`. The phone model goes in the camera's registry entry,
since the lens calibration belongs to it.

### clapper

The servo board's status LED, flashed from the host through a register in
the experiment firmware image. Like a film clapper, every recording sees
the same flash, so the camera, the DMM log and the host log can be lined
up afterwards.

### host

The computer running `osc` over the osc adapter, with the capture scripts in
this repo. Python comes through `uv`, see Setup.

## Results

Each experiment writes one file, `results/<stage>/<experiment>.json`, with
the value, its bounds, the unit and the capture it came from. A later stage
reads these files and nothing else from the stages before it.

## Setup

You will need [uv](https://docs.astral.sh/uv/). Then `uv sync` in this folder
installs Python 3.12 and the dependencies, `uv run python equipment/dps150.py read`
checks the DPS, and `uv run jupyter lab` opens the notebooks.

The principles and rules this repo follows are in [PRINCIPLES.md](PRINCIPLES.md).
