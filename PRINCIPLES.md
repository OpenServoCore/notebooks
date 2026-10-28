# Principles

How the experiments in this repo are done, and the rules that keep the
stages in order.

## Principles

### Measured, not assumed

A number carries its error bar and the notebook it came from. A claim the
data does not support is labelled as such.

### One experiment per quantity

The route the tool runs is the one that stays, graded once against the
stage below or against an instrument. A second route for the same quantity
is cut. A route shown not to work is cut as well, with the proof kept as a
line.

### Results flow down the stages

A stage reads the results of the stages before it from their result files
and does not recompute them.

### Open loop

Experiments are excited open loop. A closed loop appears in the control and
safety stages only, as the thing under test.

### One shape for every notebook

Purpose, procedure, data, method, results, limitations, open questions and
a conclusion, in that order. A reader who knows one notebook knows where to look in all of
them.

### Open items come with a test

A discrepancy is listed with the candidate causes and the measurement that
would tell them apart. Without that test it is a note, not a finding.

### One teardown per motor family

The teardown gives the masses and the pure copper resistance that score the
non-destructive methods. It is done once per family, not per servo.

### Firmware follows the notebook

A firmware change that rests on a measurement waits for the notebook. The
other direction is open, a bench finding can change a notebook at any time.
A notebook never describes what the firmware does today, since the firmware
is built from the notebooks and not the other way round. It says what the
data supports and what the firmware should carry.

## Rules

### Instruments are not dependencies

Speed from a camera, temperature from a thermocouple or current from a DMM are
rulers. They live in stage 0 and grade the rest. Stage 4 holds the
real-time estimators the firmware can run.

### Safety is the host's job during a capture

Captures run on an experiment firmware image that strips the modes, the
estimators and the safety features, keeping open-loop duty, raw streaming,
register access and two host-set protections, a raw current trip and a bus
dead-man. An experiment that cannot run on that image has a dependency it
has not declared.

### A stage is a bench setup

The experiments of one stage are captured in one sitting with the same rig.

### Board type, latest revision

The board axis is the board type. Only the latest revision of a type is
kept, since old revisions go stale quickly. A board change costs a registry
entry and a re-capture. Notebooks above stage 1 do not hardcode board
values.

### One dataset per servo and board pair

Per experiment, one dataset, the current one. A re-record replaces it in
place so the notebook and its data stay in sync. Repeats within a sitting
are the `capture-N` folders of that dataset.

### One supply

Captures run on the DPS, a FNIRSI DPS-150 bench supply, which also sweeps the
rail. Batteries vary from pack to pack, so how the firmware copes with them
is checked in the safety stage rather than measured here.

### Dataset paths

`telemetry/<servo>__<board>/<experiment>/capture-N/`. No revision and no
supply in the path. Both are fields in the metadata.

The stages these rules apply to are described in the [README](README.md).
