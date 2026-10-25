# Stage 0. Instruments

The rulers the other stages grade against. Each gets an error bar and a
clock offset here before it measures anything. No board and no servo on the
bench, so this sitting needs only the host, the DPS and the DMM, plus
the camera and the thermocouple for their own experiments.

## DMM

Targets: the lag of the DMM's log against the host clock, and its spread.
Lands in the DMM's registry entry.

Needs: DMM, DPS, host.

How: the host steps the DPS voltage at logged times while the DMM logs
DC volts. The step edges in the DMM log against the host times give the
lag. The reading accuracy comes from the datasheet and is written into the
result, not measured.

Result: `results/instruments/dmm.json`.

## power-resistors

Targets: the value of each load and its drift with power. Lands in the
loads' registry entries, which stage 1 reads.

Needs: DMM, DPS, loads, host.

How: ohms at rest from the DMM, then a DC current from the DPS with the
DMM in series for the current and the DPS's readback for the voltage.
Resistance against power gives the self-heating slope.

Result: `results/instruments/power-resistors.json`.

## camera-tag

Targets: the angle error and the noise floor of the tag pipeline on the
real bench. Lands in the camera's registry entry, which the pot experiments
in stage 3 read.

Needs: camera, a plate to glue the tags to, host.

How: two tags printed on one sheet, so their relative angle is fixed by the
print, glued to a plate. The plate is turned by hand through a full turn in
steps and filmed from above after a lens calibration from the board video.
The relative angle has to come out the same at every step, and a still of
the plate gives the frame to frame noise.

Result: `results/instruments/camera-tag.json`.

## thermocouple

Targets: the thermocouple's offset, slope and time constant. Lands in the thermocouple's
registry entry, which the thermal experiments in stage 3 read.

Needs: thermocouple, DMM, an ice bath and boiling water.

How: the ice bath gives the zero, boiling water corrected for altitude gives
the slope, and dipping the thermocouple from air into the bath gives the time
constant.

Result: `results/instruments/thermocouple.json`.

## clock-sync

Targets: the offset and drift of each instrument's clock against the host
log. Lands in the camera's and the DMM's registry entries.

Needs: clapper, camera, DMM, DPS, host, and the experiment firmware image
for the clapper.

How: the host flashes the clapper in a known pattern and steps the DPS at
the same moments. The camera sees the flash, the DMM sees the step, the
host logs both, and one fit per instrument gives offset and rate.

Result: `results/instruments/clock-sync.json`.
