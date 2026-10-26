# Stage 0. Instruments

The rulers the other stages grade against. Each gets an error bar and a
clock offset here before it measures anything. No board and no servo on the
bench, so this sitting needs only the host, the DPS and the DMM, plus
the camera and the thermocouple for their own experiments.

Each experiment is a folder here with its `capture.py`, its notebook and
the `results.json` the notebook writes. With no servo and no board, the
datasets land in `telemetry/instruments/<experiment>/capture-N/`.

## [dmm](dmm/dmm.ipynb)

Targets: the lag of the DMM's log against the host clock, and its spread.
Lands in the DMM's registry entry.

Needs: DMM, DPS, host.

How: the host steps the DPS voltage at logged times while the DMM logs
DC volts. The step edges in the DMM log against the host times give the
lag. The reading accuracy comes from the datasheet and is written into the
result, not measured.

Run: `uv run python stages/0-instruments/dmm/capture.py`, then `dmm/dmm.ipynb`.

Result: `dmm/results.json`.

## [power-resistors](power-resistors/power-resistors.ipynb)

Targets: the value of each load and its drift with power. Lands in the
loads' registry entries, which stage 1 reads.

Needs: DMM, DPS, loads, host.

How: one run with no load and the DMM on DC A carrying the DPS current, for
the lead and ammeter resistance. Then one run per load: ohms at rest from the
DMM, then the DPS current through the DMM and the load with the DPS's readback
for the voltage. Resistance against power gives the self-heating slope.

Run: `uv run python stages/0-instruments/power-resistors/capture.py <bare|cap|series|leads>`
for each, then `power-resistors/power-resistors.ipynb`.

Result: `power-resistors/results.json`.

## [camera-tag](camera-tag/camera-tag.ipynb)

Targets: the angle error and the noise floor of the tag pipeline on the
real bench. Lands in the camera's registry entry, which the pot experiments
in stage 3 read.

Needs: camera (any recent iPhone, filming 1080p slow motion at 240 fps, with
the settings in the notebook's Procedure), an overhead mount holding it about
20 cm above the desk, a plate to glue the tags to, host.

How: two tags printed on one sheet, so their relative angle is fixed by the
print, glued to a plate. The lens is calibrated from a clip of the flat
board filmed at the working distance, then the plate is turned by hand
through a full turn in steps and filmed from above at that same distance.
The relative angle has to come out the same at every step, and a still of
the plate gives the frame to frame noise.

Run: `uv run python stages/0-instruments/camera-tag/capture.py`, which prints the sheets
and waits for the clips, then `camera-tag/camera-tag.ipynb`.

Result: `camera-tag/results.json`.

## [thermocouple](thermocouple/thermocouple.ipynb)

Targets: the thermocouple's offset, slope and time constant. Lands in the thermocouple's
registry entry, which the thermal experiments in stage 3 read.

Needs: thermocouple, DMM, an ice bath and boiling water.

How: the ice bath gives the zero, boiling water corrected for altitude gives
the slope, and dipping the thermocouple from air into the bath gives the time
constant.

Run: `uv run python stages/0-instruments/thermocouple/capture.py`, then
`thermocouple/thermocouple.ipynb`.

Result: `thermocouple/results.json`.

## [clock-sync](clock-sync/capture.py)

Targets: the offset and drift of the camera's clock against the host log,
and how late the lamp follows the host's write. Lands in the camera's
registry entry. The DMM's own lag against the host is the dmm experiment.

Needs: clapper (a servo board on the bus, with a firmware that has
`stat_flash_ticks`), the `osc` tool, camera, host.

How: `osc clapper` flashes the servo's STAT lamp a dozen times at irregular
gaps, logging the host time either side of each write, while the phone
films the lamp. The lamp's level per frame comes out of the clip in the
extraction; the flash onsets against the logged writes give offset and
rate by one straight-line fit.

Run: `uv run python stages/0-instruments/clock-sync/capture.py`, then
`clock-sync/clock-sync.ipynb`.

Result: `clock-sync/results.json`.
