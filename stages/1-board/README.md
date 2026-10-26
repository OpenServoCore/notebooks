# Stage 1. Board

The controller board on its own, with known loads on its motor terminals
and no motor. Every current, voltage and temperature the later stages read
comes through this board's chain, so its conversions are graded here, against
the stage 0 instruments, before anything with a motor is measured. The board
is `dev-v006-2A-D`, the rev 2A board D; its registry entry in
`oscnb/boards.py` carries the parts and the firmware's published constants,
and the results here carry the measured corrections.

The sitting needs the board on the DPS, the adapter on its bus at 3 Mbps,
the stage 0 loads (the bare 5 ohm and the series 15 ohm power resistors),
the DMM, the thermocouple and the filament dryer for the two thermal
experiments. The `osc` tool on the PATH. Datasets land in
`telemetry/<board>/<experiment>/capture-N/`, and each notebook writes
`results/<board>.json` in its folder.

Supply levels are rail volts, VSYS, from a 1S pack's 3.3 V through USB to a
2S pack's 8 V. A swap board has no feed diode and its rail is the battery, so
the levels name the rail, and each capture trims the DPS until the board's
own rail tap reads the level. The DPS output comes up at its previous
setting for a moment when enabled, so every script turns it on before the
board's leads go on.

The bridge-states experiment on the RL dummy waits for the inductor.

Run in this order. The lap experiments share one wiring and the dryer
experiments share another, and opamp-temperature reads the ntc-curve result.

## [rest-noise](rest-noise/rest-noise.ipynb)

Targets: the noise floor of each raw channel with the bridge off, and any
structure in it.

Needs: board, DPS, bare load, host.

How: torque-off streams at the control rate, ten seconds each, three per
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

Targets: the terminal taps, the rail tap, the bias node and the ADC reference
against the meter, and whether the reference holds on a 1S rail.

Needs: board, DPS, bare load, DMM on DC V, host.

How: the same laps with the meter at one node per pass: across the load,
MOT_A to GND, VSYS to GND, and the 3V3 test point at rest at every level.

Run: `capture.py load`, `capture.py tap_a`, `capture.py rail`, `capture.py vdd`, then the notebook.

## [sample-timing](sample-timing/sample-timing.ipynb)

Targets: the duty from which each crest sample reads the drive honestly,
against the floors the firmware publishes, and the shunt's settle inside a window.

Needs: board, DPS, bare load, host.

How: duty ladders from 3 to 40 % with the 100 % rung as the control, both
directions and both decays at 7.4 V and slow decay again at 3.7 V, then
free-running ADC bursts folded onto the PWM period.

Run: `uv run python stages/1-board/sample-timing/capture.py`, then the notebook.

## [ntc-curve](ntc-curve/ntc-curve.ipynb)

Targets: the board NTC against the thermocouple, the firmware's conversion
graded, and its constants refitted.

Needs: board in the dryer, DPS, thermocouple taped to the board, DMM on temperature, host.

How: the NTC polled while the dryer takes the board to about 45 C, then 60 C,
then back to room with the lid open; the thermocouple logged throughout.

Run: `uv run python stages/1-board/ntc-curve/capture.py`, then the notebook.

## [opamp-temperature](opamp-temperature/opamp-temperature.ipynb)

Targets: the current chain's gain and zero against board temperature.

Needs: board in the dryer, DPS, bare load, DMM on DC A outside the dryer, host; the ntc-curve result.

How: the DC laps at four plateaus of the dryer, the NTC in every stream; a
line per plateau gives the gain and offset, and their slope with temperature
is the drift.

Run: `uv run python stages/1-board/opamp-temperature/capture.py`, then the notebook.
