"""The motor registry: one entry per physical motor.

A motor has no datasheet, so an entry holds only what can be counted or
read off the part: its family, the servo it came out of, the slot and brush
counts that set the commutation ripple, the pinion, and the voltage the
servo is sold for. Every electrical and mechanical constant is a stage 2
result, read from the results files.

The rated voltage is a limit, not a measurement: nothing on the bench, in
identification, in calibration or in firmware puts more than it on the
motor as rail times duty. On mg90-c, free spins at 7.4 V and 80 to 100 %
changed the brushes within seconds, where 7.4 V at 60 % and 5 V at 100 %
left them as they were.
"""
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Motor:
    key: str
    label: str
    family: str          # the servo family the motor is sold in
    source: str          # the servo it came out of
    slots: int           # rotor slots, a parts count
    brushes: int
    pinion_teeth: int
    rated_v: float       # the servo's rated voltage, the most rail x duty the motor ever sees
    notes: str = ""

    @property
    def ripple_per_rev(self):
        """Commutation events per motor revolution: the ripple in the current
        and in the coasting terminal voltage counts the speed."""
        return self.slots * self.brushes


MOTORS = {
    "mg90-b": Motor(
        key="mg90-b",
        label="the motor of MG90 clone b",
        family="mg90-clone",
        source="a new Vorpal-brand MG90 clone opened for stage 2, the servo never run",
        slots=3,
        brushes=2,
        pinion_teeth=9,
        rated_v=5.0,
        notes="runs bare on the stage 2 rig through every experiment, then is torn down for the last one",
    ),
    "mg90-c": Motor(
        key="mg90-c",
        label="the motor of MG90 clone c",
        family="mg90-clone",
        source="a second MG90 clone opened after mg90-b's teardown",
        slots=3,
        brushes=2,
        pinion_teeth=9,
        rated_v=5.0,
        notes="runs bare on the stage 2 rig as the second motor the identification and compensation methods are graded on",
    ),
}
DEFAULT = "mg90-b"


def selected():
    """The motor a capture runs on: OSC_MOTOR in the environment, else the
    default. The notebooks read the default; a capture on another motor sets
    the variable so its data lands under that motor's own folder."""
    key = os.environ.get("OSC_MOTOR", DEFAULT)
    if key not in MOTORS:
        raise SystemExit(f"OSC_MOTOR={key} is not in the registry: {', '.join(MOTORS)}")
    return MOTORS[key]
