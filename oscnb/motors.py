"""The motor registry: one entry per physical motor.

A motor has no datasheet, so an entry holds only what can be counted or
read off the part: its family, the servo it came out of, the slot and brush
counts that set the commutation ripple, and the pinion. Every electrical
and mechanical constant is a stage 2 result, read from the results files.
"""
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
        notes="runs bare on the stage 2 rig through every experiment, then is torn down for the last one; "
              "a second motor of the same batch gives the sanity readings in rest-resistance, no dataset",
    ),
}
DEFAULT = "mg90-b"
