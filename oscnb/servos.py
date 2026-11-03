"""The servo registry: one entry per physical servo.

A servo is a motor, a gear train, a pot and a case. Like a motor it comes
with no datasheet worth the name, so an entry holds what can be counted or
read off the part, the limits the bench holds it to, and the numbers earlier
work measured on another unit of the family, kept for comparison and for
sizing a capture, never as a result. Every constant a later stage reads is
a stage 3 result, read from the results files.

One key names one physical unit: the servo and the motor inside it share
it, so a capture lands under `telemetry/<servo>__<board>/` and the motor
entry of the same key carries the motor's counts.

The rated voltage is a limit, as for a bare motor: nothing puts more than it
across the motor as rail times duty. The three currents are limits too. A
lap into the horn lock draws at most the lock hold, the servo's own stall
rating, since the bolted horn puts the torque through the train with no
stopper in the path. Any other held stall, an arm stood against gravity,
draws at most the hold current the gear class allows. A lap pressed into an
end stop draws at most the lap cap, lower, since the output stopper is
plastic on a metal train and took a set on another unit at about 0.32 A.
"""
import os
from dataclasses import dataclass

from . import motors


@dataclass(frozen=True)
class Servo:
    key: str
    label: str
    family: str
    motor: str                    # the motor entry inside it, the same key
    gear_ratio: float             # motor revolutions per output revolution
    gear_ratio_source: str
    gear_class: str               # "metal" or "plastic", sets the hold current
    spline_teeth: int | None      # counted on the bench, None until counted
    pot_counts: int               # the converter's span the pot reads over
    counts_per_deg_nominal: float  # earlier work, comparison and window sizing
    travel_deg_nominal: float     # earlier work, comparison only
    rated_v: float                # the most rail x duty the motor ever sees
    lock_hold_a: float            # the most a lap into the horn lock draws
    stop_hold_a: float            # the most any other held stall draws
    lap_cap_a: float              # the most a lap into an end stop draws
    output_stopper: str
    type_motor: str               # the stage 2 motor that stands as the type
    notes: str = ""

    @property
    def motor_entry(self):
        return motors.MOTORS[self.motor]

    @property
    def counts_per_motor_rev(self):
        """Pot counts per motor revolution through the train, nominal."""
        return self.counts_per_deg_nominal * 360 / self.gear_ratio


SERVOS = {
    "mg90-d": Servo(
        key="mg90-d",
        label="MG90 clone d",
        family="mg90-vorpal",
        motor="mg90-d",
        gear_ratio=308.0533,
        gear_ratio_source="counted teeth on another unit of the family, "
                          "checked by pot-table",
        gear_class="metal",
        spline_teeth=None,
        pot_counts=4096,
        counts_per_deg_nominal=19.8,
        travel_deg_nominal=184.0,
        rated_v=5.0,
        lock_hold_a=0.85,
        stop_hold_a=0.5,
        lap_cap_a=0.45,
        output_stopper="plastic",
        type_motor="mg90-b",
        notes="a new sealed MG90 clone of the same brand as mg90-b and "
              "mg90-c, run first in stage 3; its motor is never opened, "
              "and mg90-b's stage 2 results stand as the type values, "
              "comparison only. lock_hold_a is the stock MG90's stall "
              "rating, 1.8 kg cm at 4.8 V (0.177 N m), through the 308:1 "
              "train at about 0.6 efficiency, 0.96 mN m at the motor, over "
              "the type Kt of 1.16 mN m per A: about 0.75 to 0.85 A at the "
              "motor, the top of that range taken, which a 100 % lap holds "
              "to 3.7 V by the type R",
    ),
}
DEFAULT = "mg90-d"


def selected():
    """The servo a capture runs on: OSC_SERVO in the environment, else the
    default, the way OSC_MOTOR picks a bare motor."""
    key = os.environ.get("OSC_SERVO", DEFAULT)
    if key not in SERVOS:
        raise SystemExit(f"OSC_SERVO={key} is not in the registry: "
                         f"{', '.join(SERVOS)}")
    return SERVOS[key]
