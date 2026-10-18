"""Hinge opening angle from the bend servo position (#63).

`alpha` is the opening angle between the legs: pi flat, smaller folded (see
`talos_servo.model`). It changes `alpha_per_servo_rad(leg_gear_ratio)` times
the servo angle, so with the flat servo position known:

    alpha = pi + (steps - flat_steps) * RAD_PER_STEP * alpha_per_servo

`steps` and `flat_steps` are joint steps as `Robot` reports them, after
`inverted`, so folding always lowers the steps; flip `inverted` if the hinge
folds the other way. `RobotConfig.hinge` turns the raw
`model.bend_flat_steps` into joint steps.

The servo runs in position mode, which covers one turn (0..4095) and never
crosses from 4095 to 0. So there is no wrap to count: instead the whole range
from flat to max fold must fit inside one turn, and `Hinge` refuses a flat
position where it does not. Fix that by turning the servo horn on its
spline, not by writing an EEPROM offset.

No ROS here.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from talos_servo.bus import MAX_POSITION, RAD_PER_STEP
from talos_servo.model import alpha_per_servo_rad

if TYPE_CHECKING:
    from talos_servo.config import ModelConfig

# Opening angle when the robot lies flat.
FLAT_RAD = math.pi
# A reading this far outside the hinge step range is not a real angle (the
# servo wrapped past flat or misread), so it is reported as unknown.
READ_MARGIN_STEPS = 64


def alpha_at(steps: int, flat_steps: int, leg_gear_ratio: float) -> float:
    """Opening angle (rad) at `steps` with flat at `flat_steps`, both in
    joint steps."""
    per_step = RAD_PER_STEP * alpha_per_servo_rad(leg_gear_ratio)
    return FLAT_RAD + (steps - flat_steps) * per_step


class HingeRangeError(ValueError):
    """The hinge range does not fit in one servo turn from this flat
    position."""


class Hinge:
    """Converts between the bend servo steps and the opening angle alpha."""

    def __init__(self, model: ModelConfig, flat_steps: int) -> None:
        self.joint = model.hinge_joint
        self._gear = model.leg_gear_ratio
        self.min_rad = model.bend_min_rad
        self.max_rad = model.bend_max_rad
        self.flat_steps = flat_steps
        self._rad_per_step = RAD_PER_STEP * alpha_per_servo_rad(
            model.leg_gear_ratio
        )
        low, high = self._raw_steps(self.min_rad), self._raw_steps(self.max_rad)
        if not 0 <= low <= high <= MAX_POSITION:
            raise HingeRangeError(
                f"{self.joint}: with flat at step {flat_steps} the hinge "
                f"range {math.degrees(self.min_rad):g}.."
                f"{math.degrees(self.max_rad):g} deg needs steps "
                f"{low}..{high}, outside one servo turn 0..{MAX_POSITION}. "
                "Turn the servo horn so flat sits further from the end"
            )
        self.step_range = (low, high)

    def alpha(self, steps: int) -> float:
        """Opening angle (rad) at a servo position in joint steps."""
        return alpha_at(steps, self.flat_steps, self._gear)

    def alpha_rate(self, step_speed: int) -> float:
        """Opening speed (rad/s) at a servo speed in steps/s."""
        return step_speed * self._rad_per_step

    def steps(self, alpha: float) -> int:
        """Servo goal for an opening angle, clamped to the hinge limits."""
        if not math.isfinite(alpha):
            raise ValueError(f"{self.joint}: angle must be finite, got {alpha}")
        return self._raw_steps(min(self.max_rad, max(self.min_rad, alpha)))

    def _raw_steps(self, alpha: float) -> int:
        return self.flat_steps + round((alpha - FLAT_RAD) / self._rad_per_step)
