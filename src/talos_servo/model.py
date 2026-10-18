"""Body geometry that changes with the bend hinge.

The robot is an inverted V (sketch in #76): the hinge on top and a leg of
length `l` down to each wheel pair. `alpha` is the opening angle between the
legs, pi when flat. Values come from the `model` section of
`config/robot.yaml`, the same one `description/talos.urdf.xacro` reads.
"""

from __future__ import annotations

import math


def wheelbase_m(alpha_rad: float, leg_length_m: float) -> float:
    """Front to rear axle distance at hinge opening `alpha_rad`.

    `d = 2 * l * sin(alpha / 2)`: `2 * l` flat, shorter when folded.
    """
    if not 0 < alpha_rad <= math.pi:
        raise ValueError(
            f"hinge opening must be in (0, pi] rad, got {alpha_rad}"
        )
    if leg_length_m <= 0:
        raise ValueError(f"leg length must be positive, got {leg_length_m}")
    return 2 * leg_length_m * math.sin(alpha_rad / 2)


def alpha_per_servo_rad(leg_gear_ratio: float) -> float:
    """Change of the hinge opening per radian of the bend servo.

    The servo pinion drives one gear per leg and both legs turn, so the
    opening moves twice as far as each leg gear.
    """
    if leg_gear_ratio <= 0:
        raise ValueError(
            f"leg gear ratio must be positive, got {leg_gear_ratio}"
        )
    return 2 * leg_gear_ratio
