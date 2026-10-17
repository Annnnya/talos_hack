"""Mecanum drive kinematics for four independently driven wheels."""

from __future__ import annotations

import math
from collections.abc import Sequence

from talos_servo.bus import MAX_SPEED

WHEEL_COUNT = 4
DEFAULT_WHEEL_IDS = (1, 2, 3, 4)
DEFAULT_WHEEL_DIRECTIONS = (1, 1, 1, 1)
# ROS joint names, in the same FL, FR, RL, RR order as mecanum_wheel_speeds returns.
WHEEL_JOINT_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)


def mecanum_wheel_speeds(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    *,
    wheelbase_m: float,
    track_width_m: float,
    steps_per_meter: float,
    max_speed: int,
    wheel_directions: Sequence[int] = DEFAULT_WHEEL_DIRECTIONS,
) -> tuple[int, int, int, int]:
    """Map body velocity to FL, FR, RL, RR wheel speeds."""
    values = (linear_x, linear_y, angular_z, wheelbase_m, track_width_m, steps_per_meter)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocities and kinematic parameters must be finite")
    if wheelbase_m <= 0 or track_width_m <= 0 or steps_per_meter <= 0:
        raise ValueError("wheelbase, track width, and steps per meter must be positive")
    if not 1 <= max_speed <= MAX_SPEED:
        raise ValueError(f"max_speed must be within 1..{MAX_SPEED}")
    if len(wheel_directions) != WHEEL_COUNT or any(
        sign not in (-1, 1) for sign in wheel_directions
    ):
        raise ValueError("wheel_directions must contain four values, each either -1 or 1")

    rotation_radius = (wheelbase_m + track_width_m) / 2
    wheel_velocities = (
        linear_x - linear_y - rotation_radius * angular_z,
        linear_x + linear_y + rotation_radius * angular_z,
        linear_x + linear_y - rotation_radius * angular_z,
        linear_x - linear_y + rotation_radius * angular_z,
    )
    raw_speeds = tuple(velocity * steps_per_meter for velocity in wheel_velocities)
    largest_speed = max(abs(speed) for speed in raw_speeds)
    scale = min(1.0, max_speed / largest_speed) if largest_speed else 1.0
    scaled_speeds = tuple(
        round(speed * scale) * direction
        for speed, direction in zip(raw_speeds, wheel_directions, strict=True)
    )
    return scaled_speeds[0], scaled_speeds[1], scaled_speeds[2], scaled_speeds[3]
