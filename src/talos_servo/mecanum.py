"""Mecanum drive kinematics for four independently driven wheels, in FL, FR, RL,
RR order.

Wheel speeds here always mean "positive drives the robot forward". Motors
mounted the other way round are handled by `inverted` in the robot config, not
here.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from talos_servo.bus import MAX_ACCELERATION, MAX_SPEED


def wheel_speeds(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    *,
    wheelbase_m: float,
    track_width_m: float,
    steps_per_meter: float,
    max_speed: int,
) -> tuple[int, int, int, int]:
    """Map body velocity (m/s, m/s, rad/s) to FL, FR, RL, RR wheel speeds in
    steps/s.

    If any wheel would exceed `max_speed`, all four are scaled down together so
    the robot still moves in the commanded direction.
    """
    values = (
        linear_x,
        linear_y,
        angular_z,
        wheelbase_m,
        track_width_m,
        steps_per_meter,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocities and kinematic parameters must be finite")
    if wheelbase_m <= 0 or track_width_m <= 0 or steps_per_meter <= 0:
        raise ValueError(
            "wheelbase, track width, and steps per meter must be positive"
        )
    if not 1 <= max_speed <= MAX_SPEED:
        raise ValueError(f"max_speed must be within 1..{MAX_SPEED}")

    rotation_radius = (wheelbase_m + track_width_m) / 2
    wheel_velocities = (
        linear_x - linear_y - rotation_radius * angular_z,
        linear_x + linear_y + rotation_radius * angular_z,
        linear_x + linear_y - rotation_radius * angular_z,
        linear_x - linear_y + rotation_radius * angular_z,
    )
    raw_speeds = tuple(
        velocity * steps_per_meter for velocity in wheel_velocities
    )
    largest_speed = max(abs(speed) for speed in raw_speeds)
    scale = min(1.0, max_speed / largest_speed) if largest_speed else 1.0
    fl, fr, rl, rr = (round(speed * scale) for speed in raw_speeds)
    return fl, fr, rl, rr


def wheel_accelerations(
    previous: Sequence[int], target: Sequence[int], base_acceleration: int
) -> tuple[int, ...]:
    """Scale each wheel's acceleration so all wheels finish their speed change
    together.

    The wheel with the largest speed change uses `base_acceleration`; the others
    ramp proportionally slower. Never returns 0, which the servo treats as "no
    ramp".
    """
    if len(previous) != len(target):
        raise ValueError(
            "previous and target must have the same number of wheels"
        )
    if not 1 <= base_acceleration <= MAX_ACCELERATION:
        raise ValueError(
            f"base_acceleration must be within 1..{MAX_ACCELERATION}"
        )
    deltas = [abs(new - old) for old, new in zip(previous, target, strict=True)]
    largest = max(deltas, default=0)
    if not largest:
        return tuple(base_acceleration for _ in deltas)
    return tuple(
        max(1, round(base_acceleration * delta / largest)) for delta in deltas
    )


def body_velocity(
    fl: float,
    fr: float,
    rl: float,
    rr: float,
    *,
    wheelbase_m: float,
    track_width_m: float,
    steps_per_meter: float,
) -> tuple[float, float, float]:
    """Inverse of `wheel_speeds`: wheel speeds in steps/s to (vx, vy, wz).

    Gives m/s, m/s, rad/s. Four wheels over-determine three velocities, so this
    is the least-squares fit, exact whenever the wheels do not slip.
    """
    rotation_radius = (wheelbase_m + track_width_m) / 2
    vx = (fl + fr + rl + rr) / 4
    vy = (-fl + fr + rl - rr) / 4
    wz = (-fl + fr - rl + rr) / (4 * rotation_radius)
    return (
        vx / steps_per_meter,
        vy / steps_per_meter,
        wz / steps_per_meter,
    )


def integrate_pose(
    pose: tuple[float, float, float],
    velocity: tuple[float, float, float],
    dt: float,
) -> tuple[float, float, float]:
    """Advance (x, y, yaw) by a body-frame velocity held for `dt` seconds."""
    x, y, yaw = pose
    vx, vy, wz = velocity
    mid_yaw = yaw + wz * dt / 2
    cos, sin = math.cos(mid_yaw), math.sin(mid_yaw)
    return (
        x + (vx * cos - vy * sin) * dt,
        y + (vx * sin + vy * cos) * dt,
        yaw + wz * dt,
    )
