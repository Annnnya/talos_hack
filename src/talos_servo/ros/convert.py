"""Unit and name conversions between servo bus values and ROS conventions.

Kept free of ROS imports so it can be tested without rclpy.
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterable

from talos_servo.bus import MAX_POSITION
from talos_servo.web.config import ServoConfig

STEPS_PER_TURN = MAX_POSITION + 1
# Step 2048 (half a turn) is joint angle 0, so the usable range is about -pi..pi.
CENTER_STEP = STEPS_PER_TURN // 2
RAD_PER_STEP = 2 * math.pi / STEPS_PER_TURN

_INVALID_NAME_CHARS = re.compile(r"[^a-z0-9]+")


def steps_to_rad(steps: int) -> float:
    return (steps - CENTER_STEP) * RAD_PER_STEP


def rad_to_steps(rad: float) -> int:
    """Joint angle to a goal position in steps. Range clamping is left to the controller."""
    _require_finite(rad, "position")
    return round(rad / RAD_PER_STEP) + CENTER_STEP


def step_speed_to_rad(speed: int) -> float:
    return speed * RAD_PER_STEP


def rad_speed_to_steps(rad_per_s: float) -> int:
    _require_finite(rad_per_s, "velocity")
    return round(rad_per_s / RAD_PER_STEP)


class WheelOdometer:
    """Unwrap a 0..4095 wheel encoder into a running angle in radians.

    Assumes the wheel turns less than half a revolution between two reads, so the shortest
    step between readings is the real one. The direction sign makes positive mean forward.
    """

    def __init__(self, direction: int = 1) -> None:
        if direction not in (-1, 1):
            raise ValueError(f"direction must be -1 or 1, got {direction}")
        self.direction = direction
        self._last_raw: int | None = None
        self._steps = 0

    def reset(self) -> None:
        self._last_raw = None
        self._steps = 0

    def update(self, raw_position: int) -> float:
        """Feed one encoder reading and return the unwrapped angle in radians."""
        raw = raw_position % STEPS_PER_TURN
        if self._last_raw is not None:
            delta = (raw - self._last_raw) % STEPS_PER_TURN
            if delta > STEPS_PER_TURN // 2:
                delta -= STEPS_PER_TURN
            self._steps += delta
        self._last_raw = raw
        return self.angle

    @property
    def angle(self) -> float:
        return self._steps * self.direction * RAD_PER_STEP


def joint_name(name: str) -> str:
    """Make a config name usable as a ROS name: 'Servo 1' -> 'servo_1'."""
    cleaned = _INVALID_NAME_CHARS.sub("_", name.lower()).strip("_")
    if not cleaned or cleaned[0].isdigit():
        cleaned = f"servo_{cleaned}".rstrip("_")
    return cleaned


def joint_ids(servos: Iterable[ServoConfig]) -> dict[str, int]:
    """Map ROS joint name to servo ID, refusing names that collide after cleaning."""
    ids: dict[str, int] = {}
    for servo in servos:
        name = joint_name(servo.name)
        if name in ids:
            raise ValueError(
                f"servos {ids[name]} and {servo.id} both map to joint name {name!r}, "
                "rename one in the config"
            )
        ids[name] = servo.id
    return ids


def _require_finite(value: float, what: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number, got {value}")
