"""Convert raw wheel servo readouts into ROS-friendly units (rad, rad/s)."""

from __future__ import annotations

import math

from talos_servo.bus import MAX_POSITION

STEPS_PER_REV = MAX_POSITION + 1
RAD_PER_STEP = 2 * math.pi / STEPS_PER_REV

# Same FL, FR, RL, RR order as mecanum_wheel_speeds.
WHEEL_JOINT_NAMES = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)


def steps_to_rad(steps: float) -> float:
    return steps * RAD_PER_STEP


def speed_to_rad_s(steps_per_s: float) -> float:
    return steps_per_s * RAD_PER_STEP


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
        raw = raw_position % STEPS_PER_REV
        if self._last_raw is not None:
            delta = (raw - self._last_raw) % STEPS_PER_REV
            if delta > STEPS_PER_REV // 2:
                delta -= STEPS_PER_REV
            self._steps += delta
        self._last_raw = raw
        return steps_to_rad(self._steps * self.direction)

    @property
    def angle(self) -> float:
        return steps_to_rad(self._steps * self.direction)
