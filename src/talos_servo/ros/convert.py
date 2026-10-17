"""Unit and name conversions between servo bus values and ROS conventions.

Kept free of ROS imports so it can be tested without rclpy.
"""

from __future__ import annotations

import math

from talos_servo.bus import MAX_POSITION
from talos_servo.robot import SOURCE_ROBOT, SOURCE_ROS, SOURCES

STEPS_PER_TURN = MAX_POSITION + 1
# Step 2048 (half a turn) is joint angle 0, so the usable range is about
# -pi..pi.
CENTER_STEP = STEPS_PER_TURN // 2
RAD_PER_STEP = 2 * math.pi / STEPS_PER_TURN

_FRAME_SEPARATOR = ":"


def steps_to_rad(steps: int) -> float:
    return (steps - CENTER_STEP) * RAD_PER_STEP


def rad_to_steps(rad: float) -> int:
    """Joint angle to a goal position in steps.

    Range clamping is left to the robot.
    """
    _require_finite(rad, "position")
    return round(rad / RAD_PER_STEP) + CENTER_STEP


def step_speed_to_rad(speed: int) -> float:
    return speed * RAD_PER_STEP


def rad_speed_to_steps(rad_per_s: float) -> int:
    _require_finite(rad_per_s, "velocity")
    return round(rad_per_s / RAD_PER_STEP)


def readout_topic(prefix: str, joint: str, field: str) -> str:
    """Topic for one numeric readout.

    ('/robot', 'wheel', 'voltage') -> '/robot/wheel/voltage'. A prefix of '~'
    keeps it under the node name.
    """
    return f"{prefix.rstrip('/')}/{joint}/{field}"


def command_frame(source: str, client: str | None = None) -> str:
    """Who sent a command, for `header.frame_id` of position_cmd / velocity_cmd:
    'web:3fa2c1'."""
    return f"{source}{_FRAME_SEPARATOR}{client}" if client else source


def parse_command_frame(frame_id: str) -> tuple[str, str | None]:
    """Inverse of command_frame. Anything unknown (most ROS nodes send '')
    counts as 'ros'.

    'robot' is reserved for the robot's own stops, so a sender cannot claim it.
    """
    source, _, client = frame_id.partition(_FRAME_SEPARATOR)
    if source not in SOURCES or source == SOURCE_ROBOT:
        return SOURCE_ROS, None
    return source, client or None


def _require_finite(value: float, what: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number, got {value}")
