"""Message rules of the `topic_based_ros2_control/TopicBasedSystem` plugin.

Kept free of ROS imports so it can be tested without rclpy. See
`docs/ros2_control.md` for how the plugin and the hardware bridge node fit
together.

Commands: the plugin sends one `sensor_msgs/JointState` with every joint name,
but it fills `position` and `velocity` per command interface, in joint order.
With mixed joints (the `bend` position joint and four velocity wheels) the
arrays are packed, not aligned with `name`: 5 names, 1 position, 4 velocities.

States: the plugin reads `position` for every name it knows, so position is
always sent. Only position, velocity and effort exist in a JointState, so
nothing else (current, load, temperature) can reach ros2_control this way.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from talos_servo.bus import RAD_PER_STEP, STEPS_PER_TURN
from talos_servo.config import POSITION, SPEED

HALF_TURN_STEPS = STEPS_PER_TURN // 2


def unpack_commands(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    controls: Mapping[str, str],
) -> tuple[dict[str, float], dict[str, float]]:
    """Split one command message into position and velocity goals by joint.

    `controls` maps each joint to its control (`position` or `speed`) from
    `config/robot.yaml`, which must match its command interface in the URDF.
    Arrays as long as `names` are read as aligned (a hand-written message);
    otherwise they are read as packed in joint order (the plugin). A NaN
    position means no goal for that joint, so it never blocks the wheel
    velocities in the same message. Raises ValueError for unknown joints,
    lengths that fit neither layout, infinite positions and non-finite
    velocities.
    """
    unknown = [n for n in names if n not in controls]
    if unknown:
        raise ValueError(
            f"unknown joints {unknown}, expected some of {sorted(controls)}"
        )
    position_joints = [n for n in names if controls[n] == POSITION]
    speed_joints = [n for n in names if controls[n] == SPEED]
    return (
        _pick(names, position_joints, positions, "position", skip_nan=True),
        _pick(names, speed_joints, velocities, "velocity"),
    )


def _pick(
    names: Sequence[str],
    wanted: Sequence[str],
    values: Sequence[float],
    what: str,
    *,
    skip_nan: bool = False,
) -> dict[str, float]:
    if not values:
        return {}
    if len(values) == len(names):
        picked = dict(zip(names, values, strict=True))
        pairs = [(n, picked[n]) for n in wanted]
    elif len(values) == len(wanted):
        pairs = list(zip(wanted, values, strict=True))
    else:
        raise ValueError(
            f"{len(values)} {what} values for {len(names)} joints, of which "
            f"{len(wanted)} take a {what}"
        )
    return {
        n: _finite(v, n, what)
        for n, v in pairs
        if not (skip_nan and math.isnan(v))
    }


def _finite(value: float, joint: str, what: str) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{joint}: {what} must be finite, got {value}")
    return float(value)


class Unwrapper:
    """Turns a wrapping 0..4095 servo position into a continuous angle.

    Wheels spin many turns, so their angle must keep counting instead of
    jumping back at each turn (odometry in the controllers integrates it).
    Readings must come at least once per half turn of the wheel.
    """

    def __init__(self) -> None:
        self._last: int | None = None
        self._steps = 0

    def update(self, position: int) -> float:
        """Feed the newest reading; return the total angle in rad."""
        if self._last is not None:
            delta = (position - self._last + HALF_TURN_STEPS) % (
                STEPS_PER_TURN
            ) - HALF_TURN_STEPS
            self._steps += delta
        self._last = position
        return self._steps * RAD_PER_STEP
