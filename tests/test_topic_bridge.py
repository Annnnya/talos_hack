"""Message rules of the topic based ros2_control plugin, no ROS needed."""

from __future__ import annotations

import math

import pytest

from talos_servo.ros.convert import RAD_PER_STEP
from talos_servo.ros.topic_bridge import Unwrapper, unpack_commands

CONTROLS = {
    "bend": "position",
    "front_left_wheel": "speed",
    "front_right_wheel": "speed",
}
ALL = ["bend", "front_left_wheel", "front_right_wheel"]


def test_packed_mixed_commands_follow_joint_order():
    # What the plugin sends: one position for bend, then the two velocities.
    positions, velocities = unpack_commands(ALL, [0.5], [1.0, -2.0], CONTROLS)
    assert positions == {"bend": 0.5}
    assert velocities == {"front_left_wheel": 1.0, "front_right_wheel": -2.0}


def test_aligned_commands_pick_by_name():
    positions, velocities = unpack_commands(
        ALL, [0.5, 9.0, 9.0], [9.0, 1.0, -2.0], CONTROLS
    )
    assert positions == {"bend": 0.5}
    assert velocities == {"front_left_wheel": 1.0, "front_right_wheel": -2.0}


def test_velocity_only_commands():
    names = ["front_left_wheel"]
    assert unpack_commands(names, [], [0.3], CONTROLS) == (
        {},
        {"front_left_wheel": 0.3},
    )


def test_unknown_joint_is_rejected():
    with pytest.raises(ValueError, match="unknown joints"):
        unpack_commands(["nope"], [], [1.0], CONTROLS)


def test_lengths_that_fit_no_layout_are_rejected():
    all_wheels = {**CONTROLS, "bend": "speed"}
    with pytest.raises(ValueError, match="2 velocity values for 3 joints"):
        unpack_commands(ALL, [], [1.0, 2.0], all_wheels)


@pytest.mark.parametrize("bad", [math.nan, math.inf])
def test_non_finite_commands_are_rejected(bad):
    with pytest.raises(ValueError, match="must be finite"):
        unpack_commands(["front_left_wheel"], [], [bad], CONTROLS)


def test_nan_position_is_no_goal_and_keeps_the_velocities():
    positions, velocities = unpack_commands(
        ALL, [math.nan], [1.0, -2.0], CONTROLS
    )
    assert positions == {}
    assert velocities == {"front_left_wheel": 1.0, "front_right_wheel": -2.0}


@pytest.mark.parametrize("bad", [math.inf, -math.inf])
def test_infinite_position_is_rejected(bad):
    with pytest.raises(ValueError, match="position must be finite"):
        unpack_commands(ALL, [bad], [1.0, -2.0], CONTROLS)


def test_unwrapper_starts_at_zero():
    assert Unwrapper().update(3000) == 0.0


def test_unwrapper_counts_forward_past_a_full_turn():
    u = Unwrapper()
    u.update(4000)
    assert u.update(100) == pytest.approx(196 * RAD_PER_STEP)


def test_unwrapper_counts_backward_past_zero():
    u = Unwrapper()
    u.update(100)
    assert u.update(4000) == pytest.approx(-196 * RAD_PER_STEP)


def test_unwrapper_adds_up_many_turns():
    u = Unwrapper()
    angle = 0.0
    for step in range(0, 3 * 4096 + 1, 1000):
        angle = u.update(step % 4096)
    assert angle == pytest.approx(12000 * RAD_PER_STEP)
