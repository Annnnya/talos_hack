from __future__ import annotations

import math

import pytest

from talos_servo.ros.convert import (
    CENTER_STEP,
    joint_ids,
    joint_name,
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.web.config import ServoConfig


def test_center_step_is_zero_rad():
    assert steps_to_rad(CENTER_STEP) == 0.0
    assert rad_to_steps(0.0) == CENTER_STEP


def test_half_turn_each_way():
    assert steps_to_rad(0) == pytest.approx(-math.pi)
    assert rad_to_steps(math.pi) == CENTER_STEP + 2048


@pytest.mark.parametrize("steps", [0, 1, 1000, 2048, 3333, 4095])
def test_position_round_trip(steps):
    assert rad_to_steps(steps_to_rad(steps)) == steps


@pytest.mark.parametrize("speed", [-500, -1, 0, 1, 200, 1000])
def test_speed_round_trip(speed):
    assert rad_speed_to_steps(step_speed_to_rad(speed)) == speed


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_commands_rejected(bad):
    with pytest.raises(ValueError):
        rad_to_steps(bad)
    with pytest.raises(ValueError):
        rad_speed_to_steps(bad)


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("Servo 1", "servo_1"),
        ("left-wheel", "left_wheel"),
        ("  Base  Yaw ", "base_yaw"),
        ("1", "servo_1"),
        ("!!", "servo"),
    ],
)
def test_joint_name(name, expected):
    assert joint_name(name) == expected


def test_joint_ids_maps_names():
    servos = [ServoConfig(id=1, name="Servo 1"), ServoConfig(id=7, name="wrist")]
    assert joint_ids(servos) == {"servo_1": 1, "wrist": 7}


def test_joint_ids_rejects_collisions():
    with pytest.raises(ValueError, match="both map to"):
        joint_ids([ServoConfig(id=1, name="arm 1"), ServoConfig(id=2, name="Arm-1")])
