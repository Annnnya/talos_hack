from __future__ import annotations

import math

import pytest

from talos_servo.ros.convert import (
    CENTER_STEP,
    RAD_PER_STEP,
    STEPS_PER_TURN,
    WheelOdometer,
    command_frame,
    joint_ids,
    joint_name,
    parse_command_frame,
    rad_speed_to_steps,
    rad_to_steps,
    readout_topic,
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


def _steps(steps: int) -> float:
    return steps * RAD_PER_STEP


def test_odometer_first_reading_is_zero():
    odo = WheelOdometer()
    assert odo.update(1234) == 0


def test_odometer_forward_across_wrap():
    odo = WheelOdometer()
    odo.update(4000)
    assert odo.update(100) == pytest.approx(_steps(196))


def test_odometer_backward_across_wrap():
    odo = WheelOdometer()
    odo.update(100)
    assert odo.update(4000) == pytest.approx(_steps(-196))


def test_odometer_accumulates_over_turns():
    odo = WheelOdometer()
    for raw in [0, 1000, 2000, 3000, 0, 1000]:
        odo.update(raw)
    assert odo.angle == pytest.approx(_steps(STEPS_PER_TURN + 1000))


def test_odometer_direction_sign_flips_angle():
    odo = WheelOdometer(direction=-1)
    odo.update(0)
    assert odo.update(500) == pytest.approx(_steps(-500))


def test_odometer_reset():
    odo = WheelOdometer()
    odo.update(0)
    odo.update(500)
    odo.reset()
    assert odo.angle == 0
    assert odo.update(3000) == 0


def test_odometer_rejects_bad_direction():
    with pytest.raises(ValueError):
        WheelOdometer(direction=0)


@pytest.mark.parametrize(
    ("prefix", "expected"),
    [
        ("/base", "/base/front_left_wheel/voltage"),
        ("/base/", "/base/front_left_wheel/voltage"),
        ("~", "~/front_left_wheel/voltage"),
    ],
)
def test_readout_topic(prefix, expected):
    assert readout_topic(prefix, "front_left_wheel", "voltage") == expected


@pytest.mark.parametrize(("source", "client"), [("web", "3fa2c1"), ("cli", None), ("ros", None)])
def test_command_frame_round_trip(source, client):
    assert parse_command_frame(command_frame(source, client)) == (source, client)


@pytest.mark.parametrize("frame_id", ["", "base_link", "bridge", "bridge:x", "WEB:1"])
def test_unknown_or_reserved_frames_count_as_ros(frame_id):
    assert parse_command_frame(frame_id) == ("ros", None)
