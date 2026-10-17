from __future__ import annotations

import math

import pytest

from talos_servo.ros.convert import (
    CENTER_STEP,
    RAD_PER_STEP,
    STEPS_PER_TURN,
    command_frame,
    parse_command_frame,
    rad_speed_to_steps,
    rad_to_steps,
    readout_topic,
    step_speed_to_rad,
    steps_to_rad,
)


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
    ("prefix", "expected"),
    [
        ("/base", "/base/front_left_wheel/voltage"),
        ("/base/", "/base/front_left_wheel/voltage"),
        ("~", "~/front_left_wheel/voltage"),
    ],
)
def test_readout_topic(prefix, expected):
    assert readout_topic(prefix, "front_left_wheel", "voltage") == expected


@pytest.mark.parametrize(
    ("source", "client"), [("web", "3fa2c1"), ("cli", None), ("ros", None)]
)
def test_command_frame_round_trip(source, client):
    assert parse_command_frame(command_frame(source, client)) == (
        source,
        client,
    )


@pytest.mark.parametrize(
    "frame_id", ["", "base_link", "robot", "robot:x", "WEB:1"]
)
def test_unknown_or_reserved_frames_count_as_ros(frame_id):
    assert parse_command_frame(frame_id) == ("ros", None)
