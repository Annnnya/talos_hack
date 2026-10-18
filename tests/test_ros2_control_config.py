"""Sanity checks for config/ros2_control.yaml against config/robot.yaml.

The mecanum controller copies the base geometry and limits by hand. A value
that drifts still loads, but then the robot drives at the wrong speed, turns
on the wrong radius or no longer matches the old drive (`mecanum.py`).
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

from talos_servo.bus import MAX_SPEED  # noqa: E402
from talos_servo.config import load_config  # noqa: E402
from talos_servo.mecanum import wheel_speeds  # noqa: E402

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
CONTROLLER = "mecanum_drive_controller"
# Command joint parameter of the controller for each base.wheels slot.
WHEEL_PARAMS = (
    "front_left_wheel_command_joint_name",
    "front_right_wheel_command_joint_name",
    "rear_left_wheel_command_joint_name",
    "rear_right_wheel_command_joint_name",
)


@pytest.fixture(scope="module")
def controllers() -> dict:
    return yaml.safe_load((CONFIG_DIR / "ros2_control.yaml").read_text())


@pytest.fixture(scope="module")
def params(controllers) -> dict:
    return controllers[CONTROLLER]["ros__parameters"]


@pytest.fixture(scope="module")
def robot():
    return load_config(str(CONFIG_DIR / "robot.yaml"))


def controller_wheels(params: dict, vx: float, vy: float, wz: float):
    """FL, FR, RL, RR wheel speeds in rad/s, as mecanum_drive_controller
    computes them (ros2_controllers 4.x, no base frame offset)."""
    kinematics = params["kinematics"]
    r = kinematics["wheels_radius"]
    k = kinematics["sum_of_robot_center_projection_on_X_Y_axis"]
    return (
        (vx - vy - k * wz) / r,
        (vx + vy + k * wz) / r,
        (vx + vy - k * wz) / r,
        (vx - vy + k * wz) / r,
    )


def test_controller_is_registered(controllers):
    cm = controllers["controller_manager"]["ros__parameters"]
    assert cm[CONTROLLER]["type"] == (
        "mecanum_drive_controller/MecanumDriveController"
    )


def test_wheels_in_base_order(params, robot):
    assert [params[p] for p in WHEEL_PARAMS] == list(robot.base.wheels)


def test_geometry_matches_the_base(params, robot):
    base = robot.base
    kinematics = params["kinematics"]
    assert kinematics["wheels_radius"] == pytest.approx(base.wheel_radius_m)
    # Flat wheelbase: driving with the hinge folded is out of scope.
    assert kinematics[
        "sum_of_robot_center_projection_on_X_Y_axis"
    ] == pytest.approx((base.wheelbase_m + base.track_width_m) / 2)


def test_speed_limits_match_the_base(params, robot):
    base = robot.base
    for axis, limit in (
        (params["linear"]["x"], base.max_linear_speed),
        (params["linear"]["y"], base.max_linear_speed),
        (params["angular"]["z"], base.max_angular_speed),
    ):
        assert axis["max_velocity"] == pytest.approx(limit)
        assert axis["min_velocity"] == pytest.approx(-limit)


def test_stops_like_the_bridge_watchdog(params, robot):
    # 0 would mean "reset after every update", not "never".
    assert params["reference_timeout"] == pytest.approx(robot.command_timeout_s)


def test_odometry_without_tf(params):
    # The EKF alone publishes odom -> base_link.
    assert params["enable_odom_tf"] is False
    assert params["odom_frame_id"] == "odom"
    assert params["base_frame_id"] == "base_link"


def test_every_mode_controller_exists(controllers, robot):
    cm = controllers["controller_manager"]["ros__parameters"]
    assert robot.modes is not None
    missing = robot.modes.managed - set(cm)
    assert not missing


@pytest.mark.parametrize(
    ("twist", "signs"),
    [
        ((0.05, 0.0, 0.0), (1, 1, 1, 1)),  # forward
        ((-0.05, 0.0, 0.0), (-1, -1, -1, -1)),  # backward
        ((0.0, 0.05, 0.0), (-1, 1, 1, -1)),  # sideways left
        ((0.0, -0.05, 0.0), (1, -1, -1, 1)),  # sideways right
        ((0.0, 0.0, 0.3), (-1, 1, -1, 1)),  # rotate counterclockwise
        ((0.0, 0.0, -0.3), (1, -1, 1, -1)),  # rotate clockwise
    ],
)
def test_wheel_signs(params, twist, signs):
    wheels = controller_wheels(params, *twist)
    assert tuple(int(math.copysign(1, w)) for w in wheels) == signs


@pytest.mark.parametrize(
    "twist",
    [
        (0.05, 0.0, 0.0),
        (0.0, 0.05, 0.0),
        (0.0, 0.0, 0.3),
        (0.03, -0.02, 0.2),
    ],
)
def test_same_wheel_speeds_as_the_old_drive(params, robot, twist):
    base = robot.base
    # Old drive in steps/s, without its per-wheel cap.
    old = wheel_speeds(
        *twist,
        wheelbase_m=base.wheelbase_m,
        track_width_m=base.track_width_m,
        steps_per_meter=base.steps_per_meter,
        max_speed=MAX_SPEED,
    )
    r = params["kinematics"]["wheels_radius"]
    new = [
        w * r * base.steps_per_meter for w in controller_wheels(params, *twist)
    ]
    assert new == pytest.approx(old, abs=1.0)
