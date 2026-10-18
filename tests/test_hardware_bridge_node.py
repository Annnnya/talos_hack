"""Hardware bridge node on a Robot over the fake SDK.

Skipped where ROS is missing.
"""

from __future__ import annotations

import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from conftest import BASE, SMALL, FakeClock, config  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.robot import Robot  # noqa: E402
from talos_servo.ros.convert import (  # noqa: E402
    CENTER_STEP,
    RAD_PER_STEP,
    step_speed_to_rad,
)
from talos_servo.ros.hardware_bridge_node import (  # noqa: E402
    HINGE_INITIAL_RAD,
    HardwareBridgeNode,
)

INVERTED = {
    **SMALL,
    "joints": {
        **SMALL["joints"],
        "wheel": {"id": 2, "control": "speed", "inverted": True},
    },
}


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def robot(sdk, clock):
    r = Robot(sdk.bus(), config(SMALL), clock=clock)
    r.open()
    yield r
    r.close()


@pytest.fixture
def node(robot):
    n = HardwareBridgeNode(robot)
    yield n
    n.destroy_node()


def commands(names, position=(), velocity=()) -> JointState:
    return JointState(
        name=list(names), position=list(position), velocity=list(velocity)
    )


def test_states_carry_position_velocity_and_current(node, robot):
    robot.joints["wheel"].current = 0.42
    robot.joints["wheel"].speed = 100
    msg = node.states_msg()
    assert msg.name == ["bend", "wheel"]
    # The plugin reads position for every joint, so it is always there.
    assert len(msg.position) == len(msg.velocity) == len(msg.effort) == 2
    assert msg.velocity[1] == pytest.approx(step_speed_to_rad(100))
    assert msg.effort[1] == pytest.approx(0.42)


def test_wheel_position_keeps_counting_past_a_turn(node, robot):
    robot.joints["wheel"].position = 4000
    node.states_msg()
    robot.joints["wheel"].position = 100
    assert node.states_msg().position[1] == pytest.approx(196 * RAD_PER_STEP)


def test_bend_position_is_an_angle(node, robot):
    robot.joints["bend"].position = CENTER_STEP + 100
    assert node.states_msg().position[0] == pytest.approx(100 * RAD_PER_STEP)


def test_initial_value_is_ignored_until_a_different_goal(node, robot):
    # The plugin sends 0.0 for a position joint without an initial_value
    # until a controller writes a goal.
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.0]))
    assert robot.joints["bend"].target is None
    node.on_commands(commands(["bend"], position=[0.1]))
    node.on_commands(commands(["bend"], position=[0.0]))
    assert robot.joints["bend"].target == CENTER_STEP


def test_packed_commands_spin_the_wheel_and_move_bend(node, robot):
    node.on_commands(
        commands(
            ["bend", "wheel"],
            position=[0.1],
            velocity=[step_speed_to_rad(200)],
        )
    )
    assert robot.joints["wheel"].target_speed == 200
    assert robot.joints["bend"].target == CENTER_STEP + round(
        0.1 / RAD_PER_STEP
    )


def test_nan_bend_goal_still_spins_the_wheel(node, robot):
    node.on_commands(
        commands(
            ["bend", "wheel"],
            position=[float("nan")],
            velocity=[step_speed_to_rad(200)],
        )
    )
    assert robot.joints["wheel"].target_speed == 200
    assert robot.joints["bend"].target is None


def test_inverted_wheel_turns_the_servo_the_other_way(sdk, clock):
    robot = Robot(sdk.bus(), config(INVERTED), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        node.on_commands(commands(["wheel"], velocity=[step_speed_to_rad(200)]))
        node.tick()
        speed = sdk.handler.sync_writes()[-1][2]
        assert speed[-2] | speed[-1] << 8 == 0x8000 | 200
    finally:
        node.destroy_node()
        robot.close()


def test_same_bend_goal_is_written_once(node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.1]))
    node.on_commands(commands(["bend"], position=[0.2]))
    assert [m[1] for m in moves] == [
        CENTER_STEP + round(0.1 / RAD_PER_STEP),
        CENTER_STEP + round(0.2 / RAD_PER_STEP),
    ]


def test_failed_move_is_retried_with_the_same_goal(node, robot, monkeypatch):
    moves = []

    def move(*a, **_k):
        moves.append(a)
        if len(moves) == 1:
            raise ServoError("no answer")

    monkeypatch.setattr(robot, "move", move)
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.1]))
    assert len(moves) == 2


def test_stop_all_resends_the_same_goal(node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    node.on_commands(commands(["bend"], position=[0.1]))
    node.on_stop_all(Trigger.Request(), Trigger.Response())
    node.on_commands(commands(["bend"], position=[0.1]))
    assert len(moves) == 2


def test_wheel_stops_when_commands_stop(node, robot, clock):
    node.on_commands(commands(["wheel"], velocity=[1.0]))
    assert robot.joints["wheel"].target_speed != 0
    clock.now += robot.config.command_timeout_s + 0.1
    node.tick()
    assert robot.joints["wheel"].target_speed == 0


def test_bad_commands_are_only_logged(node, robot):
    node.on_commands(commands(["nope"], velocity=[1.0]))
    node.on_commands(commands(["wheel"], velocity=[float("nan")]))
    node.on_commands(commands(["bend", "wheel"], velocity=[1.0, 2.0, 3.0]))
    assert robot.joints["wheel"].target_speed == 0
    assert robot.joints["bend"].target is None


def test_stop_all_service(node, robot):
    node.on_commands(commands(["wheel"], velocity=[1.0]))
    response = node.on_stop_all(Trigger.Request(), Trigger.Response())
    assert response.success
    assert robot.joints["wheel"].target_speed == 0


def test_readout_topics_live_under_the_node(node):
    assert "/hardware_bridge/wheel/current" in node.readouts.topic_names


# --- hinge (#63, #85) ------------------------------------------------------
MODEL = {
    "hinge_joint": "bend",
    "leg_gear_ratio": 0.5,
    "bend_min_deg": 90,
    "bend_max_deg": 180,
}
# The fake servo reads 2048, so an uncalibrated hinge is flat there.
START_STEPS = 2048
CALIBRATED_FLAT = 3000
# 90 deg of fold: the opening changes 1:1 with the servo.
QUARTER_TURN = 1024


def hinge_config(**model_changes) -> dict:
    data = copy.deepcopy(BASE)
    data["joints"]["bend"]["enabled"] = True
    return {**data, "model": {**MODEL, **model_changes}}


@pytest.fixture
def hinge_robot(sdk, clock):
    r = Robot(sdk.bus(), config(hinge_config()), clock=clock)
    r.open()
    yield r
    r.close()


@pytest.fixture
def hinge_node(hinge_robot):
    n = HardwareBridgeNode(hinge_robot)
    yield n
    n.destroy_node()


def bend_state(node) -> tuple[float, float]:
    msg = node.states_msg()
    i = msg.name.index("bend")
    return msg.position[i], msg.velocity[i]


def test_uncalibrated_hinge_starts_flat(hinge_node, hinge_robot):
    assert bend_state(hinge_node)[0] == pytest.approx(math.pi)
    hinge_robot.joints["bend"].position = START_STEPS - QUARTER_TURN
    hinge_robot.joints["bend"].speed = -100
    alpha, rate = bend_state(hinge_node)
    assert alpha == pytest.approx(math.pi / 2)
    assert rate == pytest.approx(-100 * RAD_PER_STEP)


def test_calibrated_flat_comes_from_the_config(sdk, clock):
    data = hinge_config(bend_flat_steps=CALIBRATED_FLAT)
    robot = Robot(sdk.bus(), config(data), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        assert bend_state(node)[0] == pytest.approx(
            math.pi - (CALIBRATED_FLAT - START_STEPS) * RAD_PER_STEP
        )
        node.on_commands(commands(["bend"], position=[math.pi / 2]))
        assert robot.joints["bend"].target == CALIBRATED_FLAT - QUARTER_TURN
    finally:
        node.destroy_node()
        robot.close()


def test_hinge_ignores_flat_until_a_different_goal(hinge_node, hinge_robot):
    # The URDF initial_value: what the plugin sends before any controller.
    for _ in range(3):
        hinge_node.on_commands(commands(["bend"], position=[math.pi]))
    assert hinge_robot.joints["bend"].target is None
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == START_STEPS - QUARTER_TURN
    hinge_node.on_commands(commands(["bend"], position=[math.pi]))
    assert hinge_robot.joints["bend"].target == START_STEPS


def test_hinge_goals_clamp_to_the_model_limits(hinge_node, hinge_robot):
    hinge_node.on_commands(commands(["bend"], position=[math.radians(30)]))
    assert hinge_robot.joints["bend"].target == START_STEPS - QUARTER_TURN


def test_offline_hinge_drops_goals_and_reports_flat(sdk, clock):
    sdk.present = {1, 2, 3, 4}  # the bend servo (5) does not answer
    robot = Robot(sdk.bus(), config(hinge_config()), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        assert bend_state(node)[0] == pytest.approx(math.pi)
        node.on_commands(commands(["bend"], position=[2.0]))
        assert robot.joints["bend"].target is None
    finally:
        node.destroy_node()
        robot.close()


def test_hinge_initial_value_matches_the_urdf():
    xacro = pytest.importorskip("xacro")
    root = Path(__file__).resolve().parent.parent
    urdf = ET.fromstring(
        xacro.process_file(
            str(root / "description" / "talos.urdf.xacro")
        ).toxml()
    )
    state = urdf.find(
        "ros2_control/joint[@name='bend']/state_interface[@name='position']"
        "/param[@name='initial_value']"
    )
    assert float(state.text) == HINGE_INITIAL_RAD
