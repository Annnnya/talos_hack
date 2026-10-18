"""Hardware bridge node on a Robot over the fake SDK.

Skipped where ROS is missing.
"""

from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from conftest import SMALL, FakeClock, config  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.robot import Robot  # noqa: E402
from talos_servo.ros.convert import (  # noqa: E402
    CENTER_STEP,
    RAD_PER_STEP,
    step_speed_to_rad,
)
from talos_servo.ros.hardware_bridge_node import HardwareBridgeNode  # noqa: E402

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


@pytest.fixture
def position_node(robot):
    n = HardwareBridgeNode(
        robot,
        parameter_overrides=[Parameter("accept_position_goals", value=True)],
    )
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


def test_position_goals_are_ignored_by_default(node, robot):
    # The plugin sends 0.0 for a position joint no controller claims.
    node.on_commands(commands(["bend"], position=[0.0]))
    assert robot.joints["bend"].target is None


def test_packed_commands_spin_the_wheel_and_move_bend(position_node, robot):
    position_node.on_commands(
        commands(
            ["bend", "wheel"],
            position=[0.0],
            velocity=[step_speed_to_rad(200)],
        )
    )
    assert robot.joints["wheel"].target_speed == 200
    assert robot.joints["bend"].target == CENTER_STEP


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


def test_same_bend_goal_is_written_once(position_node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    for _ in range(3):
        position_node.on_commands(commands(["bend"], position=[0.1]))
    position_node.on_commands(commands(["bend"], position=[0.2]))
    assert [m[1] for m in moves] == [
        CENTER_STEP + round(0.1 / RAD_PER_STEP),
        CENTER_STEP + round(0.2 / RAD_PER_STEP),
    ]


def test_failed_move_is_retried_with_the_same_goal(
    position_node, robot, monkeypatch
):
    moves = []

    def move(*a, **_k):
        moves.append(a)
        if len(moves) == 1:
            raise ServoError("no answer")

    monkeypatch.setattr(robot, "move", move)
    for _ in range(3):
        position_node.on_commands(commands(["bend"], position=[0.1]))
    assert len(moves) == 2


def test_stop_all_resends_the_same_goal(position_node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    position_node.on_commands(commands(["bend"], position=[0.1]))
    position_node.on_stop_all(Trigger.Request(), Trigger.Response())
    position_node.on_commands(commands(["bend"], position=[0.1]))
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
