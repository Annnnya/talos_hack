"""Servo bridge node against the simulated bus. Skipped where ROS messages are missing."""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy")
# The default pixi env has rclpy on Linux (for the mecanum base) but not these messages.
pytest.importorskip("diagnostic_msgs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("std_msgs")

from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

from talos_servo.ros.convert import CENTER_STEP, step_speed_to_rad  # noqa: E402
from talos_servo.ros.readouts import JointReadoutPublishers  # noqa: E402
from talos_servo.ros.servo_bridge_node import DEFAULT_MOVE_SPEED, ServoBridgeNode  # noqa: E402
from talos_servo.web.config import parse_config  # noqa: E402
from talos_servo.web.controller import POSITION, WHEEL, RobotController  # noqa: E402
from talos_servo.web.sim import make_bus  # noqa: E402

CONFIG = parse_config(
    {
        "max_speed": 500,
        "telemetry_hz": 50,
        "servos": [
            {"id": 1, "name": "Base", "min_pos": 1000, "max_pos": 3000},
            {"id": 2, "name": "left wheel"},
        ],
    }
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def controller():
    c = RobotController(make_bus(None, 1_000_000, mock=True, ids=[1, 2]), CONFIG)
    c.open()
    yield c
    c.close()


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def node(controller, clock):
    n = ServoBridgeNode(controller, clock=clock)
    yield n
    n.destroy_node()


def joint_cmd(names, position=(), velocity=()) -> JointState:
    return JointState(name=list(names), position=list(position), velocity=list(velocity))


def state(controller, servo_id):
    return controller.servos[servo_id]


def set_wheel_mode(node, joint, enabled=True):
    handler = node._wheel_mode_handler(node.joints[joint])
    return handler(SetBool.Request(data=enabled), SetBool.Response())


def test_joint_names_come_from_config(node):
    assert node.joints == {"base": 1, "left_wheel": 2}


def test_joint_state_reports_radians(node):
    msg = node.joint_state_msg()
    assert msg.name == ["base", "left_wheel"]
    # The simulator starts every servo at half a turn, which is 0 rad.
    assert msg.position == pytest.approx([0.0, 0.0], abs=1e-3)
    assert len(msg.velocity) == len(msg.effort) == 2


def test_position_cmd_moves_and_clamps(node, controller):
    node.on_position_cmd(joint_cmd(["base"], position=[math.pi]))
    assert state(controller, 1).target == 3000  # clamped to max_pos


def test_position_cmd_velocity_sets_move_speed(node, controller, monkeypatch):
    calls = []
    monkeypatch.setattr(controller, "move", lambda *a: calls.append(a))
    node.on_position_cmd(joint_cmd(["base"], position=[0.0], velocity=[-step_speed_to_rad(300)]))
    node.on_position_cmd(joint_cmd(["base"], position=[0.0]))
    assert calls == [(1, CENTER_STEP, 300), (1, CENTER_STEP, DEFAULT_MOVE_SPEED)]


def test_unknown_joint_does_not_block_the_others(node, controller):
    node.on_position_cmd(joint_cmd(["nope", "base"], position=[0.0, 0.1]))
    assert state(controller, 1).target is not None


def test_mismatched_lengths_are_ignored(node, controller):
    node.on_position_cmd(joint_cmd(["base"], position=[]))
    node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[]))
    assert state(controller, 1).target is None


def test_velocity_cmd_needs_wheel_mode(node, controller):
    node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[1.0]))
    assert state(controller, 2).mode == POSITION
    assert state(controller, 2).target_speed == 0


def test_velocity_cmd_spins_and_clamps(node, controller):
    assert set_wheel_mode(node, "left_wheel").success
    node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[100.0]))
    assert state(controller, 2).mode == WHEEL
    assert state(controller, 2).target_speed == 500  # clamped to max_speed


def test_watchdog_stops_stale_wheels(node, controller, clock):
    set_wheel_mode(node, "left_wheel")
    node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[1.0]))

    clock.now += node.command_timeout / 2
    node.enforce_command_timeout()
    assert state(controller, 2).target_speed != 0

    clock.now += node.command_timeout
    node.enforce_command_timeout()
    assert state(controller, 2).target_speed == 0


def test_fresh_commands_keep_wheels_running(node, controller, clock):
    set_wheel_mode(node, "left_wheel")
    for _ in range(5):
        node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[1.0]))
        clock.now += node.command_timeout * 0.8
        node.enforce_command_timeout()
    assert state(controller, 2).target_speed != 0


def test_stop_all_service(node, controller):
    set_wheel_mode(node, "left_wheel")
    node.on_velocity_cmd(joint_cmd(["left_wheel"], velocity=[1.0]))
    response = node.on_stop_all(Trigger.Request(), Trigger.Response())
    assert response.success
    assert state(controller, 2).target_speed == 0


def test_torque_service(node, controller):
    handler = node._torque_handler(1)
    assert handler(SetBool.Request(data=False), SetBool.Response()).success
    assert state(controller, 1).torque is False


def test_status_reports_each_servo(node):
    msg = node.status_msg()
    assert [s.hardware_id for s in msg.status] == ["feetech id 1", "feetech id 2"]
    assert all(s.level == DiagnosticStatus.OK for s in msg.status)


def test_readout_topics_live_under_the_node(node):
    fields = ("current", "load", "moving", "temperature", "voltage")
    expected = sorted(f"/servo_bridge/{j}/{f}" for j in ("base", "left_wheel") for f in fields)
    assert node.readouts.topic_names == expected


def test_readout_messages_carry_values():
    stamp = rclpy.clock.Clock().now().to_msg()
    msgs = JointReadoutPublishers.messages(
        "base", stamp, temperature=31, voltage=12.1, current=-0.5, load=-40, moving=True
    )
    assert msgs["temperature"].temperature == 31.0
    assert msgs["temperature"].header.frame_id == "base"
    assert msgs["voltage"].data == pytest.approx(12.1)
    assert msgs["current"].data == pytest.approx(-0.5)
    assert msgs["load"].data == -40.0
    assert msgs["moving"].data is True


def test_readouts_skip_offline_servos(node, controller, monkeypatch):
    published = []
    monkeypatch.setattr(node.readouts, "publish", lambda joint, *_a, **_k: published.append(joint))
    controller.servos[2].online = False
    node.publish_readouts()
    assert published == ["base"]
