"""Robot node on a Robot over the fake SDK. Skipped where ROS is missing."""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("diagnostic_msgs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("nav_msgs")

from conftest import BASE, SMALL, FakeClock, config  # noqa: E402
from diagnostic_msgs.msg import DiagnosticStatus  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402

from talos_servo.defaults import DEFAULT_MOVE_SPEED  # noqa: E402
from talos_servo.robot import Robot  # noqa: E402
from talos_servo.ros.convert import CENTER_STEP, step_speed_to_rad  # noqa: E402
from talos_servo.ros.readouts import JointReadoutPublishers  # noqa: E402
from talos_servo.ros.robot_node import RobotNode  # noqa: E402


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
    n = RobotNode(robot)
    yield n
    n.destroy_node()


@pytest.fixture
def base_node(sdk, clock):
    r = Robot(sdk.bus(), config(BASE), clock=clock)
    r.open()
    n = RobotNode(r)
    yield n
    n.destroy_node()
    r.close()


def joint_cmd(names, position=(), velocity=()) -> JointState:
    return JointState(
        name=list(names), position=list(position), velocity=list(velocity)
    )


def test_joint_state_reports_radians(node):
    msg = node.joint_state_msg()
    assert msg.name == ["bend", "wheel"]
    # The fake servos sit at half a turn, which is 0 rad.
    assert msg.position == pytest.approx([0.0, 0.0], abs=1e-3)
    assert len(msg.velocity) == len(msg.effort) == 2


def test_position_cmd_moves_and_clamps(node, robot):
    node.on_position_cmd(joint_cmd(["bend"], position=[math.pi]))
    assert robot.joints["bend"].target == 3000  # clamped to max_pos


def test_position_cmd_velocity_sets_move_speed(node, robot, monkeypatch):
    calls = []
    monkeypatch.setattr(robot, "move", lambda *a, **_kw: calls.append(a))
    node.on_position_cmd(
        joint_cmd(["bend"], position=[0.0], velocity=[-step_speed_to_rad(300)])
    )
    node.on_position_cmd(joint_cmd(["bend"], position=[0.0]))
    assert calls == [
        ("bend", CENTER_STEP, 300),
        ("bend", CENTER_STEP, DEFAULT_MOVE_SPEED),
    ]


def test_unknown_joint_does_not_block_the_others(node, robot):
    node.on_position_cmd(joint_cmd(["nope", "bend"], position=[0.0, 0.1]))
    assert robot.joints["bend"].target is not None


def test_mismatched_lengths_are_ignored(node, robot):
    node.on_position_cmd(joint_cmd(["bend"], position=[]))
    node.on_velocity_cmd(joint_cmd(["wheel"], velocity=[]))
    assert robot.joints["bend"].target is None


def test_velocity_cmd_on_a_position_joint_is_ignored(node, robot):
    node.on_velocity_cmd(joint_cmd(["bend"], velocity=[1.0]))
    assert robot.joints["bend"].target_speed == 0


def test_velocity_cmd_spins_clamps_and_expires(node, robot, clock):
    node.on_velocity_cmd(joint_cmd(["wheel"], velocity=[100.0]))
    assert robot.joints["wheel"].target_speed == 500  # clamped to max_speed
    clock.now += 1.0
    node.tick()
    assert robot.joints["wheel"].target_speed == 0
    assert robot.joints["wheel"].command_source == "robot"


def test_cmd_vel_drives_the_base(base_node):
    msg = Twist()
    msg.linear.x = 0.1
    base_node.on_cmd_vel(msg)
    wheels = base_node.robot.joints
    assert [wheels[w].target_speed for w in ("fl", "fr", "rl", "rr")] == [
        100,
        100,
        100,
        100,
    ]


def test_cmd_vel_without_base_is_only_logged(node, robot):
    node.on_cmd_vel(Twist())
    assert robot.joints["wheel"].target_speed == 0


def test_odom_only_with_a_base(node, base_node):
    assert node.odom_pub is None
    assert base_node.odom_pub is not None


def test_odom_reports_measured_velocity(base_node, sdk):
    robot = base_node.robot
    for name in robot.joints:
        robot.joints[name].speed = 100
    msg = base_node.odom_msg()
    assert (msg.header.frame_id, msg.child_frame_id) == ("odom", "base_link")
    assert msg.twist.twist.linear.x == pytest.approx(0.1)
    assert msg.twist.twist.angular.z == pytest.approx(0.0)
    assert msg.twist.covariance[0] > 0


def test_stop_all_service(node, robot):
    node.on_velocity_cmd(joint_cmd(["wheel"], velocity=[1.0]))
    response = node.on_stop_all(Trigger.Request(), Trigger.Response())
    assert response.success
    assert robot.joints["wheel"].target_speed == 0


def test_torque_service(node, robot):
    handler = node._torque_handler("bend")
    assert handler(SetBool.Request(data=False), SetBool.Response()).success
    assert robot.joints["bend"].torque is False


def test_status_reports_each_joint(node):
    msg = node.status_msg()
    assert [s.name for s in msg.status] == ["robot: bend", "robot: wheel"]
    assert [s.hardware_id for s in msg.status] == [
        "feetech id 1",
        "feetech id 2",
    ]
    assert all(s.level == DiagnosticStatus.OK for s in msg.status)


def test_readout_topics_live_under_the_node(node):
    fields = ("current", "load", "moving", "temperature", "voltage")
    expected = sorted(
        f"/robot/{j}/{f}" for j in ("bend", "wheel") for f in fields
    )
    assert node.readouts.topic_names == expected


def test_readout_messages_carry_values():
    stamp = rclpy.clock.Clock().now().to_msg()
    msgs = JointReadoutPublishers.messages(
        "bend",
        stamp,
        temperature=31,
        voltage=12.1,
        current=-0.5,
        load=-40,
        moving=True,
    )
    assert msgs["temperature"].temperature == 31.0
    assert msgs["temperature"].header.frame_id == "bend"
    assert msgs["voltage"].data == pytest.approx(12.1)
    assert msgs["current"].data == pytest.approx(-0.5)
    assert msgs["load"].data == -40.0
    assert msgs["moving"].data is True


def test_readouts_skip_offline_joints(node, robot, monkeypatch):
    published = []
    monkeypatch.setattr(
        node.readouts,
        "publish",
        lambda joint, *_a, **_k: published.append(joint),
    )
    robot.joints["wheel"].online = False
    node.publish_readouts()
    assert published == ["bend"]


def status_values(node, joint):
    entry = next(
        s for s in node.status_msg().status if s.name.endswith(f": {joint}")
    )
    return {kv.key: kv.value for kv in entry.values}


def test_velocity_cmd_records_sender_from_frame_id(node):
    msg = joint_cmd(["wheel"], velocity=[step_speed_to_rad(100)])
    msg.header.frame_id = "cli"
    node.on_velocity_cmd(msg)
    values = status_values(node, "wheel")
    assert values["command_source"] == "cli"
    assert values["command_client"] == ""
    assert values["target_speed"] == "100"
    assert values["control"] == "speed"


def test_position_cmd_without_frame_id_counts_as_ros(node):
    node.on_position_cmd(joint_cmd(["bend"], position=[0.0]))
    values = status_values(node, "bend")
    assert values["command_source"] == "ros"
    assert values["target"] == str(CENTER_STEP)


def test_status_has_empty_command_before_any_command(node):
    values = status_values(node, "bend")
    assert values["command_source"] == "" and values["target"] == ""
