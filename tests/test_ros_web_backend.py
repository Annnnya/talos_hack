"""RosBackend and CLI --ros commands against a real robot node.

The robot node runs on a Robot over the fake SDK. conftest.py puts every test on
a private ROS domain, so nothing here can reach a real robot.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("diagnostic_msgs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("std_srvs")
pytest.importorskip("nav_msgs")

from conftest import BASE, SMALL, FakeClock, config, wait_for  # noqa: E402
from diagnostic_msgs.msg import (  # noqa: E402
    DiagnosticArray,
    DiagnosticStatus,
    KeyValue,
)
from fastapi.testclient import TestClient  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.defaults import WEB_CMD_VEL_TOPIC  # noqa: E402
from talos_servo.robot import SOURCE_CLI, CommandError, Robot  # noqa: E402
from talos_servo.ros import cli_commands  # noqa: E402
from talos_servo.ros.robot_node import RobotNode  # noqa: E402
from talos_servo.ros.web_backend import (  # noqa: E402
    STATUS_STALE_AFTER,
    RosBackend,
)
from talos_servo.web.server import create_app  # noqa: E402

CONFIG = config(SMALL)
WAIT = 3.0


@pytest.fixture(scope="module", autouse=True)
def ros():
    # Faster status than the 2 Hz default so tests do not wait long for it.
    rclpy.init(args=["test", "--ros-args", "-p", "status_hz:=20.0"])
    yield
    rclpy.shutdown()


@pytest.fixture
def robot(sdk):
    r = Robot(sdk.bus(), CONFIG)
    r.open()
    yield r
    r.close()


@contextmanager
def spinning(node) -> Iterator[None]:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield
    finally:
        executor.shutdown()
        thread.join(2.0)
        node.destroy_node()


@pytest.fixture
def robot_node(robot):
    node = RobotNode(robot)
    with spinning(node):
        yield node


def make_backend(**kwargs) -> RosBackend:
    return RosBackend(CONFIG, context=rclpy.get_default_context(), **kwargs)


@pytest.fixture
def backend(robot_node):
    b = make_backend()
    b.open()
    yield b
    b.close()


def backend_state(backend: RosBackend, joint: str) -> dict:
    return next(s for s in backend.snapshot() if s["name"] == joint)


def feed_watchdog(backend: RosBackend, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        backend.tick()
        time.sleep(0.05)


def test_open_mirrors_robot_state(backend, robot):
    wait_for(lambda: all(s["online"] for s in backend.snapshot()), WAIT)


def test_open_without_robot_node_fails_loudly():
    backend = make_backend(wait_timeout=0.3)
    with pytest.raises(ServoError, match="no robot node at /robot"):
        backend.open()


def test_move_reaches_robot_and_is_clamped(backend, robot):
    backend.move("bend", 4000, 300)
    wait_for(lambda: robot.joints["bend"].target == 3000, WAIT)
    assert backend_state(backend, "bend")["target"] == 3000


def test_spin_on_a_position_joint_is_refused(backend):
    with pytest.raises(CommandError, match="position joint"):
        backend.spin("bend", 100)


def test_torque_goes_through_the_robot_service(backend, robot):
    backend.set_torque("bend", False)
    assert robot.joints["bend"].torque is False


def test_ticking_keeps_wheel_spinning_and_stopping_ticks_stops_it(
    backend, robot
):
    backend.spin("wheel", 900)
    wait_for(
        lambda: robot.joints["wheel"].target_speed == CONFIG.max_speed, WAIT
    )
    # Longer than the robot's 0.5 s command timeout.
    feed_watchdog(backend, 1.0)
    assert robot.joints["wheel"].target_speed == CONFIG.max_speed
    wait_for(lambda: robot.joints["wheel"].target_speed == 0, WAIT)


def test_stop_all_stops_wheels(backend, robot):
    backend.spin("wheel", 200)
    wait_for(lambda: robot.joints["wheel"].target_speed == 200, WAIT)
    backend.stop_all()
    assert robot.joints["wheel"].target_speed == 0
    assert backend_state(backend, "wheel")["target_speed"] == 0


def test_browser_commands_reach_the_robot(robot_node, robot):
    with (
        TestClient(create_app(make_backend())) as client,
        client.websocket_connect("/ws") as ws,
    ):
        ws.receive_json()
        ws.send_json(
            {"type": "move", "joint": "bend", "position": 1500, "speed": 100}
        )
        wait_for(lambda: robot.joints["bend"].target == 1500, WAIT)


def status_msg(
    level: int = DiagnosticStatus.OK, name: str = "robot: bend", **values: str
) -> DiagnosticArray:
    fields = {
        "control": "position",
        "torque": "True",
        "load": "5",
        "voltage": "12.1",
        "temp": "31",
        "current": "0.2",
        "moving": "False",
        "target": "",
        "target_speed": "0",
        "command_source": "",
        "command_client": "",
        "command_age": "",
    } | values
    status = DiagnosticStatus(
        name=name,
        level=level,
        message="ok",
        values=[KeyValue(key=k, value=v) for k, v in fields.items()],
    )
    return DiagnosticArray(status=[status])


def test_status_fields_are_parsed():
    backend = make_backend()
    backend._on_status(status_msg(temp="44", voltage="11.5"))
    s = backend_state(backend, "bend")
    assert (s["online"], s["temp"], s["voltage"], s["torque"]) == (
        True,
        44,
        11.5,
        True,
    )


def test_diagnostics_from_other_nodes_are_ignored():
    backend = make_backend()
    backend._on_status(status_msg(name="imu: bend", temp="99"))
    assert backend_state(backend, "bend")["temp"] == 0
    assert not backend._first_status.is_set()


def test_silent_robot_node_marks_joints_offline():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    backend._on_status(status_msg())
    clock.now += STATUS_STALE_AFTER + 0.1
    backend.tick()
    s = backend_state(backend, "bend")
    assert not s["online"]
    assert "no status from the robot node" in s["error"]


# --- talos-servo spin --ros / monitor --ros
# --------------------------------------


def run_in_thread(fn, *args, **kwargs) -> threading.Thread:
    thread = threading.Thread(target=fn, args=args, kwargs=kwargs, daemon=True)
    thread.start()
    return thread


def assert_stopped_by_cli(robot: Robot) -> None:
    # The robot node applies the stop on its own executor thread, so it can land
    # just after the CLI returns. Checking the source rules out the watchdog.
    wait_for(lambda: robot.joints["wheel"].target_speed == 0, WAIT)
    assert robot.joints["wheel"].command_source == SOURCE_CLI


def test_cli_spin_over_ros_spins_then_stops(robot_node, robot):
    lines: list[str] = []
    thread = run_in_thread(
        cli_commands.spin_over_ros,
        make_backend(),
        ["wheel"],
        300,
        1.0,
        out=lines.append,
    )
    wait_for(lambda: robot.joints["wheel"].target_speed == 300, WAIT)
    thread.join(5.0)
    assert_stopped_by_cli(robot)
    assert lines[-1] == "stopped"


def test_cli_monitor_over_ros_keeps_spinning_with_slow_prints(
    robot_node, robot
):
    lines: list[str] = []
    # Print interval of 1 s is twice the robot's command timeout.
    thread = run_in_thread(
        cli_commands.monitor_over_ros,
        make_backend(),
        ["wheel"],
        200,
        1.5,
        1.0,
        out=lines.append,
    )
    wait_for(lambda: robot.joints["wheel"].target_speed == 200, WAIT)
    time.sleep(1.0)
    assert robot.joints["wheel"].target_speed == 200
    thread.join(5.0)
    assert_stopped_by_cli(robot)
    assert any(line.startswith("wheel: position=") for line in lines)


def test_cli_spin_over_ros_interrupted_still_stops(
    robot_node, robot, monkeypatch
):
    def interrupt(*_args, **_kwargs):
        wait_for(lambda: robot.joints["wheel"].target_speed == 300, WAIT)
        raise KeyboardInterrupt

    monkeypatch.setattr(cli_commands, "_hold", interrupt)
    with pytest.raises(KeyboardInterrupt):
        cli_commands.spin_over_ros(
            make_backend(), ["wheel"], 300, 10.0, out=lambda _l: None
        )
    wait_for(lambda: robot.joints["wheel"].target_speed == 0, WAIT)


def test_cli_over_ros_rejects_unknown_joints_before_connecting():
    backend = make_backend()
    with pytest.raises(ServoError, match=r"joints \['nope'\]"):
        cli_commands.spin_over_ros(backend, ["nope"], 100, 1.0)
    assert backend._node is None


def test_drive_over_ros_publishes_cmd_vel_and_stops(base_node):
    keys = ["w"] * 5 + ["x"]
    sent = []
    backend = RosBackend(BASE_CONFIG, context=rclpy.get_default_context())
    real_drive = backend.drive

    def record(*v):
        sent.append(v)
        real_drive(*v)

    backend.drive = record
    cli_commands.drive_over_ros(
        backend,
        lambda _t: keys.pop(0) if keys else "x",
        linear=0.1,
        angular=0.5,
    )
    assert sent[0] == (0.1, 0.0, 0.0)
    assert sent[-1] == (0.0, 0.0, 0.0)


# --- last command: who moved a joint
# ---------------------------------------------


def test_cli_spin_shows_up_in_the_web_backend(backend, robot):
    thread = run_in_thread(
        cli_commands.spin_over_ros,
        make_backend(),
        ["wheel"],
        300,
        1.5,
        out=lambda _l: None,
    )
    wait_for(
        lambda: (
            (backend_state(backend, "wheel")["last_command"] or {}).get(
                "source"
            )
            == "cli"
            and backend_state(backend, "wheel")["target_speed"] == 300
        ),
        WAIT,
    )
    thread.join(5.0)


def test_web_backend_stops_feeding_a_wheel_someone_else_took_over(
    backend, robot
):
    backend.spin("wheel", 200)
    # Two publishers do not share an order, so let ours land first.
    wait_for(lambda: robot.joints["wheel"].target_speed == 200, WAIT)
    other = make_backend()
    other.open()
    try:
        other.spin("wheel", 100, source="cli")
        wait_for(lambda: robot.joints["wheel"].target_speed == 100, WAIT)
        # The backend only learns about the takeover from the robot's status.
        wait_for(
            lambda: (
                backend_state(backend, "wheel")["last_command"]["source"]
                == "cli"
            ),
            WAIT,
        )
    finally:
        other.release()  # like a CLI process dying without a stop
    # The web backend keeps ticking, but must not keep the CLI's wheel alive.
    feed_watchdog(backend, 1.5)
    assert robot.joints["wheel"].target_speed == 0
    wait_for(
        lambda: (
            backend_state(backend, "wheel")["last_command"]["source"] == "robot"
        ),
        WAIT,
    )


def test_own_stale_echo_does_not_move_the_slider_back():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    j = backend.joints["bend"]
    with backend._lock:
        backend._record(j, "web", "a1")
    j.target = 1500
    clock.now += 0.3
    backend._on_status(
        status_msg(
            target="1200",
            command_source="web",
            command_client="a1",
            command_age="0.2",
        )
    )
    assert backend_state(backend, "bend")["target"] == 1500


def test_command_from_another_sender_is_adopted_right_away():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    with backend._lock:
        backend._record(backend.joints["bend"], "web", "a1")
    backend._on_status(
        status_msg(target="1200", command_source="cli", command_age="0.0")
    )
    state = backend_state(backend, "bend")
    assert state["target"] == 1200
    assert state["last_command"] == {
        "source": "cli",
        "client": None,
        "age": 0.0,
    }


# --- drive through the web input of twist_mux
# ------------------------------------------------------------------------
# No twist_mux runs here and the robot node listens on /cmd_vel, so what the
# web backend sends on its mux input must not move a wheel by itself.

BASE_CONFIG = config(BASE)


@pytest.fixture
def base_robot(sdk):
    r = Robot(sdk.bus(), BASE_CONFIG)
    r.open()
    yield r
    r.close()


@pytest.fixture
def base_node(base_robot):
    node = RobotNode(base_robot)
    with spinning(node):
        yield node


@pytest.fixture
def web_input():
    """Every Twist published on the web input of twist_mux."""
    received: list[Twist] = []
    node = rclpy.create_node("web_input_listener")
    node.create_subscription(Twist, WEB_CMD_VEL_TOPIC, received.append, 10)
    with spinning(node):
        yield received


def make_web_backend(cfg=BASE_CONFIG) -> RosBackend:
    return RosBackend(
        cfg,
        context=rclpy.get_default_context(),
        cmd_vel_topic=WEB_CMD_VEL_TOPIC,
    )


def velocity(msg: Twist) -> tuple[float, float, float]:
    return msg.linear.x, msg.linear.y, msg.angular.z


def send_until_received(send: Callable[[], None], received: list) -> None:
    """Re-send until the listener has matched the publisher and got one."""
    deadline = time.monotonic() + WAIT
    while not received:
        if time.monotonic() > deadline:
            raise AssertionError("nothing arrived on the web input")
        send()
        time.sleep(0.05)


def base_wheel_speeds(robot: Robot) -> list[int]:
    return [robot.joints[w].target_speed for w in BASE_CONFIG.base.wheels]


def test_drive_publishes_clamped_twist_on_the_web_input(
    base_node, base_robot, web_input
):
    backend = make_web_backend()
    backend.open()
    try:
        send_until_received(
            lambda: backend.drive(9.0, -9.0, math.nan), web_input
        )
        assert velocity(web_input[-1]) == (0.2, -0.2, 0.0)
        time.sleep(0.2)
        assert base_wheel_speeds(base_robot) == [0, 0, 0, 0]
    finally:
        backend.release()


def test_drive_without_a_base_is_refused(backend):
    with pytest.raises(CommandError, match="no base"):
        backend.drive(0.1, 0.0, 0.0)


def test_stop_own_leaves_other_senders_alone(robot_node, robot, web_input):
    backend = make_web_backend(CONFIG)
    backend.open()
    try:
        # Another interface drives the wheel, without expiry.
        robot.spin("wheel", 300, source=SOURCE_CLI)
        send_until_received(backend.stop_own, web_input)
        assert velocity(web_input[-1]) == (0.0, 0.0, 0.0)
        time.sleep(0.2)
        assert robot.joints["wheel"].target_speed == 300
    finally:
        backend.release()


def test_stop_own_stops_the_joints_the_backend_spins(robot_node, robot):
    backend = make_web_backend(CONFIG)
    backend.open()
    try:
        backend.spin("wheel", 200)
        wait_for(lambda: robot.joints["wheel"].target_speed == 200, WAIT)
        backend.stop_own()
        wait_for(lambda: robot.joints["wheel"].target_speed == 0, WAIT)
        assert backend_state(backend, "wheel")["target_speed"] == 0
        # Nothing is fed any more.
        backend.tick()
        time.sleep(0.2)
        assert robot.joints["wheel"].target_speed == 0
    finally:
        backend.release()


def test_browser_drive_reaches_the_web_input_and_leaving_stops_only_it(
    base_node, base_robot, web_input
):
    # Another interface drives a wheel; the browser leaving must not stop it.
    base_robot.spin("fl", 100, source=SOURCE_CLI)
    command = {"type": "drive", "vx": 0.1, "vy": 0.0, "wz": 0.0}
    with TestClient(create_app(make_web_backend(), ros=True)) as client:
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
            send_until_received(lambda: ws.send_json(command), web_input)
            assert velocity(web_input[-1]) == (0.1, 0.0, 0.0)
        wait_for(lambda: velocity(web_input[-1]) == (0.0, 0.0, 0.0), WAIT)
        assert base_robot.joints["fl"].target_speed == 100
