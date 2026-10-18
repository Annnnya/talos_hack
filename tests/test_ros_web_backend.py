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
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import CompressedImage, Imu, JointState  # noqa: E402
from std_msgs.msg import Float64, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from talos_servo import teleop  # noqa: E402
from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.defaults import (  # noqa: E402
    KEYBOARD_CMD_VEL_TOPIC,
    WEB_CMD_VEL_TOPIC,
)
from talos_servo.orientation import from_rpy  # noqa: E402
from talos_servo.robot import SOURCE_CLI, CommandError, Robot  # noqa: E402
from talos_servo.ros import cli_commands  # noqa: E402
from talos_servo.ros.imu_convert import (  # noqa: E402
    not_provided_covariance,
)
from talos_servo.ros.mode_node import ModeNode  # noqa: E402
from talos_servo.ros.names import (  # noqa: E402
    ACTIVE_INPUT_TOPIC,
    CAMERA_IMAGE_TOPIC,
    EXPANSION_RATE_TOPIC,
    IMU_DATA_TOPIC,
    LATCHED,
    MODE_REASON_TOPIC,
    MODE_TOPIC,
    RELEASE_SERVICE,
    WALL_NEARBY_SERVICE,
)
from talos_servo.ros.robot_node import RobotNode  # noqa: E402
from talos_servo.ros.web_backend import (  # noqa: E402
    CAMERA_STALE_AFTER,
    STATUS_STALE_AFTER,
    TILT_STALE_AFTER,
    RosBackend,
)
from talos_servo.web.server import create_app  # noqa: E402

CONFIG = config(SMALL)
WAIT = 3.0
SHORT_WAIT = 1.0


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


@pytest.fixture
def mode_node():
    # Real mode node, no controller_manager: only STOP matters here.
    node = ModeNode(node_name="mode_for_web_backend")
    with spinning(node):
        yield node


def wait_for_mode_node(backend: RosBackend) -> None:
    client = backend._require_node().set_stop_client
    wait_for(client.service_is_ready, WAIT)


def test_release_stop_and_stop_all_go_through_the_mode_node(backend, mode_node):
    wait_for_mode_node(backend)
    assert backend.stopped()  # the mode node starts with STOP on
    backend.release_stop()
    assert not mode_node.stopped
    wait_for(lambda: not backend.stopped(), WAIT)
    backend.stop_all()
    assert mode_node.stopped
    wait_for(backend.stopped, WAIT)


def test_stopped_once_the_mode_node_goes_quiet(backend):
    node = ModeNode(node_name="mode_that_dies")
    with spinning(node):
        wait_for_mode_node(backend)
        backend.release_stop()
        wait_for(lambda: not backend.stopped(), WAIT)
    # Like twist_mux: no beats within the lock timeout means STOP on.
    wait_for(backend.stopped, WAIT)


def test_release_stop_without_a_mode_node_fails_loudly(backend):
    with pytest.raises(CommandError, match="mode node is not running"):
        backend.release_stop()


def test_stop_all_without_a_mode_node_still_stops_the_wheels(backend, robot):
    backend.spin("wheel", 300)
    wait_for(lambda: robot.joints["wheel"].target_speed == 300, WAIT)
    backend.stop_all()
    wait_for(lambda: robot.joints["wheel"].target_speed == 0, WAIT)


def test_browser_sees_stop_and_can_release_it(robot_node, mode_node):
    backend = make_backend()
    with TestClient(create_app(backend, ros=True)) as client:
        wait_for_mode_node(backend)
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
            assert ws.receive_json()["stopped"] is True
            ws.send_json({"type": "release_stop"})
            wait_for(lambda: not mode_node.stopped, WAIT)
            wait_for(lambda: ws.receive_json()["stopped"] is False, WAIT)
            ws.send_json({"type": "stop_all"})
            wait_for(lambda: mode_node.stopped, WAIT)


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


def test_nan_joint_states_keep_the_last_reading():
    backend = make_backend()
    backend._on_joint_states(
        JointState(name=["bend"], position=[0.5], velocity=[0.1])
    )
    before = backend_state(backend, "bend")
    backend._on_joint_states(
        JointState(name=["bend"], position=[math.nan], velocity=[math.nan])
    )
    after = backend_state(backend, "bend")
    assert (after["position"], after["speed"]) == (
        before["position"],
        before["speed"],
    )


# --- platform tilt from the IMU (#91) --------------------------------------


def imu_msg(roll: float, pitch: float, *, raw: bool = False) -> Imu:
    msg = Imu()
    x, y, z, w = from_rpy(roll, pitch, 0.0)
    msg.orientation.x, msg.orientation.y = x, y
    msg.orientation.z, msg.orientation.w = z, w
    if raw:
        msg.orientation_covariance = not_provided_covariance()
    return msg


def test_tilt_comes_from_the_filtered_imu():
    backend = make_backend()
    assert backend.tilt() is None
    backend._on_imu(imu_msg(math.radians(-2.0), math.radians(5.0)))
    tilt = backend.tilt()
    assert tilt["roll_deg"] == pytest.approx(-2.0)
    assert tilt["pitch_deg"] == pytest.approx(5.0)


def test_raw_imu_without_an_orientation_is_ignored():
    backend = make_backend()
    backend._on_imu(imu_msg(0.1, 0.2, raw=True))
    assert backend.tilt() is None


def test_tilt_goes_stale_when_the_imu_chain_stops():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    backend._on_imu(imu_msg(0.1, 0.0))
    assert backend.tilt() is not None
    clock.now += TILT_STALE_AFTER + 0.1
    assert backend.tilt() is None


def test_tilt_takes_the_mount_off():
    from dataclasses import replace

    from talos_servo.config import ImuMountConfig

    mount = ImuMountConfig(rpy=(0.0, 0.0, math.pi / 2))
    backend = RosBackend(
        replace(CONFIG, imu=mount), context=rclpy.get_default_context()
    )
    # The board is turned a quarter turn: its own pitch is the platform's
    # roll, with the sign from the rotation.
    from talos_servo.orientation import multiply

    x, y, z, w = multiply(from_rpy(0.2, 0.0, 0.0), from_rpy(*mount.rpy))
    msg = Imu()
    msg.orientation.x, msg.orientation.y = x, y
    msg.orientation.z, msg.orientation.w = z, w
    backend._on_imu(msg)
    tilt = backend.tilt()
    assert tilt["roll_deg"] == pytest.approx(math.degrees(0.2))
    assert tilt["pitch_deg"] == pytest.approx(0.0, abs=1e-6)


def test_tilt_arrives_over_the_imu_topic(backend):
    node = rclpy.create_node("imu_for_web_backend")
    pub = node.create_publisher(Imu, IMU_DATA_TOPIC, 10)
    try:
        wait_for(lambda: pub.get_subscription_count() > 0, WAIT)
        deadline = time.monotonic() + WAIT
        while backend.tilt() is None and time.monotonic() < deadline:
            pub.publish(imu_msg(0.0, math.radians(3.0)))
            time.sleep(0.05)
        assert backend.tilt()["pitch_deg"] == pytest.approx(3.0)
    finally:
        node.destroy_node()


# --- camera frame for the page (#147) --------------------------------------

JPEG = b"\xff\xd8 fake jpeg \xff\xd9"


def camera_msg(data: bytes = JPEG, fmt: str = "jpeg") -> CompressedImage:
    msg = CompressedImage()
    msg.format = fmt
    msg.data = data
    return msg


def test_camera_frame_is_the_latest_jpeg():
    backend = make_backend()
    assert backend.camera_frame() is None
    backend._on_camera(camera_msg(b"\xff\xd8 old \xff\xd9"))
    backend._on_camera(camera_msg())
    assert backend.camera_frame() == JPEG


def test_camera_frame_that_is_not_jpeg_is_ignored():
    backend = make_backend()
    backend._on_camera(camera_msg(b"\x89PNG", fmt="png"))
    assert backend.camera_frame() is None


def test_camera_frame_goes_stale_when_the_camera_stops():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    backend._on_camera(camera_msg())
    assert backend.camera_frame() == JPEG
    clock.now += CAMERA_STALE_AFTER + 0.1
    assert backend.camera_frame() is None


def test_camera_frame_arrives_over_the_camera_topic(backend):
    node = rclpy.create_node("camera_for_web_backend")
    pub = node.create_publisher(CompressedImage, CAMERA_IMAGE_TOPIC, 2)
    try:
        wait_for(lambda: pub.get_subscription_count() > 0, WAIT)
        deadline = time.monotonic() + WAIT
        while backend.camera_frame() is None and time.monotonic() < deadline:
            pub.publish(camera_msg())
            time.sleep(0.05)
        assert backend.camera_frame() == JPEG
    finally:
        node.destroy_node()


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


def recording_backend(sent: list, **kwargs) -> RosBackend:
    # The base config: drive refuses a config without a base.
    backend = RosBackend(
        BASE_CONFIG,
        context=rclpy.get_default_context(),
        cmd_vel_topic=KEYBOARD_CMD_VEL_TOPIC,
        **{"wait_timeout": SHORT_WAIT, **kwargs},
    )
    real_drive = backend.drive

    def record(*v):
        sent.append(v)
        real_drive(*v)

    backend.drive = record
    return backend


def scripted_keys(steps: list) -> teleop.ReadKey:
    """Key reader for the drive loop. A callable step runs first (to wait
    for or change robot state) and returns the key."""

    def read_key(_timeout: float) -> str | None:
        if not steps:
            return "x"
        step = steps.pop(0)
        return step() if callable(step) else step

    return read_key


@pytest.fixture
def listener():
    """A plain node that records Twists on a topic."""
    node = rclpy.create_node("twist_listener")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    received: list[TwistStamped] = []

    def listen(topic: str) -> list[TwistStamped]:
        node.create_subscription(TwistStamped, topic, received.append, 10)
        return received

    yield listen
    executor.shutdown()
    thread.join(2.0)
    node.destroy_node()


def test_drive_over_ros_warns_without_twist_mux(robot_node):
    lines: list[str] = []
    cli_commands.drive_over_ros(
        # Short: the twist_mux note waits up to wait_timeout.
        make_backend(
            cmd_vel_topic=KEYBOARD_CMD_VEL_TOPIC, wait_timeout=SHORT_WAIT
        ),
        scripted_keys([]),
        linear=0.1,
        angular=0.5,
        out=lines.append,
    )
    assert any("twist-mux" in line for line in lines)


def test_drive_over_ros_waits_for_twist_mux_before_warning(base_node):
    # twist_mux shows up only after open(), like DDS discovery settling.
    late = rclpy.create_node("late_twist_mux")
    backend = recording_backend([])
    original_open = backend.open

    def open_then_listen() -> None:
        original_open()
        late.create_subscription(
            TwistStamped, KEYBOARD_CMD_VEL_TOPIC, lambda _m: None, 10
        )

    backend.open = open_then_listen
    lines: list[str] = []
    try:
        cli_commands.drive_over_ros(
            backend,
            scripted_keys([]),
            linear=0.1,
            angular=0.5,
            out=lines.append,
        )
    finally:
        late.destroy_node()
    assert not any("twist-mux" in line for line in lines)


def test_space_turns_stop_on_and_enter_releases_it(base_node, mode_node):
    backend = recording_backend([])

    def until(stopped: bool) -> None:
        wait_for(lambda: mode_node.stopped is stopped, WAIT)

    cli_commands.drive_over_ros(
        backend,
        scripted_keys(
            [
                lambda: wait_for_mode_node(backend),
                "\n",
                lambda: until(False),
                " ",
                lambda: until(True),
            ]
        ),
        linear=0.1,
        angular=0.5,
        out=lambda _line: None,
    )
    assert mode_node.stopped


def test_mode_calls_run_off_the_key_loop_one_at_a_time():
    lines: list[str] = []
    calls = cli_commands._ModeCalls(lines.append)
    gate = threading.Event()
    ran: list[str] = []

    def slow_wall_nearby() -> None:
        gate.wait(WAIT)
        ran.append("wall")

    wall = calls.mode_call(slow_wall_nearby, "wall nearby")
    release = calls.mode_call(lambda: ran.append("release"), "release")
    stop = calls.stop_call(lambda: ran.append("stop"))
    wall()  # returns at once although the call is still running
    release()  # busy: ignored with a note
    stop()  # STOP does not wait for the mode call
    wait_for(lambda: "stop" in ran, WAIT)
    assert ran == ["stop"]
    assert lines == ["note: mode node still busy, release ignored"]
    gate.set()
    calls.close(WAIT)
    assert ran == ["stop", "wall"]


def test_mode_call_failures_are_printed():
    lines: list[str] = []
    calls = cli_commands._ModeCalls(lines.append)

    def fail() -> None:
        raise CommandError("release STOP: mode node is not running")

    calls.stop_call(fail)()
    calls.close(WAIT)
    assert lines == ["error: release STOP: mode node is not running"]


def test_drive_over_ros_publishes_on_the_keyboard_input(
    base_node, base_robot, listener
):
    received = listener(KEYBOARD_CMD_VEL_TOPIC)
    sent: list = []
    backend = recording_backend(sent)
    lines: list[str] = []
    cli_commands.drive_over_ros(
        backend,
        scripted_keys(
            [lambda: wait_for(backend.has_cmd_vel_listener, WAIT)] + ["w"] * 5
        ),
        linear=0.1,
        angular=0.5,
        out=lines.append,
    )
    assert sent[0] == (0.1, 0.0, 0.0)
    assert sent[-1] == (0.0, 0.0, 0.0)
    wait_for(lambda: any(m.twist.linear.x == 0.1 for m in received), WAIT)
    # The robot node only listens on /cmd_vel, which twist_mux feeds.
    assert not any(base_wheel_speeds(base_robot))


class FakeModeNode:
    """Latched mode, reason and active input topics plus the wall nearby and
    release services, like the mode node and mux_status."""

    def __init__(self, *, release_ok: bool = True) -> None:
        self.node = rclpy.create_node("fake_mode")
        self.calls: list[str] = []
        self.release_ok = release_ok
        self.mode_pub = self.node.create_publisher(String, MODE_TOPIC, LATCHED)
        self.reason_pub = self.node.create_publisher(
            String, MODE_REASON_TOPIC, LATCHED
        )
        self.active_pub = self.node.create_publisher(
            String, ACTIVE_INPUT_TOPIC, LATCHED
        )
        self.node.create_service(
            Trigger, WALL_NEARBY_SERVICE, self._on_wall_nearby
        )
        self.node.create_service(Trigger, RELEASE_SERVICE, self._on_release)
        self.publish("free", "", "none")

    def publish(self, mode: str, reason: str, active: str) -> None:
        self.mode_pub.publish(String(data=mode))
        self.reason_pub.publish(String(data=reason))
        self.active_pub.publish(String(data=active))

    def _on_wall_nearby(self, _request, response):
        self.calls.append("wall_nearby")
        self.publish("aligning", "wall nearby", "none")
        response.success, response.message = True, "mode aligning"
        return response

    def _on_release(self, _request, response):
        self.calls.append("release")
        response.success = self.release_ok
        response.message = "mode free" if self.release_ok else "not now"
        return response


@pytest.fixture
def fake_mode_node():
    def start(**kwargs) -> FakeModeNode:
        fake = FakeModeNode(**kwargs)
        executor.add_node(fake.node)
        thread.start()
        started.append(fake)
        return fake

    executor = SingleThreadedExecutor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    started: list[FakeModeNode] = []
    yield start
    executor.shutdown()
    if started:
        thread.join(2.0)
        started[0].node.destroy_node()


def wait_for_status(
    backend: RosBackend, mode: str, reason: str, active: str
) -> None:
    """Wait until all three latched topics arrived; they come in any order."""
    wait_for(lambda: backend.robot_status() == (mode, reason, active), WAIT)


def test_keyboard_follows_the_mode_from_any_interface(
    base_node, fake_mode_node
):
    fake = fake_mode_node()
    sent: list = []
    backend = recording_backend(sent)
    lines: list[str] = []

    def climb_from_the_web() -> None:
        fake.publish("climbing", "wall contact", "web")
        wait_for_status(backend, "climbing", "wall contact", "web")

    cli_commands.drive_over_ros(
        backend,
        scripted_keys(
            [
                lambda: wait_for_status(backend, "free", "", "none"),
                "n",
                # N runs off the key loop; no key while it lands, so the
                # status line prints it.
                lambda: wait_for_status(
                    backend, "aligning", "wall nearby", "none"
                ),
                climb_from_the_web,
                None,
                "q",  # turning is off while climbing
                "w",
            ]
        ),
        linear=0.1,
        angular=0.5,
        out=lines.append,
    )
    assert fake.calls == ["wall_nearby"]
    assert sent == [(0.1, 0.0, 0.0), (0.0, 0.0, 0.0)]
    assert "mode: free | driving: none" in lines
    assert "mode: aligning (wall nearby) | driving: none" in lines
    assert "mode: climbing (wall contact) | driving: web" in lines


def test_rejected_release_is_printed_and_driving_goes_on(
    base_node, fake_mode_node
):
    fake = fake_mode_node(release_ok=False)
    sent: list = []
    backend = recording_backend(sent)
    lines: list[str] = []
    cli_commands.drive_over_ros(
        backend,
        scripted_keys(
            [lambda: wait_for_status(backend, "free", "", "none"), "l", "w"]
        ),
        linear=0.1,
        angular=0.5,
        out=lines.append,
    )
    assert fake.calls == ["release"]
    assert "error: release: not now" in lines
    assert sent[0] == (0.1, 0.0, 0.0)


def test_wall_nearby_without_mode_node_says_so(robot_node):
    lines: list[str] = []
    cli_commands.drive_over_ros(
        # Short: the twist_mux note waits up to wait_timeout.
        make_backend(
            cmd_vel_topic=KEYBOARD_CMD_VEL_TOPIC, wait_timeout=SHORT_WAIT
        ),
        scripted_keys(["n"]),
        linear=0.1,
        angular=0.5,
        out=lines.append,
    )
    assert any("mode node is not running" in line for line in lines)
    assert "mode: ? | driving: ?" in lines


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
    """Every twist published on the web input of twist_mux."""
    received: list[TwistStamped] = []
    node = rclpy.create_node("web_input_listener")
    node.create_subscription(
        TwistStamped, WEB_CMD_VEL_TOPIC, received.append, 10
    )
    with spinning(node):
        yield received


def make_web_backend(cfg=BASE_CONFIG) -> RosBackend:
    return RosBackend(
        cfg,
        context=rclpy.get_default_context(),
        cmd_vel_topic=WEB_CMD_VEL_TOPIC,
    )


def velocity(msg: TwistStamped) -> tuple[float, float, float]:
    twist = msg.twist
    return twist.linear.x, twist.linear.y, twist.angular.z


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


def test_drive_over_ros_moves_the_hinge_with_r(base_node):
    received: list[Float64] = []
    node = rclpy.create_node("expansion_listener")
    node.create_subscription(Float64, EXPANSION_RATE_TOPIC, received.append, 10)
    backend = recording_backend([])
    rates: list[float] = []
    real_move = backend.move_hinge

    def record(rate: float) -> None:
        rates.append(rate)
        real_move(rate)

    backend.move_hinge = record

    def subscribed() -> str:
        wait_for(
            lambda: backend._node.expansion_pub.get_subscription_count(), WAIT
        )
        return "r"

    with spinning(node):
        cli_commands.drive_over_ros(
            backend,
            scripted_keys([subscribed, *["r"] * 4]),
            linear=0.1,
            angular=0.5,
            out=lambda _line: None,
        )
        rate = BASE_CONFIG.expansion.max_rate_rad_s
        assert rates == [rate] * 5 + [0.0]
        wait_for(lambda: any(m.data == rate for m in received), WAIT)
        wait_for(lambda: received[-1].data == 0.0, WAIT)
