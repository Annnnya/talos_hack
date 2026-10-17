"""Web backend talking to a real servo bridge node on the simulated bus, all in one process."""

from __future__ import annotations

import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("diagnostic_msgs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("std_srvs")

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.ros.servo_bridge_node import ServoBridgeNode  # noqa: E402
from talos_servo.ros.web_backend import STATUS_STALE_AFTER, RosBackend  # noqa: E402
from talos_servo.web.config import parse_config  # noqa: E402
from talos_servo.web.controller import WHEEL, CommandError, RobotController  # noqa: E402
from talos_servo.web.server import create_app  # noqa: E402
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
BASE, WHEEL_ID = 1, 2
# Never the default /servo_bridge: a real bridge on the same ROS network must not get our commands.
BRIDGE_NAME = f"test_servo_bridge_{os.getpid()}"


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture(scope="module", autouse=True)
def ros():
    # Faster bridge status than the 2 Hz default so tests do not wait long for it.
    rclpy.init(args=["test", "--ros-args", "-p", "status_hz:=20.0"])
    yield
    rclpy.shutdown()


@pytest.fixture
def controller():
    c = RobotController(make_bus(None, 1_000_000, mock=True, ids=[BASE, WHEEL_ID]), CONFIG)
    c.open()
    yield c
    c.close()


@pytest.fixture
def bridge(controller):
    node = ServoBridgeNode(controller, node_name=BRIDGE_NAME)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield node
    executor.shutdown()
    thread.join(2.0)
    node.destroy_node()


def make_backend(**kwargs) -> RosBackend:
    kwargs.setdefault("bridge", f"/{BRIDGE_NAME}")
    return RosBackend(CONFIG, context=rclpy.get_default_context(), **kwargs)


@pytest.fixture
def backend(bridge):
    b = make_backend()
    b.open()
    yield b
    b.close()


def wait_for(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, "condition not reached in time"
        time.sleep(0.02)


def backend_state(backend: RosBackend, servo_id: int) -> dict:
    return next(s for s in backend.snapshot() if s["id"] == servo_id)


def feed_watchdog(backend: RosBackend, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        backend.poll()
        time.sleep(0.05)


def test_open_mirrors_bridge_state(backend, controller):
    wait_for(lambda: all(s["online"] for s in backend.snapshot()))
    wait_for(lambda: backend_state(backend, BASE)["position"] == controller.servos[BASE].position)


def test_open_without_bridge_fails_loudly():
    backend = make_backend(bridge="/nobody_here", wait_timeout=0.3)
    with pytest.raises(ServoError, match="no servo bridge at /nobody_here"):
        backend.open()


def test_move_reaches_bridge_and_is_clamped(backend, controller):
    backend.move(BASE, 4000, 300)
    wait_for(lambda: controller.servos[BASE].target == 3000)
    assert backend_state(backend, BASE)["target"] == 3000


def test_spin_in_position_mode_is_refused(backend):
    with pytest.raises(CommandError, match="position mode"):
        backend.spin(WHEEL_ID, 100)


def test_set_mode_and_torque_go_through_bridge_services(backend, controller):
    backend.set_mode(WHEEL_ID, WHEEL)
    assert controller.servos[WHEEL_ID].mode == WHEEL
    backend.set_torque(BASE, False)
    assert controller.servos[BASE].torque is False


def test_polling_keeps_wheel_spinning_and_stopping_polls_stops_it(backend, controller):
    backend.set_mode(WHEEL_ID, WHEEL)
    backend.spin(WHEEL_ID, 900)
    wait_for(lambda: controller.servos[WHEEL_ID].target_speed == CONFIG.max_speed)
    # Longer than the bridge's 0.5 s command timeout.
    feed_watchdog(backend, 1.0)
    assert controller.servos[WHEEL_ID].target_speed == CONFIG.max_speed
    wait_for(lambda: controller.servos[WHEEL_ID].target_speed == 0, timeout=2.0)


def test_stop_all_stops_wheels(backend, controller):
    backend.set_mode(WHEEL_ID, WHEEL)
    backend.spin(WHEEL_ID, 200)
    wait_for(lambda: controller.servos[WHEEL_ID].target_speed == 200)
    backend.stop_all()
    assert controller.servos[WHEEL_ID].target_speed == 0
    assert backend_state(backend, WHEEL_ID)["target_speed"] == 0


def test_mode_changed_by_another_client_shows_up(backend, controller):
    controller.set_mode(WHEEL_ID, WHEEL)
    wait_for(lambda: backend_state(backend, WHEEL_ID)["mode"] == WHEEL)


def test_browser_commands_reach_the_bridge(bridge, controller):
    with TestClient(create_app(make_backend())) as client, client.websocket_connect("/ws") as ws:
        ws.receive_json()
        ws.send_json({"type": "move", "id": BASE, "position": 1500, "speed": 100})
        wait_for(lambda: controller.servos[BASE].target == 1500)


def status_msg(level: int = DiagnosticStatus.OK, **values: str) -> DiagnosticArray:
    fields = {
        "mode": "position",
        "torque": "True",
        "load": "5",
        "voltage": "12.1",
        "temp": "31",
        "current": "0.2",
        "moving": "False",
    } | values
    status = DiagnosticStatus(
        name="servo_bridge: base",
        level=level,
        message="ok",
        values=[KeyValue(key=k, value=v) for k, v in fields.items()],
    )
    return DiagnosticArray(status=[status])


def test_status_fields_are_parsed():
    backend = make_backend()
    backend._on_status(status_msg(temp="44", voltage="11.5"))
    s = backend_state(backend, BASE)
    assert (s["online"], s["temp"], s["voltage"], s["torque"]) == (True, 44, 11.5, True)


def test_silent_bridge_marks_servos_offline():
    clock = FakeClock()
    backend = make_backend(clock=clock)
    backend._on_status(status_msg())
    clock.now += STATUS_STALE_AFTER + 0.1
    backend.poll()
    s = backend_state(backend, BASE)
    assert not s["online"]
    assert "no status from servo bridge" in s["error"]
