from __future__ import annotations

import asyncio
import sys
import time
import types

import pytest
from fastapi.testclient import TestClient
from scservo_sdk import COMM_RX_TIMEOUT, SMS_STS_MODE

from talos_servo import cli
from talos_servo.bus import MODE_POSITION, MODE_WHEEL, ServoBus
from talos_servo.web import server
from talos_servo.web.config import ConfigError, parse_config
from talos_servo.web.controller import CommandError, RobotController
from talos_servo.web.server import create_app
from talos_servo.web.sim import sim_factories

CONFIG = parse_config(
    {
        "max_speed": 500,
        "telemetry_hz": 50,
        "servos": [
            {"id": 1, "name": "base", "min_pos": 1000, "max_pos": 3000},
            {"id": 2, "name": "wheel"},
        ],
    }
)


@pytest.fixture
def controller(sdk) -> RobotController:
    return RobotController(sdk.bus(), CONFIG)


@pytest.fixture
def client(controller):
    with TestClient(create_app(controller)) as c:
        yield c


def wait_for(predicate, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def receive_until(ws, predicate, limit: int = 200) -> dict:
    for _ in range(limit):
        msg = ws.receive_json()
        if predicate(msg):
            return msg
    raise AssertionError("expected message never arrived")


def servo(msg: dict, servo_id: int) -> dict:
    return next(s for s in msg["servos"] if s["id"] == servo_id)


# --- config ------------------------------------------------------------------


@pytest.mark.parametrize(
    "data",
    [
        {"servos": []},
        {"servos": [{"id": 1}, {"id": 1}]},
        {"servos": [{"id": 1, "min_pos": 3000, "max_pos": 1000}]},
        {"servos": [{"id": 1}], "max_speed": 0},
    ],
)
def test_invalid_config_is_rejected(data):
    with pytest.raises(ConfigError):
        parse_config(data)


# --- controller ----------------------------------------------------------------


def test_move_is_clamped_to_joint_and_speed_limits(sdk, controller):
    controller.open()
    controller.move(1, 4000, 9999, 20)
    assert ("move", 1, 3000, 500, 20) in sdk.handler.calls
    assert controller.servos[1].target == 3000


def test_spin_in_position_mode_is_refused(sdk, controller):
    controller.open()
    with pytest.raises(CommandError, match="position mode"):
        controller.spin(2, 100)
    assert sdk.handler.speed_calls() == []


def test_switching_away_from_wheel_mode_stops_first(sdk, controller):
    controller.open()
    controller.set_mode(2, "wheel")
    controller.spin(2, 300)
    controller.set_mode(2, "position")
    calls = [c for c in sdk.handler.calls if c[0] in ("speed", "write1")]
    assert calls[-2:] == [("speed", 2, 0, 20), ("write1", 2, SMS_STS_MODE, MODE_POSITION)]


def test_unreachable_servo_is_marked_offline_and_others_keep_polling(sdk, controller):
    sdk.fail[("read", 2)] = (COMM_RX_TIMEOUT, 0)
    controller.open()
    controller.poll()
    assert controller.servos[1].online
    assert not controller.servos[2].online
    assert "timeout" in controller.servos[2].error


def test_close_stops_spinning_servos_and_closes_port(sdk, controller):
    controller.open()
    controller.set_mode(2, "wheel")
    controller.spin(2, 300)
    controller.close()
    assert sdk.handler.speed_calls()[-1] == ("speed", 2, 0, 20)
    assert sdk.port.closed_count == 1


def test_full_poll_keeps_wheel_mode_so_stop_all_still_stops(sdk, controller, monkeypatch):
    monkeypatch.setattr("talos_servo.web.controller.SLOW_POLL_EVERY", 1)
    controller.open()
    controller.set_mode(2, "wheel")
    controller.spin(2, 300)
    controller.poll()
    assert controller.servos[2].mode == "wheel"
    controller.stop_all()
    assert sdk.handler.speed_calls()[-1] == ("speed", 2, 0, 20)


def test_open_reads_current_mode_from_servo(sdk, controller):
    sdk.registers[SMS_STS_MODE] = MODE_WHEEL
    controller.open()
    assert controller.servos[1].mode == "wheel"


# --- websocket server ----------------------------------------------------------


def test_initial_state_lists_configured_servos(client):
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        msg = ws.receive_json()
    assert hello["type"] == "hello" and hello["client_id"]
    assert msg["type"] == "state"
    assert [s["name"] for s in msg["servos"]] == ["base", "wheel"]
    assert msg["limits"]["speed_max"] == 500
    assert msg["ros"] is False
    assert all(s["last_command"] is None for s in msg["servos"])


def test_each_browser_gets_its_own_client_id(client):
    with client.websocket_connect("/ws") as a, client.websocket_connect("/ws") as b:
        assert a.receive_json()["client_id"] != b.receive_json()["client_id"]


def test_move_records_who_sent_it(client):
    with client.websocket_connect("/ws") as ws:
        me = ws.receive_json()["client_id"]
        ws.send_json({"type": "move", "id": 1, "position": 2000, "speed": 100})
        msg = receive_until(ws, lambda m: m["type"] == "state" and servo(m, 1)["last_command"])
    last = servo(msg, 1)["last_command"]
    assert (last["source"], last["client"]) == ("web", me)
    assert 0 <= last["age"] < 1


def test_last_command_age_grows():
    clock = FakeClock()
    port_factory, handler_factory = sim_factories([1])
    bus = ServoBus("sim", port_factory=port_factory, handler_factory=handler_factory)
    controller = RobotController(bus, parse_config({"servos": [{"id": 1}]}), clock=clock)
    controller.open()
    controller.move(1, 2100, 100, source="cli")
    clock.now += 12.34
    assert controller.snapshot()[0]["last_command"] == {
        "source": "cli",
        "client": None,
        "age": 12.3,
    }
    controller.close()


def test_settings_are_clamped_shared_and_kept_across_reconnects(client):
    with client.websocket_connect("/ws") as a, client.websocket_connect("/ws") as b:
        a.receive_json()
        b.receive_json()
        assert receive_until(b, lambda m: m["type"] == "state")["settings"]["1"] == {
            "move_speed": 200,
            "acc": 20,
        }
        a.send_json({"type": "settings", "id": 1, "move_speed": 9999, "acc": 30})
        msg = receive_until(b, lambda m: m["type"] == "state" and m["settings"]["1"]["acc"] == 30)
        assert msg["settings"]["1"]["move_speed"] == 500  # capped by max_speed
    with client.websocket_connect("/ws") as c:
        c.receive_json()
        assert c.receive_json()["settings"]["1"] == {"move_speed": 500, "acc": 30}


def test_settings_for_unknown_servo_is_an_error(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "settings", "id": 9, "acc": 10})
        msg = receive_until(ws, lambda m: m["type"] == "error")
    assert "unknown servo id 9" in msg["message"]


def test_move_command_reaches_the_bus(client, sdk):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "move", "id": 1, "position": 2000, "speed": 100, "acc": 10})
        receive_until(ws, lambda m: m["type"] == "state" and servo(m, 1)["target"] == 2000)
    assert ("move", 1, 2000, 100, 10) in sdk.handler.calls


def test_bad_command_returns_error_message(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "spin", "id": 1, "speed": 100})
        msg = receive_until(ws, lambda m: m["type"] == "error")
    assert "position mode" in msg["message"]


def test_stop_all_command(client, sdk):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "set_mode", "id": 2, "mode": "wheel"})
        ws.send_json({"type": "spin", "id": 2, "speed": 200})
        ws.send_json({"type": "stop_all"})
        receive_until(ws, lambda m: m["type"] == "state" and servo(m, 2)["mode"] == "wheel")
        wait_for(lambda: sdk.handler.speed_calls()[-1:] == [("speed", 2, 0, 20)])


def test_last_client_leaving_stops_spinning_servos(client, sdk):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "set_mode", "id": 2, "mode": "wheel"})
        ws.send_json({"type": "spin", "id": 2, "speed": 200})
        receive_until(ws, lambda m: m["type"] == "state" and servo(m, 2)["target_speed"] == 200)
    wait_for(lambda: sdk.handler.speed_calls()[-1] == ("speed", 2, 0, 20))


def test_last_client_stop_survives_connection_task_cancel(client, sdk, monkeypatch):
    # The test client cancels the server task right after disconnect. Delaying the bus
    # call makes that cancel land before the stop starts, which must not skip it.
    real_run = server._run

    async def slow_run(fn, *args):
        await asyncio.sleep(0.05)
        return await real_run(fn, *args)

    monkeypatch.setattr(server, "_run", slow_run)
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "set_mode", "id": 2, "mode": "wheel"})
        ws.send_json({"type": "spin", "id": 2, "speed": 200})
        receive_until(ws, lambda m: m["type"] == "state" and servo(m, 2)["target_speed"] == 200)
    wait_for(lambda: sdk.handler.speed_calls()[-1] == ("speed", 2, 0, 20))


# --- simulation ------------------------------------------------------------------


def test_simulated_servo_moves_toward_target():
    port_factory, handler_factory = sim_factories([1])
    bus = ServoBus("sim", port_factory=port_factory, handler_factory=handler_factory)
    controller = RobotController(bus, parse_config({"servos": [{"id": 1}]}))
    controller.open()
    start = controller.servos[1].position
    controller.move(1, start + 400, 500)
    time.sleep(0.2)
    controller.poll()
    assert start < controller.servos[1].position <= start + 400
    controller.close()


def test_simulated_spinning_servo_reports_current_and_moving():
    port_factory, handler_factory = sim_factories([1])
    with ServoBus("sim", port_factory=port_factory, handler_factory=handler_factory) as bus:
        idle = bus.read_status(1)
        assert not idle.moving
        assert idle.current == 0
        bus.set_wheel_mode(1)
        bus.set_speed(1, 1000)
        time.sleep(0.2)
        spinning = bus.read_status(1)
        assert spinning.moving
        assert spinning.current > 0


def test_simulated_unknown_id_is_offline():
    port_factory, handler_factory = sim_factories([1])
    bus = ServoBus("sim", port_factory=port_factory, handler_factory=handler_factory)
    controller = RobotController(bus, parse_config({"servos": [{"id": 1}, {"id": 9}]}))
    controller.open()
    assert controller.servos[1].online
    assert not controller.servos[9].online


# --- cli -------------------------------------------------------------------------


def test_web_command_needs_port_without_mock(monkeypatch, capsys):
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    with pytest.raises(SystemExit) as exc:
        cli.main(["web"])
    assert exc.value.code == 2
    assert "TALOS_SERVO_PORT" in capsys.readouterr().err


def test_web_command_mock_runs_without_port(monkeypatch):
    seen = {}
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    monkeypatch.setattr("talos_servo.web.server.serve", lambda **kw: seen.update(kw))
    assert cli.main(["web", "--mock"]) == 0
    assert seen["mock"] is True
    assert seen["port"] is None


def test_web_command_ros_runs_without_port(monkeypatch):
    seen = {}
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    monkeypatch.setitem(sys.modules, "talos_servo.ros.web_backend", types.ModuleType("stub"))
    monkeypatch.setattr("talos_servo.web.server.serve", lambda **kw: seen.update(kw))
    assert cli.main(["web", "--ros", "--bridge", "/arm_bridge"]) == 0
    assert seen["ros"] is True
    assert seen["bridge"] == "/arm_bridge"


def test_web_command_ros_without_ros_installed_explains(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "talos_servo.ros.web_backend", None)
    assert cli.main(["web", "--ros"]) == 1
    assert "pixi run -e ros" in capsys.readouterr().err


def test_web_command_rejects_mock_with_ros(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.main(["web", "--ros", "--mock"])
    assert exc.value.code == 2
    assert "servo-node-mock" in capsys.readouterr().err
