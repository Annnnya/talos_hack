"""Web server against a Robot on the fake SDK, plus the `web` CLI command."""

from __future__ import annotations

import asyncio
import math
import sys
import types

import pytest
from conftest import BASE, SMALL, config, wait_for
from fastapi.testclient import TestClient

from talos_servo import cli
from talos_servo.modes import RobotStatus
from talos_servo.robot import CommandError, Robot
from talos_servo.web import server
from talos_servo.web.server import create_app


@pytest.fixture
def robot(sdk) -> Robot:
    return Robot(sdk.bus(), config(SMALL))


@pytest.fixture
def client(robot):
    with TestClient(create_app(robot)) as c:
        yield c


def receive_until(ws, predicate, limit: int = 200) -> dict:
    for _ in range(limit):
        msg = ws.receive_json()
        if predicate(msg):
            return msg
    raise AssertionError("expected message never arrived")


def joint(msg: dict, name: str) -> dict:
    return next(s for s in msg["servos"] if s["name"] == name)


# --- websocket server
# ----------------------------------------------------------


def test_initial_state_lists_enabled_joints(client):
    with client.websocket_connect("/ws") as ws:
        hello = ws.receive_json()
        msg = ws.receive_json()
    assert hello["type"] == "hello" and hello["client_id"]
    assert msg["type"] == "state"
    assert [s["name"] for s in msg["servos"]] == ["bend", "wheel"]
    assert [s["control"] for s in msg["servos"]] == ["position", "speed"]
    assert msg["limits"]["speed_max"] == 500
    # SMALL has no base, so there is nothing to drive.
    assert msg["limits"]["linear_max"] is None
    assert msg["limits"]["angular_max"] is None
    assert msg["ros"] is False
    # No twist_mux on the direct bus, so no STOP lock to report.
    assert msg["stopped"] is None
    assert msg["camera_streaming"] is None
    assert msg["camera_frame_sequence"] is None
    # No mode node or twist_mux either.
    assert (msg["mode"], msg["mode_reason"], msg["active_input"]) == (
        None,
        None,
        None,
    )
    assert all(s["last_command"] is None for s in msg["servos"])


def test_camera_frame_api_returns_jpeg(client, robot, monkeypatch):
    frame = b"\xff\xd8camera-frame\xff\xd9"
    monkeypatch.setattr(robot, "camera_frame", lambda: frame)
    response = client.get("/api/camera/frame.jpg?frame=1")
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/jpeg"
    assert response.headers["cache-control"] == "no-store"
    assert response.content == frame


def test_camera_frame_api_is_unavailable_without_a_frame(client):
    response = client.get("/api/camera/frame.jpg")
    assert response.status_code == 404


def test_each_browser_gets_its_own_client_id(client):
    with (
        client.websocket_connect("/ws") as a,
        client.websocket_connect("/ws") as b,
    ):
        assert a.receive_json()["client_id"] != b.receive_json()["client_id"]


def test_move_reaches_the_bus_and_records_who_sent_it(client, sdk):
    with client.websocket_connect("/ws") as ws:
        me = ws.receive_json()["client_id"]
        ws.send_json(
            {
                "type": "move",
                "joint": "bend",
                "position": 2000,
                "speed": 100,
                "acc": 10,
            }
        )
        msg = receive_until(
            ws,
            lambda m: m["type"] == "state" and joint(m, "bend")["last_command"],
        )
    last = joint(msg, "bend")["last_command"]
    assert (last["source"], last["client"]) == ("web", me)
    assert ("move", 1, 2000, 100, 10) in sdk.handler.calls


def test_settings_are_clamped_shared_and_kept_across_reconnects(client):
    with (
        client.websocket_connect("/ws") as a,
        client.websocket_connect("/ws") as b,
    ):
        a.receive_json()
        b.receive_json()
        first = receive_until(b, lambda m: m["type"] == "state")
        assert first["settings"]["bend"] == {"move_speed": 200, "acc": 20}
        a.send_json(
            {
                "type": "settings",
                "joint": "bend",
                "move_speed": 9999,
                "acc": 30,
            }
        )
        msg = receive_until(
            b,
            lambda m: (
                m["type"] == "state" and m["settings"]["bend"]["acc"] == 30
            ),
        )
        assert msg["settings"]["bend"]["move_speed"] == 500  # max_speed
    with client.websocket_connect("/ws") as c:
        c.receive_json()
        assert c.receive_json()["settings"]["bend"] == {
            "move_speed": 500,
            "acc": 30,
        }


def test_settings_for_unknown_joint_is_an_error(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "settings", "joint": "nope", "acc": 10})
        msg = receive_until(ws, lambda m: m["type"] == "error")
    assert "unknown joint 'nope'" in msg["message"]


def test_spinning_a_position_joint_returns_an_error(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "spin", "joint": "bend", "speed": 100})
        msg = receive_until(ws, lambda m: m["type"] == "error")
    assert "position joint" in msg["message"]


def test_spin_keeps_going_until_stopped(client, robot):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "spin", "joint": "wheel", "speed": 200})
        receive_until(
            ws,
            lambda m: (
                m["type"] == "state"
                and joint(m, "wheel")["target_speed"] == 200
            ),
        )
        ws.send_json({"type": "stop_all"})
        wait_for(lambda: robot.joints["wheel"].target_speed == 0)


def test_last_client_leaving_stops_spinning_joints(client, robot):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "spin", "joint": "wheel", "speed": 200})
        receive_until(
            ws,
            lambda m: (
                m["type"] == "state"
                and joint(m, "wheel")["target_speed"] == 200
            ),
        )
    wait_for(lambda: robot.joints["wheel"].target_speed == 0)


def test_last_client_stop_survives_connection_task_cancel(
    client, robot, monkeypatch
):
    # The test client cancels the server task right after disconnect. Delaying
    # the bus call makes that cancel land before the stop starts, which must not
    # skip it.
    real_run = server._run

    async def slow_run(fn, *args):
        await asyncio.sleep(0.05)
        return await real_run(fn, *args)

    monkeypatch.setattr(server, "_run", slow_run)
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "spin", "joint": "wheel", "speed": 200})
        receive_until(
            ws,
            lambda m: (
                m["type"] == "state"
                and joint(m, "wheel")["target_speed"] == 200
            ),
        )
    wait_for(lambda: robot.joints["wheel"].target_speed == 0)


def test_last_client_leaving_stops_only_what_the_web_commanded(
    client, robot, monkeypatch
):
    calls = []
    monkeypatch.setattr(robot, "stop_own", lambda **who: calls.append(who))
    monkeypatch.setattr(
        robot, "stop_all", lambda **who: pytest.fail("stop_all called")
    )
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
    wait_for(lambda: calls)
    assert calls == [{"source": "web"}]


# --- drive
# ------------------------------------------------------------------------


class DriveRecorder:
    """Just enough backend for `_dispatch` to call `drive` on."""

    def __init__(self, mode: str | None = None) -> None:
        self.calls: list[tuple[tuple[float, ...], dict]] = []
        self.mode = mode

    def drive(self, *velocity: float, **who: object) -> None:
        self.calls.append((velocity, who))

    def robot_status(self) -> RobotStatus:
        return RobotStatus(self.mode, None, None)


def drive(**velocity: object) -> dict:
    return {"type": "drive", "vx": 0.0, "vy": 0.0, "wz": 0.0} | velocity


def test_drive_passes_velocity_and_sender():
    backend = DriveRecorder()
    msg = drive(vx=0.1, vy=-0.2, wz=0.3)
    server._dispatch(backend, msg, "abc", ros=True)()
    assert backend.calls == [
        ((0.1, -0.2, 0.3), {"source": "web", "client": "abc"})
    ]


@pytest.mark.parametrize(
    ("msg", "error"),
    [
        ({"type": "drive", "vy": 0.0, "wz": 0.0}, KeyError),
        (drive(vx="fast"), ValueError),
        (drive(vy=None), TypeError),
        (drive(wz=math.nan), CommandError),
        (drive(vx=math.inf), CommandError),
        (drive(vy=True), CommandError),
    ],
)
def test_bad_drive_is_rejected(msg, error):
    backend = DriveRecorder()
    with pytest.raises(error):
        server._dispatch(backend, msg, ros=True)
    assert backend.calls == []


def test_rotating_while_climbing_is_rejected():
    backend = DriveRecorder(mode="climbing")
    with pytest.raises(CommandError, match="climbing"):
        server._dispatch(backend, drive(wz=0.3), ros=True)
    assert backend.calls == []


@pytest.mark.parametrize("mode", ["aligning", "clamping"])
@pytest.mark.parametrize("axis", ["vx", "vy", "wz"])
def test_driving_while_aligning_or_clamping_is_rejected(mode, axis):
    # Like the keyboard: these modes move the robot by themselves.
    backend = DriveRecorder(mode=mode)
    with pytest.raises(CommandError, match=f"cannot drive while {mode}"):
        server._dispatch(backend, drive(**{axis: 0.1}), ros=True)
    assert backend.calls == []


@pytest.mark.parametrize("mode", ["free", "aligning", "clamping", "climbing"])
def test_a_zero_drive_passes_in_every_mode(mode):
    # The page sends one when a button is let go; it must always stop.
    backend = DriveRecorder(mode=mode)
    server._dispatch(backend, drive(), ros=True)()
    assert [velocity for velocity, _ in backend.calls] == [(0.0, 0.0, 0.0)]


@pytest.mark.parametrize("mode", ["climbing", "free", None])
def test_straight_drive_is_allowed_in_free_and_climbing(mode):
    backend = DriveRecorder(mode=mode)
    server._dispatch(backend, drive(vx=0.1, vy=0.1), ros=True)()
    assert [velocity for velocity, _ in backend.calls] == [(0.1, 0.1, 0.0)]


def test_rotating_in_free_is_allowed():
    backend = DriveRecorder(mode="free")
    server._dispatch(backend, drive(wz=0.3), ros=True)()
    assert [velocity for velocity, _ in backend.calls] == [(0.0, 0.0, 0.3)]


def test_drive_without_ros_is_an_error(client, robot):
    # The base is shared through twist_mux, so there is no direct drive path.
    with client.websocket_connect("/ws") as ws:
        ws.send_json(drive(vx=0.1))
        error = receive_until(ws, lambda m: m["type"] == "error")
    assert "--ros" in error["message"]
    assert robot.joints["wheel"].target_speed == 0


def test_release_stop_without_ros_is_an_error(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "release_stop"})
        error = receive_until(ws, lambda m: m["type"] == "error")
    assert "--ros" in error["message"]


def test_release_stop_reaches_the_backend():
    calls = []

    class Recorder:
        def release_stop(self, **who):
            calls.append(who)

    server._dispatch(Recorder(), {"type": "release_stop"}, "abc", ros=True)()
    assert calls == [{"source": "web", "client": "abc"}]


def test_camera_stream_reaches_the_backend():
    calls = []

    class CameraRecorder:
        def camera_stream(self, enabled):
            calls.append(enabled)

    server._dispatch(
        CameraRecorder(),
        {"type": "camera_stream", "enabled": True},
        ros=True,
    )()
    assert calls == [True]


def test_camera_stream_requires_ros(client):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": "camera_stream", "enabled": True})
        error = receive_until(ws, lambda m: m["type"] == "error")
    assert "--ros" in error["message"]


def test_camera_stream_rejects_non_boolean_enabled():
    with pytest.raises(CommandError, match="enabled must be a boolean"):
        server._dispatch(
            object(),
            {"type": "camera_stream", "enabled": "false"},
            ros=True,
        )


class ModeRecorder:
    """Just enough backend for the mode commands."""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def wall_nearby(self) -> None:
        self.calls.append("wall_nearby")

    def release_walls(self) -> None:
        self.calls.append("release_walls")


@pytest.mark.parametrize(
    ("kind", "method"),
    [("wall_nearby", "wall_nearby"), ("release", "release_walls")],
)
def test_mode_commands_reach_the_backend(kind, method):
    backend = ModeRecorder()
    command = server._dispatch(backend, {"type": kind}, "abc", ros=True)
    assert backend.calls == []
    command()
    assert backend.calls == [method]


@pytest.mark.parametrize("kind", ["wall_nearby", "release"])
def test_mode_commands_without_ros_are_an_error(client, kind):
    with client.websocket_connect("/ws") as ws:
        ws.send_json({"type": kind})
        error = receive_until(ws, lambda m: m["type"] == "error")
    assert "--ros" in error["message"]


def test_state_reports_the_robot_status(robot, monkeypatch):
    status = RobotStatus("climbing", "wall contact", "keyboard")
    monkeypatch.setattr(robot, "robot_status", lambda: status)
    with (
        TestClient(create_app(robot)) as client,
        client.websocket_connect("/ws") as ws,
    ):
        ws.receive_json()
        msg = ws.receive_json()
    assert msg["mode"] == "climbing"
    assert msg["mode_reason"] == "wall contact"
    assert msg["active_input"] == "keyboard"


def test_a_held_bad_drive_reports_its_error_once(client):
    # The browser re-sends drive every 100 ms; one error is enough.
    with client.websocket_connect("/ws") as ws:
        for _ in range(3):
            ws.send_json(drive(vx=0.1))
        # A different error marks the end of the drive replies.
        ws.send_json({"type": "stop", "joint": "nope"})
        errors = []
        while not errors or "--ros" in errors[-1]:
            msg = receive_until(ws, lambda m: m["type"] == "error")
            errors.append(msg["message"])
    assert sum("--ros" in e for e in errors) == 1


def test_state_reports_the_base_speed_limits(sdk):
    robot = Robot(sdk.bus(), config(BASE))
    with (
        TestClient(create_app(robot)) as client,
        client.websocket_connect("/ws") as ws,
    ):
        ws.receive_json()
        limits = ws.receive_json()["limits"]
    assert (limits["linear_max"], limits["angular_max"]) == (0.2, 1.0)


def test_server_shutdown_closes_the_port(sdk, robot):
    with TestClient(create_app(robot)):
        pass
    assert sdk.port.closed_count == 1


# --- static ui
# -------------------------------------------------------------------


def test_ui_serves_home_and_debug_pages(robot, tmp_path):
    # Next exports with trailingSlash, so each page is `<route>/index.html`.
    (tmp_path / "index.html").write_text("home page")
    (tmp_path / "debug").mkdir()
    (tmp_path / "debug" / "index.html").write_text("debug page")
    with TestClient(create_app(robot, ui_dir=tmp_path)) as c:
        home = c.get("/")
        debug = c.get("/debug")
    assert home.status_code == 200 and home.text == "home page"
    assert debug.status_code == 200 and debug.text == "debug page"


def test_debug_redirects_to_its_folder(robot, tmp_path):
    # Why the export needs trailingSlash: StaticFiles(html=True) only finds
    # `debug/index.html`, reached through a redirect to `/debug/`.
    (tmp_path / "index.html").write_text("home page")
    (tmp_path / "debug").mkdir()
    (tmp_path / "debug" / "index.html").write_text("debug page")
    with TestClient(create_app(robot, ui_dir=tmp_path)) as c:
        response = c.get("/debug", follow_redirects=False)
    assert response.status_code == 307
    assert response.headers["location"].endswith("/debug/")


def test_debug_html_without_trailing_slash_is_not_found(robot, tmp_path):
    # The default Next export (`debug.html`) would 404 at `/debug`.
    (tmp_path / "index.html").write_text("home page")
    (tmp_path / "debug.html").write_text("debug page")
    with TestClient(create_app(robot, ui_dir=tmp_path)) as c:
        assert c.get("/debug").status_code == 404


# --- cli
# -------------------------------------------------------------------------


def test_web_command_needs_a_port(monkeypatch, capsys):
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    with pytest.raises(SystemExit) as exc:
        cli.main(["web"])
    assert exc.value.code == 2
    assert "TALOS_SERVO_PORT" in capsys.readouterr().err


def test_web_command_ros_runs_without_port(monkeypatch):
    seen = {}
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    monkeypatch.setitem(
        sys.modules, "talos_servo.ros.web_backend", types.ModuleType("stub")
    )
    monkeypatch.setattr(
        "talos_servo.web.server.serve", lambda **kw: seen.update(kw)
    )
    assert cli.main(["web", "--ros"]) == 0
    assert seen["ros"] is True
    assert seen["port"] is None


def test_web_command_ros_without_ros_installed_explains(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, "talos_servo.ros.web_backend", None)
    assert cli.main(["web", "--ros"]) == 1
    assert "pixi run -e ros" in capsys.readouterr().err


# --- hinge
# ------------------------------------------------------------------------


class HingeRecorder(DriveRecorder):
    def move_hinge(self, rate: float) -> None:
        self.calls.append(((rate,), {}))


def test_hinge_rate_reaches_the_backend():
    backend = HingeRecorder("free")
    server._dispatch(backend, {"type": "hinge", "rate": 0.2}, ros=True)()
    assert backend.calls == [((0.2,), {})]


def test_hinge_is_refused_while_climbing_but_stops():
    backend = HingeRecorder("climbing")
    with pytest.raises(CommandError, match="hinge"):
        server._dispatch(backend, {"type": "hinge", "rate": 0.2}, ros=True)
    server._dispatch(backend, {"type": "hinge", "rate": 0}, ros=True)()
    assert backend.calls == [((0.0,), {})]


def test_hinge_needs_ros():
    with pytest.raises(CommandError, match="--ros"):
        server._dispatch(HingeRecorder(), {"type": "hinge", "rate": 0.1})


def test_state_reports_the_platform_tilt(robot, monkeypatch):
    # The direct Robot has no IMU; a ROS backend fills it in (#91).
    with (
        TestClient(create_app(robot)) as client,
        client.websocket_connect("/ws") as ws,
    ):
        ws.receive_json()
        assert ws.receive_json()["tilt"] is None
    monkeypatch.setattr(
        robot, "tilt", lambda: {"roll_deg": -1.5, "pitch_deg": 2.0}
    )
    with (
        TestClient(create_app(robot)) as client,
        client.websocket_connect("/ws") as ws,
    ):
        ws.receive_json()
        assert ws.receive_json()["tilt"] == {"roll_deg": -1.5, "pitch_deg": 2.0}
