"""FastAPI app: WebSocket commands, live telemetry and the static UI."""

from __future__ import annotations

import asyncio
import json
import logging
import math
import secrets
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from fastapi.staticfiles import StaticFiles

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    MAX_ACCELERATION,
    MAX_POSITION,
    ServoError,
)
from talos_servo.defaults import DEFAULT_MOVE_SPEED, WEB_CMD_VEL_TOPIC
from talos_servo.modes import climb_error, drive_error, hinge_error
from talos_servo.robot import SOURCE_WEB, CommandError, clamp
from talos_servo.web.backend import RobotBackend

log = logging.getLogger(__name__)

# A laptop on bad Wi-Fi must not hold up telemetry for everyone else.
CLIENT_SEND_TIMEOUT = 0.5
# Bytes of randomness in a browser connection id: enough to tell a handful of
# laptops apart.
CLIENT_ID_BYTES = 3


class Hub:
    """Connected browsers.

    Broadcasts go to all of them concurrently.
    """

    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def send(self, ws: WebSocket, message: dict[str, Any]) -> None:
        try:
            await asyncio.wait_for(
                ws.send_text(json.dumps(message)), CLIENT_SEND_TIMEOUT
            )
        except Exception:
            self.clients.discard(ws)

    async def broadcast(self, message: dict[str, Any]) -> None:
        await asyncio.gather(
            *(self.send(ws, message) for ws in list(self.clients))
        )


class JointSettings:
    """Move speed and acceleration per joint, shared by every browser.

    Kept in memory only, on purpose: after a server restart they are back to the
    low defaults.
    """

    def __init__(self, joints: list[str], max_speed: int) -> None:
        self.max_speed = max_speed
        default_speed = clamp(DEFAULT_MOVE_SPEED, 1, max_speed)
        self.values = {
            j: {"move_speed": default_speed, "acc": DEFAULT_ACCELERATION}
            for j in joints
        }

    def update(self, msg: dict[str, Any]) -> None:
        joint = str(msg["joint"])
        entry = self.values.get(joint)
        if entry is None:
            raise CommandError(f"unknown joint {joint!r}")
        if "move_speed" in msg:
            entry["move_speed"] = clamp(
                int(msg["move_speed"]), 1, self.max_speed
            )
        if "acc" in msg:
            entry["acc"] = clamp(int(msg["acc"]), 0, MAX_ACCELERATION)

    def as_message(self) -> dict[str, dict[str, int]]:
        return {j: dict(v) for j, v in self.values.items()}


async def _run(fn: Callable[..., Any], *args: Any) -> Any:
    """Run a blocking bus call off the event loop."""
    return await asyncio.get_running_loop().run_in_executor(None, fn, *args)


# Client messages that change nothing the UI shows. The browser re-sends drive
# while a button is held, so a state broadcast per message would flood it.
QUIET_COMMANDS = frozenset({"drive", "hinge", "climb", "wall_side"})
# Client messages that ask the mode node for a mode change -> backend method.
MODE_COMMANDS = {"wall_nearby": "wall_nearby", "release": "release_walls"}


def _finite(msg: dict[str, Any], key: str) -> float:
    raw = msg[key]
    # float(True) is 1.0, a full-speed drive from a malformed message.
    if isinstance(raw, bool):
        raise CommandError(f"{key} must be a number, got {raw}")
    value = float(raw)
    if not math.isfinite(value):
        raise CommandError(f"{key} must be a finite number, got {value}")
    return value


def _dispatch(
    controller: RobotBackend,
    msg: dict[str, Any],
    client: str | None = None,
    *,
    ros: bool = False,
) -> Callable[[], None]:
    """Turn a client message into a controller call (validated, not yet run)."""
    kind = msg.get("type")
    who = {"source": SOURCE_WEB, "client": client}
    if kind == "stop_all":
        return lambda: controller.stop_all(**who)
    if kind == "release_stop":
        if not ros:
            raise CommandError("releasing STOP needs `talos-servo web --ros`")
        return lambda: controller.release_stop(**who)
    if kind == "drive":
        # The base is shared through twist_mux, so there is no direct path.
        if not ros:
            raise CommandError("driving the base needs `talos-servo web --ros`")
        vx, vy, wz = (_finite(msg, k) for k in ("vx", "vy", "wz"))
        # Same per-mode rule as the keyboard; the page greys the buttons
        # out too, but a stale page must not drive the wheels while clamping.
        error = drive_error(controller.robot_status().mode, vx, vy, wz)
        if error:
            raise CommandError(error)
        return lambda: controller.drive(vx, vy, wz, **who)
    if kind == "hinge":
        if not ros:
            raise CommandError("the hinge buttons need `talos-servo web --ros`")
        rate = _finite(msg, "rate")
        error = hinge_error(controller.robot_status().mode, rate)
        if error:
            raise CommandError(error)
        return lambda: controller.move_hinge(rate)
    if kind == "hinge_hold":
        if not ros:
            raise CommandError("the hinge hold needs `talos-servo web --ros`")
        enabled = bool(msg["enabled"])
        setpoint = _finite(msg, "setpoint")
        if setpoint < 0:
            raise CommandError(f"setpoint must be 0 or more, got {setpoint}")
        setpoint = min(setpoint, controller.config.hinge_hold.max_current_a)
        return lambda: controller.hinge_hold(enabled, setpoint)
    if kind == "camera_stream":
        if not ros:
            raise CommandError("camera streaming needs `talos-servo web --ros`")
        enabled = msg.get("enabled")
        if not isinstance(enabled, bool):
            raise CommandError("enabled must be a boolean")
        return lambda: controller.camera_stream(enabled)
    if kind == "wall_side":
        if not ros:
            raise CommandError(
                "moving on the wall needs `talos-servo web --ros`"
            )
        speed = _finite(msg, "speed")
        error = climb_error(controller.robot_status().mode, speed)
        if error:
            raise CommandError(error)
        return lambda: controller.wall_side(speed)
    if kind == "climb":
        if not ros:
            raise CommandError("climbing needs `talos-servo web --ros`")
        speed = _finite(msg, "speed")
        error = climb_error(controller.robot_status().mode, speed)
        if error:
            raise CommandError(error)
        return lambda: controller.climb(speed)
    if kind in MODE_COMMANDS:
        if not ros:
            raise CommandError(f"{kind} needs `talos-servo web --ros`")
        return getattr(controller, MODE_COMMANDS[kind])
    joint = str(msg["joint"])
    acc = int(msg.get("acc", DEFAULT_ACCELERATION))
    if kind == "stop":
        return lambda: controller.stop(joint, **who)
    if kind == "torque":
        return lambda: controller.set_torque(joint, bool(msg["enabled"]))
    if kind == "move":
        position, speed = (
            int(msg["position"]),
            int(msg.get("speed", DEFAULT_MOVE_SPEED)),
        )
        return lambda: controller.move(joint, position, speed, acc, **who)
    if kind == "spin":
        speed = int(msg["speed"])
        return lambda: controller.spin(joint, speed, acc, **who)
    raise CommandError(f"unknown command {kind!r}")


def create_app(
    controller: RobotBackend,
    *,
    ui_dir: Path | None = None,
    ros: bool = False,
) -> FastAPI:
    hub = Hub()
    config = controller.config
    settings = JointSettings(
        [j.name for j in config.enabled_joints], config.max_speed
    )
    period = 1.0 / config.control_hz
    # Browsers get state at telemetry_hz; the robot ticks faster to keep wheels
    # in sync.
    ticks_per_broadcast = max(1, round(config.control_hz / config.telemetry_hz))
    base = config.base
    limits = {
        "pos_max": MAX_POSITION,
        "speed_max": config.max_speed,
        "acc_max": MAX_ACCELERATION,
        # m/s and rad/s for drive; None when the config has no base.
        "linear_max": base.max_linear_speed if base else None,
        "angular_max": base.max_angular_speed if base else None,
        # m/s cap of the Up and Down buttons.
        "climb_max": config.expansion.max_climb_speed_m_s,
    }

    def state() -> dict[str, Any]:
        status = controller.robot_status()
        return {
            "type": "state",
            "ros": ros,
            "limits": limits,
            "settings": settings.as_message(),
            "servos": controller.snapshot(),
            # True while STOP locks twist_mux; None without ROS.
            "stopped": controller.stopped(),
            # From the mode node and mux_status; None until heard or
            # without ROS.
            "mode": status.mode,
            "mode_reason": status.mode_reason,
            "active_input": status.active_input,
            # Angle and raw servo step for calibration; None without ROS.
            "hinge": controller.hinge_state(),
            # Hinge current hold (#142); None without the expansion node.
            "hinge_hold": controller.hold_state(),
            # Platform roll and pitch from the IMU (#91); None without it.
            "tilt": controller.tilt(),
            # None until a camera node publishes its latched state.
            "camera_streaming": controller.camera_streaming(),
            "camera_frame_sequence": controller.camera_frame_sequence(),
        }

    async def control_loop() -> None:
        ticks = 0
        while True:
            try:
                await _run(controller.tick)
            except Exception:
                log.exception("robot tick failed")
            ticks += 1
            if hub.clients and ticks % ticks_per_broadcast == 0:
                await hub.broadcast(state())
            await asyncio.sleep(period)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        await _run(controller.open)
        task = asyncio.create_task(control_loop())
        try:
            yield
        finally:
            task.cancel()
            # Stops every speed joint and closes the port, even on Ctrl+C.
            await _run(controller.close)

    app = FastAPI(title="talos-servo web", lifespan=lifespan)

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        return {"ok": True, "clients": len(hub.clients)}

    @app.get("/api/camera/frame.jpg")
    async def camera_frame() -> Response:
        frame = controller.camera_frame()
        if frame is None:
            return Response(status_code=404)
        return Response(
            content=frame,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store"},
        )

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        client_id = secrets.token_hex(CLIENT_ID_BYTES)
        hub.clients.add(ws)
        await hub.send(ws, {"type": "hello", "client_id": client_id})
        await hub.send(ws, state())
        # The last error a quiet command got. A held button re-sends the same
        # bad drive every 100 ms, so report each error once until it changes.
        quiet_error: str | None = None
        try:
            while True:
                raw = await ws.receive_text()
                quiet = False
                try:
                    msg = json.loads(raw)
                    quiet = msg.get("type") in QUIET_COMMANDS
                    if msg.get("type") == "settings":
                        settings.update(msg)
                    else:
                        await _run(
                            _dispatch(controller, msg, client_id, ros=ros)
                        )
                    if quiet:
                        quiet_error = None
                except (
                    CommandError,
                    ServoError,
                    ValueError,
                    KeyError,
                    TypeError,
                ) as exc:
                    message = str(exc)
                    if not (quiet and message == quiet_error):
                        error = {"type": "error", "message": message}
                        await hub.send(ws, error)
                    if quiet:
                        quiet_error = message
                if not quiet:
                    await hub.broadcast(state())
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(ws)
            if not hub.clients:
                log.warning(
                    "last client disconnected, stopping what the web commanded"
                )
                # Shielded: the connection task can be cancelled right after a
                # disconnect, and the motors must still stop. No client id: the
                # UI shows this as the web server's own safety stop. Only our
                # own commands, so keyboard driving carries on.
                await asyncio.shield(
                    _run(lambda: controller.stop_own(source=SOURCE_WEB))
                )

    if ui_dir is not None and ui_dir.is_dir():
        app.mount("/", StaticFiles(directory=ui_dir, html=True), name="ui")
    else:

        @app.get("/")
        async def no_ui() -> dict[str, str]:
            return {
                "error": (
                    "UI not built: run `npm run build` in web/ (looked in "
                    f"{ui_dir})"
                )
            }

    return app


def serve(
    *,
    config_path: str | Path,
    port: str | None,
    baudrate: int,
    host: str,
    http_port: int,
    ui_dir: str | Path,
    ros: bool = False,
) -> None:
    """Build the backend (serial bus or ROS robot node) and serve until
    Ctrl+C."""
    import uvicorn

    from talos_servo.config import load_config

    config = load_config(config_path)
    backend: RobotBackend
    if ros:
        from talos_servo.ros.web_backend import RosBackend

        backend = RosBackend(config, cmd_vel_topic=WEB_CMD_VEL_TOPIC)
        source = "ros robot node"
    else:
        from talos_servo.bus import ServoBus
        from talos_servo.robot import Robot

        if not port:
            raise ValueError("no serial port given")
        backend, source = Robot(ServoBus(port, baudrate), config), port
    app = create_app(backend, ui_dir=Path(ui_dir), ros=ros)
    url = f"http://{host}:{http_port}"
    print(f"talos-servo web: {url} ({source}), per-motor page {url}/debug")
    uvicorn.run(app, host=host, port=http_port, log_level="warning")
