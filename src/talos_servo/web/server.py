"""FastAPI app: WebSocket control channel, live telemetry broadcast and the static UI."""

from __future__ import annotations

import asyncio
import json
import logging
import secrets
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    MAX_ACCELERATION,
    MAX_POSITION,
    ServoError,
)
from talos_servo.defaults import DEFAULT_MOVE_SPEED
from talos_servo.robot import SOURCE_WEB, CommandError, clamp
from talos_servo.web.backend import RobotBackend

log = logging.getLogger(__name__)

# A laptop on bad Wi-Fi must not hold up telemetry for everyone else.
CLIENT_SEND_TIMEOUT = 0.5
# Bytes of randomness in a browser connection id: enough to tell a handful of laptops apart.
CLIENT_ID_BYTES = 3


class Hub:
    """Connected browsers. Broadcasts go to all of them concurrently."""

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

    Kept in memory only, on purpose: after a server restart they are back to the low defaults.
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


def _dispatch(
    controller: RobotBackend, msg: dict[str, Any], client: str | None = None
) -> Callable[[], None]:
    """Turn a client message into a controller call (validated, not yet run)."""
    kind = msg.get("type")
    who = {"source": SOURCE_WEB, "client": client}
    if kind == "stop_all":
        return lambda: controller.stop_all(**who)
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
    # Browsers get state at telemetry_hz; the robot ticks faster to keep wheels in sync.
    ticks_per_broadcast = max(1, round(config.control_hz / config.telemetry_hz))
    limits = {
        "pos_max": MAX_POSITION,
        "speed_max": controller.config.max_speed,
        "acc_max": MAX_ACCELERATION,
    }

    def state() -> dict[str, Any]:
        return {
            "type": "state",
            "ros": ros,
            "limits": limits,
            "settings": settings.as_message(),
            "servos": controller.snapshot(),
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

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        client_id = secrets.token_hex(CLIENT_ID_BYTES)
        hub.clients.add(ws)
        await hub.send(ws, {"type": "hello", "client_id": client_id})
        await hub.send(ws, state())
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    msg = json.loads(raw)
                    if msg.get("type") == "settings":
                        settings.update(msg)
                    else:
                        await _run(_dispatch(controller, msg, client_id))
                except (
                    CommandError,
                    ServoError,
                    ValueError,
                    KeyError,
                    TypeError,
                ) as exc:
                    await hub.send(ws, {"type": "error", "message": str(exc)})
                await hub.broadcast(state())
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(ws)
            if not hub.clients:
                log.warning(
                    "last client disconnected, stopping every speed joint"
                )
                # Shielded: the connection task can be cancelled right after a
                # disconnect, and the motors must still stop.
                # No client id: the UI shows this as the web server's own safety stop.
                await asyncio.shield(
                    _run(lambda: controller.stop_all(source=SOURCE_WEB))
                )

    if ui_dir is not None and ui_dir.is_dir():
        app.mount("/", StaticFiles(directory=ui_dir, html=True), name="ui")
    else:

        @app.get("/")
        async def no_ui() -> dict[str, str]:
            return {
                "error": f"UI not built: run `npm run build` in web/ (looked in {ui_dir})"
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
    """Build the backend (serial bus or ROS robot node) and serve until Ctrl+C."""
    import uvicorn

    from talos_servo.config import load_config

    config = load_config(config_path)
    backend: RobotBackend
    if ros:
        from talos_servo.ros.web_backend import RosBackend

        backend, source = RosBackend(config), "ros robot node"
    else:
        from talos_servo.bus import ServoBus
        from talos_servo.robot import Robot

        if not port:
            raise ValueError("no serial port given")
        backend, source = Robot(ServoBus(port, baudrate), config), port
    app = create_app(backend, ui_dir=Path(ui_dir), ros=ros)
    print(f"talos-servo web: http://{host}:{http_port} ({source})")
    uvicorn.run(app, host=host, port=http_port, log_level="warning")
