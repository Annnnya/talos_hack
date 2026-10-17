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

from talos_servo.bus import DEFAULT_ACCELERATION, MAX_ACCELERATION, MAX_POSITION, ServoError
from talos_servo.web.backend import DEFAULT_BRIDGE, RobotBackend
from talos_servo.web.controller import SOURCE_WEB, CommandError, RobotController, _clamp

log = logging.getLogger(__name__)

# A laptop on bad Wi-Fi must not hold up telemetry for everyone else.
CLIENT_SEND_TIMEOUT = 0.5
DEFAULT_MOVE_SPEED = 200
# Bytes of randomness in a browser connection id: enough to tell a handful of laptops apart.
CLIENT_ID_BYTES = 3


class Hub:
    """Connected browsers. Broadcasts go to all of them concurrently."""

    def __init__(self) -> None:
        self.clients: set[WebSocket] = set()

    async def send(self, ws: WebSocket, message: dict[str, Any]) -> None:
        try:
            await asyncio.wait_for(ws.send_text(json.dumps(message)), CLIENT_SEND_TIMEOUT)
        except Exception:
            self.clients.discard(ws)

    async def broadcast(self, message: dict[str, Any]) -> None:
        await asyncio.gather(*(self.send(ws, message) for ws in list(self.clients)))


class ServoSettings:
    """Move speed and acceleration per servo, shared by every browser.

    Kept in memory only, on purpose: after a server restart they are back to the low defaults.
    """

    def __init__(self, servo_ids: list[int], max_speed: int) -> None:
        self.max_speed = max_speed
        default_speed = _clamp(DEFAULT_MOVE_SPEED, 1, max_speed)
        self.values = {
            i: {"move_speed": default_speed, "acc": DEFAULT_ACCELERATION} for i in servo_ids
        }

    def update(self, msg: dict[str, Any]) -> None:
        servo_id = int(msg["id"])
        entry = self.values.get(servo_id)
        if entry is None:
            raise CommandError(f"unknown servo id {servo_id}")
        if "move_speed" in msg:
            entry["move_speed"] = _clamp(int(msg["move_speed"]), 1, self.max_speed)
        if "acc" in msg:
            entry["acc"] = _clamp(int(msg["acc"]), 0, MAX_ACCELERATION)

    def as_message(self) -> dict[str, dict[str, int]]:
        return {str(i): dict(v) for i, v in self.values.items()}


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
    servo_id = int(msg["id"])
    acc = int(msg.get("acc", DEFAULT_ACCELERATION))
    if kind == "stop":
        return lambda: controller.stop(servo_id, **who)
    if kind == "set_mode":
        return lambda: controller.set_mode(servo_id, str(msg["mode"]))
    if kind == "torque":
        return lambda: controller.set_torque(servo_id, bool(msg["enabled"]))
    if kind == "move":
        position, speed = int(msg["position"]), int(msg.get("speed", DEFAULT_MOVE_SPEED))
        return lambda: controller.move(servo_id, position, speed, acc, **who)
    if kind == "spin":
        speed = int(msg["speed"])
        return lambda: controller.spin(servo_id, speed, acc, **who)
    raise CommandError(f"unknown command {kind!r}")


def create_app(
    controller: RobotBackend,
    *,
    ui_dir: Path | None = None,
    mock: bool = False,
    ros: bool = False,
) -> FastAPI:
    hub = Hub()
    settings = ServoSettings([s.id for s in controller.config.servos], controller.config.max_speed)
    period = 1.0 / controller.config.telemetry_hz
    limits = {
        "pos_max": MAX_POSITION,
        "speed_max": controller.config.max_speed,
        "acc_max": MAX_ACCELERATION,
    }

    def state() -> dict[str, Any]:
        return {
            "type": "state",
            "mock": mock,
            "ros": ros,
            "limits": limits,
            "settings": settings.as_message(),
            "servos": controller.snapshot(),
        }

    async def telemetry() -> None:
        while True:
            try:
                await _run(controller.poll)
            except Exception:
                log.exception("telemetry poll failed")
            if hub.clients:
                await hub.broadcast(state())
            await asyncio.sleep(period)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        await _run(controller.open)
        task = asyncio.create_task(telemetry())
        try:
            yield
        finally:
            task.cancel()
            # Stops every spinning servo and closes the port, even on Ctrl+C.
            await _run(controller.close)

    app = FastAPI(title="talos-servo web", lifespan=lifespan)

    @app.get("/api/health")
    async def health() -> dict[str, Any]:
        return {"ok": True, "mock": mock, "clients": len(hub.clients)}

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
                except (CommandError, ServoError, ValueError, KeyError, TypeError) as exc:
                    await hub.send(ws, {"type": "error", "message": str(exc)})
                await hub.broadcast(state())
        except WebSocketDisconnect:
            pass
        finally:
            hub.clients.discard(ws)
            if not hub.clients:
                log.warning("last client disconnected, stopping wheel-mode servos")
                # Shielded: the connection task can be cancelled right after a
                # disconnect, and the motors must still stop.
                # No client id: the UI shows this as the web server's own safety stop.
                await asyncio.shield(_run(lambda: controller.stop_all(source=SOURCE_WEB)))

    if ui_dir is not None and ui_dir.is_dir():
        app.mount("/", StaticFiles(directory=ui_dir, html=True), name="ui")
    else:

        @app.get("/")
        async def no_ui() -> dict[str, str]:
            return {"error": f"UI not built: run `npm run build` in web/ (looked in {ui_dir})"}

    return app


def serve(
    *,
    config_path: str | Path,
    port: str | None,
    baudrate: int,
    mock: bool,
    host: str,
    http_port: int,
    ui_dir: str | Path,
    ros: bool = False,
    bridge: str = DEFAULT_BRIDGE,
) -> None:
    """Build the backend (serial bus, simulator or ROS bridge) and serve until Ctrl+C."""
    import uvicorn

    from talos_servo.web.config import load_config

    config = load_config(config_path)
    backend: RobotBackend
    if ros:
        from talos_servo.ros.web_backend import RosBackend

        backend, source = RosBackend(config, bridge=bridge), f"ros bridge {bridge}"
    else:
        from talos_servo.web.sim import make_bus

        bus = make_bus(port, baudrate, mock=mock, ids=(s.id for s in config.servos))
        backend, source = RobotController(bus, config), "simulated" if mock else str(port)
    app = create_app(backend, ui_dir=Path(ui_dir), mock=mock, ros=ros)
    print(f"talos-servo web: http://{host}:{http_port} ({source})")
    uvicorn.run(app, host=host, port=http_port, log_level="warning")
