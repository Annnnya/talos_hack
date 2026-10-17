"""FastAPI app: WebSocket control channel, live telemetry broadcast and the static UI."""

from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import Callable
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from talos_servo.bus import DEFAULT_ACCELERATION, MAX_ACCELERATION, MAX_POSITION, ServoError
from talos_servo.controller import CommandError, RobotController
from talos_servo.defaults import DEFAULT_BRIDGE
from talos_servo.web.backend import RobotBackend

log = logging.getLogger(__name__)

# A laptop on bad Wi-Fi must not hold up telemetry for everyone else.
CLIENT_SEND_TIMEOUT = 0.5
DEFAULT_MOVE_SPEED = 200


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


async def _run(fn: Callable[..., Any], *args: Any) -> Any:
    """Run a blocking bus call off the event loop."""
    return await asyncio.get_running_loop().run_in_executor(None, fn, *args)


def _dispatch(controller: RobotBackend, msg: dict[str, Any]) -> Callable[[], None]:
    """Turn a client message into a controller call (validated, not yet run)."""
    kind = msg.get("type")
    if kind == "stop_all":
        return controller.stop_all
    servo_id = int(msg["id"])
    acc = int(msg.get("acc", DEFAULT_ACCELERATION))
    if kind == "stop":
        return lambda: controller.stop(servo_id)
    if kind == "set_mode":
        return lambda: controller.set_mode(servo_id, str(msg["mode"]))
    if kind == "torque":
        return lambda: controller.set_torque(servo_id, bool(msg["enabled"]))
    if kind == "move":
        position, speed = int(msg["position"]), int(msg.get("speed", DEFAULT_MOVE_SPEED))
        return lambda: controller.move(servo_id, position, speed, acc)
    if kind == "spin":
        speed = int(msg["speed"])
        return lambda: controller.spin(servo_id, speed, acc)
    raise CommandError(f"unknown command {kind!r}")


def create_app(
    controller: RobotBackend, *, ui_dir: Path | None = None, mock: bool = False
) -> FastAPI:
    hub = Hub()
    period = 1.0 / controller.config.telemetry_hz
    limits = {
        "pos_max": MAX_POSITION,
        "speed_max": controller.config.max_speed,
        "acc_max": MAX_ACCELERATION,
    }

    def state() -> dict[str, Any]:
        return {"type": "state", "mock": mock, "limits": limits, "servos": controller.snapshot()}

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
        hub.clients.add(ws)
        await hub.send(ws, state())
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    await _run(_dispatch(controller, json.loads(raw)))
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
                await asyncio.shield(_run(controller.stop_all))

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

    from talos_servo.config import load_config

    config = load_config(config_path)
    backend: RobotBackend
    if ros:
        from talos_servo.ros.web_backend import RosBackend

        backend, source = RosBackend(config, bridge=bridge), f"ros bridge {bridge}"
    else:
        from talos_servo.sim import make_bus

        bus = make_bus(port, baudrate, mock=mock, ids=(s.id for s in config.servos))
        backend, source = RobotController(bus, config), "simulated" if mock else str(port)
    app = create_app(backend, ui_dir=Path(ui_dir), mock=mock)
    print(f"talos-servo web: http://{host}:{http_port} ({source})")
    uvicorn.run(app, host=host, port=http_port, log_level="warning")
