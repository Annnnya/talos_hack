#!/usr/bin/env python3
"""Robot bridge: owns the servo bus, serves the web UI and a WebSocket for control.

Run on the Pi:     python bridge/main.py
Run without hw:    python bridge/main.py --mock
Then open:         http://<pi-hostname>.local:8000
"""
from __future__ import annotations

import argparse
import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
import yaml
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from servo_bus import (ACC_MAX, MODE_POSITION, MODE_WHEEL, POS_MAX, SPEED_MAX,
                       BaseBus, MockServoBus, ServoBus)

HERE = Path(__file__).resolve().parent
WEB_OUT = HERE.parent / "web" / "out"


def parse_args():
    p = argparse.ArgumentParser(description="Servo control bridge")
    p.add_argument("--config", default=str(HERE / "config.yaml"))
    p.add_argument("--mock", action="store_true", help="simulate servos (no hardware)")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    return p.parse_args()


args = parse_args()
config = yaml.safe_load(Path(args.config).read_text())
telemetry_hz = float(config.get("telemetry_hz", 20))

bus: BaseBus = (
    MockServoBus(config["servos"])
    if args.mock
    else ServoBus(config["port"], int(config["baudrate"]), config["servos"])
)
clients: set[WebSocket] = set()


async def run_bus(fn, *a):
    """Run a blocking bus call off the event loop."""
    return await asyncio.get_running_loop().run_in_executor(None, fn, *a)


async def broadcast(message: dict):
    data = json.dumps(message)

    async def send_one(ws: WebSocket):
        # A stalled client (e.g. laptop on bad Wi-Fi) must not block the others.
        try:
            await asyncio.wait_for(ws.send_text(data), timeout=0.5)
        except Exception:
            clients.discard(ws)

    await asyncio.gather(*(send_one(ws) for ws in list(clients)))


def state_message() -> dict:
    return {
        "type": "state",
        "mock": args.mock,
        "limits": {"pos_max": POS_MAX, "speed_max": SPEED_MAX, "acc_max": ACC_MAX},
        "servos": bus.snapshot(),
    }


async def telemetry_loop():
    period = 1.0 / telemetry_hz
    while True:
        try:
            await run_bus(bus.poll)
        except Exception as exc:
            print("Telemetry error:", exc)
        if clients:
            await broadcast(state_message())
        await asyncio.sleep(period)


async def handle_command(msg: dict):
    kind = msg.get("type")
    sid = msg.get("id")
    if kind == "stop_all":
        await run_bus(bus.stop_all)
    elif kind == "stop":
        await run_bus(bus.stop, int(sid))
    elif kind == "set_mode":
        mode = msg.get("mode")
        if mode not in (MODE_POSITION, MODE_WHEEL):
            raise ValueError(f"Invalid mode {mode!r}")
        await run_bus(bus.set_mode, int(sid), mode)
    elif kind == "torque":
        await run_bus(bus.set_torque, int(sid), bool(msg.get("enabled")))
    elif kind == "move":
        await run_bus(bus.move, int(sid), int(msg["position"]),
                      int(msg.get("speed", 1000)), int(msg.get("acc", 50)))
    elif kind == "spin":
        await run_bus(bus.spin, int(sid), int(msg["speed"]), int(msg.get("acc", 50)))
    else:
        raise ValueError(f"Unknown command {kind!r}")


@asynccontextmanager
async def lifespan(_app: FastAPI):
    await run_bus(bus.open)
    task = asyncio.create_task(telemetry_loop())
    try:
        yield
    finally:
        task.cancel()
        await run_bus(bus.close)
        print("Bus closed, servos stopped.")


app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])


@app.get("/api/health")
async def health():
    return {"ok": True, "mock": args.mock, "clients": len(clients)}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps(state_message()))
    try:
        while True:
            raw = await ws.receive_text()
            try:
                await handle_command(json.loads(raw))
            except Exception as exc:
                await ws.send_text(json.dumps({"type": "error", "message": str(exc)}))
            await broadcast(state_message())
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)
        if not clients:
            # Safety: nobody is watching, so stop anything spinning.
            print("Last client disconnected: stopping wheel-mode servos")
            await run_bus(bus.stop_all)


# Serve the exported Next.js UI (built with `npm run build` in web/).
if WEB_OUT.exists():
    app.mount("/", StaticFiles(directory=WEB_OUT, html=True), name="ui")
else:
    @app.get("/")
    async def no_ui():
        return {"error": f"UI not built. Run `npm run build` in {WEB_OUT.parent}."}


if __name__ == "__main__":
    uvicorn.run(app, host=args.host, port=args.port)
