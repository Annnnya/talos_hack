# Feetech Servo Robot Control

Web UI to control the robot's 5 Feetech STS servos, each in **Position** or **Speed (wheel)** mode.

```
 laptop browser(s) ──http/ws──►  Raspberry Pi :8000  ──USB serial──►  servo bus (IDs 1-5)
                                 bridge/main.py (FastAPI)
                                 serves web/out (Next.js UI)
```

- `bridge/` – Python server that owns the serial port (via `scservo_sdk`), streams servo state at 20 Hz over a WebSocket, and serves the UI.
- `web/` – Next.js UI, built as static files into `web/out`.
- `tests/` – standalone hardware test scripts.

## Setup on the Pi

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r bridge/requirements.txt
```

The Feetech SDK with `sms_sts` is **not** the PyPI `feetech-servo-sdk` package. If the scripts in `tests/` already run on the Pi, it's installed. Otherwise:

```bash
git clone https://github.com/ftservo/FTServo_Python /tmp/ftservo
cp -r /tmp/ftservo/scservo_sdk bridge/
```

Build the UI (needs Node 20+, can be done on a laptop and copied over, or on the Pi):

```bash
cd web && npm install && npm run build   # writes web/out
```

Edit `bridge/config.yaml` for the serial port, servo IDs, names and position limits.

## Run

```bash
python bridge/main.py           # real hardware
python bridge/main.py --mock    # simulated servos, no hardware needed
```

Then from **any laptop on the same network** open:

```
http://<pi-hostname>.local:8000      e.g. http://raspberrypi.local:8000
```

(or use the Pi's IP address). Several laptops can connect at once; all of them see the same live state.

### Start automatically on boot

```bash
# edit User / paths in the file first
sudo cp bridge/robot-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-bridge
journalctl -u robot-bridge -f   # logs
```

## Using the UI

- **Position mode**: drag the target slider (moves live) or type a value and press Go. 0-4095 = one full turn. The amber marker shows the current position. *Move speed* 0 means maximum.
- **Speed mode**: centre of the slider is stop, left is reverse, right is forward.
- **STOP ALL** (or the Space key) stops every servo in speed mode.
- **Torque** off lets you move the joint by hand.

Safety: if every browser disconnects, the bridge stops all speed-mode servos. Position targets are clamped to `min_pos`/`max_pos` from `config.yaml`, so set real joint limits there before running on the robot.

## UI development

```bash
python bridge/main.py --mock                    # terminal 1
cd web && npm run dev                           # terminal 2 -> http://localhost:3000
# point dev UI at a real Pi instead:
NEXT_PUBLIC_BRIDGE_URL=ws://raspberrypi.local:8000/ws npm run dev
```

WebSocket protocol is documented in `web/lib/protocol.ts`.
