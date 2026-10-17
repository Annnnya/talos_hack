# talos_hack

Software and tests for the Talos robot from the physical AI hackathon. The robot uses Feetech
STS/SMS serial bus servos daisy-chained on one bus.

Contributing (humans and AI agents): read [AGENTS.md](AGENTS.md). Everything goes through PRs.

## Setup

Use either [uv](https://docs.astral.sh/uv/) or [pixi](https://pixi.sh). Both install the
official Feetech SDK (`ftservo-python-sdk`, imported as `scservo_sdk`) and the `talos-servo`
command from the same `pyproject.toml`.

```
uv sync          # pure Python, creates .venv
pixi install     # conda based, creates .pixi/
```

pixi is the path toward ROS 2: the workspace (`pixi.toml`) already pulls from the RoboStack
`robostack-jazzy` channel and pins Python 3.12 to match ROS 2 Jazzy. ROS packages are not added
yet. uv cannot install ROS, so ROS work will need pixi.

The examples below use `uv run`. With pixi, use `pixi run` instead (or open `pixi shell`).

When you change dependencies in `pyproject.toml`, refresh both lockfiles: `uv lock` and
`pixi install`. Conda-only packages (ROS) go in `pixi.toml` with `pixi add`.

## Usage

Find your adapter's port:

```
uv run talos-servo ports
```

Pass it with `--port` or set it once per shell:

```
# PowerShell
$env:TALOS_SERVO_PORT = "COM5"
# bash
export TALOS_SERVO_PORT=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00
```

| Command | What it does |
|---|---|
| `talos-servo ping --ids 1-3` | Check which servos answer and print their model number |
| `talos-servo spin --ids 1,2,3 --speed 100 --seconds 3` | Wheel mode + torque on, spin, then stop |
| `talos-servo monitor --ids 1` | Print position and speed every second until Ctrl+C |
| `talos-servo monitor --ids 1 --speed 500` | Same, while spinning (stops on exit) |
| `talos-servo set-id --id 1 --new-id 3` | Permanently change a servo's ID (refuses if 3 is taken) |

New servos ship with ID 1. To build a daisy chain, connect them one at a time and give each a
unique ID with `set-id` before chaining them.

Common options: `--baud` (default 1000000), `--acc` (acceleration, default 20), `-v` for debug logs.
Negative `--speed` spins the other way. Servos are always stopped on exit, including Ctrl+C and
errors.

The old scripts map to commands like this:

| Old script | Command |
|---|---|
| `tests/test_servo.py` | `talos-servo ping` |
| `tests/rotate_servo.py` | `talos-servo spin` |
| `tests/test_daisy_chain.py` | `talos-servo spin --ids 1-3 --speed 300 --seconds 60` |
| `tests/test_servo2.py` | `talos-servo monitor --ids 3 --speed 500 --acc 10` |
| `tests/change_ids.py` | `talos-servo set-id --id 1 --new-id 3` |

## IMU (MPU6050)

The MPU6050 accelerometer and gyro connects to the Raspberry Pi over I2C, separately from the
servo bus.

| MPU6050 | Pi header |
|---|---|
| VCC | pin 1 (3.3 V) |
| GND | pin 6 (GND) |
| SDA | pin 3 (GPIO2) |
| SCL | pin 5 (GPIO3) |
| AD0 | GND for address 0x68 (or 3.3 V for 0x69) |

Power it from 3.3 V: the Pi GPIO pins are not 5 V tolerant. Enable I2C with
`sudo raspi-config` (Interface Options, I2C), reboot, and check that `i2cdetect -y 1` shows `68`.

```
uv sync --extra imu                                # pixi install already includes it
uv run talos-servo imu --calibrate                 # zero the gyro, then stream until Ctrl+C
uv run talos-servo imu --seconds 5 --interval 0.05
```

Flat and still, it should read about `0 0 +1.00` g and near-zero gyro. Options: `--i2c-bus`
(default 1, or `$TALOS_IMU_BUS`), `--address` (default 0x68), `--accel-range` (2/4/8/16 g),
`--gyro-range` (250/500/1000/2000 deg/s).

```python
from talos_servo import Mpu6050

with Mpu6050() as imu:
    imu.calibrate_gyro()
    sample = imu.read()
    print(sample.accel, sample.gyro, sample.temperature)
```

## Using the library

```python
from talos_servo import ServoBus

with ServoBus("COM5") as bus:
    bus.enable_wheel_mode([1, 2])
    bus.set_speed(1, 100)
    print(bus.read_position_speed(1))
# every servo started above is stopped here, even if the block raised
```

## Development

```
uv run pytest           # tests use a fake SDK, no hardware needed
uv run ruff check .
uv run ruff format .
```

The same checks as pixi tasks:

```
pixi run test
pixi run lint
pixi run fmt
pixi run check   # lint + format check + tests, like CI
```

## Web UI

Browser control for the robot's servos: each one switches between **Position** and **Speed**
(wheel) mode, with live position, speed, load, voltage and temperature.

```
 laptop browser(s) --http/ws-->  Pi :8000  --USB serial-->  servo bus
                                 talos-servo web
                                 serves web/out (Next.js UI)
```

- `src/talos_servo/web/` - the `talos-servo web` command: FastAPI server on top of `ServoBus`
  that streams servo state over a WebSocket and serves the UI
- `web/` - Next.js UI, built as static files into `web/out`
- `config/web.yaml` - servo IDs, names, joint limits and the UI speed cap

Install the web extra (`pixi install` already includes it):

```
uv sync --extra web
```

Build the UI once (Node 20+; can be built on a laptop and copied to the Pi):

```
cd web && npm install && npm run build
```

Run it:

```
uv run talos-servo web --port /dev/ttyACM0     # real servos (or set TALOS_SERVO_PORT)
uv run talos-servo web --mock                  # simulated servos, no hardware
```

Then from any laptop on the same network open `http://<pi-hostname>.local:8000` (or the Pi's IP).
Several laptops can connect at once and see the same live state.

- **Position mode**: drag the target slider (moves live) or type a value and press Go. 0-4095 is
  one full turn; the amber marker is the current position
- **Speed mode**: slider centre is stop, left is reverse, right is forward
- **STOP ALL** (or Space) stops every servo in speed mode
- **Torque** off lets you move a joint by hand

Safety: when every browser disconnects, the server stops all speed-mode servos, and it stops them
again on shutdown. Speeds are capped by `max_speed` and targets are clamped to each servo's
`min_pos`/`max_pos` in `config/web.yaml`, so set real joint limits there before running on the
robot.

To start on boot, edit the user, paths and port in `deploy/talos-web.service`, then follow the
steps at the top of that file.

UI development with hot reload: run `uv run talos-servo web --mock`, then `cd web && npm run dev`
and open `http://localhost:3000` with `NEXT_PUBLIC_BRIDGE_URL=ws://localhost:8000/ws` set (or a
Pi's address instead of localhost).
