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

## ROS 2 Jazzy mecanum base

The `ros2_ws/src/talos_mecanum` package provides a four-wheel base driver and a terminal
keyboard interface. Pixi installs ROS 2 Jazzy and colcon for Linux ARM64, including Raspberry Pi
OS 64-bit. Build once with `pixi run ros-build`, then start the driver with `pixi run drive` and
the keyboard interface in another terminal with `pixi run teleop`. Set the servo port before
starting the driver, for example:

```
export TALOS_SERVO_PORT=/dev/serial/by-id/your-servo-adapter
```

The driver accepts `geometry_msgs/msg/Twist` on `/cmd_vel`. It provides `/base/enable`
(`std_srvs/srv/SetBool`) to ping IDs 1-4, select wheel mode, and enable torque, and `/base/stop`
(`std_srvs/srv/Trigger`) to stop all four wheels. Drive starts disabled; press `m` in the keyboard
interface to enable or disable. Use `w`/`s` to move forward/back, `a`/`d` to strafe, `q`/`e` to
turn, space to stop, and `x` to exit. Direction keys send short pulses; press repeatedly for
continuous movement. Commands stop automatically if `/cmd_vel` is not updated for 0.5 seconds.

Before enabling on the floor, edit `ros2_ws/src/talos_mecanum/config/robot.yaml` to match the
physical wheel-to-servo mapping and each wheel's direction. The defaults assume IDs 1-4 are ordered
front-left, front-right, rear-left, rear-right and use positive direction signs. Set
`wheel_ids`, `wheel_directions`, measured `wheelbase_m` and `track_width_m`, and calibrated
`steps_per_meter` for the actual robot. The geometry and scale defaults are placeholders; the
driver caps commands to 100 servo steps/s and 0.1 m/s linear speed until configured. Start with the
wheels lifted and a clear area.

On Raspberry Pi OS 64-bit, install Pixi, then run these commands from the repository:

```
pixi install
pixi run ros-build
export TALOS_SERVO_PORT=/dev/serial0
pixi run drive
```

In a second terminal, from the repository, run `pixi run teleop`. If the adapter uses another
device, set `TALOS_SERVO_PORT` to its `/dev/serial/by-id/...` path instead. Keep the wheels lifted
until wheel order, direction, and calibration are verified.
