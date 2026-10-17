# talos_hack

Software and tests for the Talos robot from the physical AI hackathon. The robot uses Feetech
STS/SMS serial bus servos daisy-chained on one bus.

Contributing (humans and AI agents): read [AGENTS.md](AGENTS.md). Everything goes through PRs.

## Setup

Requires [uv](https://docs.astral.sh/uv/).

```
uv sync
```

This creates `.venv` with the official Feetech SDK (`ftservo-python-sdk`, imported as
`scservo_sdk`) and installs the `talos-servo` command.

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
