"""Keyboard driving for `talos-servo drive`: keys in, body velocity out.

The loop only knows a `drive(vx, vy, wz)` callable, so the same code drives the
robot directly (`Robot.drive`) or through ROS (`RosBackend.drive`, which
publishes /cmd_vel).
"""

from __future__ import annotations

import sys
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager

# A held key repeats faster than this, so letting go stops the robot within this
# time.
KEY_TIMEOUT_S = 0.3
LOOP_PERIOD_S = 0.05
QUIT_KEY = "x"
STOP_KEY = " "
HELP = (
    "W/S forward/back, A/D left/right, Q/E turn, space stops, X quits. Hold a "
    "key to keep going."
)

# key -> (vx, vy, wz) as fractions of the linear and angular speed
KEYS: dict[str, tuple[float, float, float]] = {
    "w": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0),
}
STOPPED = (0.0, 0.0, 0.0)

Drive = Callable[[float, float, float], None]
# Returns the key pressed within `timeout` seconds, or None.
ReadKey = Callable[[float], str | None]


def velocity_for(
    key: str, linear: float, angular: float
) -> tuple[float, float, float] | None:
    """Body velocity for a key, or None if the key does not drive."""
    fractions = KEYS.get(key.lower())
    if fractions is None:
        return None
    fx, fy, fz = fractions
    return fx * linear, fy * linear, fz * angular


def run(
    drive: Drive,
    read_key: ReadKey,
    *,
    linear: float,
    angular: float,
    tick: Callable[[], None] = lambda: None,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Drive from keys until X.

    Sends a stop on the way out, also on Ctrl+C.
    """
    command = STOPPED
    last_key_at = clock()
    try:
        while True:
            key = read_key(LOOP_PERIOD_S)
            if key is not None:
                if key.lower() == QUIT_KEY:
                    return
                command = velocity_for(key, linear, angular) or STOPPED
                last_key_at = clock()
            if clock() - last_key_at > KEY_TIMEOUT_S:
                command = STOPPED
            drive(*command)
            tick()
    finally:
        drive(*STOPPED)
        tick()


@contextmanager
def terminal_keys() -> Iterator[ReadKey]:
    """Read single key presses from the terminal without Enter (Linux and
    macOS)."""
    try:
        import select
        import termios
        import tty
    except ImportError:
        raise RuntimeError(
            "talos-servo drive needs a Linux or macOS terminal"
        ) from None
    if not sys.stdin.isatty():
        raise RuntimeError("talos-servo drive needs an interactive terminal")
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)

    def read_key(timeout: float) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if readable else None

    try:
        tty.setcbreak(fd)
        yield read_key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
