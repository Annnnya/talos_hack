"""Keyboard driving for `talos-servo drive`: keys in, body velocity out.

The loop only knows a `drive(vx, vy, wz)` callable, so the same code drives the
robot directly (`Robot.drive`) or through ROS (`RosBackend.drive`, which
publishes on the keyboard input of twist_mux). Over ROS the loop also gets
one-shot action keys (wall nearby, release) and the drive keys the robot mode
allows.
"""

from __future__ import annotations

import sys
import time
from collections.abc import Callable, Collection, Iterator, Mapping
from contextlib import contextmanager

from talos_servo.modes import Mode

# A held key repeats faster than this, so letting go stops the robot within this
# time.
KEY_TIMEOUT_S = 0.3
LOOP_PERIOD_S = 0.05
QUIT_KEY = "x"
STOP_KEY = " "
WALL_NEARBY_KEY = "n"
RELEASE_KEY = "l"
# R and F stay free for expanding and collapsing the hinge (#95).
HELP = (
    "W/S forward/back, A/D left/right, Q/E turn, space stops, X quits. Hold a "
    "key to keep going."
)
ROS_HELP = (
    f"{HELP} N wall nearby, L release from the walls. Q/E are off while "
    "climbing, the drive keys while aligning or clamping."
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
TURN_KEYS = frozenset({"q", "e"})
ALL_DRIVE_KEYS = frozenset(KEYS)
# Drive keys each mode allows. Climbing cannot turn on the walls, and aligning
# and clamping move the robot by themselves.
DRIVE_KEYS_BY_MODE: dict[Mode, frozenset[str]] = {
    Mode.FREE: ALL_DRIVE_KEYS,
    Mode.CLIMBING: ALL_DRIVE_KEYS - TURN_KEYS,
    Mode.ALIGNING: frozenset(),
    Mode.CLAMPING: frozenset(),
}

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


def allowed_keys(mode: str | None) -> frozenset[str]:
    """Drive keys allowed in `mode`.

    An unknown mode (no mode node running) allows every key, like free.
    """
    try:
        return DRIVE_KEYS_BY_MODE[Mode(mode)]
    except ValueError:
        return ALL_DRIVE_KEYS


def run(
    drive: Drive,
    read_key: ReadKey,
    *,
    linear: float,
    angular: float,
    tick: Callable[[], None] = lambda: None,
    actions: Mapping[str, Callable[[], None]] | None = None,
    allowed: Callable[[], Collection[str]] | None = None,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Drive from keys until X.

    A held key is re-sent every loop. Letting go sends one stop, then
    nothing while idle, so another interface (the web UI) can drive without
    being cancelled. Space always sends a stop. On the way out, also on
    Ctrl+C, a stop is sent only if this loop was still driving.

    `actions` maps keys to one-shot callables that leave driving alone.
    `allowed` returns the drive keys allowed right now; a drive key it leaves
    out stops this input, like any other key that does not drive.
    """
    actions = actions or {}
    command = STOPPED
    sent = STOPPED
    last_key_at = clock()
    try:
        while True:
            key = read_key(LOOP_PERIOD_S)
            stop_now = False
            action = None if key is None else actions.get(key.lower())
            if action is not None:
                action()
            elif key is not None:
                if key.lower() == QUIT_KEY:
                    return
                if allowed is None or key.lower() in allowed():
                    command = velocity_for(key, linear, angular) or STOPPED
                else:
                    command = STOPPED
                stop_now = key == STOP_KEY
                last_key_at = clock()
            if clock() - last_key_at > KEY_TIMEOUT_S:
                command = STOPPED
            if command != STOPPED or sent != STOPPED or stop_now:
                # Set first: a drive() that raises partway may already have
                # moved some wheels, so `finally` must still stop them.
                sent = command
                drive(*command)
            tick()
    finally:
        if sent != STOPPED:
            drive(*STOPPED)
        tick()


UNKNOWN = "?"


def status_line(
    mode: str | None, reason: str | None, active_input: str | None
) -> str:
    """One line for the terminal, e.g. `mode: climbing (wall contact) |
    driving: web`."""
    line = f"mode: {mode or UNKNOWN}"
    if reason:
        line += f" ({reason})"
    return f"{line} | driving: {active_input or UNKNOWN}"


class StatusPrinter:
    """Prints the robot status line once at start and again when it changes.

    Call it every loop, e.g. as the `tick` of `run`.
    """

    def __init__(
        self,
        read: Callable[[], tuple[str | None, str | None, str | None]],
        out: Callable[[str], None] = print,
    ) -> None:
        self._read = read
        self._out = out
        self._last: str | None = None

    def __call__(self) -> None:
        line = status_line(*self._read())
        if line != self._last:
            self._last = line
            self._out(line)


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
