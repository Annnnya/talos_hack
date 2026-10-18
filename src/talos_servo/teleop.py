"""Keyboard driving for `talos-servo drive`: keys in, body velocity out.

The loop only knows a `drive(vx, vy, wz)` callable, so the same code drives the
robot directly (`Robot.drive`) or through ROS (`RosBackend.drive`, which
publishes on the keyboard input of twist_mux). Over ROS the loop also gets
one-shot action keys (wall nearby, release), the drive keys the robot mode
allows and the hinge keys R and F, held like the drive keys (#95).
"""

from __future__ import annotations

import sys
import time
from collections.abc import Callable, Collection, Iterator, Mapping
from contextlib import contextmanager

from talos_servo.modes import (
    ANGULAR_Z,
    DRIVE_AXES_BY_MODE,
    HINGE_MODES,
    LINEAR_X,
    LINEAR_Y,
    Mode,
    allowed_axes,
)

# A held key repeats faster than this, so letting go stops the robot within this
# time.
KEY_TIMEOUT_S = 0.3
LOOP_PERIOD_S = 0.05
QUIT_KEY = "x"
STOP_KEY = " "
WALL_NEARBY_KEY = "n"
RELEASE_KEY = "l"
# Enter releases STOP; terminals send either character for it.
RELEASE_STOP_KEYS = frozenset({"\n", "\r"})
HELP = (
    "W/S forward/back, A/D left/right, Q/E turn, space stops, X quits. Hold a "
    "key to keep going."
)
ROS_HELP = (
    f"{HELP} Space also turns STOP on for every input, Enter releases it. "
    "N wall nearby, L release from the walls. Q/E are off while climbing, "
    "the drive keys while aligning or clamping. Hold R/F to open/close the "
    "hinge (free and clamping)."
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
# key -> sign of the hinge opening speed: R expands, F collapses (#95).
HINGE_KEYS: dict[str, float] = {"r": 1.0, "f": -1.0}
# Same order as the fractions in KEYS.
KEY_AXES = (LINEAR_X, LINEAR_Y, ANGULAR_Z)


def _keys_within(axes: Collection[str]) -> frozenset[str]:
    """Drive keys that only use `axes`."""
    return frozenset(
        key
        for key, fractions in KEYS.items()
        if all(a in axes for a, f in zip(KEY_AXES, fractions, strict=True) if f)
    )


# Drive keys each mode allows, from the per-mode drive rule in modes.py that
# the web server enforces too.
DRIVE_KEYS_BY_MODE: dict[Mode, frozenset[str]] = {
    mode: _keys_within(axes) for mode, axes in DRIVE_AXES_BY_MODE.items()
}

Drive = Callable[[float, float, float], None]
# Takes the hinge opening speed in rad/s, 0 stops.
MoveHinge = Callable[[float], None]
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
    """Drive and hinge keys allowed in `mode`. An unknown mode allows every
    key, see `modes.allowed_axes`; the expansion node still refuses to move
    the hinge without a mode."""
    keys = _keys_within(allowed_axes(mode))
    try:
        hinge = Mode(mode) in HINGE_MODES
    except ValueError:
        hinge = True
    return keys | frozenset(HINGE_KEYS) if hinge else keys


def run(
    drive: Drive,
    read_key: ReadKey,
    *,
    linear: float,
    angular: float,
    tick: Callable[[], None] = lambda: None,
    actions: Mapping[str, Callable[[], None]] | None = None,
    allowed: Callable[[], Collection[str]] | None = None,
    on_stop: Callable[[], None] | None = None,
    hinge: MoveHinge | None = None,
    hinge_rate: float = 0.0,
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
    `on_stop` runs after the stop that Space sends, e.g. to turn STOP on
    for every input.
    With `hinge`, a held R or F sends +-`hinge_rate` to it the same way: one
    0 on letting go, Space or another key, and a 0 on the way out. Without
    it R and F stop the drive like any other key.
    """
    actions = actions or {}
    command = STOPPED
    sent = STOPPED
    hinge_command = 0.0
    hinge_sent = 0.0
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
                    sign = HINGE_KEYS.get(key.lower(), 0.0)
                    hinge_command = sign * hinge_rate if hinge else 0.0
                else:
                    command = STOPPED
                    hinge_command = 0.0
                stop_now = key == STOP_KEY
                last_key_at = clock()
            if clock() - last_key_at > KEY_TIMEOUT_S:
                command = STOPPED
                hinge_command = 0.0
            if command != STOPPED or sent != STOPPED or stop_now:
                # Set first: a drive() that raises partway may already have
                # moved some wheels, so `finally` must still stop them.
                sent = command
                drive(*command)
            if hinge is not None and (hinge_command or hinge_sent or stop_now):
                hinge_sent = hinge_command
                hinge(hinge_command)
            if stop_now and on_stop is not None:
                on_stop()
            tick()
    finally:
        try:
            if sent != STOPPED:
                drive(*STOPPED)
        finally:
            if hinge is not None and hinge_sent:
                hinge(0.0)
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
