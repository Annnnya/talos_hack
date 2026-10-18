"""Robot mode state machine: free, aligning, clamping and climbing.

Plain Python with no ROS imports. `ros/mode_node.py` wraps it, publishes the
mode and switches controllers when it changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import NamedTuple


class Mode(str, Enum):
    FREE = "free"
    ALIGNING = "aligning"
    CLAMPING = "clamping"
    CLIMBING = "climbing"


class Event(str, Enum):
    WALL_NEARBY = "wall_nearby"
    ALIGNED = "aligned"
    WALL_CONTACT = "wall_contact"
    NOT_MOVING_UP = "not_moving_up"
    RELEASE = "release"


# (from, event) -> to. RELEASE is allowed from every mode, see `next_mode`.
TRANSITIONS: dict[tuple[Mode, Event], Mode] = {
    (Mode.FREE, Event.WALL_NEARBY): Mode.ALIGNING,
    (Mode.ALIGNING, Event.ALIGNED): Mode.CLAMPING,
    (Mode.CLAMPING, Event.WALL_CONTACT): Mode.CLIMBING,
    (Mode.CLIMBING, Event.NOT_MOVING_UP): Mode.CLAMPING,
}

# Modes where the hinge presses on the walls. Releasing it there drops the
# robot, so STOP only stops the wheels.
HOLDING_MODES = frozenset({Mode.CLAMPING, Mode.CLIMBING})
# Modes where the expansion node may open and close the hinge (#76). In
# free and aligning the mode node first swaps the base controller for
# per-wheel speeds. Aligning is in until it is automated, so the user can
# clamp by hand after wall nearby.
HINGE_MODES = frozenset({Mode.FREE, Mode.ALIGNING, Mode.CLAMPING})
# Modes where the wheels may climb (front pair one way, rear pair the
# other): only after wall nearby.
CLIMB_MODES = frozenset({Mode.ALIGNING, Mode.CLAMPING})
# Modes whose own controllers do not turn the wheels one by one, so hinge
# and climb motions swap them in (`modes.hinge_motion`).
SWAP_MODES = frozenset({Mode.FREE, Mode.ALIGNING})

# Body velocity axes a driver may use in each mode, for every interface.
# Climbing cannot turn on the walls, and aligning and clamping move the robot
# by themselves.
LINEAR_X, LINEAR_Y, ANGULAR_Z = "vx", "vy", "wz"
ALL_AXES = frozenset({LINEAR_X, LINEAR_Y, ANGULAR_Z})
DRIVE_AXES_BY_MODE: dict[Mode, frozenset[str]] = {
    Mode.FREE: ALL_AXES,
    Mode.CLIMBING: frozenset({LINEAR_X, LINEAR_Y}),
    Mode.ALIGNING: frozenset(),
    Mode.CLAMPING: frozenset(),
}


def allowed_axes(mode: str | None) -> frozenset[str]:
    """Drive axes allowed in `mode`.

    An unknown mode (no mode node running) allows every axis, like free.
    That fails open on purpose: without the mode node its stop lock goes
    quiet, so twist_mux is locked and nothing moves the robot anyway.
    """
    try:
        return DRIVE_AXES_BY_MODE[Mode(mode)]
    except ValueError:
        return ALL_AXES


def drive_error(
    mode: str | None, vx: float, vy: float, wz: float
) -> str | None:
    """Why a drive is refused in `mode`, or None if it is allowed.

    A zero twist always passes, so stopping works in every mode.
    """
    used = {
        axis
        for axis, value in ((LINEAR_X, vx), (LINEAR_Y, vy), (ANGULAR_Z, wz))
        if value
    }
    allowed = allowed_axes(mode)
    if used <= allowed:
        return None
    if allowed:
        return f"cannot rotate while {mode}"
    return f"cannot drive while {mode}"


def hinge_error(mode: str | None, rate: float) -> str | None:
    """Why a hinge move is refused in `mode`, or None if it is allowed.

    Zero always passes, so stopping works in every mode. An unknown mode
    passes too: the expansion node still refuses without a mode.
    """
    if not rate:
        return None
    try:
        known = Mode(mode)
    except ValueError:
        return None
    if known in HINGE_MODES:
        return None
    return f"cannot move the hinge while {mode}"


def climb_error(mode: str | None, speed: float) -> str | None:
    """Why a climb is refused in `mode`, or None. Zero always passes."""
    if not speed:
        return None
    try:
        known = Mode(mode)
    except ValueError:
        return "cannot climb without a robot mode"
    if known in CLIMB_MODES:
        return None
    return f"cannot climb while {mode}: press wall nearby first"


class RobotStatus(NamedTuple):
    """Shared robot state from the mode node and mux_status, as clients see
    it. Each field is None until heard, and always None without ROS."""

    mode: str | None
    mode_reason: str | None
    active_input: str | None


class InvalidTransition(ValueError):
    """The event is not allowed in the current mode."""


@dataclass(frozen=True)
class Transition:
    previous: Mode
    mode: Mode
    event: Event
    reason: str


def next_mode(mode: Mode, event: Event) -> Mode:
    """The mode `event` leads to from `mode`, or raise InvalidTransition."""
    if event is Event.RELEASE:
        return Mode.FREE
    try:
        return TRANSITIONS[(mode, event)]
    except KeyError:
        raise InvalidTransition(
            f"{event.value} is not allowed in {mode.value}"
        ) from None


def stop_keeps_hinge(mode: Mode) -> bool:
    """True when STOP must stop the wheels but keep the hinge holding."""
    return mode in HOLDING_MODES


class ModeMachine:
    """Current mode and the reason for the last change."""

    def __init__(self, mode: Mode = Mode.FREE, reason: str = "started") -> None:
        self._mode = mode
        self._reason = reason

    @property
    def mode(self) -> Mode:
        return self._mode

    @property
    def reason(self) -> str:
        return self._reason

    def peek(self, event: Event) -> Mode:
        """The mode `event` would lead to, without changing anything."""
        return next_mode(self._mode, event)

    def handle(self, event: Event, reason: str = "") -> Transition:
        """Apply `event`. On InvalidTransition nothing changes."""
        new_mode = next_mode(self._mode, event)
        transition = Transition(
            previous=self._mode,
            mode=new_mode,
            event=event,
            reason=reason or event.value,
        )
        self._mode = new_mode
        self._reason = transition.reason
        return transition

    def set_reason(self, reason: str) -> None:
        """Report something (e.g. a fault) without changing the mode."""
        self._reason = reason
