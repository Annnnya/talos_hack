"""Robot mode state machine: free, aligning, clamping and climbing.

Plain Python with no ROS imports. `ros/mode_node.py` wraps it, publishes the
mode and switches controllers when it changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


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
