from __future__ import annotations

import pytest

from talos_servo.modes import (
    Event,
    InvalidTransition,
    Mode,
    ModeMachine,
    next_mode,
    stop_keeps_hinge,
)

VALID = [
    (Mode.FREE, Event.WALL_NEARBY, Mode.ALIGNING),
    (Mode.ALIGNING, Event.ALIGNED, Mode.CLAMPING),
    (Mode.CLAMPING, Event.WALL_CONTACT, Mode.CLIMBING),
    (Mode.CLIMBING, Event.NOT_MOVING_UP, Mode.CLAMPING),
]


@pytest.mark.parametrize(("start", "event", "end"), VALID)
def test_valid_transitions(start, event, end):
    machine = ModeMachine(start)
    transition = machine.handle(event, "why")
    assert machine.mode is end
    assert machine.reason == "why"
    assert (transition.previous, transition.mode) == (start, end)
    assert transition.event is event


def test_starts_free():
    machine = ModeMachine()
    assert machine.mode is Mode.FREE
    assert machine.reason == "started"


@pytest.mark.parametrize("start", list(Mode))
def test_release_goes_free_from_every_mode(start):
    machine = ModeMachine(start)
    machine.handle(Event.RELEASE)
    assert machine.mode is Mode.FREE
    assert machine.reason == "release"


@pytest.mark.parametrize(
    ("start", "event"),
    [
        (mode, event)
        for mode in Mode
        for event in Event
        if event is not Event.RELEASE
        and (mode, event) not in {(s, e) for s, e, _ in VALID}
    ],
)
def test_invalid_transition_keeps_state(start, event):
    machine = ModeMachine(start, "before")
    with pytest.raises(InvalidTransition, match=event.value):
        machine.handle(event, "after")
    assert machine.mode is start
    assert machine.reason == "before"


def test_peek_does_not_change_mode():
    machine = ModeMachine()
    assert machine.peek(Event.WALL_NEARBY) is Mode.ALIGNING
    assert machine.mode is Mode.FREE
    with pytest.raises(InvalidTransition):
        machine.peek(Event.ALIGNED)


def test_set_reason_keeps_mode():
    machine = ModeMachine(Mode.CLIMBING)
    machine.set_reason("tilt fault")
    assert (machine.mode, machine.reason) == (Mode.CLIMBING, "tilt fault")


@pytest.mark.parametrize(
    ("mode", "keeps"),
    [
        (Mode.FREE, False),
        (Mode.ALIGNING, False),
        (Mode.CLAMPING, True),
        (Mode.CLIMBING, True),
    ],
)
def test_stop_keeps_hinge_only_on_the_walls(mode, keeps):
    assert stop_keeps_hinge(mode) is keeps


def test_next_mode_values_are_plain_strings():
    assert next_mode(Mode.FREE, Event.WALL_NEARBY).value == "aligning"
