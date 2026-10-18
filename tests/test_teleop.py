"""Keyboard driving loop with scripted keys and a fake clock."""

from __future__ import annotations

import contextlib

import pytest
from conftest import FakeClock

from talos_servo import teleop


class Keys:
    """Hands out one scripted key per loop and moves the clock along."""

    def __init__(self, keys: list[str | None], clock: FakeClock) -> None:
        self.keys = keys
        self.clock = clock

    def __call__(self, timeout: float) -> str | None:
        self.clock.now += timeout
        if not self.keys:
            raise KeyboardInterrupt
        return self.keys.pop(0)


def drive_with(keys: list[str | None]) -> list[tuple[float, float, float]]:
    clock = FakeClock()
    sent: list[tuple[float, float, float]] = []
    with contextlib.suppress(KeyboardInterrupt):
        teleop.run(
            lambda *v: sent.append(v),
            Keys(keys, clock),
            linear=0.1,
            angular=0.5,
            clock=clock,
        )
    return sent


def test_keys_map_to_body_velocity():
    assert teleop.velocity_for("w", 0.1, 0.5) == (0.1, 0.0, 0.0)
    assert teleop.velocity_for("A", 0.1, 0.5) == (0.0, 0.1, 0.0)
    assert teleop.velocity_for("e", 0.1, 0.5) == (0.0, 0.0, -0.5)
    assert teleop.velocity_for("z", 0.1, 0.5) is None


def test_quit_key_stops_the_robot():
    sent = drive_with(["w", "x"])
    assert sent == [(0.1, 0.0, 0.0), teleop.STOPPED]


def test_space_stops_right_away():
    sent = drive_with(["w", " ", "x"])
    assert sent[1] == teleop.STOPPED


def test_releasing_the_key_stops_after_the_timeout():
    quiet = round(teleop.KEY_TIMEOUT_S / teleop.LOOP_PERIOD_S) + 1
    sent = drive_with(["w", *([None] * quiet), "x"])
    assert sent[1] == (0.1, 0.0, 0.0)
    assert sent[-1] == teleop.STOPPED
    assert sent.count(teleop.STOPPED) == 1


def test_idle_loop_sends_nothing():
    assert drive_with([None] * 20 + ["x"]) == []


def test_release_sends_one_stop_then_nothing():
    quiet = round(teleop.KEY_TIMEOUT_S / teleop.LOOP_PERIOD_S) + 20
    sent = drive_with(["w", *([None] * quiet)])
    assert sent[-1] == teleop.STOPPED
    assert sent.count(teleop.STOPPED) == 1


def test_quit_while_idle_sends_no_stop():
    assert drive_with(["w", " ", None, "x"]) == [
        (0.1, 0.0, 0.0),
        teleop.STOPPED,
    ]


def test_space_while_idle_still_stops():
    assert drive_with([None, " ", None, "x"]) == [teleop.STOPPED]


def test_ctrl_c_still_sends_a_stop():
    sent = drive_with(["w"])
    assert sent[-1] == teleop.STOPPED


def test_drive_that_fails_partway_still_gets_a_stop():
    # A drive() that raises may already have set some wheel speeds.
    clock = FakeClock()
    sent: list[tuple[float, float, float]] = []

    def drive(*v: float) -> None:
        sent.append(v)
        if len(sent) == 1:
            raise OSError("serial write failed")

    with pytest.raises(OSError):
        teleop.run(
            drive, Keys(["w"], clock), linear=0.1, angular=0.5, clock=clock
        )
    assert sent == [(0.1, 0.0, 0.0), teleop.STOPPED]


def test_terminal_keys_need_a_terminal(monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False, raising=False)
    with pytest.raises(RuntimeError), teleop.terminal_keys():
        pass
