"""Keyboard driving loop with scripted keys and a fake clock."""

from __future__ import annotations

import contextlib
from typing import Any

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


def drive_with(
    keys: list[str | None], **kwargs: Any
) -> list[tuple[float, float, float]]:
    clock = FakeClock()
    sent: list[tuple[float, float, float]] = []
    with contextlib.suppress(KeyboardInterrupt):
        teleop.run(
            lambda *v: sent.append(v),
            Keys(keys, clock),
            linear=0.1,
            angular=0.5,
            clock=clock,
            **kwargs,
        )
    return sent


FORWARD = (0.1, 0.0, 0.0)
TURN_LEFT = (0.0, 0.0, 0.5)


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


def test_action_keys_run_once_and_do_not_drive():
    pressed: list[str] = []
    actions = {
        teleop.WALL_NEARBY_KEY: lambda: pressed.append("wall"),
        teleop.RELEASE_KEY: lambda: pressed.append("release"),
    }
    sent = drive_with(["n", None, "L", "x"], actions=actions)
    assert pressed == ["wall", "release"]
    assert sent == []


def test_space_runs_on_stop_after_its_stop():
    events: list = []
    sent: list = []
    clock = FakeClock()

    def drive(*v):
        sent.append(v)
        events.append("drive")

    with contextlib.suppress(KeyboardInterrupt):
        teleop.run(
            drive,
            Keys(["w", " ", None, "w", "x"], clock),
            linear=0.1,
            angular=0.5,
            clock=clock,
            on_stop=lambda: events.append("stop"),
        )
    # The local zero goes out first, then STOP for every input; other keys
    # do not call it.
    assert events == ["drive", "drive", "stop", "drive", "drive"]
    assert sent[1] == teleop.STOPPED


@pytest.mark.parametrize("enter", sorted(teleop.RELEASE_STOP_KEYS))
def test_enter_is_an_action_key(enter):
    pressed: list[str] = []
    actions = dict.fromkeys(
        teleop.RELEASE_STOP_KEYS, lambda: pressed.append("release stop")
    )
    assert drive_with([enter, "x"], actions=actions) == []
    assert pressed == ["release stop"]


def test_action_key_does_not_keep_a_held_key_going():
    # The held key times out as if the action key were no key at all.
    quiet = [None] * (round(teleop.KEY_TIMEOUT_S / teleop.LOOP_PERIOD_S) + 1)
    with_action = drive_with(["w", "n", *quiet], actions={"n": lambda: None})
    without = drive_with(["w", None, *quiet])
    assert with_action == without
    assert with_action[-1] == teleop.STOPPED


@pytest.mark.parametrize(
    ("mode", "keys"),
    [
        ("free", set("wsadqerf")),
        ("climbing", set("wsad")),
        ("aligning", set("rf")),
        ("clamping", set("rf")),
        (None, set("wsadqerf")),
        ("unheard of", set("wsadqerf")),
    ],
)
def test_allowed_keys_per_mode(mode, keys):
    assert teleop.allowed_keys(mode) == keys


def test_turn_keys_stop_while_climbing():
    allowed = lambda: teleop.allowed_keys("climbing")  # noqa: E731
    assert drive_with(["w", "q", "x"], allowed=allowed) == [
        FORWARD,
        teleop.STOPPED,
    ]


@pytest.mark.parametrize("mode", ["aligning", "clamping"])
def test_drive_keys_do_nothing_while_aligning_or_clamping(mode):
    allowed = lambda: teleop.allowed_keys(mode)  # noqa: E731
    assert drive_with(["w", "a", "x"], allowed=allowed) == []


def test_keys_follow_a_mode_change_mid_run():
    mode = ["free"]

    def climb() -> None:
        mode[0] = "climbing"

    sent = drive_with(
        ["q", "n", "q", "x"],
        actions={"n": climb},
        allowed=lambda: teleop.allowed_keys(mode[0]),
    )
    assert sent == [TURN_LEFT, TURN_LEFT, teleop.STOPPED]


def test_space_stops_in_every_mode():
    allowed = lambda: teleop.allowed_keys("aligning")  # noqa: E731
    assert drive_with([" ", "x"], allowed=allowed) == [teleop.STOPPED]


def test_status_line():
    assert (
        teleop.status_line("climbing", "wall contact", "web")
        == "mode: climbing (wall contact) | driving: web"
    )
    assert teleop.status_line(None, None, None) == "mode: ? | driving: ?"


def test_status_printer_prints_at_start_and_on_change():
    status = [("free", None, "none")]
    lines: list[str] = []
    printer = teleop.StatusPrinter(lambda: status[0], lines.append)
    printer()
    printer()
    status[0] = ("aligning", "wall nearby", "none")
    printer()
    status[0] = ("aligning", "wall nearby", "keyboard")
    printer()
    printer()
    assert lines == [
        "mode: free | driving: none",
        "mode: aligning (wall nearby) | driving: none",
        "mode: aligning (wall nearby) | driving: keyboard",
    ]


def test_terminal_keys_need_a_terminal(monkeypatch):
    monkeypatch.setattr("sys.stdin.isatty", lambda: False, raising=False)
    with pytest.raises(RuntimeError), teleop.terminal_keys():
        pass


HINGE_RATE = 0.15
OPEN = HINGE_RATE
CLOSE = -HINGE_RATE


def hinge_with(
    keys: list[str | None], **kwargs: Any
) -> tuple[list[float], list[tuple[float, float, float]]]:
    """Hinge rates and drive commands sent for scripted keys."""
    rates: list[float] = []
    sent = drive_with(keys, hinge=rates.append, hinge_rate=HINGE_RATE, **kwargs)
    return rates, sent


def test_held_r_opens_the_hinge_every_loop():
    rates, sent = hinge_with(["r", "r", "r", "x"])
    assert rates == [OPEN, OPEN, OPEN, 0.0]
    assert sent == []


def test_f_closes_the_hinge():
    rates, _ = hinge_with(["f", "x"])
    assert rates == [CLOSE, 0.0]


def test_letting_go_stops_the_hinge_once():
    quiet = [None] * (round(teleop.KEY_TIMEOUT_S / teleop.LOOP_PERIOD_S) + 3)
    rates, _ = hinge_with(["r", *quiet, "x"])
    assert rates[-1] == 0.0
    assert rates.count(0.0) == 1
    assert set(rates[:-1]) == {OPEN}


def test_space_stops_the_hinge():
    rates, _ = hinge_with(["r", " ", None, "x"])
    assert rates == [OPEN, 0.0]


def test_ctrl_c_stops_the_hinge():
    rates, _ = hinge_with(["r"])
    assert rates == [OPEN, 0.0]


def test_drive_key_stops_the_hinge_and_r_stops_the_drive():
    rates, sent = hinge_with(["r", "w", "r", "x"])
    assert rates == [OPEN, 0.0, OPEN, 0.0]
    assert sent == [FORWARD, teleop.STOPPED]


@pytest.mark.parametrize("mode", ["climbing"])
def test_hinge_keys_do_nothing_outside_free_and_clamping(mode):
    allowed = lambda: teleop.allowed_keys(mode)  # noqa: E731
    rates, _ = hinge_with(["r", "f", "x"], allowed=allowed)
    assert rates == []


def test_hinge_keys_work_while_clamping():
    allowed = lambda: teleop.allowed_keys("clamping")  # noqa: E731
    rates, sent = hinge_with(["r", "w", "x"], allowed=allowed)
    assert rates == [OPEN, 0.0]
    assert sent == []


def test_hinge_keys_without_a_hinge_only_stop_the_drive():
    assert drive_with(["w", "r", "x"]) == [FORWARD, teleop.STOPPED]


def test_ros_help_names_the_hinge_keys():
    assert "R/F" in teleop.ROS_HELP
    assert "R/F" not in teleop.HELP
