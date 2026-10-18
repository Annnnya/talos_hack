"""`talos-servo calibrate-bend` key loop against a fake backend (#84).

Skipped where ROS is missing: the loop lives next to the other --ros
commands.
"""

from __future__ import annotations

from collections.abc import Iterable

import pytest

pytest.importorskip("rclpy")

from conftest import FakeClock

from talos_servo.robot import CommandError
from talos_servo.ros import cli_commands
from talos_servo.ros.names import calibrate_service


class FakeBackend:
    def __init__(self, fail: Iterable[str] = ()) -> None:
        self.calls: list[str] = []
        self.fail = set(fail)
        self.opened = self.released = False

    def open(self) -> None:
        self.opened = True

    def release(self) -> None:
        self.released = True

    def trigger(self, service: str) -> str:
        step = service.rsplit("/", 1)[1]
        assert service == calibrate_service("bend", step)
        self.calls.append(step)
        if step in self.fail:
            raise CommandError(f"{step} failed")
        return f"{step} done"

    def steps(self) -> list[str]:
        """Calls without the status polls."""
        return [c for c in self.calls if c != "status"]


def keys(*pressed: str | BaseException):
    """A read_key that hands out `pressed` in order, then fails the test."""
    queue = list(pressed)

    def read_key(_timeout: float) -> str | None:
        if not queue:
            raise AssertionError("the loop asked for more keys")
        key = queue.pop(0)
        if isinstance(key, BaseException):
            raise key
        return key

    return read_key


def run(backend: FakeBackend, read_key, lines: list[str]) -> bool:
    return cli_commands.calibrate_bend_over_ros(
        backend, read_key, "bend", out=lines.append, clock=FakeClock()
    )


def test_full_calibration_calls_each_step_in_order():
    backend, lines = FakeBackend(), []
    saved = run(backend, keys("o", "O", "f", "\n", "f", "\r", "\n"), lines)
    assert saved
    assert backend.steps() == [
        "start",
        "open",
        "open",
        "fold",
        "record_flat",
        "fold",
        "record_fold",
        "save",
    ]
    assert "save done" in lines
    assert backend.opened and backend.released


def test_x_cancels():
    backend, lines = FakeBackend(), []
    assert not run(backend, keys("o", "\n", "x"), lines)
    assert backend.steps() == ["start", "open", "record_flat", "cancel"]
    assert backend.released


def test_ctrl_c_still_cancels():
    backend, lines = FakeBackend(), []
    with pytest.raises(KeyboardInterrupt):
        run(backend, keys("o", KeyboardInterrupt()), lines)
    assert backend.steps()[-1] == "cancel"
    assert backend.released


def test_a_failed_step_is_printed_and_can_be_retried():
    backend, lines = FakeBackend(fail={"record_flat"}), []
    assert not run(backend, keys("\n", "x"), lines)
    assert "error: record_flat failed" in lines
    # Still on step 1: no fold was recorded.
    assert "record_fold" not in backend.calls


def test_status_is_polled_and_printed_once_per_change():
    backend, lines = FakeBackend(), []
    run(backend, keys(None, None, "x"), lines)
    assert "status" in backend.calls
    assert lines.count("status done") == 1


def test_failed_start_does_not_cancel():
    backend, lines = FakeBackend(fail={"start"}), []
    with pytest.raises(CommandError):
        run(backend, keys(), lines)
    assert backend.steps() == ["start"]
    assert backend.released
