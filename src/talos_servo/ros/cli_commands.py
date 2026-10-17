"""`talos-servo spin --ros` and `monitor --ros`: the same commands, through the servo bridge.

The bridge stays the only program on the serial port, so these can run next to the web UI
(`talos-servo web --ros`) and other ROS nodes, and the UI shows what they do. Each command stops
only the servos it started, not wheels someone else is driving.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence

from talos_servo.bus import ServoError
from talos_servo.controller import SOURCE_CLI, WHEEL
from talos_servo.ros.web_backend import RosBackend

Output = Callable[[str], None]
# How long stop commands are re-sent before the node goes away. If they are all lost anyway, the
# bridge's command timeout stops the wheel.
STOP_FLUSH_TIME = 0.2


def spin_over_ros(
    backend: RosBackend,
    ids: Sequence[int],
    speed: int,
    seconds: float,
    out: Output = print,
) -> None:
    _check_ids(backend, ids)
    backend.open()
    try:
        _start_spinning(backend, ids, speed, out)
        out(f"spinning {list(ids)} at speed {speed} for {seconds}s through {backend.bridge}")
        _hold(backend, seconds)
    finally:
        _stop_and_release(backend, ids)
    out("stopped")


def monitor_over_ros(
    backend: RosBackend,
    ids: Sequence[int],
    speed: int,
    seconds: float | None,
    interval: float,
    out: Output = print,
) -> None:
    _check_ids(backend, ids)
    backend.open()
    try:
        if speed:
            _start_spinning(backend, ids, speed, out)
        _hold(backend, seconds, every=interval, on_tick=lambda: _print_states(backend, ids, out))
    finally:
        if speed:
            _stop_and_release(backend, ids)
        else:
            backend.release()


def _check_ids(backend: RosBackend, ids: Sequence[int]) -> None:
    missing = [i for i in ids if i not in backend.servos]
    if missing:
        raise ServoError(
            f"servo ids {missing} are not in the config, and the bridge only drives configured "
            f"servos (configured: {sorted(backend.servos)})"
        )


def _start_spinning(backend: RosBackend, ids: Sequence[int], speed: int, out: Output) -> None:
    limit = backend.config.max_speed
    if abs(speed) > limit:
        out(f"note: speed limited to {limit} by max_speed in the config")
    for servo_id in ids:
        backend.set_mode(servo_id, WHEEL)
        backend.spin(servo_id, speed, source=SOURCE_CLI)


def _hold(
    backend: RosBackend,
    seconds: float | None,
    *,
    every: float = 0.0,
    on_tick: Callable[[], None] | None = None,
) -> None:
    """Wait `seconds` (None: until Ctrl+C) while re-sending wheel speeds to the bridge.

    Runs at the telemetry rate whatever `every` is, because a print interval longer than the
    bridge's command timeout would otherwise let the watchdog stop the wheels.
    """
    period = 1.0 / backend.config.telemetry_hz
    start = time.monotonic()
    next_tick = start
    while True:
        now = time.monotonic()
        if seconds is not None and now - start >= seconds:
            return
        if on_tick is not None and now >= next_tick:
            on_tick()
            next_tick = now + every
        backend.poll()
        time.sleep(period)


def _print_states(backend: RosBackend, ids: Sequence[int], out: Output) -> None:
    states = {s["id"]: s for s in backend.snapshot()}
    for servo_id in ids:
        s = states[servo_id]
        if s["online"]:
            out(f"servo {servo_id}: position={s['position']} speed={s['speed']}")
        else:
            out(f"servo {servo_id}: offline ({s['error'] or 'no data yet'})")


def _stop_and_release(backend: RosBackend, ids: Sequence[int]) -> None:
    # A single stop is a fire-and-forget publish a busy bridge can miss, so repeat it until the
    # node goes away.
    period = 1.0 / backend.config.telemetry_hz
    deadline = time.monotonic() + STOP_FLUSH_TIME
    try:
        while True:
            for servo_id in ids:
                backend.stop(servo_id, source=SOURCE_CLI)
            if time.monotonic() >= deadline:
                break
            time.sleep(period)
    finally:
        backend.release()
