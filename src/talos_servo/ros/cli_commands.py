"""`talos-servo spin/monitor/drive --ros`: the same commands, through the robot
node.

The robot node stays the only program on the serial port, so these can run next
to the web UI (`talos-servo web --ros`) and other ROS nodes, and the UI shows
what they do. Each command stops only the joints it started, not wheels someone
else is driving.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence

from talos_servo import teleop
from talos_servo.bus import ServoError
from talos_servo.robot import SOURCE_CLI, CommandError
from talos_servo.ros.web_backend import RosBackend

Output = Callable[[str], None]
# How long stop commands are re-sent before the node goes away. If they are all
# lost anyway, the robot's command timeout stops the wheel.
STOP_FLUSH_TIME = 0.2


def spin_over_ros(
    backend: RosBackend,
    joints: Sequence[str],
    speed: int,
    seconds: float,
    out: Output = print,
) -> None:
    _check_joints(backend, joints)
    backend.open()
    try:
        _start_spinning(backend, joints, speed, out)
        out(
            f"spinning {list(joints)} at speed {speed} for {seconds}s through "
            "the robot node"
        )
        _hold(backend, seconds)
    finally:
        _stop_and_release(backend, joints)
    out("stopped")


def monitor_over_ros(
    backend: RosBackend,
    joints: Sequence[str],
    speed: int,
    seconds: float | None,
    interval: float,
    out: Output = print,
) -> None:
    _check_joints(backend, joints)
    backend.open()
    try:
        if speed:
            _start_spinning(backend, joints, speed, out)
        _hold(
            backend,
            seconds,
            every=interval,
            on_tick=lambda: _print_states(backend, joints, out),
        )
    finally:
        if speed:
            _stop_and_release(backend, joints)
        else:
            backend.release()


def drive_over_ros(
    backend: RosBackend,
    read_key: teleop.ReadKey,
    *,
    linear: float,
    angular: float,
    out: Output = print,
) -> None:
    """Keyboard driving on the keyboard input of twist_mux.

    N and L ask the mode node for wall nearby and release. The mode decides
    which drive keys work, and a status line shows the mode, why it changed
    and which input drives. Quitting stops only this input, so the web keeps
    driving.
    """
    backend.open()
    try:
        if not backend.has_cmd_vel_listener():
            out(
                f"note: nothing reads {backend.cmd_vel_topic}, start "
                "`pixi run -e ros twist-mux` or the keys will not move the "
                "robot"
            )
        teleop.run(
            backend.drive,
            read_key,
            linear=linear,
            angular=angular,
            tick=teleop.StatusPrinter(backend.robot_status, out),
            actions={
                teleop.WALL_NEARBY_KEY: _reporting(backend.wall_nearby, out),
                teleop.RELEASE_KEY: _reporting(backend.release_walls, out),
            },
            allowed=lambda: teleop.allowed_keys(backend.robot_status().mode),
        )
    finally:
        backend.release()


def _reporting(action: Callable[[], None], out: Output) -> Callable[[], None]:
    """Run `action`, printing a failure instead of ending the drive loop."""

    def run() -> None:
        try:
            action()
        except (CommandError, ServoError) as exc:
            out(f"error: {exc}")

    return run


def _check_joints(backend: RosBackend, joints: Sequence[str]) -> None:
    missing = [j for j in joints if j not in backend.joints]
    if missing:
        raise ServoError(
            f"joints {missing} are not enabled in the config, and the robot "
            "node only drives "
            f"enabled joints (enabled: {sorted(backend.joints)})"
        )


def _start_spinning(
    backend: RosBackend, joints: Sequence[str], speed: int, out: Output
) -> None:
    limit = backend.config.max_speed
    if abs(speed) > limit:
        out(f"note: speed limited to {limit} by max_speed in the config")
    for joint in joints:
        backend.spin(joint, speed, source=SOURCE_CLI)


def _hold(
    backend: RosBackend,
    seconds: float | None,
    *,
    every: float = 0.0,
    on_tick: Callable[[], None] | None = None,
) -> None:
    """Wait `seconds` (None: until Ctrl+C) while re-sending speeds to the robot
    node.

    Runs at the telemetry rate whatever `every` is, because a print interval
    longer than the robot's command timeout would otherwise let the watchdog
    stop the wheels.
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
        backend.tick()
        time.sleep(period)


def _print_states(
    backend: RosBackend, joints: Sequence[str], out: Output
) -> None:
    states = {s["name"]: s for s in backend.snapshot()}
    for joint in joints:
        s = states[joint]
        if s["online"]:
            out(f"{joint}: position={s['position']} speed={s['speed']}")
        else:
            out(f"{joint}: offline ({s['error'] or 'no data yet'})")


def _stop_and_release(backend: RosBackend, joints: Sequence[str]) -> None:
    # A single stop is a fire-and-forget publish a busy robot node can miss, so
    # repeat it until the node goes away.
    period = 1.0 / backend.config.telemetry_hz
    deadline = time.monotonic() + STOP_FLUSH_TIME
    try:
        while True:
            for joint in joints:
                backend.stop(joint, source=SOURCE_CLI)
            if time.monotonic() >= deadline:
                break
            time.sleep(period)
    finally:
        backend.release()
