"""`talos-servo spin/monitor/drive --ros`: the same commands, through the robot
node. Also `talos-servo calibrate-bend`, which only exists through ROS.

The robot node stays the only program on the serial port, so these can run next
to the web UI (`talos-servo web --ros`) and other ROS nodes, and the UI shows
what they do. Each command stops only the joints it started, not wheels someone
else is driving.
"""

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable, Sequence

from talos_servo import teleop
from talos_servo.bus import ServoError
from talos_servo.robot import SOURCE_CLI, CommandError
from talos_servo.ros.names import (
    CALIBRATE_CANCEL,
    CALIBRATE_FOLD,
    CALIBRATE_OPEN,
    CALIBRATE_RECORD_FLAT,
    CALIBRATE_RECORD_FOLD,
    CALIBRATE_SAVE,
    CALIBRATE_START,
    CALIBRATE_STATUS,
    calibrate_service,
)
from talos_servo.ros.web_backend import SERVICE_TIMEOUT, RosBackend

Output = Callable[[str], None]
LISTENER_POLL_S = 0.05
# How long stop commands are re-sent before the node goes away. If they are all
# lost anyway, the robot's command timeout stops the wheel.
STOP_FLUSH_TIME = 0.2
# Bend calibration keys. Only this command reads them, so F does not clash
# with the R/F keys `drive` keeps for #95.
JOG_OPEN_KEY = "o"
JOG_FOLD_KEY = "f"
# How often the calibration asks for the hinge position. Well below the
# bridge's idle timeout, so it also keeps the session alive.
CALIBRATION_STATUS_S = 0.5
CALIBRATION_KEY_S = 0.05
CALIBRATION_INTRO = (
    "Calibrating {joint}. It moves slowly, keep hands clear. Controller "
    "goals for {joint} are ignored until you save or cancel.\n"
    "O opens, F folds (hold to keep moving), Enter records, X cancels."
)
CALIBRATION_PROMPTS = (
    "Step 1/3: open the hinge until the robot lies flat (180 deg), then "
    "press Enter.",
    "Step 2/3: fold until the front and rear wheels touch, then press Enter.",
    "Step 3/3: press Enter to save and apply, or X to cancel.",
)


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

    N and L ask the mode node for wall nearby and release. R and F open and
    close the hinge through the expansion node while held. Space also turns
    STOP on for every input and Enter releases it. The mode decides which
    drive keys work, and a status line shows the mode, why it changed and
    which input drives. Quitting stops only this input, so the web keeps
    driving.
    """
    backend.open()
    calls = _ModeCalls(out)
    release_stop = calls.stop_call(backend.release_stop)
    try:
        if not _wait_for_listener(backend):
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
                teleop.WALL_NEARBY_KEY: calls.mode_call(
                    backend.wall_nearby, "wall nearby"
                ),
                teleop.RELEASE_KEY: calls.mode_call(
                    backend.release_walls, "release"
                ),
                **dict.fromkeys(teleop.RELEASE_STOP_KEYS, release_stop),
            },
            allowed=lambda: teleop.allowed_keys(backend.robot_status().mode),
            on_stop=calls.stop_call(lambda: backend.set_stop(True)),
            hinge=backend.move_hinge,
            hinge_rate=backend.config.expansion.max_rate_rad_s,
        )
    finally:
        calls.close(SERVICE_TIMEOUT)
        backend.release()


def calibrate_bend_over_ros(
    backend: RosBackend,
    read_key: teleop.ReadKey,
    joint: str,
    out: Output = print,
    clock: Callable[[], float] = time.monotonic,
) -> bool:
    """Walk the user through the bend calibration on the hardware bridge.

    Uses the service replies only: the joint states carry the hinge as an
    angle, which is unknown while calibrating. Anything but a save cancels
    the session, including Ctrl+C. True when saved.
    """
    backend.open()
    try:

        def call(step: str) -> str:
            return backend.trigger(calibrate_service(joint, step))

        out(CALIBRATION_INTRO.format(joint=joint))
        out(call(CALIBRATE_START))
        saved = False
        try:
            saved = _calibration_loop(call, read_key, out, clock)
        finally:
            if not saved:
                _report(lambda: out(call(CALIBRATE_CANCEL)), out)
        return saved
    finally:
        backend.release()


def _calibration_loop(
    call: Callable[[str], str],
    read_key: teleop.ReadKey,
    out: Output,
    clock: Callable[[], float],
) -> bool:
    records = (CALIBRATE_RECORD_FLAT, CALIBRATE_RECORD_FOLD, CALIBRATE_SAVE)
    stage = 0
    out(CALIBRATION_PROMPTS[stage])
    last_status = None
    next_status = clock()
    while True:
        key = read_key(CALIBRATION_KEY_S)
        if key is not None:
            lower = key.lower()
            if lower == teleop.QUIT_KEY:
                return False
            if lower in (JOG_OPEN_KEY, JOG_FOLD_KEY):
                step = (
                    CALIBRATE_OPEN if lower == JOG_OPEN_KEY else CALIBRATE_FOLD
                )
                _report(lambda step=step: call(step), out)
            elif key in teleop.RELEASE_STOP_KEYS:
                record = records[stage]
                message = _report(lambda record=record: call(record), out)
                if message is not None:
                    out(message)
                    if record == CALIBRATE_SAVE:
                        return True
                    stage += 1
                    out(CALIBRATION_PROMPTS[stage])
        if clock() >= next_status:
            next_status = clock() + CALIBRATION_STATUS_S
            status = _report(lambda: call(CALIBRATE_STATUS), out)
            if status is not None and status != last_status:
                out(status)
                last_status = status


def _report(action: Callable[[], str | None], out: Output) -> str | None:
    """Run one service call; print a failure and go on."""
    try:
        return action()
    except (CommandError, ServoError) as exc:
        out(f"error: {exc}")
        return None


def _wait_for_listener(backend: RosBackend) -> bool:
    """True once something reads our cmd_vel topic.

    Discovery takes a moment after open(), so give twist_mux up to
    `wait_timeout` to show up before saying nobody listens.
    """
    deadline = time.monotonic() + backend.wait_timeout
    while not backend.has_cmd_vel_listener():
        if time.monotonic() > deadline:
            return False
        time.sleep(LISTENER_POLL_S)
    return True


class _ModeCalls:
    """Mode node calls off the key loop, so a slow controller switch does
    not freeze the keys. A failure is printed and driving goes on.

    STOP and release STOP go through one queue in key order and never wait
    for a mode change. Wall nearby and release run one at a time; a press
    while one is running is ignored with a note.
    """

    def __init__(self, out: Output) -> None:
        self._out = out
        self._stops: queue.Queue[Callable[[], None] | None] = queue.Queue()
        self._stop_thread = threading.Thread(
            target=self._drain, name="keyboard-stop", daemon=True
        )
        self._stop_thread.start()
        self._mode_thread: threading.Thread | None = None

    def stop_call(self, action: Callable[[], None]) -> Callable[[], None]:
        return lambda: self._stops.put(action)

    def mode_call(
        self, action: Callable[[], None], what: str
    ) -> Callable[[], None]:
        def start() -> None:
            if self._mode_thread is not None and self._mode_thread.is_alive():
                self._out(f"note: mode node still busy, {what} ignored")
                return
            self._mode_thread = threading.Thread(
                target=self._report,
                args=(action,),
                name="keyboard-mode-call",
                daemon=True,
            )
            self._mode_thread.start()

        return start

    def close(self, timeout: float) -> None:
        """Let queued STOP calls finish, then wait for a running mode
        call, at most `timeout` each."""
        self._stops.put(None)
        self._stop_thread.join(timeout)
        if self._mode_thread is not None:
            self._mode_thread.join(timeout)

    def _drain(self) -> None:
        while (action := self._stops.get()) is not None:
            self._report(action)

    def _report(self, action: Callable[[], None]) -> None:
        try:
            action()
        except (CommandError, ServoError) as exc:
            self._out(f"error: {exc}")


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
