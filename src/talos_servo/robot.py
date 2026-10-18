"""The robot: every enabled joint of `config/robot.yaml` on one servo bus.

No ROS here.

`Robot` is the only code that talks to the servos. The ROS node, the web server
and the CLI all drive the robot through it, so limits, motor direction, the
command watchdog and stopping live in exactly one place.

Speed joints are written from `tick()`: one sync-write packet with every wheel
speed, one sync-read packet with every wheel's position and speed (fresh wheel
odometry each tick), then one full servo read, round robin. Call it at
`config.control_hz`.

Safe state (`hold_safe`): every wheel at speed 0 with torque on, so it brakes,
and every position joint keeps its last goal with torque on. Torque is never
turned off on the way, so nothing goes limp while the robot hangs between two
walls. A bus fault (a wheel write fails, or a joint that was online stops
answering) puts the robot in the safe state and latches it: commands that
would move refuse with `CommandError` until the lost joints answer again and
wheel writes work for `FAULT_CLEAR_S`. Stops are always accepted. A failed
wheel sync read does not latch on its own: a wheel that stops answering is
caught by the round-robin read.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    MAX_ACCELERATION,
    MAX_SPEED,
    MODE_POSITION,
    MODE_WHEEL,
    ServoBus,
    ServoError,
    joint_position,
)
from talos_servo.config import POSITION, SPEED, JointConfig, RobotConfig
from talos_servo.mecanum import (
    body_velocity,
    wheel_accelerations,
    wheel_speeds,
)
from talos_servo.modes import RobotStatus

log = logging.getLogger(__name__)

# Who sent a joint its last move, spin or stop. Shown in the web UI next to the
# sliders.
SOURCE_WEB = "web"  # a browser; the client id tells browsers apart
SOURCE_CLI = "cli"  # talos-servo commands
SOURCE_ROS = "ros"  # any other ROS node
SOURCE_ROBOT = (
    "robot"  # the robot itself, e.g. its command watchdog stopping a wheel
)
SOURCES = (SOURCE_WEB, SOURCE_CLI, SOURCE_ROS, SOURCE_ROBOT)
# Ages are shown to people, so tenths of a second are plenty.
AGE_DECIMALS = 1

# An offline servo costs a read timeout, so only retry it every Nth tick.
RETRY_OFFLINE_EVERY = 25
# How long the bus must stay healthy before a fault unlatches. Longer than
# one offline retry period at the usual control rate, so a joint that answers
# once and drops again keeps the brake on.
FAULT_CLEAR_S = 1.0


class CommandError(ValueError):
    """A command does not make sense for this joint."""


@dataclass
class Joint:
    """Live state of one joint, in joint units: positive is forward whatever
    `inverted` says."""

    name: str
    id: int
    control: str
    min_pos: int
    max_pos: int
    online: bool = False
    torque: bool = False
    position: int = 0
    speed: int = 0
    load: int = 0
    voltage: float = 0.0
    temp: int = 0
    current: float = 0.0
    moving: bool = False
    target: int | None = None
    target_speed: int = 0
    error: str | None = None
    # The servo's own protection flag (e.g. overload) while it still answers.
    fault: str | None = None
    command_source: str | None = None
    command_client: str | None = None
    # Monotonic time of the last command, local to this process; never sent as
    # is.
    command_at: float | None = None

    @classmethod
    def from_config(cls, cfg: JointConfig) -> Joint:
        return cls(
            name=cfg.name,
            id=cfg.id,
            control=cfg.control,
            min_pos=cfg.min_pos,
            max_pos=cfg.max_pos,
        )


def clamp(value: float, low: float, high: float) -> Any:
    return max(low, min(high, value))


def check_move(
    j: Joint, position: int, speed: int, max_speed: int
) -> tuple[int, int]:
    """Validate a move for this joint; return the clamped target and speed."""
    if j.control != POSITION:
        raise CommandError(
            f"{j.name} is a speed joint, it cannot move to a position"
        )
    return clamp(position, j.min_pos, j.max_pos), clamp(speed, 1, max_speed)


def check_spin(j: Joint, speed: int, max_speed: int) -> int:
    """Validate a spin for this joint; return the clamped speed."""
    if j.control != SPEED:
        raise CommandError(f"{j.name} is a position joint, it cannot spin")
    return clamp(speed, -max_speed, max_speed)


def public_state(j: Joint, now: float) -> dict[str, Any]:
    """Joint state for clients; `command_*` becomes `last_command`."""
    data = asdict(j)
    source = data.pop("command_source")
    client = data.pop("command_client")
    at = data.pop("command_at")
    data["last_command"] = (
        None
        if source is None or at is None
        else {
            "source": source,
            "client": client,
            "age": round(max(0.0, now - at), AGE_DECIMALS),
        }
    )
    return data


def record_command(
    j: Joint, source: str, client: str | None, now: float
) -> None:
    j.command_source, j.command_client, j.command_at = source, client, now


class Robot:
    """Owns the bus. Every bus access holds one lock because the SDK is not
    thread-safe.

    Bus failures on a joint mark it offline with the error text instead of
    raising from `tick()`, so one unplugged servo never stops the others.
    """

    def __init__(
        self,
        bus: ServoBus,
        config: RobotConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.bus = bus
        self.config = config
        self.clock = clock
        self._configs = {c.name: c for c in config.enabled_joints}
        self.joints = {
            c.name: Joint.from_config(c) for c in config.enabled_joints
        }
        self._lock = threading.Lock()
        # Speed joints whose last command expires (ROS, drive): name -> time it
        # was sent.
        self._expires: dict[str, float] = {}
        self._acceleration = {
            name: DEFAULT_ACCELERATION for name in self._speed_joints()
        }
        self._ticks = 0
        self._next_read = 0
        # Last wheel write and read errors, so a stuck fault logs once, not
        # every tick.
        self._speed_error: str | None = None
        self._wheel_read_error: str | None = None
        # Latched bus fault (see the module docstring): why, the joints that
        # went offline and must answer again, and since when all is well.
        self._fault: str | None = None
        self._lost: set[str] = set()
        self._healthy_since: float | None = None

    # --- lifecycle -------------------------------------------------------
    def open(self) -> None:
        """Open the bus, put each joint in its configured mode and start from a
        stopped robot."""
        with self._lock:
            self._fault, self._healthy_since = None, None
            self._lost.clear()
            self.bus.open()
            for j in self.joints.values():
                self._try(j, lambda j=j: self._setup(j))
                log.info("%s (servo %s): online=%s", j.name, j.id, j.online)
            self._stop_speed_joints()

    def close(self) -> None:
        """Leave the robot in the safe state and close the bus.

        Safe to call more than once.
        """
        with self._lock:
            try:
                self._hold_safe("shutdown", level=logging.INFO)
            finally:
                self.bus.close()

    def hold_safe(self, reason: str) -> None:
        """Brake every wheel, keep every position joint on its goal and latch
        the fault. Never turns torque off and never raises."""
        with self._lock:
            self._hold_safe(reason)

    @property
    def fault(self) -> str | None:
        """Why the robot holds the safe state, or None while it is free."""
        return self._fault

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            now = self.clock()
            return [public_state(j, now) for j in self.joints.values()]

    # --- control loop ----------------------------------------------------
    def tick(self) -> None:
        """One control step: watchdog, all wheel speeds, all wheel positions
        and speeds, the hinge in full, one other full servo read."""
        with self._lock:
            self._ticks += 1
            self._expire(self.clock())
            self._write_speeds()
            self._read_wheels()
            self._read_hinge()
            self._read_next()
            self._update_fault()

    # --- commands --------------------------------------------------------
    def move(
        self,
        joint: str,
        position: int,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        with self._lock:
            j = self._get(joint)
            target, speed = check_move(
                j, position, speed, self.config.max_speed
            )
            self._refuse_while_faulted(j.name)
            acceleration = clamp(acceleration, 0, MAX_ACCELERATION)

            def apply() -> None:
                self.bus.move_to(
                    j.id, self._position_to_bus(j, target), speed, acceleration
                )
                j.target = target
                j.torque = True  # writing a goal turns torque on
                record_command(j, source, client, self.clock())

            self._command(j, apply)

    def spin(
        self,
        joint: str,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
        expires: bool = False,
        capped: bool = True,
    ) -> None:
        """Set a speed joint's speed; the next `tick()` sends it.

        With `expires`, the joint stops unless the command is repeated within
        `config.command_timeout_s` (streamed commands such as ROS velocity
        commands). With `capped` false the speed is clamped to the servo's
        range only, not `config.max_speed`: for callers that enforce their
        own limits (the base wheels under mecanum_drive_controller).
        """
        limit = self.config.max_speed if capped else MAX_SPEED
        with self._lock:
            j = self._get(joint)
            speed = check_spin(j, speed, limit)
            if speed:
                self._refuse_while_faulted(j.name)
            self._acceleration[joint] = clamp(acceleration, 1, MAX_ACCELERATION)
            self._set_speed(j, speed, expires, source, client)

    def drive(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        *,
        source: str = SOURCE_ROS,
        client: str | None = None,
    ) -> None:
        """Drive the mecanum base (m/s, m/s, rad/s).

        Stops unless repeated, like /cmd_vel.
        """
        base = self.config.base
        if base is None:
            raise CommandError(
                "the config has no base section, so the robot cannot drive"
            )
        speeds = wheel_speeds(
            finite_clamp(linear_x, base.max_linear_speed),
            finite_clamp(linear_y, base.max_linear_speed),
            finite_clamp(angular_z, base.max_angular_speed),
            wheelbase_m=base.wheelbase_m,
            track_width_m=base.track_width_m,
            steps_per_meter=base.steps_per_meter,
            max_speed=self.config.max_speed,
        )
        with self._lock:
            if any(speeds):
                self._refuse_while_faulted("the base")
            wheels = [self.joints[name] for name in base.wheels]
            previous = [j.target_speed for j in wheels]
            ramps = wheel_accelerations(previous, speeds, DEFAULT_ACCELERATION)
            for j, speed, ramp in zip(wheels, speeds, ramps, strict=True):
                self._acceleration[j.name] = ramp
                self._set_speed(j, speed, True, source, client)

    def base_velocity(self) -> tuple[float, float, float]:
        """Measured body velocity (m/s, m/s, rad/s) from the wheel speeds."""
        base = self.config.base
        if base is None:
            raise CommandError("the config has no base section")
        with self._lock:
            fl, fr, rl, rr = (self.joints[n].speed for n in base.wheels)
        return body_velocity(
            fl,
            fr,
            rl,
            rr,
            wheelbase_m=base.wheelbase_m,
            track_width_m=base.track_width_m,
            steps_per_meter=base.steps_per_meter,
        )

    def stop(
        self, joint: str, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Stop a speed joint now.

        Position joints hold their target, so this does nothing.
        """
        with self._lock:
            j = self._get(joint)
            if j.control != SPEED:
                return

            def apply() -> None:
                self.bus.stop(j.id)
                self._set_speed(j, 0, False, source, client)

            self._command(j, apply)

    def stop_all(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Best effort: stop every speed joint. Never raises."""
        with self._lock:
            self._stop_speed_joints(source, client)

    def stop_own(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Stop what this owner commanded: on the bus that is every speed
        joint. Never raises."""
        self.stop_all(source=source, client=client)

    def stopped(self) -> bool | None:
        """No twist_mux on the direct bus, so no STOP lock."""
        return None

    def release_stop(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        raise CommandError("releasing STOP needs `talos-servo web --ros`")

    def robot_status(self) -> RobotStatus:
        """No mode node or twist_mux on the direct bus, so nothing to
        report."""
        return RobotStatus(None, None, None)

    def wall_nearby(self) -> None:
        raise CommandError("wall nearby needs `talos-servo web --ros`")

    def release_walls(self) -> None:
        raise CommandError("release needs `talos-servo web --ros`")

    def move_hinge(self, rate: float) -> None:
        raise CommandError("the hinge buttons need `talos-servo web --ros`")

    def hinge_hold(self, enabled: bool, setpoint_a: float) -> None:
        raise CommandError("the hinge hold needs `talos-servo web --ros`")

    def hold_state(self) -> dict[str, Any] | None:
        return None

    def climb(self, speed: float) -> None:
        raise CommandError("climbing needs `talos-servo web --ros`")

    def hinge_state(self) -> dict[str, Any] | None:
        """The hinge readout comes from the hardware bridge, over ROS."""
        return None

    def set_torque(self, joint: str, enabled: bool) -> None:
        with self._lock:
            j = self._get(joint)

            def apply() -> None:
                if not enabled and j.control == SPEED:
                    self.bus.stop(j.id)
                    j.target_speed = 0
                    self._expires.pop(j.name, None)
                self.bus.set_torque(j.id, enabled)
                j.torque = enabled

            self._command(j, apply)

    # --- helpers (call with _lock held) ----------------------------------
    def _speed_joints(self) -> list[str]:
        return [name for name, c in self._configs.items() if c.control == SPEED]

    def _set_speed(
        self,
        j: Joint,
        speed: int,
        expires: bool,
        source: str,
        client: str | None,
    ) -> None:
        now = self.clock()
        j.target_speed = speed
        if speed:
            j.torque = True
        if expires and speed:
            self._expires[j.name] = now
        else:
            self._expires.pop(j.name, None)
        record_command(j, source, client, now)

    def _expire(self, now: float) -> None:
        for name, sent_at in list(self._expires.items()):
            if now - sent_at > self.config.command_timeout_s:
                log.warning(
                    "%s: no command for %.1f s, stopping", name, now - sent_at
                )
                self._set_speed(self.joints[name], 0, False, SOURCE_ROBOT, None)

    def _write_speeds(self) -> None:
        # Resent every tick: a sync write gets no reply, so a lost packet is
        # fixed by the next.
        names = self._speed_joints()
        if not names:
            return
        wheels = [self.joints[n] for n in names]
        ids = {j.id: self._speed_to_bus(j, j.target_speed) for j in wheels}
        ramps = {j.id: self._acceleration[j.name] for j in wheels}
        try:
            self.bus.set_speeds(ids, ramps)
        except ServoError as exc:
            if str(exc) != self._speed_error:
                log.error("wheel speed write failed: %s", exc)
                self._speed_error = str(exc)
            for n in names:
                self.joints[n].error = str(exc)
            if self._fault is None:
                self._hold_safe(f"wheel write error: {exc}")
            return
        if self._speed_error is not None:
            log.info("wheel speed writes work again")
            self._speed_error = None

    def _read_wheels(self) -> None:
        """Position and speed of every online speed joint in one packet.

        Online state, errors and the safe state stay with the round robin
        full read, so a servo missing here is only skipped.
        """
        wheels = {
            self.joints[n].id: self.joints[n]
            for n in self._speed_joints()
            if self.joints[n].online
        }
        if not wheels:
            return
        try:
            readings = self.bus.read_positions_speeds(list(wheels))
        except ServoError as exc:
            if str(exc) != self._wheel_read_error:
                log.warning("wheel sync read failed: %s", exc)
                self._wheel_read_error = str(exc)
            return
        if self._wheel_read_error is not None:
            log.info("wheel sync reads work again")
            self._wheel_read_error = None
        for servo_id, (position, speed) in readings.items():
            j = wheels[servo_id]
            j.position = self._position_from_bus(j, position)
            j.speed = self._speed_to_bus(j, speed)

    def _fast_hinge(self) -> Joint | None:
        """The hinge joint while online: read in full every tick, since the
        hinge current hold (#142) runs on its current."""
        hinge = self.config.hinge
        if hinge is None:
            return None
        j = self.joints.get(hinge.joint)
        return j if j is not None and j.online else None

    def _read_hinge(self) -> None:
        j = self._fast_hinge()
        if j is not None:
            self._try(j, lambda: self._read(j))

    def _read_next(self) -> None:
        """Read one joint, round robin, skipping offline joints except on retry
        ticks. An online hinge is read every tick instead."""
        fast = self._fast_hinge()
        names = [n for n in self.joints if fast is None or n != fast.name]
        if not names:
            return
        retry = self._ticks % RETRY_OFFLINE_EVERY == 0
        for _ in names:
            self._next_read %= len(names)
            j = self.joints[names[self._next_read]]
            self._next_read = (self._next_read + 1) % len(names)
            if j.online:
                self._try(j, lambda j=j: self._read(j))
                return
            if retry:
                self._try(j, lambda j=j: self._setup(j))
                return

    def _setup(self, j: Joint) -> None:
        """Bring a joint online: ping it, set its mode, read its state."""
        self.bus.ping(j.id)
        mode = MODE_WHEEL if j.control == SPEED else MODE_POSITION
        if self.bus.read_mode(j.id) != mode:
            if mode == MODE_WHEEL:
                self.bus.set_wheel_mode(j.id)
            else:
                self.bus.set_position_mode(j.id)
        j.torque = self.bus.read_torque(j.id)
        self._read(j)

    def _read(self, j: Joint) -> None:
        t = self.bus.read_telemetry(j.id)
        j.position = self._position_from_bus(j, t.position)
        j.speed = self._speed_to_bus(
            j, t.speed
        )  # flipping a sign is its own inverse
        j.load = self._speed_to_bus(j, t.status.load)
        j.voltage, j.temp = t.status.voltage, t.status.temperature
        j.current, j.moving = t.status.current, t.status.moving
        if t.fault != j.fault:
            if t.fault:
                log.warning("%s reports %s", j.name, t.fault)
            else:
                log.info("%s fault cleared", j.name)
        j.fault = t.fault

    def _stop_speed_joints(
        self, source: str | None = None, client: str | None = None
    ) -> None:
        names = self._speed_joints()
        self._expires.clear()
        for name in names:
            j = self.joints[name]
            j.target_speed = 0
            if source is not None:
                record_command(j, source, client, self.clock())
        if not names:
            return
        try:
            self.bus.stop_many([self.joints[n].id for n in names])
            return
        except Exception as exc:
            log.error("sync stop failed, stopping one by one: %s", exc)
        for name in names:
            j = self.joints[name]
            try:
                self.bus.stop(j.id)
            except Exception as exc:
                log.error("could not stop %s: %s", name, exc)
                j.error = str(exc)

    def _hold_safe(self, reason: str, *, level: int = logging.ERROR) -> None:
        """Safe state, see the module docstring. Idempotent; only a new
        fault is logged."""
        if self._fault is None:
            log.log(level, "safe state (%s): wheels braked, goals held", reason)
            self._fault = reason
        self._healthy_since = None
        self._stop_speed_joints(SOURCE_ROBOT)

    def _update_fault(self) -> None:
        """Unlatch once the lost joints answer and wheel writes work for
        FAULT_CLEAR_S."""
        if self._fault is None:
            return
        healthy = self._speed_error is None and all(
            self.joints[n].online for n in self._lost
        )
        now = self.clock()
        if not healthy:
            self._healthy_since = None
        elif self._healthy_since is None:
            self._healthy_since = now
        elif now - self._healthy_since >= FAULT_CLEAR_S:
            log.info("fault cleared (%s): commands work again", self._fault)
            self._fault, self._healthy_since = None, None
            self._lost.clear()

    def _refuse_while_faulted(self, what: str) -> None:
        if self._fault is not None:
            raise CommandError(
                f"{what} holds the safe state: {self._fault}. It frees "
                f"itself {FAULT_CLEAR_S:g} s after the bus is healthy again"
            )

    def _speed_to_bus(self, j: Joint, speed: int) -> int:
        return -speed if self._configs[j.name].inverted else speed

    def _position_to_bus(self, j: Joint, position: int) -> int:
        return joint_position(position, self._configs[j.name].inverted)

    def _position_from_bus(self, j: Joint, raw: int) -> int:
        return joint_position(raw, self._configs[j.name].inverted)

    def _get(self, joint: str) -> Joint:
        try:
            return self.joints[joint]
        except KeyError:
            raise CommandError(
                f"unknown joint {joint!r}, expected one of "
                f"{sorted(self.joints)}"
            ) from None

    def _try(self, j: Joint, action: Callable[[], None]) -> None:
        """Run a background bus access; a joint that was online and fails
        goes offline and puts the robot in the safe state."""
        try:
            action()
        except ServoError as exc:
            was_online = j.online
            j.online, j.error = False, str(exc)
            if was_online:
                self._lost.add(j.name)
                self._hold_safe(f"{j.name} stopped answering: {exc}")
            return
        j.online, j.error = True, None

    def _command(self, j: Joint, action: Callable[[], None]) -> None:
        """Run a user command; on failure record it on the joint, put the
        robot in the safe state and re-raise for the caller."""
        try:
            action()
        except ServoError as exc:
            j.error = str(exc)
            self._hold_safe(f"{j.name}: {exc}")
            raise
        j.online, j.error = True, None


def finite_clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]; NaN and infinity become 0 (stopped)."""
    return clamp(value, -limit, limit) if math.isfinite(value) else 0.0
