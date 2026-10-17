"""Thread-safe robot state on one ServoBus, shared by the web server and the ROS bridge."""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    MAX_ACCELERATION,
    MODE_WHEEL,
    ServoBus,
    ServoError,
)
from talos_servo.config import RobotConfig, ServoConfig

log = logging.getLogger(__name__)

POSITION = "position"
WHEEL = "wheel"
MODES = (POSITION, WHEEL)

# Who sent a servo its last move, spin or stop. Shown in the web UI next to the sliders.
SOURCE_WEB = "web"  # a browser; the client id tells browsers apart
SOURCE_CLI = "cli"  # talos-servo spin/monitor --ros
SOURCE_ROS = "ros"  # any other ROS node
SOURCE_BRIDGE = "bridge"  # the servo bridge itself, e.g. its command timeout stopping a wheel
SOURCES = (SOURCE_WEB, SOURCE_CLI, SOURCE_ROS, SOURCE_BRIDGE)
# Ages are shown to people, so tenths of a second are plenty.
AGE_DECIMALS = 1

# Load, voltage, temperature and torque change slowly; read them every Nth poll only.
SLOW_POLL_EVERY = 10


class CommandError(ValueError):
    """A UI command does not make sense in the servo's current state."""


@dataclass
class ServoState:
    id: int
    name: str
    min_pos: int
    max_pos: int
    online: bool = False
    mode: str = POSITION
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
    command_source: str | None = None
    command_client: str | None = None
    # Monotonic time of the last command, local to this process; never sent as is.
    command_at: float | None = None

    @classmethod
    def from_config(cls, cfg: ServoConfig) -> ServoState:
        return cls(id=cfg.id, name=cfg.name, min_pos=cfg.min_pos, max_pos=cfg.max_pos)


def _clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def public_state(s: ServoState, now: float) -> dict[str, Any]:
    """Servo state as sent to browsers: `command_*` becomes `last_command` with an age."""
    data = asdict(s)
    source = data.pop("command_source")
    client = data.pop("command_client")
    at = data.pop("command_at")
    data["last_command"] = (
        None
        if source is None or at is None
        else {"source": source, "client": client, "age": round(max(0.0, now - at), AGE_DECIMALS)}
    )
    return data


def record_command(s: ServoState, source: str, client: str | None, now: float) -> None:
    s.command_source, s.command_client, s.command_at = source, client, now


class RobotController:
    """Owns the bus. Every bus access holds one lock because the SDK is not thread-safe.

    Bus failures on a servo mark it offline with the error text instead of raising,
    so one unplugged servo never stops telemetry for the others.
    """

    def __init__(
        self,
        bus: ServoBus,
        config: RobotConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.bus = bus
        self.clock = clock
        self.config = config
        self.servos = {cfg.id: ServoState.from_config(cfg) for cfg in config.servos}
        self._lock = threading.Lock()
        self._polls = 0

    # --- lifecycle -------------------------------------------------------
    def open(self) -> None:
        with self._lock:
            self.bus.open()
            for s in self.servos.values():
                self._try(s, lambda s=s: self._read_full(s))
                log.info("servo %s (%s): online=%s mode=%s", s.id, s.name, s.online, s.mode)

    def close(self) -> None:
        with self._lock:
            self._stop_wheels()
            self.bus.close()

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            now = self.clock()
            return [public_state(s, now) for s in self.servos.values()]

    # --- telemetry -------------------------------------------------------
    def poll(self) -> None:
        self._polls += 1
        slow = self._polls % SLOW_POLL_EVERY == 0
        with self._lock:
            for s in self.servos.values():
                # Offline servos cost a timeout each, so only retry them on slow polls.
                if not s.online and not slow:
                    continue
                self._try(s, lambda s=s: self._read_full(s) if slow else self._read_fast(s))

    def _read_fast(self, s: ServoState) -> None:
        s.position, s.speed = self.bus.read_position_speed(s.id)

    def _read_full(self, s: ServoState) -> None:
        mode = WHEEL if self.bus.read_mode(s.id) == MODE_WHEEL else POSITION
        if mode != s.mode:
            s.mode, s.target, s.target_speed = mode, None, 0
        s.torque = self.bus.read_torque(s.id)
        self._read_fast(s)
        status = self.bus.read_status(s.id)
        s.load, s.voltage, s.temp = status.load, status.voltage, status.temperature
        s.current, s.moving = status.current, status.moving

    # --- commands --------------------------------------------------------
    def set_mode(self, servo_id: int, mode: str) -> None:
        if mode not in MODES:
            raise CommandError(f"unknown mode {mode!r}, expected one of {MODES}")
        with self._lock:
            s = self._get(servo_id)

            def apply() -> None:
                if s.mode == WHEEL:
                    self.bus.stop(s.id)
                if mode == WHEEL:
                    self.bus.set_wheel_mode(s.id)
                else:
                    self.bus.set_position_mode(s.id)
                s.mode, s.target_speed = mode, 0
                s.target = s.position if mode == POSITION else None

            self._command(s, apply)

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        with self._lock:
            s = self._get(servo_id)

            def apply() -> None:
                if not enabled and s.mode == WHEEL:
                    self.bus.stop(s.id)
                    s.target_speed = 0
                self.bus.set_torque(s.id, enabled)
                s.torque = enabled

            self._command(s, apply)

    def move(
        self,
        servo_id: int,
        position: int,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode != POSITION:
                raise CommandError(f"servo {servo_id} is in speed mode, switch to position first")
            target = _clamp(position, s.min_pos, s.max_pos)
            speed = _clamp(speed, 1, self.config.max_speed)
            acceleration = _clamp(acceleration, 0, MAX_ACCELERATION)

            def apply() -> None:
                self.bus.move_to(s.id, target, speed, acceleration)
                s.target = target
                s.torque = True  # writing a goal turns torque on
                record_command(s, source, client, self.clock())

            self._command(s, apply)

    def spin(
        self,
        servo_id: int,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode != WHEEL:
                raise CommandError(f"servo {servo_id} is in position mode, switch to speed first")
            speed = _clamp(speed, -self.config.max_speed, self.config.max_speed)
            acceleration = _clamp(acceleration, 0, MAX_ACCELERATION)

            def apply() -> None:
                self.bus.set_speed(s.id, speed, acceleration)
                s.target_speed = speed
                if speed:
                    s.torque = True
                record_command(s, source, client, self.clock())

            self._command(s, apply)

    def stop(self, servo_id: int, *, source: str = SOURCE_WEB, client: str | None = None) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode == WHEEL:

                def apply() -> None:
                    self.bus.stop(s.id)
                    s.target_speed = 0
                    record_command(s, source, client, self.clock())

                self._command(s, apply)

    def stop_all(self, *, source: str = SOURCE_WEB, client: str | None = None) -> None:
        with self._lock:
            self._stop_wheels(source, client)

    # --- helpers ---------------------------------------------------------
    def _stop_wheels(self, source: str | None = None, client: str | None = None) -> None:
        """Best effort: stop every wheel-mode servo, keep going if one fails."""
        for s in self.servos.values():
            if s.mode != WHEEL:
                continue
            try:
                self.bus.stop(s.id)
                s.target_speed = 0
                if source is not None:
                    record_command(s, source, client, self.clock())
            except ServoError as exc:
                log.warning("could not stop servo %s: %s", s.id, exc)
                s.error = str(exc)

    def _get(self, servo_id: int) -> ServoState:
        try:
            return self.servos[servo_id]
        except KeyError:
            raise CommandError(f"unknown servo id {servo_id}") from None

    @staticmethod
    def _try(s: ServoState, action) -> bool:
        try:
            action()
        except ServoError as exc:
            s.online, s.error = False, str(exc)
            return False
        s.online, s.error = True, None
        return True

    def _command(self, s: ServoState, action) -> None:
        """Run a user command; on failure record it on the servo and re-raise for the UI."""
        try:
            action()
        except ServoError as exc:
            s.error = str(exc)
            raise
        s.online, s.error = True, None
