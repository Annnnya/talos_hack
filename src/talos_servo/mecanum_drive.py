"""Control loop for the mecanum base: one owner of the servo bus, no ROS dependency.

Each tick first sends all wheel speeds in one sync-write packet, then reads telemetry from
at most one servo. Commands therefore never wait behind a burst of reads, and a slow or
missing servo can delay the next tick by one read timeout at most.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from talos_servo.bus import MAX_ACCELERATION, ServoBus, ServoTelemetry
from talos_servo.mecanum import mecanum_wheel_speeds, wheel_accelerations

DEFAULT_COMMAND_TIMEOUT_S = 0.5
# Telemetry reads that may fail in a row on one servo before the drive gives up on it.
DEFAULT_MAX_READ_FAILURES = 5


@dataclass(frozen=True)
class Kinematics:
    wheelbase_m: float
    track_width_m: float
    steps_per_meter: float
    max_speed: int
    wheel_directions: tuple[int, ...]


@dataclass(frozen=True)
class TickResult:
    # Index of the wheel read this tick, its telemetry (None if the read failed) and the error.
    read_index: int | None = None
    telemetry: ServoTelemetry | None = None
    read_error: Exception | None = None
    # True on the tick where the watchdog stopped the wheels for lack of commands.
    timed_out: bool = False


class DriveFault(RuntimeError):
    """The drive can no longer be trusted to follow commands and must be disabled."""


class MecanumDrive:
    def __init__(
        self,
        bus: ServoBus,
        wheel_ids: Sequence[int],
        kinematics: Kinematics,
        acceleration: int,
        *,
        command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S,
        max_read_failures: int = DEFAULT_MAX_READ_FAILURES,
    ) -> None:
        if not 1 <= acceleration <= MAX_ACCELERATION:
            raise ValueError(f"acceleration must be within 1..{MAX_ACCELERATION}")
        self.bus = bus
        self.wheel_ids = tuple(wheel_ids)
        self.kinematics = kinematics
        self.acceleration = acceleration
        self.command_timeout_s = command_timeout_s
        self.max_read_failures = max_read_failures
        self._command: tuple[float, float, float] | None = None
        self._command_at: float | None = None
        self._speeds = (0,) * len(self.wheel_ids)
        self._accelerations = (acceleration,) * len(self.wheel_ids)
        self._read_failures = [0] * len(self.wheel_ids)
        self._next_read = 0
        self._reads_requested = 0

    @property
    def speeds(self) -> tuple[int, ...]:
        """Wheel speeds sent by the last tick, in steps/s."""
        return self._speeds

    @property
    def moving(self) -> bool:
        return any(self._speeds)

    def set_command(self, linear_x: float, linear_y: float, angular_z: float, now: float) -> None:
        """Store the latest body velocity; the next tick sends it."""
        self._command = (linear_x, linear_y, angular_z)
        self._command_at = now

    def clear_command(self) -> None:
        self._command = None
        self._command_at = None

    def request_reads(self, count: int) -> None:
        """Ask the next `count` ticks to read telemetry, one servo per tick, round robin.

        Requests do not stack up, so late ticks never build a backlog of reads.
        """
        self._reads_requested = max(self._reads_requested, count)

    def tick(self, now: float) -> TickResult:
        """Send wheel speeds, then read at most one servo. Raises DriveFault on a bus failure."""
        timed_out = self._expire_command(now)
        self._write_speeds(self._target_speeds())
        if not self._reads_requested:
            return TickResult(timed_out=timed_out)
        self._reads_requested -= 1
        index = self._next_read
        self._next_read = (index + 1) % len(self.wheel_ids)
        try:
            telemetry = self.bus.read_telemetry(self.wheel_ids[index])
        except Exception as exc:
            self._read_failures[index] += 1
            if self._read_failures[index] >= self.max_read_failures:
                raise DriveFault(
                    f"servo {self.wheel_ids[index]} failed {self._read_failures[index]} "
                    f"reads in a row: {exc}"
                ) from exc
            return TickResult(read_index=index, read_error=exc, timed_out=timed_out)
        self._read_failures[index] = 0
        return TickResult(read_index=index, telemetry=telemetry, timed_out=timed_out)

    def stop(self) -> list[str]:
        """Stop every wheel, falling back to one packet per servo. Returns the errors."""
        self.clear_command()
        self._speeds = (0,) * len(self.wheel_ids)
        try:
            self.bus.stop_many(self.wheel_ids, self.acceleration)
            return []
        except Exception as exc:
            errors = [f"sync stop failed: {exc}"]
        for servo_id in self.wheel_ids:
            try:
                self.bus.stop(servo_id, self.acceleration)
            except Exception as exc:
                errors.append(f"could not stop servo {servo_id}: {exc}")
        return errors

    def _expire_command(self, now: float) -> bool:
        if self._command_at is None or now - self._command_at <= self.command_timeout_s:
            return False
        self.clear_command()
        return self.moving

    def _target_speeds(self) -> tuple[int, ...]:
        if self._command is None:
            return (0,) * len(self.wheel_ids)
        k = self.kinematics
        return mecanum_wheel_speeds(
            *self._command,
            wheelbase_m=k.wheelbase_m,
            track_width_m=k.track_width_m,
            steps_per_meter=k.steps_per_meter,
            max_speed=k.max_speed,
            wheel_directions=k.wheel_directions,
        )

    def _write_speeds(self, speeds: tuple[int, ...]) -> None:
        # Keep the previous ramp rates when the target has not changed, so wheels still ramping
        # stay in step. The packet is resent every tick since a sync write gets no reply.
        if speeds != self._speeds:
            self._accelerations = wheel_accelerations(self._speeds, speeds, self.acceleration)
        try:
            self.bus.set_speeds(
                dict(zip(self.wheel_ids, speeds, strict=True)),
                dict(zip(self.wheel_ids, self._accelerations, strict=True)),
            )
        except Exception as exc:
            raise DriveFault(f"wheel command failed: {exc}") from exc
        self._speeds = speeds
