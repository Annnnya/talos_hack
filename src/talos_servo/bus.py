"""Safe wrapper around the Feetech STS/SMS SDK for one serial bus."""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from types import TracebackType
from typing import Any

from scservo_sdk import (
    COMM_SUCCESS,
    SMS_STS_ID,
    SMS_STS_MODE,
    SMS_STS_TORQUE_ENABLE,
    PortHandler,
    sms_sts,
)
from serial import SerialException

log = logging.getLogger(__name__)

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_ACCELERATION = 20

# 254 is the broadcast ID, so the highest assignable servo ID is 253.
MAX_SERVO_ID = 253

MODE_POSITION = 0
MODE_WHEEL = 1

# Goal speed is a 15-bit magnitude plus a sign bit; acceleration is one byte.
MAX_SPEED = 32_767
MAX_ACCELERATION = 254


class ServoError(RuntimeError):
    """A servo bus operation failed."""


class ServoBus:
    """One serial bus of daisy-chained STS/SMS servos.

    Use as a context manager. On exit every servo this bus set spinning is stopped,
    even if the body raised, and the port is closed.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = DEFAULT_BAUDRATE,
        *,
        port_factory: Callable[[str], Any] = PortHandler,
        handler_factory: Callable[[Any], Any] = sms_sts,
    ) -> None:
        self.port_name = port
        self.baudrate = baudrate
        self._port = port_factory(port)
        self._handler = handler_factory(self._port)
        self._moving: set[int] = set()
        self._is_open = False

    def open(self) -> None:
        # The SDK returns False for some failures but lets pyserial exceptions escape for others.
        try:
            opened = self._port.openPort()
        except SerialException as exc:
            raise ServoError(f"could not open serial port {self.port_name}: {exc}") from exc
        if not opened:
            raise ServoError(f"could not open serial port {self.port_name}")
        self._is_open = True
        try:
            baud_ok = self._port.setBaudRate(self.baudrate)
        except SerialException as exc:
            self.close()
            raise ServoError(f"could not set baud rate {self.baudrate}: {exc}") from exc
        if not baud_ok:
            self.close()
            raise ServoError(f"could not set baud rate {self.baudrate} on {self.port_name}")

    def close(self) -> None:
        if not self._is_open:
            return
        try:
            self.stop_all()
        finally:
            self._port.closePort()
            self._is_open = False

    def __enter__(self) -> ServoBus:
        self.open()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def ping(self, servo_id: int) -> int:
        """Return the model number of the servo, or raise if it does not answer."""
        model, comm, error = self._handler.ping(servo_id)
        self._check(comm, error, f"ping servo {servo_id}")
        return model

    def set_wheel_mode(self, servo_id: int) -> None:
        """Switch the servo to continuous rotation (wheel) mode."""
        comm, error = self._handler.write1ByteTxRx(servo_id, SMS_STS_MODE, MODE_WHEEL)
        self._check(comm, error, f"set wheel mode on servo {servo_id}")

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        comm, error = self._handler.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, int(enabled))
        self._check(comm, error, f"set torque {'on' if enabled else 'off'} on servo {servo_id}")

    def set_speed(
        self, servo_id: int, speed: int, acceleration: int = DEFAULT_ACCELERATION
    ) -> None:
        """Set wheel-mode speed in steps/s. Negative values spin the other way."""
        if not -MAX_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"speed must be within +/-{MAX_SPEED}, got {speed}")
        if not 0 <= acceleration <= MAX_ACCELERATION:
            raise ValueError(
                f"acceleration must be within 0..{MAX_ACCELERATION}, got {acceleration}"
            )
        # Track before writing: if the write half-succeeds we still try to stop it later.
        if speed:
            self._moving.add(servo_id)
        comm, error = self._handler.WriteSpec(servo_id, speed, acceleration)
        self._check(comm, error, f"set speed {speed} on servo {servo_id}")
        if not speed:
            self._moving.discard(servo_id)

    def stop(self, servo_id: int, acceleration: int = DEFAULT_ACCELERATION) -> None:
        self.set_speed(servo_id, 0, acceleration)

    def stop_all(self) -> None:
        """Best-effort stop of every servo this bus started. Logs failures instead of raising."""
        for servo_id in sorted(self._moving):
            try:
                self.stop(servo_id)
            except Exception as exc:
                log.warning("could not stop servo %s: %s", servo_id, exc)

    def read_position_speed(self, servo_id: int) -> tuple[int, int]:
        position, speed, comm, error = self._handler.ReadPosSpeed(servo_id)
        self._check(comm, error, f"read position/speed of servo {servo_id}")
        return position, speed

    def is_present(self, servo_id: int) -> bool:
        """Return True if a servo answers a ping at this ID."""
        _model, comm, _error = self._handler.ping(servo_id)
        return comm == COMM_SUCCESS

    def set_id(self, current_id: int, new_id: int) -> None:
        """Permanently change a servo's ID (EEPROM write).

        Refuses if the new ID is already answering, since two servos sharing an ID on one bus
        cannot be told apart and both would need to be unplugged to fix it.
        """
        if not 0 <= new_id <= MAX_SERVO_ID:
            raise ValueError(f"new id must be within 0..{MAX_SERVO_ID}, got {new_id}")
        if new_id == current_id:
            raise ValueError(f"servo already has id {current_id}")
        self.ping(current_id)
        if self.is_present(new_id):
            raise ServoError(f"id {new_id} is already used by another servo on the bus")

        comm, error = self._handler.unLockEprom(current_id)
        self._check(comm, error, f"unlock eeprom of servo {current_id}")
        try:
            comm, error = self._handler.write1ByteTxRx(current_id, SMS_STS_ID, new_id)
            self._check(comm, error, f"write id {new_id} to servo {current_id}")
        except ServoError:
            self._handler.LockEprom(current_id)
            raise
        # From here the servo only answers at its new ID.
        comm, error = self._handler.LockEprom(new_id)
        self._check(comm, error, f"lock eeprom of servo {new_id}")
        self.ping(new_id)

    def enable_wheel_mode(self, servo_ids: Sequence[int]) -> None:
        """Put each servo in wheel mode and enable torque."""
        for servo_id in servo_ids:
            self.set_wheel_mode(servo_id)
            self.set_torque(servo_id, True)

    def _check(self, comm: int, error: int, operation: str) -> None:
        if comm != COMM_SUCCESS:
            raise ServoError(f"{operation} failed: {self._handler.getTxRxResult(comm)}")
        if error:
            raise ServoError(f"{operation} failed: {self._handler.getRxPacketError(error)}")
