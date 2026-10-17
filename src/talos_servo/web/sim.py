"""Simulated Feetech SDK objects for `talos-servo web --mock`.

They plug into ServoBus through its port_factory / handler_factory hooks, so simulation runs the
same bus and controller code as real hardware, without opening a serial port.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

from scservo_sdk import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    SMS_STS_MODE,
    SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_TORQUE_ENABLE,
)

from talos_servo.bus import (
    LOAD_SIGN_BIT,
    MAX_POSITION,
    MAX_SPEED,
    MODE_POSITION,
    MODE_WHEEL,
    ServoBus,
)

SIM_MODEL = 777
SIM_VOLTAGE_RAW = 120  # 12.0 V
SIM_TEMPERATURE = 30
STEPS_PER_TURN = MAX_POSITION + 1
MAX_SIM_LOAD = 300


class SimPort:
    def __init__(self, name: str) -> None:
        self.name = name

    def openPort(self) -> bool:
        return True

    def setBaudRate(self, _baudrate: int) -> bool:
        return True

    def closePort(self) -> None:
        pass


@dataclass
class _SimServo:
    mode: int = MODE_POSITION
    torque: bool = True
    position: float = STEPS_PER_TURN / 2
    goal: float = STEPS_PER_TURN / 2
    goal_speed: int = 0
    wheel_speed: int = 0
    speed: int = 0
    updated: float = field(default_factory=time.monotonic)


class SimHandler:
    """Answers like sms_sts for the given IDs; any other ID times out."""

    def __init__(self, _port: SimPort, ids: Iterable[int]) -> None:
        self.servos = {i: _SimServo() for i in ids}

    # --- physics ---------------------------------------------------------
    def _step(self, s: _SimServo) -> None:
        now = time.monotonic()
        dt, s.updated = now - s.updated, now
        if not s.torque:
            s.speed = 0
        elif s.mode == MODE_WHEEL:
            s.speed = s.wheel_speed
            s.position = (s.position + s.wheel_speed * dt) % STEPS_PER_TURN
        else:
            diff = s.goal - s.position
            step = math.copysign(min(abs(diff), (s.goal_speed or MAX_SPEED) * dt), diff)
            s.position += step
            s.speed = round(step / dt) if dt else 0

    def _find(self, servo_id: int) -> _SimServo | None:
        s = self.servos.get(servo_id)
        if s:
            self._step(s)
        return s

    # --- sms_sts interface used by ServoBus -------------------------------
    def ping(self, servo_id: int) -> tuple[int, int, int]:
        if servo_id not in self.servos:
            return 0, COMM_RX_TIMEOUT, 0
        return SIM_MODEL, COMM_SUCCESS, 0

    def write1ByteTxRx(self, servo_id: int, address: int, value: int) -> tuple[int, int]:
        s = self._find(servo_id)
        if s is None:
            return COMM_RX_TIMEOUT, 0
        if address == SMS_STS_MODE:
            s.mode, s.wheel_speed, s.goal = value, 0, s.position
        elif address == SMS_STS_TORQUE_ENABLE:
            s.torque = bool(value)
        return COMM_SUCCESS, 0

    def read1ByteTxRx(self, servo_id: int, address: int) -> tuple[int, int, int]:
        s = self._find(servo_id)
        if s is None:
            return 0, COMM_RX_TIMEOUT, 0
        values = {
            SMS_STS_MODE: s.mode,
            SMS_STS_TORQUE_ENABLE: int(s.torque),
            SMS_STS_PRESENT_VOLTAGE: SIM_VOLTAGE_RAW,
            SMS_STS_PRESENT_TEMPERATURE: SIM_TEMPERATURE,
        }
        return values.get(address, 0), COMM_SUCCESS, 0

    def read2ByteTxRx(self, servo_id: int, address: int) -> tuple[int, int, int]:
        s = self._find(servo_id)
        if s is None:
            return 0, COMM_RX_TIMEOUT, 0
        if address == SMS_STS_PRESENT_LOAD_L:
            load = round(abs(s.speed) / MAX_SPEED * MAX_SIM_LOAD * 10)
            raw = min(load, MAX_SIM_LOAD) | ((1 << LOAD_SIGN_BIT) if s.speed < 0 else 0)
            return raw, COMM_SUCCESS, 0
        return 0, COMM_SUCCESS, 0

    def WritePosEx(self, servo_id: int, position: int, speed: int, _acc: int) -> tuple[int, int]:
        s = self._find(servo_id)
        if s is None:
            return COMM_RX_TIMEOUT, 0
        s.goal, s.goal_speed, s.torque = float(position), speed, True
        return COMM_SUCCESS, 0

    def WriteSpec(self, servo_id: int, speed: int, _acc: int) -> tuple[int, int]:
        s = self._find(servo_id)
        if s is None:
            return COMM_RX_TIMEOUT, 0
        s.wheel_speed = speed
        if speed:
            s.torque = True
        return COMM_SUCCESS, 0

    def ReadPosSpeed(self, servo_id: int) -> tuple[int, int, int, int]:
        s = self._find(servo_id)
        if s is None:
            return 0, 0, COMM_RX_TIMEOUT, 0
        return int(s.position), s.speed, COMM_SUCCESS, 0

    @staticmethod
    def scs_tohost(value: int, sign_bit: int) -> int:
        return -(value & ~(1 << sign_bit)) if value & (1 << sign_bit) else value

    def getTxRxResult(self, comm: int) -> str:
        return "simulated timeout" if comm == COMM_RX_TIMEOUT else f"simulated comm {comm}"

    def getRxPacketError(self, error: int) -> str:
        return f"simulated packet error {error}"


def sim_factories(
    ids: Iterable[int],
) -> tuple[Callable[[str], SimPort], Callable[[SimPort], SimHandler]]:
    """Return (port_factory, handler_factory) for ServoBus that simulate the given IDs."""
    ids = list(ids)
    return SimPort, lambda port: SimHandler(port, ids)


def make_bus(port: str | None, baudrate: int, *, mock: bool, ids: Iterable[int]) -> ServoBus:
    """A real bus on `port`, or a simulated one answering for `ids` when `mock` is set."""
    if mock:
        port_factory, handler_factory = sim_factories(ids)
        return ServoBus(
            "simulated", baudrate, port_factory=port_factory, handler_factory=handler_factory
        )
    if not port:
        raise ValueError("a serial port is required unless --mock is set")
    return ServoBus(port, baudrate)
