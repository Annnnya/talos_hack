"""Fake SDK objects so tests never touch a serial port, plus shared helpers."""

from __future__ import annotations

import os
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import partial

import pytest
from scservo_sdk import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    SMS_STS_ID,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_PRESENT_SPEED_L,
)

from talos_servo.bus import ServoBus
from talos_servo.config import RobotConfig, parse_config
from talos_servo.imu import WHO_AM_I, WHO_AM_I_MPU6050

# ROS tests publish on the real topic names (/cmd_vel, /robot/...). A private
# domain that only reaches this machine keeps them away from a robot node
# running on the same network, e.g. the Pi's talos-robot service.
TEST_DOMAIN_BASE = 150
TEST_DOMAIN_COUNT = 50
os.environ["ROS_DOMAIN_ID"] = str(
    TEST_DOMAIN_BASE + os.getpid() % TEST_DOMAIN_COUNT
)
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

# A position joint and a speed joint, enough for most robot tests.
SMALL = {
    "max_speed": 500,
    "telemetry_hz": 50,
    "control_hz": 50,
    "command_timeout_s": 0.5,
    "joints": {
        "bend": {
            "id": 1,
            "control": "position",
            "min_pos": 1000,
            "max_pos": 3000,
        },
        "wheel": {"id": 2, "control": "speed"},
    },
}

# Four mecanum wheels, rear right mounted the other way round.
BASE = {
    "max_speed": 500,
    "joints": {
        "fl": {"id": 1, "control": "speed"},
        "fr": {"id": 2, "control": "speed"},
        "rl": {"id": 3, "control": "speed"},
        "rr": {"id": 4, "control": "speed", "inverted": True},
        "bend": {"id": 5, "control": "position", "enabled": False},
    },
    "base": {
        "wheels": ["fl", "fr", "rl", "rr"],
        "wheelbase_m": 0.3,
        "track_width_m": 0.3,
        "steps_per_meter": 1000.0,
        "max_linear_speed": 0.2,
        "max_angular_speed": 1.0,
    },
}


def config(data: dict) -> RobotConfig:
    return parse_config(data)


class FakeClock:
    """Monotonic clock the test moves by hand: `clock.now += 1.0`."""

    def __init__(self, now: float = 100.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def wait_for(predicate: Callable[[], object], timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("condition not met in time")
        time.sleep(0.01)


@dataclass
class FakePort:
    name: str
    open_ok: bool = True
    baud_ok: bool = True
    is_open: bool = False
    closed_count: int = 0

    def openPort(self) -> bool:
        self.is_open = self.open_ok
        return self.open_ok

    def setBaudRate(self, baudrate: int) -> bool:
        return self.baud_ok

    def closePort(self) -> None:
        self.is_open = False
        self.closed_count += 1


BROADCAST_ID = 254


@dataclass
class FakeGroupSyncWrite:
    """Mimics the SDK GroupSyncWrite; each sent packet is recorded on the
    handler."""

    handler: FakeHandler
    data: dict[int, list[int]] = field(default_factory=dict)

    def clearParam(self) -> None:
        self.data.clear()

    def addParam(self, servo_id: int, data: list[int]) -> bool:
        if servo_id in self.data:
            return False
        self.data[servo_id] = list(data)
        return True

    def txPacket(self) -> int:
        self.handler.calls.append(("sync_write", dict(self.data)))
        return self.handler._result("sync_write", BROADCAST_ID)[0]


@dataclass
class FakeHandler:
    """Records every call; `fail` maps (method, servo_id) to (comm, error) to
    inject failures.

    If `present` is set, only those IDs answer pings, and an ID write moves a
    servo to its new ID.
    """

    port: FakePort
    calls: list[tuple] = field(default_factory=list)
    fail: dict[tuple[str, int], tuple[int, int]] = field(default_factory=dict)
    model: int = 777
    position: int = 2048
    speed: int = 0
    present: set[int] | None = None
    # register address -> value returned by read1ByteTxRx / read2ByteTxRx
    registers: dict[int, int] = field(default_factory=dict)
    # (servo id, address) -> value from a successful write1ByteTxRx, read back
    # like a real servo
    written: dict[tuple[int, int], int] = field(default_factory=dict)
    groupSyncWrite: FakeGroupSyncWrite = field(init=False)

    def __post_init__(self) -> None:
        self.groupSyncWrite = FakeGroupSyncWrite(self)

    def _result(self, method: str, servo_id: int) -> tuple[int, int]:
        return self.fail.get((method, servo_id), (COMM_SUCCESS, 0))

    def _register(self, servo_id: int, address: int) -> int:
        return self.written.get(
            (servo_id, address), self.registers.get(address, 0)
        )

    def ping(self, servo_id: int) -> tuple[int, int, int]:
        self.calls.append(("ping", servo_id))
        comm, error = self._result("ping", servo_id)
        if self.present is not None and servo_id not in self.present:
            comm = COMM_RX_TIMEOUT
        return (self.model if comm == COMM_SUCCESS else 0), comm, error

    def write1ByteTxRx(
        self, servo_id: int, address: int, value: int
    ) -> tuple[int, int]:
        self.calls.append(("write1", servo_id, address, value))
        result = self._result("write1", servo_id)
        if result == (COMM_SUCCESS, 0):
            self.written[(servo_id, address)] = value
        if (
            address == SMS_STS_ID
            and result == (COMM_SUCCESS, 0)
            and self.present is not None
        ):
            self.present.discard(servo_id)
            self.present.add(value)
        return result

    def unLockEprom(self, servo_id: int) -> tuple[int, int]:
        self.calls.append(("unlock", servo_id))
        return self._result("unlock", servo_id)

    def LockEprom(self, servo_id: int) -> tuple[int, int]:
        self.calls.append(("lock", servo_id))
        return self._result("lock", servo_id)

    def WriteSpec(self, servo_id: int, speed: int, acc: int) -> tuple[int, int]:
        self.calls.append(("speed", servo_id, speed, acc))
        return self._result("speed", servo_id)

    def WritePosEx(
        self, servo_id: int, position: int, speed: int, acc: int
    ) -> tuple[int, int]:
        self.calls.append(("move", servo_id, position, speed, acc))
        return self._result("move", servo_id)

    def read1ByteTxRx(
        self, servo_id: int, address: int
    ) -> tuple[int, int, int]:
        self.calls.append(("read1", servo_id, address))
        comm, error = self._result("read1", servo_id)
        return self._register(servo_id, address), comm, error

    def read2ByteTxRx(
        self, servo_id: int, address: int
    ) -> tuple[int, int, int]:
        self.calls.append(("read2", servo_id, address))
        comm, error = self._result("read2", servo_id)
        return self._register(servo_id, address), comm, error

    @staticmethod
    def scs_tohost(value: int, sign_bit: int) -> int:
        return -(value & ~(1 << sign_bit)) if value & (1 << sign_bit) else value

    def readTxRx(
        self, servo_id: int, address: int, length: int
    ) -> tuple[list[int], int, int]:
        """Block read over a little-endian memory image built from position,
        speed, registers."""
        self.calls.append(("read_block", servo_id, address, length))
        comm, error = self._result("read_block", servo_id)
        if comm != COMM_SUCCESS:
            return [], comm, error
        memory: dict[int, int] = {}
        for start, value in self.registers.items():
            memory[start] = value & 0xFF
            if value > 0xFF:
                memory[start + 1] = value >> 8
        for start, value in (
            (SMS_STS_PRESENT_POSITION_L, self.position),
            (SMS_STS_PRESENT_SPEED_L, self.speed),
        ):
            raw = abs(value) | (0x8000 if value < 0 else 0)
            memory[start], memory[start + 1] = raw & 0xFF, raw >> 8
        return [memory.get(address + i, 0) for i in range(length)], comm, error

    def ReadPosSpeed(self, servo_id: int) -> tuple[int, int, int, int]:
        self.calls.append(("read", servo_id))
        comm, error = self._result("read", servo_id)
        return self.position, self.speed, comm, error

    def getTxRxResult(self, comm: int) -> str:
        return "timeout" if comm == COMM_RX_TIMEOUT else f"comm {comm}"

    def getRxPacketError(self, error: int) -> str:
        return f"packet error {error}"

    def speed_calls(self) -> list[tuple]:
        return [c for c in self.calls if c[0] == "speed"]

    def sync_writes(self) -> list[dict[int, list[int]]]:
        return [c[1] for c in self.calls if c[0] == "sync_write"]


@dataclass
class FakeSdk:
    port: FakePort | None = None
    handler: FakeHandler | None = None
    port_kwargs: dict = field(default_factory=dict)
    fail: dict = field(default_factory=dict)
    present: set[int] | None = None
    registers: dict[int, int] = field(default_factory=dict)

    def make_port(self, name: str) -> FakePort:
        self.port = FakePort(name, **self.port_kwargs)
        return self.port

    def make_handler(self, port: FakePort) -> FakeHandler:
        self.handler = FakeHandler(
            port, fail=self.fail, present=self.present, registers=self.registers
        )
        return self.handler

    def bus(self, port: str = "FAKE", baudrate: int = 1_000_000) -> ServoBus:
        return ServoBus(
            port,
            baudrate,
            port_factory=self.make_port,
            handler_factory=self.make_handler,
        )

    def bus_factory(self) -> partial[ServoBus]:
        return partial(
            ServoBus,
            port_factory=self.make_port,
            handler_factory=self.make_handler,
        )


@pytest.fixture
def sdk() -> FakeSdk:
    return FakeSdk()


@dataclass
class FakeI2c:
    """In-memory i2c bus with one device.

    `fail` maps an operation name to an OSError to raise.
    """

    bus_number: int
    address: int = 0x68
    registers: dict[int, int] = field(
        default_factory=lambda: {WHO_AM_I: WHO_AM_I_MPU6050}
    )
    writes: list[tuple[int, int]] = field(default_factory=list)
    fail: dict[str, OSError] = field(default_factory=dict)
    closed: bool = False

    def _maybe_fail(self, op: str, address: int) -> None:
        if op in self.fail:
            raise self.fail[op]
        if address != self.address:
            raise OSError(121, "Remote I/O error")

    def read_byte_data(self, address: int, register: int) -> int:
        self._maybe_fail("read_byte", address)
        return self.registers.get(register, 0)

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        self._maybe_fail("write_byte", address)
        self.writes.append((register, value))
        self.registers[register] = value

    def read_i2c_block_data(
        self, address: int, register: int, length: int
    ) -> list[int]:
        self._maybe_fail("read_block", address)
        return [self.registers.get(register + i, 0) for i in range(length)]

    def close(self) -> None:
        self.closed = True

    def set_words(self, start: int, words: list[int]) -> None:
        """Store signed 16-bit words big-endian from `start`, like the sensor's
        data registers."""
        for i, word in enumerate(words):
            raw = word & 0xFFFF
            self.registers[start + 2 * i] = raw >> 8
            self.registers[start + 2 * i + 1] = raw & 0xFF


@dataclass
class FakeI2cFactory:
    bus: FakeI2c | None = None
    open_error: OSError | None = None
    registers: dict[int, int] | None = None

    def __call__(self, bus_number: int) -> FakeI2c:
        if self.open_error is not None:
            raise self.open_error
        self.bus = FakeI2c(bus_number)
        if self.registers is not None:
            self.bus.registers.update(self.registers)
        return self.bus


@pytest.fixture
def i2c(monkeypatch) -> FakeI2cFactory:
    from talos_servo import imu

    monkeypatch.setattr(imu.time, "sleep", lambda _s: None)
    return FakeI2cFactory()
