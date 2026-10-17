"""Fake SDK objects so tests never touch a serial port."""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import partial

import pytest
from scservo_sdk import COMM_RX_TIMEOUT, COMM_SUCCESS

from talos_servo.bus import ServoBus


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


@dataclass
class FakeHandler:
    """Records every call; `fail` maps (method, servo_id) to (comm, error) to inject failures."""

    port: FakePort
    calls: list[tuple] = field(default_factory=list)
    fail: dict[tuple[str, int], tuple[int, int]] = field(default_factory=dict)
    model: int = 777
    position: int = 2048
    speed: int = 0

    def _result(self, method: str, servo_id: int) -> tuple[int, int]:
        return self.fail.get((method, servo_id), (COMM_SUCCESS, 0))

    def ping(self, servo_id: int) -> tuple[int, int, int]:
        self.calls.append(("ping", servo_id))
        comm, error = self._result("ping", servo_id)
        return (self.model if comm == COMM_SUCCESS else 0), comm, error

    def write1ByteTxRx(self, servo_id: int, address: int, value: int) -> tuple[int, int]:
        self.calls.append(("write1", servo_id, address, value))
        return self._result("write1", servo_id)

    def WriteSpec(self, servo_id: int, speed: int, acc: int) -> tuple[int, int]:
        self.calls.append(("speed", servo_id, speed, acc))
        return self._result("speed", servo_id)

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


@dataclass
class FakeSdk:
    port: FakePort | None = None
    handler: FakeHandler | None = None
    port_kwargs: dict = field(default_factory=dict)
    fail: dict = field(default_factory=dict)

    def make_port(self, name: str) -> FakePort:
        self.port = FakePort(name, **self.port_kwargs)
        return self.port

    def make_handler(self, port: FakePort) -> FakeHandler:
        self.handler = FakeHandler(port, fail=self.fail)
        return self.handler

    def bus(self, port: str = "FAKE", baudrate: int = 1_000_000) -> ServoBus:
        return ServoBus(
            port, baudrate, port_factory=self.make_port, handler_factory=self.make_handler
        )

    def bus_factory(self) -> partial[ServoBus]:
        return partial(ServoBus, port_factory=self.make_port, handler_factory=self.make_handler)


@pytest.fixture
def sdk() -> FakeSdk:
    return FakeSdk()
