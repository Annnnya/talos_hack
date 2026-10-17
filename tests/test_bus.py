from __future__ import annotations

import pytest
from conftest import FakePort
from scservo_sdk import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    SMS_STS_ID,
    SMS_STS_MODE,
    SMS_STS_MOVING,
    SMS_STS_PRESENT_CURRENT_L,
    SMS_STS_PRESENT_LOAD_L,
    SMS_STS_PRESENT_TEMPERATURE,
    SMS_STS_PRESENT_VOLTAGE,
    SMS_STS_TORQUE_ENABLE,
)
from serial import SerialException

from talos_servo.bus import (
    MAX_POSITION,
    MODE_POSITION,
    MODE_WHEEL,
    ServoError,
    ServoStatus,
    ServoTelemetry,
)


def test_open_failure_raises(sdk):
    sdk.port_kwargs = {"open_ok": False}
    with (
        pytest.raises(ServoError, match="could not open serial port FAKE"),
        sdk.bus(),
    ):
        pass


def test_serial_exception_on_open_becomes_servo_error(sdk, monkeypatch):
    def boom(self):
        raise SerialException("could not open port 'COM99'")

    monkeypatch.setattr(FakePort, "openPort", boom)
    with pytest.raises(ServoError, match="COM99"), sdk.bus():
        pass


def test_baud_failure_closes_port(sdk):
    sdk.port_kwargs = {"baud_ok": False}
    with pytest.raises(ServoError, match="baud rate"), sdk.bus():
        pass
    assert sdk.port.closed_count == 1


def test_ping_returns_model(sdk):
    with sdk.bus() as bus:
        assert bus.ping(1) == 777


def test_comm_failure_raises_with_sdk_message(sdk):
    sdk.fail[("ping", 2)] = (COMM_RX_TIMEOUT, 0)
    with (
        sdk.bus() as bus,
        pytest.raises(ServoError, match="ping servo 2 failed: timeout"),
    ):
        bus.ping(2)


def test_packet_error_raises(sdk):
    sdk.fail[("write1", 1)] = (COMM_SUCCESS, 32)
    with sdk.bus() as bus, pytest.raises(ServoError, match="packet error 32"):
        bus.set_torque(1, True)


def test_enable_wheel_mode_writes_mode_then_torque(sdk):
    with sdk.bus() as bus:
        bus.enable_wheel_mode([1, 2])
    assert sdk.handler.calls == [
        ("write1", 1, SMS_STS_MODE, MODE_WHEEL),
        ("write1", 1, SMS_STS_TORQUE_ENABLE, 1),
        ("write1", 2, SMS_STS_MODE, MODE_WHEEL),
        ("write1", 2, SMS_STS_TORQUE_ENABLE, 1),
    ]


def test_exit_stops_moving_servos_and_closes_port(sdk):
    with sdk.bus() as bus:
        bus.set_speed(1, 100)
        bus.set_speed(3, -50)
    assert sdk.handler.speed_calls()[-2:] == [
        ("speed", 1, 0, 20),
        ("speed", 3, 0, 20),
    ]
    assert sdk.port.closed_count == 1


def test_exit_stops_servos_when_body_raises(sdk):
    with pytest.raises(KeyboardInterrupt), sdk.bus() as bus:
        bus.set_speed(1, 100)
        raise KeyboardInterrupt
    assert sdk.handler.speed_calls()[-1] == ("speed", 1, 0, 20)
    assert sdk.port.closed_count == 1


def test_stopped_servo_is_not_stopped_again(sdk):
    with sdk.bus() as bus:
        bus.set_speed(1, 100)
        bus.stop(1)
    assert len(sdk.handler.speed_calls()) == 2


def test_stop_failure_on_exit_does_not_mask_original_error(sdk, caplog):
    sdk.fail[("speed", 2)] = (COMM_RX_TIMEOUT, 0)
    with (
        pytest.raises(ServoError, match="set speed 100 on servo 2"),
        sdk.bus() as bus,
    ):
        bus.set_speed(1, 100)
        bus.set_speed(2, 100)
    stops = [c for c in sdk.handler.speed_calls() if c[2] == 0]
    assert ("speed", 1, 0, 20) in stops
    assert "could not stop servo 2" in caplog.text
    assert sdk.port.closed_count == 1


@pytest.mark.parametrize(
    ("speed", "acc"), [(40_000, 20), (-40_000, 20), (100, 255), (100, -1)]
)
def test_set_speed_rejects_out_of_range(sdk, speed, acc):
    with sdk.bus() as bus, pytest.raises(ValueError):
        bus.set_speed(1, speed, acc)
    assert sdk.handler.speed_calls() == []


def test_read_position_speed(sdk):
    with sdk.bus() as bus:
        assert bus.read_position_speed(1) == (2048, 0)


def test_set_id_unlocks_writes_and_relocks_at_new_id(sdk):
    sdk.present = {1}
    with sdk.bus() as bus:
        bus.set_id(1, 3)
    calls = [c for c in sdk.handler.calls if c[0] != "ping"]
    assert calls == [("unlock", 1), ("write1", 1, SMS_STS_ID, 3), ("lock", 3)]
    assert sdk.present == {3}
    assert sdk.handler.calls[-1] == ("ping", 3)


def test_set_id_refuses_when_new_id_is_taken(sdk):
    sdk.present = {1, 3}
    with (
        sdk.bus() as bus,
        pytest.raises(ServoError, match="id 3 is already used"),
    ):
        bus.set_id(1, 3)
    assert not [c for c in sdk.handler.calls if c[0] in ("unlock", "write1")]


def test_set_id_fails_if_current_servo_missing(sdk):
    sdk.present = {2}
    with sdk.bus() as bus, pytest.raises(ServoError, match="ping servo 1"):
        bus.set_id(1, 3)


def test_set_id_relocks_old_id_when_write_fails(sdk):
    sdk.present = {1}
    sdk.fail[("write1", 1)] = (COMM_RX_TIMEOUT, 0)
    with sdk.bus() as bus, pytest.raises(ServoError, match="write id 3"):
        bus.set_id(1, 3)
    assert sdk.handler.calls[-1] == ("lock", 1)
    assert sdk.present == {1}


@pytest.mark.parametrize(("current", "new"), [(1, 1), (1, 254), (1, -1)])
def test_set_id_rejects_bad_ids(sdk, current, new):
    with sdk.bus() as bus, pytest.raises(ValueError):
        bus.set_id(current, new)


def test_set_position_mode_writes_mode_register(sdk):
    with sdk.bus() as bus:
        bus.set_position_mode(3)
    assert sdk.handler.calls == [("write1", 3, SMS_STS_MODE, MODE_POSITION)]


def test_read_mode_and_torque(sdk):
    sdk.registers.update({SMS_STS_MODE: MODE_WHEEL, SMS_STS_TORQUE_ENABLE: 1})
    with sdk.bus() as bus:
        assert bus.read_mode(1) == MODE_WHEEL
        assert bus.read_torque(1) is True


def test_move_to_sends_goal(sdk):
    with sdk.bus() as bus:
        bus.move_to(2, 3000, 200, 10)
    assert sdk.handler.calls == [("move", 2, 3000, 200, 10)]


@pytest.mark.parametrize(
    ("position", "speed", "acc"),
    [(-1, 100, 20), (MAX_POSITION + 1, 100, 20), (100, 0, 20), (100, 100, 255)],
)
def test_move_to_rejects_out_of_range(sdk, position, speed, acc):
    with sdk.bus() as bus, pytest.raises(ValueError):
        bus.move_to(1, position, speed, acc)
    assert sdk.handler.calls == []


def test_move_to_failure_raises(sdk):
    sdk.fail[("move", 1)] = (COMM_RX_TIMEOUT, 0)
    with sdk.bus() as bus, pytest.raises(ServoError, match="move servo 1"):
        bus.move_to(1, 100, 100)


def test_read_status_decodes_registers(sdk):
    sdk.registers.update(
        {
            SMS_STS_PRESENT_LOAD_L: (1 << 10) | 50,  # sign bit set means -50
            SMS_STS_PRESENT_VOLTAGE: 121,
            SMS_STS_PRESENT_TEMPERATURE: 34,
            SMS_STS_PRESENT_CURRENT_L: (1 << 15)
            | 100,  # sign bit set means -100 counts
            SMS_STS_MOVING: 1,
        }
    )
    with sdk.bus() as bus:
        assert bus.read_status(1) == ServoStatus(
            load=-50, voltage=12.1, temperature=34, current=-0.65, moving=True
        )


def test_read_status_failure_raises(sdk):
    sdk.fail[("read2", 1)] = (COMM_RX_TIMEOUT, 0)
    with (
        sdk.bus() as bus,
        pytest.raises(ServoError, match="read load of servo 1"),
    ):
        bus.read_status(1)


def test_set_speeds_sends_one_sync_packet(sdk):
    with sdk.bus() as bus:
        bus.set_speeds({1: 100, 2: -100, 3: 0, 4: 300}, 20)
        assert sdk.handler.sync_writes() == [
            {
                1: [20, 0, 0, 0, 0, 100, 0],
                2: [20, 0, 0, 0, 0, 100, 0x80],
                3: [20, 0, 0, 0, 0, 0, 0],
                4: [20, 0, 0, 0, 0, 300 & 0xFF, 300 >> 8],
            }
        ]
        assert sdk.handler.speed_calls() == []


def test_set_speeds_accepts_per_servo_acceleration(sdk):
    with sdk.bus() as bus:
        bus.set_speeds({1: 100, 2: 50}, {1: 20, 2: 10})
        packet = sdk.handler.sync_writes()[0]
        assert packet[1][0] == 20
        assert packet[2][0] == 10


def test_set_speeds_tracks_moving_servos_for_exit_stop(sdk):
    with sdk.bus() as bus:
        bus.set_speeds({1: 100, 2: 0})
    assert sdk.handler.speed_calls() == [("speed", 1, 0, 20)]


def test_stop_many_clears_moving_servos(sdk):
    with sdk.bus() as bus:
        bus.set_speeds({1: 100, 2: 100})
        bus.stop_many([1, 2])
        assert sdk.handler.sync_writes()[-1] == {
            1: [20, 0, 0, 0, 0, 0, 0],
            2: [20, 0, 0, 0, 0, 0, 0],
        }
    assert sdk.handler.speed_calls() == []


def test_set_speeds_comm_failure_raises_and_still_stops_on_exit(sdk):
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    with pytest.raises(ServoError, match="sync set speeds"), sdk.bus() as bus:
        bus.set_speeds({1: 100, 2: 100})
    stops = sdk.handler.speed_calls()
    assert ("speed", 1, 0, 20) in stops
    assert ("speed", 2, 0, 20) in stops


@pytest.mark.parametrize(
    ("speeds", "acc"),
    [({}, 20), ({1: 40_000}, 20), ({1: 100}, 255), ({1: 100}, -1)],
)
def test_set_speeds_rejects_bad_input_without_writing(sdk, speeds, acc):
    with sdk.bus() as bus, pytest.raises(ValueError):
        bus.set_speeds(speeds, acc)
    assert sdk.handler.sync_writes() == []


def test_read_telemetry_decodes_one_block_read(sdk):
    sdk.registers.update(
        {
            SMS_STS_PRESENT_LOAD_L: (1 << 10) | 50,
            SMS_STS_PRESENT_VOLTAGE: 121,
            SMS_STS_PRESENT_TEMPERATURE: 34,
            SMS_STS_PRESENT_CURRENT_L: (1 << 15) | 100,
            SMS_STS_MOVING: 1,
        }
    )
    with sdk.bus() as bus:
        sdk.handler.position = 3000
        sdk.handler.speed = -250
        assert bus.read_telemetry(2) == ServoTelemetry(
            position=3000,
            speed=-250,
            status=ServoStatus(
                load=-50,
                voltage=12.1,
                temperature=34,
                current=-0.65,
                moving=True,
            ),
        )
        reads = [c for c in sdk.handler.calls if c[0].startswith("read")]
        assert reads == [("read_block", 2, 56, 15)]


def test_read_telemetry_failure_raises(sdk):
    sdk.fail[("read_block", 3)] = (COMM_RX_TIMEOUT, 0)
    with (
        sdk.bus() as bus,
        pytest.raises(ServoError, match="read telemetry of servo 3"),
    ):
        bus.read_telemetry(3)


def test_read_telemetry_short_reply_raises(sdk, monkeypatch):
    with sdk.bus() as bus:
        monkeypatch.setattr(
            sdk.handler, "readTxRx", lambda *_a: ([0] * 4, COMM_SUCCESS, 0)
        )
        with pytest.raises(ServoError, match="got 4 bytes"):
            bus.read_telemetry(1)


class FakeSerial:
    def fileno(self) -> int:
        return 7


class FakeFcntl:
    LOCK_EX = 2
    LOCK_NB = 4

    def __init__(self, busy: bool) -> None:
        self.busy = busy
        self.calls: list[tuple[int, int]] = []

    def flock(self, fd: int, operation: int) -> None:
        self.calls.append((fd, operation))
        if self.busy:
            raise BlockingIOError(11, "Resource temporarily unavailable")


def test_open_locks_the_port(sdk, monkeypatch):
    from talos_servo import bus as bus_module

    lock = FakeFcntl(busy=False)
    monkeypatch.setattr(bus_module, "fcntl", lock)
    monkeypatch.setattr(FakePort, "ser", FakeSerial(), raising=False)
    with sdk.bus():
        assert lock.calls == [(7, FakeFcntl.LOCK_EX | FakeFcntl.LOCK_NB)]


def test_open_refuses_a_port_another_process_holds(sdk, monkeypatch):
    from talos_servo import bus as bus_module

    monkeypatch.setattr(bus_module, "fcntl", FakeFcntl(busy=True))
    monkeypatch.setattr(FakePort, "ser", FakeSerial(), raising=False)
    with (
        pytest.raises(ServoError, match="already in use by another process"),
        sdk.bus(),
    ):
        pass
    assert sdk.port.closed_count == 1


def test_open_without_flock_skips_locking(sdk, monkeypatch):
    from talos_servo import bus as bus_module

    monkeypatch.setattr(bus_module, "fcntl", None)
    monkeypatch.setattr(FakePort, "ser", FakeSerial(), raising=False)
    with sdk.bus() as bus:
        assert bus.ping(1) == 777
