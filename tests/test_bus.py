from __future__ import annotations

import pytest
from conftest import FakePort
from scservo_sdk import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    SMS_STS_ID,
    SMS_STS_MODE,
    SMS_STS_TORQUE_ENABLE,
)
from serial import SerialException

from talos_servo.bus import MODE_WHEEL, ServoError


def test_open_failure_raises(sdk):
    sdk.port_kwargs = {"open_ok": False}
    with pytest.raises(ServoError, match="could not open serial port FAKE"), sdk.bus():
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
    with sdk.bus() as bus, pytest.raises(ServoError, match="ping servo 2 failed: timeout"):
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
    assert sdk.handler.speed_calls()[-2:] == [("speed", 1, 0, 20), ("speed", 3, 0, 20)]
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
    with pytest.raises(ServoError, match="set speed 100 on servo 2"), sdk.bus() as bus:
        bus.set_speed(1, 100)
        bus.set_speed(2, 100)
    stops = [c for c in sdk.handler.speed_calls() if c[2] == 0]
    assert ("speed", 1, 0, 20) in stops
    assert "could not stop servo 2" in caplog.text
    assert sdk.port.closed_count == 1


@pytest.mark.parametrize(("speed", "acc"), [(40_000, 20), (-40_000, 20), (100, 255), (100, -1)])
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
    with sdk.bus() as bus, pytest.raises(ServoError, match="id 3 is already used"):
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
