"""Finding the servo port without an argument. Nothing is opened."""

from __future__ import annotations

from talos_servo.defaults import PORT_ENV_VAR
from talos_servo.ros.port_args import FALLBACK_PORT, find_servo_port

BY_ID = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B7B014358-if00"


def test_the_environment_wins():
    assert (
        find_servo_port({PORT_ENV_VAR: "/dev/ttyUSB3"}, by_id=[BY_ID])
        == "/dev/ttyUSB3"
    )


def test_one_adapter_by_id_is_the_port():
    assert find_servo_port({}, by_id=[BY_ID], fallback_exists=True) == BY_ID


def test_two_adapters_fall_back_to_the_plain_name():
    assert (
        find_servo_port({}, by_id=[BY_ID, BY_ID + "-2"], fallback_exists=True)
        == FALLBACK_PORT
    )


def test_nothing_plugged_in_is_none():
    assert find_servo_port({}, by_id=[], fallback_exists=False) is None


def test_an_empty_environment_value_does_not_count():
    assert (
        find_servo_port({PORT_ENV_VAR: ""}, by_id=[], fallback_exists=True)
        == FALLBACK_PORT
    )
