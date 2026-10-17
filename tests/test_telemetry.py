from __future__ import annotations

import math

import pytest

from talos_servo.telemetry import (
    STEPS_PER_REV,
    WheelOdometer,
    speed_to_rad_s,
    steps_to_rad,
)


def test_unit_conversions():
    assert steps_to_rad(0) == 0
    assert steps_to_rad(STEPS_PER_REV) == pytest.approx(2 * math.pi)
    assert speed_to_rad_s(-STEPS_PER_REV / 2) == pytest.approx(-math.pi)


def test_first_reading_is_zero():
    odo = WheelOdometer()
    assert odo.update(1234) == 0


def test_forward_across_wrap():
    odo = WheelOdometer()
    odo.update(4000)
    assert odo.update(100) == pytest.approx(steps_to_rad(196))


def test_backward_across_wrap():
    odo = WheelOdometer()
    odo.update(100)
    assert odo.update(4000) == pytest.approx(steps_to_rad(-196))


def test_accumulates_over_turns():
    odo = WheelOdometer()
    for raw in [0, 1000, 2000, 3000, 0, 1000]:
        odo.update(raw)
    assert odo.angle == pytest.approx(steps_to_rad(STEPS_PER_REV + 1000))


def test_direction_sign_flips_angle():
    odo = WheelOdometer(direction=-1)
    odo.update(0)
    assert odo.update(500) == pytest.approx(steps_to_rad(-500))


def test_reset():
    odo = WheelOdometer()
    odo.update(0)
    odo.update(500)
    odo.reset()
    assert odo.angle == 0
    assert odo.update(3000) == 0


def test_rejects_bad_direction():
    with pytest.raises(ValueError):
        WheelOdometer(direction=0)
