"""Quaternion helpers behind the platform tilt (#91). No ROS."""

from __future__ import annotations

import math

import pytest

from talos_servo.orientation import (
    IDENTITY,
    conjugate,
    from_rpy,
    multiply,
    platform_tilt,
    roll_pitch,
    rotate,
)

RIGHT_ANGLE = math.pi / 2


def test_identity_is_level():
    assert roll_pitch(IDENTITY) == (0.0, 0.0)


@pytest.mark.parametrize(
    ("roll", "pitch", "yaw"),
    [(0.1, 0.0, 0.0), (0.0, 0.2, 0.0), (-0.3, 0.25, 0.0), (0.1, -0.2, 1.0)],
)
def test_roll_pitch_come_back_from_rpy(roll, pitch, yaw):
    # Yaw does not change roll or pitch.
    got = roll_pitch(from_rpy(roll, pitch, yaw))
    assert got == pytest.approx((roll, pitch))


def test_rotate_by_the_identity_keeps_the_vector():
    assert rotate(IDENTITY, (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))


def test_a_quarter_turn_about_z_takes_x_to_y():
    q = from_rpy(0.0, 0.0, RIGHT_ANGLE)
    assert rotate(q, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))


def test_conjugate_undoes_the_rotation():
    q = from_rpy(0.3, -0.2, 0.7)
    assert multiply(q, conjugate(q)) == pytest.approx(IDENTITY)


def test_a_level_platform_reads_zero_through_the_mount():
    # The board is tilted on the platform: the IMU reports the mount alone.
    mount = (0.1, -0.2, 0.5)
    assert platform_tilt(from_rpy(*mount), mount) == pytest.approx((0.0, 0.0))


def test_platform_tilt_takes_the_mount_off():
    mount = (0.0, 0.0, RIGHT_ANGLE)
    # The platform rolls 0.3; the board, turned a quarter turn, sees it as
    # its own pitch.
    imu = multiply(from_rpy(0.3, 0.0, 0.0), from_rpy(*mount))
    assert platform_tilt(imu, mount) == pytest.approx((0.3, 0.0))


def test_zero_mount_is_the_imu_orientation():
    q = from_rpy(0.05, 0.1, 0.0)
    assert platform_tilt(q, (0.0, 0.0, 0.0)) == pytest.approx((0.05, 0.1))
