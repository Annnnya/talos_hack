"""Quaternion helpers for the IMU orientation (#91).

The Madgwick filter publishes the IMU's orientation as a quaternion in ROS
order (x, y, z, w). The board sits on the central platform with a fixed
mount rotation from `config/robot.yaml` (`imu.rpy`), so the platform's own
roll and pitch are the IMU orientation with that mount rotation taken off.
Angles follow REP 103: x forward, y left, z up; roll about x, pitch about y.

No ROS here, so the maths is tested without rclpy.
"""

from __future__ import annotations

import math

Quaternion = tuple[float, float, float, float]  # x, y, z, w
Vector = tuple[float, float, float]
RollPitch = tuple[float, float]

IDENTITY: Quaternion = (0.0, 0.0, 0.0, 1.0)


def from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Quaternion of the roll, pitch, yaw (rad) rotation, applied in the
    ZYX order ROS uses."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """`a` then `b` composed: rotating by the result is rotating by `b`
    first, then by `a` (the usual Hamilton product)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q: Quaternion) -> Quaternion:
    """The inverse rotation of a unit quaternion."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate(q: Quaternion, v: Vector) -> Vector:
    """`v` rotated by the unit quaternion `q`."""
    x, y, z, _w = multiply(multiply(q, (*v, 0.0)), conjugate(q))
    return (x, y, z)


def roll_pitch(q: Quaternion) -> RollPitch:
    """Roll and pitch (rad) of a unit quaternion; yaw is left out."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def platform_tilt(imu: Quaternion, mount_rpy: Vector) -> RollPitch:
    """Roll and pitch of the platform the IMU is mounted on.

    `imu` is the orientation the filter reports for the board, `mount_rpy`
    the board's fixed rotation on the platform (`imu.rpy` in the config):
    the platform orientation is the board's with the mount taken off.
    """
    mount = from_rpy(*mount_rpy)
    return roll_pitch(multiply(imu, conjugate(mount)))
