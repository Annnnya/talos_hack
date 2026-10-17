"""Unit conversions and covariances for publishing MPU6050 samples as sensor_msgs/Imu.

Kept free of ROS imports so it can be tested without rclpy.
"""

from __future__ import annotations

import math

from talos_servo.imu import Vector

STANDARD_GRAVITY = 9.80665  # m/s^2 per g

# Per-axis noise variances. The MPU6050 datasheet gives about 0.005 deg/s/sqrt(Hz) for the gyro
# and 400 ug/sqrt(Hz) for the accelerometer, which at the 44 Hz filter is roughly 6e-4 rad/s and
# 0.03 m/s^2. These defaults are several times higher because the board is a clone and the
# servos shake the frame. Madgwick ignores them; an EKF downstream uses them.
DEFAULT_ANGULAR_VELOCITY_VARIANCE = 1e-4  # (rad/s)^2
DEFAULT_LINEAR_ACCELERATION_VARIANCE = 1e-2  # (m/s^2)^2

# sensor_msgs/Imu convention: element 0 of a covariance set to -1 means "not provided".
COVARIANCE_NOT_PROVIDED = -1.0
COVARIANCE_SIZE = 9


def g_to_mps2(accel: Vector) -> Vector:
    x, y, z = accel
    return (x * STANDARD_GRAVITY, y * STANDARD_GRAVITY, z * STANDARD_GRAVITY)


def dps_to_radps(gyro: Vector) -> Vector:
    x, y, z = gyro
    return (math.radians(x), math.radians(y), math.radians(z))


def diagonal_covariance(variance: float) -> list[float]:
    """Row-major 3x3 covariance with the same variance on every axis and no cross terms."""
    if not math.isfinite(variance) or variance < 0:
        raise ValueError(f"variance must be a finite number >= 0, got {variance}")
    return [variance if row == col else 0.0 for row in range(3) for col in range(3)]


def not_provided_covariance() -> list[float]:
    return [COVARIANCE_NOT_PROVIDED] + [0.0] * (COVARIANCE_SIZE - 1)
