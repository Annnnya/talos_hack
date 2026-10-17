"""ROS 2 node that publishes the MPU6050 as raw IMU data.

Topics (relative, so `/imu/...` by default and namespaced with `--ros-args -r
__ns:=/x`):

- `imu/data_raw` (sensor_msgs/Imu): linear acceleration in m/s^2 (gravity
  included, about +9.8 on z when flat) and angular velocity in rad/s. No
  orientation: `orientation_covariance[0]` is -1. Feed it to imu_filter_madgwick
  for `imu/data` with a quaternion (`pixi run -e ros imu-filter`)
- `imu/temperature` (sensor_msgs/Temperature): die temperature in deg C, at
  `temperature_hz`

Run with `pixi run -e ros imu-node`. The gyro is zeroed at startup (`calibrate`,
default true), so keep the robot still for about a second.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from collections.abc import Sequence
from typing import Any

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

from talos_servo.defaults import IMU_BUS_ENV_VAR, QUEUE_DEPTH
from talos_servo.imu import (
    ACCEL_RANGES,
    DEFAULT_ACCEL_RANGE,
    DEFAULT_ADDRESS,
    DEFAULT_DLPF,
    DEFAULT_GYRO_RANGE,
    DEFAULT_I2C_BUS,
    GYRO_RANGES,
    MAX_DLPF,
    ImuError,
    ImuSample,
    Mpu6050,
    Vector,
)
from talos_servo.parsing import parse_int_any_base
from talos_servo.ros.imu_convert import (
    DEFAULT_ANGULAR_VELOCITY_VARIANCE,
    DEFAULT_LINEAR_ACCELERATION_VARIANCE,
    diagonal_covariance,
    dps_to_radps,
    g_to_mps2,
    not_provided_covariance,
)

NODE_NAME = "imu"
# The sensor is configured for 100 Hz output, so polling faster only repeats
# samples.
DEFAULT_RATE_HZ = 100.0
DEFAULT_TEMPERATURE_HZ = 1.0
DEFAULT_FRAME_ID = "imu_link"
# About a quarter second of failed reads at 100 Hz: long enough to ride out one
# I2C glitch, short enough that consumers never see stale data for long.
DEFAULT_MAX_READ_FAILURES = 25


def _vector3(v: Vector) -> Vector3:
    x, y, z = v
    return Vector3(x=x, y=y, z=z)


class ImuNode(Node):
    def __init__(
        self, imu: Mpu6050, *, node_name: str = NODE_NAME, **node_kwargs: Any
    ) -> None:
        super().__init__(node_name, **node_kwargs)
        self.imu = imu
        rate_hz = float(
            self.declare_parameter("rate_hz", DEFAULT_RATE_HZ).value
        )
        temperature_hz = float(
            self.declare_parameter(
                "temperature_hz", DEFAULT_TEMPERATURE_HZ
            ).value
        )
        self.frame_id = str(
            self.declare_parameter("frame_id", DEFAULT_FRAME_ID).value
        )
        self.max_read_failures = int(
            self.declare_parameter(
                "max_read_failures", DEFAULT_MAX_READ_FAILURES
            ).value
        )
        self.angular_velocity_covariance = diagonal_covariance(
            float(
                self.declare_parameter(
                    "angular_velocity_variance",
                    DEFAULT_ANGULAR_VELOCITY_VARIANCE,
                ).value
            )
        )
        self.linear_acceleration_covariance = diagonal_covariance(
            float(
                self.declare_parameter(
                    "linear_acceleration_variance",
                    DEFAULT_LINEAR_ACCELERATION_VARIANCE,
                ).value
            )
        )
        if rate_hz <= 0 or temperature_hz <= 0:
            raise ValueError("rate_hz and temperature_hz must be > 0")
        if self.max_read_failures < 1:
            raise ValueError(
                f"max_read_failures must be >= 1, got {self.max_read_failures}"
            )

        if self.declare_parameter("calibrate", True).value:
            self.get_logger().info("calibrating gyro, keep the robot still")
            offset = imu.calibrate_gyro()
            self.get_logger().info(
                "gyro offset deg/s: " + " ".join(f"{v:+.3f}" for v in offset)
            )

        self.consecutive_failures = 0
        self.last_temperature: float | None = None
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", QUEUE_DEPTH)
        self.temperature_pub = self.create_publisher(
            Temperature, "imu/temperature", QUEUE_DEPTH
        )
        self.create_timer(1.0 / rate_hz, self.tick)
        self.create_timer(1.0 / temperature_hz, self.publish_temperature)
        self.get_logger().info(
            f"publishing imu/data_raw at {rate_hz} Hz in frame {self.frame_id}"
        )

    def tick(self) -> None:
        """Read one sample and publish it.

        Raises ImuError after too many failed reads in a row.
        """
        try:
            sample = self.imu.read()
        except ImuError as exc:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_read_failures:
                raise ImuError(
                    f"{self.consecutive_failures} imu reads failed in a row, "
                    f"last: {exc}"
                ) from exc
            self.get_logger().warning(
                f"imu read failed: {exc}", throttle_duration_sec=1.0
            )
            return
        self.consecutive_failures = 0
        self.last_temperature = sample.temperature
        self.imu_pub.publish(self.imu_msg(sample))

    def imu_msg(self, sample: ImuSample) -> Imu:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance = not_provided_covariance()
        msg.angular_velocity = _vector3(dps_to_radps(sample.gyro))
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        msg.linear_acceleration = _vector3(g_to_mps2(sample.accel))
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance
        return msg

    def publish_temperature(self) -> None:
        if self.last_temperature is None:
            return
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.temperature = self.last_temperature
        self.temperature_pub.publish(msg)


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        prog="talos-imu-ros", description=__doc__.splitlines()[0]
    )
    parser.add_argument(
        "--i2c-bus",
        type=int,
        default=os.environ.get(IMU_BUS_ENV_VAR, str(DEFAULT_I2C_BUS)),
        help=(
            f"i2c bus number, /dev/i2c-N (default: ${IMU_BUS_ENV_VAR} or "
            f"{DEFAULT_I2C_BUS})"
        ),
    )
    parser.add_argument(
        "--address",
        type=parse_int_any_base,
        default=DEFAULT_ADDRESS,
        help=(
            "i2c address, 0x69 if AD0 is high (default: "
            f"0x{DEFAULT_ADDRESS:02x})"
        ),
    )
    parser.add_argument(
        "--accel-range",
        type=int,
        choices=sorted(ACCEL_RANGES),
        default=DEFAULT_ACCEL_RANGE,
        help="accelerometer full scale in g (default: %(default)s)",
    )
    parser.add_argument(
        "--gyro-range",
        type=int,
        choices=sorted(GYRO_RANGES),
        default=DEFAULT_GYRO_RANGE,
        help="gyro full scale in deg/s (default: %(default)s)",
    )
    parser.add_argument(
        "--dlpf",
        type=int,
        choices=range(MAX_DLPF + 1),
        default=DEFAULT_DLPF,
        help=(
            "low-pass filter setting, higher is smoother and slower (default: "
            "%(default)s)"
        ),
    )
    # Everything else (e.g. --ros-args -p frame_id:=base_imu) goes to rclpy.
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.INFO)
    imu = Mpu6050(
        args.i2c_bus,
        args.address,
        accel_range=args.accel_range,
        gyro_range=args.gyro_range,
        dlpf=args.dlpf,
    )

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        # Leaving the with block puts the sensor to sleep and closes the bus,
        # even on Ctrl+C.
        with imu:
            node = ImuNode(imu)
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (ImuError, ValueError) as exc:
        print(f"talos-imu-ros: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
