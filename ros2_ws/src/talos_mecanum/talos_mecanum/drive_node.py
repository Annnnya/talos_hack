from __future__ import annotations

import math
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import DEFAULT_ACCELERATION, DEFAULT_BAUDRATE, ServoBus, ServoStatus
from talos_servo.mecanum import (
    DEFAULT_WHEEL_DIRECTIONS,
    DEFAULT_WHEEL_IDS,
    WHEEL_JOINT_NAMES,
    mecanum_wheel_speeds,
)
from talos_servo.ros.convert import WheelOdometer, step_speed_to_rad

COMMAND_TIMEOUT_S = 0.5
WATCHDOG_PERIOD_S = 0.1
DEFAULT_JOINT_STATE_RATE_HZ = 10.0
DEFAULT_DIAGNOSTICS_RATE_HZ = 1.0
READ_ERROR_LOG_THROTTLE_S = 5.0


class MecanumDriveNode(Node):
    def __init__(self) -> None:
        super().__init__("talos_mecanum_drive")
        self.declare_parameter("port", os.environ.get("TALOS_SERVO_PORT", ""))
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)
        self.declare_parameter("wheel_ids", list(DEFAULT_WHEEL_IDS))
        self.declare_parameter("wheel_directions", list(DEFAULT_WHEEL_DIRECTIONS))
        self.declare_parameter("wheelbase_m", 0.3)
        self.declare_parameter("track_width_m", 0.3)
        self.declare_parameter("steps_per_meter", 1000.0)
        self.declare_parameter("max_speed", 100)
        self.declare_parameter("acceleration", DEFAULT_ACCELERATION)
        self.declare_parameter("max_linear_speed", 0.1)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("joint_state_rate_hz", DEFAULT_JOINT_STATE_RATE_HZ)
        self.declare_parameter("diagnostics_rate_hz", DEFAULT_DIAGNOSTICS_RATE_HZ)

        self._port = str(self.get_parameter("port").value)
        self._baudrate = int(self.get_parameter("baudrate").value)
        self._wheel_ids = tuple(int(value) for value in self.get_parameter("wheel_ids").value)
        self._wheel_directions = tuple(
            int(value) for value in self.get_parameter("wheel_directions").value
        )
        self._wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self._track_width_m = float(self.get_parameter("track_width_m").value)
        self._steps_per_meter = float(self.get_parameter("steps_per_meter").value)
        self._max_speed = int(self.get_parameter("max_speed").value)
        self._acceleration = int(self.get_parameter("acceleration").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._joint_state_rate_hz = float(self.get_parameter("joint_state_rate_hz").value)
        self._diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)
        self._validate_parameters()

        self._bus: ServoBus | None = None
        self._enabled = False
        self._motion_active = False
        self._last_command_at: float | None = None
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(SetBool, "/base/enable", self._on_enable)
        self.create_service(Trigger, "/base/stop", self._on_stop)
        self.create_timer(WATCHDOG_PERIOD_S, self._watchdog)

        self._odometers = [WheelOdometer(direction) for direction in self._wheel_directions]
        self._last_raw: list[tuple[int, int] | None] = [None] * len(self._wheel_ids)
        self._last_status: list[ServoStatus | None] = [None] * len(self._wheel_ids)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / self._joint_state_rate_hz, self._publish_joint_states)
        self.create_timer(1.0 / self._diagnostics_rate_hz, self._publish_diagnostics)
        self.get_logger().info("Ready; call /base/enable with data=true before moving the robot")

    def _validate_parameters(self) -> None:
        if not self._wheel_ids or len(self._wheel_ids) != 4 or len(set(self._wheel_ids)) != 4:
            raise ValueError("wheel_ids must contain four unique servo IDs")
        if not 0 <= min(self._wheel_ids) <= max(self._wheel_ids) <= 253:
            raise ValueError("wheel_ids must be in 0..253")
        if not 0 <= self._acceleration <= 254:
            raise ValueError("acceleration must be in 0..254")
        for value, name in (
            (self._max_linear_speed, "max_linear_speed"),
            (self._max_angular_speed, "max_angular_speed"),
            (self._joint_state_rate_hz, "joint_state_rate_hz"),
            (self._diagnostics_rate_hz, "diagnostics_rate_hz"),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite value")
        mecanum_wheel_speeds(
            0.0,
            0.0,
            0.0,
            wheelbase_m=self._wheelbase_m,
            track_width_m=self._track_width_m,
            steps_per_meter=self._steps_per_meter,
            max_speed=self._max_speed,
            wheel_directions=self._wheel_directions,
        )

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            errors = self._deactivate()
            response.success = not errors
            response.message = "Drive disabled" if not errors else "; ".join(errors)
            return response
        if self._enabled:
            response.success = True
            response.message = "Drive is already enabled"
            return response
        if not self._port:
            response.success = False
            response.message = "Set the port parameter or TALOS_SERVO_PORT before enabling"
            return response

        bus: ServoBus | None = None
        bus_open = False
        try:
            bus = ServoBus(self._port, self._baudrate)
            bus.open()
            bus_open = True
            for servo_id in self._wheel_ids:
                bus.ping(servo_id)
            bus.enable_wheel_mode(self._wheel_ids)
        except Exception as exc:
            cleanup_errors = []
            if bus is not None and bus_open:
                cleanup_errors = self._deactivate_bus(bus)
            response.success = False
            response.message = f"Could not enable drive: {exc}"
            if cleanup_errors:
                response.message += "; cleanup failed: " + "; ".join(cleanup_errors)
            return response

        assert bus is not None
        self._reset_telemetry()
        self._bus = bus
        self._enabled = True
        self._last_command_at = time.monotonic()
        self._motion_active = False
        response.success = True
        response.message = "Drive enabled; waiting for /cmd_vel"
        self.get_logger().warning("Drive enabled; verify the area is clear")
        return response

    def _on_stop(self, _request: Trigger.Request, response: Trigger.Response):
        if self._bus is None:
            response.success = True
            response.message = "Drive is already stopped"
            return response
        errors = self._stop_wheels(self._bus)
        self._motion_active = False
        response.success = not errors
        response.message = "Wheels stopped" if not errors else "; ".join(errors)
        if errors:
            cleanup_errors = self._deactivate()
            response.message += "; drive disabled after stop failure"
            if cleanup_errors:
                response.message += "; " + "; ".join(cleanup_errors)
        return response

    def _on_cmd_vel(self, command: Twist) -> None:
        if not self._enabled or self._bus is None:
            return
        self._last_command_at = time.monotonic()
        linear_x = self._clamp(command.linear.x, self._max_linear_speed)
        linear_y = self._clamp(command.linear.y, self._max_linear_speed)
        angular_z = self._clamp(command.angular.z, self._max_angular_speed)
        try:
            speeds = mecanum_wheel_speeds(
                linear_x,
                linear_y,
                angular_z,
                wheelbase_m=self._wheelbase_m,
                track_width_m=self._track_width_m,
                steps_per_meter=self._steps_per_meter,
                max_speed=self._max_speed,
                wheel_directions=self._wheel_directions,
            )
            for servo_id, speed in zip(self._wheel_ids, speeds, strict=True):
                self._bus.set_speed(servo_id, speed, self._acceleration)
            self._motion_active = any(speeds)
        except Exception as exc:
            self.get_logger().error(f"Wheel command failed, disabling drive: {exc}")
            for error in self._deactivate():
                self.get_logger().error(error)

    def _watchdog(self) -> None:
        if (
            self._enabled
            and self._motion_active
            and self._last_command_at is not None
            and time.monotonic() - self._last_command_at > COMMAND_TIMEOUT_S
        ):
            self.get_logger().warning("No recent /cmd_vel; stopping wheels")
            errors = self._stop_wheels(self._bus)
            self._motion_active = False
            if errors:
                self.get_logger().error("Watchdog stop failed; disabling drive")
                for error in self._deactivate():
                    self.get_logger().error(error)

    def _reset_telemetry(self) -> None:
        for odometer in self._odometers:
            odometer.reset()
        self._last_raw = [None] * len(self._wheel_ids)
        self._last_status = [None] * len(self._wheel_ids)

    def _publish_joint_states(self) -> None:
        """Publish wheel angle, velocity and load. Reads only; never commands motion."""
        bus = self._bus
        if not self._enabled or bus is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for index, servo_id in enumerate(self._wheel_ids):
            try:
                position, speed = bus.read_position_speed(servo_id)
            except Exception as exc:
                self._last_raw[index] = None
                self.get_logger().error(
                    f"could not read position/speed of servo {servo_id}: {exc}",
                    throttle_duration_sec=READ_ERROR_LOG_THROTTLE_S,
                )
                continue
            self._last_raw[index] = (position, speed)
            odometer = self._odometers[index]
            status = self._last_status[index]
            msg.name.append(WHEEL_JOINT_NAMES[index])
            msg.position.append(odometer.update(position))
            msg.velocity.append(step_speed_to_rad(speed * odometer.direction))
            msg.effort.append(float(status.load * odometer.direction) if status else 0.0)
        if msg.name:
            self._joint_state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        """Publish per-servo readouts, with ERROR level on failed reads."""
        bus = self._bus
        if not self._enabled or bus is None:
            return
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for index, servo_id in enumerate(self._wheel_ids):
            entry = DiagnosticStatus(
                name=f"talos_base: {WHEEL_JOINT_NAMES[index]}",
                hardware_id=f"servo {servo_id}",
            )
            try:
                status = bus.read_status(servo_id)
            except Exception as exc:
                self._last_status[index] = None
                entry.level = DiagnosticStatus.ERROR
                entry.message = f"read failed: {exc}"
                self.get_logger().error(
                    f"could not read status of servo {servo_id}: {exc}",
                    throttle_duration_sec=READ_ERROR_LOG_THROTTLE_S,
                )
            else:
                self._last_status[index] = status
                entry.level = DiagnosticStatus.OK
                entry.message = "OK"
                entry.values = [
                    KeyValue(key="voltage_v", value=f"{status.voltage:.1f}"),
                    KeyValue(key="temperature_c", value=str(status.temperature)),
                    KeyValue(key="load", value=str(status.load)),
                    KeyValue(key="current_a", value=f"{status.current:.3f}"),
                    KeyValue(key="moving", value=str(status.moving)),
                ]
                raw = self._last_raw[index]
                if raw is not None:
                    entry.values += [
                        KeyValue(key="position_steps", value=str(raw[0])),
                        KeyValue(key="speed_steps_per_s", value=str(raw[1])),
                    ]
            msg.status.append(entry)
        self._diagnostics_pub.publish(msg)

    def _deactivate(self) -> list[str]:
        self._enabled = False
        self._motion_active = False
        self._last_command_at = None
        bus, self._bus = self._bus, None
        return self._deactivate_bus(bus) if bus is not None else []

    def _deactivate_bus(self, bus: ServoBus) -> list[str]:
        errors = self._stop_wheels(bus)
        for servo_id in self._wheel_ids:
            try:
                bus.set_torque(servo_id, False)
            except Exception as exc:
                errors.append(f"could not disable torque on servo {servo_id}: {exc}")
        try:
            bus.close()
        except Exception as exc:
            errors.append(f"could not close servo bus: {exc}")
        return errors

    def _stop_wheels(self, bus: ServoBus | None) -> list[str]:
        if bus is None:
            return []
        errors = []
        for servo_id in self._wheel_ids:
            try:
                bus.stop(servo_id, self._acceleration)
            except Exception as exc:
                errors.append(f"could not stop servo {servo_id}: {exc}")
        return errors

    def destroy_node(self) -> bool:
        errors = self._deactivate()
        for error in errors:
            self.get_logger().error(error)
        return super().destroy_node()

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MecanumDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
