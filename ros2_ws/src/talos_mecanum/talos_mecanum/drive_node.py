from __future__ import annotations

import math
import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    DEFAULT_BAUDRATE,
    MAX_ACCELERATION,
    ServoBus,
    ServoTelemetry,
)
from talos_servo.mecanum import (
    DEFAULT_WHEEL_DIRECTIONS,
    DEFAULT_WHEEL_IDS,
    WHEEL_JOINT_NAMES,
    mecanum_wheel_speeds,
)
from talos_servo.mecanum_drive import Kinematics, MecanumDrive
from talos_servo.ros.convert import WheelOdometer, step_speed_to_rad
from talos_servo.ros.readouts import JointReadoutPublishers

COMMAND_TIMEOUT_S = 0.5
DEFAULT_CONTROL_RATE_HZ = 50.0
DEFAULT_JOINT_STATE_RATE_HZ = 10.0
DEFAULT_DIAGNOSTICS_RATE_HZ = 1.0
READ_ERROR_LOG_THROTTLE_S = 5.0
READOUT_PREFIX = "/base"


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
        self.declare_parameter("control_rate_hz", DEFAULT_CONTROL_RATE_HZ)
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
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._joint_state_rate_hz = float(self.get_parameter("joint_state_rate_hz").value)
        self._diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)
        self._validate_parameters()

        self._kinematics = Kinematics(
            wheelbase_m=self._wheelbase_m,
            track_width_m=self._track_width_m,
            steps_per_meter=self._steps_per_meter,
            max_speed=self._max_speed,
            wheel_directions=self._wheel_directions,
        )

        self._bus: ServoBus | None = None
        self._drive: MecanumDrive | None = None
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_service(SetBool, "/base/enable", self._on_enable)
        self.create_service(Trigger, "/base/stop", self._on_stop)

        count = len(self._wheel_ids)
        self._odometers = [WheelOdometer(direction) for direction in self._wheel_directions]
        self._wheel_angles: list[float | None] = [None] * count
        self._telemetry: list[ServoTelemetry | None] = [None] * count
        self._read_errors: list[str | None] = [None] * count
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._readouts = JointReadoutPublishers(self, READOUT_PREFIX, WHEEL_JOINT_NAMES)
        # One timer owns the bus. The others only schedule reads or publish cached data.
        self.create_timer(1.0 / self._control_rate_hz, self._control_tick)
        self.create_timer(1.0 / self._joint_state_rate_hz, self._request_telemetry)
        self.create_timer(1.0 / self._diagnostics_rate_hz, self._publish_diagnostics)
        self.get_logger().info("Ready; call /base/enable with data=true before moving the robot")

    def _validate_parameters(self) -> None:
        if not self._wheel_ids or len(self._wheel_ids) != 4 or len(set(self._wheel_ids)) != 4:
            raise ValueError("wheel_ids must contain four unique servo IDs")
        if not 0 <= min(self._wheel_ids) <= max(self._wheel_ids) <= 253:
            raise ValueError("wheel_ids must be in 0..253")
        # 0 means no ramp at all on STS servos, so the drive always ramps.
        if not 1 <= self._acceleration <= MAX_ACCELERATION:
            raise ValueError(f"acceleration must be in 1..{MAX_ACCELERATION}")
        for value, name in (
            (self._max_linear_speed, "max_linear_speed"),
            (self._max_angular_speed, "max_angular_speed"),
            (self._control_rate_hz, "control_rate_hz"),
            (self._joint_state_rate_hz, "joint_state_rate_hz"),
            (self._diagnostics_rate_hz, "diagnostics_rate_hz"),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite value")
        # Each telemetry cycle reads one wheel per control tick.
        if self._joint_state_rate_hz * len(self._wheel_ids) > self._control_rate_hz:
            raise ValueError(
                "joint_state_rate_hz times the wheel count must not exceed control_rate_hz"
            )
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
        if self._drive is not None:
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
                cleanup_errors = self._deactivate_bus(bus, None)
            response.success = False
            response.message = f"Could not enable drive: {exc}"
            if cleanup_errors:
                response.message += "; cleanup failed: " + "; ".join(cleanup_errors)
            return response

        assert bus is not None
        self._reset_telemetry()
        self._bus = bus
        self._drive = MecanumDrive(
            bus,
            self._wheel_ids,
            self._kinematics,
            self._acceleration,
            command_timeout_s=COMMAND_TIMEOUT_S,
        )
        response.success = True
        response.message = "Drive enabled; waiting for /cmd_vel"
        self.get_logger().warning("Drive enabled; verify the area is clear")
        return response

    def _on_stop(self, _request: Trigger.Request, response: Trigger.Response):
        if self._drive is None:
            response.success = True
            response.message = "Drive is already stopped"
            return response
        errors = self._drive.stop()
        response.success = not errors
        response.message = "Wheels stopped" if not errors else "; ".join(errors)
        if errors:
            cleanup_errors = self._deactivate()
            response.message += "; drive disabled after stop failure"
            if cleanup_errors:
                response.message += "; " + "; ".join(cleanup_errors)
        return response

    def _on_cmd_vel(self, command: Twist) -> None:
        """Store the command only; the control tick sends it, so it never waits on reads."""
        if self._drive is None:
            return
        self._drive.set_command(
            self._clamp(command.linear.x, self._max_linear_speed),
            self._clamp(command.linear.y, self._max_linear_speed),
            self._clamp(command.angular.z, self._max_angular_speed),
            now=time.monotonic(),
        )

    def _request_telemetry(self) -> None:
        if self._drive is not None:
            self._drive.request_reads(len(self._wheel_ids))

    def _control_tick(self) -> None:
        drive = self._drive
        if drive is None:
            return
        try:
            result = drive.tick(time.monotonic())
        except Exception as exc:
            # DriveFault for bus failures; anything else is a bug, and stopping is still right.
            self.get_logger().error(f"{exc}; disabling drive")
            for error in self._deactivate():
                self.get_logger().error(error)
            return
        if result.timed_out:
            self.get_logger().warning("No recent /cmd_vel; stopping wheels")
        if result.read_index is None:
            return
        index = result.read_index
        if result.telemetry is None:
            self._telemetry[index] = None
            self._wheel_angles[index] = None
            self._read_errors[index] = str(result.read_error)
            self.get_logger().error(
                f"could not read servo {self._wheel_ids[index]}: {result.read_error}",
                throttle_duration_sec=READ_ERROR_LOG_THROTTLE_S,
            )
        else:
            self._telemetry[index] = result.telemetry
            self._wheel_angles[index] = self._odometers[index].update(result.telemetry.position)
            self._read_errors[index] = None
        if index == len(self._wheel_ids) - 1:
            self._publish_joint_states()

    def _reset_telemetry(self) -> None:
        for odometer in self._odometers:
            odometer.reset()
        count = len(self._wheel_ids)
        self._wheel_angles = [None] * count
        self._telemetry = [None] * count
        self._read_errors = [None] * count

    def _publish_joint_states(self) -> None:
        """Publish wheel angle, velocity and load from the read cycle that just finished."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for index, telemetry in enumerate(self._telemetry):
            angle = self._wheel_angles[index]
            if telemetry is None or angle is None:
                continue
            direction = self._odometers[index].direction
            msg.name.append(WHEEL_JOINT_NAMES[index])
            msg.position.append(angle)
            msg.velocity.append(step_speed_to_rad(telemetry.speed * direction))
            msg.effort.append(float(telemetry.status.load * direction))
        if msg.name:
            self._joint_state_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        """Publish the latest per-servo readouts. Never touches the bus."""
        if self._drive is None:
            return
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for index, servo_id in enumerate(self._wheel_ids):
            entry = DiagnosticStatus(
                name=f"talos_base: {WHEEL_JOINT_NAMES[index]}",
                hardware_id=f"servo {servo_id}",
            )
            telemetry = self._telemetry[index]
            if telemetry is None:
                entry.level = DiagnosticStatus.ERROR
                entry.message = f"read failed: {self._read_errors[index] or 'no data yet'}"
                msg.status.append(entry)
                continue
            status = telemetry.status
            entry.level = DiagnosticStatus.OK
            entry.message = "OK"
            entry.values = [
                KeyValue(key="voltage_v", value=f"{status.voltage:.1f}"),
                KeyValue(key="temperature_c", value=str(status.temperature)),
                KeyValue(key="load", value=str(status.load)),
                KeyValue(key="current_a", value=f"{status.current:.3f}"),
                KeyValue(key="moving", value=str(status.moving)),
                KeyValue(key="position_steps", value=str(telemetry.position)),
                KeyValue(key="speed_steps_per_s", value=str(telemetry.speed)),
            ]
            self._readouts.publish(
                WHEEL_JOINT_NAMES[index],
                msg.header.stamp,
                temperature=status.temperature,
                voltage=status.voltage,
                current=status.current,
                load=status.load * self._odometers[index].direction,
                moving=status.moving,
            )
            msg.status.append(entry)
        self._diagnostics_pub.publish(msg)

    def _deactivate(self) -> list[str]:
        bus, self._bus = self._bus, None
        drive, self._drive = self._drive, None
        return self._deactivate_bus(bus, drive) if bus is not None else []

    def _deactivate_bus(self, bus: ServoBus, drive: MecanumDrive | None) -> list[str]:
        if drive is not None:
            errors = drive.stop()
        else:
            errors = []
            for servo_id in self._wheel_ids:
                try:
                    bus.stop(servo_id, self._acceleration)
                except Exception as exc:
                    errors.append(f"could not stop servo {servo_id}: {exc}")
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
    except (KeyboardInterrupt, ExternalShutdownException):
        # On Jazzy, Ctrl+C shuts the context down itself and spin raises ExternalShutdownException.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
