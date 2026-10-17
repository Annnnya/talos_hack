"""ROS 2 node that owns the robot: the only program on the servo serial port.

- `/cmd_vel` (geometry_msgs/Twist, in): drive the mecanum base. Stops unless repeated within
  `command_timeout_s` from the config
- `/odom` (nav_msgs/Odometry): wheel odometry in frame `odom`, child
  `base_link`, no TF (the EKF publishes odom -> base_link)
- `/joint_states` (sensor_msgs/JointState): position rad, velocity rad/s, effort = raw load
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): one entry per joint named `robot: <joint>`
  with online, control, torque, voltage, temperature, current, moving, error, last command
- `/robot/position_cmd` (sensor_msgs/JointState, in): goal angles by joint name, optional
  velocity (rad/s) as move speed
- `/robot/velocity_cmd` (sensor_msgs/JointState, in): speed joint velocities by name. Stops
  unless repeated within `command_timeout_s`
- `/robot/<joint>/temperature`, `voltage`, `current`, `load`, `moving`: plain numeric readouts,
  see `talos_servo.ros.readouts`
- `/robot/stop_all` (std_srvs/Trigger), `/robot/<joint>/set_torque` (std_srvs/SetBool)

Run with `pixi run -e ros robot-node`.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
from collections.abc import Callable, Sequence

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import DEFAULT_BAUDRATE, ServoBus, ServoError
from talos_servo.config import load_config
from talos_servo.defaults import (
    DEFAULT_CONFIG,
    DEFAULT_MOVE_SPEED,
    PORT_ENV_VAR,
    QUEUE_DEPTH,
    ROBOT_NODE_NAME,
)
from talos_servo.mecanum import integrate_pose
from talos_servo.robot import SOURCE_ROS, CommandError, Robot
from talos_servo.ros.convert import (
    parse_command_frame,
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.readouts import JointReadoutPublishers

DEFAULT_STATUS_HZ = 2.0
JOINT_STATES_TOPIC = "/joint_states"
DIAGNOSTICS_TOPIC = "/diagnostics"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
# Twist covariance for vx, vy and yaw rate. Wheel odometry is only a rough
# guess on mecanum wheels, which slip; the EKF weighs it against the IMU.
ODOM_VELOCITY_VARIANCE = 0.01
# Index of each fused field in a row-major 6x6 covariance.
COVARIANCE_DIAGONAL = (0, 7, 14, 21, 28, 35)
VX, VY, WZ = 0, 1, 5
# Joint fields reported as key/values on /diagnostics.
STATUS_KEYS = (
    "control",
    "torque",
    "position",
    "speed",
    "load",
    "voltage",
    "temp",
    "current",
    "moving",
    # Last command, so clients such as the web UI can show who moved a joint and when.
    "target",
    "target_speed",
    "command_source",
    "command_client",
    "command_age",
)


class RobotNode(Node):
    def __init__(
        self, robot: Robot, *, node_name: str = ROBOT_NODE_NAME
    ) -> None:
        super().__init__(node_name)
        self.robot = robot
        config = robot.config
        status_hz = float(
            self.declare_parameter("status_hz", DEFAULT_STATUS_HZ).value
        )

        self.joint_pub = self.create_publisher(
            JointState, JOINT_STATES_TOPIC, QUEUE_DEPTH
        )
        self.status_pub = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, QUEUE_DEPTH
        )
        self.readouts = JointReadoutPublishers(self, "~", robot.joints)
        self.create_subscription(
            Twist, CMD_VEL_TOPIC, self.on_cmd_vel, QUEUE_DEPTH
        )
        self.create_subscription(
            JointState, "~/position_cmd", self.on_position_cmd, QUEUE_DEPTH
        )
        self.create_subscription(
            JointState, "~/velocity_cmd", self.on_velocity_cmd, QUEUE_DEPTH
        )
        self.create_service(Trigger, "~/stop_all", self.on_stop_all)
        for joint in robot.joints:
            self.create_service(
                SetBool, f"~/{joint}/set_torque", self._torque_handler(joint)
            )

        self.odom_pub = None
        self._pose = (0.0, 0.0, 0.0)
        self._odom_at = self.get_clock().now()
        if config.base is not None:
            self.odom_pub = self.create_publisher(
                Odometry, ODOM_TOPIC, QUEUE_DEPTH
            )
        self.create_timer(1.0 / config.control_hz, self.tick)
        self.create_timer(1.0 / config.telemetry_hz, self.publish_joint_states)
        self.create_timer(1.0 / status_hz, self.publish_status)
        self.get_logger().info(
            f"serving joints {sorted(robot.joints)}, command timeout {config.command_timeout_s} s"
        )

    # --- telemetry -------------------------------------------------------
    def tick(self) -> None:
        try:
            self.robot.tick()
        except Exception as exc:  # keep ticking: the watchdog lives in tick()
            self.get_logger().error(
                f"tick failed: {exc}", throttle_duration_sec=1.0
            )

    def publish_joint_states(self) -> None:
        self.joint_pub.publish(self.joint_state_msg())
        if self.odom_pub is not None:
            self.odom_pub.publish(self.odom_msg())

    def odom_msg(self) -> Odometry:
        """Integrate the measured base velocity since the last call."""
        now = self.get_clock().now()
        dt = (now - self._odom_at).nanoseconds / 1e9
        self._odom_at = now
        velocity = self.robot.base_velocity()
        self._pose = integrate_pose(self._pose, velocity, dt)
        x, y, yaw = self._pose
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ODOM_FRAME
        msg.child_frame_id = BASE_FRAME
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation = _yaw_quaternion(yaw)
        twist = msg.twist.twist
        twist.linear.x, twist.linear.y, twist.angular.z = velocity
        covariance = [0.0] * 36
        for field in (VX, VY, WZ):
            covariance[COVARIANCE_DIAGONAL[field]] = ODOM_VELOCITY_VARIANCE
        msg.twist.covariance = covariance
        return msg

    def joint_state_msg(self) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in self.robot.snapshot():
            msg.name.append(j["name"])
            msg.position.append(steps_to_rad(j["position"]))
            msg.velocity.append(step_speed_to_rad(j["speed"]))
            msg.effort.append(float(j["load"]))
        return msg

    def publish_status(self) -> None:
        self.status_pub.publish(self.status_msg())
        self.publish_readouts()

    def publish_readouts(self) -> None:
        """Numeric readouts for every online joint; offline ones show up in /diagnostics."""
        stamp = self.get_clock().now().to_msg()
        for j in self.robot.snapshot():
            if not j["online"]:
                continue
            self.readouts.publish(
                j["name"],
                stamp,
                temperature=j["temp"],
                voltage=j["voltage"],
                current=j["current"],
                load=j["load"],
                moving=j["moving"],
            )

    def status_msg(self) -> DiagnosticArray:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in self.robot.snapshot():
            status = DiagnosticStatus()
            status.name = f"{self.get_name()}: {j['name']}"
            status.hardware_id = f"feetech id {j['id']}"
            if not j["online"]:
                status.level, status.message = (
                    DiagnosticStatus.ERROR,
                    j["error"] or "offline",
                )
            elif j["error"]:
                status.level, status.message = DiagnosticStatus.WARN, j["error"]
            else:
                status.level, status.message = DiagnosticStatus.OK, "ok"
            last = j["last_command"] or {}
            fields = {
                **j,
                "command_source": last.get("source"),
                "command_client": last.get("client"),
                "command_age": last.get("age"),
            }
            for key in STATUS_KEYS:
                value = fields[key]
                status.values.append(
                    KeyValue(key=key, value="" if value is None else str(value))
                )
            msg.status.append(status)
        return msg

    # --- commands --------------------------------------------------------
    def on_cmd_vel(self, msg: Twist) -> None:
        self._safely(
            "cmd_vel",
            lambda: self.robot.drive(msg.linear.x, msg.linear.y, msg.angular.z),
        )

    def on_position_cmd(self, msg: JointState) -> None:
        if len(msg.position) != len(msg.name):
            self._reject("position_cmd needs one position per name")
            return
        has_speed = len(msg.velocity) == len(msg.name)
        source, client = parse_command_frame(msg.header.frame_id)
        for i, joint in enumerate(msg.name):
            speed = abs(rad_speed_to_steps(msg.velocity[i])) if has_speed else 0
            self._safely(
                joint,
                lambda i=i, joint=joint, speed=speed: self.robot.move(
                    joint,
                    rad_to_steps(msg.position[i]),
                    speed or DEFAULT_MOVE_SPEED,
                    source=source,
                    client=client,
                ),
            )

    def on_velocity_cmd(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            self._reject("velocity_cmd needs one velocity per name")
            return
        source, client = parse_command_frame(msg.header.frame_id)
        for i, joint in enumerate(msg.name):
            self._safely(
                joint,
                lambda i=i, joint=joint: self.robot.spin(
                    joint,
                    rad_speed_to_steps(msg.velocity[i]),
                    source=source,
                    client=client,
                    expires=True,
                ),
            )

    def on_stop_all(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        self.robot.stop_all(source=SOURCE_ROS)
        response.success, response.message = True, "stopped every speed joint"
        return response

    def _torque_handler(self, joint: str):
        def handle(request: SetBool.Request, response: SetBool.Response):
            try:
                self.robot.set_torque(joint, request.data)
            except (CommandError, ServoError) as exc:
                response.success, response.message = False, str(exc)
            else:
                response.success, response.message = True, "ok"
            return response

        return handle

    # --- helpers ---------------------------------------------------------
    def _safely(self, what: str, action: Callable[[], None]) -> None:
        """Run one command; bad commands are logged, never raised into rclpy."""
        try:
            action()
        except (CommandError, ServoError, ValueError) as exc:
            self._reject(f"{what}: {exc}")

    def _reject(self, message: str) -> None:
        self.get_logger().warning(message, throttle_duration_sec=1.0)


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        prog="talos-robot-ros", description=__doc__.splitlines()[0]
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG, help="robot config (%(default)s)"
    )
    parser.add_argument(
        "--port",
        default=os.environ.get(PORT_ENV_VAR),
        help=f"serial port, e.g. /dev/ttyACM0 (default: ${PORT_ENV_VAR})",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUDRATE, help="(%(default)s)"
    )
    # Everything else (e.g. --ros-args -p status_hz:=5.0) goes to rclpy.
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.INFO)
    if not args.port:
        print(
            f"talos-robot-ros: pass --port or set {PORT_ENV_VAR}",
            file=sys.stderr,
        )
        return 2
    try:
        config = load_config(args.config)
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"talos-robot-ros: {exc}", file=sys.stderr)
        return 2
    robot = Robot(ServoBus(args.port, args.baud), config)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        robot.open()
        node = RobotNode(robot)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ServoError as exc:
        print(f"talos-robot-ros: {exc}", file=sys.stderr)
        return 1
    finally:
        # Stops every speed joint and closes the port, even on Ctrl+C.
        robot.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
