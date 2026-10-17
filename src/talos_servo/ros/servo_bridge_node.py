"""ROS 2 node that exposes the servo bus: joint states out, position and velocity commands in.

Topics and services live under the node name (default `/servo_bridge/...`):

- `joint_states` (sensor_msgs/JointState): position rad, velocity rad/s, effort = raw load
- `status` (diagnostic_msgs/DiagnosticArray): online, mode, torque, voltage, temperature,
  current, moving, error
- `position_cmd` (sensor_msgs/JointState, in): goal angles by joint name, optional velocity
  (rad/s) as move speed
- `velocity_cmd` (sensor_msgs/JointState, in): wheel speeds by joint name. A wheel servo stops
  if it gets no new command within `command_timeout` seconds
- `<joint>/temperature`, `voltage`, `current`, `load`, `moving`: plain numeric readouts at the
  status rate, see `talos_servo.ros.readouts`
- `stop_all` (std_srvs/Trigger), `<joint>/set_torque` and `<joint>/set_wheel_mode`
  (std_srvs/SetBool)

Run with `pixi run -e ros servo-node` (or `servo-node-mock` for the simulator).
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections.abc import Callable, Sequence

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import DEFAULT_BAUDRATE, ServoError
from talos_servo.config import load_config
from talos_servo.controller import POSITION, WHEEL, CommandError, RobotController
from talos_servo.defaults import DEFAULT_CONFIG, PORT_ENV_VAR
from talos_servo.ros.convert import (
    joint_ids,
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.readouts import JointReadoutPublishers
from talos_servo.sim import make_bus

NODE_NAME = "servo_bridge"
DEFAULT_COMMAND_TIMEOUT = 0.5
DEFAULT_STATUS_HZ = 2.0
# Same low default as the web UI when a position command carries no velocity.
DEFAULT_MOVE_SPEED = 200
QUEUE_DEPTH = 10
# ServoState fields reported as key/values on the status topic.
STATUS_KEYS = (
    "mode",
    "torque",
    "position",
    "speed",
    "load",
    "voltage",
    "temp",
    "current",
    "moving",
)


class ServoBridgeNode(Node):
    def __init__(
        self,
        controller: RobotController,
        *,
        node_name: str = NODE_NAME,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__(node_name)
        self.controller = controller
        self._monotonic = clock
        self.joints = joint_ids(controller.config.servos)
        self.names = {servo_id: name for name, servo_id in self.joints.items()}
        self.command_timeout = float(
            self.declare_parameter("command_timeout", DEFAULT_COMMAND_TIMEOUT).value
        )
        status_hz = float(self.declare_parameter("status_hz", DEFAULT_STATUS_HZ).value)
        # servo id -> clock time of its last velocity command
        self._last_velocity_cmd: dict[int, float] = {}

        self.joint_pub = self.create_publisher(JointState, "~/joint_states", QUEUE_DEPTH)
        self.status_pub = self.create_publisher(DiagnosticArray, "~/status", QUEUE_DEPTH)
        self.readouts = JointReadoutPublishers(self, "~", self.joints)
        self.create_subscription(JointState, "~/position_cmd", self.on_position_cmd, QUEUE_DEPTH)
        self.create_subscription(JointState, "~/velocity_cmd", self.on_velocity_cmd, QUEUE_DEPTH)
        self.create_service(Trigger, "~/stop_all", self.on_stop_all)
        for name, servo_id in self.joints.items():
            self.create_service(SetBool, f"~/{name}/set_torque", self._torque_handler(servo_id))
            self.create_service(
                SetBool, f"~/{name}/set_wheel_mode", self._wheel_mode_handler(servo_id)
            )

        self.create_timer(1.0 / controller.config.telemetry_hz, self.tick)
        self.create_timer(1.0 / status_hz, self.publish_status)
        self.get_logger().info(
            f"serving joints {sorted(self.joints)}, command timeout {self.command_timeout} s"
        )

    # --- telemetry -------------------------------------------------------
    def tick(self) -> None:
        try:
            self.controller.poll()
        except Exception as exc:  # keep publishing and enforcing the watchdog
            self.get_logger().error(f"poll failed: {exc}")
        self.enforce_command_timeout()
        self.joint_pub.publish(self.joint_state_msg())

    def joint_state_msg(self) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for s in self.controller.snapshot():
            msg.name.append(self.names[s["id"]])
            msg.position.append(steps_to_rad(s["position"]))
            msg.velocity.append(step_speed_to_rad(s["speed"]))
            msg.effort.append(float(s["load"]))
        return msg

    def publish_status(self) -> None:
        self.status_pub.publish(self.status_msg())
        self.publish_readouts()

    def publish_readouts(self) -> None:
        """Numeric readouts for every online servo; offline ones show up in `status` instead."""
        stamp = self.get_clock().now().to_msg()
        for s in self.controller.snapshot():
            if not s["online"]:
                continue
            self.readouts.publish(
                self.names[s["id"]],
                stamp,
                temperature=s["temp"],
                voltage=s["voltage"],
                current=s["current"],
                load=s["load"],
                moving=s["moving"],
            )

    def status_msg(self) -> DiagnosticArray:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for s in self.controller.snapshot():
            status = DiagnosticStatus()
            status.name = f"{self.get_name()}: {self.names[s['id']]}"
            status.hardware_id = f"feetech id {s['id']}"
            if not s["online"]:
                status.level, status.message = DiagnosticStatus.ERROR, s["error"] or "offline"
            elif s["error"]:
                status.level, status.message = DiagnosticStatus.WARN, s["error"]
            else:
                status.level, status.message = DiagnosticStatus.OK, "ok"
            for key in STATUS_KEYS:
                status.values.append(KeyValue(key=key, value=str(s[key])))
            msg.status.append(status)
        return msg

    # --- commands --------------------------------------------------------
    def on_position_cmd(self, msg: JointState) -> None:
        if len(msg.position) != len(msg.name):
            self._reject("position_cmd needs one position per name")
            return
        has_speed = len(msg.velocity) == len(msg.name)
        for i, name in enumerate(msg.name):
            speed = abs(rad_speed_to_steps(msg.velocity[i])) if has_speed else 0
            self._run(
                name,
                lambda servo_id, i=i, speed=speed: self.controller.move(
                    servo_id, rad_to_steps(msg.position[i]), speed or DEFAULT_MOVE_SPEED
                ),
            )

    def on_velocity_cmd(self, msg: JointState) -> None:
        if len(msg.velocity) != len(msg.name):
            self._reject("velocity_cmd needs one velocity per name")
            return
        now = self._monotonic()
        for i, name in enumerate(msg.name):

            def spin(servo_id: int, i: int = i) -> None:
                self.controller.spin(servo_id, rad_speed_to_steps(msg.velocity[i]))
                self._last_velocity_cmd[servo_id] = now

            self._run(name, spin)

    def enforce_command_timeout(self) -> None:
        """Stop wheel servos whose last velocity command is older than command_timeout."""
        now = self._monotonic()
        for s in self.controller.snapshot():
            if s["mode"] != WHEEL or not s["target_speed"]:
                continue
            last = self._last_velocity_cmd.get(s["id"])
            if last is None or now - last > self.command_timeout:
                self.get_logger().warning(
                    f"{self.names[s['id']]}: no velocity command for {self.command_timeout} s, "
                    "stopping"
                )
                self._run(self.names[s["id"]], self.controller.stop)

    def on_stop_all(self, _request: Trigger.Request, response: Trigger.Response):
        try:
            self.controller.stop_all()
        except ServoError as exc:
            response.success, response.message = False, str(exc)
        else:
            response.success, response.message = True, "stopped all wheel-mode servos"
        return response

    def _torque_handler(self, servo_id: int):
        def handle(request: SetBool.Request, response: SetBool.Response):
            return self._service_call(
                response, lambda: self.controller.set_torque(servo_id, request.data)
            )

        return handle

    def _wheel_mode_handler(self, servo_id: int):
        def handle(request: SetBool.Request, response: SetBool.Response):
            mode = WHEEL if request.data else POSITION
            return self._service_call(response, lambda: self.controller.set_mode(servo_id, mode))

        return handle

    # --- helpers ---------------------------------------------------------
    def _run(self, name: str, action: Callable[[int], None]) -> None:
        """Run one per-joint command; bad commands are logged, never raised into rclpy."""
        servo_id = self.joints.get(name)
        if servo_id is None:
            self._reject(f"unknown joint {name!r}, expected one of {sorted(self.joints)}")
            return
        try:
            action(servo_id)
        except (CommandError, ServoError, ValueError) as exc:
            self._reject(f"{name}: {exc}")

    def _reject(self, message: str) -> None:
        self.get_logger().warning(message, throttle_duration_sec=1.0)

    @staticmethod
    def _service_call(response, action: Callable[[], None]):
        try:
            action()
        except (CommandError, ServoError) as exc:
            response.success, response.message = False, str(exc)
        else:
            response.success, response.message = True, "ok"
        return response


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(prog="talos-servo-ros", description=__doc__.splitlines()[0])
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="servo layout (%(default)s)")
    parser.add_argument(
        "--port",
        default=os.environ.get(PORT_ENV_VAR),
        help=f"serial port, e.g. /dev/ttyACM0 (default: ${PORT_ENV_VAR})",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE, help="(%(default)s)")
    parser.add_argument("--mock", action="store_true", help="simulate the servos, no serial port")
    # Everything else (e.g. --ros-args -p command_timeout:=1.0) goes to rclpy.
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.INFO)
    try:
        config = load_config(args.config)
        bus = make_bus(args.port, args.baud, mock=args.mock, ids=(s.id for s in config.servos))
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"talos-servo-ros: {exc}", file=sys.stderr)
        return 2
    controller = RobotController(bus, config)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        controller.open()
        # A crash may have left wheels spinning; start from a stopped robot.
        controller.stop_all()
        node = ServoBridgeNode(controller)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ServoError as exc:
        print(f"talos-servo-ros: {exc}", file=sys.stderr)
        return 1
    finally:
        # Stops every wheel-mode servo and closes the port, even on Ctrl+C.
        controller.close()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
