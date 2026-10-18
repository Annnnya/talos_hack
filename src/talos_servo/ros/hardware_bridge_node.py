"""ROS 2 node behind the `topic_based_ros2_control` hardware plugin.

The controller_manager loads `topic_based_ros2_control/TopicBasedSystem` as
its hardware. That plugin publishes joint commands and reads joint states on
two topics; this node answers them with the `Robot` on the servo bus. See
`docs/ros2_control.md`.

- `/talos/joint_commands` (sensor_msgs/JointState, in): velocity (rad/s) for
  speed joints, position (rad) for position joints. Wheels stop unless the
  command is repeated within `command_timeout_s`, so a dead controller_manager
  never leaves them spinning
- `/talos/joint_states` (sensor_msgs/JointState, out): position (rad, wheels
  keep counting past a full turn), velocity (rad/s), effort = current (A)
- `/hardware_bridge/<joint>/temperature`, `voltage`, `current`, `load`,
  `moving`: readouts the plugin cannot carry, see `talos_servo.ros.readouts`
- `/hardware_bridge/stop_all` (std_srvs/Trigger)

Like the robot node, this is the only program on the servo port: do not run
both. Run with `pixi run -e ros hardware-bridge`.
"""

from __future__ import annotations

import logging
import sys
from collections.abc import Callable, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from talos_servo.bus import ServoBus, ServoError
from talos_servo.config import SPEED, load_config
from talos_servo.defaults import DEFAULT_MOVE_SPEED, PORT_ENV_VAR, QUEUE_DEPTH
from talos_servo.robot import SOURCE_ROS, CommandError, Robot
from talos_servo.ros.convert import (
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.readouts import JointReadoutPublishers
from talos_servo.ros.robot_node import DEFAULT_STATUS_HZ, parse_args
from talos_servo.ros.topic_bridge import Unwrapper, unpack_commands

NODE_NAME = "hardware_bridge"
# Must match joint_commands_topic and joint_states_topic of the plugin in the
# URDF.
COMMANDS_TOPIC = "/talos/joint_commands"
STATES_TOPIC = "/talos/joint_states"


class HardwareBridgeNode(Node):
    def __init__(self, robot: Robot, *, node_name: str = NODE_NAME) -> None:
        super().__init__(node_name)
        self.robot = robot
        config = robot.config
        status_hz = float(
            self.declare_parameter("status_hz", DEFAULT_STATUS_HZ).value
        )
        self._controls = {name: j.control for name, j in robot.joints.items()}
        self._unwrappers = {
            name: Unwrapper()
            for name, control in self._controls.items()
            if control == SPEED
        }
        # Last goal sent per position joint. The plugin repeats the same goal
        # every cycle; only a change is written to the servo.
        self._goals: dict[str, int] = {}

        # The plugin subscribes best effort, so sensor data QoS fits.
        self.states_pub = self.create_publisher(
            JointState, STATES_TOPIC, qos_profile_sensor_data
        )
        self.readouts = JointReadoutPublishers(self, "~", robot.joints)
        self.create_subscription(
            JointState, COMMANDS_TOPIC, self.on_commands, QUEUE_DEPTH
        )
        self.create_service(Trigger, "~/stop_all", self.on_stop_all)
        self.create_timer(1.0 / config.control_hz, self.tick)
        self.create_timer(1.0 / config.telemetry_hz, self.publish_states)
        self.create_timer(1.0 / status_hz, self.publish_readouts)
        self.get_logger().info(
            f"bridging joints {sorted(robot.joints)} on {COMMANDS_TOPIC} and "
            f"{STATES_TOPIC}, command timeout {config.command_timeout_s} s"
        )

    # --- states ----------------------------------------------------------
    def tick(self) -> None:
        try:
            self.robot.tick()
        except Exception as exc:  # keep ticking: the watchdog lives in tick()
            self.get_logger().error(
                f"tick failed: {exc}", throttle_duration_sec=1.0
            )

    def publish_states(self) -> None:
        self.states_pub.publish(self.states_msg())

    def states_msg(self) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in self.robot.snapshot():
            name = j["name"]
            unwrapper = self._unwrappers.get(name)
            msg.name.append(name)
            msg.position.append(
                steps_to_rad(j["position"])
                if unwrapper is None
                else unwrapper.update(j["position"])
            )
            msg.velocity.append(step_speed_to_rad(j["speed"]))
            msg.effort.append(float(j["current"]))
        return msg

    def publish_readouts(self) -> None:
        self.readouts.publish_snapshot(
            self.robot.snapshot(), self.get_clock().now().to_msg()
        )

    # --- commands --------------------------------------------------------
    def on_commands(self, msg: JointState) -> None:
        try:
            positions, velocities = unpack_commands(
                msg.name, msg.position, msg.velocity, self._controls
            )
        except ValueError as exc:
            self._reject(f"joint_commands: {exc}")
            return
        for joint, velocity in velocities.items():
            self._safely(
                joint,
                lambda joint=joint, velocity=velocity: self.robot.spin(
                    joint,
                    rad_speed_to_steps(velocity),
                    source=SOURCE_ROS,
                    expires=True,
                ),
            )
        for joint, position in positions.items():
            self._safely(
                joint,
                lambda joint=joint, position=position: self._move(
                    joint, rad_to_steps(position)
                ),
            )

    def _move(self, joint: str, steps: int) -> None:
        if self._goals.get(joint) == steps:
            return
        self.robot.move(joint, steps, DEFAULT_MOVE_SPEED, source=SOURCE_ROS)
        self._goals[joint] = steps

    def on_stop_all(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.robot.stop_all(source=SOURCE_ROS)
        response.success, response.message = True, "stopped every speed joint"
        return response

    # --- helpers ---------------------------------------------------------
    def _safely(self, what: str, action: Callable[[], None]) -> None:
        """Run one command; bad commands are logged, never raised into rclpy."""
        try:
            action()
        except (CommandError, ServoError, ValueError) as exc:
            self._reject(f"{what}: {exc}")

    def _reject(self, message: str) -> None:
        self.get_logger().warning(message, throttle_duration_sec=1.0)


def main(argv: Sequence[str] | None = None) -> int:
    prog = "talos-hardware-bridge-ros"
    args, ros_args = parse_args(
        sys.argv[1:] if argv is None else argv, prog=prog
    )
    logging.basicConfig(level=logging.INFO)
    if not args.port:
        print(f"{prog}: pass --port or set {PORT_ENV_VAR}", file=sys.stderr)
        return 2
    try:
        config = load_config(args.config)
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"{prog}: {exc}", file=sys.stderr)
        return 2
    robot = Robot(ServoBus(args.port, args.baud), config)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        robot.open()
        node = HardwareBridgeNode(robot)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ServoError as exc:
        print(f"{prog}: {exc}", file=sys.stderr)
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
