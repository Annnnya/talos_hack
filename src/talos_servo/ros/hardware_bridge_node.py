"""ROS 2 node behind the `topic_based_ros2_control` hardware plugin.

The controller_manager loads `topic_based_ros2_control/TopicBasedSystem` as
its hardware. That plugin publishes joint commands and reads joint states on
two topics; this node answers them with the `Robot` on the servo bus. See
`docs/ros2_control.md`.

- `/talos/joint_commands` (sensor_msgs/JointState, in): velocity (rad/s) for
  speed joints, position (rad) for position joints. Wheels stop unless the
  command is repeated within `command_timeout_s`, so a dead controller_manager
  never leaves them spinning. The plugin sends the URDF `initial_value` of a
  position joint until a controller writes a goal, so each position joint
  ignores goals equal to it until the first different one: starting the
  stack never moves the hinge. A NaN position is no goal
- `/talos/joint_states` (sensor_msgs/JointState, out): position (rad, wheels
  keep counting past a full turn), velocity (rad/s), effort = current (A)

The hinge (`model.hinge_joint`, `bend`) is the opening angle alpha in rad, pi
flat, converted by `talos_servo.hinge.Hinge`. Its flat servo position is
`model.bend_flat_steps` (#84), or else where the hinge was when the bridge
started.
- `/hardware_bridge/<joint>/temperature`, `voltage`, `current`, `load`,
  `moving`: readouts the plugin cannot carry, see `talos_servo.ros.readouts`
- `/hardware_bridge/stop_all` (std_srvs/Trigger)

Like the robot node, this is the only program on the servo port: do not run
both. Run with `pixi run -e ros hardware-bridge`.
"""

from __future__ import annotations

import logging
import math
import sys
from collections.abc import Callable, Sequence
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from talos_servo.bus import ServoBus, ServoError
from talos_servo.config import SPEED, load_config
from talos_servo.defaults import DEFAULT_MOVE_SPEED, PORT_ENV_VAR, QUEUE_DEPTH
from talos_servo.hinge import FLAT_RAD, Hinge, HingeRangeError
from talos_servo.robot import SOURCE_ROS, CommandError, Robot
from talos_servo.ros.convert import (
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.port_args import DEFAULT_STATUS_HZ, parse_args
from talos_servo.ros.readouts import JointReadoutPublishers
from talos_servo.ros.topic_bridge import Unwrapper, unpack_commands

NODE_NAME = "hardware_bridge"
# Must match joint_commands_topic and joint_states_topic of the plugin in the
# URDF.
COMMANDS_TOPIC = "/talos/joint_commands"
STATES_TOPIC = "/talos/joint_states"
# Goals the plugin sends for a position joint before any controller writes
# one: the URDF initial_value of its position state interface (flat for the
# hinge, which must match description/talos.urdf.xacro), or 0.0, the plugin
# default.
HINGE_INITIAL_RAD = FLAT_RAD
PLUGIN_INITIAL_RAD = 0.0
INITIAL_TOLERANCE_RAD = 1e-9


class HardwareBridgeNode(Node):
    def __init__(
        self, robot: Robot, *, node_name: str = NODE_NAME, **node_kwargs
    ) -> None:
        super().__init__(node_name, **node_kwargs)
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
        model = config.model
        self._hinge_model = (
            model
            if model is not None and model.hinge_joint in robot.joints
            else None
        )
        self._hinge: Hinge | None = None
        # Position joints that got a goal other than the plugin's initial
        # value; only those follow goals.
        self._armed: set[str] = set()
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
        self._ensure_hinge(self._states())

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
        states = self._states()
        hinge = self._ensure_hinge(states)
        for name, j in states.items():
            msg.name.append(name)
            position, velocity = self._position_velocity(name, j, hinge)
            msg.position.append(position)
            msg.velocity.append(velocity)
            msg.effort.append(float(j["current"]))
        return msg

    def _position_velocity(
        self, name: str, j: dict[str, Any], hinge: Hinge | None
    ) -> tuple[float, float]:
        unwrapper = self._unwrappers.get(name)
        if unwrapper is not None:
            return unwrapper.update(j["position"]), step_speed_to_rad(
                j["speed"]
            )
        if self._is_hinge(name):
            if hinge is None:  # not read yet: report the flat assumption
                return HINGE_INITIAL_RAD, 0.0
            return hinge.alpha(j["position"]), hinge.alpha_rate(j["speed"])
        return steps_to_rad(j["position"]), step_speed_to_rad(j["speed"])

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
            if not self._armed_for(joint, position):
                continue
            self._safely(
                joint,
                lambda joint=joint, position=position: self._move(
                    joint, self._goal_steps(joint, position)
                ),
            )

    def _armed_for(self, joint: str, position: float) -> bool:
        """False while a position joint only gets the plugin's initial
        value."""
        if joint in self._armed:
            return True
        initial = (
            HINGE_INITIAL_RAD if self._is_hinge(joint) else PLUGIN_INITIAL_RAD
        )
        if math.isclose(position, initial, abs_tol=INITIAL_TOLERANCE_RAD):
            return False
        self._armed.add(joint)
        self.get_logger().info(
            f"{joint}: first goal {position:.3f} rad, following goals now"
        )
        return True

    def _goal_steps(self, joint: str, position: float) -> int:
        if not self._is_hinge(joint):
            return rad_to_steps(position)
        hinge = self._ensure_hinge(self._states())
        if hinge is None:
            raise CommandError(
                "hinge angle unknown until the servo answers, goal dropped"
            )
        return hinge.steps(position)

    def _move(self, joint: str, steps: int) -> None:
        if self._goals.get(joint) == steps:
            return
        # Forget the goal first: if the move fails, the next identical goal
        # is tried again instead of being skipped.
        self._goals.pop(joint, None)
        self.robot.move(joint, steps, DEFAULT_MOVE_SPEED, source=SOURCE_ROS)
        self._goals[joint] = steps

    def on_stop_all(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.robot.stop_all(source=SOURCE_ROS)
        # After a stop the controller may repeat its last goal; write it again.
        self._goals.clear()
        response.success, response.message = True, "stopped every speed joint"
        return response

    # --- hinge -----------------------------------------------------------
    def _is_hinge(self, joint: str) -> bool:
        return (
            self._hinge_model is not None
            and joint == self._hinge_model.hinge_joint
        )

    def _ensure_hinge(self, states: dict[str, dict[str, Any]]) -> Hinge | None:
        """The hinge converter, built once its flat position is known.

        Without a calibration (#84) the first reading counts as flat.
        """
        model = self._hinge_model
        if self._hinge is not None or model is None:
            return self._hinge
        flat = model.bend_flat_steps
        origin = "calibrated"
        if flat is None:
            j = states[model.hinge_joint]
            if not j["online"]:
                self.get_logger().error(
                    f"{model.hinge_joint}: servo offline, its angle is "
                    "unknown and its goals are dropped",
                    throttle_duration_sec=5.0,
                )
                return None
            flat, origin = j["position"], "not calibrated: start position"
        try:
            self._hinge = Hinge(model, flat)
        except HingeRangeError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return None
        low, high = self._hinge.step_range
        self.get_logger().info(
            f"{model.hinge_joint}: flat at step {flat} ({origin}), "
            f"steps {low}..{high} for "
            f"{math.degrees(model.bend_min_rad):g}.."
            f"{math.degrees(model.bend_max_rad):g} deg"
        )
        return self._hinge

    # --- helpers ---------------------------------------------------------
    def _states(self) -> dict[str, dict[str, Any]]:
        return {j["name"]: j for j in self.robot.snapshot()}

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
