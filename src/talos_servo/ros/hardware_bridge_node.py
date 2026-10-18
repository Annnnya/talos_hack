"""ROS 2 node behind the `topic_based_ros2_control` hardware plugin.

The controller_manager loads `topic_based_ros2_control/TopicBasedSystem` as
its hardware. That plugin publishes joint commands and reads joint states on
two topics; this node answers them with the `Robot` on the servo bus. See
`docs/ros2_control.md`.

- `/talos/joint_commands` (sensor_msgs/JointState, in): velocity (rad/s) for
  speed joints, position (rad) for position joints. Wheels stop unless the
  command is repeated within `command_timeout_s`, so a dead controller_manager
  never leaves them spinning. The plugin repeats the last goal of a position
  joint (its URDF `initial_value` before any controller writes one, or a
  running controller's last goal when only this node restarts). So each
  position joint ignores the first goal it gets and every repeat of it, and
  follows goals from the first different one. The same holds again after
  the commands go quiet for `command_timeout_s` (controller_manager
  restarted). So starting or restarting either side never moves the hinge.
  A NaN position is no goal
- `/talos/joint_states` (sensor_msgs/JointState, out): position (rad, wheels
  keep counting past a full turn), velocity (rad/s), effort = current (A)
- `/hardware_bridge/<joint>/temperature`, `voltage`, `current`, `load`,
  `moving`: readouts the plugin cannot carry, see `talos_servo.ros.readouts`
- `/hardware_bridge/stop_all` (std_srvs/Trigger)

The hinge (`model.hinge_joint`, `bend`) is the opening angle alpha in rad, pi
flat, converted by `talos_servo.hinge.Hinge` from the fixed flat position
`model.bend_flat_steps`. While the servo is offline, or reads outside the
hinge range (a wrap past flat), its position and velocity are NaN: unknown,
never a guess. Its goals are dropped meanwhile: from a wrapped reading the
servo would take the long way round.

Safe state (`Robot.hold_safe`): wheels at speed 0 with torque on (braked), the
hinge on its last goal with torque on. Torque is never turned off.
- Deactivate or a dead controller_manager: the plugin just goes quiet, so the
  wheels stop after `command_timeout_s` and the hinge keeps its goal
- Read or write error: the `Robot` brakes at once and latches; moving
  commands are refused (logged) until the lost joints answer again and wheel
  writes work for `talos_servo.robot.FAULT_CLEAR_S`
- Shutdown (Ctrl+C, SIGTERM, an exception): `Robot.close()` in `finally`

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
from talos_servo.hinge import Hinge
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
# Goals closer than this to a position joint's first goal count as repeats.
GOAL_TOLERANCE_RAD = 1e-9
# A hinge reading this far outside its step range is not a real angle (the
# servo wrapped past flat or misread), so it is reported as unknown.
HINGE_READ_MARGIN_STEPS = 64


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
        hinge = config.hinge
        self._hinge: Hinge | None = (
            hinge if hinge is not None and hinge.joint in robot.joints else None
        )
        # First goal per position joint, ignored with its repeats; joints in
        # _armed got a different one and follow goals.
        self._first_goals: dict[str, float] = {}
        self._armed: set[str] = set()
        # Clock time of the last command message, to notice a quiet stream.
        self._last_commands_at: float | None = None
        # Last goal sent per position joint. The plugin repeats the same goal
        # every cycle; only a change is written to the servo.
        self._goals: dict[str, int] = {}
        # Last fault seen on the robot, to log changes only.
        self._fault: str | None = None

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
        if self._hinge is not None:
            low, high = self._hinge.step_range
            self.get_logger().info(
                f"{self._hinge.joint}: flat at step {self._hinge.flat_steps}, "
                f"steps {low}..{high} for "
                f"{math.degrees(self._hinge.min_rad):g}.."
                f"{math.degrees(self._hinge.max_rad):g} deg"
            )

    # --- states ----------------------------------------------------------
    def tick(self) -> None:
        try:
            self.robot.tick()
        except Exception as exc:  # keep ticking: the watchdog lives in tick()
            self.get_logger().error(
                f"tick failed: {exc}", throttle_duration_sec=1.0
            )
        self._report_fault()

    def _report_fault(self) -> None:
        """Log once when the safe state latches and once when it clears."""
        fault = self.robot.fault
        if fault == self._fault:
            return
        if fault is None:
            self.get_logger().info("fault cleared, following commands again")
        else:
            self.get_logger().error(
                f"safe state: {fault}. Wheels braked, goals held, commands "
                "that move are refused until the bus is healthy"
            )
        self._fault = fault

    def publish_states(self) -> None:
        self.states_pub.publish(self.states_msg())

    def states_msg(self) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name, j in self._states().items():
            msg.name.append(name)
            position, velocity = self._position_velocity(name, j)
            msg.position.append(position)
            msg.velocity.append(velocity)
            msg.effort.append(float(j["current"]))
        return msg

    def _position_velocity(
        self, name: str, j: dict[str, Any]
    ) -> tuple[float, float]:
        unwrapper = self._unwrappers.get(name)
        if unwrapper is not None:
            return unwrapper.update(j["position"]), step_speed_to_rad(
                j["speed"]
            )
        hinge = self._hinge
        if hinge is not None and name == hinge.joint:
            return self._hinge_state(hinge, j)
        return steps_to_rad(j["position"]), step_speed_to_rad(j["speed"])

    def publish_readouts(self) -> None:
        self.readouts.publish_snapshot(
            self.robot.snapshot(), self.get_clock().now().to_msg()
        )

    # --- commands --------------------------------------------------------
    def on_commands(self, msg: JointState) -> None:
        self._disarm_if_quiet()
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

    def _disarm_if_quiet(self) -> None:
        """After a quiet spell the plugin may be new and resend its initial
        value, so position joints wait for a new goal again."""
        now = self.robot.clock()
        last, self._last_commands_at = self._last_commands_at, now
        if last is None or now - last <= self.robot.config.command_timeout_s:
            return
        if self._armed:
            self.get_logger().warning(
                f"no commands for {now - last:.1f} s: position joints "
                f"{sorted(self._armed)} hold until a new goal"
            )
        self._armed.clear()
        self._first_goals.clear()
        self._goals.clear()

    def _armed_for(self, joint: str, position: float) -> bool:
        """False while a position joint only gets its first goal again."""
        if joint in self._armed:
            return True
        first = self._first_goals.get(joint)
        if first is None:
            self._first_goals[joint] = position
            self.get_logger().info(
                f"{joint}: holding, ignoring the repeated goal "
                f"{position:.3f} rad until a new one"
            )
            return False
        if math.isclose(position, first, abs_tol=GOAL_TOLERANCE_RAD):
            return False
        self._armed.add(joint)
        self.get_logger().info(
            f"{joint}: new goal {position:.3f} rad, following goals now"
        )
        return True

    def _goal_steps(self, joint: str, position: float) -> int:
        hinge = self._hinge
        if hinge is None or joint != hinge.joint:
            return rad_to_steps(position)
        j = self.robot.joints[joint]
        problem = self._hinge_problem(hinge, j.online, j.position)
        if problem is not None:
            raise CommandError(f"{problem}, angle unknown, goal dropped")
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
    def _hinge_state(
        self, hinge: Hinge, j: dict[str, Any]
    ) -> tuple[float, float]:
        """Opening angle and rate, or NaN while they are unknown."""
        problem = self._hinge_problem(hinge, j["online"], j["position"])
        if problem is None:
            return hinge.alpha(j["position"]), hinge.alpha_rate(j["speed"])
        self.get_logger().error(
            f"{hinge.joint}: {problem}, angle unknown",
            throttle_duration_sec=5.0,
        )
        return math.nan, math.nan

    @staticmethod
    def _hinge_problem(hinge: Hinge, online: bool, steps: int) -> str | None:
        """Why the hinge reading is no real angle, or None if it is."""
        if not online:
            return "servo offline"
        low, high = hinge.step_range
        margin = HINGE_READ_MARGIN_STEPS
        if not low - margin <= steps <= high + margin:
            return f"reads step {steps}, outside its range {low}..{high}"
        return None

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
