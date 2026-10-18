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
  keep counting past a full turn), velocity (rad/s), effort = current (A).
  Sent after every `Robot.tick` (`control_hz`), which reads every wheel's
  position and speed, so the mecanum odometry gets fresh wheel states
- `/hardware_bridge/<joint>/temperature`, `voltage`, `current`, `load`,
  `moving`: readouts the plugin cannot carry, see `talos_servo.ros.readouts`
- `/hardware_bridge/stop_all` (std_srvs/Trigger)
- `/hardware_bridge/<hinge>/calibrate/{start,open,fold,record_flat,
  record_fold,status,save,cancel}` (std_srvs/Trigger): the bend calibration
  (#84) behind `talos-servo calibrate-bend`, see
  `talos_servo.bend_calibrator`. While it runs, controller goals for the
  hinge are ignored and its angle reads NaN. A save applies the new hinge
  at once. With no call for `IDLE_TIMEOUT_S` the session is cancelled
- `/diagnostics` (diagnostic_msgs/DiagnosticArray): one entry per joint named
  `<node name>: <joint>`, the same as the robot node's. Run as node `robot`
  (`--ros-args -r __node:=robot`, as launch/robot.launch.py does) and the web
  server reads it like the robot node: status, `/robot/stop_all` and the
  `/robot/<joint>/*` readouts. Per-joint commands from `/debug` are not
  served: the controllers own the joints

The hinge (`model.hinge_joint`, `bend`) is the opening angle alpha in rad, pi
flat, converted by `talos_servo.hinge.Hinge` from the flat position in
`config/bend_calibration.yaml`, or `model.bend_flat_steps` until the hinge
is calibrated. While the servo is offline, or reads outside the
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
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from talos_servo.bend_calibrator import FOLD, OPEN, BendCalibrator
from talos_servo.bus import ServoBus, ServoError
from talos_servo.config import SPEED, load_config
from talos_servo.defaults import DEFAULT_MOVE_SPEED, PORT_ENV_VAR, QUEUE_DEPTH
from talos_servo.hinge import READ_MARGIN_STEPS, Hinge
from talos_servo.robot import SOURCE_ROS, CommandError, Robot
from talos_servo.ros.convert import (
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.diagnostics import DIAGNOSTICS_TOPIC, joint_diagnostics
from talos_servo.ros.names import (
    CALIBRATE_CANCEL,
    CALIBRATE_FOLD,
    CALIBRATE_OPEN,
    CALIBRATE_RECORD_FLAT,
    CALIBRATE_RECORD_FOLD,
    CALIBRATE_SAVE,
    CALIBRATE_START,
    CALIBRATE_STATUS,
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


class HardwareBridgeNode(Node):
    def __init__(
        self,
        robot: Robot,
        *,
        calibrator: BendCalibrator | None = None,
        node_name: str = NODE_NAME,
        **node_kwargs,
    ) -> None:
        super().__init__(node_name, **node_kwargs)
        self.robot = robot
        config = robot.config
        status_hz = float(
            self.declare_parameter("status_hz", DEFAULT_STATUS_HZ).value
        )
        self._controls = {name: j.control for name, j in robot.joints.items()}
        # mecanum_drive_controller limits the body velocity from the base
        # limits, so the per-joint max_speed cap does not apply to these.
        self._uncapped = (
            frozenset(config.base.wheels)
            if config.base is not None
            else frozenset()
        )
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
        self.status_pub = self.create_publisher(
            DiagnosticArray, DIAGNOSTICS_TOPIC, QUEUE_DEPTH
        )
        self.create_subscription(
            JointState, COMMANDS_TOPIC, self.on_commands, QUEUE_DEPTH
        )
        self.create_service(Trigger, "~/stop_all", self.on_stop_all)
        self.calibrator = calibrator if self._hinge is not None else None
        # Calibration service step -> action, empty without a calibrator.
        self.calibration_steps: dict[str, Callable[[], str]] = {}
        if self.calibrator is not None:
            self._serve_calibration(self.calibrator)
        self.create_timer(1.0 / config.control_hz, self.tick)
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
        calibrator = self.calibrator
        if calibrator is not None and calibrator.expired(self.robot.clock()):
            self.get_logger().warning(
                "bend calibration got no call for a while, cancelling it"
            )
            self._safely(
                "calibration", lambda: self._calibrate(calibrator.cancel)
            )
        try:
            self.robot.tick()
        except Exception as exc:  # keep ticking: the watchdog lives in tick()
            self.get_logger().error(
                f"tick failed: {exc}", throttle_duration_sec=1.0
            )
        self._report_fault()
        self.publish_states()

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
        snapshot = self.robot.snapshot()
        stamp = self.get_clock().now().to_msg()
        self.readouts.publish_snapshot(snapshot, stamp)
        self.status_pub.publish(
            joint_diagnostics(self.get_name(), snapshot, stamp)
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
                    capped=joint not in self._uncapped,
                ),
            )
        for joint, position in positions.items():
            if self._calibrating(joint):
                continue
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
        if self._calibrating(hinge.joint):
            # The hinge is being measured: no angle to trust yet.
            return math.nan, math.nan
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
        margin = READ_MARGIN_STEPS
        if not low - margin <= steps <= high + margin:
            return f"reads step {steps}, outside its range {low}..{high}"
        return None

    # --- calibration -----------------------------------------------------
    def _serve_calibration(self, calibrator: BendCalibrator) -> None:
        self.calibration_steps = {
            CALIBRATE_START: calibrator.start,
            CALIBRATE_OPEN: lambda: calibrator.jog(OPEN),
            CALIBRATE_FOLD: lambda: calibrator.jog(FOLD),
            CALIBRATE_RECORD_FLAT: calibrator.record_flat,
            CALIBRATE_RECORD_FOLD: calibrator.record_fold,
            CALIBRATE_STATUS: calibrator.status,
            CALIBRATE_SAVE: calibrator.save,
            CALIBRATE_CANCEL: calibrator.cancel,
        }
        for step, action in self.calibration_steps.items():

            def serve(
                _request: Trigger.Request,
                response: Trigger.Response,
                step: str = step,
                action: Callable[[], str] = action,
            ) -> Trigger.Response:
                return self.on_calibrate(step, action, response)

            self.create_service(
                Trigger, f"~/{calibrator.joint}/calibrate/{step}", serve
            )

    def on_calibrate(
        self,
        step: str,
        action: Callable[[], str],
        response: Trigger.Response,
    ) -> Trigger.Response:
        try:
            response.message = self._calibrate(action)
            response.success = True
        except (CommandError, ServoError, ValueError) as exc:
            # ConfigError and CalibrationError are ValueErrors too.
            response.success, response.message = False, str(exc)
            self.get_logger().warning(f"calibrate {step}: {exc}")
        return response

    def _calibrate(self, action: Callable[[], str]) -> str:
        """Run one calibration step. When the session starts or ends the
        hinge waits for a new controller goal, so the last one (from
        before, or in the old calibration) never moves it."""
        calibrator = self.calibrator
        assert calibrator is not None
        was_active = calibrator.active
        message = action()
        if calibrator.active != was_active:
            self._disarm(calibrator.joint)
            self._hinge = self.robot.config.hinge
            self.get_logger().info(
                f"{calibrator.joint} calibration "
                f"{'started' if calibrator.active else 'ended'}: {message}"
            )
        return message

    def _calibrating(self, joint: str) -> bool:
        calibrator = self.calibrator
        return (
            calibrator is not None
            and calibrator.active
            and joint == calibrator.joint
        )

    def _disarm(self, joint: str) -> None:
        self._armed.discard(joint)
        self._first_goals.pop(joint, None)
        self._goals.pop(joint, None)

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
        config = load_config(args.config, args.calibration)
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"{prog}: {exc}", file=sys.stderr)
        return 2
    robot = Robot(ServoBus(args.port, args.baud), config)

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    try:
        robot.open()
        calibrator = (
            BendCalibrator(
                robot,
                config_path=args.config,
                calibration=args.calibration,
                source=SOURCE_ROS,
            )
            if config.hinge is not None and config.hinge.joint in robot.joints
            else None
        )
        node = HardwareBridgeNode(robot, calibrator=calibrator)
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
