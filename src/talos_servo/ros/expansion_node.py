"""ROS 2 node that opens and closes the bend hinge with the wheels rolling
along (#76). The math is in `talos_servo.expansion`.

- `/expansion/rate` (std_msgs/Float64, in): opening speed in rad/s, positive
  expands, negative collapses, 0 stops. Repeat it at least every
  `expansion.command_timeout_s` or the motion stops, like twist_mux inputs
- `/joint_states` (in): the measured hinge angle
- `/robot/mode` and `/robot/stop_lock` (in): the hinge only moves in free
  and clamping, and never while STOP is on or its beats are missing
- `/bend_position_controller/commands` and
  `/wheel_velocity_controller/commands` (std_msgs/Float64MultiArray, out):
  the hinge goal in rad and the four wheel speeds in rad/s

In free mode the wheels belong to mecanum_drive_controller, so before moving
the node asks the mode node (`/robot/hinge_motion`) to swap in
wheel_velocity_controller, and gives the wheels back when it stops. In
clamping that controller is always active.

When a motion stops (rate 0, timeout, STOP, mode change, a hinge limit) the
wheels get zeros, since wheel_velocity_controller keeps its last command,
and the hinge keeps its last goal, so a clamped robot keeps holding.

Run with `pixi run -e ros expansion-node`.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from collections.abc import Callable, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool

from talos_servo.config import ConfigError, RobotConfig, load_config
from talos_servo.defaults import (
    DEFAULT_CONFIG,
    QUEUE_DEPTH,
    STOP_LOCK_TIMEOUT_S,
)
from talos_servo.expansion import STILL, Expansion, Wheels
from talos_servo.modes import HINGE_MODES, Mode
from talos_servo.ros.names import (
    EXPANSION_RATE_TOPIC,
    HINGE_COMMANDS_TOPIC,
    HINGE_MOTION_SERVICE,
    LATCHED,
    MODE_TOPIC,
    STOP_LOCK_TOPIC,
    WHEEL_COMMANDS_TOPIC,
)

EXPANSION_NODE_NAME = "expansion"
JOINT_STATES_TOPIC = "/joint_states"
# Wait this long before asking the mode node again after a refusal.
GRANT_RETRY_S = 1.0
LOG_THROTTLE_S = 2.0


class ExpansionNode(Node):
    def __init__(
        self,
        config: RobotConfig,
        *,
        node_name: str = EXPANSION_NODE_NAME,
        clock: Callable[[], float] = time.monotonic,
        **node_kwargs,
    ) -> None:
        if config.model is None or config.base is None:
            raise ConfigError("the expansion node needs model and base")
        super().__init__(node_name, **node_kwargs)
        self.settings = config.expansion
        self.expansion = Expansion(config.model, config.base, self.settings)
        self.hinge = config.model.hinge_joint
        # Free mode needs its controllers swapped only if the config names
        # some to swap in.
        self.needs_grant = bool(config.modes and config.modes.hinge_motion)
        self.period_s = 1.0 / self.settings.rate_hz
        self._now = clock
        self._rate = 0.0
        self._rate_stamp: float | None = None
        self._alpha = math.nan
        self._mode: Mode | None = None
        self._stopped = True
        self._stop_stamp: float | None = None
        # Free mode: the mode node swapped in the wheel controller.
        self._granted = False
        self._pending: Future | None = None
        self._retry_at = 0.0
        self._moving = False
        self._held: str | None = None

        self.hinge_pub = self.create_publisher(
            Float64MultiArray, HINGE_COMMANDS_TOPIC, QUEUE_DEPTH
        )
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, WHEEL_COMMANDS_TOPIC, QUEUE_DEPTH
        )
        self.create_subscription(
            Float64, EXPANSION_RATE_TOPIC, self.on_rate, QUEUE_DEPTH
        )
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, self.on_joint_state, QUEUE_DEPTH
        )
        self.create_subscription(String, MODE_TOPIC, self.on_mode, LATCHED)
        self.create_subscription(
            Bool, STOP_LOCK_TOPIC, self.on_stop_lock, QUEUE_DEPTH
        )
        self.hinge_motion_client = self.create_client(
            SetBool, HINGE_MOTION_SERVICE
        )
        self.create_timer(self.period_s, self.tick)

    @property
    def alpha(self) -> float:
        return self._alpha

    @property
    def mode(self) -> Mode | None:
        return self._mode

    @property
    def granted(self) -> bool:
        return self._granted

    def on_rate(self, msg: Float64) -> None:
        self._rate = self.expansion.clamp_rate(float(msg.data))
        self._rate_stamp = self._now()

    def on_joint_state(self, msg: JointState) -> None:
        try:
            index = list(msg.name).index(self.hinge)
            self._alpha = float(msg.position[index])
        except (ValueError, IndexError):
            pass

    def on_mode(self, msg: String) -> None:
        try:
            mode = Mode(msg.data)
        except ValueError:
            mode = None
        if mode != self._mode:
            # The mode node ends any swap on a mode change.
            self._granted = False
        self._mode = mode

    def on_stop_lock(self, msg: Bool) -> None:
        self._stopped = bool(msg.data)
        self._stop_stamp = self._now()

    def blocked(self) -> str | None:
        """Why the hinge may not move now, or None."""
        now = self._now()
        if self._mode is None:
            return "no robot mode yet"
        if self._mode not in HINGE_MODES:
            return f"the hinge cannot move while {self._mode.value}"
        if (
            self._stop_stamp is None
            or now - self._stop_stamp > STOP_LOCK_TIMEOUT_S
        ):
            return "no STOP state from the mode node"
        if self._stopped:
            return "STOP is on"
        return None

    def requested_rate(self) -> float:
        """The last rate, or 0 once it is older than the timeout."""
        if self._rate_stamp is None:
            return 0.0
        if self._now() - self._rate_stamp > self.settings.command_timeout_s:
            return 0.0
        return self._rate

    def tick(self) -> None:
        rate = self.requested_rate()
        why = self.blocked() if rate else None
        if rate == 0.0 or why is not None:
            self.halt(why)
            return
        if self._needs_swap() and not self._granted:
            self._ask(True)
            self._still("waiting for the wheel controller")
            return
        step = self.expansion.step(rate, self._alpha, self.period_s)
        if step.goal is not None:
            self.hinge_pub.publish(Float64MultiArray(data=[step.goal]))
        self._send_wheels(step.wheels)
        self._moving = step.wheels != STILL
        self._note(step.held)

    def halt(self, why: str | None = None) -> None:
        """Stop the wheels, keep the hinge goal and give the wheels back."""
        self._still(why)
        if self._granted:
            self._ask(False)

    def _still(self, why: str | None) -> None:
        self.expansion.stop()
        if self._moving:
            self._send_wheels(STILL)
            self._moving = False
        self._note(why)

    def _needs_swap(self) -> bool:
        return self.needs_grant and self._mode is Mode.FREE

    def _send_wheels(self, wheels: Wheels) -> None:
        self.wheel_pub.publish(Float64MultiArray(data=list(wheels)))

    def _note(self, why: str | None) -> None:
        if why and why != self._held:
            self.get_logger().info(f"hinge held: {why}")
        self._held = why

    def _ask(self, wanted: bool) -> None:
        """Ask the mode node to start (True) or end (False) the swap."""
        if self._pending is not None:
            return
        if wanted and self._now() < self._retry_at:
            return
        if not self.hinge_motion_client.service_is_ready():
            if wanted:
                self.get_logger().warning(
                    "mode node not reachable for hinge motion",
                    throttle_duration_sec=LOG_THROTTLE_S,
                )
            else:
                self._granted = False
            return
        if not wanted:
            # Count it as given back now: the mode node clears it anyway on
            # STOP and mode changes, and the next motion asks again.
            self._granted = False
        future = self.call_hinge_motion(wanted)
        self._pending = future
        future.add_done_callback(lambda f: self._answered(f, wanted))

    def call_hinge_motion(self, wanted: bool) -> Future:
        return self.hinge_motion_client.call_async(SetBool.Request(data=wanted))

    def _answered(self, future: Future, wanted: bool) -> None:
        self._pending = None
        response = future.result()
        ok = response is not None and response.success
        if wanted:
            self._granted = ok
            if not ok:
                self._retry_at = self._now() + GRANT_RETRY_S
                message = response.message if response else "no answer"
                self.get_logger().warning(
                    f"hinge motion refused: {message}",
                    throttle_duration_sec=LOG_THROTTLE_S,
                )

    def shutdown(self) -> None:
        """Zero the wheels on exit. The hinge keeps its goal."""
        self.expansion.stop()
        self._send_wheels(STILL)
        self._moving = False
        if self._granted and self.hinge_motion_client.service_is_ready():
            self.call_hinge_motion(False)
            self._granted = False


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        prog="talos-expansion-ros", description=__doc__.splitlines()[0]
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG, help="robot config (%(default)s)"
    )
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(args.config)
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"talos-expansion-ros: {exc}", file=sys.stderr)
        return 2

    # rclpy's own handler would shut the context down before `finally`, and
    # then the zero wheel command could not go out. The launch stops nodes
    # with SIGINT, then SIGTERM.
    rclpy.init(
        args=[sys.argv[0], *ros_args],
        signal_handler_options=SignalHandlerOptions.NO,
    )
    signal.signal(signal.SIGTERM, _interrupt)
    node = None
    try:
        node = ExpansionNode(config)
        rclpy.spin(node)
    except ConfigError as exc:
        print(f"talos-expansion-ros: {exc}", file=sys.stderr)
        return 2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


def _interrupt(_signum: int, _frame: object) -> None:
    raise KeyboardInterrupt


if __name__ == "__main__":
    sys.exit(main())
