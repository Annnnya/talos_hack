"""ROS 2 node that owns the robot mode: free, aligning, clamping or climbing.

- `/robot/mode` (std_msgs/String, transient local): the current mode
- `/robot/mode_reason` (std_msgs/String, transient local): why it last
  changed, or the last fault reported while the mode was kept
- `/robot/wall_nearby` (std_srvs/Trigger): free -> aligning
- `/robot/release` (std_srvs/Trigger): any mode -> free

On every mode change the node deactivates the old mode's ros2_control
controllers and activates the new ones through
`/controller_manager/switch_controller`. Controller names come from `modes:`
in the config. While controller_manager is not running (the controllers do
not exist yet) the switch is skipped with a warning and the mode still
changes. A switch that controller_manager rejects keeps the old mode.

Run with `pixi run -e ros mode-node`.
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from collections.abc import Sequence

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from talos_servo.config import ModesConfig, load_config
from talos_servo.defaults import DEFAULT_CONFIG, ROBOT_NS
from talos_servo.modes import (
    Event,
    InvalidTransition,
    Mode,
    ModeMachine,
    Transition,
)

try:
    from controller_manager_msgs.srv import SwitchController
except ImportError:  # not in every environment yet; switching is skipped
    SwitchController = None

MODE_NODE_NAME = "mode"
MODE_TOPIC = f"{ROBOT_NS}/mode"
MODE_REASON_TOPIC = f"{ROBOT_NS}/mode_reason"
WALL_NEARBY_SERVICE = f"{ROBOT_NS}/wall_nearby"
RELEASE_SERVICE = f"{ROBOT_NS}/release"
DEFAULT_CONTROLLER_MANAGER = "/controller_manager"
SWITCH_POLL_INTERVAL = 0.05
# Late subscribers (a UI that starts after us) still get the current mode.
LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class SwitchError(RuntimeError):
    """controller_manager refused or did not answer a controller switch."""


class ControllerSwitcher:
    """Calls controller_manager's switch_controller service."""

    def __init__(
        self,
        node: Node,
        controller_manager: str,
        timeout_s: float,
        callback_group: ReentrantCallbackGroup,
    ) -> None:
        self._node = node
        self._timeout_s = timeout_s
        self._client = None
        if SwitchController is not None:
            self._client = node.create_client(
                SwitchController,
                f"{controller_manager}/switch_controller",
                callback_group=callback_group,
            )

    @property
    def ready(self) -> bool:
        """True once controller_manager's service is reachable."""
        return self._client is not None and self._client.service_is_ready()

    def switch(
        self, deactivate: Sequence[str], activate: Sequence[str]
    ) -> None:
        """Swap controllers or raise SwitchError. Skipped (with a warning)
        when there is nothing to swap or no controller_manager to ask."""
        if not deactivate and not activate:
            return
        logger = self._node.get_logger()
        if self._client is None:
            logger.warning(
                "controller_manager_msgs is not installed, not switching "
                "controllers"
            )
            return
        if not self._client.service_is_ready():
            logger.warning(
                "controller_manager is not running, not switching "
                f"controllers (deactivate {list(deactivate)}, activate "
                f"{list(activate)})"
            )
            return
        request = SwitchController.Request()
        request.deactivate_controllers = list(deactivate)
        request.activate_controllers = list(activate)
        # Strict: a controller that fails to switch fails the whole mode
        # change, so the robot never runs half the controllers of a mode.
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(seconds=self._timeout_s).to_msg()
        response = self._call(request)
        if not response.ok:
            message = getattr(response, "message", "") or "rejected"
            raise SwitchError(f"controller switch failed: {message}")

    def _call(self, request):
        """Wait for the answer; the executor's other threads deliver it."""
        assert self._client is not None
        future = self._client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + self._timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                self._client.remove_pending_request(future)
                raise SwitchError(
                    "controller_manager did not answer within "
                    f"{self._timeout_s} s"
                )
            done.wait(SWITCH_POLL_INTERVAL)
        response = future.result()
        if response is None:
            raise SwitchError("controller_manager sent no response")
        return response


class ModeNode(Node):
    def __init__(
        self,
        modes: ModesConfig | None = None,
        *,
        node_name: str = MODE_NODE_NAME,
        **node_kwargs,
    ) -> None:
        super().__init__(node_name, **node_kwargs)
        self.modes = modes or ModesConfig(controllers={})
        self.machine = ModeMachine()
        # One change at a time: a second request waits for the first switch.
        self._lock = threading.Lock()
        controller_manager = str(
            self.declare_parameter(
                "controller_manager", DEFAULT_CONTROLLER_MANAGER
            ).value
        )
        group = ReentrantCallbackGroup()
        self.mode_pub = self.create_publisher(String, MODE_TOPIC, LATCHED)
        self.reason_pub = self.create_publisher(
            String, MODE_REASON_TOPIC, LATCHED
        )
        self.create_service(
            Trigger,
            WALL_NEARBY_SERVICE,
            self.on_wall_nearby,
            callback_group=group,
        )
        self.create_service(
            Trigger, RELEASE_SERVICE, self.on_release, callback_group=group
        )
        self.switcher = ControllerSwitcher(
            self, controller_manager, self.modes.switch_timeout_s, group
        )
        self.publish_mode()
        self.get_logger().info(f"mode {self.machine.mode.value}")

    @property
    def mode(self) -> Mode:
        return self.machine.mode

    def request(self, event: Event, reason: str = "") -> Transition:
        """Change mode on `event`, switching controllers first.

        Raises InvalidTransition or SwitchError and keeps the old mode then.
        Later nodes (alignment, clamping, climbing) call this through their
        own topics or services.
        """
        with self._lock:
            old = self.machine.mode
            new = self.machine.peek(event)
            old_set = self.modes.for_mode(old.value)
            new_set = self.modes.for_mode(new.value)
            self.switcher.switch(
                [c for c in old_set if c not in new_set],
                [c for c in new_set if c not in old_set],
            )
            transition = self.machine.handle(event, reason)
            self.publish_mode()
        self.get_logger().info(
            f"mode {old.value} -> {new.value}: {transition.reason}"
        )
        return transition

    def report(self, reason: str) -> None:
        """Publish a reason (e.g. a fault) without changing the mode."""
        with self._lock:
            self.machine.set_reason(reason)
            self.publish_mode()

    def publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.machine.mode.value))
        self.reason_pub.publish(String(data=self.machine.reason))

    def on_wall_nearby(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        return self._trigger(Event.WALL_NEARBY, "wall nearby", response)

    def on_release(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        # Goes straight to free for now; the landing sequence is #86.
        return self._trigger(Event.RELEASE, "released", response)

    def _trigger(
        self, event: Event, reason: str, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            transition = self.request(event, reason)
        except (InvalidTransition, SwitchError) as exc:
            self.get_logger().warning(f"{event.value}: {exc}")
            response.success, response.message = False, str(exc)
        else:
            response.success = True
            response.message = f"mode {transition.mode.value}"
        return response


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        prog="talos-mode-ros", description=__doc__.splitlines()[0]
    )
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG, help="robot config (%(default)s)"
    )
    # Everything else (e.g. --ros-args -p controller_manager:=/cm) goes to
    # rclpy.
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)
    logging.basicConfig(level=logging.INFO)
    try:
        config = load_config(args.config)
    except ValueError as exc:  # ConfigError is a ValueError
        print(f"talos-mode-ros: {exc}", file=sys.stderr)
        return 2

    rclpy.init(args=[sys.argv[0], *ros_args])
    node = None
    # Service callbacks wait on controller_manager, so they need a second
    # thread to receive its answer.
    executor = MultiThreadedExecutor()
    try:
        node = ModeNode(config.modes)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
