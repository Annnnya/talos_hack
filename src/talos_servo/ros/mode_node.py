"""ROS 2 node that owns the robot mode: free, aligning, clamping or climbing.

- `/robot/mode` (std_msgs/String, transient local): the current mode
- `/robot/mode_reason` (std_msgs/String, transient local): why it last
  changed, or the last fault reported while the mode was kept
- `/robot/wall_nearby` (std_srvs/Trigger): free -> aligning
- `/robot/release` (std_srvs/Trigger): any mode -> free
- `/robot/set_stop` (std_srvs/SetBool): true turns STOP on, false releases
  it. STOP starts on, so a restarted mode node never releases it
- `/robot/hinge_motion` (std_srvs/SetBool): the expansion node (#76) sets it
  while the hinge opens or closes. In free mode the `modes.hinge_motion`
  controllers then replace the free ones. Refused outside free and
  clamping and while STOP is on. A mode change or STOP clears it
- `/robot/stop_lock` (std_msgs/Bool): the STOP state, repeated every
  `STOP_LOCK_PERIOD_S` for twist_mux. Its lock has a timeout, so a quiet
  lock (this node is down) also stops the base, and a restarted twist_mux
  is locked again by the next beat (#103)

On every mode change the node asks controller_manager which controllers are
active (`list_controllers`) and switches (`switch_controller`) so that
exactly the new mode's controllers are active. Controller names come from
`modes:` in the config; controllers no mode names (e.g. the joint state
broadcaster) are never touched. A switch that controller_manager rejects
keeps the old mode. A switch that times out but was applied anyway is
accepted. Every `RECONCILE_PERIOD_S` the node also brings the active
controllers back to the current mode's set, so a controller_manager that
starts late, restarts or applies a timed out switch later is corrected.

While controller_manager is not running (the controllers do not exist yet)
the mode still changes with a warning, unless the
`require_controller_manager` parameter is true: then the mode change fails.

Run with `pixi run -e ros mode-node`.
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from collections.abc import Iterable, Sequence

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from talos_servo.config import ModesConfig, load_config
from talos_servo.defaults import (
    DEFAULT_CONFIG,
    QUEUE_DEPTH,
    STOP_LOCK_PERIOD_S,
)
from talos_servo.modes import (
    HINGE_MODES,
    Event,
    InvalidTransition,
    Mode,
    ModeMachine,
    Transition,
)
from talos_servo.ros.names import (
    HINGE_MOTION_SERVICE,
    LATCHED,
    MODE_REASON_TOPIC,
    MODE_TOPIC,
    RELEASE_SERVICE,
    SET_STOP_SERVICE,
    STOP_LOCK_TOPIC,
    WALL_NEARBY_SERVICE,
)

try:
    from controller_manager_msgs.srv import ListControllers, SwitchController
except ImportError:  # not in every environment yet; switching is skipped
    ListControllers = SwitchController = None

MODE_NODE_NAME = "mode"
DEFAULT_CONTROLLER_MANAGER = "/controller_manager"
SWITCH_POLL_INTERVAL = 0.05
RECONCILE_PERIOD_S = 1.0
ACTIVE = "active"


class SwitchError(RuntimeError):
    """controller_manager refused or did not answer a controller switch."""


class SwitchTimeout(SwitchError):
    """controller_manager did not answer in time."""


class ControllerSwitcher:
    """Makes controller_manager run exactly a given set of controllers."""

    def __init__(
        self,
        node: Node,
        controller_manager: str,
        timeout_s: float,
        callback_group: ReentrantCallbackGroup,
        *,
        required: bool = False,
    ) -> None:
        self._node = node
        self._timeout_s = timeout_s
        self._required = required
        self._switch_client = None
        self._list_client = None
        if SwitchController is not None:
            self._switch_client = node.create_client(
                SwitchController,
                f"{controller_manager}/switch_controller",
                callback_group=callback_group,
            )
            self._list_client = node.create_client(
                ListControllers,
                f"{controller_manager}/list_controllers",
                callback_group=callback_group,
            )

    @property
    def ready(self) -> bool:
        """True once controller_manager's services are reachable."""
        return (
            self._switch_client is not None
            and self._list_client is not None
            and self._switch_client.service_is_ready()
            and self._list_client.service_is_ready()
        )

    def sync(self, target: Iterable[str], managed: Iterable[str]) -> bool:
        """Make the `managed` controllers that are active exactly `target`.

        Diffs against what controller_manager reports as active, so it never
        asks to (de)activate a controller already in that state (STRICT
        rejects that). Returns True if a switch was sent. Raises SwitchError,
        or warns and returns False when there is no controller_manager and
        it is not required.
        """
        if not self.ready:
            reason = (
                "controller_manager_msgs is not installed"
                if self._switch_client is None
                else "controller_manager is not running"
            )
            if self._required:
                raise SwitchError(reason)
            self._node.get_logger().warning(
                f"{reason}, not switching controllers",
                throttle_duration_sec=5.0,
            )
            return False
        want = set(target)
        managed = set(managed)
        active = self.active() & managed
        deactivate = sorted(active - want)
        activate = sorted(want - active)
        if not deactivate and not activate:
            return False
        self._node.get_logger().info(
            f"switching controllers: deactivate {deactivate}, activate "
            f"{activate}"
        )
        request = SwitchController.Request()
        request.deactivate_controllers = deactivate
        request.activate_controllers = activate
        # Strict: a controller that fails to switch fails the whole mode
        # change, so the robot never runs half the controllers of a mode.
        request.strictness = SwitchController.Request.STRICT
        request.timeout = Duration(seconds=self._timeout_s).to_msg()
        try:
            response = self._call(self._switch_client, request)
        except SwitchTimeout:
            # controller_manager may still have applied it.
            if self.active() & managed == want:
                self._node.get_logger().warning(
                    "controller switch answered late but was applied"
                )
                return True
            raise
        if not response.ok:
            message = getattr(response, "message", "") or "rejected"
            raise SwitchError(f"controller switch failed: {message}")
        return True

    def active(self) -> set[str]:
        """Names of the controllers controller_manager runs now."""
        response = self._call(self._list_client, ListControllers.Request())
        return {c.name for c in response.controller if c.state == ACTIVE}

    def _call(self, client, request):
        """Wait for the answer; the executor's other threads deliver it."""
        assert client is not None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + self._timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                client.remove_pending_request(future)
                raise SwitchTimeout(
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
        require_cm = bool(
            self.declare_parameter("require_controller_manager", False).value
        )
        group = ReentrantCallbackGroup()
        # Separate from self._lock, which a controller switch holds: STOP
        # must never wait for a switch.
        self._stop_lock = threading.Lock()
        self._stopped = True
        # Set through /robot/hinge_motion while the expansion node moves.
        self._hinge_motion = False
        # Default volatile QoS, like twist_mux's lock subscription; the
        # beats, not latching, reach a twist_mux that starts later.
        self.stop_lock_pub = self.create_publisher(
            Bool, STOP_LOCK_TOPIC, QUEUE_DEPTH
        )
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
            self,
            controller_manager,
            self.modes.switch_timeout_s,
            group,
            required=require_cm,
        )
        self.create_timer(
            RECONCILE_PERIOD_S, self.reconcile, callback_group=group
        )
        self.create_service(
            SetBool, SET_STOP_SERVICE, self.on_set_stop, callback_group=group
        )
        self.create_service(
            SetBool,
            HINGE_MOTION_SERVICE,
            self.on_hinge_motion,
            callback_group=group,
        )
        self.create_timer(
            STOP_LOCK_PERIOD_S, self.publish_stop_lock, callback_group=group
        )
        self.publish_mode()
        self.publish_stop_lock()
        self.get_logger().info(
            f"mode {self.machine.mode.value}, STOP on until /robot/set_stop "
            "false"
        )

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
            # A mode change ends any hinge motion; the expansion node sees
            # the new mode and asks again if it may.
            self._hinge_motion = False
            self.switcher.sync(self.target(new), self.modes.managed)
            transition = self.machine.handle(event, reason)
            self.publish_mode()
        self.get_logger().info(
            f"mode {old.value} -> {new.value}: {transition.reason}"
        )
        return transition

    @property
    def hinge_motion(self) -> bool:
        return self._hinge_motion

    def target(self, mode: Mode) -> tuple[str, ...]:
        """Controllers that should be active in `mode` right now."""
        if mode is Mode.FREE and self._hinge_motion and self.modes.hinge_motion:
            return self.modes.hinge_motion
        return self.modes.for_mode(mode.value)

    @property
    def stopped(self) -> bool:
        return self._stopped

    def set_stopped(self, stopped: bool) -> None:
        """Turn STOP on or off and tell twist_mux right away."""
        with self._stop_lock:
            changed = stopped != self._stopped
            self._stopped = stopped
            self.stop_lock_pub.publish(Bool(data=stopped))
        if stopped:
            # No switch here (STOP never waits for one): the next reconcile
            # brings the free controllers back.
            self._hinge_motion = False
        if changed:
            self.get_logger().info("STOP on" if stopped else "STOP released")

    def publish_stop_lock(self) -> None:
        with self._stop_lock:
            self.stop_lock_pub.publish(Bool(data=self._stopped))

    def on_set_stop(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        self.set_stopped(bool(request.data))
        response.success = True
        response.message = "stopped" if request.data else "released"
        return response

    def on_hinge_motion(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        wanted = bool(request.data)
        with self._lock:
            mode = self.machine.mode
            error = None
            if wanted and mode not in HINGE_MODES:
                error = f"the hinge cannot move while {mode.value}"
            elif wanted and self._stopped:
                error = "STOP is on"
            if error is None:
                self._hinge_motion = wanted
                try:
                    self.switcher.sync(self.target(mode), self.modes.managed)
                except SwitchError as exc:
                    self._hinge_motion = False
                    error = str(exc)
        if error is not None:
            self.get_logger().warning(f"hinge motion: {error}")
            response.success, response.message = False, error
            return response
        response.success = True
        response.message = "hinge motion on" if wanted else "hinge motion off"
        return response

    def reconcile(self) -> None:
        """Bring the active controllers back to the current mode's set."""
        if not self.switcher.ready:
            return
        # A mode change in progress syncs the controllers itself.
        if not self._lock.acquire(blocking=False):
            return
        try:
            mode = self.machine.mode
            if self.switcher.sync(self.target(mode), self.modes.managed):
                self.get_logger().info(
                    f"controllers reconciled to {mode.value}"
                )
        except SwitchError as exc:
            self.get_logger().warning(
                f"reconcile: {exc}", throttle_duration_sec=5.0
            )
        finally:
            self._lock.release()

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
