"""Web server backend that drives the robot through the ROS servo bridge.

With it the bridge node is the only process on the serial port, so the browser UI and other ROS
nodes can drive the robot at the same time. It mirrors `RobotController`'s command rules (mode
checks, clamping) so the UI behaves the same in both modes.

Differences from driving the bus directly:

- Wheel speeds are re-sent every telemetry tick to keep the bridge's `velocity_cmd` watchdog fed.
  If this process dies, the bridge stops the wheels by itself.
- The bridge has no acceleration input, so the `acceleration` argument is ignored.
- If another ROS node commands the same joint, the last command wins.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from dataclasses import asdict
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.client import Client
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import DEFAULT_ACCELERATION, ServoError
from talos_servo.config import WebConfig
from talos_servo.controller import (
    MODES,
    POSITION,
    WHEEL,
    CommandError,
    ServoState,
    _clamp,
)
from talos_servo.defaults import DEFAULT_BRIDGE
from talos_servo.ros.convert import (
    joint_ids,
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.web_client_node import WebClientNode

log = logging.getLogger(__name__)

# How long open() waits for the bridge's services and first status message.
DEFAULT_WAIT_TIMEOUT = 5.0
SERVICE_TIMEOUT = 2.0
SERVICE_POLL_INTERVAL = 0.01
# The bridge publishes status at 2 Hz by default; several missed messages mean it is gone.
STATUS_STALE_AFTER = 2.0
# A status message sent just before a mode change can arrive just after it. Trust our own
# change for this long instead of flipping the UI back.
MODE_SETTLE_TIME = 1.0
EXECUTOR_JOIN_TIMEOUT = 2.0


class RosBackend:
    """`RobotBackend` on top of the servo bridge's topics and services.

    Pass `context` to share an already initialised rclpy context (tests); otherwise the backend
    initialises and shuts down its own.
    """

    def __init__(
        self,
        config: WebConfig,
        *,
        bridge: str = DEFAULT_BRIDGE,
        context: Context | None = None,
        wait_timeout: float = DEFAULT_WAIT_TIMEOUT,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self.bridge = bridge
        self.wait_timeout = wait_timeout
        self.joints = joint_ids(config.servos)
        self.names = {servo_id: name for name, servo_id in self.joints.items()}
        self.servos = {cfg.id: ServoState.from_config(cfg) for cfg in config.servos}
        self._clock = clock
        self._lock = threading.Lock()
        self._context = context
        self._owns_context = context is None
        self._node: WebClientNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._first_status = threading.Event()
        self._last_status: float | None = None
        self._mode_set_at: dict[int, float] = {}

    # --- lifecycle -------------------------------------------------------
    def open(self) -> None:
        if self._owns_context:
            self._context = Context()
            rclpy.init(context=self._context)
        try:
            self._node = WebClientNode(
                self.bridge,
                self.joints,
                self._on_joint_states,
                self._on_status,
                context=self._context,
            )
            self._executor = MultiThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._executor.spin, name="ros-web-backend", daemon=True
            )
            self._thread.start()
            hint = "start it with `pixi run -e ros servo-node`"
            if not self._node.stop_all_client.wait_for_service(timeout_sec=self.wait_timeout):
                raise ServoError(f"no servo bridge at {self.bridge} ({hint})")
            if not self._first_status.wait(self.wait_timeout):
                raise ServoError(f"servo bridge at {self.bridge} publishes no status ({hint})")
        except BaseException:
            self.release()
            raise
        log.info("connected to servo bridge at %s", self.bridge)

    def close(self) -> None:
        try:
            if self._node is not None:
                self.stop_all()
        finally:
            self.release()

    def release(self) -> None:
        """Disconnect without `stop_all`, for callers that stop only the servos they started.

        A wheel left spinning still stops within the bridge's command timeout.
        """
        if self._executor is not None:
            self._executor.shutdown()
        if self._thread is not None:
            self._thread.join(EXECUTOR_JOIN_TIMEOUT)
        if self._node is not None:
            self._node.destroy_node()
        if self._owns_context and self._context is not None:
            rclpy.try_shutdown(context=self._context)
        self._node = self._executor = self._thread = None

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return [asdict(s) for s in self.servos.values()]

    # --- telemetry -------------------------------------------------------
    def poll(self) -> None:
        """Feed the bridge watchdog for every spinning wheel and notice a silent bridge."""
        with self._lock:
            spinning = {s.id: s.target_speed for s in self.servos.values() if s.target_speed}
            if self._last_status is not None:
                silent = self._clock() - self._last_status
                if silent > STATUS_STALE_AFTER:
                    for s in self.servos.values():
                        s.online = False
                        s.error = f"no status from servo bridge for {silent:.1f} s"
        if spinning:
            self._publish_velocity(spinning)

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                s = self._state_for(name)
                if s is None:
                    continue
                if i < len(msg.position):
                    s.position = rad_to_steps(msg.position[i])
                if i < len(msg.velocity):
                    s.speed = rad_speed_to_steps(msg.velocity[i])

    def _on_status(self, msg: DiagnosticArray) -> None:
        now = self._clock()
        with self._lock:
            for status in msg.status:
                # The bridge names each entry "<node name>: <joint>".
                s = self._state_for(status.name.rsplit(": ", 1)[-1])
                if s is None:
                    continue
                try:
                    self._apply_status(s, status, now)
                except (KeyError, ValueError) as exc:
                    log.warning("bad status for %s: %s", status.name, exc)
            self._last_status = now
        self._first_status.set()

    def _apply_status(self, s: ServoState, status: DiagnosticStatus, now: float) -> None:
        values = {kv.key: kv.value for kv in status.values}
        s.online = status.level != DiagnosticStatus.ERROR
        s.error = None if status.level == DiagnosticStatus.OK else status.message
        s.torque = values["torque"] == "True"
        s.moving = values["moving"] == "True"
        s.load, s.temp = int(values["load"]), int(values["temp"])
        s.voltage, s.current = float(values["voltage"]), float(values["current"])
        mode = values["mode"]
        settling = now - self._mode_set_at.get(s.id, -MODE_SETTLE_TIME) < MODE_SETTLE_TIME
        if mode in MODES and mode != s.mode and not settling:
            # Changed by someone else; drop our targets like RobotController does.
            s.mode, s.target, s.target_speed = mode, None, 0

    # --- commands --------------------------------------------------------
    def set_mode(self, servo_id: int, mode: str) -> None:
        if mode not in MODES:
            raise CommandError(f"unknown mode {mode!r}, expected one of {MODES}")
        with self._lock:
            s = self._get(servo_id)
            s.target_speed = 0  # stop feeding the watchdog before the bridge switches modes
        joint = self.names[servo_id]
        self._call(
            self._require_node().wheel_mode_clients[joint],
            SetBool.Request(data=mode == WHEEL),
            f"{joint}: set mode",
        )
        with self._lock:
            s.mode = mode
            s.target = s.position if mode == POSITION else None
            self._mode_set_at[servo_id] = self._clock()

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        with self._lock:
            s = self._get(servo_id)
            if not enabled:
                s.target_speed = 0  # the bridge stops a wheel before releasing torque
        joint = self.names[servo_id]
        self._call(
            self._require_node().torque_clients[joint],
            SetBool.Request(data=enabled),
            f"{joint}: set torque",
        )
        with self._lock:
            s.torque = enabled

    def move(
        self, servo_id: int, position: int, speed: int, acceleration: int = DEFAULT_ACCELERATION
    ) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode != POSITION:
                raise CommandError(f"servo {servo_id} is in speed mode, switch to position first")
            target = _clamp(position, s.min_pos, s.max_pos)
            speed = _clamp(speed, 1, self.config.max_speed)
            s.target, s.torque = target, True
        self._require_node().position_pub.publish(
            JointState(
                name=[self.names[servo_id]],
                position=[steps_to_rad(target)],
                velocity=[step_speed_to_rad(speed)],
            )
        )

    def spin(self, servo_id: int, speed: int, acceleration: int = DEFAULT_ACCELERATION) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode != WHEEL:
                raise CommandError(f"servo {servo_id} is in position mode, switch to speed first")
            speed = _clamp(speed, -self.config.max_speed, self.config.max_speed)
            s.target_speed = speed
            if speed:
                s.torque = True
        self._publish_velocity({servo_id: speed})

    def stop(self, servo_id: int) -> None:
        with self._lock:
            s = self._get(servo_id)
            if s.mode != WHEEL:
                return
            s.target_speed = 0
        self._publish_velocity({servo_id: 0})

    def stop_all(self) -> None:
        with self._lock:
            wheels = [s.id for s in self.servos.values() if s.mode == WHEEL]
            for servo_id in wheels:
                self.servos[servo_id].target_speed = 0
        try:
            if wheels:
                self._publish_velocity(dict.fromkeys(wheels, 0))
            self._call(self._require_node().stop_all_client, Trigger.Request(), "stop all")
        except (CommandError, ServoError) as exc:
            # No more velocity commands go out, so the bridge watchdog still stops the wheels.
            log.warning("stop all through the bridge failed: %s", exc)

    # --- helpers ---------------------------------------------------------
    def _publish_velocity(self, speeds: dict[int, int]) -> None:
        self._require_node().velocity_pub.publish(
            JointState(
                name=[self.names[i] for i in speeds],
                velocity=[step_speed_to_rad(v) for v in speeds.values()],
            )
        )

    def _call(self, client: Client, request: Any, what: str) -> None:
        """Call a bridge service from a worker thread; the executor thread delivers the answer.

        Never hold `_lock` here: the subscription callbacks need it while we wait.
        """
        future = client.call_async(request)
        deadline = self._clock() + SERVICE_TIMEOUT
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        while not future.done():
            if self._clock() > deadline:
                client.remove_pending_request(future)
                raise ServoError(f"{what}: servo bridge did not answer within {SERVICE_TIMEOUT} s")
            done.wait(SERVICE_POLL_INTERVAL)
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "no response"
            raise CommandError(f"{what}: {message}")

    def _require_node(self) -> WebClientNode:
        if self._node is None:
            raise ServoError("not connected to the servo bridge, call open() first")
        return self._node

    def _state_for(self, joint: str) -> ServoState | None:
        servo_id = self.joints.get(joint)
        return None if servo_id is None else self.servos[servo_id]

    def _get(self, servo_id: int) -> ServoState:
        try:
            return self.servos[servo_id]
        except KeyError:
            raise CommandError(f"unknown servo id {servo_id}") from None
