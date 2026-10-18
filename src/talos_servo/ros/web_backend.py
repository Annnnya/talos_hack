"""Robot backend for the web server and the CLI that goes through the running
ROS robot node.

With it the robot node is the only process on the serial port, so the browser
UI, the CLI and other ROS nodes can drive the robot at the same time. Command
checks come from `talos_servo.robot`, so the UI behaves the same in both modes.

Differences from driving the bus directly:

- Speeds this backend commanded are re-sent every tick to keep the robot's
  command watchdog fed. If this process dies, the robot stops the wheels by
  itself.
- The robot node has no acceleration input, so the `acceleration` argument is
  ignored.
- If another ROS node commands the same joint, the last command wins. The
  robot's diagnostics report who sent it, and the UI shows it.
- `drive` publishes on `cmd_vel_topic`. The web server passes its twist_mux
  input, so it shares the base with the keyboard instead of fighting it.
- `stop_all` also turns STOP on in the mode node (`/robot/set_stop`), which
  locks twist_mux against every input until `release_stop`.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistStamped
from rclpy.client import Client
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import DEFAULT_ACCELERATION, ServoError
from talos_servo.config import SPEED, RobotConfig
from talos_servo.defaults import (
    ROBOT_NODE_NAME,
    ROBOT_NS,
    STOP_LOCK_TIMEOUT_S,
)
from talos_servo.modes import RobotStatus
from talos_servo.robot import (
    SOURCE_WEB,
    CommandError,
    Joint,
    check_move,
    check_spin,
    finite_clamp,
    public_state,
    record_command,
)
from talos_servo.ros.convert import (
    command_frame,
    rad_speed_to_steps,
    rad_to_steps,
    step_speed_to_rad,
    steps_to_rad,
)
from talos_servo.ros.robot_node import BASE_FRAME, CMD_VEL_TOPIC
from talos_servo.ros.web_client_node import WebClientNode

log = logging.getLogger(__name__)

# How long open() waits for the robot node's services and first status message.
DEFAULT_WAIT_TIMEOUT = 5.0
SERVICE_TIMEOUT = 2.0
SERVICE_POLL_INTERVAL = 0.01
# The robot node publishes status at 2 Hz by default; several missed messages
# mean it is gone.
STATUS_STALE_AFTER = 2.0
# Diagnostics entries from the robot node are named "<node name>: <joint>".
STATUS_PREFIX = f"{ROBOT_NODE_NAME}: "
HINT = "start it with `pixi run -e ros robot-node`"
MODE_HINT = "start it with `pixi run -e ros mode-node`"
# Status lags our own commands. A command the robot reports with a frame this
# backend sends, and no newer than this after our last send, is our own echo
# (maybe of an older command), so it must not move a slider back. Commands from
# other senders are always applied.
COMMAND_SETTLE_TIME = 1.0
EXECUTOR_JOIN_TIMEOUT = 2.0


class RosBackend:
    """`RobotBackend` on top of the robot node's topics and services.

    Pass `context` to share an already initialised rclpy context (tests);
    otherwise the backend initialises and shuts down its own.
    """

    def __init__(
        self,
        config: RobotConfig,
        *,
        context: Context | None = None,
        wait_timeout: float = DEFAULT_WAIT_TIMEOUT,
        clock: Callable[[], float] = time.monotonic,
        cmd_vel_topic: str = CMD_VEL_TOPIC,
    ) -> None:
        self.config = config
        self.cmd_vel_topic = cmd_vel_topic
        self.wait_timeout = wait_timeout
        self._mode: str | None = None
        self._mode_reason: str | None = None
        self._active_input: str | None = None
        self.joints = {
            c.name: Joint.from_config(c) for c in config.enabled_joints
        }
        self._clock = clock
        self._lock = threading.Lock()
        self._context = context
        self._owns_context = context is None
        self._node: WebClientNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._thread: threading.Thread | None = None
        self._first_status = threading.Event()
        self._last_status: float | None = None
        # Speeds this backend commanded and keeps re-sending for the robot's
        # watchdog, with the frame_id they went out with. Never filled from
        # status: a wheel started by a CLI that has died must still time out.
        self._feeding: dict[str, tuple[int, str]] = {}
        self._sent_at: dict[str, float] = {}
        self._own_frames: set[str] = set()
        # Last STOP state from the mode node and when it came.
        self._stop_beat: tuple[bool, float] | None = None

    # --- lifecycle -------------------------------------------------------
    def open(self) -> None:
        if self._owns_context:
            self._context = Context()
            rclpy.init(context=self._context)
        try:
            self._node = WebClientNode(
                self.joints,
                self._on_joint_states,
                self._on_status,
                on_stop_lock=self._on_stop_lock,
                on_mode=self._on_mode,
                on_mode_reason=self._on_mode_reason,
                on_active_input=self._on_active_input,
                cmd_vel_topic=self.cmd_vel_topic,
                context=self._context,
            )
            self._executor = MultiThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(
                target=self._executor.spin,
                name="ros-robot-backend",
                daemon=True,
            )
            self._thread.start()
            if not self._node.stop_all_client.wait_for_service(
                timeout_sec=self.wait_timeout
            ):
                raise ServoError(f"no robot node at {ROBOT_NS} ({HINT})")
            if not self._first_status.wait(self.wait_timeout):
                raise ServoError(
                    f"robot node at {ROBOT_NS} publishes no status ({HINT})"
                )
        except BaseException:
            self.release()
            raise
        log.info("connected to the robot node at %s", ROBOT_NS)

    def close(self) -> None:
        """Stop what this backend commanded and disconnect.

        Not `stop_all`: closing one interface must not stop motion another one
        is commanding.
        """
        try:
            if self._node is not None:
                self.stop_own()
        finally:
            self.release()

    def release(self) -> None:
        """Disconnect without `stop_all`, for callers that stop only the joints
        they started.

        A wheel left spinning still stops within the robot's command timeout.
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
            now = self._clock()
            return [public_state(j, now) for j in self.joints.values()]

    # --- telemetry -------------------------------------------------------
    def tick(self) -> None:
        """Feed the robot's watchdog for the joints we spin, and notice a silent
        robot node."""
        with self._lock:
            feeding = dict(self._feeding)
            if self._last_status is not None:
                silent = self._clock() - self._last_status
                if silent > STATUS_STALE_AFTER:
                    for j in self.joints.values():
                        j.online = False
                        j.error = (
                            f"no status from the robot node for {silent:.1f} s"
                        )
        by_frame: dict[str, dict[str, int]] = {}
        for joint, (speed, frame) in feeding.items():
            by_frame.setdefault(frame, {})[joint] = speed
        for frame, speeds in by_frame.items():
            self._publish_velocity(speeds, frame)

    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                j = self.joints.get(name)
                if j is None:
                    continue
                if i < len(msg.position):
                    j.position = rad_to_steps(msg.position[i])
                if i < len(msg.velocity):
                    j.speed = rad_speed_to_steps(msg.velocity[i])

    def _on_status(self, msg: DiagnosticArray) -> None:
        now = self._clock()
        ours = [s for s in msg.status if s.name.startswith(STATUS_PREFIX)]
        if not ours:  # /diagnostics from some other node
            return
        with self._lock:
            for status in ours:
                j = self.joints.get(status.name.removeprefix(STATUS_PREFIX))
                if j is None:
                    continue
                try:
                    self._apply_status(j, status, now)
                except (KeyError, ValueError) as exc:
                    log.warning("bad status for %s: %s", status.name, exc)
            self._last_status = now
        self._first_status.set()

    def _on_stop_lock(self, msg: Bool) -> None:
        with self._lock:
            self._stop_beat = (bool(msg.data), self._clock())

    def stopped(self) -> bool:
        """True while twist_mux's STOP lock is on.

        That is the mode node saying so, or no word from it within the lock
        timeout (no mode node), which twist_mux also treats as STOP on.
        """
        with self._lock:
            beat, now = self._stop_beat, self._clock()
        if beat is None:
            return True
        on, heard_at = beat
        return on or now - heard_at > STOP_LOCK_TIMEOUT_S

    def robot_status(self) -> RobotStatus:
        """Mode, mode reason and active twist_mux input; None until heard."""
        with self._lock:
            return RobotStatus(
                self._mode, self._mode_reason, self._active_input
            )

    def has_cmd_vel_listener(self) -> bool:
        """True if something (twist_mux, the robot node) reads our
        cmd_vel topic."""
        return self._require_node().cmd_vel_pub.get_subscription_count() > 0

    def _on_mode(self, msg: String) -> None:
        with self._lock:
            self._mode = msg.data

    def _on_mode_reason(self, msg: String) -> None:
        with self._lock:
            self._mode_reason = msg.data

    def _on_active_input(self, msg: String) -> None:
        with self._lock:
            self._active_input = msg.data

    def _apply_status(
        self, j: Joint, status: DiagnosticStatus, now: float
    ) -> None:
        values = {kv.key: kv.value for kv in status.values}
        j.online = status.level != DiagnosticStatus.ERROR
        j.error = (
            None if status.level == DiagnosticStatus.OK else status.message
        )
        j.torque = values["torque"] == "True"
        j.moving = values["moving"] == "True"
        j.load, j.temp = int(values["load"]), int(values["temp"])
        j.voltage, j.current = (
            float(values["voltage"]),
            float(values["current"]),
        )
        self._apply_last_command(j, values, now)

    def _apply_last_command(
        self, j: Joint, values: dict[str, str], now: float
    ) -> None:
        """Adopt the robot's last command for this joint unless it is our own
        stale echo."""
        source = values.get("command_source") or None
        if source is None:  # no command for this joint yet
            return
        reported_at = now - float(values["command_age"])
        client = values["command_client"] or None
        sent_at = self._sent_at.get(j.name)
        own = command_frame(source, client) in self._own_frames
        if (
            own
            and sent_at is not None
            and reported_at <= sent_at + COMMAND_SETTLE_TIME
        ):
            return
        target = values["target"]
        j.target = int(target) if target else None
        j.target_speed = int(values["target_speed"])
        record_command(j, source, client, reported_at)
        # Someone else has the joint now. Their own commands keep it alive, not
        # ours.
        self._feeding.pop(j.name, None)

    # --- commands --------------------------------------------------------
    def set_torque(self, joint: str, enabled: bool) -> None:
        with self._lock:
            j = self._get(joint)
            if not enabled:
                j.target_speed = (
                    0  # the robot stops a speed joint before releasing torque
                )
                self._feeding.pop(joint, None)
        self._call(
            self._require_node().torque_clients[joint],
            SetBool.Request(data=enabled),
            f"{joint}: set torque",
        )
        with self._lock:
            j.torque = enabled

    def move(
        self,
        joint: str,
        position: int,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        with self._lock:
            j = self._get(joint)
            target, speed = check_move(
                j, position, speed, self.config.max_speed
            )
            j.target, j.torque = target, True
            self._record(j, source, client)
        msg = JointState(
            name=[joint],
            position=[steps_to_rad(target)],
            velocity=[step_speed_to_rad(speed)],
        )
        msg.header.frame_id = command_frame(source, client)
        self._require_node().position_pub.publish(msg)

    def spin(
        self,
        joint: str,
        speed: int,
        acceleration: int = DEFAULT_ACCELERATION,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        frame = command_frame(source, client)
        with self._lock:
            j = self._get(joint)
            speed = check_spin(j, speed, self.config.max_speed)
            j.target_speed = speed
            if speed:
                j.torque = True
                self._feeding[joint] = (speed, frame)
            else:
                self._feeding.pop(joint, None)
            self._record(j, source, client)
        self._publish_velocity({joint: speed}, frame)

    def wall_nearby(self) -> None:
        """Ask the mode node to start aligning (free -> aligning)."""
        self._call_mode_node(
            self._require_node().wall_nearby_client, "wall nearby"
        )

    def release_walls(self) -> None:
        """Ask the mode node to release the walls and go back to free."""
        self._call_mode_node(self._require_node().release_client, "release")

    def _call_mode_node(self, client: Client, what: str) -> None:
        # The mode node is optional, so do not wait the full service timeout
        # for one that is not there.
        if not client.service_is_ready():
            raise CommandError(
                f"{what}: mode node is not running ({MODE_HINT})"
            )
        self._call(client, Trigger.Request(), what)

    def drive(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        *,
        source: str = SOURCE_WEB,
        client: str | None = None,
    ) -> None:
        """Publish one twist (m/s, m/s, rad/s) on `cmd_vel_topic`, clamped
        to the base limits.

        The robot stops unless this is repeated, so call it in a loop. A twist
        carries no sender, so `source` and `client` are not sent.
        """
        base = self.config.base
        if base is None:
            raise CommandError(
                "the config has no base section, so the robot cannot drive"
            )
        self._publish_twist(
            finite_clamp(linear_x, base.max_linear_speed),
            finite_clamp(linear_y, base.max_linear_speed),
            finite_clamp(angular_z, base.max_angular_speed),
        )

    def stop(
        self, joint: str, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        with self._lock:
            j = self._get(joint)
            if j.control != SPEED:
                return
            j.target_speed = 0
            self._feeding.pop(joint, None)
            self._record(j, source, client)
        self._publish_velocity({joint: 0}, command_frame(source, client))

    def stop_all(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Turn STOP on in the mode node, then stop every speed joint.

        STOP locks twist_mux, so a browser or keyboard still sending cannot
        drive the base again. Best effort, never raises.
        """
        try:
            self.set_stop(True)
        except (CommandError, ServoError) as exc:
            # Without the mode node twist_mux is locked anyway: its stop
            # lock went quiet.
            log.warning("STOP through the mode node failed: %s", exc)
        with self._lock:
            self._feeding.clear()
            wheels = [j for j in self.joints.values() if j.control == SPEED]
            for j in wheels:
                j.target_speed = 0
                self._record(j, source, client)
        try:
            if wheels:
                speeds = dict.fromkeys((j.name for j in wheels), 0)
                self._publish_velocity(speeds, command_frame(source, client))
            self._call(
                self._require_node().stop_all_client,
                Trigger.Request(),
                "stop all",
            )
        except (CommandError, ServoError) as exc:
            # No more velocity commands go out, so the robot's watchdog still
            # stops the wheels.
            log.warning("stop all through the robot node failed: %s", exc)

    def set_stop(self, stopped: bool) -> None:
        """Turn STOP on (True) or release it (False) in the mode node."""
        client = self._require_node().set_stop_client
        # The mode node is optional, so do not wait the full service
        # timeout for one that is not there.
        if not client.service_is_ready():
            raise CommandError(f"STOP: mode node is not running ({MODE_HINT})")
        what = "STOP" if stopped else "release STOP"
        self._call(client, SetBool.Request(data=stopped), what)

    def release_stop(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Release STOP, so twist_mux passes the inputs on again."""
        self.set_stop(False)

    def stop_own(
        self, *, source: str = SOURCE_WEB, client: str | None = None
    ) -> None:
        """Stop only what this backend commanded: one zero twist on its drive
        topic and zero speed for the joints it spins. Never raises.

        Unlike `stop_all` it leaves the robot node's `stop_all` alone, so
        another interface (the keyboard) keeps driving.
        """
        with self._lock:
            fed = list(self._feeding)
            self._feeding.clear()
            for name in fed:
                j = self.joints[name]
                j.target_speed = 0
                self._record(j, source, client)
        try:
            self._publish_twist(0.0, 0.0, 0.0)
            if fed:
                self._publish_velocity(
                    dict.fromkeys(fed, 0), command_frame(source, client)
                )
        except (CommandError, ServoError) as exc:
            # Nothing is re-sent any more, so the mux and the robot's watchdog
            # still stop the motion.
            log.warning("stopping our own commands failed: %s", exc)

    # --- helpers ---------------------------------------------------------
    def _publish_twist(
        self, linear_x: float, linear_y: float, angular_z: float
    ) -> None:
        node = self._require_node()
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        twist = msg.twist
        twist.linear.x, twist.linear.y, twist.angular.z = (
            linear_x,
            linear_y,
            angular_z,
        )
        node.cmd_vel_pub.publish(msg)

    def _record(self, j: Joint, source: str, client: str | None) -> None:
        """Note a command we are about to send.

        Call with `_lock` held.
        """
        now = self._clock()
        record_command(j, source, client, now)
        self._sent_at[j.name] = now
        self._own_frames.add(command_frame(source, client))

    def _publish_velocity(self, speeds: dict[str, int], frame: str) -> None:
        msg = JointState(
            name=list(speeds),
            velocity=[step_speed_to_rad(v) for v in speeds.values()],
        )
        msg.header.frame_id = frame
        self._require_node().velocity_pub.publish(msg)

    def _call(self, client: Client, request: Any, what: str) -> None:
        """Call a robot service from a worker thread; the executor thread
        delivers the answer.

        Never hold `_lock` here: the subscription callbacks need it while we
        wait.
        """
        future = client.call_async(request)
        deadline = self._clock() + SERVICE_TIMEOUT
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        while not future.done():
            if self._clock() > deadline:
                client.remove_pending_request(future)
                raise ServoError(
                    f"{what}: robot node did not answer within "
                    f"{SERVICE_TIMEOUT} s"
                )
            done.wait(SERVICE_POLL_INTERVAL)
        response = future.result()
        if response is None or not response.success:
            message = (
                response.message if response is not None else "no response"
            )
            raise CommandError(f"{what}: {message}")

    def _require_node(self) -> WebClientNode:
        if self._node is None:
            raise ServoError(
                "not connected to the robot node, call open() first"
            )
        return self._node

    def _get(self, joint: str) -> Joint:
        try:
            return self.joints[joint]
        except KeyError:
            raise CommandError(
                f"unknown joint {joint!r}, expected one of "
                f"{sorted(self.joints)}"
            ) from None
