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

import json
import logging
import math
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
from sensor_msgs.msg import CompressedImage, Imu, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    ServoError,
    joint_position,
)
from talos_servo.config import SPEED, RobotConfig
from talos_servo.defaults import (
    ROBOT_NODE_NAME,
    ROBOT_NS,
    STOP_LOCK_TIMEOUT_S,
)
from talos_servo.modes import RobotStatus
from talos_servo.orientation import platform_tilt
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
from talos_servo.ros.imu_convert import COVARIANCE_NOT_PROVIDED
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
HINT = "start it with `pixi run -e ros robot`"
MODE_HINT = "start it with `pixi run -e ros mode-node`"
# Status lags our own commands. A command the robot reports with a frame this
# backend sends, and no newer than this after our last send, is our own echo
# (maybe of an older command), so it must not move a slider back. Commands from
# other senders are always applied.
COMMAND_SETTLE_TIME = 1.0
EXECUTOR_JOIN_TIMEOUT = 2.0
# The Madgwick filter publishes at the IMU rate (100 Hz); this much silence
# means the IMU chain is gone and the tilt is unknown.
TILT_STALE_AFTER = 0.5
# The camera node streams at several frames a second; this much silence means
# the stream is off or the node is gone.
CAMERA_STALE_AFTER = 2.0
JPEG_FORMAT = "jpeg"


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
        # Trigger clients made on first use, by service name.
        self._triggers: dict[str, Client] = {}
        # Hinge opening angle (rad) from /joint_states, None while unknown.
        self._hinge_alpha: float | None = None
        self._hinge_current: float | None = None
        # Last /hinge_hold/state from the expansion node, and when it came.
        self._hold: dict[str, Any] | None = None
        self._hold_at: float | None = None
        # Platform roll and pitch (rad) from the IMU, and when they came.
        self._tilt: tuple[float, float] | None = None
        self._tilt_at: float | None = None
        # Latest JPEG from the camera node, and when it came.
        self._frame: bytes | None = None
        self._frame_at: float | None = None

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
                on_hold_state=self._on_hold_state,
                on_imu=self._on_imu,
                on_camera=self._on_camera,
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
        self._triggers.clear()

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
        # The bridge sends NaN while a reading is unknown (the hinge offline or
        # out of range): keep the last value instead of crashing the executor.
        hinge = self.config.hinge
        with self._lock:
            for i, name in enumerate(msg.name):
                if hinge is not None and name == hinge.joint:
                    alpha = msg.position[i] if i < len(msg.position) else None
                    finite = alpha is not None and math.isfinite(alpha)
                    self._hinge_alpha = alpha if finite else None
                    if i < len(msg.effort) and math.isfinite(msg.effort[i]):
                        self._hinge_current = msg.effort[i]
                j = self.joints.get(name)
                if j is None:
                    continue
                if i < len(msg.position) and math.isfinite(msg.position[i]):
                    j.position = rad_to_steps(msg.position[i])
                if i < len(msg.velocity) and math.isfinite(msg.velocity[i]):
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
        j.fault = values.get("fault") or None
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

    def move_hinge(self, rate: float) -> None:
        """Publish one hinge opening speed (rad/s, + opens, 0 stops) for the
        expansion node, clamped to `expansion.max_rate_rad_s`.

        The hinge stops unless this is repeated, so call it in a loop.
        """
        limit = self.config.expansion.max_rate_rad_s
        self._require_node().expansion_pub.publish(
            Float64(data=finite_clamp(rate, limit))
        )

    def climb(self, speed: float) -> None:
        """Publish one climb speed (m/s, + up, 0 stops) for the expansion
        node, clamped to `expansion.max_climb_speed_m_s`. Stops unless
        repeated."""
        limit = self.config.expansion.max_climb_speed_m_s
        self._require_node().climb_pub.publish(
            Float64(data=finite_clamp(speed, limit))
        )

    def wall_side(self, speed: float) -> None:
        """Publish one sideways speed on the wall (m/s, + left, 0 stops),
        clamped to the climb cap. Stops unless repeated."""
        limit = self.config.expansion.max_climb_speed_m_s
        self._require_node().side_pub.publish(
            Float64(data=finite_clamp(speed, limit))
        )

    def hinge_state(self) -> dict[str, Any] | None:
        """Hinge readout for calibrating `model.bend_flat_steps`, or None
        without a hinge. `angle_deg` and `raw_steps` are None while the
        angle is unknown; `raw_steps` is the servo position as
        `talos-servo monitor` shows it."""
        hinge, model = self.config.hinge, self.config.model
        if hinge is None or model is None:
            return None
        joint = next(j for j in self.config.joints if j.name == hinge.joint)
        with self._lock:
            alpha = self._hinge_alpha
        raw = None
        if alpha is not None:
            raw = joint_position(hinge.steps_at(alpha), joint.inverted)
        return {
            "joint": hinge.joint,
            "angle_deg": None if alpha is None else math.degrees(alpha),
            "raw_steps": raw,
            "flat_steps": model.bend_flat_steps,
            "min_deg": math.degrees(hinge.min_rad),
            "max_deg": math.degrees(hinge.max_rad),
            "max_rate": self.config.expansion.max_rate_rad_s,
            "current_a": self._hinge_current,
            "wheel_currents": self._wheel_currents(),
        }

    def _wheel_currents(self) -> dict[str, float] | None:
        base = self.config.base
        if base is None:
            return None
        with self._lock:
            return {
                w: self.joints[w].current
                for w in base.wheels
                if w in self.joints and self.joints[w].online
            }

    def _on_hold_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._hold, self._hold_at = state, self._clock()

    def hold_state(self) -> dict[str, Any] | None:
        """Hinge current hold state from the expansion node, None when it
        has gone quiet."""
        with self._lock:
            if self._hold_at is None:
                return None
            if self._clock() - self._hold_at > STATUS_STALE_AFTER:
                return None
            return dict(self._hold)

    def _on_imu(self, msg: Imu) -> None:
        # The raw IMU topic has no orientation; only the filtered one counts.
        if msg.orientation_covariance[0] == COVARIANCE_NOT_PROVIDED:
            return
        q = msg.orientation
        tilt = platform_tilt((q.x, q.y, q.z, q.w), self.config.imu.rpy)
        with self._lock:
            self._tilt, self._tilt_at = tilt, self._clock()

    def tilt(self) -> dict[str, float] | None:
        """Platform roll and pitch (deg) from the IMU, None while the IMU
        chain is quiet."""
        with self._lock:
            if self._tilt_at is None or self._tilt is None:
                return None
            if self._clock() - self._tilt_at > TILT_STALE_AFTER:
                return None
            roll, pitch = self._tilt
        return {
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
        }

    def _on_camera(self, msg: CompressedImage) -> None:
        # The page shows the frame as is, so only JPEG works.
        if JPEG_FORMAT not in msg.format.lower():
            return
        frame = bytes(msg.data)
        with self._lock:
            self._frame, self._frame_at = frame, self._clock()

    def camera_frame(self) -> bytes | None:
        """Latest JPEG from the camera node, None while it is quiet."""
        with self._lock:
            if self._frame_at is None:
                return None
            if self._clock() - self._frame_at > CAMERA_STALE_AFTER:
                return None
            return self._frame

    def hinge_hold(self, enabled: bool, setpoint_a: float) -> None:
        """Set the hold setpoint (A), then turn the hold on or off."""
        node = self._require_node()
        node.hold_setpoint_pub.publish(Float64(data=float(setpoint_a)))
        client = node.hold_enable_client
        if not client.service_is_ready():
            raise CommandError("hinge hold: the expansion node is not running")
        self._call(client, SetBool.Request(data=enabled), "hinge hold")

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

    def trigger(self, service: str) -> str:
        """Call a std_srvs/Trigger service by full name; its message."""
        client = self._triggers.get(service)
        if client is None:
            client = self._require_node().create_client(Trigger, service)
            self._triggers[service] = client
        if not client.wait_for_service(timeout_sec=self.wait_timeout):
            raise CommandError(f"no service {service}")
        return self._call(client, Trigger.Request(), service)

    def _call(self, client: Client, request: Any, what: str) -> str:
        """Call a robot service from a worker thread; the executor thread
        delivers the answer, and its message is returned.

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
        return response.message

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
