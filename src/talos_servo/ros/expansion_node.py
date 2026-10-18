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

Entering clamping opens the hinge by itself (#78, `talos_servo.clamp`)
until the hinge or a wheel current (the `effort` of `/joint_states`) shows
wall contact; then it calls `/robot/wall_contact` and the mode node moves on
to climbing. If the hinge opens fully first it calls `/robot/no_wall` and
holds. STOP pauses the clamp and releasing STOP goes on. A rate on
`/expansion/rate` while clamping cancels the clamp: the operator takes over
until clamping is entered again.

Level (#82, `talos_servo.level`): with `/level/enable` on, the platform roll
and pitch from `/imu/data` trim the wheel pairs while on the wall, also at
rest, so a clamped robot levels itself. It stops with STOP, outside the wall
modes and on a tilt fault. `/level/state` reports it as JSON.

Run with `pixi run -e ros expansion-node`.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from collections.abc import Callable, Sequence

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32, Float64, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from talos_servo.clamp import Clamp, Outcome
from talos_servo.climb import wheel_speeds as climb_wheel_speeds
from talos_servo.config import ConfigError, RobotConfig, load_config
from talos_servo.defaults import (
    DEFAULT_CONFIG,
    QUEUE_DEPTH,
    ROBOT_NS,
    STOP_LOCK_TIMEOUT_S,
)
from talos_servo.expansion import STILL, Expansion, Wheels
from talos_servo.hinge_hold import HingeHold
from talos_servo.level import NO_TRIM, Leveler, Trim
from talos_servo.modes import (
    CLIMB_MODES,
    HINGE_MODES,
    HOLD_MODES,
    SWAP_MODES,
    Mode,
)
from talos_servo.orientation import platform_tilt
from talos_servo.ros.imu_convert import COVARIANCE_NOT_PROVIDED
from talos_servo.ros.names import (
    CLIMB_TOPIC,
    EXPANSION_RATE_TOPIC,
    HINGE_COMMANDS_TOPIC,
    HINGE_MOTION_SERVICE,
    HOLD_ENABLE_SERVICE,
    HOLD_SETPOINT_TOPIC,
    HOLD_STATE_TOPIC,
    IMU_DATA_TOPIC,
    LATCHED,
    LEVEL_ENABLE_SERVICE,
    LEVEL_STATE_TOPIC,
    MODE_TOPIC,
    NO_WALL_SERVICE,
    STOP_LOCK_TOPIC,
    WALL_CONTACT_SERVICE,
    WALL_SIDE_TOPIC,
    WHEEL_COMMANDS_TOPIC,
)

EXPANSION_NODE_NAME = "expansion"
JOINT_STATES_TOPIC = "/joint_states"
# Wait this long before asking the mode node again after a refusal (hinge
# motion, wall contact, no wall).
GRANT_RETRY_S = 1.0
LOG_THROTTLE_S = 2.0
# Publish the hold state every this many ticks (10 Hz at 50 Hz).
HOLD_STATE_EVERY = 5
# Raw servo load counts per percent, and when a load reading is too old.
LOAD_PER_PCT = 10.0
LOAD_STALE_S = 1.0


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
        self.wheels = tuple(config.base.wheels)
        # Free mode needs its controllers swapped only if the config names
        # some to swap in.
        self.needs_grant = bool(config.modes and config.modes.hinge_motion)
        self.period_s = 1.0 / self.settings.rate_hz
        self._now = clock
        self._rate = 0.0
        self._rate_stamp: float | None = None
        self._climb = 0.0
        self._climb_stamp: float | None = None
        self._side = 0.0
        self._side_stamp: float | None = None
        self.max_climb = self.settings.max_climb_speed_m_s
        self.wheel_radius_m = config.base.wheel_radius_m
        self._alpha = math.nan
        # Motor currents (A) by joint name, from the joint state effort.
        self._currents: dict[str, float] = {}
        # Set while clamping (#78) until a manual rate cancels it.
        self._clamp: Clamp | None = None
        self._clamp_told = False
        self._clamp_pending: Future | None = None
        self._clamp_retry_at = 0.0
        self._current = math.nan
        self._mode: Mode | None = None
        self._stopped = True
        self._stop_stamp: float | None = None
        # Free mode: the mode node swapped in the wheel controller.
        self._granted = False
        self._pending: Future | None = None
        self._retry_at = 0.0
        self._moving = False
        self._held: str | None = None
        # Hinge current hold (#142); None without a calibrated hinge.
        self.hold = (
            HingeHold(
                config.hinge_hold, config.hinge, self.settings.max_rate_rad_s
            )
            if config.hinge is not None
            else None
        )
        self._hold_note: str | None = None
        self._ticks = 0
        # Level loop (#82): platform roll and pitch (rad) from the IMU with
        # the mount taken off, and when they came.
        self.leveler = Leveler(config.level)
        self.imu_rpy = config.imu.rpy
        self._level_active = False
        self._level_note: str | None = None
        self._tilt: tuple[float, float] | None = None
        self._tilt_at: float | None = None
        self._trim: Trim = NO_TRIM

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
            Float64, CLIMB_TOPIC, self.on_climb, QUEUE_DEPTH
        )
        self.create_subscription(
            Float64, WALL_SIDE_TOPIC, self.on_side, QUEUE_DEPTH
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
        self.clamp_clients = {
            Outcome.CONTACT: self.create_client(Trigger, WALL_CONTACT_SERVICE),
            Outcome.NO_WALL: self.create_client(Trigger, NO_WALL_SERVICE),
        }
        self.create_subscription(
            Float64, HOLD_SETPOINT_TOPIC, self.on_hold_setpoint, LATCHED
        )
        self.create_service(SetBool, HOLD_ENABLE_SERVICE, self.on_hold_enable)
        # Hinge load from the bridge readouts (raw, 0.1 % each), for the
        # hold's load limit.
        self._load_pct: float | None = None
        self._load_at: float | None = None
        self.create_subscription(
            Float32,
            f"{ROBOT_NS}/{self.hinge}/load",
            self.on_hinge_load,
            QUEUE_DEPTH,
        )
        self.hold_state_pub = self.create_publisher(
            String, HOLD_STATE_TOPIC, QUEUE_DEPTH
        )
        self.create_subscription(
            Imu, IMU_DATA_TOPIC, self.on_imu, qos_profile_sensor_data
        )
        self.create_service(SetBool, LEVEL_ENABLE_SERVICE, self.on_level_enable)
        self.level_state_pub = self.create_publisher(
            String, LEVEL_STATE_TOPIC, QUEUE_DEPTH
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

    @property
    def clamp(self) -> Clamp | None:
        return self._clamp

    @property
    def currents(self) -> dict[str, float]:
        return self._currents

    def on_rate(self, msg: Float64) -> None:
        self._rate = self.expansion.clamp_rate(float(msg.data))
        self._rate_stamp = self._now()
        if self._rate != 0.0 and self._clamp is not None:
            self._clamp = None
            self.expansion.stop()
            self.get_logger().info("clamp cancelled: manual hinge motion")
        if self._rate and self.hold is not None and self.hold.active:
            # A manual move and the hold would fight over the hinge goal.
            self._stop_hold("manual hinge move")

    # --- hinge current hold (#142) ---------------------------------------
    def on_hold_setpoint(self, msg: Float64) -> None:
        if self.hold is None:
            return
        try:
            self.hold.set_setpoint(float(msg.data))
        except ValueError as exc:
            self.get_logger().warning(f"hold setpoint: {exc}")

    def on_hold_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        error = None
        if self.hold is None:
            error = "no calibrated hinge in the config"
        elif not request.data:
            self._stop_hold("turned off")
        elif self._mode not in HOLD_MODES:
            error = "press wall nearby first"
        elif not math.isfinite(self._alpha):
            error = "the hinge angle is unknown"
        else:
            if self._clamp is not None:
                self._clamp = None
                self.expansion.stop()
                self.get_logger().info("clamp cancelled: hinge hold on")
            self.hold.start(self._alpha)
            self._hold_note = None
            self.get_logger().info(
                f"hinge hold on at {self.hold.setpoint_a:.2f} A"
            )
        response.success = error is None
        response.message = error or ("hold on" if request.data else "hold off")
        return response

    def on_hinge_load(self, msg: Float32) -> None:
        self._load_pct = abs(float(msg.data)) / LOAD_PER_PCT
        self._load_at = self._now()

    def hinge_load_pct(self) -> float | None:
        """Last hinge load in %, None once it is stale."""
        if self._load_at is None or self._now() - self._load_at > LOAD_STALE_S:
            return None
        return self._load_pct

    def _stop_hold(self, why: str) -> None:
        if self.hold is not None and self.hold.active:
            self.hold.stop()
            self.get_logger().info(f"hinge hold off: {why}")
        self._hold_note = why

    def hold_tick(self) -> None:
        """Run the hold loop: through STOP too, since letting go of the walls
        drops the robot. Only the toggle, a manual move or leaving the wall
        modes ends it."""
        hold = self.hold
        if hold is None:
            return
        if hold.active and self._mode not in HOLD_MODES:
            self._stop_hold(f"mode {self._mode.value if self._mode else '?'}")
        load = self.hinge_load_pct()
        step = hold.step(self._alpha, self._current, self.period_s, load)
        if step.stopped:
            self._stop_hold(step.stopped)
            self.get_logger().warning(f"hinge hold: {step.stopped}")
        if step.goal is not None:
            self.hinge_pub.publish(Float64MultiArray(data=[step.goal]))
        self._ticks += 1
        if self._ticks % HOLD_STATE_EVERY == 0:
            self.hold_state_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "active": hold.active,
                            "setpoint_a": hold.setpoint_a,
                            "measured_a": round(step.measured_a, 4),
                            "offset_rad": round(step.offset_rad, 4),
                            "max_current_a": hold.settings.max_current_a,
                            "note": self._hold_note,
                            "load_pct": load,
                            "load_limited": step.load_limited,
                        }
                    )
                )
            )

    # --- level loop (#82) --------------------------------------------------
    def on_imu(self, msg: Imu) -> None:
        # The raw IMU topic has no orientation; only the filtered one counts.
        if msg.orientation_covariance[0] == COVARIANCE_NOT_PROVIDED:
            return
        q = msg.orientation
        self._tilt = platform_tilt((q.x, q.y, q.z, q.w), self.imu_rpy)
        self._tilt_at = self._now()

    def tilt(self) -> tuple[float, float] | None:
        """Platform roll and pitch (rad), None while the IMU is quiet."""
        if self._tilt_at is None or self._tilt is None:
            return None
        if self._now() - self._tilt_at > self.leveler.settings.imu_timeout_s:
            return None
        return self._tilt

    @property
    def level_active(self) -> bool:
        return self._level_active

    def on_level_enable(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        error = None
        tilt = self.tilt()
        if not request.data:
            self._stop_level("turned off")
        elif self._mode not in CLIMB_MODES:
            error = "press wall nearby first"
        elif tilt is None:
            error = "no orientation from the IMU"
        elif self.leveler.tilt_fault(*tilt) is not None:
            error = "the platform tilts past the limit"
        else:
            self._level_active = True
            self._level_note = None
            self.get_logger().info("level on")
        response.success = error is None
        response.message = error or (
            "level on" if request.data else "level off"
        )
        return response

    def _stop_level(self, why: str) -> None:
        if self._level_active:
            self._level_active = False
            self.get_logger().info(f"level off: {why}")
        self._level_note = why
        self._trim = NO_TRIM

    def level_tick(self) -> None:
        """Update the trims, stop on a tilt fault and report the state."""
        tilt = self.tilt()
        fault = None
        if tilt is not None:
            fault = self.leveler.tilt_fault(*tilt)
        if self._level_active and self._mode not in CLIMB_MODES:
            self._stop_level(f"mode {self._mode.value if self._mode else '?'}")
        if self._level_active and fault is not None:
            self._stop_level(fault)
            self.get_logger().warning(f"level: {fault}, wheels stopped")
            self._still(fault)
        if self._level_active and tilt is not None:
            self._trim = self.leveler.trim(*tilt)
        else:
            self._trim = NO_TRIM
        if self._ticks % HOLD_STATE_EVERY == 0:
            roll, pitch = tilt if tilt is not None else (math.nan, math.nan)
            self.level_state_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "active": self._level_active,
                            "imu_ok": tilt is not None,
                            "roll_deg": _deg(roll),
                            "pitch_deg": _deg(pitch),
                            "front_rear_m_s": round(self._trim.front_rear, 4),
                            "left_right_m_s": round(self._trim.left_right, 4),
                            "fault": fault,
                            "note": self._level_note,
                        }
                    )
                )
            )

    def on_climb(self, msg: Float64) -> None:
        speed = float(msg.data)
        if not math.isfinite(speed):
            speed = 0.0
        self._climb = max(-self.max_climb, min(self.max_climb, speed))
        self._climb_stamp = self._now()

    def requested_climb(self) -> float:
        """The last climb speed, or 0 once it is older than the timeout."""
        if self._climb_stamp is None:
            return 0.0
        if self._now() - self._climb_stamp > self.settings.command_timeout_s:
            return 0.0
        return self._climb

    def on_side(self, msg: Float64) -> None:
        speed = float(msg.data)
        if not math.isfinite(speed):
            speed = 0.0
        self._side = max(-self.max_climb, min(self.max_climb, speed))
        self._side_stamp = self._now()

    def requested_side(self) -> float:
        """The last sideways speed, or 0 once it is older than the timeout."""
        if self._side_stamp is None:
            return 0.0
        if self._now() - self._side_stamp > self.settings.command_timeout_s:
            return 0.0
        return self._side

    def climb_wheels(
        self, speed: float, side: float = 0.0, trim: Trim = NO_TRIM
    ) -> Wheels:
        """Wheel speeds (rad/s) on the wall, see `talos_servo.climb`."""
        return climb_wheel_speeds(
            speed, side, self.wheel_radius_m, self.max_climb, trim
        )

    def on_joint_state(self, msg: JointState) -> None:
        names = list(msg.name)
        try:
            index = names.index(self.hinge)
            self._alpha = float(msg.position[index])
            if index < len(msg.effort):
                self._current = float(msg.effort[index])
        except (ValueError, IndexError):
            pass
        for joint in (self.hinge, *self.wheels):
            try:
                index = names.index(joint)
                self._currents[joint] = float(msg.effort[index])
            except (ValueError, IndexError):
                # No reading: a stale current must not count as contact.
                self._currents.pop(joint, None)

    def on_mode(self, msg: String) -> None:
        try:
            mode = Mode(msg.data)
        except ValueError:
            mode = None
        if mode != self._mode:
            # The mode node ends any swap on a mode change.
            self._granted = False
            self._clamp = None
            if mode is Mode.CLAMPING:
                self._stop_hold("clamping opens to the walls")
                self._clamp = Clamp(
                    self.expansion, self.settings, self.hinge, self.wheels
                )
                self._clamp_told = False
                self.get_logger().info("clamping: opening to the walls")
        self._mode = mode

    def on_stop_lock(self, msg: Bool) -> None:
        self._stopped = bool(msg.data)
        self._stop_stamp = self._now()

    def blocked(self, modes: frozenset[Mode] = HINGE_MODES) -> str | None:
        """Why the hinge (or with CLIMB_MODES the climb) may not move now, or
        None."""
        now = self._now()
        if self._mode is None:
            return "no robot mode yet"
        if self._mode not in modes:
            return f"cannot move while {self._mode.value}"
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
        if self._clamp is not None:
            # The clamp owns the hinge goal; the hold is off meanwhile.
            self.tick_clamp(self._clamp)
            return
        self.hold_tick()
        self.level_tick()
        rate = self.requested_rate()
        climb = self.requested_climb()
        side = self.requested_side()
        if (climb or side or self._trim) and not rate:
            self.climb_tick(climb, side)
            return
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

    def tick_clamp(self, clamp: Clamp) -> None:
        """Open to the walls, or hold once the clamp is over."""
        why = self.blocked()
        if why is not None:
            clamp.pause()
            self.halt(why)
            return
        if clamp.done:
            self._still(clamp.reason)
            self._tell(clamp)
            return
        step = clamp.step(
            self._alpha, self._currents, self._now(), self.period_s
        )
        if step.goal is not None:
            self.hinge_pub.publish(Float64MultiArray(data=[step.goal]))
        if step.wheels != STILL or self._moving:
            self._send_wheels(step.wheels)
        self._moving = step.wheels != STILL
        self._note(step.held)
        if clamp.done:
            self.get_logger().info(f"clamp over: {clamp.reason}")
            self._tell(clamp)

    def _tell(self, clamp: Clamp) -> None:
        """Tell the mode node how the clamp ended, once."""
        if self._clamp_told or self._clamp_pending is not None:
            return
        if self._now() < self._clamp_retry_at:
            return
        client = self.clamp_clients[clamp.outcome]
        if not client.service_is_ready():
            self.get_logger().warning(
                f"mode node not reachable for {clamp.outcome.value}",
                throttle_duration_sec=LOG_THROTTLE_S,
            )
            return
        future = self.call_clamp_service(clamp.outcome)
        self._clamp_pending = future
        future.add_done_callback(lambda f: self._told(f, clamp))

    def call_clamp_service(self, outcome: Outcome) -> Future:
        return self.clamp_clients[outcome].call_async(Trigger.Request())

    def _told(self, future: Future, clamp: Clamp) -> None:
        self._clamp_pending = None
        if clamp is not self._clamp:
            return
        response = future.result()
        if response is not None and response.success:
            self._clamp_told = True
            return
        message = response.message if response else "no answer"
        self._clamp_retry_at = self._now() + GRANT_RETRY_S
        self.get_logger().warning(
            f"{clamp.outcome.value} refused: {message}",
            throttle_duration_sec=LOG_THROTTLE_S,
        )

    def climb_tick(self, speed: float, side: float = 0.0) -> None:
        """Turn the front and rear pairs opposite ways; the hinge keeps its
        goal so a clamped robot keeps pressing on the walls."""
        why = self.blocked(CLIMB_MODES)
        if why is not None:
            self.halt(why)
            return
        if self._needs_swap() and not self._granted:
            self._ask(True)
            self._still("waiting for the wheel controller")
            return
        self._send_wheels(self.climb_wheels(speed, side, self._trim))
        self._moving = True
        self._note(None)

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
        return self.needs_grant and self._mode in SWAP_MODES

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


def _deg(rad: float) -> float | None:
    return round(math.degrees(rad), 2) if math.isfinite(rad) else None


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
