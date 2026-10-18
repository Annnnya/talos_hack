"""Topic and service names of the robot's ROS nodes, and their QoS.

Kept apart from the nodes, so a client (the web server, the keyboard) can use
them without importing the node that serves them.
"""

from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from talos_servo.defaults import ROBOT_NS

# Mode node
MODE_TOPIC = f"{ROBOT_NS}/mode"
MODE_REASON_TOPIC = f"{ROBOT_NS}/mode_reason"
WALL_NEARBY_SERVICE = f"{ROBOT_NS}/wall_nearby"
RELEASE_SERVICE = f"{ROBOT_NS}/release"
SET_STOP_SERVICE = f"{ROBOT_NS}/set_stop"
STOP_LOCK_TOPIC = f"{ROBOT_NS}/stop_lock"
HINGE_MOTION_SERVICE = f"{ROBOT_NS}/hinge_motion"
WALL_CONTACT_SERVICE = f"{ROBOT_NS}/wall_contact"
NO_WALL_SERVICE = f"{ROBOT_NS}/no_wall"

# Expansion node (#76) in, and the controllers it feeds
# (config/ros2_control.yaml) out.
EXPANSION_RATE_TOPIC = "/expansion/rate"
# Climb speed in m/s (std_msgs/Float64, + up): front wheels one way, rear
# wheels the other. Repeat it like the rate.
CLIMB_TOPIC = "/expansion/climb"
# Sideways speed on the wall in m/s (std_msgs/Float64, + left), mixed with the
# climb speed. Repeat it like the climb speed.
WALL_SIDE_TOPIC = "/expansion/side"
# Hinge current hold (#142): setpoint in A (std_msgs/Float64, latched), on
# and off (std_srvs/SetBool), and its state as JSON (std_msgs/String).
HOLD_SETPOINT_TOPIC = "/hinge_hold/setpoint"
HOLD_ENABLE_SERVICE = "/hinge_hold/enable"
HOLD_STATE_TOPIC = "/hinge_hold/state"
HINGE_COMMANDS_TOPIC = "/bend_position_controller/commands"
WHEEL_COMMANDS_TOPIC = "/wheel_velocity_controller/commands"

# Hardware bridge: bend calibration (#84), std_srvs/Trigger services at
# <node>/<hinge joint>/calibrate/<step>; the node runs as `robot`.
CALIBRATE_START = "start"
CALIBRATE_OPEN = "open"
CALIBRATE_FOLD = "fold"
CALIBRATE_RECORD_FLAT = "record_flat"
CALIBRATE_RECORD_FOLD = "record_fold"
CALIBRATE_STATUS = "status"
CALIBRATE_SAVE = "save"
CALIBRATE_CANCEL = "cancel"


def calibrate_service(joint: str, step: str) -> str:
    """Full name of a calibration service on the robot node."""
    return f"{ROBOT_NS}/{joint}/calibrate/{step}"


# mux_status node
ACTIVE_INPUT_TOPIC = f"{ROBOT_NS}/active_input"

# Late subscribers (a UI that starts after the node) still get the last value.
LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
