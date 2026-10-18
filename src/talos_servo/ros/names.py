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

# Expansion node (#76) in, and the controllers it feeds
# (config/ros2_control.yaml) out.
EXPANSION_RATE_TOPIC = "/expansion/rate"
HINGE_COMMANDS_TOPIC = "/bend_position_controller/commands"
WHEEL_COMMANDS_TOPIC = "/wheel_velocity_controller/commands"

# mux_status node
ACTIVE_INPUT_TOPIC = f"{ROBOT_NS}/active_input"

# Late subscribers (a UI that starts after the node) still get the last value.
LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)
