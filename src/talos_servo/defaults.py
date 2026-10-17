"""Defaults shared by the CLI, the web server and the ROS nodes.

Plain values only, so any entry point can import this without optional extras or rclpy.
"""

PORT_ENV_VAR = "TALOS_SERVO_PORT"
IMU_BUS_ENV_VAR = "TALOS_IMU_BUS"
DEFAULT_CONFIG = "config/robot.yaml"
# Name of the ROS node that owns the robot; its topics and services live under /robot/.
ROBOT_NODE_NAME = "robot"
ROBOT_NS = f"/{ROBOT_NODE_NAME}"
QUEUE_DEPTH = 10
# Move speed (steps/s) when a position command does not say. Low, like every default here.
DEFAULT_MOVE_SPEED = 200
