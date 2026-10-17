"""Defaults shared by the CLI, the web server and the ROS nodes.

Plain values only, so any entry point can import this without optional extras or rclpy.
"""

PORT_ENV_VAR = "TALOS_SERVO_PORT"
IMU_BUS_ENV_VAR = "TALOS_IMU_BUS"
DEFAULT_CONFIG = "config/robot.yaml"
# Namespace of the ROS servo bridge node that `--ros` clients talk to.
DEFAULT_BRIDGE = "/servo_bridge"
