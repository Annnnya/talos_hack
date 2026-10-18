"""Defaults shared by the CLI, the web server and the ROS nodes.

Plain values only, so any entry point can import this without optional extras or
rclpy.
"""

PORT_ENV_VAR = "TALOS_SERVO_PORT"
IMU_BUS_ENV_VAR = "TALOS_IMU_BUS"
DEFAULT_CONFIG = "config/robot.yaml"
# Name of the ROS node that owns the robot; its topics and services live under
# /robot/.
ROBOT_NODE_NAME = "robot"
ROBOT_NS = f"/{ROBOT_NODE_NAME}"
QUEUE_DEPTH = 10
# Move speed (steps/s) when a position command does not say. Low, like every
# default here.
DEFAULT_MOVE_SPEED = 200
# How often the mode node repeats the STOP state on /robot/stop_lock. Well
# below the stop lock timeout in config/twist_mux.yaml, so a few late beats
# never lock the base by mistake.
STOP_LOCK_PERIOD_S = 0.1
# twist_mux's stop lock timeout (config/twist_mux.yaml). A STOP state older
# than this means the beats stopped, which twist_mux treats as STOP on.
STOP_LOCK_TIMEOUT_S = 0.5
# twist_mux input the web server drives the base on (config/twist_mux.yaml).
WEB_CMD_VEL_TOPIC = "/web/cmd_vel"
