"""Launch pieces for the ros2_control stack, shared by launch/robot.launch.py
and launch/ros2_control.launch.py.

robot_state_publisher with description/talos.urdf.xacro, the
controller_manager with the topic based hardware plugin and the controller
spawners. The hardware bridge (the only program on the servo port) is started
by each launch file itself. Needs the ROS launch packages, so only launch
files import this.
"""

from __future__ import annotations

from pathlib import Path

import xacro
from launch_ros.actions import Node

from talos_servo.config import load_config

# Relative to the repo root, which each launch file passes in.
XACRO = Path("description") / "talos.urdf.xacro"
CONTROLLERS = Path("config") / "ros2_control.yaml"
BASE_CONTROLLER = "mecanum_drive_controller"
# twist_mux feeds the base controller here (geometry_msgs/TwistStamped).
BASE_REFERENCE_TOPIC = f"/{BASE_CONTROLLER}/reference"
# The base controller's odometry goes to /odom for the EKF.
ODOM_REMAP = (f"/{BASE_CONTROLLER}/odometry", "/odom")
SPAWNED = ("joint_state_broadcaster", BASE_CONTROLLER)
# Only spawned while the hinge is enabled: it claims the hinge joint.
HINGE_CONTROLLER = "bend_position_controller"
# Per-wheel speeds for the expansion node (#76). Loaded but inactive: it
# claims the same wheels as the base controller, and the mode node switches
# between the two.
WHEEL_CONTROLLER = "wheel_velocity_controller"
INACTIVE = (WHEEL_CONTROLLER,)


def spawned(config_path: str) -> tuple[str, ...]:
    """Controllers to spawn active for this robot config."""
    config = load_config(config_path)
    model = config.model
    enabled = {j.name for j in config.enabled_joints}
    if model is not None and model.hinge_joint in enabled:
        return (*SPAWNED, HINGE_CONTROLLER)
    return SPAWNED


def control_nodes(root: Path, config_path: str) -> list[Node]:
    """robot_state_publisher, controller_manager and the spawners, with the
    model and controllers from the repo at `root`."""
    # Forward slashes: xacro reads the path inside a Python string literal.
    config = Path(config_path).resolve().as_posix()
    mappings = {"robot_config": config}
    model = load_config(config_path).model
    if model is not None:
        # The calibrated max fold, which robot.yaml alone does not hold.
        mappings["bend_min_deg"] = repr(model.bend_min_deg)
    description = xacro.process_file(
        str(root / XACRO), mappings=mappings
    ).toxml()
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[str(root / CONTROLLERS)],
            remappings=[
                ("~/robot_description", "/robot_description"),
                ODOM_REMAP,
            ],
            output="screen",
        ),
        *(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name],
            )
            for name in spawned(config)
        ),
        *(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name, "--inactive"],
            )
            for name in INACTIVE
        ),
    ]
