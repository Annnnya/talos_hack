"""The whole robot through ros2_control (#85).

Starts robot_state_publisher with description/talos.urdf.xacro, the
controller_manager with the topic based hardware plugin, a joint state
broadcaster, forward command controllers for the wheels and the hinge, and
our hardware bridge node on the servo port. Stop talos-robot (the robot node)
first: only one program may own the port.

    pixi run -e ros ros2-control port:=/dev/ttyACM0

Port defaults to $TALOS_SERVO_PORT. The model and the bridge read the same
robot config. See docs/ros2_control.md.
"""

from __future__ import annotations

import os
from pathlib import Path

import xacro
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from talos_servo.config import load_config

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "description" / "talos.urdf.xacro"
CONTROLLERS = ROOT / "config" / "ros2_control.yaml"
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"
PORT_ENV_VAR = "TALOS_SERVO_PORT"
SPAWNED = ("joint_state_broadcaster", "wheel_velocity_controller")
# Only spawned while the hinge is enabled: it claims the hinge joint.
HINGE_CONTROLLER = "bend_position_controller"


def spawned(config_path: str) -> tuple[str, ...]:
    """Controllers to spawn for this robot config."""
    config = load_config(config_path)
    model = config.model
    enabled = {j.name for j in config.enabled_joints}
    if model is not None and model.hinge_joint in enabled:
        return (*SPAWNED, HINGE_CONTROLLER)
    return SPAWNED


def _robot(context: LaunchContext) -> list:
    port = LaunchConfiguration("port").perform(context)
    # Forward slashes: xacro reads the path inside a Python string literal.
    config = (
        Path(LaunchConfiguration("config").perform(context))
        .resolve()
        .as_posix()
    )
    # Without a port the bridge exits at once while controller_manager and
    # the spawners keep running, which is confusing on the bench.
    if not port:
        raise RuntimeError(
            f"no servo port: pass port:=/dev/ttyACM0 or set {PORT_ENV_VAR}"
        )
    description = xacro.process_file(
        str(XACRO), mappings={"robot_config": config}
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
            parameters=[str(CONTROLLERS)],
            remappings=[("~/robot_description", "/robot_description")],
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
        ExecuteProcess(
            cmd=[
                "talos-hardware-bridge-ros",
                "--port",
                port,
                "--config",
                config,
            ],
            output="screen",
            # Nothing moves without the bridge: end the launch.
            on_exit=Shutdown(reason="hardware bridge exited"),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value=os.environ.get(PORT_ENV_VAR, ""),
                description=f"servo serial port (default ${PORT_ENV_VAR})",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=str(ROBOT_CONFIG),
                description="robot config read by the model and the bridge",
            ),
            OpaqueFunction(function=_robot),
        ]
    )
