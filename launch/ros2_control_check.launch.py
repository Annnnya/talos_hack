"""Bench check for #88: one wheel through ros2_control.

Starts robot_state_publisher, the controller_manager with the topic based
hardware plugin, a joint state broadcaster, a wheel velocity controller and
our hardware bridge node on the servo port. Stop talos-robot (the robot node)
first: only one program may own the port.

    pixi run -e ros ros2-control-check port:=/dev/ttyACM0

Port defaults to $TALOS_SERVO_PORT. See docs/ros2_control.md.
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    Shutdown,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent.parent
URDF = ROOT / "description" / "ros2_control_check.urdf"
CONTROLLERS = ROOT / "config" / "ros2_control_check.yaml"
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"
PORT_ENV_VAR = "TALOS_SERVO_PORT"
SPAWNED = ("joint_state_broadcaster", "wheel_velocity_controller")


def _require_port(context: LaunchContext) -> list:
    # Without a port the bridge exits at once while controller_manager and
    # the spawners keep running, which is confusing on the bench.
    if not LaunchConfiguration("port").perform(context):
        raise RuntimeError(
            f"no servo port: pass port:=/dev/ttyACM0 or set {PORT_ENV_VAR}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    config = LaunchConfiguration("config")
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
                description="robot config with the joint ids",
            ),
            OpaqueFunction(function=_require_port),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": URDF.read_text()}],
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
                for name in SPAWNED
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
                # The check is pointless without the bridge: end the launch.
                on_exit=Shutdown(reason="hardware bridge exited"),
            ),
        ]
    )
