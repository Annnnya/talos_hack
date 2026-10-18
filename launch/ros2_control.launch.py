"""The whole robot through ros2_control (#85), without the teleop nodes.

Starts robot_state_publisher with description/talos.urdf.xacro, the
controller_manager with the topic based hardware plugin, a joint state
broadcaster, mecanum_drive_controller for the base (#65), a forward command
controller for the hinge, and our hardware bridge node on the servo port.
Stop talos-robot (the robot node) first: only one program may own the port.

    pixi run -e ros ros2-control port:=/dev/ttyACM0

Port defaults to $TALOS_SERVO_PORT. The model and the bridge read the same
robot config. See docs/ros2_control.md.
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

from talos_servo.ros.control_launch import (  # noqa: F401 (tests use them)
    BASE_CONTROLLER,
    HINGE_CONTROLLER,
    control_nodes,
    spawned,
)

ROOT = Path(__file__).resolve().parent.parent
ROBOT_CONFIG = ROOT / "config" / "robot.yaml"
PORT_ENV_VAR = "TALOS_SERVO_PORT"


def _robot(context: LaunchContext) -> list:
    port = LaunchConfiguration("port").perform(context)
    config = str(Path(LaunchConfiguration("config").perform(context)).resolve())
    # Without a port the bridge exits at once while controller_manager and
    # the spawners keep running, which is confusing on the bench.
    if not port:
        raise RuntimeError(
            f"no servo port: pass port:=/dev/ttyACM0 or set {PORT_ENV_VAR}"
        )
    return [
        *control_nodes(ROOT, config),
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
