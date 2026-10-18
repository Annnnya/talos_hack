"""Show the robot model (#64) in RViz2 with sliders for every joint.

Starts robot_state_publisher with description/talos.urdf.xacro,
joint_state_publisher_gui and RViz2. Touches no hardware.

    pixi run -e viz view-model

Drag `bend` between max fold and flat (180 deg = pi rad) to check that the
legs fold symmetrically and the platform stays level.
"""

from __future__ import annotations

from pathlib import Path

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "description" / "talos.urdf.xacro"
RVIZ_CONFIG = ROOT / "description" / "view_model.rviz"


def generate_launch_description() -> LaunchDescription:
    description = xacro.process_file(str(XACRO)).toxml()
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(RVIZ_CONFIG)],
            ),
        ]
    )
