"""The whole robot in one command (#122).

Starts what driving needs: the ros2_control stack (robot_state_publisher,
controller_manager, mecanum_drive_controller for the base, #65) with the
hardware bridge on the servo port, the mode node (STOP lock), the expansion
node (hinge, #76), twist_mux, mux_status, the web server and the IMU chain
(IMU node, Madgwick filter, EKF; #91). The camera is opt-in and the IMU
chain opt-out.

    pixi run -e ros robot
    pixi run -e ros robot camera:=true port:=/dev/ttyACM0
    pixi run -e ros robot imu:=false          # no IMU on the bench
    pixi run -e ros robot viz:=true   # then `pixi run -e viz view-live`

`viz:=true` shows the live robot in RViz (#127): the joints already reach
robot_state_publisher through ros2_control, and the EKF (started even
without the IMU) publishes odom -> base_link from wheel odometry.

The IMU mount (`imu` in the robot config) reaches tf through the URDF under
ros2_control; without it a static transform publisher sends the same
values.

`ros2_control:=false` drives with the old robot node instead (needed for the
per-motor `/debug` page: under ros2_control the controllers own the joints).

The port is $TALOS_SERVO_PORT, else the one USB serial adapter the Pi lists
under /dev/serial/by-id, else /dev/ttyACM0. STOP is on at start: press
Release STOP on the web page (or Enter in `pixi run -e ros drive`) to drive.
Ctrl+C stops every process through its own stop path. The commands match
the pixi tasks of the same name in pixi.toml; keep both in step.
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

from talos_servo.config import ImuMountConfig, load_config
from talos_servo.defaults import ROBOT_NODE_NAME
from talos_servo.ros.control_launch import BASE_REFERENCE_TOPIC, control_nodes
from talos_servo.ros.port_args import find_servo_port

ROOT = Path(__file__).resolve().parent.parent
CONFIG_DIR = ROOT / "config"
ROBOT_CONFIG = CONFIG_DIR / "robot.yaml"
UI_DIR = ROOT / "web" / "out"
PORT_ENV_VAR = "TALOS_SERVO_PORT"
ROBOT_NODE = "robot_node"
# Runs under the robot node's name (ROBOT_NODE_NAME), so the web server finds
# its status and /robot/stop_all like the robot node's.
HARDWARE_BRIDGE = "hardware_bridge"
WEB = "web"
# The web server gives up when the robot node does not answer within a few
# seconds, so it is restarted until the robot node is up.
WEB_RESPAWN_DELAY_S = 2.0
TRUE = ("true", "1", "yes", "on")


def processes(
    port: str,
    config: str,
    *,
    web: bool = True,
    camera: bool = False,
    imu: bool = True,
    ros2_control: bool = True,
    viz: bool = False,
    imu_mount: ImuMountConfig | None = None,
) -> dict[str, list[str]]:
    """Name and command of every process to start, in start order.

    With `ros2_control` the controller_manager nodes from `control_nodes`
    start too; they are launch_ros nodes, not commands, so not listed here.
    `imu_mount` is where the IMU sits (`imu` in the robot config); the
    robot node path publishes it as a static transform.
    """
    twist_mux = str(CONFIG_DIR / "twist_mux.yaml")
    cmds: dict[str, list[str]] = {}
    if ros2_control:
        cmds[HARDWARE_BRIDGE] = [
            *("talos-hardware-bridge-ros", "--port", port, "--config", config),
            *("--ros-args", "-r", f"__node:={ROBOT_NODE_NAME}"),
        ]
        mux_out = BASE_REFERENCE_TOPIC
    else:
        cmds[ROBOT_NODE] = [
            *("talos-robot-ros", "--port", port, "--config", config)
        ]
        mux_out = "/cmd_vel"
    cmds["mode_node"] = ["talos-mode-ros", "--config", config]
    if ros2_control:
        # Feeds the hinge and wheel controllers, so only with ros2_control.
        cmds["expansion_node"] = ["talos-expansion-ros", "--config", config]
    cmds |= {
        "twist_mux": [
            *("ros2", "run", "twist_mux", "twist_mux", "--ros-args"),
            *("--params-file", twist_mux, "-r", f"cmd_vel_out:={mux_out}"),
        ],
        "mux_status": [
            *("talos-mux-status-ros", "--ros-args"),
            *("--params-file", twist_mux),
        ],
    }
    if web:
        cmds[WEB] = [
            *("talos-servo", "web", "--ros", "--config", config),
            *("--ui-dir", str(UI_DIR)),
        ]
    if camera:
        cmds["camera_node"] = [
            *("talos-camera-ros", "--ros-args", "--params-file"),
            str(CONFIG_DIR / "camera.yaml"),
        ]
    if imu:
        cmds["imu_node"] = ["talos-imu-ros"]
        cmds["imu_filter"] = [
            *("ros2", "run", "imu_filter_madgwick"),
            *("imu_filter_madgwick_node", "--ros-args", "--params-file"),
            str(CONFIG_DIR / "imu_filter.yaml"),
        ]
        if not ros2_control:
            # robot_state_publisher is not running, so publish the mount
            # from the config (the URDF carries it under ros2_control).
            cmds["imu_tf"] = imu_tf_command(imu_mount or ImuMountConfig())
    # The only odom -> base_link publisher, so RViz needs it too. Without the
    # IMU it runs on wheel odometry alone.
    if imu or viz:
        cmds["ekf"] = [
            *("ros2", "run", "robot_localization", "ekf_node"),
            *("--ros-args", "--params-file", str(CONFIG_DIR / "ekf.yaml")),
        ]
    return cmds


def imu_tf_command(mount: ImuMountConfig) -> list[str]:
    """`static_transform_publisher` for base_link -> imu_link."""
    x, y, z = mount.xyz
    roll, pitch, yaw = mount.rpy
    return [
        *("ros2", "run", "tf2_ros", "static_transform_publisher"),
        *("--frame-id", "base_link", "--child-frame-id", "imu_link"),
        *("--x", str(x), "--y", str(y), "--z", str(z)),
        *("--roll", str(roll), "--pitch", str(pitch), "--yaw", str(yaw)),
    ]


def _flag(context: LaunchContext, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context)
    return value.strip().lower() in TRUE


def _robot(context: LaunchContext) -> list:
    # No argument needed on the robot: the one USB serial adapter is it.
    port = LaunchConfiguration("port").perform(context) or find_servo_port()
    config = str(Path(LaunchConfiguration("config").perform(context)).resolve())
    web = _flag(context, "web")
    ros2_control = _flag(context, "ros2_control")
    viz = _flag(context, "viz")
    # Fail before anything starts, not with half a robot running.
    if not port:
        raise RuntimeError(
            "no servo port found: plug the servo adapter in, or pass "
            f"port:=/dev/ttyACM0 or set {PORT_ENV_VAR}"
        )
    if web and not (UI_DIR / "index.html").is_file():
        raise RuntimeError(
            f"no web UI in {UI_DIR}: run `cd web && npm install && npm run "
            "build` once, or pass web:=false"
        )
    # Without ros2_control nothing runs robot_state_publisher, and the robot
    # node reports the raw hinge servo angle, not the model one.
    if viz and not ros2_control:
        raise RuntimeError("viz:=true needs ros2_control:=true")
    # Fails here on a broken config, before any node starts.
    imu_mount = load_config(config).imu
    cmds = processes(
        port,
        config,
        web=web,
        camera=_flag(context, "camera"),
        imu=_flag(context, "imu"),
        ros2_control=ros2_control,
        viz=viz,
        imu_mount=imu_mount,
    )
    actions: list = control_nodes(ROOT, config) if ros2_control else []
    for name, cmd in cmds.items():
        options: dict = {}
        if name in (ROBOT_NODE, HARDWARE_BRIDGE):
            # Nothing drives without the port owner: end the launch.
            options["on_exit"] = Shutdown(reason=f"{name} exited")
        elif name == WEB:
            options["respawn"] = True
            options["respawn_delay"] = WEB_RESPAWN_DELAY_S
        actions.append(
            ExecuteProcess(
                cmd=cmd,
                name=name,
                output="screen",
                # SIGINT runs each program's Ctrl+C path (stop the wheels).
                sigterm_timeout="10",
                **options,
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value=os.environ.get(PORT_ENV_VAR, ""),
                description=(
                    f"servo serial port (default ${PORT_ENV_VAR}, else the "
                    "one adapter under /dev/serial/by-id, else /dev/ttyACM0)"
                ),
            ),
            DeclareLaunchArgument(
                "config",
                default_value=str(ROBOT_CONFIG),
                description="robot config for every node",
            ),
            DeclareLaunchArgument(
                "web", default_value="true", description="web UI on :8000"
            ),
            DeclareLaunchArgument(
                "ros2_control",
                default_value="true",
                description="drive through ros2_control (false: robot node)",
            ),
            DeclareLaunchArgument(
                "camera", default_value="false", description="camera node"
            ),
            DeclareLaunchArgument(
                "imu",
                default_value="true",
                description="IMU node, Madgwick filter and EKF",
            ),
            DeclareLaunchArgument(
                "viz",
                default_value="false",
                description="EKF for the live robot in RViz (view-live)",
            ),
            OpaqueFunction(function=_robot),
        ]
    )
