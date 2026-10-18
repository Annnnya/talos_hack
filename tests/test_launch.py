"""launch/robot.launch.py starts the right processes for each flag. Nothing is
started and no port is opened."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

ROOT = Path(__file__).resolve().parent.parent
DRIVE = ["hardware_bridge", "mode_node", "twist_mux", "mux_status"]
ROBOT_NODE_DRIVE = ["robot_node", *DRIVE[1:]]
IMU = ["imu_node", "imu_filter", "imu_tf", "ekf"]
CONTROLLER_REFERENCE = "/mecanum_drive_controller/reference"


@pytest.fixture(scope="module")
def launch():
    spec = importlib.util.spec_from_file_location(
        "robot_launch", ROOT / "launch" / "robot.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_defaults_start_the_drive_nodes_and_the_web(launch):
    assert list(launch.processes("/dev/ttyX", "robot.yaml")) == [
        *DRIVE,
        "web",
    ]


def test_flags_add_the_camera_and_the_imu_chain(launch):
    cmds = launch.processes(
        "/dev/ttyX", "robot.yaml", web=False, camera=True, imu=True
    )
    assert list(cmds) == [*DRIVE, "camera_node", *IMU]


def test_viz_adds_the_ekf_without_the_imu(launch):
    # The EKF alone publishes odom -> base_link, which RViz needs.
    cmds = launch.processes("/dev/ttyX", "robot.yaml", web=False, viz=True)
    assert list(cmds) == [*DRIVE, "ekf"]


def test_viz_with_the_imu_starts_one_ekf(launch):
    cmds = launch.processes(
        "/dev/ttyX", "robot.yaml", web=False, imu=True, viz=True
    )
    assert list(cmds) == [*DRIVE, *IMU]


def test_viz_needs_ros2_control(launch):
    from launch import LaunchContext

    context = LaunchContext()
    context.launch_configurations.update(
        {
            "port": "/dev/ttyX",
            "config": str(ROOT / "config" / "robot.yaml"),
            "web": "false",
            "ros2_control": "false",
            "viz": "true",
        }
    )
    with pytest.raises(RuntimeError, match="viz"):
        launch._robot(context)


def test_robot_node_instead_of_ros2_control(launch):
    cmds = launch.processes("/dev/ttyX", "robot.yaml", ros2_control=False)
    assert list(cmds) == [*ROBOT_NODE_DRIVE, "web"]


@pytest.mark.parametrize(
    ("ros2_control", "owner"),
    [(True, "hardware_bridge"), (False, "robot_node")],
)
def test_only_the_port_owner_gets_the_port(launch, ros2_control, owner):
    cmds = launch.processes(
        "/dev/ttyX",
        "robot.yaml",
        camera=True,
        imu=True,
        ros2_control=ros2_control,
    )
    with_port = [name for name, cmd in cmds.items() if "/dev/ttyX" in cmd]
    assert with_port == [owner]


def test_bridge_answers_as_the_robot_node(launch):
    # The web server looks for /robot/stop_all and "robot: <joint>" status.
    cmd = launch.processes("/dev/ttyX", "robot.yaml")["hardware_bridge"]
    assert "__node:=robot" in cmd


def test_web_goes_through_the_robot_node(launch):
    assert "--ros" in launch.processes("/dev/ttyX", "robot.yaml")["web"]


@pytest.mark.parametrize(
    ("ros2_control", "target"),
    [(True, CONTROLLER_REFERENCE), (False, "/cmd_vel")],
)
def test_twist_mux_feeds_the_base(launch, ros2_control, target):
    cmd = launch.processes(
        "/dev/ttyX", "robot.yaml", ros2_control=ros2_control
    )["twist_mux"]
    assert f"cmd_vel_out:={target}" in cmd


def test_config_files_exist(launch):
    cmds = launch.processes("/dev/ttyX", "robot.yaml", camera=True, imu=True)
    files = [
        Path(arg)
        for cmd in cmds.values()
        for arg in cmd
        if arg.endswith(".yaml") and arg != "robot.yaml"
    ]
    assert files
    assert all(f.is_file() for f in files)


def test_description_builds(launch):
    assert launch.generate_launch_description().entities


def test_controller_odometry_feeds_the_ekf():
    yaml = pytest.importorskip("yaml")
    from talos_servo.ros.control_launch import BASE_CONTROLLER, ODOM_REMAP

    ekf = yaml.safe_load((ROOT / "config" / "ekf.yaml").read_text())
    assert (
        f"/{BASE_CONTROLLER}/odometry",
        ekf["/**"]["ros__parameters"]["odom0"],
    ) == ODOM_REMAP


def test_control_nodes_build_from_the_robot_config():
    from talos_servo.ros.control_launch import control_nodes, spawned

    config = str(ROOT / "config" / "robot.yaml")
    # robot_state_publisher, controller_manager and one spawner each.
    assert len(control_nodes(ROOT, config)) == 2 + len(spawned(config))


def test_live_rviz_watches_the_ekf_frame():
    yaml = pytest.importorskip("yaml")
    ekf = yaml.safe_load((ROOT / "config" / "ekf.yaml").read_text())
    rviz = yaml.safe_load((ROOT / "description" / "view_live.rviz").read_text())
    fixed = rviz["Visualization Manager"]["Global Options"]["Fixed Frame"]
    assert fixed == ekf["/**"]["ros__parameters"]["odom_frame"]
