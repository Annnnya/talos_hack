"""launch/robot.launch.py starts the right processes for each flag. Nothing is
started and no port is opened."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

ROOT = Path(__file__).resolve().parent.parent
DRIVE = ["robot_node", "mode_node", "twist_mux", "mux_status"]
IMU = ["imu_node", "imu_filter", "imu_tf", "ekf"]


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


def test_only_the_robot_node_gets_the_port(launch):
    cmds = launch.processes("/dev/ttyX", "robot.yaml", camera=True, imu=True)
    with_port = [name for name, cmd in cmds.items() if "/dev/ttyX" in cmd]
    assert with_port == ["robot_node"]


def test_web_goes_through_the_robot_node(launch):
    assert "--ros" in launch.processes("/dev/ttyX", "robot.yaml")["web"]


def test_twist_mux_feeds_the_robot_node(launch):
    cmd = launch.processes("/dev/ttyX", "robot.yaml")["twist_mux"]
    assert "cmd_vel_out:=/cmd_vel" in cmd


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
