"""The URDF model agrees with config/robot.yaml and the wheelbase helper."""

from __future__ import annotations

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from talos_servo.config import POSITION, SPEED, load_config
from talos_servo.model import wheelbase_m

xacro = pytest.importorskip("xacro")

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "description" / "talos.urdf.xacro"
CONFIG = load_config(ROOT / "config" / "robot.yaml")
CONTROLLERS = ROOT / "config" / "ros2_control.yaml"
COMMAND_INTERFACE = {SPEED: "velocity", POSITION: "position"}
HINGE = CONFIG.model.hinge_joint
LEGS = ("front_leg_joint", "rear_leg_joint")


@pytest.fixture(scope="module")
def robot() -> ET.Element:
    return ET.fromstring(xacro.process_file(str(XACRO)).toxml())


def joints(robot: ET.Element) -> dict[str, ET.Element]:
    return {j.get("name"): j for j in robot.findall("joint")}


def test_model_joints_match_the_config(robot):
    """Both ways: every servo is in the model and every moving model joint
    is a servo. Leg joints only mimic the hinge."""
    moving = {
        name
        for name, joint in joints(robot).items()
        if joint.get("type") != "fixed" and joint.find("mimic") is None
    }
    assert moving == {j.name for j in CONFIG.joints}


def test_bend_limits_match_the_config(robot):
    limit = joints(robot)[HINGE].find("limit")
    assert float(limit.get("lower")) == pytest.approx(CONFIG.model.bend_min_rad)
    assert float(limit.get("upper")) == pytest.approx(CONFIG.model.bend_max_rad)


def test_legs_mimic_the_hinge(robot):
    for leg in LEGS:
        assert joints(robot)[leg].find("mimic").get("joint") == HINGE


def hardware_joints(robot: ET.Element) -> dict[str, ET.Element]:
    return {j.get("name"): j for j in robot.findall("ros2_control/joint")}


def test_ros2_control_has_every_enabled_joint(robot):
    """The bridge drops a whole command naming a joint it does not drive."""
    expected = {j.name for j in CONFIG.enabled_joints}
    assert set(hardware_joints(robot)) == expected
    assert HINGE in expected


def test_hinge_starts_flat(robot):
    """The plugin sends this until a controller writes a goal; the bridge
    ignores it, so starting never moves the hinge."""
    state = hardware_joints(robot)[HINGE].find(
        "state_interface[@name='position']/param[@name='initial_value']"
    )
    assert float(state.text) == pytest.approx(math.pi)


def test_command_interfaces_match_the_joint_control(robot):
    controls = {j.name: j.control for j in CONFIG.joints}
    for name, joint in hardware_joints(robot).items():
        commands = [c.get("name") for c in joint.findall("command_interface")]
        assert commands == [COMMAND_INTERFACE[controls[name]]]


def test_no_transmission(robot):
    """The plugin ignores transmissions; the bridge converts the hinge."""
    assert robot.find("ros2_control/transmission") is None


def test_controllers_cover_every_hardware_joint(robot):
    params = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
    claimed = [
        joint
        for name, section in params.items()
        if name != "controller_manager"
        for joint in section["ros__parameters"]["joints"]
    ]
    assert sorted(claimed) == sorted(hardware_joints(robot))


def disabled_hinge_config(tmp_path: Path) -> Path:
    data = yaml.safe_load(
        (ROOT / "config" / "robot.yaml").read_text(encoding="utf-8")
    )
    data["joints"][HINGE]["enabled"] = False
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def test_disabled_hinge_stays_out_of_ros2_control(tmp_path):
    config = disabled_hinge_config(tmp_path)
    robot = ET.fromstring(
        xacro.process_file(
            str(XACRO), mappings={"robot_config": config.as_posix()}
        ).toxml()
    )
    assert HINGE not in hardware_joints(robot)
    # The model joint stays: the legs still mimic it.
    assert HINGE in joints(robot)


def test_launch_spawns_the_hinge_controller_only_when_enabled(tmp_path):
    pytest.importorskip("launch_ros")
    spec = importlib.util.spec_from_file_location(
        "ros2_control_launch", ROOT / "launch" / "ros2_control.launch.py"
    )
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    repo = launch.spawned(str(ROOT / "config" / "robot.yaml"))
    assert launch.HINGE_CONTROLLER in repo
    disabled = launch.spawned(str(disabled_hinge_config(tmp_path)))
    assert launch.HINGE_CONTROLLER not in disabled


def _leg_angle(joint: ET.Element, alpha: float) -> float:
    mimic = joint.find("mimic")
    return float(mimic.get("multiplier")) * alpha + float(mimic.get("offset"))


def _wheel_xz(robot: ET.Element, wheel: str, alpha: float) -> tuple:
    """Wheel axle in base_link. Every hinge and leg axis is +y."""
    all_joints = joints(robot)
    wheel_joint = all_joints[wheel]
    leg = wheel_joint.find("parent").get("link")
    leg_joint = next(
        j for j in all_joints.values() if j.find("child").get("link") == leg
    )
    x = float(wheel_joint.find("origin").get("xyz").split()[0])
    theta = _leg_angle(leg_joint, alpha)
    return x * math.cos(theta), -x * math.sin(theta)


@pytest.mark.parametrize("alpha_deg", [180, 135, 90])
def test_model_wheelbase_matches_the_helper(robot, alpha_deg):
    alpha = math.radians(alpha_deg)
    fl, _, rl, _ = CONFIG.base.wheels
    front_x, front_z = _wheel_xz(robot, fl, alpha)
    rear_x, rear_z = _wheel_xz(robot, rl, alpha)
    assert front_x - rear_x == pytest.approx(
        wheelbase_m(alpha, CONFIG.model.leg_length_m)
    )
    # Both axles at the same height: the platform stays level.
    assert front_z == pytest.approx(rear_z)
    # Folding lifts the hinge: the axles sit below it.
    assert front_z <= 0
