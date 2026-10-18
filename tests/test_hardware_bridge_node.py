"""Hardware bridge node on a Robot over the fake SDK.

Skipped where ROS is missing.
"""

from __future__ import annotations

import copy
import math

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from conftest import BASE, SMALL, FakeClock, config, wait_for  # noqa: E402
from scservo_sdk import COMM_RX_TIMEOUT, SMS_STS_TORQUE_ENABLE  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from talos_servo.bend_calibrator import (  # noqa: E402
    CALIBRATION_SPEED,
    IDLE_TIMEOUT_S,
    BendCalibrator,
)
from talos_servo.bus import ServoError  # noqa: E402
from talos_servo.config import load_config  # noqa: E402
from talos_servo.defaults import ROBOT_NODE_NAME  # noqa: E402
from talos_servo.robot import FAULT_CLEAR_S, Robot  # noqa: E402
from talos_servo.ros.convert import (  # noqa: E402
    CENTER_STEP,
    RAD_PER_STEP,
    step_speed_to_rad,
)
from talos_servo.ros.hardware_bridge_node import (  # noqa: E402
    HardwareBridgeNode,
)
from talos_servo.ros.names import (  # noqa: E402
    CALIBRATE_CANCEL,
    CALIBRATE_FOLD,
    CALIBRATE_OPEN,
    CALIBRATE_RECORD_FLAT,
    CALIBRATE_RECORD_FOLD,
    CALIBRATE_SAVE,
    CALIBRATE_START,
    CALIBRATE_STATUS,
)
from talos_servo.ros.web_backend import (  # noqa: E402
    STATUS_PREFIX,
    RosBackend,
)

INVERTED = {
    **SMALL,
    "joints": {
        **SMALL["joints"],
        "wheel": {"id": 2, "control": "speed", "inverted": True},
    },
}


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def robot(sdk, clock):
    r = Robot(sdk.bus(), config(SMALL), clock=clock)
    r.open()
    yield r
    r.close()


@pytest.fixture
def node(robot):
    n = HardwareBridgeNode(robot)
    yield n
    n.destroy_node()


def commands(names, position=(), velocity=()) -> JointState:
    return JointState(
        name=list(names), position=list(position), velocity=list(velocity)
    )


def hold(node, position: float = 0.0) -> None:
    """The plugin's first goal for bend, which the bridge ignores."""
    node.on_commands(commands(["bend"], position=[position]))


def test_states_carry_position_velocity_and_current(node, robot):
    robot.joints["wheel"].current = 0.42
    robot.joints["wheel"].speed = 100
    msg = node.states_msg()
    assert msg.name == ["bend", "wheel"]
    # The plugin reads position for every joint, so it is always there.
    assert len(msg.position) == len(msg.velocity) == len(msg.effort) == 2
    assert msg.velocity[1] == pytest.approx(step_speed_to_rad(100))
    assert msg.effort[1] == pytest.approx(0.42)


def test_wheel_position_keeps_counting_past_a_turn(node, robot):
    robot.joints["wheel"].position = 4000
    node.states_msg()
    robot.joints["wheel"].position = 100
    assert node.states_msg().position[1] == pytest.approx(196 * RAD_PER_STEP)


def test_bend_position_is_an_angle(node, robot):
    robot.joints["bend"].position = CENTER_STEP + 100
    assert node.states_msg().position[0] == pytest.approx(100 * RAD_PER_STEP)


def test_first_goal_is_ignored_until_a_different_goal(node, robot):
    # The plugin sends 0.0 for a position joint without an initial_value
    # until a controller writes a goal.
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.0]))
    assert robot.joints["bend"].target is None
    node.on_commands(commands(["bend"], position=[0.1]))
    node.on_commands(commands(["bend"], position=[0.0]))
    assert robot.joints["bend"].target == CENTER_STEP


def test_packed_commands_spin_the_wheel_and_move_bend(node, robot):
    hold(node)
    node.on_commands(
        commands(
            ["bend", "wheel"],
            position=[0.1],
            velocity=[step_speed_to_rad(200)],
        )
    )
    assert robot.joints["wheel"].target_speed == 200
    assert robot.joints["bend"].target == CENTER_STEP + round(
        0.1 / RAD_PER_STEP
    )


def test_nan_bend_goal_still_spins_the_wheel(node, robot):
    node.on_commands(
        commands(
            ["bend", "wheel"],
            position=[float("nan")],
            velocity=[step_speed_to_rad(200)],
        )
    )
    assert robot.joints["wheel"].target_speed == 200
    assert robot.joints["bend"].target is None


def test_inverted_wheel_turns_the_servo_the_other_way(sdk, clock):
    robot = Robot(sdk.bus(), config(INVERTED), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        node.on_commands(commands(["wheel"], velocity=[step_speed_to_rad(200)]))
        node.tick()
        speed = sdk.handler.sync_writes()[-1][2]
        assert speed[-2] | speed[-1] << 8 == 0x8000 | 200
    finally:
        node.destroy_node()
        robot.close()


def test_base_wheels_skip_the_joint_speed_cap(sdk, clock):
    # The mecanum controller limits the body velocity instead.
    robot = Robot(sdk.bus(), config(BASE), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        fast = step_speed_to_rad(BASE["max_speed"] * 2)
        node.on_commands(commands(["fl"], velocity=[fast]))
        assert robot.joints["fl"].target_speed == BASE["max_speed"] * 2
    finally:
        node.destroy_node()
        robot.close()


def test_other_speed_joints_keep_the_cap(node, robot):
    fast = step_speed_to_rad(SMALL["max_speed"] * 2)
    node.on_commands(commands(["wheel"], velocity=[fast]))
    assert robot.joints["wheel"].target_speed == SMALL["max_speed"]


def test_same_bend_goal_is_written_once(node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    hold(node)
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.1]))
    node.on_commands(commands(["bend"], position=[0.2]))
    assert [m[1] for m in moves] == [
        CENTER_STEP + round(0.1 / RAD_PER_STEP),
        CENTER_STEP + round(0.2 / RAD_PER_STEP),
    ]


def test_failed_move_is_retried_with_the_same_goal(node, robot, monkeypatch):
    moves = []

    def move(*a, **_k):
        moves.append(a)
        if len(moves) == 1:
            raise ServoError("no answer")

    monkeypatch.setattr(robot, "move", move)
    hold(node)
    for _ in range(3):
        node.on_commands(commands(["bend"], position=[0.1]))
    assert len(moves) == 2


def test_stop_all_resends_the_same_goal(node, robot, monkeypatch):
    moves = []
    monkeypatch.setattr(robot, "move", lambda *a, **_k: moves.append(a))
    hold(node)
    node.on_commands(commands(["bend"], position=[0.1]))
    node.on_stop_all(Trigger.Request(), Trigger.Response())
    node.on_commands(commands(["bend"], position=[0.1]))
    assert len(moves) == 2


def test_wheel_stops_when_commands_stop(node, robot, clock):
    node.on_commands(commands(["wheel"], velocity=[1.0]))
    assert robot.joints["wheel"].target_speed != 0
    clock.now += robot.config.command_timeout_s + 0.1
    node.tick()
    assert robot.joints["wheel"].target_speed == 0


def test_bad_commands_are_only_logged(node, robot):
    node.on_commands(commands(["nope"], velocity=[1.0]))
    node.on_commands(commands(["wheel"], velocity=[float("nan")]))
    node.on_commands(commands(["bend", "wheel"], velocity=[1.0, 2.0, 3.0]))
    assert robot.joints["wheel"].target_speed == 0
    assert robot.joints["bend"].target is None


def test_stop_all_service(node, robot):
    node.on_commands(commands(["wheel"], velocity=[1.0]))
    response = node.on_stop_all(Trigger.Request(), Trigger.Response())
    assert response.success
    assert robot.joints["wheel"].target_speed == 0


def test_status_as_robot_is_read_by_the_web_backend(robot, monkeypatch):
    # launch/robot.launch.py runs the bridge as node "robot", so the web
    # server takes it for the robot node.
    node = HardwareBridgeNode(robot, node_name=ROBOT_NODE_NAME)
    sent = []
    monkeypatch.setattr(node.status_pub, "publish", sent.append)
    try:
        robot.tick()
        node.publish_readouts()
        (msg,) = sent
        assert sorted(s.name for s in msg.status) == sorted(
            f"{STATUS_PREFIX}{joint}" for joint in robot.joints
        )
        backend = RosBackend(robot.config)
        backend._on_status(msg)
        assert backend._first_status.is_set()
        assert all(j.online for j in backend.joints.values())
    finally:
        node.destroy_node()


def test_every_tick_sends_fresh_states(node, sdk, monkeypatch):
    sent = []
    monkeypatch.setattr(node.states_pub, "publish", sent.append)
    sdk.handler.speed = 300
    node.tick()
    node.tick()
    assert len(sent) == 2
    i = sent[-1].name.index("wheel")
    assert sent[-1].velocity[i] == pytest.approx(step_speed_to_rad(300))


def test_readout_topics_live_under_the_node(node):
    assert "/hardware_bridge/wheel/current" in node.readouts.topic_names


# --- hinge (#63, #85) ------------------------------------------------------
# Flat at joint step 3000; the fake servo reads 2048, inside the hinge range.
FLAT = 3000
START_STEPS = 2048
# 90 deg of fold: the opening changes 1:1 with the servo.
QUARTER_TURN = 1024
MODEL = {
    "hinge_joint": "bend",
    "leg_gear_ratio": 0.5,
    "bend_min_deg": 90,
    "bend_max_deg": 180,
    "bend_flat_steps": FLAT,
}


def hinge_config() -> dict:
    data = copy.deepcopy(BASE)
    data["joints"]["bend"]["enabled"] = True
    return {**data, "model": MODEL}


@pytest.fixture
def hinge_robot(sdk, clock):
    r = Robot(sdk.bus(), config(hinge_config()), clock=clock)
    r.open()
    yield r
    r.close()


@pytest.fixture
def hinge_node(hinge_robot):
    n = HardwareBridgeNode(hinge_robot)
    yield n
    n.destroy_node()


def bend_state(node) -> tuple[float, float]:
    msg = node.states_msg()
    i = msg.name.index("bend")
    return msg.position[i], msg.velocity[i]


def test_hinge_angle_comes_from_the_fixed_flat(hinge_node, hinge_robot):
    assert bend_state(hinge_node)[0] == pytest.approx(
        math.pi - (FLAT - START_STEPS) * RAD_PER_STEP
    )
    hinge_robot.joints["bend"].position = FLAT - QUARTER_TURN
    hinge_robot.joints["bend"].speed = -100
    alpha, rate = bend_state(hinge_node)
    assert alpha == pytest.approx(math.pi / 2)
    assert rate == pytest.approx(-100 * RAD_PER_STEP)


def test_hinge_ignores_the_initial_value_until_a_new_goal(
    hinge_node, hinge_robot
):
    # The URDF initial_value: what the plugin sends before any controller.
    for _ in range(3):
        hold(hinge_node, math.pi)
    assert hinge_robot.joints["bend"].target is None
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN
    hinge_node.on_commands(commands(["bend"], position=[math.pi]))
    assert hinge_robot.joints["bend"].target == FLAT


def test_restarted_bridge_ignores_the_last_goal(hinge_node, hinge_robot):
    # Only the bridge restarted: the plugin repeats the controller's goal.
    for _ in range(3):
        hold(hinge_node, 2.8)
    assert hinge_robot.joints["bend"].target is None
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN


def test_plain_position_joint_ignores_a_repeated_pi(node, robot):
    # No model: bend is a plain joint, and pi is not its initial value.
    for _ in range(3):
        hold(node, math.pi)
    assert robot.joints["bend"].target is None


def test_hinge_goals_clamp_to_the_model_limits(hinge_node, hinge_robot):
    hold(hinge_node, math.pi)
    hinge_node.on_commands(commands(["bend"], position=[math.radians(30)]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN


def test_offline_hinge_angle_is_unknown(sdk, clock):
    sdk.present = {1, 2, 3, 4}  # the bend servo (5) does not answer
    robot = Robot(sdk.bus(), config(hinge_config()), clock=clock)
    robot.open()
    node = HardwareBridgeNode(robot)
    try:
        alpha, rate = bend_state(node)
        assert math.isnan(alpha)
        assert math.isnan(rate)
    finally:
        node.destroy_node()
        robot.close()


def test_hinge_reading_far_outside_its_range_is_unknown(
    hinge_node, hinge_robot
):
    # A wrap past flat reads near 0, not a real fold.
    hinge_robot.joints["bend"].position = 3
    assert math.isnan(bend_state(hinge_node)[0])


def test_hinge_goals_are_dropped_while_the_reading_is_wrapped(
    hinge_node, hinge_robot
):
    # From step 3 the servo would turn the long way round to any goal.
    hold(hinge_node, math.pi)
    hinge_robot.joints["bend"].position = 3
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target is None
    hinge_robot.joints["bend"].position = START_STEPS
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN


def test_quiet_commands_disarm_position_joints(hinge_node, hinge_robot, clock):
    hold(hinge_node, math.pi)
    hinge_node.on_commands(commands(["bend"], position=[2.0]))
    target = hinge_robot.joints["bend"].target
    # controller_manager restarts: quiet, then its initial value again.
    clock.now += hinge_robot.config.command_timeout_s + 0.1
    for _ in range(3):
        hold(hinge_node, math.pi)
    assert hinge_robot.joints["bend"].target == target
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN


def test_steady_commands_stay_armed(hinge_node, hinge_robot, clock):
    hold(hinge_node, math.pi)
    hinge_node.on_commands(commands(["bend"], position=[2.0]))
    clock.now += hinge_robot.config.command_timeout_s / 2
    hinge_node.on_commands(commands(["bend"], position=[math.pi / 2]))
    assert hinge_robot.joints["bend"].target == FLAT - QUARTER_TURN


# --- safe state (#94) --------------------------------------------------------


def torque_writes(sdk) -> list[tuple]:
    return [
        c
        for c in sdk.handler.calls
        if c[0] == "write1" and c[2] == SMS_STS_TORQUE_ENABLE
    ]


def moves(sdk) -> list[tuple]:
    return [c for c in sdk.handler.calls if c[0] == "move"]


def drive_wheel_and_bend(node) -> None:
    hold(node)
    node.on_commands(
        commands(["bend", "wheel"], position=[0.2], velocity=[1.0])
    )
    node.tick()
    assert node.robot.joints["wheel"].target_speed != 0
    assert node.robot.joints["bend"].target is not None


def test_quiet_commands_brake_the_wheel_and_hold_bend(node, robot, sdk, clock):
    drive_wheel_and_bend(node)
    goal = robot.joints["bend"].target
    sent = len(moves(sdk))
    clock.now += robot.config.command_timeout_s + 0.1
    node.tick()
    assert robot.joints["wheel"].target_speed == 0
    assert robot.joints["bend"].target == goal
    assert len(moves(sdk)) == sent
    assert torque_writes(sdk) == []


def test_bus_error_brakes_and_refuses_commands(node, robot, sdk):
    drive_wheel_and_bend(node)
    sent = len(moves(sdk))
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    node.tick()
    del sdk.fail[("sync_write", 254)]
    assert node._fault is not None
    node.on_commands(
        commands(["bend", "wheel"], position=[0.3], velocity=[1.0])
    )
    node.tick()
    assert robot.joints["wheel"].target_speed == 0
    assert len(moves(sdk)) == sent
    assert torque_writes(sdk) == []


def test_bridge_follows_commands_once_the_fault_clears(node, robot, sdk, clock):
    drive_wheel_and_bend(node)
    sdk.fail[("sync_write", 254)] = (COMM_RX_TIMEOUT, 0)
    node.tick()
    del sdk.fail[("sync_write", 254)]
    node.tick()
    clock.now += FAULT_CLEAR_S
    node.tick()
    assert node._fault is None
    node.on_commands(commands(["wheel"], velocity=[1.0]))
    assert robot.joints["wheel"].target_speed != 0


def test_shutdown_brakes_without_turning_torque_off(node, robot, sdk):
    drive_wheel_and_bend(node)
    sent = len(moves(sdk))
    robot.close()
    assert robot.joints["wheel"].target_speed == 0
    packet = sdk.handler.sync_writes()[-1][2]
    assert packet[5] | packet[6] << 8 == 0
    assert len(moves(sdk)) == sent
    assert torque_writes(sdk) == []


# --- bend calibration (#84) ----------------------------------------------
@pytest.fixture
def cal_paths(tmp_path):
    robot = tmp_path / "robot.yaml"
    robot.write_text(yaml.safe_dump(hinge_config()), "utf-8")
    return robot, tmp_path / "bend_calibration.yaml"


@pytest.fixture
def cal_robot(sdk, clock, cal_paths):
    r = Robot(sdk.bus(), load_config(*cal_paths), clock=clock)
    r.open()
    yield r
    r.close()


@pytest.fixture
def cal_node(cal_robot, cal_paths):
    config_path, cal_path = cal_paths
    calibrator = BendCalibrator(
        cal_robot, config_path=config_path, calibration=cal_path
    )
    n = HardwareBridgeNode(cal_robot, calibrator=calibrator)
    yield n
    n.destroy_node()


def calibrate(node, step: str) -> Trigger.Response:
    return node.on_calibrate(
        step, node.calibration_steps[step], Trigger.Response()
    )


def ok(node, step: str) -> str:
    response = calibrate(node, step)
    assert response.success, response.message
    return response.message


def bend_goals(robot, node, position: float) -> int | None:
    node.on_commands(commands(["bend"], position=[position]))
    return robot.joints["bend"].target


def test_calibration_services_are_served(cal_node):
    expected = {
        f"/hardware_bridge/bend/calibrate/{step}"
        for step in (
            CALIBRATE_START,
            CALIBRATE_OPEN,
            CALIBRATE_FOLD,
            CALIBRATE_RECORD_FLAT,
            CALIBRATE_RECORD_FOLD,
            CALIBRATE_STATUS,
            CALIBRATE_SAVE,
            CALIBRATE_CANCEL,
        )
    }
    assert set(cal_node.calibration_steps) == {
        name.rsplit("/", 1)[1] for name in expected
    }
    wait_for(
        lambda: (
            expected
            <= {name for name, _ in cal_node.get_service_names_and_types()}
        )
    )


def test_no_calibration_without_a_hinge(robot):
    # SMALL has no model, so no hinge to calibrate even if asked.
    calibrator = object()
    n = HardwareBridgeNode(robot, calibrator=calibrator)
    try:
        assert n.calibrator is None
        assert n.calibration_steps == {}
    finally:
        n.destroy_node()


def test_hinge_goals_are_ignored_while_calibrating(cal_node, cal_robot):
    hold(cal_node, math.pi)
    ok(cal_node, CALIBRATE_START)
    cal_node.on_commands(
        commands(["bend", "fl"], position=[2.0], velocity=[1.0])
    )
    assert cal_robot.joints["bend"].target is None
    # The wheels still follow their controller.
    assert cal_robot.joints["fl"].target_speed != 0


def test_hinge_angle_is_unknown_while_calibrating(cal_node):
    ok(cal_node, CALIBRATE_START)
    assert math.isnan(bend_state(cal_node)[0])


def test_jog_moves_the_hinge_slowly(cal_node, cal_robot, sdk):
    ok(cal_node, CALIBRATE_START)
    ok(cal_node, CALIBRATE_OPEN)
    move = [c for c in sdk.handler.calls if c[0] == "move"][-1]
    assert move[1:4] == (5, START_STEPS + 11, CALIBRATION_SPEED)


def test_save_applies_the_new_hinge_at_once(
    cal_node, cal_robot, sdk, cal_paths
):
    hold(cal_node, math.pi)
    assert bend_goals(cal_robot, cal_node, 2.5) is not None  # armed
    ok(cal_node, CALIBRATE_START)
    sdk.handler.position = 3100
    ok(cal_node, CALIBRATE_RECORD_FLAT)
    sdk.handler.position = 2100
    ok(cal_node, CALIBRATE_RECORD_FOLD)
    assert "steps 2100..3100" in ok(cal_node, CALIBRATE_SAVE)
    assert cal_paths[1].exists()
    cal_robot.joints["bend"].position = 3100
    assert bend_state(cal_node)[0] == pytest.approx(math.pi)
    # The controller's old goal repeats: it must not move the hinge.
    target = cal_robot.joints["bend"].target
    assert bend_goals(cal_robot, cal_node, 2.5) == target
    # A new goal does, in the new calibration.
    assert bend_goals(cal_robot, cal_node, math.pi) == 3100


def test_cancel_keeps_the_old_hinge_and_waits_for_a_new_goal(
    cal_node, cal_robot
):
    hold(cal_node, math.pi)
    assert bend_goals(cal_robot, cal_node, 2.5) is not None
    ok(cal_node, CALIBRATE_START)
    ok(cal_node, CALIBRATE_CANCEL)
    target = cal_robot.joints["bend"].target
    assert bend_goals(cal_robot, cal_node, 2.5) == target
    assert cal_robot.joints["bend"].target == target
    assert cal_node._hinge.flat_steps == FLAT
    bend = cal_robot.joints["bend"]
    assert (bend.min_pos, bend.max_pos) == cal_node._hinge.step_range


def test_idle_calibration_is_cancelled(cal_node, cal_robot, clock):
    ok(cal_node, CALIBRATE_START)
    clock.now += IDLE_TIMEOUT_S + 1
    cal_node.tick()
    assert not cal_node.calibrator.active
    bend = cal_robot.joints["bend"]
    assert (bend.min_pos, bend.max_pos) == cal_node._hinge.step_range


def test_calibration_errors_come_back_as_failures(cal_node):
    response = calibrate(cal_node, CALIBRATE_SAVE)
    assert not response.success
    assert "no calibration running" in response.message


def test_unexpected_calibration_error_keeps_the_bridge_up(cal_node):
    def boom() -> str:
        raise RuntimeError("disk on fire")

    response = cal_node.on_calibrate(CALIBRATE_SAVE, boom, Trigger.Response())
    assert not response.success
    assert "RuntimeError: disk on fire" in response.message
