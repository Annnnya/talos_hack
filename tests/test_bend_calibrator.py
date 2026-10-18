"""Bend calibration session on a Robot over the fake SDK (#84)."""

from __future__ import annotations

import copy
import math

import pytest
import yaml
from conftest import BASE, FakeClock
from scservo_sdk import COMM_RX_TIMEOUT, SMS_STS_MOVING

from talos_servo.bend_calibration import BendCalibration, load_calibration
from talos_servo.bend_calibrator import (
    CALIBRATION_SPEED,
    FOLD,
    IDLE_TIMEOUT_S,
    JOG_LEAD,
    OPEN,
    BendCalibrator,
)
from talos_servo.bus import MAX_POSITION, ServoError
from talos_servo.config import load_config
from talos_servo.robot import CommandError, Robot

BEND_ID = 5
FLAT = 4000
START = 3500


def robot_data() -> dict:
    data = copy.deepcopy(BASE)
    data["joints"]["bend"]["enabled"] = True
    data["model"] = {
        "hinge_joint": "bend",
        "leg_gear_ratio": 0.5,
        "bend_min_deg": 90,
        "bend_max_deg": 180,
        "bend_flat_steps": FLAT,
    }
    return data


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def paths(tmp_path):
    robot = tmp_path / "robot.yaml"
    robot.write_text(yaml.safe_dump(robot_data()), "utf-8")
    return robot, tmp_path / "bend_calibration.yaml"


@pytest.fixture
def robot(sdk, clock, paths):
    config_path, cal_path = paths
    r = Robot(sdk.bus(), load_config(config_path, cal_path), clock=clock)
    r.open()
    sdk.handler.position = START
    yield r
    r.close()


@pytest.fixture
def calibrator(robot, paths):
    config_path, cal_path = paths
    return BendCalibrator(robot, config_path=config_path, calibration=cal_path)


def moves(sdk) -> list[tuple]:
    return [c for c in sdk.handler.calls if c[0] == "move"]


def at(sdk, position: int) -> None:
    sdk.handler.position = position


def test_jog_step_is_about_one_degree(calibrator):
    # 1 deg is 11.4 steps at 1:1.
    assert calibrator.jog_steps == 11


def test_start_lets_the_hinge_use_the_whole_turn(calibrator, robot):
    assert "raw 3500" in calibrator.start()
    bend = robot.joints["bend"]
    assert (bend.min_pos, bend.max_pos) == (0, MAX_POSITION)


def test_jog_moves_slowly_by_one_step(calibrator, sdk):
    calibrator.start()
    calibrator.jog(OPEN)
    assert moves(sdk)[-1][1:4] == (BEND_ID, START + 11, CALIBRATION_SPEED)
    calibrator.jog(FOLD)
    assert moves(sdk)[-1][2] == START


def test_jog_goal_stays_close_to_the_hinge(calibrator, sdk):
    calibrator.start()
    for _ in range(10):
        calibrator.jog(OPEN)  # the hinge has not moved yet
    assert moves(sdk)[-1][2] == START + JOG_LEAD * 11


def test_jog_reaches_past_the_old_limits(calibrator, sdk):
    # The robot config clamps bend to 2976..4000; flat may sit higher.
    calibrator.start()
    at(sdk, 4090)
    calibrator.jog(OPEN)
    assert moves(sdk)[-1][2] == MAX_POSITION


def test_jog_needs_a_direction(calibrator):
    calibrator.start()
    with pytest.raises(CommandError):
        calibrator.jog(2)


def test_calls_need_a_running_session(calibrator):
    for call in (
        lambda: calibrator.jog(OPEN),
        calibrator.record_flat,
        calibrator.record_fold,
        calibrator.status,
        calibrator.save,
        calibrator.cancel,
    ):
        with pytest.raises(CommandError, match="no calibration running"):
            call()


def test_second_start_is_refused(calibrator):
    calibrator.start()
    with pytest.raises(CommandError, match="already running"):
        calibrator.start()


def test_record_waits_until_the_hinge_stops(calibrator, sdk):
    calibrator.start()
    sdk.handler.registers[SMS_STS_MOVING] = 1
    with pytest.raises(CommandError, match="still moving"):
        calibrator.record_flat()


def test_record_fails_loudly_when_offline(calibrator, sdk):
    calibrator.start()
    sdk.handler.fail[("read_block", BEND_ID)] = (COMM_RX_TIMEOUT, 0)
    with pytest.raises(ServoError):
        calibrator.record_flat()


def test_status_shows_the_points(calibrator, sdk):
    calibrator.start()
    at(sdk, 3950)
    calibrator.record_flat()
    assert calibrator.status() == (
        "bend at raw 3950; flat raw 3950, max fold not recorded"
    )


def test_save_needs_both_points(calibrator):
    calibrator.start()
    calibrator.record_flat()
    with pytest.raises(CommandError, match="before saving"):
        calibrator.save()
    assert calibrator.active


def record(calibrator, sdk, flat: int, fold: int) -> None:
    calibrator.start()
    at(sdk, flat)
    calibrator.record_flat()
    at(sdk, fold)
    calibrator.record_fold()


def test_save_writes_the_file_and_applies_it(calibrator, sdk, robot, paths):
    record(calibrator, sdk, 4050, 3050)
    message = calibrator.save()
    assert "steps 3050..4050" in message
    assert load_calibration(paths[1]) == BendCalibration(4050, 3050)
    assert not calibrator.active
    hinge = robot.config.hinge
    assert hinge.flat_steps == 4050
    assert hinge.alpha(3050) == pytest.approx(robot.config.model.bend_min_rad)
    assert robot.config.model.bend_min_deg == pytest.approx(
        180 - 1000 * 360 / 4096
    )
    bend = robot.joints["bend"]
    assert (bend.min_pos, bend.max_pos) == (3050, 4050)


def test_saved_calibration_loads_at_start(calibrator, sdk, paths):
    record(calibrator, sdk, 4050, 3050)
    calibrator.save()
    config = load_config(*paths)
    assert config.hinge.flat_steps == 4050
    assert config.hinge.alpha(4050) == pytest.approx(math.pi)


def test_bad_points_are_not_saved(calibrator, sdk, robot, paths):
    record(calibrator, sdk, 3000, 3500)  # swapped
    with pytest.raises(ValueError, match="swapped"):
        calibrator.save()
    assert not paths[1].exists()
    assert calibrator.active
    assert robot.config.hinge.flat_steps == FLAT


def test_cancel_restores_the_limits_and_moves_nothing(calibrator, robot, sdk):
    calibrator.start()
    before = len(moves(sdk))
    assert calibrator.cancel() == "calibration cancelled"
    assert len(moves(sdk)) == before
    bend = robot.joints["bend"]
    assert (bend.min_pos, bend.max_pos) == robot.config.hinge.step_range
    assert not calibrator.active


def test_cancel_far_outside_the_range_warns(calibrator, sdk, robot):
    calibrator.start()
    at(sdk, 100)
    calibrator.status()  # reads the new position
    assert "outside its range" in calibrator.cancel()


def test_idle_session_expires(calibrator, clock):
    calibrator.start()
    assert not calibrator.expired(clock.now + IDLE_TIMEOUT_S)
    calibrator.status()
    clock.now += IDLE_TIMEOUT_S + 1
    assert calibrator.expired(clock.now)


def test_disabled_hinge_cannot_be_calibrated(sdk, clock, tmp_path):
    data = copy.deepcopy(BASE)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data), "utf-8")
    r = Robot(sdk.bus(), load_config(path), clock=clock)
    with pytest.raises(CommandError, match="no enabled hinge"):
        BendCalibrator(r, config_path=path)


def test_jog_never_moves_the_other_way(calibrator, sdk):
    # The hinge was pushed far past the last goal: open must still open.
    calibrator.start()
    at(sdk, 3800)
    calibrator.jog(OPEN)
    assert moves(sdk)[-1][2] == 3811
