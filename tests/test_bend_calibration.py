"""Bend calibration points, checks and file (#84)."""

from __future__ import annotations

import copy

import pytest
from conftest import BASE

from talos_servo.bend_calibration import (
    CALIBRATION_ENV_VAR,
    CALIBRATION_FILE,
    BendCalibration,
    CalibrationError,
    apply_calibration,
    calibration_path,
    calibration_yaml,
    load_calibration,
    min_deg,
    save_calibration,
    validate,
    warnings,
)
from talos_servo.config import parse_config

MODEL = {
    "hinge_joint": "bend",
    "leg_gear_ratio": 0.5,
    "bend_min_deg": 90,
    "bend_max_deg": 180,
    "bend_flat_steps": 4000,
}
# 1024 steps is a quarter turn: 90 deg of fold at 1:1.
QUARTER = 1024


def robot(model: dict | None = None, inverted: bool = False):
    data = copy.deepcopy(BASE)
    data["joints"]["bend"].update(enabled=True, inverted=inverted)
    config = parse_config({**data, "model": {**MODEL, **(model or {})}})
    joint = next(j for j in config.joints if j.name == "bend")
    return config.model, joint


def test_min_angle_comes_from_the_fold_point():
    model, joint = robot()
    cal = BendCalibration(flat_steps=4000, fold_steps=4000 - QUARTER)
    assert min_deg(cal, model, joint) == pytest.approx(90.0)


def test_min_angle_follows_the_gear_ratio():
    # Ratio 1: the opening angle turns twice the servo angle.
    model, joint = robot({"leg_gear_ratio": 1.0, "bend_min_deg": 10})
    cal = BendCalibration(flat_steps=4000, fold_steps=4000 - QUARTER // 2)
    assert min_deg(cal, model, joint) == pytest.approx(90.0)


def test_inverted_hinge_points_are_raw_positions():
    # Folding raises the raw position on an inverted servo.
    model, joint = robot({"bend_flat_steps": 100}, inverted=True)
    cal = BendCalibration(flat_steps=100, fold_steps=100 + QUARTER)
    assert min_deg(cal, model, joint) == pytest.approx(90.0)
    validate(cal, model, joint)


def test_apply_replaces_flat_and_min_angle():
    model, joint = robot()
    applied = apply_calibration(model, joint, BendCalibration(3900, 2900))
    assert applied.bend_flat_steps == 3900
    assert applied.bend_min_deg == pytest.approx(180 - 1000 * 360 / 4096)


@pytest.mark.parametrize(
    ("cal", "message"),
    [
        (BendCalibration(3000, 3000), "below flat"),
        (BendCalibration(3000, 3500), "swapped"),
        (BendCalibration(4000, 3990), "same point"),
        (BendCalibration(4096, 3000), r"0\.\.4095"),
        (BendCalibration(-1, 3000), r"0\.\.4095"),
        (BendCalibration(True, 3000), "whole step"),
        (BendCalibration(4000, 3000.0), "whole step"),
        # 270 deg of fold: past a closed hinge.
        (BendCalibration(4000, 4000 - 3 * QUARTER), "min angle"),
    ],
)
def test_bad_points_are_rejected(cal, message):
    model, joint = robot()
    with pytest.raises(CalibrationError, match=message):
        validate(cal, model, joint)


def test_good_points_pass():
    model, joint = robot()
    validate(BendCalibration(3000, 2000), model, joint)


def test_points_near_the_end_of_the_turn_warn():
    _, joint = robot()
    assert warnings(BendCalibration(4090, 3000), joint)
    assert warnings(BendCalibration(3000, 10), joint)
    assert warnings(BendCalibration(4000, 3000), joint) == []


def test_path_is_next_to_the_config(tmp_path, monkeypatch):
    monkeypatch.delenv(CALIBRATION_ENV_VAR, raising=False)
    config = tmp_path / "robot.yaml"
    assert calibration_path(config) == tmp_path / CALIBRATION_FILE


def test_env_var_overrides_the_path(tmp_path, monkeypatch):
    monkeypatch.setenv(CALIBRATION_ENV_VAR, str(tmp_path / "x.yaml"))
    assert calibration_path("config/robot.yaml") == tmp_path / "x.yaml"


def test_explicit_path_wins(tmp_path, monkeypatch):
    monkeypatch.setenv(CALIBRATION_ENV_VAR, str(tmp_path / "x.yaml"))
    explicit = tmp_path / "y.yaml"
    assert calibration_path("config/robot.yaml", explicit) == explicit


def test_save_then_load(tmp_path):
    path = tmp_path / "sub" / CALIBRATION_FILE
    cal = BendCalibration(4000, 2976)
    save_calibration(path, cal, 90.0)
    assert load_calibration(path) == cal
    # Written in one step: no temporary file left behind.
    assert [p.name for p in path.parent.iterdir()] == [CALIBRATION_FILE]


def test_save_replaces_an_old_file(tmp_path):
    path = tmp_path / CALIBRATION_FILE
    save_calibration(path, BendCalibration(4000, 3000), 92.1)
    save_calibration(path, BendCalibration(3900, 2900), 92.1)
    assert load_calibration(path) == BendCalibration(3900, 2900)


def test_missing_file_is_no_calibration(tmp_path):
    assert load_calibration(tmp_path / "none.yaml") is None


@pytest.mark.parametrize(
    "text",
    [
        "flat_steps: [",
        "- 1\n- 2\n",
        "flat_steps: 4000\n",
        "flat_steps: 4000\nfold_steps: 3000\nextra: 1\n",
        "flat_steps: 4000\nfold_steps: 3000.5\n",
        "flat_steps: yes\nfold_steps: 3000\n",
    ],
)
def test_bad_file_is_rejected(tmp_path, text):
    path = tmp_path / CALIBRATION_FILE
    path.write_text(text, encoding="utf-8")
    with pytest.raises(CalibrationError, match=str(path.name)):
        load_calibration(path)


def test_file_text_names_both_points_and_the_angle():
    text = calibration_yaml(BendCalibration(4000, 2976), 90.04)
    assert "flat_steps: 4000\n" in text
    assert "fold_steps: 2976\n" in text
    assert "90.0 deg" in text
    assert text.startswith("#")
