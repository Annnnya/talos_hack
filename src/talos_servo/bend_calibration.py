"""Bend hinge calibration (#84): the two measured points and their file.

`talos-servo calibrate-bend` records the raw bend servo position when the
robot lies flat (180 deg) and at max fold (front and rear wheels touching),
and saves both to `config/bend_calibration.yaml` next to the robot config.
No EEPROM is written.

When the file exists it overrides `model.bend_flat_steps` and
`model.bend_min_deg`: flat is the flat point, and the min angle is the
opening angle at the fold point, through the gear ratio. Without the file
the robot config values stay in use.

The session that jogs the hinge and records the points is
`talos_servo.bend_calibrator`. No ROS here, and no `Robot` either, so
`talos_servo.config` can import this.
"""

from __future__ import annotations

import math
import os
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import TYPE_CHECKING

import yaml

from talos_servo.bus import MAX_POSITION, joint_position
from talos_servo.hinge import (
    FLAT_RAD,
    READ_MARGIN_STEPS,
    Hinge,
    HingeRangeError,
    alpha_at,
)

if TYPE_CHECKING:
    from talos_servo.config import JointConfig, ModelConfig

CALIBRATION_FILE = "bend_calibration.yaml"
# Overrides where the file is looked for, e.g. to keep tests away from a
# local calibration.
CALIBRATION_ENV_VAR = "TALOS_BEND_CALIBRATION"
# Fold and flat closer than this are the same point recorded twice.
MIN_FOLD_SPAN_DEG = 5.0
_FLAT_DEG = math.degrees(FLAT_RAD)
_KEYS = ("flat_steps", "fold_steps")


class CalibrationError(ValueError):
    """The calibration is invalid or its file cannot be read."""


@dataclass(frozen=True)
class BendCalibration:
    """Raw bend servo positions (0-4095, as `talos-servo monitor` shows
    them) when flat and at max fold."""

    flat_steps: int
    fold_steps: int


def calibration_path(
    config_path: str | Path, calibration: str | Path | None = None
) -> Path:
    """Where the calibration lives: `calibration` if given, else
    $TALOS_BEND_CALIBRATION, else next to the robot config."""
    if calibration is not None:
        return Path(calibration)
    env = os.environ.get(CALIBRATION_ENV_VAR)
    if env:
        return Path(env)
    return Path(config_path).parent / CALIBRATION_FILE


def min_deg(
    cal: BendCalibration, model: ModelConfig, joint: JointConfig
) -> float:
    """Opening angle (deg) at the fold point."""
    flat = joint_position(cal.flat_steps, joint.inverted)
    fold = joint_position(cal.fold_steps, joint.inverted)
    return math.degrees(alpha_at(fold, flat, model.leg_gear_ratio))


def apply_calibration(
    model: ModelConfig, joint: JointConfig, cal: BendCalibration
) -> ModelConfig:
    """`model` with flat and the min angle from the calibration. Checks the
    calibration first."""
    validate(cal, model, joint)
    return _applied(model, joint, cal)


def validate(
    cal: BendCalibration, model: ModelConfig, joint: JointConfig
) -> None:
    """Raise CalibrationError unless the two points make a usable hinge."""
    for key in _KEYS:
        value = getattr(cal, key)
        if isinstance(value, bool) or not isinstance(value, int):
            raise CalibrationError(f"{key} must be a whole step, got {value!r}")
        if not 0 <= value <= MAX_POSITION:
            raise CalibrationError(
                f"{key} must be in 0..{MAX_POSITION}, got {value}"
            )
    flat = joint_position(cal.flat_steps, joint.inverted)
    fold = joint_position(cal.fold_steps, joint.inverted)
    if fold >= flat:
        raise CalibrationError(
            f"max fold (raw {cal.fold_steps}) must sit below flat (raw "
            f"{cal.flat_steps}) in joint steps. The points are swapped, or "
            f"joints.{joint.name}.inverted is wrong"
        )
    fold_deg = min_deg(cal, model, joint)
    if _FLAT_DEG - fold_deg < MIN_FOLD_SPAN_DEG:
        raise CalibrationError(
            f"max fold is only {_FLAT_DEG - fold_deg:.1f} deg from flat, at "
            f"least {MIN_FOLD_SPAN_DEG:g} deg needed: the same point "
            "recorded twice?"
        )
    if not 0.0 < fold_deg < model.bend_max_deg:
        raise CalibrationError(
            f"max fold gives a min angle of {fold_deg:.1f} deg, it must be "
            f"in 0..{model.bend_max_deg:g} deg"
        )
    try:
        Hinge(_applied(model, joint, cal), flat)
    except HingeRangeError as exc:
        raise CalibrationError(str(exc)) from exc


def warnings(cal: BendCalibration, joint: JointConfig) -> list[str]:
    """Points so close to the end of the servo turn that a small overshoot
    reads as a wrap."""
    flat = joint_position(cal.flat_steps, joint.inverted)
    fold = joint_position(cal.fold_steps, joint.inverted)
    found = []
    if flat > MAX_POSITION - READ_MARGIN_STEPS:
        found.append(
            f"flat is {MAX_POSITION - flat} steps from the end of the servo "
            "turn: turn the servo horn so flat sits further from it"
        )
    if fold < READ_MARGIN_STEPS:
        found.append(
            f"max fold is {fold} steps from the start of the servo turn: "
            "turn the servo horn so it sits further from it"
        )
    return found


def load_calibration(path: str | Path) -> BendCalibration | None:
    """The saved calibration, or None if there is no file."""
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as exc:
        raise CalibrationError(f"could not read {path}: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"{path} is not valid YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise CalibrationError(f"{path} must hold {', '.join(_KEYS)}")
    missing = [k for k in _KEYS if k not in data]
    unknown = sorted(str(k) for k in data if k not in _KEYS)
    if missing or unknown:
        raise CalibrationError(
            f"{path} must hold exactly {', '.join(_KEYS)} "
            f"(missing {missing}, unknown {unknown})"
        )
    for key in _KEYS:
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, int):
            raise CalibrationError(
                f"{path}: {key} must be a whole step, got {value!r}"
            )
    return BendCalibration(data["flat_steps"], data["fold_steps"])


def calibration_yaml(cal: BendCalibration, fold_deg: float) -> str:
    """The calibration file text."""
    return (
        "# Bend hinge calibration, written by `talos-servo calibrate-bend`\n"
        "# (#84). Raw servo positions (0-4095, as `talos-servo monitor`\n"
        "# shows them). Overrides model.bend_flat_steps and bend_min_deg in\n"
        "# robot.yaml; delete this file to fall back to them.\n"
        f"# Max fold is {fold_deg:.1f} deg.\n"
        f"flat_steps: {cal.flat_steps}\n"
        f"fold_steps: {cal.fold_steps}\n"
    )


def save_calibration(
    path: str | Path, cal: BendCalibration, fold_deg: float
) -> None:
    """Write the file in one step, so a crash never leaves half of it.
    Raises CalibrationError when it cannot be written."""
    path = Path(path)
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(
            dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
        )
    except OSError as exc:
        raise CalibrationError(f"could not write {path}: {exc}") from exc
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(calibration_yaml(cal, fold_deg))
        os.replace(tmp, path)
    except BaseException as exc:
        Path(tmp).unlink(missing_ok=True)
        if isinstance(exc, OSError):
            raise CalibrationError(f"could not write {path}: {exc}") from exc
        raise


def _applied(
    model: ModelConfig, joint: JointConfig, cal: BendCalibration
) -> ModelConfig:
    return replace(
        model,
        bend_flat_steps=cal.flat_steps,
        bend_min_deg=min_deg(cal, model, joint),
    )
