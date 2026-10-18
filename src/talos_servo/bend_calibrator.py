"""Bend hinge calibration session on a `Robot` (#84).

`start` lets the hinge move over the whole servo turn, since its real range
is what is being measured. `jog` steps it slowly open or folded,
`record_flat` and `record_fold` take the two points, and `save` writes
`config/bend_calibration.yaml` (see `talos_servo.bend_calibration`) and
applies the new hinge to the running robot. `cancel` puts the old limits
back without moving anything.

Every method returns a line for the user. No ROS here: the hardware bridge
serves these as services and `talos-servo calibrate-bend` calls them.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

from talos_servo.bend_calibration import (
    BendCalibration,
    calibration_path,
    save_calibration,
    warnings,
)
from talos_servo.bus import MAX_POSITION, RAD_PER_STEP, joint_position
from talos_servo.config import (
    JointConfig,
    RobotConfig,
    parse_config,
    read_config_data,
)
from talos_servo.hinge import READ_MARGIN_STEPS
from talos_servo.model import alpha_per_servo_rad
from talos_servo.robot import SOURCE_ROBOT, CommandError, Robot, clamp

# One key press opens or folds the hinge by this much.
JOG_DEG = 1.0
# Servo speed in steps/s while calibrating: about 5 deg/s at the hinge.
CALIBRATION_SPEED = 60
# The jog goal never runs more than this many jog steps ahead of the
# hinge, so a held key stops close to where it is released.
JOG_LEAD = 2
# With no call for this long the session ends by itself, e.g. when the CLI
# was killed.
IDLE_TIMEOUT_S = 10.0
OPEN = 1
FOLD = -1


class BendCalibrator:
    def __init__(
        self,
        robot: Robot,
        *,
        config_path: str | Path,
        calibration: str | Path | None = None,
        source: str = SOURCE_ROBOT,
        clock: Callable[[], float] | None = None,
    ) -> None:
        model = robot.config.model
        if model is None or model.hinge_joint not in robot.joints:
            raise CommandError("no enabled hinge to calibrate")
        self.robot = robot
        self.joint = model.hinge_joint
        self.config_path = Path(config_path)
        self.path = calibration_path(config_path, calibration)
        self._source = source
        self._clock = clock or robot.clock
        # robot.yaml as it was at start. Save applies the calibration to
        # this, so later edits to the file never reach the running robot.
        self._data = read_config_data(self.config_path)
        joint = _joint_config(robot.config, self.joint)
        self._id, self._inverted = joint.id, joint.inverted
        self.jog_steps = max(
            1,
            round(
                math.radians(JOG_DEG)
                / (RAD_PER_STEP * alpha_per_servo_rad(model.leg_gear_ratio))
            ),
        )
        self.active = False
        self._target: int | None = None
        self._flat: int | None = None
        self._fold: int | None = None
        self._last_call = 0.0

    # --- session ---------------------------------------------------------
    def start(self) -> str:
        if self.active:
            raise CommandError("calibration already running: cancel it first")
        position, _ = self.robot.read_position(self.joint)
        self.robot.set_position_limits(self.joint, 0, MAX_POSITION)
        self.active = True
        self._target = position
        self._flat = self._fold = None
        self._touch()
        return f"calibrating {self.joint}, at raw {self._raw(position)}"

    def jog(self, direction: int) -> str:
        """Step the hinge open (OPEN) or folded (FOLD) by one jog step."""
        self._check_active()
        if direction not in (OPEN, FOLD):
            raise CommandError(f"direction must be {OPEN} or {FOLD}")
        position, _ = self.robot.read_position(self.joint)
        lead = JOG_LEAD * self.jog_steps
        # Go on from the last goal while the hinge keeps up with it. A goal
        # far from the hinge (pushed by hand) or on the other side of it
        # (stalled against a stop, then reversed) restarts from the hinge,
        # so a key never moves it the other way.
        base = self._target
        if (
            base is None
            or abs(base - position) > lead
            or (base - position) * direction < 0
        ):
            base = position
        target = clamp(
            base + direction * self.jog_steps,
            position - lead,
            position + lead,
        )
        target = clamp(target, 0, MAX_POSITION)
        self.robot.move(
            self.joint, target, CALIBRATION_SPEED, source=self._source
        )
        self._target = target
        return f"{self.joint} at raw {self._raw(position)}"

    def record_flat(self) -> str:
        self._flat = self._still_raw()
        return f"flat at raw {self._flat}"

    def record_fold(self) -> str:
        self._fold = self._still_raw()
        return f"max fold at raw {self._fold}"

    def status(self) -> str:
        self._check_active()
        position, _ = self.robot.read_position(self.joint)
        return (
            f"{self.joint} at raw {self._raw(position)}; flat "
            f"{_point(self._flat)}, max fold {_point(self._fold)}"
        )

    def save(self) -> str:
        """Check the two points, write the file and use them from now on."""
        self._check_active()
        if self._flat is None or self._fold is None:
            raise CommandError("record flat and max fold before saving")
        cal = BendCalibration(self._flat, self._fold)
        new = parse_config(self._data, cal)
        if new.model is None or new.hinge is None:
            raise CommandError(f"{self.config_path} has no hinge model")
        joint = _joint_config(new, self.joint)
        if (joint.id, joint.inverted) != (self._id, self._inverted):
            # The points were measured with the servo the robot started
            # with; they mean something else for another one.
            raise CommandError(
                f"joints.{self.joint} changed in {self.config_path} since "
                "start: restart the robot and calibrate again"
            )
        save_calibration(self.path, cal, new.model.bend_min_deg)
        self.robot.update_hinge(new)
        self._end()
        low, high = new.hinge.step_range
        notes = warnings(cal, joint)
        notes += self._outside_note(new)
        return " ".join(
            [
                f"saved to {self.path}: {self.joint} "
                f"{new.model.bend_min_deg:.1f}.."
                f"{new.model.bend_max_deg:g} deg, steps {low}..{high}.",
                *notes,
            ]
        )

    def cancel(self) -> str:
        """End the session with the old limits. Nothing moves."""
        self._check_active()
        self.robot.reset_position_limits(self.joint)
        self._end()
        return " ".join(
            ["calibration cancelled", *self._outside_note(self.robot.config)]
        )

    def expired(self, now: float) -> bool:
        """True when a running session got no call for IDLE_TIMEOUT_S."""
        return self.active and now - self._last_call > IDLE_TIMEOUT_S

    # --- helpers ---------------------------------------------------------
    def _check_active(self) -> None:
        if not self.active:
            raise CommandError("no calibration running: start one first")
        self._touch()

    def _touch(self) -> None:
        self._last_call = self._clock()

    def _end(self) -> None:
        self.active = False
        self._target = None

    def _still_raw(self) -> int:
        self._check_active()
        position, moving = self.robot.read_position(self.joint)
        if moving:
            raise CommandError(
                f"{self.joint} is still moving, record once it stops"
            )
        return self._raw(position)

    def _raw(self, steps: int) -> int:
        return joint_position(steps, self._inverted)

    def _outside_note(self, config: RobotConfig) -> list[str]:
        """A warning when the hinge sits where the bridge cannot read it as
        an angle, so controller goals would be dropped."""
        hinge = config.hinge
        if hinge is None:
            return []
        position = self.robot.joints[self.joint].position
        low, high = hinge.step_range
        if low - READ_MARGIN_STEPS <= position <= high + READ_MARGIN_STEPS:
            return []
        return [
            f"Note: the hinge is at step {position}, outside its range "
            f"{low}..{high}, so its angle reads as unknown. Jog it back "
            "with `talos-servo calibrate-bend`."
        ]


def _point(raw: int | None) -> str:
    return "not recorded" if raw is None else f"raw {raw}"


def _joint_config(config: RobotConfig, name: str) -> JointConfig:
    joint = next((j for j in config.joints if j.name == name), None)
    if joint is None:
        raise CommandError(f"no joint {name} in the robot config")
    return joint
