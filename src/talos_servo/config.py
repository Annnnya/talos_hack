"""Robot layout from `config/robot.yaml`: joints, their limits and the mecanum
base geometry."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path
from types import MappingProxyType

import yaml

from talos_servo.bend_calibration import (
    BendCalibration,
    CalibrationError,
    apply_calibration,
    calibration_path,
    load_calibration,
)
from talos_servo.bus import (
    MAX_POSITION,
    MAX_SERVO_ID,
    MAX_SPEED,
    RAD_PER_STEP,
    STEPS_PER_TURN,
    joint_position,
)
from talos_servo.defaults import DEFAULT_MOVE_SPEED
from talos_servo.hinge import Hinge, HingeRangeError
from talos_servo.model import alpha_per_servo_rad

SPEED = "speed"
POSITION = "position"
CONTROLS = (SPEED, POSITION)

# Low on purpose: raise it in the config once the robot is known to behave.
DEFAULT_MAX_SPEED = 1000
DEFAULT_TELEMETRY_HZ = 20.0
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_COMMAND_TIMEOUT_S = 0.5
DEFAULT_MAX_LINEAR_SPEED = 0.1
DEFAULT_MAX_ANGULAR_SPEED = 0.5
WHEEL_COUNT = 4
DEFAULT_SWITCH_TIMEOUT_S = 2.0
# Robot modes in `talos_servo.modes`; plain strings here so the config does
# not depend on the state machine.
MODE_NAMES = ("free", "aligning", "clamping", "climbing")
# Hinge opening angle when the robot lies flat.
FLAT_DEG = 180.0
# Expansion (#76): low on purpose, well below the hinge servo speed.
DEFAULT_EXPANSION_RATE_RAD_S = 0.15
# Low on purpose; the wheels top out near 0.15 m/s.
DEFAULT_CLIMB_SPEED_M_S = 0.05
DEFAULT_EXPANSION_HZ = 50.0
DEFAULT_EXPANSION_LEAD_RAD = 0.05

# Joint names are ROS names too (topics, services, JointState), so keep them
# plain.
_JOINT_NAME = re.compile(r"[a-z][a-z0-9_]*")


class ConfigError(ValueError):
    """The robot config file is invalid."""


@dataclass(frozen=True)
class JointConfig:
    name: str
    id: int
    control: str
    inverted: bool = False
    enabled: bool = True
    min_pos: int = 0
    max_pos: int = MAX_POSITION


@dataclass(frozen=True)
class BaseConfig:
    """Mecanum base.

    `wheels` names the speed joints in FL, FR, RL, RR order.
    """

    wheels: tuple[str, str, str, str]
    wheelbase_m: float
    track_width_m: float
    wheel_radius_m: float
    max_linear_speed: float = DEFAULT_MAX_LINEAR_SPEED
    max_angular_speed: float = DEFAULT_MAX_ANGULAR_SPEED

    @property
    def steps_per_meter(self) -> float:
        """Servo steps for one metre of wheel travel."""
        return STEPS_PER_TURN / (2 * math.pi * self.wheel_radius_m)


@dataclass(frozen=True)
class ModelConfig:
    """Body geometry shared with the URDF (`description/talos.urdf.xacro`).

    The `hinge_joint` (`bend`) position is the opening angle between the
    legs: 180 deg flat, smaller when folded. Each leg gear turns
    `leg_gear_ratio` times the servo.
    `leg_length_m` (hinge to wheel axle) is half the flat `base.wheelbase_m`.
    `bend_flat_steps` is the raw bend servo position when flat, fixed by
    aligning the servo horn to the leg. It is required while the hinge joint
    is enabled.
    """

    hinge_joint: str
    leg_length_m: float
    leg_gear_ratio: float
    bend_min_deg: float
    bend_max_deg: float = FLAT_DEG
    bend_flat_steps: int | None = None

    @property
    def bend_min_rad(self) -> float:
        return math.radians(self.bend_min_deg)

    @property
    def bend_max_rad(self) -> float:
        return math.radians(self.bend_max_deg)


@dataclass(frozen=True)
class ModesConfig:
    """ros2_control controllers that are active in each robot mode.

    The mode node deactivates the old mode's controllers and activates the
    new ones on every mode change.
    """

    controllers: Mapping[str, tuple[str, ...]]
    switch_timeout_s: float = DEFAULT_SWITCH_TIMEOUT_S
    # Replace the free mode's controllers while the hinge opens or closes.
    hinge_motion: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        # Read-only like the rest of the frozen config.
        object.__setattr__(
            self, "controllers", MappingProxyType(dict(self.controllers))
        )

    def for_mode(self, mode: str) -> tuple[str, ...]:
        return self.controllers.get(mode, ())

    @property
    def managed(self) -> frozenset[str]:
        """Every controller named by any mode."""
        return frozenset(
            c
            for names in (*self.controllers.values(), self.hinge_motion)
            for c in names
        )


@dataclass(frozen=True)
class ExpansionConfig:
    """Opening and closing the hinge (`talos_servo.expansion`).

    `max_rate_rad_s` caps the opening speed, `rate_hz` is how often the hinge
    goal and the wheel speeds are sent, `max_lead_rad` is how far the goal
    may run ahead of the measured angle and `command_timeout_s` stops a
    motion whose rate is not repeated.
    """

    max_rate_rad_s: float = DEFAULT_EXPANSION_RATE_RAD_S
    # Wheel surface speed cap (m/s) for climbing.
    max_climb_speed_m_s: float = DEFAULT_CLIMB_SPEED_M_S
    rate_hz: float = DEFAULT_EXPANSION_HZ
    max_lead_rad: float = DEFAULT_EXPANSION_LEAD_RAD
    command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S


@dataclass(frozen=True)
class HingeHoldConfig:
    """Holding the hinge at a current setpoint (`talos_servo.hinge_hold`).

    `max_current_a` caps the setpoint. `ki` (rad/s per A) and `kp` (rad per
    A) move the goal lead from the current error, `filter_s` smooths the
    measured current, `max_offset_rad` caps how far the goal may lead the
    measured angle and `no_contact_rad` is how far the hinge may open with
    hardly any current before the hold gives up.
    """

    max_current_a: float = 1.0
    ki: float = 0.3
    kp: float = 0.0
    filter_s: float = 0.1
    max_offset_rad: float = 0.25
    no_contact_rad: float = 0.3
    # Back the goal off above this servo load (%); the servo cuts its drive
    # after 2 s above 50 %.
    max_load_pct: float = 40.0


@dataclass(frozen=True)
class RobotConfig:
    joints: tuple[JointConfig, ...]
    base: BaseConfig | None = None
    modes: ModesConfig | None = None
    model: ModelConfig | None = None
    expansion: ExpansionConfig = field(default_factory=ExpansionConfig)
    hinge_hold: HingeHoldConfig = field(default_factory=HingeHoldConfig)
    max_speed: int = DEFAULT_MAX_SPEED
    telemetry_hz: float = DEFAULT_TELEMETRY_HZ
    control_hz: float = DEFAULT_CONTROL_HZ
    command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S
    # The hinge angle converter, built on load from model.bend_flat_steps;
    # None without one.
    hinge: Hinge | None = field(default=None, compare=False)
    # The saved bend calibration in use (#84), None when the model values
    # from robot.yaml apply.
    bend_calibration: BendCalibration | None = None

    @property
    def enabled_joints(self) -> tuple[JointConfig, ...]:
        return tuple(j for j in self.joints if j.enabled)


def parse_config(
    data: dict, calibration: BendCalibration | None = None
) -> RobotConfig:
    """The robot config from parsed YAML. `calibration` overrides the
    hinge flat position and min angle."""
    raw_joints = data.get("joints") or {}
    if not isinstance(raw_joints, dict) or not raw_joints:
        raise ConfigError("config needs at least one entry under 'joints'")
    joints = tuple(_parse_joint(name, raw) for name, raw in raw_joints.items())
    ids = [j.id for j in joints]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"duplicate servo ids in config: {ids}")

    max_speed = int(data.get("max_speed", DEFAULT_MAX_SPEED))
    if not 1 <= max_speed <= MAX_SPEED:
        raise ConfigError(
            f"max_speed must be in 1..{MAX_SPEED}, got {max_speed}"
        )
    config = RobotConfig(
        joints=joints,
        max_speed=max_speed,
        telemetry_hz=_positive(data, "telemetry_hz", DEFAULT_TELEMETRY_HZ),
        control_hz=_positive(data, "control_hz", DEFAULT_CONTROL_HZ),
        command_timeout_s=_positive(
            data, "command_timeout_s", DEFAULT_COMMAND_TIMEOUT_S
        ),
    )
    raw_base = data.get("base")
    if raw_base is not None:
        config = replace(
            config, base=_parse_base(raw_base, {j.name: j for j in joints})
        )
    raw_modes = data.get("modes")
    if raw_modes is not None:
        config = replace(config, modes=_parse_modes(raw_modes))
    raw_model = data.get("model")
    if raw_model is not None:
        config = replace(
            config,
            model=_parse_model(
                raw_model, config.base, {j.name: j for j in joints}
            ),
        )
        if calibration is not None:
            config = _calibrate(config, calibration)
        config = _clamp_hinge(config)
    elif calibration is not None:
        raise ConfigError("a bend calibration needs a model section")
    raw_hold = data.get("hinge_hold")
    if raw_hold is not None:
        config = replace(config, hinge_hold=_parse_hinge_hold(raw_hold))
    raw_expansion = data.get("expansion")
    if raw_expansion is not None:
        config = replace(
            config, expansion=_parse_expansion(raw_expansion, config.model)
        )
    return config


def _clamp_hinge(config: RobotConfig) -> RobotConfig:
    """Build the hinge and narrow its joint's min_pos/max_pos to the model
    bend range, so every move is clamped to it, whoever sends it."""
    model = config.model
    assert model is not None
    joint = next(j for j in config.joints if j.name == model.hinge_joint)
    if model.bend_flat_steps is None:
        if joint.enabled:
            raise ConfigError(
                f"model.bend_flat_steps is required while {joint.name} is "
                "enabled: the raw servo position when the robot lies flat"
            )
        return config
    flat = joint_position(model.bend_flat_steps, joint.inverted)
    try:
        hinge = Hinge(model, flat)
    except HingeRangeError as exc:
        raise ConfigError(f"model.bend_flat_steps: {exc}") from exc
    low, high = hinge.step_range
    min_pos, max_pos = max(joint.min_pos, low), min(joint.max_pos, high)
    if min_pos > max_pos:
        raise ConfigError(
            f"joints.{joint.name}: min_pos..max_pos {joint.min_pos}.."
            f"{joint.max_pos} does not overlap the hinge range {low}..{high}"
        )
    clamped = replace(joint, min_pos=min_pos, max_pos=max_pos)
    return replace(
        config,
        joints=tuple(clamped if j is joint else j for j in config.joints),
        hinge=hinge,
    )


def _calibrate(config: RobotConfig, cal: BendCalibration) -> RobotConfig:
    model = config.model
    assert model is not None
    joint = next(j for j in config.joints if j.name == model.hinge_joint)
    return replace(
        config,
        model=apply_calibration(model, joint, cal),
        bend_calibration=cal,
    )


def load_config(
    path: str | Path, calibration: str | Path | None = None
) -> RobotConfig:
    """The robot config at `path`, with the bend calibration from
    `calibration` (default: see `bend_calibration.calibration_path`) if
    that file exists."""
    data = read_config_data(path)
    cal_path = calibration_path(path, calibration)
    try:
        cal = load_calibration(cal_path)
        return parse_config(data, cal)
    except CalibrationError as exc:
        raise ConfigError(
            f"bend calibration {cal_path}: {exc}. Delete the file to fall "
            "back to the model values in the robot config, then run "
            "`talos-servo calibrate-bend` again"
        ) from exc


def read_config_data(path: str | Path) -> dict:
    """The robot config file as parsed YAML."""
    try:
        return yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except OSError as exc:
        raise ConfigError(f"could not read config {path}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"config {path} is not valid YAML: {exc}") from exc


def _parse_joint(name: str, raw: dict) -> JointConfig:
    if not _JOINT_NAME.fullmatch(str(name)):
        raise ConfigError(
            f"joint name {name!r} must be lowercase letters, digits and _, "
            "starting with a letter"
        )
    if not isinstance(raw, dict) or "id" not in raw or "control" not in raw:
        raise ConfigError(f"joint {name}: needs at least 'id' and 'control'")
    joint = JointConfig(
        name=str(name),
        id=int(raw["id"]),
        control=str(raw["control"]),
        inverted=_flag(name, raw, "inverted", False),
        enabled=_flag(name, raw, "enabled", True),
        min_pos=int(raw.get("min_pos", 0)),
        max_pos=int(raw.get("max_pos", MAX_POSITION)),
    )
    if joint.control not in CONTROLS:
        raise ConfigError(
            f"joint {name}: control must be one of {CONTROLS}, got "
            f"{joint.control!r}"
        )
    if not 0 <= joint.id <= MAX_SERVO_ID:
        raise ConfigError(
            f"joint {name}: id must be in 0..{MAX_SERVO_ID}, got {joint.id}"
        )
    if not 0 <= joint.min_pos <= joint.max_pos <= MAX_POSITION:
        raise ConfigError(
            f"joint {name}: need 0 <= min_pos <= max_pos <= {MAX_POSITION}, "
            f"got {joint.min_pos}..{joint.max_pos}"
        )
    return joint


def _parse_base(raw: dict, joints: dict[str, JointConfig]) -> BaseConfig:
    wheels = tuple(str(w) for w in raw.get("wheels") or ())
    if len(wheels) != WHEEL_COUNT or len(set(wheels)) != WHEEL_COUNT:
        raise ConfigError(
            f"base.wheels must name {WHEEL_COUNT} different joints (FL, FR, "
            "RL, RR)"
        )
    for wheel in wheels:
        joint = joints.get(wheel)
        if joint is None:
            raise ConfigError(f"base.wheels: {wheel!r} is not a joint")
        if joint.control != SPEED or not joint.enabled:
            raise ConfigError(
                f"base.wheels: {wheel} must be an enabled speed joint"
            )
    return BaseConfig(
        wheels=(wheels[0], wheels[1], wheels[2], wheels[3]),
        wheelbase_m=_positive(raw, "wheelbase_m"),
        track_width_m=_positive(raw, "track_width_m"),
        wheel_radius_m=_positive(raw, "wheel_radius_m"),
        max_linear_speed=_positive(
            raw, "max_linear_speed", DEFAULT_MAX_LINEAR_SPEED
        ),
        max_angular_speed=_positive(
            raw, "max_angular_speed", DEFAULT_MAX_ANGULAR_SPEED
        ),
    )


def _parse_modes(raw: dict) -> ModesConfig:
    if not isinstance(raw, dict):
        raise ConfigError("modes must be a mapping")
    raw_controllers = raw.get("controllers") or {}
    if not isinstance(raw_controllers, dict):
        raise ConfigError("modes.controllers must map a mode to a list")
    unknown = set(raw_controllers) - set(MODE_NAMES)
    if unknown:
        raise ConfigError(
            f"modes.controllers: unknown modes {sorted(unknown)}, expected "
            f"some of {MODE_NAMES}"
        )
    controllers: dict[str, tuple[str, ...]] = {}
    for mode, names in raw_controllers.items():
        names = [] if names is None else names
        if not isinstance(names, list) or not all(
            isinstance(n, str) and n for n in names
        ):
            raise ConfigError(
                f"modes.controllers.{mode} must be a list of controller names"
            )
        controllers[mode] = tuple(names)
    hinge_motion = raw.get("hinge_motion") or []
    if not isinstance(hinge_motion, list) or not all(
        isinstance(n, str) and n for n in hinge_motion
    ):
        raise ConfigError("modes.hinge_motion must be a list of controllers")
    return ModesConfig(
        controllers=controllers,
        switch_timeout_s=_positive(
            raw, "switch_timeout_s", DEFAULT_SWITCH_TIMEOUT_S
        ),
        hinge_motion=tuple(hinge_motion),
    )


def _parse_expansion(raw: dict, model: ModelConfig | None) -> ExpansionConfig:
    if not isinstance(raw, dict):
        raise ConfigError("expansion must be a mapping")
    try:
        expansion = ExpansionConfig(
            max_rate_rad_s=_positive(
                raw, "max_rate_rad_s", DEFAULT_EXPANSION_RATE_RAD_S
            ),
            max_climb_speed_m_s=_positive(
                raw, "max_climb_speed_m_s", DEFAULT_CLIMB_SPEED_M_S
            ),
            rate_hz=_positive(raw, "rate_hz", DEFAULT_EXPANSION_HZ),
            max_lead_rad=_positive(
                raw, "max_lead_rad", DEFAULT_EXPANSION_LEAD_RAD
            ),
            command_timeout_s=_positive(
                raw, "command_timeout_s", DEFAULT_COMMAND_TIMEOUT_S
            ),
        )
    except ConfigError as exc:
        raise ConfigError(f"expansion: {exc}") from exc
    if model is not None:
        # The bridge moves the hinge servo at DEFAULT_MOVE_SPEED: a faster
        # goal stream leaves the hinge behind and the wheels ahead.
        servo = (
            DEFAULT_MOVE_SPEED
            * RAD_PER_STEP
            * alpha_per_servo_rad(model.leg_gear_ratio)
        )
        if expansion.max_rate_rad_s >= servo:
            raise ConfigError(
                f"expansion.max_rate_rad_s must stay below the hinge servo "
                f"speed {servo:.3f} rad/s, got {expansion.max_rate_rad_s}"
            )
    return expansion


def _parse_hinge_hold(raw: dict) -> HingeHoldConfig:
    if not isinstance(raw, dict):
        raise ConfigError("hinge_hold must be a mapping")
    d = HingeHoldConfig()
    try:
        kp = float(raw.get("kp", d.kp))
        if not math.isfinite(kp) or kp < 0:
            raise ConfigError(f"kp must be 0 or more, got {kp}")
        return HingeHoldConfig(
            max_current_a=_positive(raw, "max_current_a", d.max_current_a),
            ki=_positive(raw, "ki", d.ki),
            kp=kp,
            filter_s=_positive(raw, "filter_s", d.filter_s),
            max_offset_rad=_positive(raw, "max_offset_rad", d.max_offset_rad),
            no_contact_rad=_positive(raw, "no_contact_rad", d.no_contact_rad),
            max_load_pct=_positive(raw, "max_load_pct", d.max_load_pct),
        )
    except ConfigError as exc:
        raise ConfigError(f"hinge_hold: {exc}") from exc


def _parse_model(
    raw: dict, base: BaseConfig | None, joints: dict[str, JointConfig]
) -> ModelConfig:
    if base is None:
        raise ConfigError(
            "model needs a base section: the legs and wheels come from it"
        )
    hinge = str(raw.get("hinge_joint", ""))
    joint = joints.get(hinge)
    if joint is None or joint.control != POSITION:
        raise ConfigError(
            f"model.hinge_joint: {hinge!r} must name a position joint"
        )
    model = ModelConfig(
        hinge_joint=hinge,
        leg_length_m=base.wheelbase_m / 2,
        leg_gear_ratio=_positive(raw, "leg_gear_ratio"),
        bend_min_deg=_positive(raw, "bend_min_deg"),
        bend_max_deg=_positive(raw, "bend_max_deg", FLAT_DEG),
    )
    if not model.bend_min_deg < model.bend_max_deg <= FLAT_DEG:
        raise ConfigError(
            "model: need 0 < bend_min_deg < bend_max_deg <= "
            f"{FLAT_DEG:g}, got {model.bend_min_deg}..{model.bend_max_deg}"
        )
    flat = raw.get("bend_flat_steps")
    if flat is not None:
        if isinstance(flat, bool) or not isinstance(flat, int):
            raise ConfigError(
                f"model.bend_flat_steps must be a whole step, got {flat!r}"
            )
        if not 0 <= flat <= MAX_POSITION:
            raise ConfigError(
                f"model.bend_flat_steps must be in 0..{MAX_POSITION}, "
                f"got {flat}"
            )
        model = replace(model, bend_flat_steps=flat)
    return model


def _positive(raw: dict, key: str, default: float | None = None) -> float:
    if key not in raw and default is None:
        raise ConfigError(f"missing {key!r}")
    value = float(raw.get(key, default))
    if not math.isfinite(value) or value <= 0:
        raise ConfigError(f"{key} must be a positive number, got {value}")
    return value


def _flag(joint: str, raw: dict, key: str, default: bool) -> bool:
    value = raw.get(key, default)
    if not isinstance(value, bool):
        raise ConfigError(
            f"joint {joint}: {key} must be true or false, got {value!r}"
        )
    return value
