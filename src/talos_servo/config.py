"""Robot layout from `config/robot.yaml`: joints, their limits and the mecanum
base geometry."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from types import MappingProxyType

import yaml

from talos_servo.bus import (
    MAX_POSITION,
    MAX_SERVO_ID,
    MAX_SPEED,
    STEPS_PER_TURN,
)
from talos_servo.hinge import Hinge, HingeRangeError

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
    `bend_flat_steps` is the hinge servo position when flat, from the
    calibration (#84); None means the hinge is flat when the robot starts.
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
            c for names in self.controllers.values() for c in names
        )


@dataclass(frozen=True)
class RobotConfig:
    joints: tuple[JointConfig, ...]
    base: BaseConfig | None = None
    modes: ModesConfig | None = None
    model: ModelConfig | None = None
    max_speed: int = DEFAULT_MAX_SPEED
    telemetry_hz: float = DEFAULT_TELEMETRY_HZ
    control_hz: float = DEFAULT_CONTROL_HZ
    command_timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S

    @property
    def enabled_joints(self) -> tuple[JointConfig, ...]:
        return tuple(j for j in self.joints if j.enabled)


def parse_config(data: dict) -> RobotConfig:
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
    return config


def load_config(path: str | Path) -> RobotConfig:
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except OSError as exc:
        raise ConfigError(f"could not read config {path}: {exc}") from exc
    return parse_config(data)


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
    return ModesConfig(
        controllers=controllers,
        switch_timeout_s=_positive(
            raw, "switch_timeout_s", DEFAULT_SWITCH_TIMEOUT_S
        ),
    )


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
        model = replace(model, bend_flat_steps=flat)
        try:
            Hinge(model, flat)
        except HingeRangeError as exc:
            raise ConfigError(f"model.bend_flat_steps: {exc}") from exc
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
