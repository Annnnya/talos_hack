from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
from conftest import BASE, SMALL

from talos_servo.config import (
    MODE_NAMES,
    ConfigError,
    ModesConfig,
    load_config,
    parse_config,
)
from talos_servo.modes import Mode

ROBOT_CONFIG = Path(__file__).parent.parent / "config" / "robot.yaml"


def with_changes(data: dict, **joint_changes: dict) -> dict:
    changed = copy.deepcopy(data)
    for joint, fields in joint_changes.items():
        changed["joints"][joint].update(fields)
    return changed


def test_joint_defaults():
    joint = parse_config(SMALL).joints[1]
    assert (joint.inverted, joint.enabled) == (False, True)
    assert (joint.min_pos, joint.max_pos) == (0, 4095)


@pytest.mark.parametrize(
    "data",
    [
        {"joints": {}},
        with_changes(SMALL, wheel={"id": 1}),
        with_changes(SMALL, bend={"min_pos": 3000, "max_pos": 1000}),
        with_changes(SMALL, wheel={"control": "wheel"}),
        with_changes(SMALL, wheel={"inverted": "yes"}),
        with_changes(SMALL, wheel={"id": 254}),
        {**SMALL, "max_speed": 0},
        {**SMALL, "control_hz": 0},
        {"joints": {"Front Left": {"id": 1, "control": "speed"}}},
        {"joints": {"wheel": {"id": 1}}},
    ],
)
def test_invalid_config_is_rejected(data):
    with pytest.raises(ConfigError):
        parse_config(data)


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"wheels": ["fl", "fr", "rl"]}, "4 different joints"),
        ({"wheels": ["fl", "fr", "rl", "nope"]}, "not a joint"),
        ({"wheels": ["fl", "fr", "rl", "bend"]}, "enabled speed joint"),
        ({"wheelbase_m": -1}, "positive"),
    ],
)
def test_invalid_base_is_rejected(change, message):
    data = copy.deepcopy(BASE)
    data["base"].update(change)
    with pytest.raises(ConfigError, match=message):
        parse_config(data)


def test_base_is_optional():
    assert parse_config(SMALL).base is None


def test_modes_are_optional():
    assert parse_config(SMALL).modes is None


def test_modes_parse_controllers_per_mode():
    modes = parse_config(
        {
            **SMALL,
            "modes": {
                "controllers": {"free": ["drive"], "clamping": None},
                "switch_timeout_s": 1.5,
            },
        }
    ).modes
    assert modes is not None
    assert modes.for_mode("free") == ("drive",)
    assert modes.for_mode("clamping") == ()
    assert modes.for_mode("climbing") == ()
    assert modes.switch_timeout_s == 1.5


def test_mode_names_match_the_state_machine():
    assert set(MODE_NAMES) == {m.value for m in Mode}


def test_mode_controllers_are_read_only():
    modes = ModesConfig(controllers={"free": ("drive",)})
    with pytest.raises(TypeError):
        modes.controllers["free"] = ("wall",)  # type: ignore[index]
    assert modes.managed == {"drive"}


@pytest.mark.parametrize(
    ("modes", "message"),
    [
        ([], "mapping"),
        ({"controllers": ["drive"]}, "map a mode"),
        ({"controllers": {"flying": ["drive"]}}, "unknown modes"),
        ({"controllers": {"free": "drive"}}, "list of controller names"),
        ({"controllers": {"free": [""]}}, "list of controller names"),
        ({"switch_timeout_s": 0}, "positive"),
    ],
)
def test_invalid_modes_are_rejected(modes, message):
    with pytest.raises(ConfigError, match=message):
        parse_config({**SMALL, "modes": modes})


def test_repo_config_loads_modes():
    config = load_config(
        Path(__file__).resolve().parents[1] / "config" / "robot.yaml"
    )
    assert config.modes is not None
    assert config.modes.for_mode("free")


MODEL = {
    "hinge_joint": "bend",
    "leg_gear_ratio": 0.5,
    "bend_min_deg": 90,
    "bend_max_deg": 180,
}


def test_model_is_parsed():
    model = parse_config({**BASE, "model": MODEL}).model
    assert model.leg_length_m == 0.15
    assert model.bend_min_rad == pytest.approx(math.pi / 2)
    assert model.bend_max_rad == pytest.approx(math.pi)


def test_model_needs_a_base():
    with pytest.raises(ConfigError, match="base section"):
        parse_config({**SMALL, "model": MODEL})


def test_model_is_optional():
    assert parse_config(BASE).model is None


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"bend_min_deg": 180}, "bend_min_deg < bend_max_deg"),
        ({"bend_max_deg": 200}, "bend_max_deg <= 180"),
        ({"leg_gear_ratio": 0}, "positive"),
        ({"hinge_joint": "nope"}, "hinge_joint"),
        ({"hinge_joint": "fl"}, "position joint"),
        ({"bend_flat_steps": 2048.5}, "whole step"),
        ({"bend_flat_steps": True}, "whole step"),
        # 90 deg of fold needs 1024 steps below flat.
        ({"bend_flat_steps": 1000}, "outside one servo turn"),
        ({"bend_flat_steps": 4096}, r"0\.\.4095"),
    ],
)
def test_invalid_model_is_rejected(change, message):
    with pytest.raises(ConfigError, match=message):
        parse_config({**BASE, "model": {**MODEL, **change}})


def hinge_data(bend: dict | None = None, **model_changes) -> dict:
    data = with_changes(BASE, bend={"enabled": True, **(bend or {})})
    return {**data, "model": {**MODEL, **model_changes}}


def bend_joint(data: dict):
    return next(j for j in parse_config(data).joints if j.name == "bend")


def test_enabled_hinge_needs_a_flat_position():
    with pytest.raises(ConfigError, match="bend_flat_steps is required"):
        parse_config(hinge_data())


def test_disabled_hinge_needs_no_flat_position():
    assert parse_config({**BASE, "model": MODEL}).hinge is None


def test_bend_limits_narrow_to_the_hinge_range():
    # 90 deg of fold is 1024 steps below flat (1:1 with the servo).
    joint = bend_joint(hinge_data(bend_flat_steps=4095))
    assert (joint.min_pos, joint.max_pos) == (3071, 4095)


def test_tighter_bend_limits_are_kept():
    data = hinge_data({"min_pos": 3500, "max_pos": 4000}, bend_flat_steps=4095)
    joint = bend_joint(data)
    assert (joint.min_pos, joint.max_pos) == (3500, 4000)


def test_bend_limits_outside_the_hinge_range_are_rejected():
    data = hinge_data({"min_pos": 0, "max_pos": 2000}, bend_flat_steps=4095)
    with pytest.raises(ConfigError, match="does not overlap"):
        parse_config(data)


def test_inverted_hinge_flat_is_a_raw_servo_position():
    # Raw 0 on an inverted servo is joint step 4095.
    data = hinge_data({"inverted": True}, bend_flat_steps=0)
    assert parse_config(data).hinge.flat_steps == 4095
    joint = bend_joint(data)
    assert (joint.min_pos, joint.max_pos) == (3071, 4095)


def test_repo_config_is_valid():
    config = load_config(ROBOT_CONFIG)
    assert config.model is not None
    assert config.base is not None


def test_repo_config_clamps_bend_to_the_hinge_range():
    config = load_config(ROBOT_CONFIG)
    hinge = config.hinge
    assert hinge is not None
    bend = next(j for j in config.joints if j.name == hinge.joint)
    assert (bend.min_pos, bend.max_pos) == hinge.step_range


def test_steps_per_meter_follow_the_wheel_radius():
    data = copy.deepcopy(BASE)
    data["base"]["wheel_radius_m"] = 0.03
    base = parse_config(data).base
    assert base.steps_per_meter == pytest.approx(4096 / (2 * math.pi * 0.03))


def test_wheel_radius_is_required():
    data = copy.deepcopy(BASE)
    del data["base"]["wheel_radius_m"]
    with pytest.raises(ConfigError, match="wheel_radius_m"):
        parse_config(data)


def test_modes_parse_hinge_motion_controllers():
    modes = parse_config(
        {
            **SMALL,
            "modes": {
                "controllers": {"free": ["drive"]},
                "hinge_motion": ["wheels"],
            },
        }
    ).modes
    assert modes is not None
    assert modes.hinge_motion == ("wheels",)
    assert modes.managed == {"drive", "wheels"}


def test_invalid_hinge_motion_is_rejected():
    with pytest.raises(ConfigError, match="hinge_motion"):
        parse_config({**SMALL, "modes": {"hinge_motion": "wheels"}})


def test_expansion_defaults_without_a_section():
    expansion = parse_config(SMALL).expansion
    assert 0 < expansion.max_rate_rad_s < 0.3
    assert expansion.command_timeout_s == 0.5


def test_expansion_is_parsed():
    expansion = parse_config(
        {**hinge_data(bend_flat_steps=4000), "expansion": {"rate_hz": 25}}
    ).expansion
    assert expansion.rate_hz == 25


@pytest.mark.parametrize(
    ("expansion", "message"),
    [
        ([], "mapping"),
        ({"max_rate_rad_s": 0}, "positive"),
        ({"max_lead_rad": -1}, "positive"),
        # The bridge moves the hinge servo at about 0.31 rad/s.
        ({"max_rate_rad_s": 0.5}, "below the hinge servo speed"),
    ],
)
def test_invalid_expansion_is_rejected(expansion, message):
    data = {**hinge_data(bend_flat_steps=4000), "expansion": expansion}
    with pytest.raises(ConfigError, match=message):
        parse_config(data)


def test_repo_config_moves_the_hinge_in_clamping():
    config = load_config(ROBOT_CONFIG)
    assert config.modes is not None
    assert config.modes.hinge_motion
    assert set(config.modes.hinge_motion) <= set(
        config.modes.for_mode("clamping")
    )
