from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
from conftest import BASE, SMALL

from talos_servo.config import (
    ConfigError,
    load_config,
    parse_config,
)

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


MODEL = {
    "wheel_radius_m": 0.03,
    "leg_length_m": 0.15,
    "leg_gear_ratio": 0.5,
    "bend_min_deg": 90,
    "bend_max_deg": 180,
}


def test_model_is_parsed():
    model = parse_config({**BASE, "model": MODEL}).model
    assert model.leg_length_m == 0.15
    assert model.bend_min_rad == pytest.approx(math.pi / 2)
    assert model.bend_max_rad == pytest.approx(math.pi)


def test_model_is_optional():
    assert parse_config(BASE).model is None


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ({"leg_length_m": 0.2}, "base.wheelbase_m"),
        ({"bend_min_deg": 180}, "bend_min_deg < bend_max_deg"),
        ({"bend_max_deg": 200}, "bend_max_deg <= 180"),
        ({"wheel_radius_m": 0}, "positive"),
    ],
)
def test_invalid_model_is_rejected(change, message):
    with pytest.raises(ConfigError, match=message):
        parse_config({**BASE, "model": {**MODEL, **change}})


def test_repo_config_is_valid():
    config = load_config(ROBOT_CONFIG)
    assert config.model is not None
    assert config.base is not None
