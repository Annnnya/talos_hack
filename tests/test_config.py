from __future__ import annotations

import copy

import pytest
from conftest import BASE, SMALL

from talos_servo.config import (
    ConfigError,
    parse_config,
)


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
