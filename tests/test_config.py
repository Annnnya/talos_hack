from __future__ import annotations

import copy
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
