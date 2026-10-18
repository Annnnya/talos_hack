"""Sanity checks for config/twist_mux.yaml.

A typo in a topic or priority still loads in twist_mux, but then one interface
silently never drives, or STOP does not mask it.
"""

from __future__ import annotations

from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

from talos_servo.ros.mux_status import (  # noqa: E402
    LOCKS_PREFIX,
    TOPICS_PREFIX,
    MuxEntry,
    parse_entries,
)

MUX_CONFIG = Path(__file__).resolve().parents[1] / "config" / "twist_mux.yaml"
# Long enough to ride out jitter between key repeats or web messages, short
# enough that a quiet input hands over to the other one quickly.
MAX_INPUT_TIMEOUT_S = 1.0


def flatten(data: dict, prefix: str = "") -> dict:
    flat = {}
    for key, value in data.items():
        name = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(flatten(value, f"{name}."))
        else:
            flat[name] = value
    return flat


@pytest.fixture(scope="module")
def data() -> dict:
    return yaml.safe_load(MUX_CONFIG.read_text())


@pytest.fixture(scope="module")
def params(data) -> dict:
    return flatten(data["/**"]["ros__parameters"])


@pytest.fixture(scope="module")
def inputs(params) -> dict[str, MuxEntry]:
    return {e.name: e for e in parse_entries(params, TOPICS_PREFIX)}


@pytest.fixture(scope="module")
def locks(params) -> dict[str, MuxEntry]:
    return {e.name: e for e in parse_entries(params, LOCKS_PREFIX)}


def test_plain_twist_like_the_robot_node(params):
    assert params["use_stamped"] is False


def test_web_and_keyboard_inputs(inputs):
    assert {name: e.topic for name, e in inputs.items()} == {
        "keyboard": "/keyboard/cmd_vel",
        "web": "/web/cmd_vel",
    }


def test_keyboard_overrides_web(inputs):
    assert inputs["keyboard"].priority > inputs["web"].priority


def test_priorities_are_distinct(inputs):
    # Equal priorities in twist_mux pick by list order, not the latest input.
    priorities = [e.priority for e in inputs.values()]
    assert len(set(priorities)) == len(priorities)


@pytest.mark.parametrize("name", ["keyboard", "web"])
def test_quiet_input_times_out_quickly(inputs, name):
    assert 0 < inputs[name].timeout_s <= MAX_INPUT_TIMEOUT_S


def test_stop_lock_masks_every_input(inputs, locks):
    stop = locks["stop"]
    assert stop.topic == "/robot/stop_lock"
    assert all(stop.priority > e.priority for e in inputs.values())


def test_stop_lock_holds_until_released(locks):
    # With a timeout the lock would also turn on whenever nobody publishes.
    assert locks["stop"].timeout_s == 0


def test_status_node_settings(data):
    status = data["mux_status"]["ros__parameters"]
    assert status["active_input_topic"] == "/robot/active_input"
    assert status["rate_hz"] > 0
