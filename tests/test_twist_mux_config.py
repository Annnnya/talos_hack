"""Sanity checks for config/twist_mux.yaml.

A typo in a topic or priority still loads in twist_mux, but then one interface
silently never drives, or STOP does not mask it.
"""

from __future__ import annotations

import pytest
from conftest import MUX_CONFIG, flatten

yaml = pytest.importorskip("yaml")

from talos_servo.defaults import (  # noqa: E402
    KEYBOARD_CMD_VEL_TOPIC,
    STOP_LOCK_PERIOD_S,
    STOP_LOCK_TIMEOUT_S,
    WEB_CMD_VEL_TOPIC,
)
from talos_servo.ros.mux_status import (  # noqa: E402
    LOCKS_PREFIX,
    TOPICS_PREFIX,
    MuxEntry,
    parse_entries,
)

MIN_BEATS_PER_TIMEOUT = 3
# Long enough to ride out jitter between key repeats or web messages, short
# enough that a quiet input hands over to the other one quickly.
MAX_INPUT_TIMEOUT_S = 1.0


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


def test_stamped_like_the_mecanum_controller(params):
    assert params["use_stamped"] is True


def test_web_and_keyboard_inputs(inputs):
    assert {name: e.topic for name, e in inputs.items()} == {
        "keyboard": KEYBOARD_CMD_VEL_TOPIC,
        "web": WEB_CMD_VEL_TOPIC,
    }


def test_web_server_drives_on_the_web_input(inputs):
    assert inputs["web"].topic == WEB_CMD_VEL_TOPIC


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


def test_stop_lock_turns_on_when_quiet(locks):
    # Timeout 0 would keep STOP only in twist_mux's memory, lost on a
    # restart (#103). With a timeout a quiet lock counts as on.
    assert locks["stop"].timeout_s > 0


def test_clients_know_the_stop_lock_timeout(locks):
    # RosBackend.stopped() treats a quieter lock as on, like twist_mux.
    assert locks["stop"].timeout_s == STOP_LOCK_TIMEOUT_S


def test_mode_node_beats_well_within_the_stop_lock_timeout(locks):
    # Missing a couple of beats must not lock the base by mistake.
    timeout_s = locks["stop"].timeout_s
    assert timeout_s >= STOP_LOCK_PERIOD_S * MIN_BEATS_PER_TIMEOUT


def test_status_node_settings(data):
    status = data["mux_status"]["ros__parameters"]
    assert status["active_input_topic"] == "/robot/active_input"
    assert status["rate_hz"] > 0
