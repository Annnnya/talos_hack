"""Active twist_mux input logic, without ROS."""

from __future__ import annotations

import pytest

from talos_servo.ros.mux_status import (
    LOCKED,
    NONE,
    MuxEntry,
    MuxState,
    parse_entries,
)

KEYBOARD = MuxEntry("keyboard", "/keyboard/cmd_vel", 0.5, 20)
WEB = MuxEntry("web", "/web/cmd_vel", 0.5, 10)
STOP = MuxEntry("stop", "/robot/stop_lock", 0.0, 100)


@pytest.fixture
def mux() -> MuxState:
    return MuxState([KEYBOARD, WEB], [STOP])


def test_none_when_idle(mux):
    assert mux.active(0.0) == NONE


def test_single_input_drives(mux):
    mux.on_input("web", 1.0)
    assert mux.active(1.1) == "web"


def test_higher_priority_wins_while_both_send(mux):
    mux.on_input("web", 1.0)
    mux.on_input("keyboard", 1.1)
    mux.on_input("web", 1.2)
    assert mux.active(1.3) == "keyboard"


def test_quiet_input_hands_over_to_the_other(mux):
    mux.on_input("keyboard", 1.0)
    mux.on_input("web", 1.0)
    mux.on_input("web", 1.4)
    assert mux.active(1.6) == "web"


def test_expired_input_drops_out(mux):
    mux.on_input("web", 1.0)
    assert mux.active(1.5) == "web"
    assert mux.active(1.51) == NONE


def test_lock_masks_every_input(mux):
    mux.on_input("keyboard", 1.0)
    mux.on_input("web", 1.0)
    mux.on_lock("stop", True, 1.1)
    assert mux.active(1.2) == LOCKED


def test_lock_without_timeout_holds_until_released(mux):
    mux.on_lock("stop", True, 1.0)
    mux.on_input("web", 100.0)
    assert mux.active(100.1) == LOCKED
    mux.on_lock("stop", False, 100.2)
    mux.on_input("web", 100.3)
    assert mux.active(100.4) == "web"


def test_lock_with_timeout_is_on_until_it_hears_false():
    # twist_mux treats an expired lock as on, including before its first
    # message.
    heartbeat = MuxEntry("heartbeat", "/heartbeat", 1.0, 50)
    mux = MuxState([WEB], [heartbeat])
    mux.on_input("web", 1.0)
    assert mux.active(1.1) == LOCKED
    mux.on_lock("heartbeat", False, 1.2)
    assert mux.active(1.3) == "web"
    mux.on_input("web", 2.5)
    assert mux.active(2.6) == LOCKED


def test_stop_heartbeat_survives_a_mux_restart():
    beat = MuxEntry("stop", "/robot/stop_lock", 0.5, 100)
    mux = MuxState([WEB], [beat])
    mux.on_lock("stop", False, 1.0)
    mux.on_input("web", 1.0)
    assert mux.active(1.1) == "web"
    mux.on_lock("stop", True, 1.2)
    # A restarted mux has heard nothing: locked, then kept locked by the
    # next true beat while the web keeps sending.
    restarted = MuxState([WEB], [beat])
    restarted.on_input("web", 1.25)
    assert restarted.active(1.25) == LOCKED
    restarted.on_lock("stop", True, 1.3)
    restarted.on_input("web", 1.35)
    assert restarted.active(1.4) == LOCKED


def test_lock_below_an_input_does_not_mask_it():
    low = MuxEntry("low", "/low_lock", 0.0, 15)
    mux = MuxState([KEYBOARD, WEB], [low])
    mux.on_lock("low", True, 1.0)
    mux.on_input("web", 1.0)
    assert mux.active(1.1) == LOCKED
    mux.on_input("keyboard", 1.2)
    assert mux.active(1.3) == "keyboard"


def test_input_without_timeout_never_expires():
    forever = MuxEntry("forever", "/forever", 0.0, 5)
    mux = MuxState([forever], [])
    mux.on_input("forever", 1.0)
    assert mux.active(1e6) == "forever"


def test_parse_entries_from_flat_params():
    params = {
        "use_stamped": False,
        "topics.web.topic": "/web/cmd_vel",
        "topics.web.timeout": 0.5,
        "topics.web.priority": 10,
        "locks.stop.topic": "/robot/stop_lock",
        "locks.stop.timeout": 0,
        "locks.stop.priority": 100,
    }
    assert parse_entries(params, "topics") == [WEB]
    assert parse_entries(params, "locks") == [STOP]


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("topic", "", "non-empty"),
        ("timeout", -1.0, ">= 0"),
        ("timeout", "fast", "number"),
        ("priority", 256, "0..255"),
        ("priority", 1.5, "integer"),
    ],
)
def test_parse_entries_rejects_bad_fields(field, value, message):
    params = {
        "topics.web.topic": "/web/cmd_vel",
        "topics.web.timeout": 0.5,
        "topics.web.priority": 10,
        f"topics.web.{field}": value,
    }
    with pytest.raises(ValueError, match=message):
        parse_entries(params, "topics")


def test_parse_entries_reports_missing_fields():
    with pytest.raises(ValueError, match="missing priority"):
        parse_entries(
            {"topics.web.topic": "/web/cmd_vel", "topics.web.timeout": 0.5},
            "topics",
        )
