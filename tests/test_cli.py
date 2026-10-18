from __future__ import annotations

import argparse
import contextlib
import sys
import types
from pathlib import Path

import pytest
from scservo_sdk import COMM_RX_TIMEOUT

from talos_servo import cli, parsing
from talos_servo.defaults import KEYBOARD_CMD_VEL_TOPIC

ROBOT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"


@pytest.fixture
def run(sdk, monkeypatch):
    monkeypatch.setattr(cli, "ServoBus", sdk.bus_factory())
    monkeypatch.setattr(cli.time, "sleep", lambda _s: None)
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    return cli.main


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        ("1", [1]),
        ("3,1,2", [1, 2, 3]),
        ("1-3", [1, 2, 3]),
        ("1-2,5", [1, 2, 5]),
        ("2,2", [2]),
    ],
)
def test_parse_ids(value, expected):
    assert parsing.parse_ids(value) == expected


@pytest.mark.parametrize("value", ["", "a", "1,x", "254", "-1"])
def test_parse_ids_rejects_bad_input(value):
    with pytest.raises(argparse.ArgumentTypeError):
        parsing.parse_ids(value)


def test_missing_port_is_an_error(run, capsys):
    with pytest.raises(SystemExit) as exc:
        run(["ping"])
    assert exc.value.code == 2
    assert "TALOS_SERVO_PORT" in capsys.readouterr().err


def test_port_from_env(run, sdk, monkeypatch):
    monkeypatch.setenv(cli.PORT_ENV_VAR, "COM7")
    # Parser defaults are read at build time, so the env var must be set before
    # main().
    assert run(["ping"]) == 0
    assert sdk.port.name == "COM7"


def test_ping_reports_each_servo(run, sdk, capsys):
    sdk.fail[("ping", 2)] = (COMM_RX_TIMEOUT, 0)
    assert run(["ping", "--port", "COM5", "--ids", "1-3"]) == 1
    out = capsys.readouterr().out
    assert "servo 1: ok (model 777)" in out
    assert "servo 2: ping servo 2 failed: timeout" in out
    assert "servo 3: ok" in out


def test_spin_runs_then_stops_every_servo(run, sdk):
    assert (
        run(
            [
                "spin",
                "--port",
                "COM5",
                "--ids",
                "1,2",
                "--speed",
                "150",
                "--acc",
                "10",
            ]
        )
        == 0
    )
    assert sdk.handler.speed_calls() == [
        ("speed", 1, 150, 10),
        ("speed", 2, 150, 10),
        ("speed", 1, 0, 10),
        ("speed", 2, 0, 10),
    ]
    assert sdk.port.closed_count == 1


def test_spin_interrupted_still_stops(run, sdk, monkeypatch, capsys):
    def interrupt(_s):
        raise KeyboardInterrupt

    monkeypatch.setattr(cli.time, "sleep", interrupt)
    assert run(["spin", "--port", "COM5", "--ids", "1"]) == cli.EXIT_INTERRUPTED
    assert sdk.handler.speed_calls()[-1] == ("speed", 1, 0, 20)
    assert "servos stopped" in capsys.readouterr().err


def test_servo_error_exits_nonzero(run, sdk, capsys):
    sdk.fail[("write1", 1)] = (COMM_RX_TIMEOUT, 0)
    assert run(["spin", "--port", "COM5"]) == 1
    assert (
        "error: set wheel mode on servo 1 failed: timeout"
        in capsys.readouterr().err
    )


def test_monitor_reads_without_moving_by_default(run, sdk, capsys):
    assert (
        run(
            [
                "monitor",
                "--port",
                "COM5",
                "--seconds",
                "0.05",
                "--interval",
                "0",
            ]
        )
        == 0
    )
    assert sdk.handler.speed_calls() == []
    assert "servo 1: position=2048 speed=0" in capsys.readouterr().out


def test_set_id(run, sdk, capsys):
    sdk.present = {1}
    assert run(["set-id", "--port", "COM5", "--id", "1", "--new-id", "3"]) == 0
    assert sdk.present == {3}
    assert "servo 1 is now servo 3" in capsys.readouterr().out


def test_set_id_rejects_id_lists(run):
    with pytest.raises(SystemExit):
        run(["set-id", "--port", "COM5", "--id", "1,2", "--new-id", "3"])


# --- --ros: spin and monitor through the servo bridge
# -----------------------------------


@pytest.fixture
def over_ros(monkeypatch, tmp_path):
    """Stub cli_commands so these run without rclpy; records what each got."""
    import talos_servo.ros

    calls = {}
    stub = types.ModuleType("talos_servo.ros.cli_commands")
    stub.RosBackend = lambda config, **options: {
        "joints": [j.name for j in config.enabled_joints],
        **options,
    }
    stub.spin_over_ros = lambda backend, *a: calls.update(spin=(backend, a))
    stub.monitor_over_ros = lambda backend, *a: calls.update(
        monitor=(backend, a)
    )
    stub.drive_over_ros = lambda backend, read_key, **kw: calls.update(
        drive=(backend, read_key, kw)
    )
    monkeypatch.setitem(sys.modules, "talos_servo.ros.cli_commands", stub)
    monkeypatch.setattr(talos_servo.ros, "cli_commands", stub, raising=False)
    monkeypatch.delenv(cli.PORT_ENV_VAR, raising=False)
    config = tmp_path / "robot.yaml"
    config.write_text(
        "joints:\n"
        "  bend: {id: 1, control: position}\n"
        "  wheel: {id: 2, control: speed}\n"
    )
    return calls, str(config)


def test_spin_over_ros_maps_ids_to_joints(over_ros):
    calls, config = over_ros
    args = ["spin", "--ros", "--config", config, "--ids", "2"]
    assert cli.main([*args, "--speed", "300"]) == 0
    assert calls["spin"] == (
        {"joints": ["bend", "wheel"]},
        (["wheel"], 300, 1.0),
    )


def test_over_ros_rejects_ids_not_in_the_config(over_ros, capsys):
    _calls, config = over_ros
    assert cli.main(["spin", "--ros", "--config", config, "--ids", "7"]) == 1
    assert "servo ids [7]" in capsys.readouterr().err


def test_monitor_over_ros_passes_interval(over_ros):
    calls, config = over_ros
    args = ["monitor", "--ros", "--config", config, "--seconds", "2"]
    assert cli.main([*args, "--interval", "0.5"]) == 0
    assert calls["monitor"][1] == (["bend"], 0, 2.0, 0.5)


def test_over_ros_says_acceleration_is_ignored(over_ros, capsys):
    _calls, config = over_ros
    args = ["spin", "--ros", "--config", config, "--ids", "2", "--acc", "50"]
    assert cli.main(args) == 0
    assert "--acc is ignored" in capsys.readouterr().out


@pytest.fixture
def fake_terminal(monkeypatch):
    """`terminal_keys` without a real terminal; hands out the key reader."""
    from talos_servo import teleop

    def read_key(_timeout: float) -> str | None:
        return None

    @contextlib.contextmanager
    def terminal_keys():
        yield read_key

    monkeypatch.setattr(teleop, "terminal_keys", terminal_keys)
    return read_key


def test_drive_over_ros_uses_the_keyboard_mux_input(
    over_ros, fake_terminal, capsys
):
    calls, _config = over_ros
    assert cli.main(["drive", "--ros", "--config", str(ROBOT_CONFIG)]) == 0
    backend, read_key, kw = calls["drive"]
    assert backend["cmd_vel_topic"] == KEYBOARD_CMD_VEL_TOPIC
    assert read_key is fake_terminal
    assert kw == {"linear": 0.3, "angular": 0.5}
    out = capsys.readouterr().out
    assert "N wall nearby" in out
    assert "L release" in out


def test_drive_over_ros_takes_speeds_from_options(over_ros, fake_terminal):
    calls, _config = over_ros
    args = ["drive", "--ros", "--config", str(ROBOT_CONFIG)]
    assert cli.main([*args, "--linear", "0.05", "--angular", "0.2"]) == 0
    assert calls["drive"][2] == {"linear": 0.05, "angular": 0.2}


def test_over_ros_without_ros_installed_explains(monkeypatch, capsys):
    import talos_servo.ros

    monkeypatch.setitem(sys.modules, "talos_servo.ros.cli_commands", None)
    monkeypatch.delattr(talos_servo.ros, "cli_commands", raising=False)
    assert cli.main(["spin", "--ros"]) == 1
    assert "pixi run -e ros talos-servo spin --ros" in capsys.readouterr().err
