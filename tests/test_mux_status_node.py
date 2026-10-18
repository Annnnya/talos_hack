"""mux_status node with the real twist_mux params file.

Skipped where ROS messages are missing.
"""

from __future__ import annotations

from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("geometry_msgs")
pytest.importorskip("std_msgs")

from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from talos_servo.ros.mux_status import LOCKED, NONE  # noqa: E402
from talos_servo.ros.mux_status_node import MuxStatusNode  # noqa: E402

MUX_CONFIG = Path(__file__).resolve().parents[1] / "config" / "twist_mux.yaml"


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init(args=["test", "--ros-args", "--params-file", str(MUX_CONFIG)])
    yield
    rclpy.shutdown()


@pytest.fixture
def node(monkeypatch):
    n = MuxStatusNode()
    sent: list[str] = []
    monkeypatch.setattr(
        n.active_pub, "publish", lambda msg: sent.append(msg.data)
    )
    n.sent = sent
    yield n
    n.destroy_node()


def test_reads_inputs_and_lock_from_the_params_file(node):
    assert set(node.state.inputs) == {"keyboard", "web"}
    assert node.state.locks["stop"].topic == "/robot/stop_lock"


def release(node) -> None:
    """A STOP released beat from the mode node, then forget what was sent."""
    node.on_lock("stop", Bool(data=False))
    node.sent.clear()


def test_locked_until_the_mode_node_beats(node):
    # The stop lock has a timeout, so it is on before its first message.
    node.publish_active()
    assert node.sent == [LOCKED]
    node.on_lock("stop", Bool(data=False))
    assert node.sent == [LOCKED, NONE]


def test_publishes_who_drives(node):
    release(node)
    node.on_input("web", Twist())
    node.on_input("keyboard", Twist())
    node.on_lock("stop", Bool(data=True))
    node.on_lock("stop", Bool(data=False))
    assert node.sent == ["web", "keyboard", LOCKED, "keyboard"]


def test_repeats_only_on_change_from_messages(node):
    release(node)
    node.on_input("web", Twist())
    node.on_input("web", Twist())
    assert node.sent == ["web"]


def test_timer_republishes(node):
    release(node)
    node.publish_active()
    assert node.sent == [NONE]


def test_rejects_missing_inputs():
    with pytest.raises(ValueError, match="no twist_mux inputs"):
        MuxStatusNode(
            use_global_arguments=False,
            cli_args=[],
        )
