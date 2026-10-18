"""Expansion node (#76): rate in, hinge goals and wheel speeds out.

The node is never spun: tests feed its callbacks and call `tick` with a fake
clock. Skipped where ROS is missing; conftest.py keeps it on a private ROS
domain.
"""

from __future__ import annotations

import math
import threading
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("std_srvs")

from conftest import FakeClock, wait_for  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.task import Future  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import (  # noqa: E402
    Bool,
    Float64,
    Float64MultiArray,
    String,
)
from std_srvs.srv import SetBool  # noqa: E402

from talos_servo.config import load_config  # noqa: E402
from talos_servo.expansion import STILL  # noqa: E402
from talos_servo.ros.expansion_node import ExpansionNode  # noqa: E402
from talos_servo.ros.mode_node import ModeNode  # noqa: E402
from talos_servo.ros.names import (  # noqa: E402
    EXPANSION_RATE_TOPIC,
    WHEEL_COMMANDS_TOPIC,
)

CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"
RATE = 0.1
ALPHA = math.radians(120)
_names = iter(range(1_000_000))


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Recorder:
    """Stands in for the node's publishers and the mode node service."""

    def __init__(self, node: ExpansionNode, monkeypatch) -> None:
        self.goals: list[float] = []
        self.wheels: list[tuple[float, ...]] = []
        self.calls: list[bool] = []
        self.answers: list[Future] = []
        self.ready = True
        self.auto_answer: bool | None = True
        monkeypatch.setattr(
            node.hinge_pub, "publish", lambda m: self.goals.append(*m.data)
        )
        monkeypatch.setattr(
            node.wheel_pub,
            "publish",
            lambda m: self.wheels.append(tuple(m.data)),
        )
        monkeypatch.setattr(
            node.hinge_motion_client, "service_is_ready", lambda: self.ready
        )
        monkeypatch.setattr(node, "call_hinge_motion", self.call)

    def call(self, wanted: bool) -> Future:
        self.calls.append(wanted)
        future = Future()
        self.answers.append(future)
        if self.auto_answer is not None:
            self.answer(future, self.auto_answer)
        return future

    @staticmethod
    def answer(future: Future, ok: bool) -> None:
        future.set_result(SetBool.Response(success=ok, message="test"))


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def node(clock):
    expansion = ExpansionNode(
        load_config(CONFIG),
        node_name=f"expansion_{next(_names)}",
        clock=clock,
    )
    yield expansion
    expansion.destroy_node()


@pytest.fixture
def out(node, monkeypatch) -> Recorder:
    return Recorder(node, monkeypatch)


def ready(node: ExpansionNode, mode: str = "clamping") -> None:
    """Mode known, STOP released and the hinge at ALPHA."""
    node.on_mode(String(data=mode))
    node.on_stop_lock(Bool(data=False))
    joint_state(node, ALPHA)


def joint_state(node: ExpansionNode, alpha: float) -> None:
    node.on_joint_state(
        JointState(name=["front_left_wheel", "bend"], position=[0.0, alpha])
    )


def test_reads_the_hinge_angle(node):
    joint_state(node, ALPHA)
    assert node.alpha == pytest.approx(ALPHA)


def test_rate_moves_the_hinge_and_turns_the_wheels(node, out):
    ready(node)
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert out.goals == [pytest.approx(ALPHA + RATE * node.period_s)]
    (wheels,) = out.wheels
    assert wheels[0] == wheels[1] > 0 > wheels[2] == wheels[3]
    # Clamping already runs the wheel controller.
    assert out.calls == []


def test_no_rate_sends_nothing(node, out):
    ready(node)
    node.tick()
    assert (out.goals, out.wheels) == ([], [])


def test_zero_rate_stops_the_wheels_once(node, out):
    ready(node)
    node.on_rate(Float64(data=RATE))
    node.tick()
    node.on_rate(Float64(data=0.0))
    node.tick()
    node.tick()
    assert out.wheels[1:] == [STILL]
    assert len(out.goals) == 1


def test_quiet_rate_times_out(node, out, clock):
    ready(node)
    node.on_rate(Float64(data=RATE))
    node.tick()
    clock.now += node.settings.command_timeout_s + 0.1
    node.on_stop_lock(Bool(data=False))
    node.tick()
    assert out.wheels[-1] == STILL


def test_stop_stops_the_wheels(node, out):
    ready(node)
    node.on_rate(Float64(data=RATE))
    node.tick()
    node.on_stop_lock(Bool(data=True))
    node.tick()
    assert out.wheels[-1] == STILL
    assert len(out.goals) == 1


def test_missing_stop_beats_count_as_stop(node, out, clock):
    ready(node)
    node.on_rate(Float64(data=RATE))
    clock.now += 1.0
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert (out.goals, out.wheels) == ([], [])


@pytest.mark.parametrize("mode", ["aligning", "climbing", "nonsense"])
def test_hinge_stays_in_other_modes(node, out, mode):
    ready(node, mode)
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert (out.goals, out.wheels) == ([], [])


def test_free_mode_asks_for_the_wheels_first(node, out):
    ready(node, "free")
    out.auto_answer = None
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert out.calls == [True]
    assert (out.goals, out.wheels) == ([], [])
    # Only one question at a time.
    node.tick()
    assert out.calls == [True]
    out.answer(out.answers[0], True)
    assert node.granted
    node.tick()
    assert len(out.goals) == 1
    assert out.wheels[-1] != STILL


def test_free_mode_gives_the_wheels_back(node, out):
    ready(node, "free")
    node.on_rate(Float64(data=RATE))
    node.tick()
    node.tick()
    node.on_rate(Float64(data=0.0))
    node.tick()
    assert out.calls == [True, False]
    assert out.wheels[-1] == STILL
    assert not node.granted


def test_refused_swap_waits_before_asking_again(node, out, clock):
    ready(node, "free")
    out.auto_answer = False
    node.on_rate(Float64(data=RATE))
    node.tick()
    node.tick()
    assert out.calls == [True]
    assert not node.granted
    clock.now += 1.5
    node.on_rate(Float64(data=RATE))
    node.on_stop_lock(Bool(data=False))
    node.tick()
    assert out.calls == [True, True]


def test_mode_change_drops_the_swap(node, out):
    ready(node, "free")
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert node.granted
    node.on_mode(String(data="aligning"))
    assert not node.granted


def test_unknown_angle_holds_the_wheels(node, out):
    ready(node)
    joint_state(node, math.nan)
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert out.goals == []
    assert out.wheels in ([], [STILL])


def test_fully_open_refuses_to_expand(node, out):
    ready(node)
    joint_state(node, math.pi)
    node.on_rate(Float64(data=RATE))
    node.tick()
    assert out.goals == []


def test_shutdown_zeroes_the_wheels(node, out):
    ready(node, "free")
    node.on_rate(Float64(data=RATE))
    node.tick()
    node.tick()
    node.shutdown()
    assert out.wheels[-1] == STILL
    assert out.calls[-1] is False


def test_rate_reaches_the_wheels_through_the_mode_node():
    """Over real topics and the real mode node service (no
    controller_manager, so the swap itself is skipped)."""
    config = load_config(CONFIG)
    mode = ModeNode(config.modes, node_name=f"mode_{next(_names)}")
    expansion = ExpansionNode(config, node_name=f"expansion_{next(_names)}")
    helper = Node(f"helper_{next(_names)}")
    wheels: list[tuple[float, ...]] = []
    helper.create_subscription(
        Float64MultiArray,
        WHEEL_COMMANDS_TOPIC,
        lambda m: wheels.append(tuple(m.data)),
        10,
    )
    rate_pub = helper.create_publisher(Float64, EXPANSION_RATE_TOPIC, 10)
    states_pub = helper.create_publisher(JointState, "/joint_states", 10)

    def feed() -> None:
        rate_pub.publish(Float64(data=RATE))
        states_pub.publish(JointState(name=["bend"], position=[ALPHA]))

    helper.create_timer(0.05, feed)
    executor = MultiThreadedExecutor()
    for each in (mode, expansion, helper):
        executor.add_node(each)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        mode.set_stopped(False)
        wait_for(lambda: mode.hinge_motion, 5.0)
        wait_for(lambda: wheels and wheels[-1][0] > 0, 5.0)
        mode.set_stopped(True)
        wait_for(lambda: wheels[-1] == STILL, 5.0)
    finally:
        executor.shutdown()
        thread.join(2.0)
        for each in (mode, expansion, helper):
            each.destroy_node()
