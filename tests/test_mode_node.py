"""Mode node against a fake controller_manager.

Skipped where ROS is missing. conftest.py puts every test on a private ROS
domain, so nothing here can reach a real robot.
"""

from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("std_srvs")
pytest.importorskip("controller_manager_msgs")

from conftest import wait_for  # noqa: E402
from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import (  # noqa: E402
    ListControllers,
    SwitchController,
)
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from test_twist_mux_config import (  # noqa: E402
    MUX_CONFIG,
    flatten,
)
from test_twist_mux_config import (  # noqa: E402
    STOP_LOCK_PERIOD_S as CONFIG_TEST_PERIOD_S,
)

from talos_servo.config import ModesConfig  # noqa: E402
from talos_servo.modes import Event, Mode  # noqa: E402
from talos_servo.ros.mode_node import (  # noqa: E402
    LATCHED,
    MODE_REASON_TOPIC,
    MODE_TOPIC,
    RECONCILE_PERIOD_S,
    STOP_LOCK_PERIOD_S,
    STOP_LOCK_TOPIC,
    ModeNode,
)
from talos_servo.ros.mux_status import (  # noqa: E402
    LOCKED,
    LOCKS_PREFIX,
    TOPICS_PREFIX,
    MuxState,
    parse_entries,
)

MODES = ModesConfig(
    controllers={
        "free": ("drive",),
        "aligning": ("drive", "align"),
        "clamping": ("expand",),
        "climbing": ("wall",),
    },
    switch_timeout_s=0.5,
)
WAIT = 3.0
# Discovery plus a few reconcile rounds.
RECONCILE_WAIT = WAIT + 3 * RECONCILE_PERIOD_S
_names = iter(range(1_000_000))
# What twist_mux 4.5.0 subscribes to locks with: a restarted mux gets no old
# message, only new ones.
TWIST_MUX_LOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.RELIABLE,
)


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class FakeControllerManager(Node):
    """Tracks which controllers are active and rejects a STRICT request to
    activate an active one or deactivate an inactive one, like Jazzy."""

    def __init__(self, name: str, active: set[str] | None = None) -> None:
        super().__init__(name)
        self.requests: list[SwitchController.Request] = []
        self.active: set[str] = set(active or ())
        self.ok = True
        self.delay_s = 0.0
        # True: apply the switch, then answer late. False: answer late, then
        # apply.
        self.apply_first = False
        self.idle = threading.Event()
        self.idle.set()
        group = ReentrantCallbackGroup()
        self.create_service(
            SwitchController,
            f"/{name}/switch_controller",
            self.on_switch,
            callback_group=group,
        )
        self.create_service(
            ListControllers,
            f"/{name}/list_controllers",
            self.on_list,
            callback_group=group,
        )

    def on_switch(self, request, response):
        self.idle.clear()
        try:
            self.requests.append(request)
            deactivate = set(request.deactivate_controllers)
            activate = set(request.activate_controllers)
            valid = deactivate <= self.active and not (
                activate & (self.active - deactivate)
            )
            response.ok = self.ok and valid
            if response.ok and self.apply_first:
                self.active = (self.active - deactivate) | activate
            time.sleep(self.delay_s)
            if response.ok and not self.apply_first:
                self.active = (self.active - deactivate) | activate
            return response
        finally:
            self.idle.set()

    def on_list(self, _request, response):
        response.controller = [
            ControllerState(name=name, state="active")
            for name in sorted(self.active)
        ]
        return response


@pytest.fixture
def executor():
    executor = MultiThreadedExecutor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield executor
    executor.shutdown()
    thread.join(2.0)


def remove(executor, node: Node) -> None:
    executor.remove_node(node)
    node.destroy_node()


def spin_mode_node(
    executor, cm_name: str, *, require_cm: bool = False
) -> ModeNode:
    node = ModeNode(
        MODES,
        node_name=f"mode_{next(_names)}",
        parameter_overrides=[
            Parameter("controller_manager", value=f"/{cm_name}"),
            Parameter("require_controller_manager", value=require_cm),
        ],
    )
    executor.add_node(node)
    return node


def spin_cm(executor, name: str, active=None) -> FakeControllerManager:
    fake = FakeControllerManager(name, active)
    executor.add_node(fake)
    return fake


def stop_cm(executor, fake: FakeControllerManager) -> None:
    fake.idle.wait(WAIT)
    remove(executor, fake)


@pytest.fixture
def cm(executor):
    # Starts with the free controllers active, like after the spawners.
    fake = spin_cm(executor, f"cm_{next(_names)}", {"drive"})
    yield fake
    stop_cm(executor, fake)


@pytest.fixture
def node(executor, cm):
    mode_node = spin_mode_node(executor, cm.get_name())
    wait_for(lambda: mode_node.switcher.ready, WAIT)
    yield mode_node
    remove(executor, mode_node)


@pytest.fixture
def lonely_node(executor):
    """A mode node with no controller_manager running."""
    mode_node = spin_mode_node(executor, f"absent_{next(_names)}")
    yield mode_node
    remove(executor, mode_node)


def call(handler) -> Trigger.Response:
    return handler(Trigger.Request(), Trigger.Response())


def test_wall_nearby_switches_only_changed_controllers(node, cm):
    response = call(node.on_wall_nearby)
    assert response.success, response.message
    assert node.mode is Mode.ALIGNING
    (request,) = cm.requests
    assert list(request.deactivate_controllers) == []
    assert list(request.activate_controllers) == ["align"]
    assert request.strictness == SwitchController.Request.STRICT


def test_release_returns_to_free(node, cm):
    node.request(Event.WALL_NEARBY)
    node.request(Event.ALIGNED)
    response = call(node.on_release)
    assert response.success, response.message
    assert (node.mode, node.machine.reason) == (Mode.FREE, "released")
    last = cm.requests[-1]
    assert list(last.deactivate_controllers) == ["expand"]
    assert list(last.activate_controllers) == ["drive"]


def test_rejected_switch_keeps_the_mode(node, cm):
    cm.ok = False
    response = call(node.on_wall_nearby)
    assert not response.success
    assert "controller switch failed" in response.message
    assert node.mode is Mode.FREE


def test_switch_timeout_keeps_the_mode(node, cm):
    cm.delay_s = MODES.switch_timeout_s * 3
    response = call(node.on_wall_nearby)
    assert not response.success
    assert "did not answer" in response.message
    assert node.mode is Mode.FREE


def test_switch_applied_after_a_timeout_is_undone(node, cm):
    cm.delay_s = MODES.switch_timeout_s * 3
    call(node.on_wall_nearby)
    # The late switch lands; reconcile puts the free controllers back.
    wait_for(lambda: "align" in cm.active, WAIT)
    cm.delay_s = 0.0
    wait_for(lambda: cm.active == {"drive"}, RECONCILE_WAIT)
    assert node.mode is Mode.FREE


def test_switch_that_answers_late_but_applied_changes_the_mode(node, cm):
    cm.delay_s = MODES.switch_timeout_s * 3
    cm.apply_first = True
    response = call(node.on_wall_nearby)
    assert response.success, response.message
    assert node.mode is Mode.ALIGNING
    assert cm.active == {"drive", "align"}


def test_switch_diffs_against_the_active_controllers(node, cm):
    # The wrong controllers are running: STRICT must not see a request to
    # activate "drive" again or to deactivate an inactive one.
    cm.active = {"drive", "wall"}
    response = call(node.on_wall_nearby)
    assert response.success, response.message
    assert cm.active == {"drive", "align"}
    assert list(cm.requests[-1].deactivate_controllers) == ["wall"]
    assert list(cm.requests[-1].activate_controllers) == ["align"]


def test_unmanaged_controllers_are_left_alone(node, cm):
    cm.active = {"drive", "joint_state_broadcaster"}
    call(node.on_wall_nearby)
    assert "joint_state_broadcaster" in cm.active


def test_invalid_event_fails_without_switching(node, cm):
    call(node.on_wall_nearby)
    response = call(node.on_wall_nearby)
    assert not response.success
    assert "not allowed" in response.message
    assert node.mode is Mode.ALIGNING
    assert len(cm.requests) == 1


def test_no_controller_manager_still_changes_mode(lonely_node):
    response = call(lonely_node.on_wall_nearby)
    assert response.success, response.message
    assert lonely_node.mode is Mode.ALIGNING


def test_required_controller_manager_blocks_the_mode_change(executor):
    mode_node = spin_mode_node(
        executor, f"absent_{next(_names)}", require_cm=True
    )
    try:
        response = call(mode_node.on_wall_nearby)
        assert not response.success
        assert "not running" in response.message
        assert mode_node.mode is Mode.FREE
    finally:
        remove(executor, mode_node)


def test_late_controller_manager_gets_the_current_mode(executor):
    name = f"late_{next(_names)}"
    mode_node = spin_mode_node(executor, name)
    try:
        call(mode_node.on_wall_nearby)
        fake = spin_cm(executor, name)
        try:
            wait_for(lambda: fake.active == {"drive", "align"}, RECONCILE_WAIT)
        finally:
            stop_cm(executor, fake)
    finally:
        remove(executor, mode_node)


def test_mode_and_reason_are_latched_for_late_subscribers(node, executor):
    node.request(Event.WALL_NEARBY, "wall nearby")
    node.report("tilt fault")
    listener = Node(f"listener_{next(_names)}")
    modes: list[str] = []
    reasons: list[str] = []
    listener.create_subscription(
        String, MODE_TOPIC, lambda m: modes.append(m.data), LATCHED
    )
    listener.create_subscription(
        String, MODE_REASON_TOPIC, lambda m: reasons.append(m.data), LATCHED
    )
    executor.add_node(listener)
    try:
        wait_for(lambda: modes and reasons, WAIT)
        assert modes[-1] == "aligning"
        assert reasons[-1] == "tilt fault"
    finally:
        remove(executor, listener)


class StopLockListener:
    """Stands in for twist_mux: records stop lock beats from now on."""

    def __init__(self, executor) -> None:
        self.executor = executor
        self.node = Node(f"stop_listener_{next(_names)}")
        self.beats: list[tuple[float, bool]] = []
        self.node.create_subscription(
            Bool, STOP_LOCK_TOPIC, self.on_beat, TWIST_MUX_LOCK_QOS
        )
        executor.add_node(self.node)

    def on_beat(self, msg: Bool) -> None:
        self.beats.append((time.monotonic(), bool(msg.data)))

    def last(self) -> bool | None:
        return self.beats[-1][1] if self.beats else None

    def close(self) -> None:
        remove(self.executor, self.node)


@pytest.fixture
def listener(executor):
    stop_listener = StopLockListener(executor)
    yield stop_listener
    stop_listener.close()


def set_stop(node: ModeNode, stopped: bool) -> SetBool.Response:
    return node.on_set_stop(SetBool.Request(data=stopped), SetBool.Response())


def real_mux() -> MuxState:
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(MUX_CONFIG.read_text())
    params = flatten(data["/**"]["ros__parameters"])
    return MuxState(
        parse_entries(params, TOPICS_PREFIX),
        parse_entries(params, LOCKS_PREFIX),
    )


def test_stop_lock_period_matches_the_config_test():
    assert STOP_LOCK_PERIOD_S == CONFIG_TEST_PERIOD_S


def test_stop_starts_on(lonely_node, listener):
    assert lonely_node.stopped
    wait_for(lambda: listener.beats, WAIT)
    assert all(data for _, data in listener.beats)


def test_set_stop_releases_and_stops_again(lonely_node, listener):
    response = set_stop(lonely_node, False)
    assert (response.success, response.message) == (True, "released")
    wait_for(lambda: listener.last() is False, WAIT)
    response = set_stop(lonely_node, True)
    assert (response.success, response.message) == (True, "stopped")
    wait_for(lambda: listener.last() is True, WAIT)
    # STOP is not a mode change.
    assert lonely_node.mode is Mode.FREE


def test_stop_lock_keeps_beating(lonely_node, listener):
    wait_for(lambda: listener.beats, WAIT)
    window = 5 * STOP_LOCK_PERIOD_S
    start = time.monotonic()
    time.sleep(window)
    beats = [t for t, _ in listener.beats if t >= start]
    # Leaves room for a slow CI machine; twist_mux only needs one per 0.5 s.
    assert len(beats) >= 2


def test_restarted_twist_mux_is_locked_again(lonely_node, executor):
    # STOP goes on while the first twist_mux runs.
    first = StopLockListener(executor)
    try:
        set_stop(lonely_node, False)
        wait_for(lambda: first.last() is False, WAIT)
        set_stop(lonely_node, True)
        wait_for(lambda: first.last() is True, WAIT)
    finally:
        first.close()
    # A fresh subscriber with twist_mux's QoS gets no old message, but the
    # next beats lock it.
    restarted = StopLockListener(executor)
    try:
        wait_for(lambda: len(restarted.beats) >= 2, WAIT)
        assert all(data for _, data in restarted.beats)
        # mux_status, fed the same beats while the web keeps sending,
        # reports locked.
        mux = real_mux()
        for stamp, data in restarted.beats:
            mux.on_lock("stop", data, stamp)
            mux.on_input("web", stamp)
        assert mux.active(restarted.beats[-1][0]) == LOCKED
    finally:
        restarted.close()


def test_stop_does_not_wait_for_a_controller_switch(node, cm, listener):
    cm.delay_s = MODES.switch_timeout_s * 2
    switching = threading.Thread(target=call, args=(node.on_wall_nearby,))
    switching.start()
    try:
        wait_for(lambda: not cm.idle.is_set(), WAIT)
        start = time.monotonic()
        set_stop(node, False)
        assert time.monotonic() - start < MODES.switch_timeout_s
        wait_for(lambda: listener.last() is False, WAIT)
    finally:
        switching.join(WAIT)
