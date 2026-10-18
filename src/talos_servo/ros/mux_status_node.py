"""ROS 2 node that publishes which twist_mux input drives the base.

Reads the same `topics.*` and `locks.*` parameters as twist_mux
(`config/twist_mux.yaml`), listens to every input and lock, and publishes on
`active_input_topic` (std_msgs/String, default `/robot/active_input`): the
input name, `none` or `locked`. Latched, so a UI that starts late still gets
the current value. Run with `pixi run -e ros mux-status`.

Never opens the servo port.
"""

from __future__ import annotations

import sys
from collections.abc import Sequence
from functools import partial
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from talos_servo.defaults import QUEUE_DEPTH
from talos_servo.ros.mux_status import (
    LOCKS_PREFIX,
    TOPICS_PREFIX,
    MuxState,
    parse_entries,
)
from talos_servo.ros.names import ACTIVE_INPUT_TOPIC

NODE_NAME = "mux_status"
DEFAULT_ACTIVE_INPUT_TOPIC = ACTIVE_INPUT_TOPIC
DEFAULT_RATE_HZ = 5.0
NANOSECONDS_PER_SECOND = 1e9
# rclpy's list_parameters depth for "every level".
ALL_DEPTHS = 0


class MuxStatusNode(Node):
    def __init__(self, *, node_name: str = NODE_NAME, **node_kwargs: Any):
        # twist_mux params are nested maps with names we only know at run
        # time, so take whatever the params file brings.
        super().__init__(
            node_name,
            automatically_declare_parameters_from_overrides=True,
            **node_kwargs,
        )
        topic = str(
            self._param("active_input_topic", DEFAULT_ACTIVE_INPUT_TOPIC)
        )
        rate_hz = float(self._param("rate_hz", DEFAULT_RATE_HZ))
        if rate_hz <= 0:
            raise ValueError(f"rate_hz must be > 0, got {rate_hz}")

        names = self.list_parameters(
            [TOPICS_PREFIX, LOCKS_PREFIX], ALL_DEPTHS
        ).names
        params = {name: self.get_parameter(name).value for name in names}
        inputs = parse_entries(params, TOPICS_PREFIX)
        locks = parse_entries(params, LOCKS_PREFIX)
        if not inputs:
            raise ValueError(
                f"no twist_mux inputs under '{TOPICS_PREFIX}', "
                "pass --params-file config/twist_mux.yaml"
            )
        self.state = MuxState(inputs, locks)

        for entry in inputs:
            self.create_subscription(
                Twist,
                entry.topic,
                partial(self.on_input, entry.name),
                QUEUE_DEPTH,
            )
        for entry in locks:
            self.create_subscription(
                Bool,
                entry.topic,
                partial(self.on_lock, entry.name),
                QUEUE_DEPTH,
            )
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.active_pub = self.create_publisher(String, topic, latched)
        self.last_active: str | None = None
        self.create_timer(1.0 / rate_hz, self.publish_active)
        self.publish_active()
        self.get_logger().info(
            f"publishing the active input on {topic}: "
            + ", ".join(f"{e.name} ({e.topic})" for e in inputs)
        )

    def _param(self, name: str, default: Any) -> Any:
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / NANOSECONDS_PER_SECOND

    def on_input(self, name: str, _msg: Twist) -> None:
        self.state.on_input(name, self.now_s())
        self.publish_active(only_on_change=True)

    def on_lock(self, name: str, msg: Bool) -> None:
        self.state.on_lock(name, bool(msg.data), self.now_s())
        self.publish_active(only_on_change=True)

    def publish_active(self, *, only_on_change: bool = False) -> None:
        active = self.state.active(self.now_s())
        if only_on_change and active == self.last_active:
            return
        self.last_active = active
        self.active_pub.publish(String(data=active))


def main(argv: Sequence[str] | None = None) -> int:
    rclpy.init(args=list(sys.argv if argv is None else argv))
    node = None
    try:
        node = MuxStatusNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as exc:
        print(f"talos-mux-status-ros: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
