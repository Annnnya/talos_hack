"""ROS 2 node the web server uses to talk to a running servo bridge instead of the serial port.

It only wires up topics and services under the bridge namespace (default `/servo_bridge`).
State tracking and command rules live in `talos_servo.ros.web_backend.RosBackend`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable

from diagnostic_msgs.msg import DiagnosticArray
from rclpy.context import Context
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

NODE_NAME = "servo_web"
QUEUE_DEPTH = 10


class WebClientNode(Node):
    def __init__(
        self,
        bridge: str,
        joints: Iterable[str],
        on_joint_states: Callable[[JointState], None],
        on_status: Callable[[DiagnosticArray], None],
        *,
        node_name: str = NODE_NAME,
        context: Context | None = None,
    ) -> None:
        super().__init__(node_name, context=context)
        ns = bridge.rstrip("/")
        self.position_pub = self.create_publisher(JointState, f"{ns}/position_cmd", QUEUE_DEPTH)
        self.velocity_pub = self.create_publisher(JointState, f"{ns}/velocity_cmd", QUEUE_DEPTH)
        self.create_subscription(JointState, f"{ns}/joint_states", on_joint_states, QUEUE_DEPTH)
        self.create_subscription(DiagnosticArray, f"{ns}/status", on_status, QUEUE_DEPTH)
        self.stop_all_client = self.create_client(Trigger, f"{ns}/stop_all")
        self.torque_clients = {}
        self.wheel_mode_clients = {}
        for joint in joints:
            self.torque_clients[joint] = self.create_client(SetBool, f"{ns}/{joint}/set_torque")
            self.wheel_mode_clients[joint] = self.create_client(
                SetBool, f"{ns}/{joint}/set_wheel_mode"
            )
