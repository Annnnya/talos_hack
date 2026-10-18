"""ROS 2 node that clients (web server, CLI) use to talk to the running robot
node.

It only wires up topics and services. State tracking lives in
`talos_servo.ros.web_backend.RosBackend`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.context import Context
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger

from talos_servo.defaults import QUEUE_DEPTH, ROBOT_NS
from talos_servo.ros.names import SET_STOP_SERVICE, STOP_LOCK_TOPIC
from talos_servo.ros.robot_node import (
    CMD_VEL_TOPIC,
    DIAGNOSTICS_TOPIC,
    JOINT_STATES_TOPIC,
)

NODE_NAME = "robot_client"


def _ignore(_msg: object) -> None:
    pass


class WebClientNode(Node):
    def __init__(
        self,
        joints: Iterable[str],
        on_joint_states: Callable[[JointState], None],
        on_status: Callable[[DiagnosticArray], None],
        *,
        on_stop_lock: Callable[[Bool], None] = _ignore,
        node_name: str = NODE_NAME,
        cmd_vel_topic: str = CMD_VEL_TOPIC,
        context: Context | None = None,
    ) -> None:
        super().__init__(node_name, context=context)
        self.position_pub = self.create_publisher(
            JointState, f"{ROBOT_NS}/position_cmd", QUEUE_DEPTH
        )
        self.velocity_pub = self.create_publisher(
            JointState, f"{ROBOT_NS}/velocity_cmd", QUEUE_DEPTH
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, cmd_vel_topic, QUEUE_DEPTH
        )
        self.create_subscription(
            JointState, JOINT_STATES_TOPIC, on_joint_states, QUEUE_DEPTH
        )
        self.create_subscription(
            DiagnosticArray, DIAGNOSTICS_TOPIC, on_status, QUEUE_DEPTH
        )
        # Default volatile QoS like twist_mux: the mode node beats every
        # 0.1 s, so a late subscriber hears the STOP state right away.
        self.create_subscription(
            Bool, STOP_LOCK_TOPIC, on_stop_lock, QUEUE_DEPTH
        )
        self.set_stop_client = self.create_client(SetBool, SET_STOP_SERVICE)
        self.stop_all_client = self.create_client(
            Trigger, f"{ROBOT_NS}/stop_all"
        )
        self.torque_clients = {
            joint: self.create_client(SetBool, f"{ROBOT_NS}/{joint}/set_torque")
            for joint in joints
        }
