"""ROS 2 node that clients (web server, CLI) use to talk to the running robot
node.

It only wires up topics and services. State tracking lives in
`talos_servo.ros.web_backend.RosBackend`.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, Trigger

from talos_servo.defaults import QUEUE_DEPTH, ROBOT_NS
from talos_servo.ros.names import (
    ACTIVE_INPUT_TOPIC,
    CLIMB_TOPIC,
    EXPANSION_RATE_TOPIC,
    HOLD_ENABLE_SERVICE,
    HOLD_SETPOINT_TOPIC,
    HOLD_STATE_TOPIC,
    IMU_DATA_TOPIC,
    LATCHED,
    MODE_REASON_TOPIC,
    MODE_TOPIC,
    RELEASE_SERVICE,
    SET_STOP_SERVICE,
    STOP_LOCK_TOPIC,
    WALL_NEARBY_SERVICE,
    WALL_SIDE_TOPIC,
)
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
        on_mode: Callable[[String], None] = _ignore,
        on_mode_reason: Callable[[String], None] = _ignore,
        on_active_input: Callable[[String], None] = _ignore,
        on_hold_state: Callable[[String], None] = _ignore,
        on_imu: Callable[[Imu], None] = _ignore,
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
            TwistStamped, cmd_vel_topic, QUEUE_DEPTH
        )
        # Hinge opening speed for the expansion node (#76).
        self.expansion_pub = self.create_publisher(
            Float64, EXPANSION_RATE_TOPIC, QUEUE_DEPTH
        )
        self.climb_pub = self.create_publisher(
            Float64, CLIMB_TOPIC, QUEUE_DEPTH
        )
        self.side_pub = self.create_publisher(
            Float64, WALL_SIDE_TOPIC, QUEUE_DEPTH
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
        # Latched, so a client that starts late still gets the current values.
        self.create_subscription(String, MODE_TOPIC, on_mode, LATCHED)
        self.create_subscription(
            String, MODE_REASON_TOPIC, on_mode_reason, LATCHED
        )
        self.create_subscription(
            String, ACTIVE_INPUT_TOPIC, on_active_input, LATCHED
        )
        self.stop_all_client = self.create_client(
            Trigger, f"{ROBOT_NS}/stop_all"
        )
        self.wall_nearby_client = self.create_client(
            Trigger, WALL_NEARBY_SERVICE
        )
        self.release_client = self.create_client(Trigger, RELEASE_SERVICE)
        # Hinge current hold in the expansion node (#142).
        self.hold_setpoint_pub = self.create_publisher(
            Float64, HOLD_SETPOINT_TOPIC, LATCHED
        )
        self.hold_enable_client = self.create_client(
            SetBool, HOLD_ENABLE_SERVICE
        )
        self.create_subscription(
            String, HOLD_STATE_TOPIC, on_hold_state, QUEUE_DEPTH
        )
        # Filtered IMU with orientation, for the platform tilt (#91).
        self.create_subscription(
            Imu, IMU_DATA_TOPIC, on_imu, qos_profile_sensor_data
        )
        self.torque_clients = {
            joint: self.create_client(SetBool, f"{ROBOT_NS}/{joint}/set_torque")
            for joint in joints
        }
