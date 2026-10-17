"""Per joint numeric readout topics published by the robot node.

For every joint, under `<prefix>/<joint>/`:

- `temperature` (sensor_msgs/Temperature): deg C, `header.frame_id` is the joint name
- `voltage` (std_msgs/Float32): supply voltage seen by the servo, V
- `current` (std_msgs/Float32): present current, A
- `load` (std_msgs/Float32): raw present load, about -1000..1000
- `moving` (std_msgs/Bool): the servo's own moving flag

Plain numbers on their own topics so they can be plotted (rqt_plot, PlotJuggler) and echoed
without digging through DiagnosticArray key/values.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import Bool, Float32

from talos_servo.defaults import QUEUE_DEPTH
from talos_servo.ros.convert import readout_topic

READOUT_TYPES: dict[str, type] = {
    "temperature": Temperature,
    "voltage": Float32,
    "current": Float32,
    "load": Float32,
    "moving": Bool,
}


class JointReadoutPublishers:
    def __init__(
        self, node: Node, prefix: str, joint_names: Iterable[str]
    ) -> None:
        self._publishers = {
            joint: {
                field: node.create_publisher(
                    msg_type, readout_topic(prefix, joint, field), QUEUE_DEPTH
                )
                for field, msg_type in READOUT_TYPES.items()
            }
            for joint in joint_names
        }

    @property
    def topic_names(self) -> list[str]:
        return sorted(
            pub.topic_name
            for pubs in self._publishers.values()
            for pub in pubs.values()
        )

    @staticmethod
    def messages(
        joint: str,
        stamp: Time,
        *,
        temperature: float,
        voltage: float,
        current: float,
        load: float,
        moving: bool,
    ) -> dict[str, Any]:
        temperature_msg = Temperature(
            temperature=float(temperature), variance=0.0
        )
        temperature_msg.header.stamp = stamp
        temperature_msg.header.frame_id = joint
        return {
            "temperature": temperature_msg,
            "voltage": Float32(data=float(voltage)),
            "current": Float32(data=float(current)),
            "load": Float32(data=float(load)),
            "moving": Bool(data=bool(moving)),
        }

    def publish(
        self,
        joint: str,
        stamp: Time,
        *,
        temperature: float,
        voltage: float,
        current: float,
        load: float,
        moving: bool,
    ) -> None:
        messages = self.messages(
            joint,
            stamp,
            temperature=temperature,
            voltage=voltage,
            current=current,
            load=load,
            moving=moving,
        )
        for field, msg in messages.items():
            self._publishers[joint][field].publish(msg)
