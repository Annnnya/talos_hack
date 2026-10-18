"""Per-joint `/diagnostics` from a `Robot.snapshot()`.

Shared by the nodes that own the servo port (the robot node and the hardware
bridge), so clients such as the web server read the same entries from
either. Each joint is one status named `<node name>: <joint>`.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

DIAGNOSTICS_TOPIC = "/diagnostics"
# Joint fields reported as key/values.
STATUS_KEYS = (
    "control",
    "torque",
    "position",
    "speed",
    "load",
    "voltage",
    "temp",
    "current",
    "moving",
    # Last command, so clients such as the web UI can show who moved a joint
    # and when.
    "target",
    "target_speed",
    "command_source",
    "command_client",
    "command_age",
)


def joint_diagnostics(
    node_name: str, snapshot: Iterable[dict[str, Any]], stamp: Time
) -> DiagnosticArray:
    """One status per joint: ERROR offline, WARN with an error, else OK."""
    msg = DiagnosticArray()
    msg.header.stamp = stamp
    for j in snapshot:
        status = DiagnosticStatus()
        status.name = f"{node_name}: {j['name']}"
        status.hardware_id = f"feetech id {j['id']}"
        if not j["online"]:
            status.level, status.message = (
                DiagnosticStatus.ERROR,
                j["error"] or "offline",
            )
        elif j["error"]:
            status.level, status.message = DiagnosticStatus.WARN, j["error"]
        else:
            status.level, status.message = DiagnosticStatus.OK, "ok"
        last = j["last_command"] or {}
        fields = {
            **j,
            "command_source": last.get("source"),
            "command_client": last.get("client"),
            "command_age": last.get("age"),
        }
        for key in STATUS_KEYS:
            value = fields[key]
            status.values.append(
                KeyValue(key=key, value="" if value is None else str(value))
            )
        msg.status.append(status)
    return msg
