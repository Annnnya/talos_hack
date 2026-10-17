"""What the web server needs from whatever drives the servos.

Two implementations: `RobotController` owns the serial bus directly, and
`talos_servo.ros.web_backend.RosBackend` goes through the running ROS servo bridge.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:  # config needs pyyaml
    from talos_servo.config import RobotConfig


class RobotBackend(Protocol):
    config: RobotConfig

    def open(self) -> None: ...

    def close(self) -> None:
        """Stop every wheel-mode servo and release the connection."""

    def poll(self) -> None:
        """Called once per telemetry tick from a worker thread."""

    def snapshot(self) -> list[dict[str, Any]]: ...

    def stop_all(self) -> None:
        """Best effort: never raises, because it runs when the last browser disconnects."""

    def stop(self, servo_id: int) -> None: ...

    def set_mode(self, servo_id: int, mode: str) -> None: ...

    def set_torque(self, servo_id: int, enabled: bool) -> None: ...

    def move(self, servo_id: int, position: int, speed: int, acceleration: int = ...) -> None: ...

    def spin(self, servo_id: int, speed: int, acceleration: int = ...) -> None: ...
