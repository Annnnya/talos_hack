"""What the web server needs from whatever drives the robot.

Two implementations: `talos_servo.robot.Robot` owns the serial bus directly, and
`talos_servo.ros.web_backend.RosBackend` goes through the running ROS robot
node.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:  # config needs pyyaml
    from talos_servo.config import RobotConfig


class RobotBackend(Protocol):
    config: RobotConfig

    def open(self) -> None: ...

    def close(self) -> None:
        """Stop every speed joint and release the connection."""

    def tick(self) -> None:
        """Called at `config.control_hz` from a worker thread."""

    def snapshot(self) -> list[dict[str, Any]]: ...

    # `source` and `client` say who sent a command (see SOURCE_* in robot.py);
    # the UI shows it next to the sliders.
    def stop_all(self, *, source: str = ..., client: str | None = None) -> None:
        """Best effort, never raises: it runs when the last browser leaves."""

    def stop(
        self, joint: str, *, source: str = ..., client: str | None = None
    ) -> None: ...

    def set_torque(self, joint: str, enabled: bool) -> None: ...

    def move(
        self,
        joint: str,
        position: int,
        speed: int,
        acceleration: int = ...,
        *,
        source: str = ...,
        client: str | None = None,
    ) -> None: ...

    def spin(
        self,
        joint: str,
        speed: int,
        acceleration: int = ...,
        *,
        source: str = ...,
        client: str | None = None,
    ) -> None: ...
