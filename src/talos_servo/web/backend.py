"""What the web server needs from whatever drives the robot.

Two implementations: `talos_servo.robot.Robot` owns the serial bus directly, and
`talos_servo.ros.web_backend.RosBackend` goes through the running ROS robot
node.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Protocol

if TYPE_CHECKING:  # config needs pyyaml
    from talos_servo.config import RobotConfig
    from talos_servo.modes import RobotStatus


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
        """Stop every speed joint, and over ROS turn STOP on. Best effort,
        never raises."""

    def stop_own(self, *, source: str = ..., client: str | None = None) -> None:
        """Stop only what this backend commanded, so another interface keeps
        driving. Best effort, never raises: it runs when the last browser
        leaves."""

    def release_stop(
        self, *, source: str = ..., client: str | None = None
    ) -> None:
        """Release STOP so twist_mux drives again. Over ROS only."""

    def stopped(self) -> bool | None:
        """True while STOP locks twist_mux; None without twist_mux."""

    def robot_status(self) -> RobotStatus:
        """Mode, mode reason and active twist_mux input, each None until
        heard or without ROS."""

    def wall_nearby(self) -> None:
        """Ask the mode node to start aligning. Over ROS only."""

    def release_walls(self) -> None:
        """Ask the mode node to release the walls and go back to free. Over
        ROS only."""

    def move_hinge(self, rate: float) -> None:
        """Hinge opening speed (rad/s, + opens). Stops unless repeated. Over
        ROS only."""

    def climb(self, speed: float) -> None:
        """Climb speed (m/s, + up): front wheels one way, rear the other.
        Stops unless repeated. Over ROS only."""

    def hinge_state(self) -> dict[str, Any] | None:
        """Hinge angle and raw servo step for calibration; None without ROS
        or without a hinge."""

    def drive(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        *,
        source: str = ...,
        client: str | None = None,
    ) -> None:
        """Drive the base (m/s, m/s, rad/s). Stops unless repeated."""

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
