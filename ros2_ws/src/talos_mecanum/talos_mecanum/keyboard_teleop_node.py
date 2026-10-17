from __future__ import annotations

import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool, Trigger

KEY_TIMEOUT_S = 0.3
PUBLISH_PERIOD_S = 0.05


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("talos_keyboard_teleop")
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._enable_client = self.create_client(SetBool, "/base/enable")
        self._stop_client = self.create_client(Trigger, "/base/stop")
        self._enabled = False
        self._command = Twist()
        self._last_key_at = 0.0
        self.declare_parameter("linear_speed", 0.1)
        self.declare_parameter("angular_speed", 0.4)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._angular_speed = float(self.get_parameter("angular_speed").value)

    def handle_key(self, key: str) -> bool:
        if key == "x":
            return False
        if key == "m":
            self._toggle_enable()
            self._set_command(0.0, 0.0, 0.0)
            return True
        if key == " ":
            self._set_command(0.0, 0.0, 0.0)
            self._call_stop()
            return True

        velocities = {
            "w": (self._linear_speed, 0.0, 0.0),
            "s": (-self._linear_speed, 0.0, 0.0),
            "a": (0.0, self._linear_speed, 0.0),
            "d": (0.0, -self._linear_speed, 0.0),
            "q": (0.0, 0.0, self._angular_speed),
            "e": (0.0, 0.0, -self._angular_speed),
        }
        if key in velocities and self._enabled:
            self._set_command(*velocities[key])
        return True

    def publish_command(self) -> None:
        if time.monotonic() - self._last_key_at > KEY_TIMEOUT_S:
            self._set_command(0.0, 0.0, 0.0)
        self._publisher.publish(self._command if self._enabled else Twist())

    def _set_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        self._command = Twist()
        self._command.linear.x = linear_x
        self._command.linear.y = linear_y
        self._command.angular.z = angular_z
        self._last_key_at = time.monotonic()

    def _toggle_enable(self) -> None:
        if not self._enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Drive node service /base/enable is unavailable")
            return
        request = SetBool.Request()
        request.data = not self._enabled
        future = self._enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.done() and future.result() is not None:
            result = future.result()
            self._enabled = result.success and request.data
            self.get_logger().info(result.message)
        else:
            self.get_logger().error("Timed out waiting for /base/enable")

    def _call_stop(self) -> None:
        if not self._stop_client.wait_for_service(timeout_sec=0.2):
            return
        self._stop_client.call_async(Trigger.Request())


def main(args: list[str] | None = None) -> None:
    if not sys.stdin.isatty():
        raise RuntimeError("Keyboard teleop requires an interactive terminal")
    # rclpy's own SIGINT handler shuts the context down from another thread, which races the stop
    # published below. Without it, Ctrl+C is a plain KeyboardInterrupt and the context stays valid.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    terminal_state = termios.tcgetattr(sys.stdin.fileno())
    print("W/A/S/D move, Q/E turn, space stops, M toggles drive, X exits")
    print("Each direction key is a short pulse; press repeatedly for continuous movement.")
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], PUBLISH_PERIOD_S)
            if readable and not node.handle_key(sys.stdin.read(1).lower()):
                break
            node.publish_command()
            rclpy.spin_once(node, timeout_sec=0.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Restore the terminal first so a failure below cannot leave it in cbreak mode.
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, terminal_state)
        # If something else shut the context down, the drive node watchdog stops the wheels once
        # /cmd_vel goes quiet.
        if rclpy.ok():
            node._set_command(0.0, 0.0, 0.0)
            node.publish_command()
            if node._enabled:
                node._toggle_enable()
        node.destroy_node()
        rclpy.try_shutdown()
