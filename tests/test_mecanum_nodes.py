"""Ctrl+C shutdown of the talos_mecanum nodes. Skipped where ROS messages are missing.

Each node runs as a real subprocess and gets a real SIGINT, because the failure depends on how
rclpy itself handles the signal. No servo port is set, and the drive node only opens the bus on
/base/enable, so these tests never touch a serial port.
"""

from __future__ import annotations

import os
import random
import select
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(sys.platform == "win32", reason="needs POSIX signals and a pty")

pytest.importorskip("rclpy")
pytest.importorskip("geometry_msgs")
pytest.importorskip("std_srvs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("diagnostic_msgs")
pytest.importorskip("std_msgs")

import termios  # noqa: E402

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "talos_mecanum"
READY_TIMEOUT_S = 20.0
EXIT_TIMEOUT_S = 10.0
SPIN_SETTLE_S = 1.0
DRIVE_READY = "Ready; call /base/enable"
TELEOP_READY = "W/A/S/D move"


def node_env(tmp_path: Path) -> dict[str, str]:
    env = dict(os.environ)
    env.pop("TALOS_SERVO_PORT", None)
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [str(PACKAGE_DIR), env.get("PYTHONPATH")]))
    env["PYTHONUNBUFFERED"] = "1"
    env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"
    env["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    # Private domain on localhost only, so a test run never talks to a robot or another run.
    env["ROS_DOMAIN_ID"] = str(random.randint(100, 200))
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    return env


def start_node(module: str, tmp_path: Path, stdin: int | None = None) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        [sys.executable, "-c", f"from talos_mecanum.{module} import main; main()"],
        stdin=stdin,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        env=node_env(tmp_path),
    )


def wait_for_output(proc: subprocess.Popen[bytes], text: str) -> bytes:
    """Read output until `text` shows up; fail if the node exits or takes too long."""
    assert proc.stdout is not None
    output = b""
    deadline = time.monotonic() + READY_TIMEOUT_S
    while text.encode() not in output:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            pytest.fail(f"node never printed {text!r}; output:\n{output.decode(errors='replace')}")
        readable, _, _ = select.select([proc.stdout], [], [], remaining)
        if readable:
            chunk = os.read(proc.stdout.fileno(), 4096)
            if not chunk:
                pytest.fail(f"node exited early; output:\n{output.decode(errors='replace')}")
            output += chunk
    return output


def interrupt(proc: subprocess.Popen[bytes], ready_text: str) -> tuple[int, str]:
    """Wait until ready, send SIGINT, return (exit code, all output)."""
    try:
        early = wait_for_output(proc, ready_text)
        # The ready text prints just before spinning starts. A SIGINT inside rclpy's executor
        # setup makes rclpy itself print an ignored __del__ traceback, which is not our bug.
        time.sleep(SPIN_SETTLE_S)
        proc.send_signal(signal.SIGINT)
        rest, _ = proc.communicate(timeout=EXIT_TIMEOUT_S)
        return proc.returncode, (early + rest).decode(errors="replace")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def assert_clean(code: int, output: str) -> None:
    assert "Traceback" not in output, output
    assert "RCLError" not in output, output
    assert code == 0, output


def test_drive_node_exits_cleanly_on_sigint(tmp_path: Path) -> None:
    code, output = interrupt(start_node("drive_node", tmp_path), DRIVE_READY)
    assert_clean(code, output)


def test_keyboard_teleop_exits_cleanly_and_restores_terminal_on_sigint(tmp_path: Path) -> None:
    # Teleop refuses to run without a terminal, so give it a pty as stdin.
    master, slave = os.openpty()
    try:
        original = termios.tcgetattr(slave)
        proc = start_node("keyboard_teleop_node", tmp_path, stdin=slave)
        code, output = interrupt(proc, TELEOP_READY)
        assert_clean(code, output)
        assert termios.tcgetattr(slave) == original, "teleop left the terminal in cbreak mode"
    finally:
        os.close(slave)
        os.close(master)
