"""Command line options shared by the nodes that own the servo port (the
robot node and the hardware bridge), so neither node imports the other."""

from __future__ import annotations

import argparse
import os
from collections.abc import Sequence

from talos_servo.bus import DEFAULT_BAUDRATE
from talos_servo.defaults import DEFAULT_CONFIG, PORT_ENV_VAR

# Rate of the per-joint readout topics (temperature, voltage, ...).
DEFAULT_STATUS_HZ = 2.0


def parse_args(
    argv: Sequence[str], *, prog: str
) -> tuple[argparse.Namespace, list[str]]:
    """Options of every node that owns the servo port."""
    parser = argparse.ArgumentParser(prog=prog)
    parser.add_argument(
        "--config", default=DEFAULT_CONFIG, help="robot config (%(default)s)"
    )
    parser.add_argument(
        "--calibration",
        default=None,
        help=(
            "bend calibration file (default: $TALOS_BEND_CALIBRATION, else "
            "bend_calibration.yaml next to --config)"
        ),
    )
    parser.add_argument(
        "--port",
        default=os.environ.get(PORT_ENV_VAR),
        help=f"serial port, e.g. /dev/ttyACM0 (default: ${PORT_ENV_VAR})",
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUDRATE, help="(%(default)s)"
    )
    # Everything else (e.g. --ros-args -p status_hz:=5.0) goes to rclpy.
    return parser.parse_known_args(argv)
