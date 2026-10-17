"""`talos-servo` command line tool for bench-testing the servo bus."""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections.abc import Callable, Sequence
from typing import Any

from serial.tools import list_ports

from talos_servo.bus import (
    DEFAULT_ACCELERATION,
    DEFAULT_BAUDRATE,
    MAX_SERVO_ID,
    ServoBus,
    ServoError,
)
from talos_servo.imu import (
    ACCEL_RANGES,
    DEFAULT_ACCEL_RANGE,
    DEFAULT_ADDRESS,
    DEFAULT_GYRO_RANGE,
    DEFAULT_I2C_BUS,
    GYRO_RANGES,
    ImuError,
    Mpu6050,
)
from talos_servo.web.backend import DEFAULT_BRIDGE

PORT_ENV_VAR = "TALOS_SERVO_PORT"
IMU_BUS_ENV_VAR = "TALOS_IMU_BUS"
DEFAULT_SPEED = 100
EXIT_INTERRUPTED = 130
DEFAULT_WEB_CONFIG = "config/web.yaml"
DEFAULT_UI_DIR = "web/out"


def parse_ids(value: str) -> list[int]:
    """Parse "1,2,3" or "1-3" (or a mix) into a sorted list of unique servo IDs."""
    ids: set[int] = set()
    try:
        for part in value.split(","):
            part = part.strip()
            if "-" in part:
                start, end = (int(x) for x in part.split("-", 1))
                ids.update(range(start, end + 1))
            elif part:
                ids.add(int(part))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid servo id list: {value!r}") from exc
    if not ids or any(not 0 <= i <= MAX_SERVO_ID for i in ids):
        raise argparse.ArgumentTypeError(f"servo ids must be in 0..{MAX_SERVO_ID}: {value!r}")
    return sorted(ids)


def parse_id(value: str) -> int:
    ids = parse_ids(value)
    if len(ids) != 1:
        raise argparse.ArgumentTypeError(f"expected a single servo id: {value!r}")
    return ids[0]


def parse_int_any_base(value: str) -> int:
    """Accept "104" or "0x68"."""
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number: {value!r}") from exc


def build_parser() -> argparse.ArgumentParser:
    port_opts = argparse.ArgumentParser(add_help=False)
    port_opts.add_argument(
        "--port",
        default=os.environ.get(PORT_ENV_VAR),
        help=f"serial port, e.g. COM5 or /dev/ttyACM0 (default: ${PORT_ENV_VAR})",
    )
    port_opts.add_argument(
        "--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)"
    )

    bus_opts = argparse.ArgumentParser(add_help=False, parents=[port_opts])
    bus_opts.add_argument(
        "--ids", type=parse_ids, default=[1], help='servo ids, e.g. "1,2,3" or "1-3" (default: 1)'
    )

    motion_opts = argparse.ArgumentParser(add_help=False)
    motion_opts.add_argument(
        "--acc", type=int, default=DEFAULT_ACCELERATION, help="acceleration (default: %(default)s)"
    )

    ros_opts = argparse.ArgumentParser(add_help=False)
    ros_opts.add_argument(
        "--ros",
        action="store_true",
        help="go through the running ROS servo bridge instead of the serial port, so this can run "
        "next to it and the web UI (needs the pixi ros environment; --port is ignored)",
    )
    ros_opts.add_argument(
        "--bridge",
        default=DEFAULT_BRIDGE,
        help="servo bridge namespace for --ros (default: %(default)s)",
    )
    ros_opts.add_argument(
        "--config", default=DEFAULT_WEB_CONFIG, help="servo layout file (default: %(default)s)"
    )

    parser = argparse.ArgumentParser(prog="talos-servo", description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="list available serial ports")

    sub.add_parser("ping", parents=[bus_opts], help="check which servos answer")

    spin = sub.add_parser(
        "spin",
        parents=[bus_opts, motion_opts, ros_opts],
        help="spin servos in wheel mode, then stop",
    )
    spin.add_argument(
        "--speed",
        type=int,
        default=DEFAULT_SPEED,
        help="steps/s, negative reverses (default: %(default)s)",
    )
    spin.add_argument("--seconds", type=float, default=1.0, help="run time (default: %(default)s)")

    monitor = sub.add_parser(
        "monitor",
        parents=[bus_opts, motion_opts, ros_opts],
        help="print position and speed, optionally while spinning",
    )
    monitor.add_argument(
        "--speed", type=int, default=0, help="spin at this speed while monitoring (default: 0)"
    )
    monitor.add_argument(
        "--seconds", type=float, default=None, help="stop after this long (default: until Ctrl+C)"
    )
    monitor.add_argument(
        "--interval", type=float, default=1.0, help="seconds between reads (default: %(default)s)"
    )

    set_id = sub.add_parser(
        "set-id",
        parents=[port_opts],
        help="permanently change a servo's id (refuses if the new id is already in use)",
    )
    set_id.add_argument("--id", type=parse_id, required=True, help="current servo id")
    set_id.add_argument("--new-id", type=parse_id, required=True, help="id to assign")

    web = sub.add_parser(
        "web",
        parents=[port_opts, ros_opts],
        help="serve the browser control UI (needs the web extra)",
    )
    web.add_argument("--mock", action="store_true", help="simulate the servos, no serial port")
    web.add_argument(
        "--host", default="0.0.0.0", help="address to listen on (default: %(default)s)"
    )
    web.add_argument("--http-port", type=int, default=8000, help="http port (default: %(default)s)")
    web.add_argument("--ui-dir", default=DEFAULT_UI_DIR, help="built web UI (default: %(default)s)")

    imu = sub.add_parser("imu", help="stream MPU6050 accel, gyro and temperature over i2c")
    imu.add_argument(
        "--i2c-bus",
        type=int,
        default=os.environ.get(IMU_BUS_ENV_VAR, str(DEFAULT_I2C_BUS)),
        help=f"i2c bus number, /dev/i2c-N (default: ${IMU_BUS_ENV_VAR} or {DEFAULT_I2C_BUS})",
    )
    imu.add_argument(
        "--address",
        type=parse_int_any_base,
        default=DEFAULT_ADDRESS,
        help=f"i2c address, 0x69 if AD0 is high (default: 0x{DEFAULT_ADDRESS:02x})",
    )
    imu.add_argument(
        "--accel-range",
        type=int,
        choices=sorted(ACCEL_RANGES),
        default=DEFAULT_ACCEL_RANGE,
        help="accelerometer full scale in g (default: %(default)s)",
    )
    imu.add_argument(
        "--gyro-range",
        type=int,
        choices=sorted(GYRO_RANGES),
        default=DEFAULT_GYRO_RANGE,
        help="gyro full scale in deg/s (default: %(default)s)",
    )
    imu.add_argument(
        "--interval", type=float, default=0.1, help="seconds between reads (default: %(default)s)"
    )
    imu.add_argument(
        "--seconds", type=float, default=None, help="stop after this long (default: until Ctrl+C)"
    )
    imu.add_argument(
        "--calibrate", action="store_true", help="zero the gyro first (keep the sensor still)"
    )
    return parser


def cmd_ports(_args: argparse.Namespace) -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("no serial ports found")
        return 1
    for p in ports:
        print(f"{p.device}\t{p.description}")
    return 0


def cmd_ping(args: argparse.Namespace) -> int:
    failed = 0
    with ServoBus(args.port, args.baud) as bus:
        for servo_id in args.ids:
            try:
                print(f"servo {servo_id}: ok (model {bus.ping(servo_id)})")
            except ServoError as exc:
                failed += 1
                print(f"servo {servo_id}: {exc}")
    return 1 if failed else 0


def _ros_missing(exc: ImportError, example: str) -> int:
    print(
        f"error: --ros needs ROS 2 ({exc.name} missing): run it in the pixi ros environment, "
        f"e.g. `{example}`",
        file=sys.stderr,
    )
    return 1


def _over_ros(args: argparse.Namespace, run: Callable[[Any, Any], None]) -> int:
    """Run `run(cli_commands, backend)` for a spin or monitor command with --ros."""
    try:
        from talos_servo.ros import cli_commands
        from talos_servo.web.config import ConfigError, load_config
        from talos_servo.web.controller import CommandError
    except ImportError as exc:
        return _ros_missing(exc, f"pixi run -e ros talos-servo {args.command} --ros ...")
    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    if args.acc != DEFAULT_ACCELERATION:
        print("note: --acc is ignored with --ros, the servo bridge has no acceleration input")
    try:
        run(cli_commands, cli_commands.RosBackend(config, bridge=args.bridge))
    except CommandError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


def cmd_spin(args: argparse.Namespace) -> int:
    if args.ros:
        return _over_ros(
            args, lambda m, backend: m.spin_over_ros(backend, args.ids, args.speed, args.seconds)
        )
    with ServoBus(args.port, args.baud) as bus:
        bus.enable_wheel_mode(args.ids)
        print(f"spinning {args.ids} at speed {args.speed} for {args.seconds}s")
        for servo_id in args.ids:
            bus.set_speed(servo_id, args.speed, args.acc)
        time.sleep(args.seconds)
        # Stop explicitly so a stop failure is reported, not just logged on exit.
        for servo_id in args.ids:
            bus.stop(servo_id, args.acc)
    print("stopped")
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    if args.ros:
        return _over_ros(
            args,
            lambda m, backend: m.monitor_over_ros(
                backend, args.ids, args.speed, args.seconds, args.interval
            ),
        )
    with ServoBus(args.port, args.baud) as bus:
        if args.speed:
            bus.enable_wheel_mode(args.ids)
            for servo_id in args.ids:
                bus.set_speed(servo_id, args.speed, args.acc)
        deadline = None if args.seconds is None else time.monotonic() + args.seconds
        while deadline is None or time.monotonic() < deadline:
            for servo_id in args.ids:
                position, speed = bus.read_position_speed(servo_id)
                print(f"servo {servo_id}: position={position} speed={speed}")
            time.sleep(args.interval)
    return 0


def cmd_set_id(args: argparse.Namespace) -> int:
    with ServoBus(args.port, args.baud) as bus:
        bus.set_id(args.id, args.new_id)
    print(f"servo {args.id} is now servo {args.new_id}")
    return 0


def cmd_web(args: argparse.Namespace) -> int:
    try:
        from talos_servo.web.config import ConfigError
        from talos_servo.web.server import serve
    except ImportError as exc:
        print(
            f"error: the web command needs the web extra ({exc.name} missing): uv sync --extra web",
            file=sys.stderr,
        )
        return 1
    if args.ros:
        try:
            import talos_servo.ros.web_backend  # noqa: F401
        except ImportError as exc:
            return _ros_missing(exc, "pixi run -e ros web-ros")
    try:
        serve(
            config_path=args.config,
            port=args.port,
            baudrate=args.baud,
            mock=args.mock,
            host=args.host,
            http_port=args.http_port,
            ui_dir=args.ui_dir,
            ros=args.ros,
            bridge=args.bridge,
        )
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


def cmd_imu(args: argparse.Namespace) -> int:
    with Mpu6050(
        args.i2c_bus,
        args.address,
        accel_range=args.accel_range,
        gyro_range=args.gyro_range,
    ) as imu:
        if args.calibrate:
            print("calibrating gyro, keep the sensor still...")
            ox, oy, oz = imu.calibrate_gyro()
            print(f"gyro offset [dps] {ox:+.2f} {oy:+.2f} {oz:+.2f}")
        deadline = None if args.seconds is None else time.monotonic() + args.seconds
        while deadline is None or time.monotonic() < deadline:
            sample = imu.read()
            ax, ay, az = sample.accel
            gx, gy, gz = sample.gyro
            print(
                f"acc[g] {ax:+.2f} {ay:+.2f} {az:+.2f} | "
                f"gyro[dps] {gx:+6.1f} {gy:+6.1f} {gz:+6.1f} | {sample.temperature:.1f}C"
            )
            time.sleep(args.interval)
    return 0


COMMANDS = {
    "ports": cmd_ports,
    "ping": cmd_ping,
    "spin": cmd_spin,
    "monitor": cmd_monitor,
    "set-id": cmd_set_id,
    "web": cmd_web,
    "imu": cmd_imu,
}


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    without_port = getattr(args, "ros", False) or (args.command == "web" and args.mock)
    if args.command == "web" and args.mock and args.ros:
        parser.error(
            "--mock and --ros do not mix: for simulated servos over ROS, run the bridge "
            "with `pixi run -e ros servo-node-mock`"
        )
    needs_port = args.command not in ("ports", "imu") and not without_port
    if needs_port and not args.port:
        parser.error(f"no serial port: pass --port or set {PORT_ENV_VAR} (see `talos-servo ports`)")
    try:
        return COMMANDS[args.command](args)
    except (ServoError, ImuError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print(
            "interrupted" if args.command == "imu" else "interrupted, servos stopped",
            file=sys.stderr,
        )
        return EXIT_INTERRUPTED


if __name__ == "__main__":
    sys.exit(main())
