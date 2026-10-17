"""`talos-servo` command line tool for bench-testing the servo bus."""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from collections.abc import Sequence

from serial.tools import list_ports

from talos_servo.bus import DEFAULT_ACCELERATION, DEFAULT_BAUDRATE, ServoBus, ServoError

PORT_ENV_VAR = "TALOS_SERVO_PORT"
DEFAULT_SPEED = 100
EXIT_INTERRUPTED = 130


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
    if not ids or any(not 0 <= i <= 253 for i in ids):
        raise argparse.ArgumentTypeError(f"servo ids must be in 0..253: {value!r}")
    return sorted(ids)


def build_parser() -> argparse.ArgumentParser:
    bus_opts = argparse.ArgumentParser(add_help=False)
    bus_opts.add_argument(
        "--port",
        default=os.environ.get(PORT_ENV_VAR),
        help=f"serial port, e.g. COM5 or /dev/ttyACM0 (default: ${PORT_ENV_VAR})",
    )
    bus_opts.add_argument(
        "--baud", type=int, default=DEFAULT_BAUDRATE, help="baud rate (default: %(default)s)"
    )
    bus_opts.add_argument(
        "--ids", type=parse_ids, default=[1], help='servo ids, e.g. "1,2,3" or "1-3" (default: 1)'
    )

    motion_opts = argparse.ArgumentParser(add_help=False)
    motion_opts.add_argument(
        "--acc", type=int, default=DEFAULT_ACCELERATION, help="acceleration (default: %(default)s)"
    )

    parser = argparse.ArgumentParser(prog="talos-servo", description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ports", help="list available serial ports")

    sub.add_parser("ping", parents=[bus_opts], help="check which servos answer")

    spin = sub.add_parser(
        "spin", parents=[bus_opts, motion_opts], help="spin servos in wheel mode, then stop"
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
        parents=[bus_opts, motion_opts],
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


def cmd_spin(args: argparse.Namespace) -> int:
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


COMMANDS = {"ports": cmd_ports, "ping": cmd_ping, "spin": cmd_spin, "monitor": cmd_monitor}


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    if args.command != "ports" and not args.port:
        parser.error(f"no serial port: pass --port or set {PORT_ENV_VAR} (see `talos-servo ports`)")
    try:
        return COMMANDS[args.command](args)
    except ServoError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("interrupted, servos stopped", file=sys.stderr)
        return EXIT_INTERRUPTED


if __name__ == "__main__":
    sys.exit(main())
