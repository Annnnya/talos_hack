"""Argparse type converters shared by the CLI and the ROS node entry points."""

from __future__ import annotations

import argparse

from talos_servo.bus import MAX_SERVO_ID


def parse_ids(value: str) -> list[int]:
    """Parse "1,2,3" or "1-3" (or a mix) into a sorted list of unique servo
    IDs."""
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
        raise argparse.ArgumentTypeError(
            f"invalid servo id list: {value!r}"
        ) from exc
    if not ids or any(not 0 <= i <= MAX_SERVO_ID for i in ids):
        raise argparse.ArgumentTypeError(
            f"servo ids must be in 0..{MAX_SERVO_ID}: {value!r}"
        )
    return sorted(ids)


def parse_id(value: str) -> int:
    ids = parse_ids(value)
    if len(ids) != 1:
        raise argparse.ArgumentTypeError(
            f"expected a single servo id: {value!r}"
        )
    return ids[0]


def parse_int_any_base(value: str) -> int:
    """Accept "104" or "0x68"."""
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid number: {value!r}") from exc
