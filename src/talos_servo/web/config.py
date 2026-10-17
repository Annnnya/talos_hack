"""Robot layout for the web UI: which servos exist, their names and safe limits."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from talos_servo.bus import MAX_POSITION, MAX_SERVO_ID, MAX_SPEED

# STS3215 rated no-load speed at 12 V: 0.222 s/60 deg = 45 rpm, about 3000 steps/s.
DEFAULT_MAX_SPEED = 3000
DEFAULT_TELEMETRY_HZ = 20.0


class ConfigError(ValueError):
    """The web config file is invalid."""


@dataclass(frozen=True)
class ServoConfig:
    id: int
    name: str
    min_pos: int = 0
    max_pos: int = MAX_POSITION


@dataclass(frozen=True)
class WebConfig:
    servos: tuple[ServoConfig, ...]
    max_speed: int = DEFAULT_MAX_SPEED
    telemetry_hz: float = DEFAULT_TELEMETRY_HZ


def parse_config(data: dict) -> WebConfig:
    raw_servos = data.get("servos") or []
    if not raw_servos:
        raise ConfigError("config needs at least one entry under 'servos'")
    servos = []
    for raw in raw_servos:
        servo = ServoConfig(
            id=int(raw["id"]),
            name=str(raw.get("name", f"Servo {raw['id']}")),
            min_pos=int(raw.get("min_pos", 0)),
            max_pos=int(raw.get("max_pos", MAX_POSITION)),
        )
        if not 0 <= servo.id <= MAX_SERVO_ID:
            raise ConfigError(f"servo id must be in 0..{MAX_SERVO_ID}, got {servo.id}")
        if not 0 <= servo.min_pos <= servo.max_pos <= MAX_POSITION:
            raise ConfigError(
                f"servo {servo.id}: need 0 <= min_pos <= max_pos <= {MAX_POSITION}, "
                f"got {servo.min_pos}..{servo.max_pos}"
            )
        servos.append(servo)
    ids = [s.id for s in servos]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"duplicate servo ids in config: {ids}")

    max_speed = int(data.get("max_speed", DEFAULT_MAX_SPEED))
    if not 1 <= max_speed <= MAX_SPEED:
        raise ConfigError(f"max_speed must be in 1..{MAX_SPEED}, got {max_speed}")
    telemetry_hz = float(data.get("telemetry_hz", DEFAULT_TELEMETRY_HZ))
    if telemetry_hz <= 0:
        raise ConfigError(f"telemetry_hz must be > 0, got {telemetry_hz}")
    return WebConfig(servos=tuple(servos), max_speed=max_speed, telemetry_hz=telemetry_hz)


def load_config(path: str | Path) -> WebConfig:
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except OSError as exc:
        raise ConfigError(f"could not read config {path}: {exc}") from exc
    return parse_config(data)
