"""Sanity checks for config/ekf.yaml. A *_config list with 14 or 16 entries, or a flag one slot
off, still loads in robot_localization but fuses the wrong quantity."""

from __future__ import annotations

from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

EKF_CONFIG = Path(__file__).resolve().parents[1] / "config" / "ekf.yaml"

# robot_localization's fixed order for every sensor *_config list.
STATE = (
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
    "vx",
    "vy",
    "vz",
    "vroll",
    "vpitch",
    "vyaw",
    "ax",
    "ay",
    "az",
)
EXPECTED_FUSED = {
    "odom0": {"vx", "vy", "vyaw"},
    "imu0": {"vyaw"},
}


@pytest.fixture(scope="module")
def params() -> dict:
    data = yaml.safe_load(EKF_CONFIG.read_text())
    return data["/**"]["ros__parameters"]


def fused(params: dict, sensor: str) -> set[str]:
    flags = params[f"{sensor}_config"]
    assert len(flags) == len(STATE), f"{sensor}_config needs {len(STATE)} entries"
    assert all(isinstance(flag, bool) for flag in flags)
    return {name for name, on in zip(STATE, flags, strict=True) if on}


@pytest.mark.parametrize("sensor", sorted(EXPECTED_FUSED))
def test_sensor_fuses_the_intended_quantities(params, sensor):
    assert fused(params, sensor) == EXPECTED_FUSED[sensor]


def test_every_sensor_config_is_checked(params):
    sensors = {key.removesuffix("_config") for key in params if key.endswith("_config")}
    assert sensors == set(EXPECTED_FUSED)


def test_topics(params):
    assert params["odom0"] == "/odom"
    assert params["imu0"] == "/imu/data_raw"


def test_planar_odom_frame_with_tf(params):
    assert params["two_d_mode"] is True
    assert params["publish_tf"] is True
    assert params["world_frame"] == params["odom_frame"] == "odom"
    assert params["base_link_frame"] == "base_link"
