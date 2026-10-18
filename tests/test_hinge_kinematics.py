"""The hinge wheel-speed table (#60) agrees with the wheelbase and config.

Each row is checked against a numeric derivative of `wheelbase_m`, not the
closed form in docs/hinge_kinematics.md, so a slip in the derivation shows.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from talos_servo.config import FLAT_DEG, load_config
from talos_servo.model import wheelbase_m

ROOT = Path(__file__).resolve().parent.parent
TABLE = yaml.safe_load(
    (ROOT / "tests" / "data" / "hinge_wheel_speeds.yaml").read_text()
)
CONFIG = load_config(ROOT / "config" / "robot.yaml")
VECTORS = TABLE["vectors"]

STEP_RAD = 1e-6
# Table values are rounded to 6 decimals.
TOLERANCE = 1e-5


def _wheelbase_rate(alpha_rad: float, leg_m: float) -> float:
    """d(wheelbase)/d(alpha), one-sided at flat where alpha cannot grow."""
    if alpha_rad + STEP_RAD > math.pi:
        ahead = wheelbase_m(alpha_rad, leg_m)
        behind = wheelbase_m(alpha_rad - STEP_RAD, leg_m)
        return (ahead - behind) / STEP_RAD
    ahead = wheelbase_m(alpha_rad + STEP_RAD, leg_m)
    behind = wheelbase_m(alpha_rad - STEP_RAD, leg_m)
    return (ahead - behind) / (2 * STEP_RAD)


@pytest.mark.parametrize(
    "row", VECTORS, ids=lambda r: f"{r['alpha_deg']}deg@{r['alpha_rate_rad_s']}"
)
def test_wheels_roll_with_the_wheelbase(row):
    leg_m = TABLE["leg_length_m"]
    radius_m = TABLE["wheel_radius_m"]
    alpha = math.radians(row["alpha_deg"])
    # The hinge stays centred, so each wheel pair takes half the change.
    pair_speed = _wheelbase_rate(alpha, leg_m) * row["alpha_rate_rad_s"] / 2
    omega = pair_speed / radius_m
    expected = [omega, omega, -omega, -omega]
    assert row["wheels_rad_s"] == pytest.approx(expected, abs=TOLERANCE)


@pytest.mark.parametrize(
    "row", VECTORS, ids=lambda r: f"{r['alpha_deg']}deg@{r['alpha_rate_rad_s']}"
)
def test_front_and_rear_pairs_turn_opposite(row):
    fl, fr, rl, rr = row["wheels_rad_s"]
    assert fl == fr
    assert rl == rr
    assert fl == -rl
    rate = row["alpha_rate_rad_s"]
    if rate and row["alpha_deg"] < FLAT_DEG:
        assert math.copysign(1, fl) == math.copysign(1, rate)


def test_table_uses_the_robot_geometry():
    assert CONFIG.model is not None
    assert TABLE["leg_length_m"] == pytest.approx(CONFIG.model.leg_length_m)
    assert TABLE["wheel_radius_m"] == pytest.approx(CONFIG.base.wheel_radius_m)


def test_table_covers_the_bend_limits():
    assert CONFIG.model is not None
    angles = {row["alpha_deg"] for row in VECTORS}
    assert CONFIG.model.bend_min_deg in angles
    assert CONFIG.model.bend_max_deg in angles
    assert any(
        CONFIG.model.bend_min_deg < a < CONFIG.model.bend_max_deg
        for a in angles
    )
    rates = {row["alpha_rate_rad_s"] for row in VECTORS}
    assert any(r > 0 for r in rates)
    assert any(r < 0 for r in rates)
    assert 0.0 in rates
