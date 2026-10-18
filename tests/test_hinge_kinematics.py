"""The hinge wheel-speed table (#60) agrees with the robot model and config.

Each row is checked against the kinematic chain of
description/talos.urdf.xacro, not the closed form in docs/hinge_kinematics.md,
so a slip in the derivation shows. A wheel joint sits on its leg, so its rate
is the axle's roll over the ground minus the leg's own turn; both come from
numeric derivatives.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from talos_servo.config import load_config
from talos_servo.model import wheelbase_m

ROOT = Path(__file__).resolve().parent.parent
TABLE = yaml.safe_load(
    (ROOT / "tests" / "data" / "hinge_wheel_speeds.yaml").read_text()
)
CONFIG = load_config(ROOT / "config" / "robot.yaml")
URDF = (ROOT / "description" / "talos.urdf.xacro").read_text()
VECTORS = TABLE["vectors"]

STEP_RAD = 1e-6
# Table values are rounded to 6 decimals.
TOLERANCE = 1e-5
# Leg sign in the URDF: +1 front (points along +x), -1 rear.
FRONT, REAR = 1, -1


def _derivative(f, alpha_rad: float) -> float:
    """df/d(alpha), one-sided at flat where alpha cannot grow."""
    if alpha_rad + STEP_RAD > math.pi:
        return (f(alpha_rad) - f(alpha_rad - STEP_RAD)) / STEP_RAD
    ahead = f(alpha_rad + STEP_RAD)
    behind = f(alpha_rad - STEP_RAD)
    return (ahead - behind) / (2 * STEP_RAD)


def _axle_x_m(alpha_rad: float, sign: int, leg_m: float) -> float:
    """Axle position along x from the hinge; the hinge stays centred."""
    return sign * wheelbase_m(alpha_rad, leg_m) / 2


def _leg_joint_rad(alpha_rad: float, sign: int) -> float:
    """The URDF leg joint: mimics the hinge with multiplier -sign / 2 and
    offset sign * pi / 2, i.e. tilts the leg down by (pi - alpha) / 2."""
    return -sign / 2 * alpha_rad + sign * math.pi / 2


def _wheel_joint_rate(
    alpha_rad: float, rate: float, sign: int, leg_m: float, radius_m: float
) -> float:
    """Wheel joint speed (relative to its leg) that rolls without slip.

    Joint axes are all +y, so world rates add: the wheel turns at
    leg rate + joint rate, and rolling forward at axle speed v needs v / r.
    """
    axle_speed = _derivative(lambda a: _axle_x_m(a, sign, leg_m), alpha_rad)
    leg_rate = _derivative(lambda a: _leg_joint_rad(a, sign), alpha_rad)
    return (axle_speed / radius_m - leg_rate) * rate


@pytest.mark.parametrize(
    "row", VECTORS, ids=lambda r: f"{r['alpha_deg']}deg@{r['alpha_rate_rad_s']}"
)
def test_wheels_follow_the_kinematic_chain(row):
    leg_m = TABLE["leg_length_m"]
    radius_m = TABLE["wheel_radius_m"]
    alpha = math.radians(row["alpha_deg"])
    rate = row["alpha_rate_rad_s"]
    front = _wheel_joint_rate(alpha, rate, FRONT, leg_m, radius_m)
    rear = _wheel_joint_rate(alpha, rate, REAR, leg_m, radius_m)
    expected = [front, front, rear, rear]
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
    # The leg turn keeps the servos moving even at flat.
    if rate:
        assert math.copysign(1, fl) == math.copysign(1, rate)


def test_chain_matches_the_urdf():
    # Wheels hang on the legs, and the legs mimic the hinge as modelled
    # above. If the URDF changes, the table must be rederived.
    assert '<parent link="${leg}"/>' in URDF
    assert 'multiplier="${-sign / 2}"' in URDF
    assert 'offset="${sign * pi / 2}"' in URDF


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
