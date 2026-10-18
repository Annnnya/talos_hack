"""Hinge opening angle from the bend servo steps (#63)."""

from __future__ import annotations

import math

import pytest

from talos_servo.bus import MAX_POSITION, RAD_PER_STEP
from talos_servo.config import ModelConfig
from talos_servo.hinge import FLAT_RAD, Hinge, HingeRangeError

FLAT = 3000
# Opening changes 1:1 with the servo, so 90 deg of fold is a quarter turn.
QUARTER_TURN = 1024


def model(**changes) -> ModelConfig:
    fields = {
        "hinge_joint": "bend",
        "leg_length_m": 0.15,
        "leg_gear_ratio": 0.5,
        "bend_min_deg": 90.0,
        "bend_max_deg": 180.0,
        **changes,
    }
    return ModelConfig(**fields)


@pytest.fixture
def hinge() -> Hinge:
    return Hinge(model(), FLAT)


def test_flat_position_is_pi(hinge):
    assert hinge.alpha(FLAT) == pytest.approx(FLAT_RAD)
    assert hinge.steps(FLAT_RAD) == FLAT


def test_folding_lowers_the_steps(hinge):
    assert hinge.alpha(FLAT - QUARTER_TURN) == pytest.approx(math.pi / 2)
    assert hinge.steps(math.pi / 2) == FLAT - QUARTER_TURN


@pytest.mark.parametrize("steps", [FLAT - QUARTER_TURN, FLAT - 500, FLAT])
def test_round_trip(hinge, steps):
    assert hinge.steps(hinge.alpha(steps)) == steps


def test_goals_clamp_to_the_hinge_limits(hinge):
    assert hinge.steps(math.radians(45)) == FLAT - QUARTER_TURN
    assert hinge.steps(4.0) == FLAT


def test_gear_ratio_scales_the_angle():
    # Leg gears at the servo speed: the opening moves twice the servo.
    hinge = Hinge(model(leg_gear_ratio=1.0), FLAT)
    assert hinge.alpha(FLAT - 100) == pytest.approx(
        FLAT_RAD - 200 * RAD_PER_STEP
    )
    assert hinge.alpha_rate(100) == pytest.approx(200 * RAD_PER_STEP)


def test_step_range_covers_flat_to_max_fold(hinge):
    assert hinge.step_range == (FLAT - QUARTER_TURN, FLAT)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_goals_are_rejected(hinge, bad):
    with pytest.raises(ValueError, match="finite"):
        hinge.steps(bad)


@pytest.mark.parametrize("flat", [QUARTER_TURN - 1, MAX_POSITION + 1])
def test_range_outside_one_turn_is_rejected(flat):
    # Position mode never crosses 4095 -> 0, so there is no wrap to count.
    with pytest.raises(HingeRangeError, match="outside one servo turn"):
        Hinge(model(), flat)


def test_range_that_just_fits_is_accepted():
    assert Hinge(model(), QUARTER_TURN).step_range == (0, QUARTER_TURN)
