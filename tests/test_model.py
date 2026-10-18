from __future__ import annotations

import math

import pytest

from talos_servo.model import alpha_per_servo_rad, wheelbase_m

LEG_M = 0.155


@pytest.mark.parametrize(
    ("alpha_deg", "expected_m"),
    [
        (180, 0.31),  # flat
        (135, 0.31 * math.sin(math.radians(67.5))),  # half way
        (90, 0.31 * math.sin(math.radians(45))),  # max fold
    ],
)
def test_wheelbase_follows_the_hinge(alpha_deg, expected_m):
    assert wheelbase_m(math.radians(alpha_deg), LEG_M) == pytest.approx(
        expected_m
    )


def test_wheelbase_shrinks_as_the_hinge_folds():
    angles = [math.radians(a) for a in (180, 150, 120, 90)]
    lengths = [wheelbase_m(a, LEG_M) for a in angles]
    assert lengths == sorted(lengths, reverse=True)


@pytest.mark.parametrize(
    ("alpha_rad", "leg_m"),
    [(0.0, LEG_M), (-1.0, LEG_M), (math.pi + 0.1, LEG_M), (math.pi, 0.0)],
)
def test_wheelbase_rejects_impossible_values(alpha_rad, leg_m):
    with pytest.raises(ValueError):
        wheelbase_m(alpha_rad, leg_m)


def test_alpha_moves_one_to_one_with_the_servo():
    assert alpha_per_servo_rad(0.5) == 1.0


def test_alpha_per_servo_rejects_bad_ratio():
    with pytest.raises(ValueError):
        alpha_per_servo_rad(0.0)
