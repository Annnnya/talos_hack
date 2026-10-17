from __future__ import annotations

import math

import pytest

from talos_servo.ros.imu_convert import (
    COVARIANCE_NOT_PROVIDED,
    STANDARD_GRAVITY,
    diagonal_covariance,
    dps_to_radps,
    g_to_mps2,
    not_provided_covariance,
)


def test_g_to_mps2():
    assert g_to_mps2((0.0, -0.5, 1.0)) == (0.0, -0.5 * STANDARD_GRAVITY, STANDARD_GRAVITY)


def test_dps_to_radps():
    x, y, z = dps_to_radps((180.0, -90.0, 0.0))
    assert x == pytest.approx(math.pi)
    assert y == pytest.approx(-math.pi / 2)
    assert z == 0.0


def test_diagonal_covariance_is_row_major_3x3():
    assert diagonal_covariance(0.5) == [0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5]


@pytest.mark.parametrize("bad", [-1.0, math.nan, math.inf])
def test_diagonal_covariance_rejects_bad_variance(bad):
    with pytest.raises(ValueError, match="variance"):
        diagonal_covariance(bad)


def test_not_provided_covariance_marks_first_element():
    cov = not_provided_covariance()
    assert len(cov) == 9
    assert cov[0] == COVARIANCE_NOT_PROVIDED
    assert cov[1:] == [0.0] * 8
