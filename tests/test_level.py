"""Level loop maths (#82) and the wall wheel mapping with trims. No ROS."""

from __future__ import annotations

import math

import pytest

from talos_servo.climb import wheel_speeds
from talos_servo.config import LevelConfig
from talos_servo.level import NO_TRIM, Leveler, Trim

R = 0.03
CAP = 0.3
SETTINGS = LevelConfig(
    pitch_gain_m_s_per_rad=0.1,
    roll_gain_m_s_per_rad=0.1,
    max_trim_m_s=0.02,
    deadband_rad=0.02,
    tilt_fault_rad=0.35,
)


def test_level_platform_gets_no_trim():
    assert Leveler(SETTINGS).trim(0.0, 0.0) == NO_TRIM
    assert not NO_TRIM


def test_nose_down_pitch_speeds_the_front_pair_up():
    trim = Leveler(SETTINGS).trim(0.0, 0.12)
    assert trim.front_rear == pytest.approx(0.1 * (0.12 - 0.02))
    assert trim.left_right == 0.0
    fl, fr, rl, rr = wheel_speeds(0.0, 0.0, R, CAP, trim)
    # Front pair rolls "up" (forward), rear pair "up" too, which for the
    # rear pair is rolling backward, so it reads as the same sign change.
    assert fl == pytest.approx(trim.front_rear / R)
    assert fr == pytest.approx(trim.front_rear / R)
    assert rl == pytest.approx(trim.front_rear / R)
    assert rr == pytest.approx(trim.front_rear / R)


def test_right_side_low_roll_speeds_the_right_pair_up():
    trim = Leveler(SETTINGS).trim(0.12, 0.0)
    assert trim.left_right == pytest.approx(0.1 * 0.1)
    fl, fr, rl, rr = wheel_speeds(0.0, 0.0, R, CAP, trim)
    # Right pair up: FR forward, RR backward; left pair down: the reverse.
    assert fr == pytest.approx(trim.left_right / R)
    assert rr == pytest.approx(-trim.left_right / R)
    assert fl == pytest.approx(-trim.left_right / R)
    assert rl == pytest.approx(trim.left_right / R)


def test_trims_add_to_the_climb_speed():
    trim = Trim(front_rear=0.01, left_right=0.0)
    fl, _fr, rl, _rr = wheel_speeds(0.1, 0.0, R, CAP, trim)
    assert fl == pytest.approx(0.11 / R)
    assert rl == pytest.approx(-0.09 / R)


def test_deadband_cap_and_signs():
    leveler = Leveler(SETTINGS)
    assert leveler.trim(0.019, -0.019) == NO_TRIM
    assert leveler.trim(0.0, 1.0).front_rear == SETTINGS.max_trim_m_s
    assert leveler.trim(-1.0, 0.0).left_right == -SETTINGS.max_trim_m_s
    flipped = Leveler(
        LevelConfig(
            **{**SETTINGS.__dict__, "pitch_sign": -1.0, "roll_sign": -1.0}
        )
    )
    assert flipped.trim(0.12, 0.12).front_rear < 0
    assert flipped.trim(0.12, 0.12).left_right < 0


def test_unknown_angles_give_no_trim():
    assert Leveler(SETTINGS).trim(math.nan, 0.1) == NO_TRIM


def test_tilt_fault_has_hysteresis():
    leveler = Leveler(SETTINGS)
    assert leveler.tilt_fault(0.0, 0.3) is None
    fault = leveler.tilt_fault(0.0, 0.4)
    assert fault and fault.startswith("tilt fault")
    # Just under the limit is still faulted; well under clears it.
    assert leveler.tilt_fault(0.0, 0.33) is not None
    assert leveler.tilt_fault(0.0, 0.25) is None


def test_wheel_speeds_without_trim_match_the_wall_pattern():
    fl, fr, rl, rr = wheel_speeds(0.06, 0.03, R, CAP)
    assert (fl, fr, rl, rr) == pytest.approx(
        (0.03 / R, 0.09 / R, -0.09 / R, -0.03 / R)
    )


def test_wheel_speeds_scale_down_to_the_cap():
    speeds = wheel_speeds(0.3, 0.3, R, CAP)
    assert max(abs(v) for v in speeds) == pytest.approx(CAP / R)
