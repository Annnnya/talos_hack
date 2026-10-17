from __future__ import annotations

import pytest

from talos_servo.mecanum import (
    WHEEL_COUNT,
    WHEEL_JOINT_NAMES,
    mecanum_wheel_speeds,
    wheel_accelerations,
)


@pytest.mark.parametrize(
    ("velocity", "expected"),
    [
        ((0.1, 0.0, 0.0), (100, 100, 100, 100)),
        ((-0.1, 0.0, 0.0), (-100, -100, -100, -100)),
        ((0.0, 0.1, 0.0), (-100, 100, 100, -100)),
        ((0.0, 0.0, 1.0), (-100, 100, -100, 100)),
    ],
)
def test_basic_directions(velocity, expected):
    assert (
        mecanum_wheel_speeds(
            *velocity,
            wheelbase_m=0.3,
            track_width_m=0.3,
            steps_per_meter=1000,
            max_speed=100,
        )
        == expected
    )


def test_wheel_directions_reverse_selected_motors():
    assert mecanum_wheel_speeds(
        0.1,
        0.0,
        0.0,
        wheelbase_m=0.3,
        track_width_m=0.3,
        steps_per_meter=1000,
        max_speed=100,
        wheel_directions=(1, -1, 1, -1),
    ) == (100, -100, 100, -100)


def test_speed_limit_scales_mixed_motion_without_changing_ratios():
    assert mecanum_wheel_speeds(
        0.1,
        0.05,
        0.2,
        wheelbase_m=0.3,
        track_width_m=0.3,
        steps_per_meter=2000,
        max_speed=100,
    ) == (-5, 100, 43, 52)


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"wheelbase_m": 0}, "must be positive"),
        ({"track_width_m": -0.1}, "must be positive"),
        ({"steps_per_meter": 0}, "must be positive"),
        ({"max_speed": 0}, "max_speed"),
        ({"wheel_directions": (1, 1, 1)}, "four values"),
    ],
)
def test_invalid_kinematic_parameters_are_rejected(overrides, message):
    parameters = {
        "wheelbase_m": 0.3,
        "track_width_m": 0.3,
        "steps_per_meter": 1000,
        "max_speed": 100,
    }
    parameters.update(overrides)
    with pytest.raises(ValueError, match=message):
        mecanum_wheel_speeds(0.0, 0.0, 0.0, **parameters)


def test_one_unique_joint_name_per_wheel():
    assert len(set(WHEEL_JOINT_NAMES)) == WHEEL_COUNT


def test_accelerations_scale_with_speed_change():
    assert wheel_accelerations((0, 0, 0, 0), (100, 50, -100, 25), 20) == (20, 10, 20, 5)


def test_accelerations_count_reversal_as_larger_change():
    assert wheel_accelerations((100, 0, 0, 0), (-100, 100, 0, 0), 20) == (20, 10, 1, 1)


def test_accelerations_never_zero():
    assert min(wheel_accelerations((0, 0, 0, 0), (1000, 1, 0, 0), 20)) == 1


def test_accelerations_without_change_use_base():
    assert wheel_accelerations((5, 5, 5, 5), (5, 5, 5, 5), 20) == (20, 20, 20, 20)


@pytest.mark.parametrize(
    ("previous", "target", "base"), [((0,), (0, 0), 20), ((0,), (1,), 0), ((0,), (1,), 255)]
)
def test_accelerations_reject_bad_input(previous, target, base):
    with pytest.raises(ValueError):
        wheel_accelerations(previous, target, base)
