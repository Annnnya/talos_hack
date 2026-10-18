"""Clamping (#78): open the hinge until a motor current shows wall contact."""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from talos_servo.clamp import NO_WALL_REASON, Clamp, ContactDetector, Outcome
from talos_servo.config import load_config
from talos_servo.expansion import STILL, Expansion

CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot.yaml"
DT = 0.02
ALPHA = math.radians(120)
HOLD = 0.1
GRACE = 0.3


@pytest.fixture
def config():
    config = load_config(CONFIG)
    settings = replace(
        config.expansion,
        hinge_contact_current_a=1.0,
        wheel_contact_current_a=0.5,
        contact_hold_s=HOLD,
        contact_grace_s=GRACE,
    )
    return replace(config, expansion=settings)


@pytest.fixture
def clamp(config) -> Clamp:
    expansion = Expansion(config.model, config.base, config.expansion)
    return Clamp(
        expansion,
        config.expansion,
        config.model.hinge_joint,
        tuple(config.base.wheels),
    )


def run(clamp: Clamp, currents: dict, start: float, until: float) -> list:
    """Step every DT from `start` to `until` at ALPHA."""
    steps = []
    now = start
    while now <= until + 1e-9:
        steps.append(clamp.step(ALPHA, currents, now, DT))
        now += DT
    return steps


def test_opens_at_the_clamp_rate(clamp, config):
    step = clamp.step(ALPHA, {}, 0.0, DT)
    rate = config.expansion.clamp_rate_rad_s
    assert step.goal == pytest.approx(ALPHA + rate * DT)
    assert step.rate == pytest.approx(rate)
    assert step.wheels[0] > 0 > step.wheels[2]
    assert clamp.outcome is Outcome.OPENING


def test_hinge_current_means_contact(clamp):
    run(clamp, {}, 0.0, GRACE)
    steps = run(clamp, {"bend": 1.5}, GRACE + DT, GRACE + 2 * DT + HOLD)
    assert clamp.outcome is Outcome.CONTACT
    assert "bend" in clamp.reason
    # Stop pushing: hold at the measured angle with the wheels still.
    touched = next(s for s in steps if s.held == clamp.reason)
    assert touched.goal == pytest.approx(ALPHA)
    assert touched.wheels == STILL
    assert steps[-1].wheels == STILL


def test_one_wheel_current_means_contact(clamp):
    run(clamp, {}, 0.0, GRACE)
    run(clamp, {"rear_left_wheel": -0.8}, GRACE + DT, GRACE + 2 * DT + HOLD)
    assert clamp.outcome is Outcome.CONTACT
    assert "rear_left_wheel" in clamp.reason


def test_start_up_current_is_ignored(clamp):
    run(clamp, {"bend": 2.0}, 0.0, GRACE - DT)
    assert clamp.outcome is Outcome.OPENING


def test_short_spike_is_ignored(clamp):
    run(clamp, {}, 0.0, GRACE)
    run(clamp, {"bend": 2.0}, GRACE + DT, GRACE + HOLD / 2)
    run(clamp, {}, GRACE + HOLD, GRACE + HOLD + DT)
    run(clamp, {"bend": 2.0}, GRACE + 2 * HOLD, GRACE + 2.5 * HOLD)
    assert clamp.outcome is Outcome.OPENING


def test_current_below_the_threshold_is_not_contact(clamp):
    run(clamp, {"bend": 0.9, "front_left_wheel": 0.4}, 0.0, 2.0)
    assert clamp.outcome is Outcome.OPENING


def test_nan_current_is_not_contact(clamp):
    run(clamp, {"bend": math.nan}, 0.0, 2.0)
    assert clamp.outcome is Outcome.OPENING


def test_fully_open_means_no_wall(clamp):
    step = clamp.step(math.pi, {}, 0.0, DT)
    assert clamp.outcome is Outcome.NO_WALL
    assert clamp.reason == NO_WALL_REASON
    assert step.wheels == STILL


def test_goal_reaching_the_limit_means_no_wall(clamp, config):
    near = config.model.bend_max_rad - 1e-4
    clamp.step(near, {}, 0.0, DT)
    step = clamp.step(near, {}, DT, DT)
    assert clamp.outcome is Outcome.NO_WALL
    assert step.wheels == STILL


def test_done_clamp_holds(clamp):
    clamp.step(math.pi, {}, 0.0, DT)
    step = clamp.step(ALPHA, {}, DT, DT)
    assert (step.goal, step.wheels) == (None, STILL)
    assert clamp.outcome is Outcome.NO_WALL


def test_pause_starts_a_new_grace_time(clamp):
    run(clamp, {}, 0.0, GRACE)
    clamp.pause()
    run(clamp, {"bend": 2.0}, 1.0, 1.0 + GRACE - DT)
    assert clamp.outcome is Outcome.OPENING


def test_detector_names_what_touched():
    detector = ContactDetector({"a": 1.0, "b": 2.0}, hold_s=0.0, grace_s=0.0)
    assert detector.update({"a": 0.5, "b": 2.5}, 0.0) == "b current 2.50 A"
