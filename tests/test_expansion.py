"""Hinge goals and wheel speeds for opening and closing the hinge (#76),
checked against the test vectors from #60."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

from talos_servo.config import (  # noqa: E402
    BaseConfig,
    ExpansionConfig,
    ModelConfig,
)
from talos_servo.expansion import STILL, Expansion, wheel_speeds  # noqa: E402

VECTORS = Path(__file__).parent / "data" / "hinge_wheel_speeds.yaml"
TOLERANCE = 1e-5
DT = 0.02
RATE = 0.1


@pytest.fixture(scope="module")
def table() -> dict:
    return yaml.safe_load(VECTORS.read_text(encoding="utf-8"))


def model(table: dict) -> ModelConfig:
    return ModelConfig(
        hinge_joint="bend",
        leg_length_m=table["leg_length_m"],
        leg_gear_ratio=0.5,
        bend_min_deg=36,
        bend_max_deg=180,
    )


def base(table: dict) -> BaseConfig:
    return BaseConfig(
        wheels=("fl", "fr", "rl", "rr"),
        wheelbase_m=2 * table["leg_length_m"],
        track_width_m=0.128,
        wheel_radius_m=table["wheel_radius_m"],
    )


@pytest.fixture
def expansion(table) -> Expansion:
    settings = ExpansionConfig(max_rate_rad_s=0.15, max_lead_rad=0.05)
    return Expansion(model(table), base(table), settings)


def test_wheel_speeds_match_the_test_vectors(table):
    for row in table["vectors"]:
        wheels = wheel_speeds(
            math.radians(row["alpha_deg"]),
            row["alpha_rate_rad_s"],
            table["leg_length_m"],
            table["wheel_radius_m"],
        )
        assert wheels == pytest.approx(row["wheels_rad_s"], abs=TOLERANCE)


def test_step_follows_the_test_vectors(table):
    settings = ExpansionConfig(max_rate_rad_s=0.6, max_lead_rad=0.05)
    for row in table["vectors"]:
        alpha = math.radians(row["alpha_deg"])
        rate = row["alpha_rate_rad_s"]
        step = Expansion(model(table), base(table), settings).step(
            rate, alpha, DT
        )
        at_limit = (rate > 0 and row["alpha_deg"] >= 180) or (
            rate < 0 and row["alpha_deg"] <= 36
        )
        if rate == 0 or at_limit:
            assert step.wheels == STILL
        else:
            assert step.goal == pytest.approx(alpha + rate * DT)
            assert step.wheels == pytest.approx(
                row["wheels_rad_s"], abs=TOLERANCE
            )


def test_expanding_moves_front_forward_and_rear_backward(expansion):
    step = expansion.step(RATE, math.radians(120), DT)
    front_left, front_right, rear_left, rear_right = step.wheels
    assert front_left == front_right > 0
    assert rear_left == rear_right < 0
    assert step.rate == pytest.approx(RATE)


def test_collapsing_turns_the_wheels_the_other_way(expansion):
    step = expansion.step(-RATE, math.radians(120), DT)
    assert step.wheels[0] < 0 < step.wheels[2]


def test_goal_advances_from_the_measured_angle(expansion):
    alpha = math.radians(120)
    first = expansion.step(RATE, alpha, DT)
    second = expansion.step(RATE, alpha + RATE * DT, DT)
    assert first.goal == pytest.approx(alpha + RATE * DT)
    assert second.goal == pytest.approx(alpha + 2 * RATE * DT)


def test_rate_is_capped(expansion):
    step = expansion.step(10.0, math.radians(120), DT)
    assert step.rate == pytest.approx(0.15)


def test_not_finite_rate_stops(expansion):
    step = expansion.step(math.nan, math.radians(120), DT)
    assert (step.goal, step.wheels) == (None, STILL)


def test_refuses_to_expand_when_fully_open(expansion):
    step = expansion.step(RATE, math.pi, DT)
    assert (step.goal, step.wheels) == (None, STILL)
    assert step.held == "hinge fully open"
    # Collapsing from flat still works.
    assert expansion.step(-RATE, math.pi, DT).wheels != STILL


def test_refuses_to_collapse_at_max_fold(expansion):
    step = expansion.step(-RATE, math.radians(36), DT)
    assert (step.goal, step.wheels) == (None, STILL)
    assert step.held == "hinge fully folded"


def test_goal_stops_at_the_limit(expansion):
    alpha = math.pi - 0.001
    step = expansion.step(RATE, alpha, DT)
    assert step.goal == pytest.approx(math.pi)
    # Only the part of the step that fits before the limit turns the wheels.
    assert step.rate == pytest.approx(0.001 / DT)
    held = expansion.step(RATE, alpha, DT)
    assert held.wheels == STILL
    assert held.held == "hinge at its limit"


def test_unknown_angle_holds(expansion):
    step = expansion.step(RATE, math.nan, DT)
    assert (step.goal, step.wheels) == (None, STILL)
    assert step.held == "hinge angle unknown"


def test_waits_for_a_lagging_hinge(expansion):
    alpha = math.radians(120)
    # The hinge does not move while the goal runs ahead.
    steps = [expansion.step(RATE, alpha, DT) for _ in range(40)]
    held = [s for s in steps if s.held == "hinge lags its goal"]
    assert held
    assert all(s.wheels == STILL and s.goal is None for s in held)
    goals = [s.goal for s in steps if s.goal is not None]
    assert max(goals) - alpha <= 0.05 + RATE * DT


def test_stop_restarts_from_the_measured_angle(expansion):
    expansion.step(RATE, math.radians(120), DT)
    assert expansion.stop().wheels == STILL
    alpha = math.radians(130)
    step = expansion.step(RATE, alpha, DT)
    assert step.goal == pytest.approx(alpha + RATE * DT)
