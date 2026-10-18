"""Open and close the bend hinge with the wheels rolling along (#76).

When only the hinge moves, the wheels drag and fight the opening angle. So
every step moves the hinge goal a little and turns the wheels at the speed
from docs/hinge_kinematics.md (#60): front pair forward and rear pair
backward to expand, the other way to collapse.

The hardware bridge moves the hinge servo at one fixed speed, so the hinge
speed is set here by streaming small goal steps instead. The goal never runs
more than `max_lead_rad` ahead of the measured angle: if the hinge falls
behind (a wall, a stall) the goal and the wheels wait for it.

`ros/expansion_node.py` wraps this. No ROS here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from talos_servo.config import BaseConfig, ExpansionConfig, ModelConfig

Wheels = tuple[float, float, float, float]
STILL: Wheels = (0.0, 0.0, 0.0, 0.0)


def wheel_speeds(
    alpha: float,
    alpha_rate: float,
    leg_length_m: float,
    wheel_radius_m: float,
) -> Wheels:
    """Wheel servo speeds (rad/s, FL, FR, RL, RR, positive rolls forward)
    that keep the wheels rolling while the hinge opens at `alpha_rate`.

    `alpha` is the opening angle (rad, pi flat). See
    docs/hinge_kinematics.md for the derivation.
    """
    omega = (alpha_rate / 2) * (
        leg_length_m * math.cos(alpha / 2) / wheel_radius_m + 1
    )
    return (omega, omega, -omega, -omega)


@dataclass(frozen=True)
class Step:
    """One control step.

    `goal` is the new hinge goal (rad), or None to leave the hinge where it
    was told last. `rate` is the opening speed the wheels follow, 0 when
    holding. `held` says why the motion is held, or is None while moving or
    when no motion was asked.
    """

    goal: float | None
    wheels: Wheels
    rate: float = 0.0
    held: str | None = None


class Expansion:
    """Turns a requested opening speed into hinge goals and wheel speeds."""

    def __init__(
        self,
        model: ModelConfig,
        base: BaseConfig,
        settings: ExpansionConfig,
    ) -> None:
        self.min_rad = model.bend_min_rad
        self.max_rad = model.bend_max_rad
        self.leg_length_m = model.leg_length_m
        self.wheel_radius_m = base.wheel_radius_m
        self.max_rate = settings.max_rate_rad_s
        self.max_lead = settings.max_lead_rad
        self._goal: float | None = None

    def clamp_rate(self, rate: float) -> float:
        """`rate` limited to +-max_rate_rad_s; anything not finite is 0."""
        if not math.isfinite(rate):
            return 0.0
        return max(-self.max_rate, min(self.max_rate, rate))

    def stop(self) -> Step:
        """Stop the wheels and leave the hinge at its last goal. The next
        motion starts again from the measured angle."""
        self._goal = None
        return Step(goal=None, wheels=STILL)

    def step(self, rate: float, alpha: float, dt: float) -> Step:
        """Move the hinge goal by `rate` (rad/s, + opens) for `dt` seconds
        from the measured angle `alpha` (rad)."""
        rate = self.clamp_rate(rate)
        if rate == 0.0:
            return self.stop()
        if not math.isfinite(alpha):
            return self._hold("hinge angle unknown")
        if rate > 0 and alpha >= self.max_rad:
            return self._hold("hinge fully open")
        if rate < 0 and alpha <= self.min_rad:
            return self._hold("hinge fully folded")
        if self._goal is None:
            self._goal = self._clamp(alpha)
        lead = (self._goal - alpha) * math.copysign(1.0, rate)
        if lead > self.max_lead:
            # The hinge is behind its goal: let it catch up before going on.
            return Step(goal=None, wheels=STILL, held="hinge lags its goal")
        goal = self._clamp(self._goal + rate * dt)
        moved = (goal - self._goal) / dt
        self._goal = goal
        if moved == 0.0:
            return Step(goal=goal, wheels=STILL, held="hinge at its limit")
        wheels = wheel_speeds(
            self._clamp(alpha), moved, self.leg_length_m, self.wheel_radius_m
        )
        return Step(goal=goal, wheels=wheels, rate=moved)

    def _hold(self, why: str) -> Step:
        self._goal = None
        return Step(goal=None, wheels=STILL, held=why)

    def _clamp(self, alpha: float) -> float:
        return max(self.min_rad, min(self.max_rad, alpha))
