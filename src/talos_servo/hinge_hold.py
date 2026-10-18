"""Holding the hinge at a current setpoint while climbing (#142).

The hinge servo stays in position mode. It pushes roughly in proportion to
how far its goal is from where it is, so a PI loop on the current moves the
goal: further open than the measured angle to press the legs harder onto the
walls, back towards it to press less.

    error  = setpoint - |current|        (current filtered)
    offset = ki * integral(error) + kp * error, clamped to 0..max_offset
    goal   = alpha + offset, clamped to the hinge range and rate limited

A positive offset opens the hinge. If the hinge opens by `no_contact_rad`
while the current stays low, there is no wall to press on and the hold stops
itself; the hinge keeps its last goal.

No ROS here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from talos_servo.config import HingeHoldConfig
from talos_servo.hinge import Hinge

# Below this share of the setpoint the legs count as not touching a wall.
NO_CONTACT_SHARE = 0.3
# Above `max_load_pct` the goal backs off as if the current were this many
# A over the setpoint: the servo cuts to 20 % drive after 2 s above 50 %
# load, which would drop a climbing robot.
LOAD_BACKOFF_A = 0.5


@dataclass(frozen=True)
class HoldStep:
    """One loop step. `goal` is None when nothing should be sent."""

    goal: float | None
    measured_a: float
    offset_rad: float
    # Why the hold stopped by itself, once.
    stopped: str | None = None
    # True while the load limit overrides the current loop.
    load_limited: bool = False


class HingeHold:
    def __init__(
        self, settings: HingeHoldConfig, hinge: Hinge, max_rate_rad_s: float
    ) -> None:
        self.settings = settings
        self.hinge = hinge
        self.max_rate = max_rate_rad_s
        self.setpoint_a = 0.0
        self.active = False
        self.measured_a = 0.0
        self._filtered: float | None = None
        self._integral = 0.0
        self._goal: float | None = None
        self._start_alpha = math.nan

    def set_setpoint(self, current_a: float) -> float:
        if not math.isfinite(current_a):
            raise ValueError(f"setpoint must be finite, got {current_a}")
        self.setpoint_a = min(self.settings.max_current_a, max(0.0, current_a))
        return self.setpoint_a

    def start(self, alpha: float) -> None:
        if not math.isfinite(alpha):
            raise ValueError("the hinge angle is unknown")
        self.active = True
        self._integral = 0.0
        self._goal = alpha
        self._start_alpha = alpha

    def stop(self) -> None:
        self.active = False
        self._goal = None

    def measure(self, current_a: float, dt: float) -> float:
        """Filter |current| into `measured_a`; runs even while not holding."""
        if math.isfinite(current_a):
            value = abs(current_a)
            if self._filtered is None:
                self._filtered = value
            else:
                k = dt / (self.settings.filter_s + dt)
                self._filtered += k * (value - self._filtered)
            self.measured_a = self._filtered
        return self.measured_a

    def step(
        self,
        alpha: float,
        current_a: float,
        dt: float,
        load_pct: float | None = None,
    ) -> HoldStep:
        measured = self.measure(current_a, dt)
        if not self.active or self._goal is None:
            return HoldStep(None, measured, 0.0)
        if not math.isfinite(alpha):
            # Keep the last goal until the angle is known again.
            return HoldStep(None, measured, 0.0)
        s = self.settings
        error = self.setpoint_a - measured
        limited = load_pct is not None and load_pct >= s.max_load_pct
        if limited:
            error = min(error, -LOAD_BACKOFF_A)
        self._integral = min(
            s.max_offset_rad, max(0.0, self._integral + s.ki * error * dt)
        )
        offset = min(s.max_offset_rad, max(0.0, self._integral + s.kp * error))
        wanted = min(
            self.hinge.max_rad, max(self.hinge.min_rad, alpha + offset)
        )
        limit = self.max_rate * dt
        goal = self._goal + min(limit, max(-limit, wanted - self._goal))
        self._goal = goal
        opened = alpha - self._start_alpha
        if (
            opened > s.no_contact_rad
            and measured < NO_CONTACT_SHARE * self.setpoint_a
        ):
            self.stop()
            return HoldStep(None, measured, 0.0, stopped="no wall contact")
        return HoldStep(goal, measured, goal - alpha, load_limited=limited)
