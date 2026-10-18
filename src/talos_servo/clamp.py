"""Open the hinge until the wheels press on both walls (#78).

In clamping mode the hinge opens at a slow fixed rate with the wheels rolling
along (`talos_servo.expansion`). Pressing on a wall makes the motors work
harder, so wall contact is when the hinge current or any wheel current stays
over its threshold for `contact_hold_s`. The first `contact_grace_s` after
the motion starts are ignored, since starting draws current too.

On contact the wheels stop and the hinge goal goes to the measured angle, so
it holds there instead of pushing on (the hold force is #79). If the hinge
opens fully without contact, no wall was found.

`ros/expansion_node.py` runs this. No ROS here.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from enum import Enum
from typing import TYPE_CHECKING

from talos_servo.expansion import AT_LIMIT, FULLY_OPEN, STILL, Step

if TYPE_CHECKING:
    from talos_servo.config import ExpansionConfig
    from talos_servo.expansion import Expansion


class Outcome(str, Enum):
    OPENING = "opening"
    CONTACT = "contact"
    NO_WALL = "no wall"


NO_WALL_REASON = "no wall found within the hinge range"


class ContactDetector:
    """Says when a motor current stays over its threshold long enough."""

    def __init__(
        self,
        thresholds: Mapping[str, float],
        hold_s: float,
        grace_s: float,
    ) -> None:
        self.thresholds = dict(thresholds)
        self.hold_s = hold_s
        self.grace_s = grace_s
        self._start: float | None = None
        self._over_since: dict[str, float] = {}

    def reset(self) -> None:
        """Start again, with a new grace time."""
        self._start = None
        self._over_since.clear()

    def update(self, currents: Mapping[str, float], now: float) -> str | None:
        """Feed the latest currents (A, by joint name). Returns what touched,
        or None. A missing or non-finite current never counts."""
        if self._start is None:
            self._start = now
        if now - self._start < self.grace_s:
            return None
        for name, threshold in self.thresholds.items():
            current = abs(currents.get(name, math.nan))
            if not current > threshold:
                self._over_since.pop(name, None)
                continue
            since = self._over_since.setdefault(name, now)
            if now - since >= self.hold_s:
                return f"{name} current {current:.2f} A"
        return None


class Clamp:
    """One clamp: open until contact or the hinge limit."""

    def __init__(
        self,
        expansion: Expansion,
        settings: ExpansionConfig,
        hinge: str,
        wheels: tuple[str, ...],
    ) -> None:
        self.expansion = expansion
        self.rate = settings.clamp_rate_rad_s
        thresholds = {hinge: settings.hinge_contact_current_a}
        thresholds.update((w, settings.wheel_contact_current_a) for w in wheels)
        self.detector = ContactDetector(
            thresholds, settings.contact_hold_s, settings.contact_grace_s
        )
        self.outcome = Outcome.OPENING
        self.reason: str | None = None
        self.expansion.stop()

    @property
    def done(self) -> bool:
        return self.outcome is not Outcome.OPENING

    def pause(self) -> None:
        """Stop for now (STOP, a lost beat). The next step starts the motion
        again, with a new grace time."""
        self.expansion.stop()
        self.detector.reset()

    def step(
        self,
        alpha: float,
        currents: Mapping[str, float],
        now: float,
        dt: float,
    ) -> Step:
        """One control step from the measured angle `alpha` (rad)."""
        if self.done:
            return Step(goal=None, wheels=STILL, held=self.reason)
        touched = self.detector.update(currents, now)
        if touched is not None:
            self.expansion.stop()
            self.outcome = Outcome.CONTACT
            self.reason = f"wall contact: {touched}"
            goal = (
                self.expansion.clamp_angle(alpha)
                if math.isfinite(alpha)
                else None
            )
            return Step(goal=goal, wheels=STILL, held=self.reason)
        step = self.expansion.step(self.rate, alpha, dt)
        if step.held in (FULLY_OPEN, AT_LIMIT):
            self.expansion.stop()
            self.outcome = Outcome.NO_WALL
            self.reason = NO_WALL_REASON
            return Step(goal=step.goal, wheels=STILL, held=self.reason)
        return step
