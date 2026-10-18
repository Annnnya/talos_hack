"""Keep the central platform level with the IMU while on the wall (#82).

Two P loops on the wheel pairs. Pitch (nose down positive, REP 103) means
the front pair sits lower than the rear pair, so the front pair gets a
little extra "up" speed and the rear pair a little less. Roll (rotation
about x, the left side rising) means the right pair sits lower, so the
right pair gets the extra. Each trim has a deadband, a cap and a sign flag,
since the mount and the leg geometry decide the real direction on the bench.

Past `tilt_fault_rad` on either axis the platform is tilting too far for a
trim to fix: `tilt_fault` reports it (with hysteresis) so the caller stops
the wheels. No ROS here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from talos_servo.config import LevelConfig

TILT_FAULT = "tilt fault"
# The fault clears this far (rad) inside the limit, so it does not flicker.
FAULT_HYSTERESIS_RAD = 0.05


@dataclass(frozen=True)
class Trim:
    """Speed trims in m/s, as "up" speeds on the wall: the front pair gets
    `+front_rear` and the rear pair `-front_rear`; the right pair gets
    `+left_right` and the left pair `-left_right`."""

    front_rear: float = 0.0
    left_right: float = 0.0

    def __bool__(self) -> bool:
        return self.front_rear != 0.0 or self.left_right != 0.0


NO_TRIM = Trim()


class Leveler:
    def __init__(self, settings: LevelConfig) -> None:
        self.settings = settings
        self._faulted = False

    def trim(self, roll: float, pitch: float) -> Trim:
        """Trims for the platform `roll` and `pitch` (rad). Non-finite
        angles give no trim."""
        s = self.settings
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            return NO_TRIM
        return Trim(
            front_rear=self._axis(
                pitch, s.pitch_gain_m_s_per_rad, s.pitch_sign
            ),
            left_right=self._axis(roll, s.roll_gain_m_s_per_rad, s.roll_sign),
        )

    def _axis(self, angle: float, gain: float, sign: float) -> float:
        s = self.settings
        if abs(angle) <= s.deadband_rad:
            return 0.0
        # Act on the part past the deadband, so the trim starts from zero.
        error = angle - math.copysign(s.deadband_rad, angle)
        trim = sign * gain * error
        return max(-s.max_trim_m_s, min(s.max_trim_m_s, trim))

    @property
    def faulted(self) -> bool:
        return self._faulted

    def tilt_fault(self, roll: float, pitch: float) -> str | None:
        """`TILT_FAULT` with the angles while the platform tilts past the
        limit, None once it is back inside the limit minus the hysteresis.
        Unknown angles keep the last answer."""
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            return self._reason(roll, pitch) if self._faulted else None
        worst = max(abs(roll), abs(pitch))
        limit = self.settings.tilt_fault_rad
        if self._faulted:
            self._faulted = worst > limit - FAULT_HYSTERESIS_RAD
        else:
            self._faulted = worst > limit
        return self._reason(roll, pitch) if self._faulted else None

    @staticmethod
    def _reason(roll: float, pitch: float) -> str:
        return (
            f"{TILT_FAULT}: roll {math.degrees(roll):.0f} deg, pitch "
            f"{math.degrees(pitch):.0f} deg"
        )
