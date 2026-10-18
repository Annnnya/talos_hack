"""Wheel speeds for moving on the wall (#80, #82).

Between two walls the front pair presses on one wall and the rear pair on
the other, so "up" is the front pair rolling one way and the rear pair the
other (the signs of `expansion.wheel_speeds`). Sideways along the walls is
the ground mecanum pattern with the rear pair flipped too, found live: the
plain ground pattern twists the robot. The level trims (`talos_servo.level`)
add a little "up" speed per pair. No ROS here.
"""

from __future__ import annotations

from talos_servo.expansion import Wheels
from talos_servo.level import NO_TRIM, Trim


def wheel_speeds(
    speed: float,
    side: float,
    wheel_radius_m: float,
    max_speed_m_s: float,
    trim: Trim = NO_TRIM,
) -> Wheels:
    """Wheel servo speeds (rad/s, FL, FR, RL, RR, positive rolls forward)
    for `speed` m/s up the wall (+ up) and `side` m/s along it (+ left),
    with the level `trim`. Scaled down together so no wheel passes
    `max_speed_m_s`."""
    fr, lr = trim.front_rear, trim.left_right
    # "Up" speed of each wheel: the front pair and the right pair get the
    # positive trims.
    up = (speed + fr - lr, speed + fr + lr, speed - fr - lr, speed - fr + lr)
    surface = (up[0] - side, up[1] + side, -up[2] - side, -up[3] + side)
    largest = max(abs(v) for v in surface)
    scale = min(1.0, max_speed_m_s / largest) if largest else 1.0
    fl, fr_, rl, rr = (v * scale / wheel_radius_m for v in surface)
    return (fl, fr_, rl, rr)
