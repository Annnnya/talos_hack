# Wheel speeds for opening and closing the hinge

Math for #60, used by the expansion controller (#76).

When only the `bend` hinge moves, the wheels stay still and drag on the
ground, so they fight the opening angle. To open or close smoothly the
wheels must roll exactly as far as the hinge pushes them.

![Inverted V: hinge on top, a leg of length l to each wheel pair, opening
angle alpha, wheelbase d, both pairs move outward at v when
expanding](hinge_kinematics.svg)

## Names

All values come from `config/robot.yaml` through `talos_servo.config`.

- `l`: leg length, hinge to wheel axle. `ModelConfig.leg_length_m`, half of
  `base.wheelbase_m`: 0.155 m
- `r`: wheel radius. `base.wheel_radius_m`: 0.03 m
- `alpha`: opening angle between the legs, the `bend` joint. 180 deg flat,
  smaller when folded
- Leg gear per servo turn: `model.leg_gear_ratio`, 0.5
- Hinge limits: `model.bend_min_deg` to `model.bend_max_deg`, 90 to 180 deg

## Derivation

The front to rear axle distance is (`talos_servo.model.wheelbase_m`)

```
d = 2 * l * sin(alpha / 2)
```

Differentiate in time:

```
dd/dt = l * cos(alpha / 2) * dalpha/dt
```

The robot is symmetric and the hinge stays centred over the wheels, so each
wheel pair takes half of that change, one outward and one the other way:

```
v     = (l / 2) * cos(alpha / 2) * dalpha/dt     (m/s, per pair)
omega = v / r                                    (rad/s, per wheel)
```

The hinge servo drives one gear per leg and both legs turn, so the opening
moves `2 * leg_gear_ratio` times the servo (`alpha_per_servo_rad`). With the
ratio 0.5 that is 1:1, so `dalpha/dt` equals the servo speed.

## Signs

Wheel speeds are in FL, FR, RL, RR order with positive rolling forward
(`joints.*.inverted` already makes forward positive for every wheel).

| Motion | `dalpha/dt` | Front pair | Rear pair |
|---|---|---|---|
| Expand (open) | > 0 | `+omega` forward | `-omega` backward |
| Collapse (fold) | < 0 | `-omega` backward | `+omega` forward |

Left and right wheels of a pair turn the same. This is not a body twist
(front and rear move apart, not together), so the mecanum controller cannot
produce it: #76 sends per-wheel velocities.

## Notes

- `cos(alpha / 2)` goes to 0 at flat (180 deg): near flat the hinge mostly
  lifts the platform and the wheels barely roll. At max fold (90 deg) they
  roll fastest for the same hinge speed.
- The formula holds anywhere in the limits from #64. Stopping at
  `bend_min_deg` and `bend_max_deg` is up to #76.
- Wheel slip, tyre squash and mecanum roller play are ignored.

## Worked example

At max fold, `alpha = 90 deg`, expanding at `dalpha/dt = 0.5 rad/s`:

```
v     = (0.155 / 2) * cos(45 deg) * 0.5 = 0.0775 * 0.7071 * 0.5 = 0.0274 m/s
omega = 0.0274 / 0.03 = 0.913 rad/s
```

So FL, FR = +0.913 rad/s and RL, RR = -0.913 rad/s, the first test vector
below.

## Test vectors

`tests/data/hinge_wheel_speeds.yaml` lists `(alpha, dalpha/dt)` and the
four wheel speeds for flat, half open (135 deg), max fold and points in
between, expanding, collapsing and still. #76 loads it in its tests.
`tests/test_hinge_kinematics.py` checks every row against a numeric
derivative of `wheelbase_m` (not the closed form above) and checks that the
table's `l`, `r` and angle range still match `config/robot.yaml`.
