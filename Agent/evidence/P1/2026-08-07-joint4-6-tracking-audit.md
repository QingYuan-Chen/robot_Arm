# P1 joint4-6 tracking audit

Date: 2026-08-07

## Same high-level command

```text
joint1..joint6 = [1.4, -0.785, -0.785, 0.710, 0.785, 1.570] rad
gripper_width = 0.04 m
```

The current model was tested with a 3-second linear ramp from its zero keyframe. The upstream
model was tested through its native `RebotArmMujoco` controller with the same high-level target.

## Current baseline: 3-second linear ramp

| Joint | Final abs error (rad) | Max abs velocity (rad/s) | Max actuator force |
|---|---:|---:|---:|
| joint1 | 0.0500 | 1.6200 | 25.5993 |
| joint2 | 0.0010 | 0.3568 | 8.0309 |
| joint3 | 0.0323 | 0.7453 | 20.2107 |
| joint4 | 0.3437 | 2.4863 | 7.0 |
| joint5 | 0.7843 | 3.7198 | 7.0 |
| joint6 | 0.0343 | 0.9107 | 7.0 |

## Upstream baseline: same command, 3 seconds

```text
max_final_abs_error = 0.005234 rad
max_abs_velocity = 1.977918 rad/s
max_abs_actuator_force = 20.022315
```

Final positions were approximately:

```text
joint1 1.405234, joint2 -0.787502, joint3 -0.787671,
joint4 0.708409, joint5 0.782922, joint6 1.569930
```

## Interpretation and blocker

The current joint4/joint5 residual remains large even with a retimed linear ramp. This is a
current-model dynamics/calibration issue, not an actuator-contract test failure. The current
arm profile limits joint4-6 to `forcerange=-7 7`; the upstream arm model uses `-12.5 12.5`
torque limits for those joints. Increasing the current limit or changing gains without real
hardware evidence would change the safety and physics baseline, so no automatic parameter change
was made.

P1 tracking/collision/contact calibration remains open pending a decision on whether the current
URDF effort values are authoritative or whether a separately evidenced MuJoCo calibration profile
is allowed.

## Approved upstream arm force profile check

After approval, an isolated `upstream_arm` profile was generated with joint4-6
`forcerange=-12.5 12.5` while retaining the current position-actuator controller, geometry,
gains, damping, and reset state. The same 3-second linear ramp produced:

```text
joint4 final error 0.343706 rad, max force 12.5
joint5 final error 0.784344 rad, max force 12.5
joint6 final error 0.034275 rad, max force 8.9793
```

The joint4/joint5 errors were unchanged from the ±7 current profile. Therefore the upstream force
ceiling alone is not the reason for the upstream baseline's better tracking. The effective
difference is the full upstream torque-control path: cascaded position/velocity control, gravity
compensation, torque rate limiting/filtering, and upstream dynamics. The upstream controller also
clips commanded joint4-6 torque using URDF effort=7 even though its MuJoCo actuator XML allows
±12.5, so copying only the XML actuator ceiling cannot reproduce upstream behavior.

No hardware channel was opened and no real trajectory or gripper command was sent.
