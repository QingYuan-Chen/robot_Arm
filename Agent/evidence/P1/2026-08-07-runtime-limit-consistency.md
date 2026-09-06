# P1 trajectory runtime limit consistency evidence

Date: 2026-08-07

## Limit sources

- Position, hardware effort, hardware velocity ceiling: `rebotarm.urdf`.
- Planner velocity, acceleration, jerk: `rebotarm_moveit_config/config/joint_limits.yaml`.
- Simulated effort ceiling: generated MuJoCo actuator `forcerange`.
- Runtime enforcement: `rebotarm_motion.trajectory_runtime_limits` used by
  `MuJoCoRosAdapterNode`.

## Arm-joint values

| Joints | URDF effort | MuJoCo effort | URDF velocity | MoveIt/runtime velocity | Runtime acceleration | Runtime jerk |
|---|---:|---:|---:|---:|---:|---:|
| joint1-3 | 27 | 27 | 50 | 3.0 | 5.0 | 20.0 |
| joint4-6 | 7 | 7 | 200 | 1.8 | 5.0 | 20.0 |

MoveIt velocity values are conservative planner/runtime limits and remain below the raw URDF
hardware ceilings; they are not expected to equal the URDF values. MuJoCo arm force limits do
match the URDF effort values exactly.

The gripper is deliberately excluded from the arm consistency equality: the current model's
direct finger force actuator and the upstream independent force actuators do not share the same
unit/actuation contract as the URDF motor-side effort field.

## Runtime behavior

The guard checks:

- measured absolute actuator effort;
- measured absolute joint velocity;
- finite-difference acceleration;
- finite-difference jerk.

A violation aborts the MuJoCo FollowJointTrajectory goal, returns a non-success result with the
joint/kind/value/limit in `error_string`, and invokes current-position hold cleanup.

## Verification

```text
tests/test_trajectory_runtime_limits.py: 5 passed
tests/test_mujoco_limit_consistency.py under ROS 2 + MuJoCo: 5 passed
execute-loop integration including runtime violation: 7 passed
MuJoCo node construction: runtime guard loaded with joint1-3=3.0 rad/s,
joint4-6=1.8 rad/s
```

No hardware channel was opened and no real trajectory or gripper command was sent.
