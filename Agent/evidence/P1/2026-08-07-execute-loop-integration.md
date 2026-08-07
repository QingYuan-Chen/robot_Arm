# P1 MuJoCo execute-loop integration evidence

Date: 2026-08-07

Scope: `MuJoCoRosAdapterNode._execute_goal` terminal-state and cleanup semantics.

## Covered outcomes

- success -> action succeeded;
- cancel -> action canceled and current-position hold;
- trajectory stop -> action canceled and current-position hold;
- path tolerance violation -> action aborted and current-position hold;
- goal tolerance violation -> action aborted and current-position hold;
- wall-clock timeout -> action aborted, non-success error code, and current-position hold.

## ROS 2 + MuJoCo environment

```text
source /opt/ros/jazzy/setup.bash
source install/setup.bash
third_party/rebotarm_mujoco_venv/bin/python -m pytest \
  tests/test_mujoco_adapter_core.py \
  tests/test_mujoco_ros_adapter_launch.py \
  tests/test_mujoco_execute_loop_integration.py -q

23 passed
```

The six execute-loop cases run with the ROS 2 interfaces and MuJoCo environment available:

```text
third_party/rebotarm_mujoco_venv/bin/python -m pytest \
  tests/test_mujoco_execute_loop_integration.py -q

6 passed
```

The normal system-Python suite skips those six physical-environment integration cases when
`rebotarm_msgs` or MuJoCo is not importable; the explicit ROS 2 + MuJoCo invocation above is
the acceptance evidence.

## Repository regression

```text
tests/test_package_layering.py: 18 passed
full tests: 501 passed, 11 skipped
compileall: passed
git diff --check: passed
```

No hardware channel was opened and no real trajectory or gripper command was sent.
