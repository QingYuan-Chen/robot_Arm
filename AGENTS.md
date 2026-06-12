# Repository Instructions for Coding Agents

Before changing code in this repository, read:

- `docs/architecture.md`
- `CONTEXT.md`
- `tests/test_package_layering.py`

The package split is intentional. New code must follow the ownership boundaries
below.

## Hard Rules

- Do not add implementation logic to rebotarm_interactive_control.
- Use `rebotarm_interactive_control` only for compatibility wrappers around old
  import paths or old console scripts.
- New layered packages must not import `rebotarm_interactive_control`.
- Keep hardware access inside `rebotarmcontroller`.
- Keep trajectory generation, validation, MoveIt planning adapters, retiming,
  collision checks, and runtime trajectory guards inside `rebotarm_motion`.
- Keep teach recording, teach file management, prepared trajectory workflow, and
  teach replay orchestration inside `rebotarm_teach`.
- Keep keyboard, web teleop command adapters, gripper teleop adapters, and RViz
  interactive-marker operator input inside `rebotarm_teleop`.
- Keep web UI, HTTP routes, SSE state, and dashboard asset serving inside
  `rebotarm_dashboard`.
- Keep URDF/SRDF/planning groups/collision model configuration inside
  `rebotarm_moveit_config`.
- Keep perception, depth, detection, and grasp candidate logic inside
  `rebotarm_vision`.
- Put hand-eye, TCP, and TF validation tools in `rebotarm_calibration` when that
  package exists.

## Required Checks

After moving package ownership or adding a module, run:

```bash
python -m pytest tests/test_package_layering.py -q
python -m pytest tests -q
python -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion src/rebotarm_interactive_control/rebotarm_interactive_control -q
```

If the change affects launch files, also compile:

```bash
python -m compileall src/rebotarm_bringup/launch -q
```

## Placement Guide

When adding a feature, decide ownership by asking what the feature owns:

- motor mode or final hardware safety: `rebotarmcontroller`
- trajectory math or MoveIt validation: `rebotarm_motion`
- teach replay policy or file workflow: `rebotarm_teach`
- operator command adapter: `rebotarm_teleop`
- web page or dashboard API: `rebotarm_dashboard`
- robot model or MoveIt config: `rebotarm_moveit_config`
- perception or grasp candidate generation: `rebotarm_vision`
- calibration / TF / TCP: `rebotarm_calibration`
- old import compatibility only: `rebotarm_interactive_control`

If one feature crosses multiple responsibilities, split it into small modules
instead of creating a large node that owns the whole workflow.
