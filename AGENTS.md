# Repository Instructions for Coding Agents

Before changing code in this repository, read:

- `docs/architecture.md`
- `CONTEXT.md`
- `tests/test_package_layering.py`
- `Agent/README.md`
- `Agent/MEMORY.md`
- `Agent/PROJECT_STATUS.md`
- `Agent/STATE.json`

The package split is intentional. New code must follow the ownership boundaries
below.

## Hard Rules

- Never automatically disable a healthy real arm away from the current run's
  verified baseline because perception, planning, calibration capture, or
  another recoverable task-level check failed. On a recoverable failure, stop
  active motion and keep `enabled hold`, or execute a guarded controlled return
  to the captured baseline, verify the return, and only then disable. If the
  controlled return fails while controller/motor status remains healthy, keep
  enabled hold and require operator recovery; do not fall through to disable.
  Protective disable away from baseline is reserved for critical conditions in
  which holding torque cannot be trusted, such as controller/motor error,
  communication loss, stale feedback, or an explicit emergency-stop request.
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
- Keep MuJoCo models, simulated execution, physics metrics, and contact feedback
  inside `rebotarm_simulation`.
- Keep launch composition and mutually exclusive real/simulation backend
  selection inside `rebotarm_bringup`.
- Put hand-eye, TCP, and TF validation tools in `rebotarm_calibration` when that
  package exists.

## Agent State and Memory

`Agent/` is the live project-state source. Retired root-level plans, usage
guides, and status snapshots remain available in Git history; they must not
override current code evidence or `Agent/STATE.json`.

For every implementation task:

1. At task start, inspect the Agent files and run:

   ```bash
   python3 Agent/update_state.py --event start --actor <agent> --note "<task>"
   ```

2. When a fact, decision, blocker, active phase, or handoff changes, update
   `Agent/MEMORY.md` in the same task.
3. Mark `Agent/PROJECT_STATUS.md` items complete only with code, test, runtime,
   or hardware evidence. Automated tests do not count as hardware acceptance.
4. After a milestone or verification, refresh `Agent/STATE.json` with
   `Agent/update_state.py` and append an activity event.
5. Before handoff, record verification and next steps. Never hand-edit
   `Agent/STATE.json` or rewrite old `Agent/ACTIVITY_LOG.md` entries.

## Required Checks

After moving package ownership or adding a module, run:

```bash
python3 -m pytest tests/test_package_layering.py -q
python3 -m pytest tests -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion src/rebotarm_interactive_control/rebotarm_interactive_control -q
```

If the change affects launch files, also compile:

```bash
python3 -m compileall src/rebotarm_bringup/launch -q
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
- MuJoCo model, simulated controller, physics metric, or contact feedback:
  `rebotarm_simulation`
- launch composition or real/simulation backend selection: `rebotarm_bringup`
- calibration / TF / TCP: `rebotarm_calibration`
- old import compatibility only: `rebotarm_interactive_control`

If one feature crosses multiple responsibilities, split it into small modules
instead of creating a large node that owns the whole workflow.
