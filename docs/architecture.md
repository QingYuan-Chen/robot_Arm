# reBotArm ROS2 Architecture

This repository is organized as layered ROS2 packages. New code must follow
these ownership boundaries instead of adding more logic to the legacy
`rebotarm_interactive_control` package.

## Ownership Rules

### Hardware ownership

`rebotarmcontroller` owns real hardware communication and last-line safety.

It is responsible for:

- motor SDK / serial channel access
- arm and gripper state publication
- `follow_joint_trajectory`
- `trajectory_stop`
- `safe_home`
- enable / disable
- gripper execution
- rejecting unsafe or malformed low-level commands

It must not own:

- web UI
- teach file management
- visual grasp policy
- MoveIt planning policy
- user-facing workflow state

### Motion ownership

`rebotarm_motion` owns motion generation and motion validation.

It is responsible for:

- point-to-point preview and execution nodes
- MoveIt planning client adapters
- pose preview / IK preview helpers
- trajectory retiming
- velocity / acceleration / jerk checks
- replay runtime tracking guard
- collision precheck
- start alignment for teach replay
- `JointTrajectory` construction utilities

It may call MoveIt services and controller actions, but it must not talk
directly to the motor SDK.

### Teach ownership

`rebotarm_teach` owns the teach workflow.

It is responsible for:

- gravity-comp teach recording
- teach record file format and file listing
- prepared trajectory generation
- teach replay service / node entry points
- teach replay dry-run / execute gating
- teach replay settings and replay status payloads

It may use `rebotarm_motion` for retiming, alignment, collision checks, and
trajectory validation. It must not implement dashboard HTML or direct motor SDK
logic.

### Operator interaction ownership

`rebotarm_teleop` owns operator command adapters.

It is responsible for:

- keyboard teleop
- web teleop command validation / adapter logic
- gripper teleop adapter logic
- RViz interactive marker input
- RViz gripper visual joint-state bridge
- legacy interactive target node compatibility

It may publish target commands or call controller-facing ROS actions/services.
It must not own teach replay quality policy or dashboard rendering.

### Dashboard ownership

`rebotarm_dashboard` owns the web application boundary.

It is responsible for:

- HTML / JS / CSS assets
- HTTP routes
- SSE status stream
- dashboard state aggregation
- calling teleop / teach / controller services
- dashboard-specific URDF and mesh serving

It must not contain complex motion planning, retiming, teach replay algorithms,
or hardware SDK code. The dashboard may display motion and teach results, but
the algorithms live in `rebotarm_motion` and `rebotarm_teach`.

### MoveIt configuration ownership

`rebotarm_moveit_config` owns only MoveIt model and planning configuration.

It is responsible for:

- URDF/SRDF used by MoveIt
- planning groups
- end effector configuration
- collision geometry
- joint limits for planning
- RViz MotionPlanning configuration

It must not contain executable business logic.

### Vision ownership

`rebotarm_vision` owns perception and grasp candidates.

It is responsible for:

- camera / depth inputs
- object detection
- depth fusion
- grasp candidate generation
- grasp pose scoring
- visual grasp executor integration points

It must not bypass motion validation or controller safety. A visual grasp must
go through planning, collision checking, and execution gates.

### Calibration ownership

`rebotarm_calibration` is the intended owner for calibration tools.

It is responsible for:

- hand-eye calibration
- TCP calibration
- TF validation tools
- camera intrinsic / extrinsic checks

Calibration outputs should be consumed by vision and motion layers through
configuration or TF, not copied into dashboard or controller code.

### Compatibility layer

`rebotarm_interactive_control` is now a compatibility layer.

It keeps old imports and old console scripts working through wrappers. New
implementation code must not be added there unless the change is explicitly a
compatibility shim.

Layered packages must not import rebotarm_interactive_control:

- `rebotarm_motion`
- `rebotarm_teach`
- `rebotarm_teleop`
- `rebotarm_dashboard`

## Dependency Direction

Allowed dependency direction:

```text
rebotarm_dashboard
  -> rebotarm_teleop
  -> rebotarm_teach
  -> rebotarm_motion
  -> MoveIt / ROS messages / controller actions

rebotarm_teach -> rebotarm_motion
rebotarm_teleop -> rebotarm_motion when using legacy interactive preview helpers
rebotarm_vision -> rebotarm_motion / MoveIt interfaces for validation and execution
```

Forbidden dependency direction:

```text
rebotarm_motion -> rebotarm_dashboard
rebotarm_motion -> rebotarm_teach
rebotarm_motion -> rebotarm_teleop
rebotarm_motion -> rebotarm_interactive_control

rebotarm_teach -> rebotarm_dashboard
rebotarm_teach -> rebotarm_interactive_control

rebotarm_teleop -> rebotarm_dashboard
rebotarm_teleop -> rebotarm_interactive_control

rebotarm_dashboard -> motor SDK
rebotarm_vision -> motor SDK
```

## Authority Matrix

| Package | May directly command hardware | May call controller ROS services/actions | May call MoveIt | May own files/UI | May publish operator targets |
| --- | --- | --- | --- | --- | --- |
| `rebotarmcontroller` | yes | owns them | no | no | no |
| `rebotarm_motion` | no | yes | yes | no | no |
| `rebotarm_teach` | no | yes, through replay/record workflows | yes, through motion helpers | teach records only | no |
| `rebotarm_teleop` | no | yes | only through motion helpers | no | yes |
| `rebotarm_dashboard` | no | yes | no direct planning logic | dashboard assets only | via teleop adapters |
| `rebotarm_moveit_config` | no | no | configuration only | model/config files only | no |
| `rebotarm_vision` | no | only through planned execution interfaces | yes, for validation/execution gates | perception assets/models only | no |
| `rebotarm_calibration` | no | no, except explicit validation tools | no, except validation tools | calibration outputs only | no |
| `rebotarm_interactive_control` | no | no new logic | no new logic | compatibility only | no new logic |

If a package needs authority outside its row, create a small interface in the
owning package and call that interface. Do not copy the implementation across
layers.

## Workflow Boundaries

### Point-to-point execution

Point-to-point execution means moving from the current robot state to one target
state. It is owned by `rebotarm_motion`.

Required properties:

- target state is validated before execution
- generated output is a valid `JointTrajectory`
- final target velocity is zero
- controller stop path remains available
- hardware execution goes through `rebotarmcontroller`

### Teach replay

Teach replay means reproducing a recorded teach trajectory safely. It is owned
by `rebotarm_teach` with motion services from `rebotarm_motion`.

Required properties:

- raw records are treated as input data, not directly trusted execution commands
- prepared trajectory is used for replay
- retiming enforces velocity / acceleration / jerk limits
- collision precheck can block replay
- runtime tracking guard can stop replay
- final hold uses zero velocity

### Web teleop

Web teleop means the dashboard sends operator-intended joint or gripper targets.
The dashboard owns UI; `rebotarm_teleop` owns command adaptation; the controller
owns hardware execution.

Required properties:

- dashboard does not build low-level motor commands
- stop / safe_home / enable / disable call controller-facing services
- replay state can lock unsafe arm commands
- web preview and execute are separate concepts unless explicitly confirmed

### RViz MoveIt Drag Control

RViz drag control is now the native MoveIt MotionPlanning workflow. It does not
use the retired custom `ee_target` marker, `PreviewNode`, `ExecutionNode`, or
`/interactive_control/execute_preview` service.

Current split:

- RViz MotionPlanning creates the goal pose and requests a MoveIt plan
- MoveIt `move_group` computes the trajectory
- `rebotarmcontroller` executes the resulting `FollowJointTrajectory`

## Where New Code Goes

Use this table before adding a file:

| New feature | Package |
| --- | --- |
| New hardware service, motor mode, safe stop behavior | `rebotarmcontroller` |
| New trajectory validator, retimer, planner adapter | `rebotarm_motion` |
| New teach file operation or replay policy | `rebotarm_teach` |
| New keyboard/web/gripper/RViz operator command adapter | `rebotarm_teleop` |
| New web panel, route, SSE payload formatting | `rebotarm_dashboard` |
| New URDF/SRDF/collision/planning group config | `rebotarm_moveit_config` |
| New detection/depth/grasp candidate logic | `rebotarm_vision` |
| New hand-eye/TCP/TF check tool | `rebotarm_calibration` |
| Old import path compatibility only | `rebotarm_interactive_control` |

If a feature seems to belong in multiple packages, split it by responsibility
instead of making one large node own the whole workflow.

## Testing Rules

Architecture rules are guarded by `tests/test_package_layering.py`.

When adding new modules:

- add unit tests for pure logic
- add package-layering tests when changing ownership
- keep legacy wrapper imports tested if an old path must continue working
- run `python -m pytest tests -q`
- run `python -m compileall` on changed Python packages
