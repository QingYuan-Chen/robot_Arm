# reBotArm ROS2 架构

## 当前部署范围

当前支持的视觉路线是 Ubuntu 24.04 / ROS 2 Jazzy：原生 Gemini 2、本地 YOLO、ROS RGB-D/CameraInfo/检测结果，以及本机进程内 GraspNet。Windows、HTTP、MJPEG、远程 JSON 和独立 GraspNet 服务路线已经废弃，不得恢复。

本仓库由分层 ROS2 包组成。新代码必须遵守以下职责边界。

## 职责边界

### 硬件职责（Hardware ownership）

`rebotarmcontroller` 负责真实硬件通信和最后一道安全防线。

负责：

- motor SDK / serial channel access
- arm and gripper state publication
- `follow_joint_trajectory`
- `trajectory_stop`
- `safe_home`
- enable / disable
- gripper execution
- rejecting unsafe or malformed low-level commands

不得负责：

- web UI
- teach file management
- visual grasp policy
- MoveIt planning policy
- user-facing workflow state

### 运动职责（Motion ownership）

`rebotarm_motion` 负责运动生成和运动校验。

负责：

- point-to-point preview and execution nodes
- MoveIt planning client adapters
- pose preview / IK preview helpers
- trajectory retiming
- velocity / acceleration / jerk checks
- replay runtime tracking guard
- collision precheck
- start alignment for teach replay
- `JointTrajectory` construction utilities

可以调用 MoveIt 服务和控制器 Action，但不得直接访问电机 SDK。

### 示教职责

`rebotarm_teach` 负责示教工作流。

负责：

- gravity-comp teach recording
- teach record file format and file listing
- prepared trajectory generation
- teach replay workflow entry point
- teach replay dry-run / execute gating
- teach replay settings and replay status payloads

`TeachReplayWorkflow` is the sole replay implementation and owns the replay lifecycle, including
preparation, dry-run tokens, alignment, collision checks, action callbacks and
tracking state. It receives explicit snapshot/status callbacks and ROS adapters;
it neither imports dashboard modules nor controls HTTP command authorization.

The retired `TeachReplayNode`, `teach_replay.launch.py`, and standalone recording
launch are intentionally not maintained as parallel command-line paths. Both
recording and replay are initiated from the Dashboard composition so the same
safety gates and authorization state are always used.

`TeachRecorderNode` is the only recording service/file owner. The controller
publishes an atomic verified batch with a stable timestamp for its receive
identity; publishing that batch again does not create a new recording sample.
Recording checks source age and matching per-motor batch stamps. The configured
recording rate is an upper bound, not a claim that feedback arrived at that rate.

It may use `rebotarm_motion` for retiming, alignment, collision checks, and
trajectory validation. It must not implement dashboard HTML or direct motor SDK
logic.

### 操作交互职责（Operator interaction ownership）

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

### Dashboard 职责（Dashboard ownership）

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

### MoveIt 配置职责

`rebotarm_moveit_config` owns only MoveIt model and planning configuration.

The canonical URDF is `config/rebotarm.urdf`; its mesh URIs resolve to this
package's `meshes/` directory. Bringup, dashboard and simulation consume these
resources without a reverse dependency on bringup or the compatibility package.
Cross-node launch parameter profiles live in `rebotarm_bringup/config`.
Simulation owns its frozen firmware reference and torque calibration values;
changing a hardware profile must not silently retune the simulation.

It is responsible for:

- URDF/SRDF used by MoveIt
- planning groups
- end effector configuration
- collision geometry
- joint limits for planning
- RViz MotionPlanning configuration

It must not contain executable business logic.

### 视觉职责

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

Ready-pose motion (`visual_ready_node` and its parameter profile) lives in
`rebotarm_motion`. The old vision Python/console entry remains a compatibility
alias; bringup launches the motion owner directly.

### 仿真职责

`rebotarm_simulation` owns offline robot physics and the simulated controller
backend.

It is responsible for:

- MuJoCo model generation and validation
- simulated `FollowJointTrajectory` execution
- simulated joint and gripper state
- headless physics checks and optional viewer integration
- trajectory metrics, step-response benchmarks, and simulated contact feedback

It must not import or call the real motor SDK. A simulation launch must not
start `rebotarmcontroller`, open a hardware channel, or expose a second active
`FollowJointTrajectory` server under the same name.

### Bringup 职责

`rebotarm_bringup` owns launch-time composition and backend selection.

It is responsible for:

- launch files and cross-package startup composition
- selecting exactly one real or simulated execution backend
- propagating `use_hardware`, `execution_mode`, and `use_sim_time`
- safe launch defaults and mutually exclusive node conditions

The real controller is composed through the single
`rebotarm_bringup/launch/hardware_controller.launch.py` fragment. Other bringup
launch files may forward public hardware arguments, but must not duplicate the
`reBotArmController` node declaration. See
[launch structure and functions](../src/rebotarm_bringup/launch/README.md).

It must not implement motor control, motion planning, perception, or calibration
algorithms inside launch files.

### 标定职责

`rebotarm_calibration` is the intended owner for calibration tools.

It is responsible for:

- hand-eye calibration
- TCP calibration
- TF validation tools
- camera intrinsic / extrinsic checks

The calibration ROS node owns session files, synchronized capture and solving.
Dashboard owns `/calibration`, HTTP/SSE and ROS clients; it must not read calibration
files or implement calibration mathematics. Explicit gravity-mode operator requests
use existing controller services and do not belong to the calibration solver.

Calibration outputs should be consumed by vision and motion layers through
configuration or TF, not copied into dashboard or controller code.

## 依赖方向

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
rebotarm_bringup -> package launch entry points and configuration only
rebotarm_simulation -> ROS messages / simulated execution libraries only
```

The retired MuJoCo ROS adapter is no longer shipped in the active package;
`rebotarm_simulation` must not import or declare a dependency on `rebotarm_motion`.
Launch interpreters are selected explicitly per process by launch arguments or
environment variables, never by probing workspace virtual-environment directories
or injecting vision site-packages into a whole launch group. See
[launch Python configuration](launch_python_configuration.md).

Forbidden dependency direction:

```text
rebotarm_motion -> rebotarm_dashboard
rebotarm_motion -> rebotarm_teach
rebotarm_motion -> rebotarm_teleop
rebotarm_teach -> rebotarm_dashboard

rebotarm_teleop -> rebotarm_dashboard

rebotarm_dashboard -> motor SDK
rebotarm_vision -> motor SDK
rebotarm_simulation -> motor SDK
rebotarm_simulation -> rebotarmcontroller implementation
rebotarm_bringup -> package implementation internals
```

## 权限矩阵

| Package | May directly command hardware | May call controller ROS services/actions | May call MoveIt | May own files/UI | May publish operator targets |
| --- | --- | --- | --- | --- | --- |
| `rebotarmcontroller` | yes | owns them | no | no | no |
| `rebotarm_motion` | no | yes | yes | no | no |
| `rebotarm_teach` | no | yes, through replay/record workflows | yes, through motion helpers | teach records only | no |
| `rebotarm_teleop` | no | yes | only through motion helpers | no | yes |
| `rebotarm_dashboard` | no | yes | no direct planning logic | dashboard assets only | via teleop adapters |
| `rebotarm_moveit_config` | no | no | configuration only | model/config files only | no |
| `rebotarm_vision` | no | only through planned execution interfaces | yes, for validation/execution gates | perception assets/models only | no |
| `rebotarm_simulation` | simulated backend only | owns simulated equivalents | no direct planning policy | generated simulation artifacts only | no |
| `rebotarm_bringup` | no | no business logic | no business logic | launch/config only | no |
| `rebotarm_calibration` | no | no, except explicit validation tools | no, except validation tools | calibration outputs only | no |

If a package needs authority outside its row, create a small interface in the
owning package and call that interface. Do not copy the implementation across
layers.

## 工作流边界

### 点到点执行

Point-to-point execution means moving from the current robot state to one target
state. It is owned by `rebotarm_motion`.

Required properties:

- target state is validated before execution
- generated output is a valid `JointTrajectory`
- final target velocity is zero
- controller stop path remains available
- hardware execution goes through `rebotarmcontroller`

### 示教回放

Teach replay means reproducing a recorded teach trajectory safely. It is owned
by `rebotarm_teach` with motion services from `rebotarm_motion`.

Required properties:

- raw records are treated as input data, not directly trusted execution commands
- prepared trajectory is used for replay
- retiming enforces velocity / acceleration / jerk limits
- collision precheck can block replay
- runtime tracking guard can stop replay
- final hold uses zero velocity

### Web 遥操作

Web teleop means the dashboard sends operator-intended joint or gripper targets.
The dashboard owns UI; `rebotarm_teleop` owns command adaptation; the controller
owns hardware execution.

Required properties:

- dashboard does not build low-level motor commands
- stop / safe_home / enable / disable call controller-facing services
- replay state can lock unsafe arm commands
- web preview and execute are separate concepts unless explicitly confirmed

### RViz MoveIt 末端拖动

RViz drag control is now the native MoveIt MotionPlanning workflow. It does not
use the retired custom `ee_target` marker, `PreviewNode`, `ExecutionNode`, or
`/interactive_control/execute_preview` service.

Current split:

- RViz MotionPlanning creates the goal pose and requests a MoveIt plan
- MoveIt `move_group` computes the trajectory
- `rebotarmcontroller` executes the resulting `FollowJointTrajectory`

## 新代码归属

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
| New MuJoCo model, simulated controller, physics metric, or contact feedback | `rebotarm_simulation` |
| New launch composition or mutually exclusive backend selection | `rebotarm_bringup` |
| New hand-eye/TCP/TF check tool | `rebotarm_calibration` |

If a feature seems to belong in multiple packages, split it by responsibility
instead of making one large node own the whole workflow.

## 测试规则

Architecture rules are guarded by `tests/test_package_layering.py`.

When adding new modules:

- add unit tests for pure logic
- add package-layering tests when changing ownership
- run `python -m pytest tests -q`
- run `python -m compileall` on changed Python packages
