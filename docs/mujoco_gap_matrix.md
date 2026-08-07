# MuJoCo current/upstream capability gap matrix

> 核验日期：2026-08-06。该矩阵只比较当前仓库证据与可观察的上游能力；它不是把上游代码列为实现依赖，也不把“有示例”误判为“已完成验收”。

## 判定规则

- **keep**：当前实现已经覆盖目标能力，继续维护本仓库实现。
- **implement**：差距明确、属于本包职责，进入当前 P1 队列。
- **defer**：需要真实硬件、标定数据或跨阶段条件，留在 P2-P6，不提前宣称完成。
- **reject**：来源授权或安全边界不满足，禁止复制、修改、分发或作为默认运行依赖。
- **missing**：当前没有足够的代码、测试或运行证据；只作为差距记录，不得勾选验收项。
- `robotarm_ros2` 是当前 P1 主迁移候选；`reBotArm_develop_hjx`（HJX）仅允许行为级观察。所有固定 commit、许可证核验和引用边界见 [`mujoco_upstream_sources.md`](mujoco_upstream_sources.md)。

## Matrix

| Capability | Current evidence | Upstream observation | Decision | Owner | Next action |
|---|---|---|---|---|---|
| model generation and packaging | `rebotarm_simulation/mujoco_model_profile.py` generates the physical profile from package-owned `assets/rebotarm_base.xml`; package data installs both XML files. Model smoke is finite with `nq=8`, `nv=8`, `nu=7`. | `robotarm_ros2` has checked-in `models/rebotarm/robot.xml`, `scene.xml`, an `urdf_to_mjcf.py` generator and model contract tests; its package declares Apache-2.0 but the root license file is not yet verified. Upstream robot XML has `nu=8` and 26 sensors, while current generated XML has `nu=7` and no sensors. | keep current model; migrate generator checks selectively | `rebotarm_simulation` | Preserve the current 7-actuator gripper contract; evaluate staging/determinism tests and sensors separately before any model replacement. |
| headless smoke and metrics | `mujoco_health.py` now performs a package-owned finite-state health check; `mujoco_runner.py`, `mujoco_metrics.py`, CLI smoke/grasp/step-response commands, and focused tests cover finite state, contacts, summaries, and error metrics. | Upstream provides `mujoco_health`, headless CLI duration reporting and acceptance documentation; its result schema differs from the current summary contract. | keep localized health check; adapt only useful fields; keep current metrics | `rebotarm_simulation` | Compare renderer probe, duration/error schema and ROS health wiring before further localization. |
| ROS 2 trajectory adapter | `mujoco_ros_adapter_node.py` and `sim_trajectory_controller_node.py` publish joint states, accept FollowJointTrajectory, expose stop/gripper endpoints, and have launch tests. | Upstream exposes the same action/service family plus `/clock`, `mujoco_sim.py` and a ROS launch; clock ownership and parameter names differ. | compare then localize | `rebotarm_simulation` | Map endpoint/clock behavior without creating a second action server or weakening hardware isolation. |
| grasp-scene keyframe | `assets/rebotarm_grasp_scene.xml` now stores home arm `qpos` and matching `ctrl`; keyframe reset test passes and the 1 s benchmark is finite. The benchmark still reports `contact_without_lift` because it does not send a lift trajectory. | `reBotArm_develop_hjx` examples reset to XML keyframes and use a grasp scene, but the observed keyframe is not evidence for this model's geometry or safe grasp pose. | keep; defer contact calibration | `rebotarm_simulation` | Wire timeout/execute-loop behavior next; calibrate object/finger collision and lift policy later. |
| timeout and cancellation | Adapter now uses a monotonic wall-clock deadline (`trajectory duration + execution_timeout_margin_sec`), maps timeout to a non-success action result, and holds current arm positions on timeout/cancel/stop/tolerance failure; ROS 2 + MuJoCo execute-loop harness covers all six outcomes. | `reBotArm_develop_hjx` control scripts contain wall-clock loop/timeout checks, but no reusable ROS action timeout contract was observed. | keep | `rebotarm_simulation` | Preserve the integration harness while adding future hardware-side timeout evidence. |
| trajectory limit consistency | URDF position limits and MuJoCo `ctrlrange` remain checked; URDF effort matches generated arm actuator `forcerange`; MoveIt velocity/acceleration/jerk limits are loaded by `rebotarm_motion` runtime guard and violations abort plus hold. | `reBotArm_develop_hjx` scripts clamp to model joint ranges in individual examples; effort/velocity/acceleration/jerk policy is not a shared contract. | keep | `rebotarm_motion` + `rebotarm_simulation` | Preserve cross-layer evidence and extend the same guard to hardware execution when the real limit contract is frozen. |
| tracking, collision, contact, and calibration | Collision geoms and finger pads exist, but a 3 s linear ramp still leaves current joint4/joint5 at `0.3437/0.7843 rad`; isolated upstream arm force profile ±12.5 alone does not change those residuals. | Upstream has cascaded motor control, gravity compensation, collision baseline, gripper force calibration and scene acceptance commits, but its calibration is not evidence for this robot instance. | migrate candidates; defer final calibration | `rebotarm_simulation` + `rebotarm_calibration` | Decide whether to isolate the full upstream torque-controller/dynamics candidate before collecting hardware tracking evidence. |
| source authorization and environment | MuJoCo 3.3.0, NumPy 1.26.4, cffi 1.17.1 and related dependencies are pinned; `pip check` passes. Default XML has no HJX path. | `robotarm_ros2` package metadata declares Apache-2.0 but its root `LICENSE` is missing from the pinned tree; `reBotArm_develop_hjx` remains `NOASSERTION`; MuJoCo engine is Apache-2.0. | provisional upstream; reject unverified copy | `rebotarm_simulation` | Record file-level license evidence before copying upstream assets or source; keep current model as fallback. |

## P1 execution order after this matrix

1. Pin and preflight the `robotarm_ros2` MuJoCo snapshot, including source/license/file inventory.
2. Run upstream health/headless/model/ROS/motor/collision tests as a separate evidence set.
3. Localize only selected upstream model, converter, control and viewer capabilities into the current package.
4. Wire timeout into the adapter execute loop, then add success/cancel/stop/tolerance/timeout integration tests.
5. Add effort/velocity/acceleration/jerk consistency checks.
6. Revisit joint4-6 tracking, collision, gripper contact and grasp calibration after the dual-baseline control contract is deterministic.

## Evidence boundary

The matrix references current source and candidate upstream paths rather than copying upstream implementation. The `robotarm_ros2` snapshot is not a runtime dependency until source audit and localized regression pass. `reBotArm_develop_hjx` paths are retained only to make behavior observation auditable; they must not be imported, packaged, or loaded by the default runtime.
