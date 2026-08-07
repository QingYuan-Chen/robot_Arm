# P1 上游 MuJoCo 完整本地快照与双基线对比设计

**Status:** Approved direction; specification ready for review.  
**Date:** 2026-08-07  
**Scope:** 将 `huangbinai/robotarm_ros2` 的 MuJoCo package 完整保留到当前 Git，作为
独立 upstream comparison baseline；当前 package-owned MuJoCo 继续作为默认 baseline。

## 1. Goal and non-goals

### Goal

在不改变当前实机安全边界、ROS 默认接口和现有模型的前提下，将上游固定快照的 MuJoCo
runtime、models、launch、tests、configuration 和 documentation 纳入当前仓库，形成可重复
运行的 A/B 仿真环境，用统一指标比较两套实现的模型、控制、夹爪、碰撞、接触和执行效果。

### Non-goals

- 不把上游 package 直接覆盖当前 `src/rebotarm_simulation`。
- 不在同一个 colcon workspace 中同时暴露两个同名 `rebotarm_simulation` package。
- 不改变当前 `rebotarmcontroller`、真实硬件 launch 或 P0 enable/disable 流程。
- 不把上游通过测试直接当作当前 package 或真机验收证据。
- 不在 license evidence 补全前对外发布或声称上游快照可再分发。

## 2. Source and provenance boundary

- Repository: `https://github.com/huangbinai/robotarm_ros2.git`
- Branch: `main`
- Pinned commit: `fb28dcdd358b45de79eb47adfb333e2e94e9d5b4`
- Authoritative local checkout: `/home/a/project/rebot_refer`
- Candidate scope: `src/rebotarm_simulation/`、MuJoCo tests、MuJoCo README、相关 config/launch。
- Target in current Git: `third_party/robotarm_ros2_mujoco_snapshot/`。
- Snapshot must include a provenance manifest containing remote, branch, commit, source path,
  file inventory, SHA-256 and license evidence.
- `package.xml`/`setup.py` declare Apache-2.0, but the pinned tree has no root `LICENSE` found;
  snapshot status remains `PROVISIONAL` and is restricted to the project’s private comparison use.

The snapshot is copied byte-for-byte where possible. Any mechanical path or workspace wrapper is
kept outside the snapshot and recorded separately so upstream behavior remains auditable.

## 3. Repository layout

```text
rebot_Arm/
├── src/rebotarm_simulation/                 # current baseline, unchanged default
├── third_party/robotarm_ros2_mujoco_snapshot/
│   ├── src/rebotarm_simulation/              # upstream package snapshot
│   ├── tests/                                # upstream MuJoCo tests
│   ├── UPSTREAM_PROVENANCE.json
│   └── README_LOCALIZATION.md
├── scripts/p1_upstream_mujoco/               # comparison/build wrappers only
└── Agent/evidence/P1/                        # JSON reports and audit notes
```

The upstream package remains outside `src/` so normal `colcon build` cannot discover a duplicate
package. `scripts/p1_upstream_mujoco/` creates or refreshes an isolated workspace under
`build/upstream_mujoco_ws/` (ignored build/install/log outputs) and passes explicit `PYTHONPATH`
or `AMENT_PREFIX_PATH` only to comparison commands.

## 4. Runtime and data flow

```text
current command ──> current rebotarm_simulation ──> current metrics/evidence

upstream command ──> isolated upstream workspace ──> upstream metrics/evidence
                                          │
                                          └──> normalized A/B report (no shared runtime state)
```

Each backend gets its own model path, Python environment, ROS namespace and output directory.
The comparison harness must reject a command if both backends would claim the same
`FollowJointTrajectory` action name in one ROS graph. Headless comparison is the default;
viewer startup is explicit and never part of automated tests.

## 5. Comparison contract

The harness records one JSON document per backend and a normalized differential report with:

- source commit and model SHA-256;
- model dimensions (`nq`, `nv`, `nu`, joints, bodies, geoms, sensors);
- health/headless result and finite-state checks;
- one-second and configurable trajectory response metrics;
- joint positions, velocities and actuator forces;
- gripper width/force/target reach behavior;
- contact pairs, collision baseline and grasp status;
- ROS endpoints, namespace, `/clock` ownership and exit/timeout result;
- test command, environment interpreter and pass/skip/fail counts.

The report distinguishes:

1. exact interface mismatch (e.g. current `nu=7` position/gripper contract vs upstream `nu=8`
   independent motor/finger contract);
2. numerical tolerance differences;
3. model geometry or calibration differences;
4. harness or test defects, including the known nested `mapping -> tuple` `pytest.approx`
   assertion in the upstream replay test.

## 6. Safety and rollback

- No upstream launch may import `rebotarmcontroller`, open `/dev/ttyACM0`, or call enable,
  disable, trajectory, safe-home or gripper hardware services.
- Current hardware and simulation launch files remain unchanged until comparison evidence is
  reviewed.
- Snapshot removal is a single Git revert; generated `build/upstream_mujoco_ws/` is ignored and
  can be deleted without affecting source or current runtime.
- A failed upstream process is cleaned by PID/process-group scope for the isolated workspace only;
  cleanup must not stop current ROS daemon or real driver processes.

## 7. Implementation stages

1. Copy the pinned package into the snapshot directory and write provenance/license manifest.
2. Add isolated workspace wrapper and verify the snapshot can run independently.
3. Add normalized comparison harness and JSON evidence schema.
4. Run current and upstream health/model/headless/trajectory/gripper/contact checks.
5. Fix only harness compatibility issues; do not patch upstream behavior silently.
6. Publish a differential report and decide which upstream capabilities, if any, should later be
   localized into the current package.

## 8. Acceptance criteria

- Current package regression remains green and its default launch behavior is unchanged.
- Upstream snapshot identity and every copied target file are auditable.
- Both backends run independently without duplicate action servers or hardware access.
- At least one reproducible A/B report exists for health, model contract, trajectory and gripper.
- Known upstream replay assertion defect is recorded separately from runtime results.
- No P1 checkbox is marked complete merely because the upstream snapshot tests pass; localized
  behavior still requires current-package tests and explicit interface/calibration evidence.

## 9. Open risks

- Missing upstream root `LICENSE` keeps the snapshot provisional.
- Upstream and current actuator/sensor contracts differ; full visual/control parity is not implied.
- Upstream ROS dependencies may not be installed in the current environment; the wrapper must
  report missing dependencies without falling back to the real backend.
- Viewer/renderer availability may differ from headless physics and must be reported separately.
