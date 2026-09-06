# Upstream MuJoCo Source Swap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Archive the current `src/rebotarm_simulation` package and make the pinned upstream MuJoCo runtime the active package source while preserving the current project APIs as a minimal compatibility/fallback layer.

**Architecture:** The current package tree is copied byte-for-byte into `third_party/rebotarm_simulation_current_baseline/` with a manifest before the active source is replaced. The upstream package becomes the active core in `src/rebotarm_simulation`, including its `mujoco_sim`, `mujoco_ros_node`, motor controller, models, configs, and launch. Non-colliding current modules required by the rest of this repository remain as explicit compatibility modules; the current ROS adapter stays available as a fallback entrypoint but is not the default runtime.

**Tech Stack:** ROS 2 Jazzy, `ament_python`, MuJoCo 3.3.0, Python 3.12, pytest, colcon.

---

### Task 1: Add source-layout regression tests

**Files:**
- Create: `tests/test_simulation_source_swap.py`
- Test: `src/rebotarm_simulation/setup.py`, `src/rebotarm_simulation/package.xml`, `third_party/rebotarm_simulation_current_baseline/ARCHIVE_MANIFEST.json`

- [ ] **Step 1: Write failing tests**

  Assert that the active package exposes upstream `rebotarm_mujoco_node`, the upstream model/config directories, the current adapter compatibility entrypoint, and an archive manifest containing the original package file hashes.

- [ ] **Step 2: Run the tests to verify the expected failure**

  Run: `python3 -m pytest tests/test_simulation_source_swap.py -q`

  Expected: FAIL because the archive and upstream source replacement have not happened yet.

### Task 2: Archive the current source package

**Files:**
- Create: `third_party/rebotarm_simulation_current_baseline/` (complete copy of current `src/rebotarm_simulation`)
- Create: `third_party/rebotarm_simulation_current_baseline/README.md`
- Create: `third_party/rebotarm_simulation_current_baseline/ARCHIVE_MANIFEST.json`

- [ ] **Step 1: Copy the complete current tree before changing `src`**

  Preserve tracked, untracked, generated asset, and launch files with `cp -a`; do not delete the source until the archive file count and hashes match.

- [ ] **Step 2: Generate and verify the manifest**

  Record relative path, size, and SHA-256 for every non-cache file. Verify the archive and pre-swap source have identical manifests.

- [ ] **Step 3: Run the archive regression tests**

  Run: `python3 -m pytest tests/test_simulation_source_swap.py -q`

  Expected: archive assertions pass; active upstream assertions remain red until Task 3.

### Task 3: Replace the active package with upstream core

**Files:**
- Replace: `src/rebotarm_simulation/` with the pinned package from `third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation/`
- Modify: `src/rebotarm_simulation/setup.py`
- Modify: `src/rebotarm_simulation/package.xml`
- Create/restore: local compatibility modules `mujoco_adapter_core.py`, `mujoco_grasp_quality.py`, `mujoco_limit_checks.py`, `mujoco_metrics.py`, `mujoco_model_profile.py`, `mujoco_runner.py`, `mujoco_ros_adapter_node.py`, `mujoco_cli.py`, and `upstream_backend.py` from the archived source where current repository tests require them

- [ ] **Step 1: Copy upstream package files into `src`**

  Use the snapshot package as the source of truth for upstream files. Do not edit the upstream `mujoco_sim.py`, `mujoco_ros_node.py`, `motor_control.py`, `mujoco_types.py`, `trajectory_sampler.py`, `mujoco_viewer.py`, or `urdf_to_mjcf.py` implementations.

- [ ] **Step 2: Add only compatibility entrypoints and modules**

  Keep `rebotarm_mujoco_node`, `rebotarm_mujoco_health`, `rebotarm_mujoco_viewer`, and `rebotarm_urdf_to_mjcf` mapped to upstream implementations. Keep `rebotarm_mujoco_adapter` and the current metrics/profile CLI mapped to archived current compatibility modules. Merge the health API so both upstream `collect_health()` and current `check_model_health()` remain callable.

- [ ] **Step 3: Add the project MoveIt wrapper launch**

  Restore `mujoco_moveit_sim.launch.py` as a local wrapper that launches the upstream `rebotarm_mujoco_node` by default, passes `use_sim_time`, and starts MoveIt without fake joint states. It must not start `rebotarmcontroller`.

- [ ] **Step 4: Run focused source and package tests**

  Run: `python3 -m pytest tests/test_simulation_source_swap.py tests/test_mujoco_ros_adapter_launch.py tests/test_mujoco_health.py tests/test_mujoco_model_profile.py -q`

  Expected: all focused tests pass before running the full suite.

### Task 4: Rebuild and run upstream/current runtime verification

**Files:**
- Modify: `docs/mujoco_sim.md`
- Modify: `docs/mujoco_upstream_sources.md`
- Create: `Agent/evidence/P1/2026-08-07-upstream-src-swap.md`
- Create: `Agent/evidence/P1/2026-08-07-upstream-src-swap.json`

- [ ] **Step 1: Rebuild the active package**

  Run: `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select rebotarm_simulation rebotarm_moveit_config rebotarm_bringup`

- [ ] **Step 2: Run upstream health/headless checks from the active `src` package**

  Run the upstream health CLI and one-second headless CLI with the pinned MuJoCo environment. Record model dimensions and finite-state result.

- [ ] **Step 3: Run ROS launch and a simulation trajectory**

  Launch `mujoco_moveit_sim.launch.py` with `use_rviz:=false` and a private `ROS_DOMAIN_ID`; verify `/clock`, joint states, the single trajectory action server, and a one-second six-joint goal returning `error_code=0`.

- [ ] **Step 4: Run the current compatibility fallback**

  Start the fallback current adapter using its explicit compatibility entrypoint and verify it still exposes its action/service contract without hardware access.

### Task 5: Full regression and state handoff

**Files:**
- Modify: `Agent/MEMORY.md`, `Agent/PROJECT_STATUS.md`, `Agent/EXECUTION_FLOW.md`, `Agent/STATE.json`

- [ ] **Step 1: Run required checks**

  Run:

  ```bash
  python3 -m pytest tests/test_package_layering.py -q
  python3 -m pytest tests -q
  python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion src/rebotarm_interactive_control/rebotarm_interactive_control src/rebotarm_bringup/launch src/rebotarm_simulation -q
  git diff --check
  ```

- [ ] **Step 2: Record verified status**

  Update Agent memory and evidence with the archive path, active upstream source, compatibility boundary, test counts, runtime result, license status, and rollback command. Refresh state with `python3 Agent/update_state.py --event verified ...`.

- [ ] **Step 3: Keep unrelated worktree changes untouched**

  Do not stage or rewrite existing user changes outside the source swap, compatibility modules, evidence, and state records.
