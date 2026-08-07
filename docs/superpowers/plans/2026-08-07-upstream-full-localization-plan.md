# P1 Upstream MuJoCo Full Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Vendor the pinned `robotarm_ros2` MuJoCo package into the current Git as an isolated comparison baseline and produce reproducible A/B evidence against the current package-owned simulation.

**Architecture:** Keep `src/rebotarm_simulation` as the default current backend. Store the byte-preserved upstream package and its MuJoCo tests under `third_party/robotarm_ros2_mujoco_snapshot/`, outside the default colcon source path. Use a small Python comparison harness to run each backend in separate subprocess/PYTHONPATH contexts and write normalized JSON evidence without sharing ROS action names or runtime state.

**Tech Stack:** Python 3.12, MuJoCo 3.3.0, pytest, setuptools/ROS 2 Jazzy package layout, JSON evidence, `git` provenance.

---

### Task 1: Vendor and attest the pinned upstream snapshot

**Files:**
- Create: `third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation/` (byte-preserved upstream package)
- Create: `third_party/robotarm_ros2_mujoco_snapshot/tests/` (upstream MuJoCo and URDF-to-MJCF tests)
- Create: `third_party/robotarm_ros2_mujoco_snapshot/UPSTREAM_PROVENANCE.json`
- Create: `third_party/robotarm_ros2_mujoco_snapshot/README_LOCALIZATION.md`
- Test: `tests/test_upstream_snapshot_provenance.py`

- [x] **Step 1: Write the failing provenance test**

```python
def test_upstream_snapshot_is_pinned_and_outside_default_src():
    root = Path("third_party/robotarm_ros2_mujoco_snapshot")
    manifest = json.loads((root / "UPSTREAM_PROVENANCE.json").read_text())
    assert manifest["remote"] == "https://github.com/huangbinai/robotarm_ros2.git"
    assert manifest["commit"] == "fb28dcdd358b45de79eb47adfb333e2e94e9d5b4"
    assert (root / "src/rebotarm_simulation/package.xml").is_file()
    assert not (Path("src") / "rebotarm_simulation_upstream").exists()
```

- [x] **Step 2: Run the test to verify it fails**

Run: `python3 -m pytest tests/test_upstream_snapshot_provenance.py -q`  
Expected: FAIL because the snapshot and manifest do not exist yet.

- [x] **Step 3: Copy the exact source and test scope**

Run from the repository root:

```bash
mkdir -p third_party/robotarm_ros2_mujoco_snapshot
mkdir -p third_party/robotarm_ros2_mujoco_snapshot/src
cp -a /home/a/project/rebot_refer/src/rebotarm_simulation third_party/robotarm_ros2_mujoco_snapshot/src/
mkdir -p third_party/robotarm_ros2_mujoco_snapshot/tests
cp -a /home/a/project/rebot_refer/tests/test_mujoco_*.py third_party/robotarm_ros2_mujoco_snapshot/tests/
cp -a /home/a/project/rebot_refer/tests/test_urdf_to_mjcf.py third_party/robotarm_ros2_mujoco_snapshot/tests/
cp -a /home/a/project/rebot_refer/src/rebotarm_simulation/README_mujoco.md third_party/robotarm_ros2_mujoco_snapshot/
mkdir -p third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_bringup/config third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_bringup/launch
mkdir -p third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_moveit_config/config third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_moveit_config/launch
cp -a /home/a/project/rebot_refer/src/rebotarm_bringup/config/arm.yaml /home/a/project/rebot_refer/src/rebotarm_bringup/config/gripper.yaml third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_bringup/config/
cp -a /home/a/project/rebot_refer/src/rebotarm_bringup/launch/interactive_system.launch.py third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_bringup/launch/
cp -a /home/a/project/rebot_refer/src/rebotarm_moveit_config/config/rebotarm.urdf third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_moveit_config/config/
cp -a /home/a/project/rebot_refer/src/rebotarm_moveit_config/launch/demo.launch.py third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_moveit_config/launch/
cp -a /home/a/project/rebot_refer/.gitattributes third_party/robotarm_ros2_mujoco_snapshot/
mkdir -p third_party/robotarm_ros2_mujoco_snapshot/docs
cp -a /home/a/project/rebot_refer/docs/rebotarm_feature_commands.md third_party/robotarm_ros2_mujoco_snapshot/docs/
```

Write `UPSTREAM_PROVENANCE.json` with the remote, branch, pinned commit, source path,
license result `PROVISIONAL`, target file inventory and SHA-256 for package.xml, setup.py,
robot.xml, scene.xml, mujoco_sim.py, mujoco_ros_node.py, motor_control.py and mujoco_health.py.

- [x] **Step 4: Run the provenance test and snapshot diff**

Run:

```bash
python3 -m pytest tests/test_upstream_snapshot_provenance.py -q
diff -qr /home/a/project/rebot_refer/src/rebotarm_simulation third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation
```

Expected: test passes and `diff -qr` prints no differences for the copied package.

- [x] **Step 5: Record the snapshot boundary**

Update `Agent/MEMORY.md`, `Agent/PROJECT_STATUS.md` only if the manifest and diff are valid,
and record the exact copy command and license status in
`Agent/evidence/P1/2026-08-07-upstream-snapshot-localization.md`.

### Task 2: Add an isolated upstream runner

**Files:**
- Create: `scripts/p1_upstream_mujoco/run_upstream.py`
- Create: `tests/test_upstream_runner.py`
- Modify: `.gitignore` (ignore only `build/upstream_mujoco_ws/`, `install/upstream_mujoco_ws/`, and `log/upstream_mujoco_ws/`)

- [x] **Step 1: Write failing runner tests**

Test that `snapshot_python()` points to `third_party/rebotarm_ros2_mujoco_snapshot/src`,
that the environment sets `PYTHONPATH` to the snapshot package, and that a command rejecting
`/dev/ttyACM0` or `rebotarmcontroller` raises `ValueError` before subprocess execution.

- [x] **Step 2: Run the tests and confirm the missing module failure**

Run: `python3 -m pytest tests/test_upstream_runner.py -q`  
Expected: FAIL with the runner module missing.

- [x] **Step 3: Implement the minimal runner**

Expose:

```python
SNAPSHOT_ROOT = Path("third_party/robotarm_ros2_mujoco_snapshot")

def snapshot_python(repo_root: Path) -> Path: ...
def snapshot_env(repo_root: Path) -> dict[str, str]: ...
def validate_safe_command(command: Sequence[str]) -> None: ...
def run_upstream(repo_root: Path, command: Sequence[str], *, cwd: Path | None = None) -> CompletedProcess[str]: ...
```

The runner must use `third_party/rebotarm_mujoco_venv/bin/python` when present, otherwise
report a clear dependency error. It must set `PYTHONPATH` only for the upstream snapshot,
never source `install/setup.bash`, and reject hardware words before `subprocess.run`.

- [x] **Step 4: Run runner tests and a safe upstream health command**

Run:

```bash
python3 -m pytest tests/test_upstream_runner.py -q
python3 scripts/p1_upstream_mujoco/run_upstream.py \
  -m rebotarm_simulation.mujoco_health \
  --model "$(pwd)/third_party/robotarm_ros2_mujoco_snapshot/src/rebotarm_simulation/models/rebotarm/scene.xml" \
  --skip-renderer
```

Expected: runner tests pass and health JSON reports `ok=true`; no hardware process is started.

### Task 3: Add the current/upstream comparison harness

**Files:**
- Create: `scripts/p1_upstream_mujoco/compare_baselines.py`
- Create: `tests/test_mujoco_baseline_comparison.py`
- Create: `Agent/evidence/P1/2026-08-07-baseline-comparison.json`

- [x] **Step 1: Write failing normalization tests**

Test `normalize_model()` produces `nq/nv/nu/njnt/nbody/ngeom/nsensor`, and
`compare_records()` preserves backend labels, numeric values, command strings and failures
without coercing interface mismatches into pass/fail booleans.

- [x] **Step 2: Run comparison tests to verify they fail**

Run: `python3 -m pytest tests/test_mujoco_baseline_comparison.py -q`  
Expected: FAIL because the harness module is missing.

- [x] **Step 3: Implement normalization and isolated subprocess calls**

The harness must run:

```python
current_health = check_model_health(current_xml, steps=2)
current_smoke = run_smoke(current_generated_xml, seconds=1.0)
current_step = run_step_response_suite(current_generated_xml, targets=default_step_response_targets(MOTOR_PROFILES), seconds=1.0)
current_grasp = run_grasp_benchmark(current_scene_xml, seconds=1.0)
upstream_health = run_upstream(...)
upstream_headless = run_upstream(..., "-m", "rebotarm_simulation.mujoco_cli", "--headless", "--duration", "1")
```

Store raw stdout/stderr and parsed summaries. Mark the known upstream replay assertion as
`test_defect`, not `runtime_failure`, with its exact test node id.

- [x] **Step 4: Run the harness and write the first A/B report**

Run: `third_party/rebotarm_mujoco_venv/bin/python scripts/p1_upstream_mujoco/compare_baselines.py --duration 1 --output Agent/evidence/P1/2026-08-07-baseline-comparison.json`
Expected: JSON contains current/upstream records, model dimensions, health, smoke, trajectory,
grasp and test summary fields; current backend remains `nu=7`, upstream robot remains `nu=8`.

### Task 4: Run the full copied upstream test set independently

**Files:**
- Modify: `Agent/PROJECT_STATUS.md`
- Modify: `Agent/MEMORY.md`
- Create: `Agent/evidence/P1/2026-08-07-upstream-localized-test-report.md`

- [x] **Step 1: Run copied tests from the snapshot root**

Run:

```bash
cd third_party/robotarm_ros2_mujoco_snapshot
PYTHONPATH="$PWD/src/rebotarm_simulation" \
  ../rebotarm_mujoco_venv/bin/python -m pytest \
  tests/test_mujoco_*.py tests/test_urdf_to_mjcf.py -q
```

Expected baseline is `219 passed, 1 skipped, 1 failed` until the known nested
`mapping -> tuple` `pytest.approx` assertion is corrected upstream.

- [x] **Step 2: Run copied health/headless/model checks**

Run the snapshot runner for `mujoco_health --skip-renderer`, `mujoco_cli --headless --duration 1`,
and the model contract tests. Record command, commit, output and any failure classification.

- [x] **Step 3: Update state without overclaiming**

Only mark the P1 independent-test checkbox complete if the report contains a reviewed test
defect classification and all required runtime checks pass; do not call the upstream suite
fully green while the replay assertion remains unchanged.

### Task 5: Review, verify and hand off

**Files:**
- Modify: `docs/mujoco_gap_matrix.md`
- Modify: `Agent/EXECUTION_FLOW.md`
- Modify: `Agent/STATE.json` via `Agent/update_state.py`
- Modify: `Agent/MEMORY.md`

- [x] **Step 1: Run required repository checks**

Run:

```bash
python3 -m pytest tests/test_package_layering.py -q
python3 -m pytest tests -q
python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion src/rebotarm_interactive_control/rebotarm_interactive_control src/rebotarm_simulation/rebotarm_simulation -q
git diff --check
```

- [x] **Step 2: Refresh state and evidence**

Run `python3 Agent/update_state.py --event verified --actor Codex --note ... --verification ...`
only after the A/B report and required checks exist. Keep current and upstream test counts
separate in `Agent/MEMORY.md` and `Agent/STATE.json`.

- [x] **Step 3: Report comparison effect and remaining gaps**

The handoff must state which backend was better for model health, trajectory residual, gripper,
contact and startup isolation, and must list unresolved license, actuator-contract, calibration,
viewer and replay-test issues.
