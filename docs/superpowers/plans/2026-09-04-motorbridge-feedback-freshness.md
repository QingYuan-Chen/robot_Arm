# MotorBridge Feedback Freshness Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent cached MotorBridge state from ever being re-stamped as fresh without proof that each motor supplied a new feedback frame.

**Architecture:** Add an atomic per-motor feedback sequence to a pinned MotorBridge 0.4.6 source patch, then make `rebotarmcontroller` the sole owner of a per-motor verified cache. The active 50 Hz acquisition path pipelines sequence-based requests so the 500 Hz command loop does not block; disabled lifecycle gates use a bounded synchronous sequence wait.

**Tech Stack:** Rust 2021, MotorBridge C ABI, Python 3.12/ctypes, ROS 2 Jazzy, pytest.

**Spec:** `docs/superpowers/specs/2026-09-04-motorbridge-feedback-freshness-design.md`

## Global Constraints

- Do not start, restart, enable, disable, zero, or command real hardware.
- Preserve the motor absolute-zero behavior; never add startup homing or a calibration-state gate.
- Keep hardware access and final safety in `rebotarmcontroller`.
- Do not change visual grasp, TCP, jaw mapping, motion speed, force, torque, or stale-threshold parameters.
- Use MotorBridge upstream `v0.4.6` commit `38b8a5681887514b301dbcab96e01a473cbd7173` plus a source-only patch; do not commit wheels, shared libraries, build trees, logs, weights, engines, evidence payloads, or another Git repository.
- Reject an unpatched MotorBridge runtime; do not retain the false-fresh fallback.
- Preserve all unrelated and pre-existing dirty-worktree changes.

---

### Task 1: Add new-frame evidence to the pinned MotorBridge patch

**Files:**
- Create: `patches/motorbridge/0001-add-feedback-sequence-api.patch`
- Modify in temporary upstream checkout before generating the patch: `motor_vendors/damiao/src/motor.rs`
- Modify in temporary upstream checkout before generating the patch: `motor_abi/src/state_ffi.rs`
- Modify in temporary upstream checkout before generating the patch: `motor_abi/include/motor_abi.h`
- Modify in temporary upstream checkout before generating the patch: `motor_abi/src/lib.rs`
- Modify in temporary upstream checkout before generating the patch: `bindings/python/src/motorbridge/abi.py`
- Modify in temporary upstream checkout before generating the patch: `bindings/python/src/motorbridge/core.py`
- Modify in temporary upstream checkout before generating the patch: `bindings/python/src/motorbridge/_version.py`
- Test in temporary upstream checkout: `motor_vendors/damiao/src/motor.rs`
- Test in temporary upstream checkout: `bindings/python/tests/test_feedback_sequence.py`

**Interfaces:**
- Produces C ABI: `motor_handle_get_state_with_sequence(MotorHandle *, MotorState *, uint64_t *) -> int32_t`.
- Produces Rust: `DamiaoMotor::latest_state_with_sequence() -> Option<(MotorFeedbackState, u64)>`.
- Produces Python: `Motor.get_state_with_sequence() -> tuple[MotorState | None, int]`.
- Produces capability: `abi_capabilities()["features"]["feedback_sequence"] is True`.

- [ ] **Step 1: Write the failing Rust test**

  Add a Damiao unit test that feeds the same valid sensor frame twice and
  asserts literal sequences `1` then `2` while the two returned numeric states
  remain equal. The production mutation this catches is reusing the cached
  state without advancing receive identity.

- [ ] **Step 2: Run the Rust test and verify RED**

  Run:

  ```bash
  cargo test -p motor_vendor_damiao identical_feedback_frames_advance_sequence
  ```

  Expected: compile failure because `latest_state_with_sequence` does not exist.

- [ ] **Step 3: Add the minimal atomic sequence implementation**

  Keep state and sequence under the existing state mutex. The stored shape is
  `Option<(MotorFeedbackState, Instant, u64)>`; an accepted sensor frame uses
  `previous_sequence.checked_add(1).unwrap_or(1)` and register replies do not
  increment it. Preserve `latest_state()` by projecting only the state.

- [ ] **Step 4: Run the Rust test and verify GREEN**

  Run the focused test, then:

  ```bash
  cargo test -p motor_vendor_damiao -p motor_abi
  ```

  Expected: all selected Rust tests pass.

- [ ] **Step 5: Write the failing Python binding test**

  Add a binding test with a fake ABI library that fills `CState` and a literal
  `c_uint64(7)`, then assert:

  ```python
  state, sequence = motor.get_state_with_sequence()
  assert state.pos == 1.25
  assert sequence == 7
  ```

  Expected before binding code: `AttributeError` for the missing method.

- [ ] **Step 6: Implement the additive ABI and Python method**

  Validate all three C pointers, dispatch only `MotorHandleInner::Damiao`, fill
  state and sequence from one mutex snapshot, return sequence `0` with an empty
  state before the first frame, and return a clear unsupported error for other
  vendors. Bind the symbol with `c_uint64` and keep the old `get_state()` path
  intact. Set Python package version to `0.4.6+rebotarm.1` and advertise the
  capability.

- [ ] **Step 7: Verify binding tests and generate the source patch**

  Run the binding tests, `cargo fmt --check`, and
  `cargo clippy -p motor_abi -p motor_vendor_damiao -- -D warnings`. Generate
  one patch relative to exact upstream commit `38b8a568...`, add it under
  `patches/motorbridge/`, and verify `git apply --check` against a clean
  checkout of that commit.

### Task 2: Make patched MotorBridge installation reproducible and fail closed

**Files:**
- Create: `tools/setup_motorbridge_fresh_feedback.py`
- Create: `tests/test_motorbridge_freshness_setup.py`
- Modify: `requirements-runtime.txt`
- Modify: `docs/local_setup_zh.md`
- Modify: `README_zh.md`
- Modify: `.gitignore`

**Interfaces:**
- Consumes: Task 1 patch and exact upstream commit.
- Produces CLI: `python3 tools/setup_motorbridge_fresh_feedback.py --build-only`, `--install-user`, and `--check-installed`.
- Produces pure validator: `validate_runtime_contract(module) -> None`.

- [ ] **Step 1: Write failing setup-contract tests**

  Test a fake module missing `Motor.get_state_with_sequence`, a module with the
  method but missing `features.feedback_sequence`, and a fully patched fake.
  The first two must raise `RuntimeError`; the last must return normally.

- [ ] **Step 2: Run the focused tests and verify RED**

  Run:

  ```bash
  python3 -m pytest tests/test_motorbridge_freshness_setup.py -q
  ```

  Expected: import failure because the setup module does not exist.

- [ ] **Step 3: Implement the minimal reproducible builder**

  Clone or reuse an ignored build checkout, verify its `HEAD` equals the pinned
  SHA, refuse dirty or unexpected checkouts, apply the patch idempotently,
  build `motor_abi` and `ws_gateway`, build the Python wheel, and smoke-test it
  in a temporary virtual environment. `--build-only` must not change the user
  Python. `--install-user` performs the explicit reversible user install only
  after the smoke test. `--check-installed` performs no network or build.

- [ ] **Step 4: Verify setup behavior**

  Run the focused tests, `python3 -m py_compile` on the setup tool, and
  `--check-installed` against the current unpatched package, expecting a clear
  nonzero diagnostic rather than hardware access.

- [ ] **Step 5: Update installation guidance**

  Keep the PyPI pin as a bootstrap dependency but state that it is insufficient
  for this controller. Document the exact patched setup command and the
  `--check-installed` gate. Ignore only the tool's named build checkout/output
  directory.

### Task 3: Replace request-success timestamps with verified per-motor samples

**Files:**
- Modify: `src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`
- Modify: `tests/test_hardware_manager_p0_safety.py`

**Interfaces:**
- Consumes: `Motor.get_state_with_sequence()` from Task 1.
- Produces immutable `_VerifiedFeedbackSample(state, sequence, observed_at)`.
- Produces cache map `_verified_feedback_by_label` for `joint1` through `joint6` and `gripper`.
- Produces outstanding request maps keyed by motor label.

- [ ] **Step 1: Replace the wrong freshness test with RED regressions**

  Add separate behavior tests proving:

  ```text
  finite old state + unchanged sequence -> no timestamp advance
  identical numeric state + incremented sequence -> timestamp advances
  six advanced arm sequences + unchanged gripper sequence -> arm fresh, gripper stale
  delayed sequence within the pending window -> accepted on a later scheduler tick
  deadline expiry -> old value retained only for diagnostics and status 255
  ```

  Do not assert mock call existence as the result; assert timestamps, returned
  values, status and failure reason.

- [ ] **Step 2: Run each new test and verify RED**

  Run each named test individually. Confirm failure is caused by the old
  unconditional `_record_*_feedback_success` behavior, not fixture errors.

- [ ] **Step 3: Add verified samples and sequence-only acquisition**

  Require `get_state_with_sequence` for every owned motor before the first
  request. Observe outstanding baselines before issuing the next batch; only a
  strictly greater sequence updates that label. The active scheduler keeps
  requests pending without blocking the 500 Hz command loop. Forced disabled
  refresh performs a bounded condition wait and succeeds only after all labels
  advance. Remove the legacy controller-map fallback that cannot prove a new
  frame.

- [ ] **Step 4: Derive aggregate health without cross-refreshing**

  Compute arm freshness from the oldest of six verified joint timestamps and
  gripper freshness from only the gripper timestamp. A per-label miss records
  its exact label and sequence baseline; successful labels recover
  independently. Shared transport exceptions still mark their controller
  group unavailable.

- [ ] **Step 5: Run focused scheduler tests until GREEN**

  Run:

  ```bash
  python3 -m pytest tests/test_hardware_manager_p0_safety.py -q
  ```

  Expected: all tests pass without opening a device.

### Task 4: Route every connected consumer through the verified cache

**Files:**
- Modify: `src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`
- Modify: `tests/test_hardware_manager_p0_safety.py`
- Test as needed: `tests/test_gripper_action_completion.py`

**Interfaces:**
- Consumes: Task 3 verified cache.
- Changes connected-state readers: `_validated_joint_feedback`, `_validated_gripper_status`, `get_joint_status_codes`, `set_gripper_target`, `get_gripper_state`, `send_gripper_motor_cmd`, and `_gripper_tick`.

- [ ] **Step 1: Write cache-only RED tests**

  Make the raw motor `get_state()`/`get_state_with_sequence()` raise after a
  verified sample has been seeded. Assert joint status, gripper publication,
  gripper start and gripper tick still use the verified sample. Assert these
  reads leave sequence and `observed_at` unchanged.

- [ ] **Step 2: Run the cache-only tests and verify RED**

  Expected: current direct raw reads raise or mutate the returned result.

- [ ] **Step 3: Remove direct raw-cache reads from consumers**

  Make acquisition the only connected path that reads MotorBridge feedback.
  Gripper start/arrival/stall logic uses the verified cached position, velocity,
  torque and status. Stale/error state returns status `255` and never updates
  the cached sample time. Preserve all existing command, neutralization,
  mapping and effort behavior.

- [ ] **Step 4: Verify controller/action regressions**

  Run:

  ```bash
  python3 -m pytest tests/test_hardware_manager_p0_safety.py tests/test_gripper_action_completion.py -q
  ```

  Expected: all selected tests pass.

### Task 5: Build, verify, record and commit the software-only fix

**Files:**
- Modify: `Agent/MEMORY.md`
- Update through script: `Agent/STATE.json`
- Append through script: `Agent/ACTIVITY_LOG.md`

**Interfaces:**
- Consumes: Tasks 1-4 implementation and test evidence.
- Produces: reproducible patched wheel, durable project record, and a local source-only commit.

- [ ] **Step 1: Build and smoke-test the patched dependency**

  Run `--build-only`, install its wheel into a temporary venv, and verify the
  local version, capability and Python method. Do not open a Controller or a
  serial channel.

- [ ] **Step 2: Run required repository checks**

  Run:

  ```bash
  python3 -m pytest tests/test_package_layering.py -q
  python3 -m pytest tests -q
  python3 -m compileall src/rebotarm_dashboard/rebotarm_dashboard src/rebotarm_teleop/rebotarm_teleop src/rebotarm_teach/rebotarm_teach src/rebotarm_motion/rebotarm_motion src/rebotarm_interactive_control/rebotarm_interactive_control -q
  python3 -m compileall src/rebotarmcontroller/rebotarmcontroller -q
  git diff --check
  ```

- [ ] **Step 3: Install the verified patched wheel and check the runtime**

  Run the setup tool with `--install-user`, then `--check-installed`. This is a
  Python dependency update only: do not start ROS, instantiate a Controller,
  open `/dev/ttyACM0`, or touch hardware.

- [ ] **Step 4: Record exact evidence and remaining hardware gate**

  Append the confirmed root cause, API contract, pinned commit/patch version,
  tests/build results and no-motion boundary to `Agent/MEMORY.md`. Use
  `Agent/update_state.py` for checkpoint and final events. Do not mark a P6
  hardware checkbox complete.

- [ ] **Step 5: Review and commit only source artifacts**

  Review the complete diff and status. Exclude evidence payloads, wheels,
  shared libraries, build/install/log trees, weights, engines and unrelated
  dirty files. Commit the intended source, tests, patch, docs and generated
  Agent state changes locally; do not push without separate authorization.
