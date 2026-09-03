# Gripper Shared-Bus Scheduler Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the independent gripper bus loop and make arm, gripper, feedback, and ROS publication share a bounded, freshness-aware schedule.

**Architecture:** Wrap the existing vendor arm callback with one local hardware-loop callback that also emits gripper commands and performs rate-limited batched feedback.  ROS publishers and high-rate observers consume validated cache; synchronous refresh remains available only for lifecycle gates when no control loop owns the bus.

**Tech Stack:** Python 3.12, ROS 2 Jazzy, pytest, MotorBridge 0.4.6, legacy `reBotArm_control_py`.

**Spec:** `docs/superpowers/specs/2026-09-03-gripper-shared-bus-scheduler-design.md`

## Global Constraints

- Do not start, restart, enable, or command real hardware.
- Keep all final hardware access in `rebotarmcontroller`.
- Keep MotorBridge at 0.4.6; do not upgrade dependencies.
- Preserve the gripper mapping, zero calibration, torque settings, and 0.5 rad/s command ramp.
- Use a 50 Hz default shared feedback rate, 100 Hz cache publication rate, and 0.15 s default stale threshold.
- Preserve unrelated dirty-tree work and never stage `Agent/evidence`, weights, engines, build/install/log, or untracked vendor repositories.

---

### Task 1: Lock the shared-loop contract with failing tests

**Files:**
- Modify: `tests/test_hardware_manager_p0_safety.py`
- Modify: `tests/test_gripper_safety_configuration.py`
- Modify: `tests/test_rebotarm_app_launch.py`
- Create: `tests/test_controller_feedback_cache.py`

**Interfaces:**
- Consumes: `HardwareManager.set_gripper_target`, vendor `start_control_loop(callback)`.
- Produces: tests for `_hardware_control_tick`, `refresh_feedback_if_due`, and `get_cached_joint_state`.

- [ ] Add a fake arm that records the installed callback and a shared fake controller containing six arm motors plus one gripper motor.
- [ ] Assert starting a gripper target leaves the thread count/loop start count unchanged and that invoking the installed arm callback emits both arm and gripper commands.
- [ ] Assert 500 callback invocations over one simulated second create approximately 50 feedback batches, not 500.
- [ ] Assert a failed/stale batch prevents an additional torque-carrying gripper command.
- [ ] Assert `JointStatePublisher.publish()` calls `get_cached_joint_state()` and never `get_joint_state()`.
- [ ] Run the new focused tests and confirm failures name the missing shared-loop/cache behavior.

### Task 2: Implement the unified command and feedback scheduler

**Files:**
- Modify: `src/rebotarmcontroller/rebotarmcontroller/hardware_manager.py`

**Interfaces:**
- Consumes: vendor arm callbacks, motor `request_feedback`, controller `poll_feedback_once`, cached motor `get_state`.
- Produces: `_hardware_control_tick(arm, dt, arm_callback)`, `refresh_feedback_if_due(force=False, now=None)`, and `get_cached_joint_state()`.

- [ ] Add validated configuration for `hardware_feedback_rate_hz` in the 20--100 Hz range and state for next refresh, arm timestamp, and arm error.
- [ ] Implement one batched transaction per controller: request each owned arm/gripper motor, poll once, validate every cached state, and record one successful timestamp.
- [ ] Make the 500 Hz wrapper call the selected arm callback, run a due feedback batch, and then run the command-only gripper tick.
- [ ] Start pos-vel and gravity-compensation modes with the wrapper callback.
- [ ] Turn `_start_gripper_loop` into a compatibility assertion/no-op and remove the independent thread implementation.
- [ ] Make `_gripper_safe_mit` command-only; only successful scheduler batches update freshness.
- [ ] Make enabled state reads use fresh cache and disabled lifecycle gates force a synchronous batch.
- [ ] Run focused scheduler and existing gripper tests until green.

### Task 3: Convert ROS state consumers to cache and expose configuration

**Files:**
- Modify: `src/rebotarmcontroller/rebotarmcontroller/rebotarm_controller.py`
- Modify: `src/rebotarmcontroller/rebotarmcontroller/ros_publishers.py`
- Modify: `src/rebotarmcontroller/rebotarmcontroller/ros_actions.py`
- Modify: `src/rebotarmcontroller/rebotarmcontroller/teach_recorder.py`
- Modify: real-hardware launch files under `src/rebotarm_bringup/launch/`

**Interfaces:**
- Consumes: `HardwareManager.refresh_feedback_if_due` and `get_cached_joint_state`.
- Produces: `hardware_feedback_rate_hz` ROS parameter and 100 Hz cache-only state publication.

- [ ] Declare/forward `hardware_feedback_rate_hz=50.0` and change gripper stale default to 0.15 s.
- [ ] Use 100 Hz as the default real-hardware `joint_state_rate` while retaining configurability.
- [ ] Make the publisher trigger only the rate-limited disabled refresh and then read cached arm/gripper state.
- [ ] Use cached reads in action feedback and teach sampling; retain forced fresh reads for initial safety gates.
- [ ] Run configuration, publisher, action, and launch tests until green.

### Task 4: Document decisions and verify the software-only result

**Files:**
- Modify: `Agent/MEMORY.md`
- Update through script: `Agent/STATE.json`, `Agent/ACTIVITY_LOG.md`

**Interfaces:**
- Consumes: test/build results and upstream commit evidence.
- Produces: durable root-cause boundary, verification record, and real-test gates.

- [ ] Run focused gripper/controller tests and package layering tests.
- [ ] Run `python3 -m pytest tests -q`.
- [ ] Run the required compileall commands, including bringup launch because launch files change.
- [ ] Review `git diff --check`, the complete diff, and excluded-asset status.
- [ ] Record confirmed contention, inferred historical causality, MotorBridge API limitation, test results, and the no-motion boundary in Agent memory/state.
- [ ] Commit and push the isolated fix branch only after all software verification is green.
