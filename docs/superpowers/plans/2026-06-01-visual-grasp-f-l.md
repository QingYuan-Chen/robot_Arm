# Visual Grasp F-L Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the mature-but-safe visual grasp pipeline from filtered candidate execution through retry, lift verification, optional visual servo, place, and recovery.

**Architecture:** Keep ROS message compatibility. Put pure decision logic in small policy modules and keep `visual_grasp_executor_node.py` as the ROS orchestration layer. Defaults stay conservative for real hardware: plan-only by default, retry/place/visual servo disabled unless explicitly enabled, pregrasp refresh required when enabled.

**Tech Stack:** ROS 2 Jazzy, Python, MoveIt services, `rebotarm_msgs`, pytest.

---

## File Structure

- `src/rebotarm_vision/rebotarm_vision/grasp_retry_policy.py`: orders filtered candidates and decides whether another attempt is allowed.
- `src/rebotarm_vision/rebotarm_vision/grasp_verification_policy.py`: combines gripper contact/position and optional visual lift evidence.
- `src/rebotarm_vision/rebotarm_vision/place_task_policy.py`: builds place/open/retreat stages from a configured place pose.
- `src/rebotarm_vision/rebotarm_vision/trajectory_recovery_policy.py`: maps stage failures into stop, retreat, retry, or abort decisions.
- `src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py`: subscribes to filtered candidates, runs attempts through the policies, verifies after lift, and optionally places.
- `src/rebotarm_bringup/launch/visual_grasp_system.launch.py`: exposes safe defaults and operator parameters.
- `tests/test_visual_grasp_sequence.py` and `tests/test_visual_grasp_wiring.py`: TDD coverage for each stage.
- `docs/visual_grasp_commands.md`: Chinese operation flow and copy-paste commands.

## Tasks

### Task 1: Stage G Candidate Retry

**Files:**
- Create: `src/rebotarm_vision/rebotarm_vision/grasp_retry_policy.py`
- Modify: `src/rebotarm_vision/rebotarm_vision/visual_grasp_executor_node.py`
- Test: `tests/test_visual_grasp_sequence.py`, `tests/test_visual_grasp_wiring.py`

- [ ] Write tests for best-index-first retry order and max attempts.
- [ ] Implement the retry policy as a pure module.
- [ ] Wire executor to `/grasp/filtered_candidates` without changing msg definitions.
- [ ] Keep `auto_retry_enabled:=false` by default.

### Task 2: Stage H Lift Verification

**Files:**
- Create: `src/rebotarm_vision/rebotarm_vision/grasp_verification_policy.py`
- Modify: `visual_grasp_executor_node.py`, launch file
- Test: `tests/test_visual_grasp_sequence.py`, `tests/test_visual_grasp_wiring.py`

- [ ] Write tests for gripper contact success, insufficient closure, and optional visual lift evidence.
- [ ] Store close-stage contact result from `/rebotarm/gripper/grasp`.
- [ ] Verify immediately after `lift`; fail safely before place.

### Task 3: Stage K Place State Machine

**Files:**
- Create: `src/rebotarm_vision/rebotarm_vision/place_task_policy.py`
- Modify: `visual_grasp_executor_node.py`, launch file
- Test: `tests/test_visual_grasp_sequence.py`, `tests/test_visual_grasp_wiring.py`

- [ ] Write tests for place stages: move_to_place, open_gripper, place_retreat.
- [ ] Add disabled-by-default place flow after successful verification.
- [ ] Use configured base-frame place pose, not an interactive optional list.

### Task 4: Stage L Recovery

**Files:**
- Create: `src/rebotarm_vision/rebotarm_vision/trajectory_recovery_policy.py`
- Modify: `visual_grasp_executor_node.py`, launch file
- Test: `tests/test_visual_grasp_sequence.py`, `tests/test_visual_grasp_wiring.py`

- [ ] Write tests for pregrasp failure retry, approach failure retry, close/lift failure abort or retry.
- [ ] Request trajectory stop on every failed motion.
- [ ] Optionally call safe retreat before retry when configured.

### Task 5: Docs, Sync, Runtime Verification

**Files:**
- Modify: `docs/visual_grasp_commands.md`

- [ ] Document F-L parameters in Chinese.
- [ ] Add plan-only and explicit execute commands.
- [ ] Run pytest and py_compile locally.
- [ ] Sync to Ubuntu, run colcon build and pytest there.
