# Teach Replay Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a first-stage local safety gate for hand-guided teach replay so risky JSONL records are blocked or retimed before any real trajectory is sent.

**Architecture:** Keep `rebotarmcontroller` as the only execution core. Put replay quality analysis and safe time parameterization in `teach_recording.py`, then reuse it from both `TeachReplayNode` and the web status panel.

**Tech Stack:** ROS2 Jazzy Python nodes, `rclpy`, `FollowJointTrajectory`, JSONL teach records, native HTML/CSS/JS Canvas for trajectory viewing.

---

### Task 1: Core Safety Functions

**Files:**
- Modify: `src/rebotarm_interactive_control/rebotarm_interactive_control/teach_recording.py`
- Test: `tests/test_teleop_teach_core.py`

- [ ] Add tests for green, yellow, red trajectory classification.
- [ ] Add tests for monotonic safe retiming and velocity capping.
- [ ] Add tests for downsampled trajectory preview payload.
- [ ] Implement dataclasses and pure functions only; no ROS imports.

### Task 2: Replay Node Safety Gate

**Files:**
- Modify: `src/rebotarm_interactive_control/rebotarm_interactive_control/teach_replay_node.py`
- Modify: `src/rebotarm_bringup/launch/teach_replay.launch.py`
- Modify: `src/rebotarm_interactive_control/config/teleop_control.yaml`
- Test: `tests/test_teleop_teach_core.py`

- [ ] Default direct start threshold to `0.01`.
- [ ] Publish replay status with `RELIABLE + TRANSIENT_LOCAL`.
- [ ] Include quality summary in dry-run and real replay status.
- [ ] Reject red records for real replay.
- [ ] Clamp yellow replay speed and build trajectories with safe retiming.

### Task 3: Web Panel Trajectory Viewer

**Files:**
- Modify: `src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py`

- [ ] Add `/api/teach_trajectory`.
- [ ] Render quality summary in existing teach file panel.
- [ ] Add a Canvas 2D joint position vs time viewer.
- [ ] Mark yellow/red trajectory samples.
- [ ] Keep the page read-first: viewer does not send motion commands.

### Task 4: Verification

**Commands:**
- `python -m compileall src/rebotarm_interactive_control/rebotarm_interactive_control`
- `python -m pytest tests/test_teleop_teach_core.py -q`
- On Ubuntu VM: `colcon build --symlink-install --packages-select rebotarm_interactive_control rebotarm_bringup`
- On Ubuntu VM: `python3 -m pytest tests/test_teleop_teach_core.py -q`

**Safety boundary:** Do not run real teach replay during this implementation verification unless the user explicitly asks for a hardware test.
