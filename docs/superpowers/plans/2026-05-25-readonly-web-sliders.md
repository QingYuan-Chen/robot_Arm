# Read-Only Web Sliders Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add read-only arm joint and gripper sliders to the existing teleop status web panel.

**Architecture:** Keep the existing `TeleopStatusPanelNode` and SSE data path. Extend the browser UI only, using current snapshot fields from `TeleopStatusStore`; no ROS command publisher, HTTP execute API, or real-motion path is added in this slice.

**Tech Stack:** ROS2 Jazzy, `rclpy`, Python `ThreadingHTTPServer`, Server-Sent Events, native HTML/CSS/JavaScript.

---

### Task 1: Add UI Formatting Helpers

**Files:**
- Modify: `src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_state.py`
- Modify: `tests/test_teleop_teach_core.py`

- [ ] **Step 1: Add tests for radians and degrees display.**

Add tests that verify a radians value can be formatted as radians and degrees for the web UI.

- [ ] **Step 2: Implement the helper.**

Add a small pure helper that converts radians to degrees and formats both values.

- [ ] **Step 3: Run focused tests.**

Run `python -m pytest tests/test_teleop_teach_core.py -q`.

### Task 2: Add Read-Only Slider Markup and Styling

**Files:**
- Modify: `src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py`

- [ ] **Step 1: Add a new panel section above the motor table.**

The new section contains six disabled arm sliders and one disabled gripper slider.

- [ ] **Step 2: Add CSS for dense operator-style sliders.**

Keep the status panel compact and readable. Do not add control buttons.

- [ ] **Step 3: Add JavaScript update logic.**

On each SSE status event, update slider values, rad labels, degree labels, and gripper status text.

### Task 3: Verify and Sync

**Files:**
- Test: `tests/test_teleop_teach_core.py`
- Sync to Ubuntu: `/home/u24/robotarm_ros2`

- [ ] **Step 1: Run local tests and compile check.**

Run:

```bash
python -m pytest tests/test_teleop_teach_core.py -q
python -m compileall src/rebotarm_interactive_control/rebotarm_interactive_control/teleop_status_panel_node.py src/rebotarm_interactive_control/rebotarm_interactive_control/status_panel_state.py
```

- [ ] **Step 2: Sync modified files to Ubuntu.**

Copy the changed ROS package files into `/home/u24/robotarm_ros2`.

- [ ] **Step 3: Build on Ubuntu.**

Run:

```bash
cd ~/robotarm_ros2
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rebotarm_interactive_control rebotarm_bringup
```

- [ ] **Step 4: Smoke test status panel HTML.**

Start or query the status panel and confirm the HTML contains the read-only slider section.

