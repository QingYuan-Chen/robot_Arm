# Web Teleop Panel Design

## Goal

Extend the existing `rebotarm_interactive_control` web status panel from a read-only motor table into a staged web teleoperation surface. The final direction is:

1. Read-only joint and gripper sliders.
2. Web 3D robot arm and gripper display.
3. Preview sliders that only move the web model.
4. Preview + Execute confirmed sending.
5. Hardware safety validation.

The first implementation slice is only step 1. It must not send motion commands.

## Existing Context

- `TeleopStatusPanelNode` already runs a local `ThreadingHTTPServer`.
- Browser state is updated by Server-Sent Events.
- The node already subscribes to:
  - `/rebotarm/joint_states`
  - `/rebotarm/joints/<joint>/state`
  - `/rebotarm/gripper/state`
  - teleop, recording, and replay status topics
- `rebotarmcontroller` remains the only hardware execution core.
- Real arm motion must continue through `/rebotarm/follow_joint_trajectory`.

## Stage 1: Read-Only Sliders

Add a read-only slider section to the current status page:

- Arm sliders:
  - `joint1` through `joint6`
  - value source: latest joint or motor position
  - slider range: use configured/default joint limits where available
  - display both radians and degrees
  - sliders are disabled
- Gripper display:
  - source: `/rebotarm/gripper/state`
  - show position/opening if available
  - show state/status code alongside the slider
  - disabled/read-only

This stage is a visualization feature only. It does not add buttons, HTTP command APIs, or ROS publishers/actions for control.

## Stage 2: Web 3D Arm Display

Use browser-side Three.js for visualization:

- Three.js renders the scene.
- URDFLoader loads the robot model and mesh assets.
- `/rebotarm/joint_states` drives `joint1` through `joint6`.
- `/rebotarm/gripper/state` drives the gripper joint or finger joints.

RViz remains the primary ROS-native visualization and debugging tool. The web view is for integrated operator feedback in the teleop panel.

## Stage 3: Preview Sliders

Make a separate preview mode:

- User can drag arm and gripper sliders.
- Only the browser-side Three.js model moves.
- No ROS command is sent while dragging.
- The page clearly separates live state from preview target.

## Stage 4: Preview + Execute

Add confirmed execution after preview:

- `Execute Arm` sends a joint trajectory through the existing ROS action path.
- `Execute Gripper` uses the existing gripper action or service.
- Dragging sliders never directly moves hardware.
- Execution requires an explicit confirmation step.
- The node validates limits, speed, and current state before sending.

## Stage 5: Hardware Safety Validation

Test order:

1. Stage 1 with hardware and no motion.
2. Stage 2 with hardware and no motion.
3. Stage 3 preview without hardware motion.
4. Arm execute with dry-run or simulated path.
5. Arm execute with tiny low-speed joint deltas.
6. Gripper execute alone.
7. Combined arm + gripper execution.

Reject execution when:

- joint target is outside limits,
- current state is stale,
- action server is unavailable,
- emergency/disabled/error state is active,
- requested movement exceeds configured delta or speed thresholds.

## First Implementation Scope

Implement only:

- read-only sliders for `joint1` to `joint6`,
- read-only gripper state display,
- radians and degrees display for arm joints,
- real-time SSE updates,
- focused tests for the formatting/state helper logic.

Do not implement yet:

- Three.js,
- URDF loading,
- draggable preview sliders,
- HTTP Execute APIs,
- real hardware web control.

