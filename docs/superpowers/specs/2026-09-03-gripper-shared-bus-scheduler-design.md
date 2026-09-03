# Gripper Shared-Bus Scheduler Design

## Problem

The arm and gripper share one MotorBridge serial controller and therefore one
physical bus.  The current downstream driver starts an independent 500 Hz
gripper thread on the first gripper command while ROS state publication calls
the six-axis synchronous feedback path at 200 Hz.  Both paths repeatedly take
the same bus lock.  A gripper command can consequently starve arm feedback and
ROS callbacks; the same starvation can leave the gripper command running while
the application observes an old position.

The two failed bottle runs are consistent with this mechanism: both failed
during the initial open command, the physical gripper travelled much farther
than the reported 2--3 mm, and no close command had yet been issued.  Bus
starvation is confirmed by the current control topology.  It remains an
inference, not yet a hardware-proven causal reconstruction, that starvation was
the sole cause of those exact two over-open events.

## Upstream basis

The Seeed upstream controller at
`e134941b71236523e831f15b470bc81186b0649f` and SDK at
`1bcd81b22c182ec257bf04f5746e1e3d556a5f1c` put arm and gripper command output
in one control-loop callback.  This design backports that ownership pattern to
the local legacy SDK instead of upgrading MotorBridge or replacing the local
driver wholesale.

Installed MotorBridge Python/ABI version 0.4.6 exposes `request_feedback()`,
`poll_feedback_once()`, and cached `get_state()`.  It does not expose a Python
`request_fresh_state` method or a feedback timestamp, so the driver must stamp
a feedback batch only after its explicit request/poll transaction succeeds.

## Architecture

1. Keep one 500 Hz hardware command loop.  Its callback sends the active arm
   command and, when requested, one gripper command.  No independent gripper
   thread is created.
2. In that same loop, rate-limit shared-bus feedback transactions to a
   configurable 50 Hz.  Group all motors by controller, request every motor in
   the group, poll that controller once, then validate and timestamp the whole
   batch.
3. Publish ROS joint/gripper state at 100 Hz from the validated cache.  The
   publisher must not synchronously transact on the bus while the command loop
   owns it.  While disabled, the publisher may trigger the same rate-limited
   feedback scheduler because there is no command writer.
4. Treat 150 ms as the default stale threshold.  An active gripper command
   checks freshness before advancing its ramp or sending another command.  A
   failed or stale batch marks the command failed, switches it to idle, and
   attempts one neutral zero-stiffness command.
5. Service and action completion continue to use the same
   `wait_gripper_target`/`gripper_reached_target` state.  Both must return an
   explicit failure reason on feedback failure or staleness.

## Configuration and compatibility

- `hardware_feedback_rate_hz`: default 50.0, accepted range 20--100 Hz.
- `joint_state_rate`: default 100.0 in real-hardware launch paths.  This is a
  cache publication rate, not a serial polling rate.
- `gripper_feedback_stale_timeout_sec`: default 0.15 s.
- Gripper position ramp remains configurable with the accepted 0.5 rad/s
  default.  Feedback at 50 Hz does not limit this motion speed; command output
  remains 500 Hz.
- Existing position-command torque configuration remains unchanged.  This bus
  fix neither restores 0.15 N.m nor changes the current configured cap.
- The 0--90 mm to 0..-5 rad mapping, verified 85 mm installed opening limit,
  and zero calibration remain unchanged.
- No dependency upgrade is part of this change.

## Diagnostics

Log feedback batch failures with controller identity and exception detail.
Log gripper failure with goal angle, last commanded angle, cached feedback,
feedback timestamp/age, and the reason.  Recovery is logged once when a later
batch succeeds.  Feedback exceptions must never be converted to status code 0.

## Test strategy

All hardware-facing tests use fakes only.  Regression tests must first fail
against the backed-up implementation and prove that:

- a gripper target does not create a second thread;
- the arm callback and gripper command are emitted by the same loop callback;
- feedback is requested at 50 Hz rather than every 500 Hz command tick;
- a single feedback batch covers arm motors and the gripper;
- the ROS publisher reads cache and does not invoke synchronous refresh;
- stale/failed feedback prevents the next gripper command and produces a
  nonzero/unknown status plus an explicit reason;
- action and service completion remain aligned.

No test may open the serial port, enable hardware, send a real trajectory, or
move the real gripper.
