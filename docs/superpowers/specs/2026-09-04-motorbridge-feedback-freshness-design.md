# MotorBridge Feedback Freshness Design

## Problem and confirmed root cause

`rebotarmcontroller` currently treats this sequence as proof of fresh hardware
feedback:

```text
request_feedback() -> poll_feedback_once() -> get_state() -> stamp now
```

That proof is invalid. MotorBridge 0.4.6 keeps the last decoded Damiao state in
an internal cache. `poll_feedback_once()` can return successfully without
receiving a frame, while `get_state()` can still return the old finite cache.
The controller then gives that old cache a new application timestamp. This is
the source of the false-fresh feedback and the resulting false gripper position.

The motor's persistent absolute zero is a separate concern. This fix must not
run startup homing, add a calibration gate, call `set_zero_position()`, or infer
freshness from whether the numeric position changed.

## Freshness contract

For every accepted Damiao feedback frame, MotorBridge assigns a monotonically
increasing `uint64` receive sequence to the motor. State and sequence are read
atomically through an additive API:

```c
int32_t motor_handle_get_state_with_sequence(
    MotorHandle *motor,
    MotorState *out_state,
    uint64_t *out_sequence);
```

The Python binding exposes:

```python
Motor.get_state_with_sequence() -> tuple[MotorState | None, int]
```

Sequence `0` means that no feedback frame has ever been accepted. Two frames
with identical position, velocity, torque and status still have distinct
sequences. This is essential because a stationary motor legitimately reports
the same numbers repeatedly.

The interface is additive: the existing `get_state()` ABI and Python method
remain unchanged. Non-Damiao handles return a clear unsupported-operation
error from the new API rather than fabricating a sequence.

## Dependency delivery

The upstream source remains MotorBridge tag `v0.4.6`, commit
`38b8a5681887514b301dbcab96e01a473cbd7173`. The repository stores a small,
reviewable source patch rather than a native binary or a second vendored Git
repository. `tools/setup_motorbridge_fresh_feedback.py` clones that exact
commit into the ignored build area, verifies the commit, applies the patch,
builds a wheel, smoke-tests it in an isolated virtual environment, and only
installs to the user Python when explicitly passed `--install-user`.

The patched Python distribution uses local version
`0.4.6+rebotarm.1` and advertises `features.feedback_sequence=true` through
`abi_capabilities()`. Runtime code fails closed with an installation command
if the additive API is absent. It never silently falls back to the old
request/poll/cache timestamp heuristic.

## Controller acquisition model

`rebotarmcontroller` remains the only hardware owner. It maintains one
verified sample per motor containing the decoded state, receive sequence, and
the application monotonic time at which that new sequence was observed.

The 500 Hz hardware loop must not block for a serial-response timeout. Its
50 Hz feedback scheduler therefore pipelines requests:

1. Inspect any outstanding per-motor request baselines.
2. Accept a motor sample only if its current sequence is greater than the
   sequence captured before that request.
3. Update only that motor's verified cache and timestamp.
4. Keep an unsatisfied request pending for a bounded response window while
   re-requesting at the configured feedback cadence.
5. On deadline expiry, record a per-motor feedback error and start a new
   bounded request generation; never refresh the old sample's timestamp.

Disabled lifecycle gates use the same sequence rule but may wait synchronously
for a bounded period because no 500 Hz command loop owns the bus. A force
refresh succeeds only after every required motor has advanced beyond its
pre-request sequence.

Arm freshness is derived from all six verified joint samples. Gripper
freshness is independent. A missing gripper response cannot make an old arm
sample fresh, and a healthy arm response cannot make an old gripper sample
fresh.

## Single trusted cache

Only the feedback acquisition path may call
`Motor.get_state_with_sequence()`. When connected, these consumers read the
verified controller cache and never the raw MotorBridge cache:

- joint state and status publication;
- gripper state publication and dashboard data;
- gripper position start, arrival and stall decisions;
- arm/gripper lifecycle status validation;
- low-level command defaults when a field is omitted.

The cached values remain available for diagnostics after a miss, but status is
reported as `255/unknown` and the sample age/error remains visible. Reading or
publishing a sample never changes its freshness timestamp.

## Failure and safety behavior

- A missing sequence advance never counts as successful feedback.
- During an active gripper command, gripper feedback error/staleness stops
  target progression and queues the existing zero-stiffness neutral command
  for the sole hardware writer.
- A shared controller I/O exception continues to mark the affected bus group
  unavailable and follows the repository's communication-loss safety policy.
- No startup homing, automatic zero write, visual parameter change, TCP change,
  force change, jaw mapping change, or motion is part of this work.

## Verification

MotorBridge tests must prove that two identical decoded frames increment the
sequence and that state/sequence are returned together. Controller tests must
prove:

- a finite old cache with no sequence advance does not update freshness;
- identical numeric state with a sequence advance is accepted;
- arm and gripper update independently;
- delayed feedback inside the bounded window is accepted;
- missing feedback expires without stamping old data;
- publishers and gripper control paths consume only the verified cache;
- the unpatched MotorBridge API is rejected before hardware operation.

All automated verification is software-only. It does not constitute hardware
acceptance. A later physical test requires a separate explicit authorization
and must capture the raw sequence, controller sample, ROS gripper topic and
observed physical position together.
