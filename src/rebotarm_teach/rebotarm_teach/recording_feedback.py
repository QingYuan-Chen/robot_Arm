"""Build teach records from new, time-aligned ROS feedback only."""

import math

from .teach_recording import TeachSample


def stamp_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def feedback_teach_sample(
    msg, *, joint_names, motor_status, motor_stamps, arm_state,
    now_ns: int, started_ns: int, last_stamp_ns: int | None,
    timeout_sec: float, require_motor_status: bool,
) -> TeachSample | None:
    stamp = stamp_nanoseconds(msg.header.stamp)
    if stamp < started_ns or (last_stamp_ns is not None and stamp <= last_stamp_ns):
        return None
    if not 0 <= now_ns - stamp <= int(timeout_sec * 1_000_000_000):
        raise ValueError("joint feedback timestamp is stale or in the future")
    names = tuple(msg.name)
    if len(names) != len(set(names)) or not set(joint_names).issubset(names):
        raise ValueError("joint feedback is incomplete or duplicated")
    if len(msg.position) != len(names):
        raise ValueError("joint feedback position length mismatch")
    indices = [names.index(name) for name in joint_names]

    def ordered(values):
        if len(values) != len(names):
            raise ValueError("joint feedback vector length mismatch")
        result = tuple(float(values[i]) for i in indices)
        if not all(math.isfinite(value) for value in result):
            raise ValueError("joint feedback contains non-finite values")
        return result

    if require_motor_status:
        if any(motor_stamps.get(name) != stamp for name in joint_names):
            raise ValueError("waiting for motor status from the same feedback batch")
        if any(motor_status.get(name) not in (0, 1) for name in joint_names):
            raise ValueError("motor feedback is unknown or unhealthy")
    return TeachSample(
        stamp=(stamp - started_ns) / 1_000_000_000,
        joint_names=tuple(joint_names), positions=ordered(msg.position),
        velocities=ordered(msg.velocity) if msg.velocity else (),
        efforts=ordered(msg.effort) if msg.effort else (),
        motor_status={name: motor_status[name] for name in joint_names if name in motor_status},
        arm_state=arm_state,
    )
