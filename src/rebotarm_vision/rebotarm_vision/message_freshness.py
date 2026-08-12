from __future__ import annotations


def stamp_to_ns(stamp) -> int | None:
    """Convert a ROS-style stamp to nanoseconds, rejecting an unset stamp."""

    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec == 0 and nanosec == 0:
        return None
    return sec * 1_000_000_000 + nanosec


def message_age_sec(stamp, *, now_ns: int) -> float | None:
    stamp_ns = stamp_to_ns(stamp)
    if stamp_ns is None:
        return None
    return (int(now_ns) - stamp_ns) / 1_000_000_000.0


def is_message_fresh(
    stamp,
    *,
    now_ns: int,
    max_age_sec: float,
    max_future_sec: float = 0.25,
) -> bool:
    age_sec = message_age_sec(stamp, now_ns=now_ns)
    if age_sec is None:
        return False
    return -abs(float(max_future_sec)) <= age_sec <= max(float(max_age_sec), 0.0)
