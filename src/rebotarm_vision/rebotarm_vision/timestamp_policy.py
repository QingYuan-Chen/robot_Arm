from __future__ import annotations


def select_frame_timestamp_ns(
    camera_timestamp_ns: int | None,
    receipt_timestamp_ns: int,
    *,
    use_sim_time: bool,
) -> int:
    """Select a frame stamp in the node clock domain.

    Hardware system timestamps are useful with wall-clock ROS time, but they
    cannot be compared with a simulation clock.  In simulation-time
    compositions, stamp real-camera frames at receipt using the active ROS
    clock so freshness checks remain meaningful.
    """

    if use_sim_time:
        return int(receipt_timestamp_ns)
    if camera_timestamp_ns is not None and int(camera_timestamp_ns) > 0:
        return int(camera_timestamp_ns)
    return int(receipt_timestamp_ns)
