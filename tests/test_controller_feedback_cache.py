from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from rebotarmcontroller.ros_publishers import JointStatePublisher


class _Recorder:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class _Clock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace())


class _Node:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return SimpleNamespace(warn=self.warnings.append)


class _CacheOnlyHardware:
    joint_names = [f"joint{i}" for i in range(1, 7)]

    def __init__(self) -> None:
        self.refresh_calls = 0

    def refresh_feedback_if_due(self) -> None:
        self.refresh_calls += 1

    def get_cached_joint_state(self):
        return np.zeros(6), np.zeros(6), np.zeros(6)

    def get_joint_state(self):
        raise AssertionError("publisher attempted synchronous serial feedback")

    def get_joint_status_codes(self):
        return [1] * 6

    def get_gripper_state(self):
        return -1.0, 0.0, 0.0, 1

    def gripper_position_m(self):
        return 0.018


def test_joint_state_publisher_reads_validated_cache_without_serial_io() -> None:
    """Calling the old synchronous getter from the 100 Hz timer is the bus bug."""
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()

    publisher.publish()

    assert publisher._hardware.refresh_calls == 1
    assert len(publisher._publisher.messages) == 1
    assert len(publisher._gripper_state_publisher.messages) == 1


def test_joint_state_publisher_reports_status_when_feedback_cache_is_invalid() -> None:
    publisher = JointStatePublisher.__new__(JointStatePublisher)
    publisher._node = _Node()
    publisher._hardware = _CacheOnlyHardware()
    publisher._hardware.get_cached_joint_state = lambda: (_ for _ in ()).throw(
        RuntimeError("arm feedback stale")
    )
    publisher._publisher = _Recorder()
    publisher._joint_state_publishers = {
        name: _Recorder() for name in publisher._hardware.joint_names
    }
    publisher._gripper_state_publisher = _Recorder()
    status_calls: list[int] = []
    publisher.publish_status = lambda: status_calls.append(1)

    publisher.publish()

    assert status_calls == [1]
    assert publisher._publisher.messages == []
    assert publisher._node.warnings == ["joint state read failed: arm feedback stale"]
