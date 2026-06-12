from __future__ import annotations

from pathlib import Path
import sys

import pytest


ROS2_ROOT = Path(__file__).resolve().parents[1]
VISION_SRC = ROS2_ROOT / "src" / "rebotarm_vision"
if str(VISION_SRC) not in sys.path:
    sys.path.insert(0, str(VISION_SRC))


def _pose(x: float, y: float, z: float):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def test_candidate_tf_adapter_returns_same_pose_when_frames_match():
    from rebotarm_vision.candidate_tf_adapter import transform_candidate_pose_to_target_frame

    called = False

    def lookup(_target_frame: str, _source_frame: str):
        nonlocal called
        called = True
        raise AssertionError("lookup should not be called when frames already match")

    transformed = transform_candidate_pose_to_target_frame(
        _pose(0.1, 0.2, 0.3),
        source_frame="base_link",
        target_frame="base_link",
        lookup_transform=lookup,
    )

    assert not called
    assert transformed.position.x == pytest.approx(0.1)
    assert transformed.position.y == pytest.approx(0.2)
    assert transformed.position.z == pytest.approx(0.3)


def test_candidate_tf_adapter_uses_lookup_when_frames_differ():
    from geometry_msgs.msg import TransformStamped

    from rebotarm_vision.candidate_tf_adapter import transform_candidate_pose_to_target_frame

    def lookup(target_frame: str, source_frame: str):
        assert target_frame == "base_link"
        assert source_frame == "camera_depth_frame"
        transform = TransformStamped()
        transform.transform.translation.x = 0.3
        transform.transform.translation.y = -0.1
        transform.transform.translation.z = 0.2
        transform.transform.rotation.w = 1.0
        return transform

    transformed = transform_candidate_pose_to_target_frame(
        _pose(0.1, 0.2, 0.3),
        source_frame="camera_depth_frame",
        target_frame="base_link",
        lookup_transform=lookup,
    )

    assert transformed.position.x == pytest.approx(0.4)
    assert transformed.position.y == pytest.approx(0.1)
    assert transformed.position.z == pytest.approx(0.5)
