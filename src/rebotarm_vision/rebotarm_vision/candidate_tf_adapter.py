from __future__ import annotations

from copy import deepcopy
from typing import Callable

from geometry_msgs.msg import Pose

from .grasp_preview_sender_node import _transform_from_msg, transform_pose_message


TransformLookup = Callable[[str, str], object]


def transform_candidate_pose_to_target_frame(
    pose: Pose,
    *,
    source_frame: str,
    target_frame: str,
    lookup_transform: TransformLookup,
) -> Pose:
    if not target_frame or not source_frame or source_frame == target_frame:
        return deepcopy(pose)
    transform_msg = lookup_transform(target_frame, source_frame)
    return transform_pose_message(deepcopy(pose), _transform_from_msg(transform_msg))
