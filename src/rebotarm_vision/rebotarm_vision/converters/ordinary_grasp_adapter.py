from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any

import numpy as np
import cv2
from geometry_msgs.msg import Pose

from rebotarm_msgs.msg import Detection2D, Detection2DArray, GraspCandidate, GraspCandidateArray, GraspPlan

from ..depth_quality import DepthQualityConfig, evaluate_detection_depth_quality
from ..grasp_candidate_policy import GraspCandidateScoringConfig, score_grasp_candidate


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def as_matrix(self) -> np.ndarray:
        return np.asarray(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )


@dataclass(frozen=True)
class OrdinaryGraspConfig:
    ordinary_grasp_root: Path
    depth_quantile: float = 0.75
    output_frame_id: str = "camera_depth_frame"
    pregrasp_offset_m: float = 0.08


class _Boxes:
    def __init__(self, detections: list[Detection2D]) -> None:
        self._detections = detections

    def __len__(self) -> int:
        return len(self._detections)

    def __getitem__(self, index: int) -> Any:
        det = self._detections[index]
        return type(
            "BoxRow",
            (),
            {
                "xyxy": _TensorLike(
                    [[det.x_min, det.y_min, det.x_max, det.y_max]],
                ),
                "cls": _TensorLike([index]),
                "conf": _TensorLike([det.confidence]),
            },
        )()


class _Obb:
    def __init__(self, detections: list[Detection2D]) -> None:
        self._detections = detections
        self.cls = np.asarray([index for index, _ in enumerate(detections)], dtype=np.float32)
        self.conf = np.asarray([det.confidence for det in detections], dtype=np.float32)
        self.xyxy = np.asarray(
            [[det.x_min, det.y_min, det.x_max, det.y_max] for det in detections],
            dtype=np.float32,
        )
        self.xywhr = np.asarray(
            [
                [
                    det.obb_cx,
                    det.obb_cy,
                    det.obb_w,
                    det.obb_h,
                    det.obb_theta,
                ]
                for det in detections
            ],
            dtype=np.float32,
        )
        self.xyxyxyxy = np.asarray(
            [
                np.asarray(det.obb_points_xy, dtype=np.float32).reshape(4, 2)
                for det in detections
            ],
            dtype=np.float32,
        )

    def __len__(self) -> int:
        return len(self._detections)


class _YoloLikeResult:
    def __init__(self, detections: list[Detection2D], image_shape: tuple[int, int]) -> None:
        self.names = {index: det.class_name for index, det in enumerate(detections)}
        self.boxes = _Boxes(detections)
        self.obb = _Obb(detections) if all(_has_valid_obb(det) for det in detections) else None
        self.masks = _Masks(detections, image_shape) if any(_has_valid_mask(det) for det in detections) else None
        self.orig_shape = image_shape


class _Masks:
    def __init__(self, detections: list[Detection2D], image_shape: tuple[int, int]) -> None:
        height, width = image_shape
        masks = []
        for det in detections:
            mask = np.zeros((height, width), dtype=np.float32)
            if _has_valid_mask(det):
                polygon = np.asarray(det.mask_polygon_xy, dtype=np.float32).reshape(-1, 2)
                cv2.fillPoly(mask, [np.round(polygon).astype(np.int32)], 1.0)
            masks.append(mask)
        self.data = _MaskData(masks)


class _MaskData:
    def __init__(self, masks: list[np.ndarray]) -> None:
        self._masks = masks

    def __len__(self) -> int:
        return len(self._masks)

    def __getitem__(self, index: int):
        return _TensorLike(self._masks[index])


class _TensorLike:
    def __init__(self, values) -> None:
        self._values = np.asarray(values, dtype=np.float32)

    def __getitem__(self, index):
        return _TensorLike(self._values[index])

    def __array__(self, dtype=None):
        if dtype is None:
            return self._values
        return self._values.astype(dtype)

    def __float__(self) -> float:
        return float(self._values.reshape(-1)[0])

    def __int__(self) -> int:
        return int(float(self))

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self._values


def _has_valid_obb(det: Detection2D) -> bool:
    return bool(det.has_obb and len(det.obb_points_xy) == 8)


def _has_valid_mask(det: Detection2D) -> bool:
    return bool(getattr(det, "has_mask", False) and len(getattr(det, "mask_polygon_xy", [])) >= 6)


def _import_ordinary_grasp(root: Path):
    root = Path(root).resolve()
    root_str = str(root)
    if root_str not in sys.path:
        sys.path.insert(0, root_str)
    from utils.ordinary_grasp import estimate_grasps, select_best_grasp
    from utils.transforms import canonicalize_parallel_gripper_tcp_rotation

    return estimate_grasps, select_best_grasp, canonicalize_parallel_gripper_tcp_rotation


def _quaternion_from_matrix(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (matrix[2, 1] - matrix[1, 2]) * s
        qy = (matrix[0, 2] - matrix[2, 0]) * s
        qz = (matrix[1, 0] - matrix[0, 1]) * s
    else:
        idx = int(np.argmax(np.diag(matrix)))
        if idx == 0:
            s = 2.0 * np.sqrt(max(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2], 1e-12))
            qw = (matrix[2, 1] - matrix[1, 2]) / s
            qx = 0.25 * s
            qy = (matrix[0, 1] + matrix[1, 0]) / s
            qz = (matrix[0, 2] + matrix[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(max(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2], 1e-12))
            qw = (matrix[0, 2] - matrix[2, 0]) / s
            qx = (matrix[0, 1] + matrix[1, 0]) / s
            qy = 0.25 * s
            qz = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1], 1e-12))
            qw = (matrix[1, 0] - matrix[0, 1]) / s
            qx = (matrix[0, 2] + matrix[2, 0]) / s
            qy = (matrix[1, 2] + matrix[2, 1]) / s
            qz = 0.25 * s
    quat = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quat /= max(float(np.linalg.norm(quat)), 1e-12)
    return tuple(float(value) for value in quat)


def _pose_from_position_rotation(position: np.ndarray, rotation: np.ndarray) -> Pose:
    qx, qy, qz, qw = _quaternion_from_matrix(rotation)
    pose = Pose()
    pose.position.x = round(float(position[0]), 6)
    pose.position.y = round(float(position[1]), 6)
    pose.position.z = round(float(position[2]), 6)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _empty_plan(frame_id: str, reason: str) -> GraspPlan:
    plan = GraspPlan()
    plan.header.frame_id = frame_id
    plan.valid = False
    plan.source = "ordinary_grasp"
    plan.reason = reason
    return plan


def _source_name(*, has_obb_input: bool, has_mask_input: bool) -> str:
    if has_obb_input:
        return "ordinary_grasp_obb_depth"
    if has_mask_input:
        return "ordinary_grasp_mask_depth"
    return "ordinary_grasp_bbox_depth"


def _candidate_from_grasp(grasp, output_frame_id: str, source: str) -> GraspCandidate:
    candidate = GraspCandidate()
    candidate.header.frame_id = output_frame_id
    candidate.class_name = str(grasp.class_name)
    candidate.confidence = float(grasp.conf)
    candidate.pose = _pose_from_position_rotation(
        np.asarray(grasp.position, dtype=np.float64),
        np.asarray(grasp.tcp_rotation, dtype=np.float64),
    )
    candidate.jaw_width = float(grasp.jaw_width_m)
    candidate.object_length = float(grasp.object_length_m)
    candidate.valid = True
    candidate.source = source
    return candidate


def build_candidate_array_from_grasps(
    grasps: list,
    *,
    output_frame_id: str,
    source: str,
    scoring_config: GraspCandidateScoringConfig | None = None,
) -> GraspCandidateArray:
    scored: list[tuple[float, GraspCandidate]] = []
    for grasp in grasps:
        score = score_grasp_candidate(
            confidence=float(getattr(grasp, "conf", 0.0) or 0.0),
            jaw_width_m=float(getattr(grasp, "jaw_width_m", 0.0) or 0.0),
            valid=bool(getattr(grasp, "is_valid", False)),
            config=scoring_config,
        )
        if score < 0.0:
            continue
        candidate = _candidate_from_grasp(grasp, output_frame_id, source)
        scored.append((score, candidate))

    scored.sort(key=lambda item: item[0], reverse=True)
    array = GraspCandidateArray()
    array.header.frame_id = output_frame_id
    array.candidates = [candidate for _, candidate in scored]
    array.best_index = 0 if array.candidates else -1
    return array


def _plan_from_best_candidate(
    *,
    detections: Detection2DArray,
    candidates: GraspCandidateArray,
    pregrasp_offset_m: float,
    reason_if_empty: str,
) -> GraspPlan:
    if candidates.best_index < 0 or not candidates.candidates:
        return _empty_plan(candidates.header.frame_id, reason_if_empty)
    best_candidate = candidates.candidates[int(candidates.best_index)]
    tcp_rotation = np.eye(3, dtype=np.float64)
    # Recover pregrasp direction from the candidate quaternion through the same canonical TCP x-axis.
    q = (
        float(best_candidate.pose.orientation.x),
        float(best_candidate.pose.orientation.y),
        float(best_candidate.pose.orientation.z),
        float(best_candidate.pose.orientation.w),
    )
    from ..transform_points import quaternion_to_rotation_matrix

    tcp_rotation_list = quaternion_to_rotation_matrix(q)
    tcp_x = np.asarray([tcp_rotation_list[0][0], tcp_rotation_list[1][0], tcp_rotation_list[2][0]], dtype=np.float64)
    grasp_position = np.asarray(
        [
            float(best_candidate.pose.position.x),
            float(best_candidate.pose.position.y),
            float(best_candidate.pose.position.z),
        ],
        dtype=np.float64,
    )
    pregrasp_position = grasp_position - tcp_x * float(pregrasp_offset_m)
    plan = GraspPlan()
    plan.header = detections.header
    plan.header.frame_id = candidates.header.frame_id
    plan.candidate = best_candidate
    plan.grasp_pose = best_candidate.pose
    plan.pregrasp_pose = _pose_from_position_rotation(pregrasp_position, np.asarray(tcp_rotation, dtype=np.float64))
    plan.pregrasp_pose.orientation = best_candidate.pose.orientation
    plan.jaw_width = float(best_candidate.jaw_width)
    plan.valid = True
    plan.source = "ordinary_grasp"
    plan.reason = ""
    return plan


def plan_and_candidates_from_detections_and_depth(
    detections: Detection2DArray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    ordinary_grasp_root: str | Path,
    output_frame_id: str = "camera_depth_frame",
    depth_quantile: float = 0.75,
    pregrasp_offset_m: float = 0.08,
    scoring_config: GraspCandidateScoringConfig | None = None,
    depth_quality_config: DepthQualityConfig | None = None,
) -> tuple[GraspPlan, GraspCandidateArray]:
    if depth_mm is None or depth_mm.size == 0:
        empty = GraspCandidateArray()
        empty.header.frame_id = output_frame_id
        empty.best_index = -1
        return _empty_plan(output_frame_id, "missing depth image"), empty
    depth_quality = depth_quality_config or DepthQualityConfig()
    filtered = []
    rejected_reasons: list[str] = []
    for det in detections.detections:
        if det.confidence <= 0.0:
            continue
        quality = evaluate_detection_depth_quality(det, depth_mm, depth_quality)
        if quality.accepted:
            filtered.append(det)
        else:
            class_name = str(getattr(det, "class_name", "") or "object")
            rejected_reasons.append(
                f"{class_name}:{quality.reason}"
                f"(valid={quality.valid_depth_pixels}, ratio={quality.valid_depth_ratio:.2f}, "
                f"z={quality.z_median_m:.3f}, mad={quality.z_mad_m:.3f})"
            )
    if not filtered:
        empty = GraspCandidateArray()
        empty.header.frame_id = output_frame_id
        empty.best_index = -1
        if rejected_reasons:
            return _empty_plan(
                output_frame_id,
                "depth quality rejected all detections: " + "; ".join(rejected_reasons),
            ), empty
        return _empty_plan(output_frame_id, "no detections"), empty

    has_obb_input = any(_has_valid_obb(det) for det in filtered)
    has_mask_input = any(_has_valid_mask(det) for det in filtered)
    estimate_grasps, _select_best_grasp, canonicalize_tcp_rotation = _import_ordinary_grasp(
        Path(ordinary_grasp_root)
    )
    result = _YoloLikeResult(filtered, depth_mm.shape[:2])
    raw_grasps = estimate_grasps([result], depth_mm, intrinsics.as_matrix(), depth_quantile=depth_quantile)
    grasps = []
    for grasp in raw_grasps:
        if getattr(grasp, "is_valid", False):
            grasp.tcp_rotation = canonicalize_tcp_rotation(grasp.tcp_rotation)
        grasps.append(grasp)

    candidates = build_candidate_array_from_grasps(
        grasps,
        output_frame_id=output_frame_id,
        source=_source_name(has_obb_input=has_obb_input, has_mask_input=has_mask_input),
        scoring_config=scoring_config,
    )
    plan = _plan_from_best_candidate(
        detections=detections,
        candidates=candidates,
        pregrasp_offset_m=pregrasp_offset_m,
        reason_if_empty="ordinary grasp found no valid candidate",
    )
    return plan, candidates


def plan_from_detections_and_depth(
    detections: Detection2DArray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    ordinary_grasp_root: str | Path,
    output_frame_id: str = "camera_depth_frame",
    depth_quantile: float = 0.75,
    pregrasp_offset_m: float = 0.08,
) -> GraspPlan:
    plan, _candidates = plan_and_candidates_from_detections_and_depth(
        detections,
        depth_mm,
        intrinsics,
        ordinary_grasp_root=ordinary_grasp_root,
        output_frame_id=output_frame_id,
        depth_quantile=depth_quantile,
        pregrasp_offset_m=pregrasp_offset_m,
    )
    return plan
