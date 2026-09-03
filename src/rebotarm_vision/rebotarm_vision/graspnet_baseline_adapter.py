from __future__ import annotations

from dataclasses import dataclass
import importlib
import importlib.util
import math
import sys
from pathlib import Path
from typing import Any, Iterable, Protocol

import numpy as np

from rebotarm_msgs.msg import Detection2D, GraspCandidate, GraspCandidateArray


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class PointCloudCrop:
    points: np.ndarray
    colors: np.ndarray


def closest_timestamped_frame(frames, target_timestamp_ns: int):
    """Return the cached frame nearest to a non-zero target timestamp."""
    if target_timestamp_ns <= 0 or not frames:
        return None
    return min(frames, key=lambda item: abs(int(item[0]) - target_timestamp_ns))


@dataclass(frozen=True)
class GraspNetPrediction:
    score: float
    translation_xyz: tuple[float, float, float]
    rotation_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    width_m: float
    object_length_m: float = 0.0


class GraspNetBackendProtocol(Protocol):
    @property
    def available(self) -> bool:
        ...

    def infer(self, *, points: np.ndarray, colors: np.ndarray, max_grasps: int) -> list[GraspNetPrediction]:
        ...


def build_point_cloud_for_detection(
    depth_mm: np.ndarray,
    color_bgr: np.ndarray | None,
    detection: Detection2D,
    intrinsics: CameraIntrinsics,
    *,
    min_depth_m: float,
    max_depth_m: float,
) -> PointCloudCrop:
    depth = np.asarray(depth_mm)
    if depth.ndim != 2:
        raise ValueError("depth image must be a 2D array")
    height, width = depth.shape
    x_min = max(0, min(width - 1, int(detection.x_min)))
    y_min = max(0, min(height - 1, int(detection.y_min)))
    x_max = max(x_min + 1, min(width, int(detection.x_max)))
    y_max = max(y_min + 1, min(height, int(detection.y_max)))

    roi_depth_m = depth[y_min:y_max, x_min:x_max].astype(np.float32) * 0.001
    valid = np.isfinite(roi_depth_m) & (roi_depth_m >= float(min_depth_m)) & (roi_depth_m <= float(max_depth_m))
    v_local, u_local = np.nonzero(valid)
    if len(u_local) == 0:
        return PointCloudCrop(points=np.empty((0, 3), dtype=np.float32), colors=np.empty((0, 3), dtype=np.float32))

    z = roi_depth_m[v_local, u_local]
    u = u_local.astype(np.float32) + float(x_min)
    v = v_local.astype(np.float32) + float(y_min)
    x = (u - float(intrinsics.cx)) * z / float(intrinsics.fx)
    y = (v - float(intrinsics.cy)) * z / float(intrinsics.fy)
    points = np.column_stack((x, y, z)).astype(np.float32)

    if color_bgr is None:
        colors = np.ones_like(points, dtype=np.float32) * 0.5
    else:
        color = np.asarray(color_bgr)
        if color.shape[:2] != depth.shape:
            raise ValueError("color and depth image sizes must match")
        bgr = color[v.astype(np.int32), u.astype(np.int32), :3].astype(np.float32) / 255.0
        colors = bgr[:, ::-1].astype(np.float32)
    return PointCloudCrop(points=points, colors=colors)


def predictions_to_candidate_array(
    predictions: Iterable[GraspNetPrediction],
    *,
    frame_id: str,
    class_name: str,
    max_candidates: int,
    source: str = "graspnet_baseline",
) -> GraspCandidateArray:
    array = GraspCandidateArray()
    array.header.frame_id = frame_id
    array.best_index = -1
    for prediction in list(predictions)[: max(0, int(max_candidates))]:
        candidate = GraspCandidate()
        candidate.header.frame_id = frame_id
        candidate.class_name = class_name
        candidate.confidence = float(prediction.score)
        candidate.pose.position.x = float(prediction.translation_xyz[0])
        candidate.pose.position.y = float(prediction.translation_xyz[1])
        candidate.pose.position.z = float(prediction.translation_xyz[2])
        qx, qy, qz, qw = _rotation_matrix_to_quaternion(prediction.rotation_matrix)
        candidate.pose.orientation.x = qx
        candidate.pose.orientation.y = qy
        candidate.pose.orientation.z = qz
        candidate.pose.orientation.w = qw
        candidate.jaw_width = float(prediction.width_m)
        candidate.object_length = float(prediction.object_length_m)
        candidate.valid = True
        candidate.source = source
        array.candidates.append(candidate)
    if array.candidates:
        array.best_index = 0
    return array


def payload_to_candidate_array(
    payload: dict,
    *,
    fallback_frame_id: str,
    max_candidates: int,
) -> GraspCandidateArray:
    predictions: list[GraspNetPrediction] = []
    class_names: list[str] = []
    for item in list(payload.get("candidates", []))[: max(0, int(max_candidates))]:
        if not isinstance(item, dict):
            continue
        try:
            predictions.append(_prediction_from_raw(item))
            class_names.append(str(item.get("class_name", payload.get("class_name", ""))))
        except Exception:
            continue
    source = str(payload.get("source", "windows_graspnet_baseline"))
    frame_id = str(payload.get("frame_id", fallback_frame_id) or fallback_frame_id)
    class_name = class_names[0] if class_names else str(payload.get("class_name", ""))
    candidates = predictions_to_candidate_array(
        predictions,
        frame_id=frame_id,
        class_name=class_name,
        max_candidates=max_candidates,
        source=source,
    )
    for candidate, name in zip(candidates.candidates, class_names):
        candidate.class_name = name
    timestamp_ns = int(payload.get("timestamp_ns", 0) or 0)
    if timestamp_ns > 0:
        candidates.header.stamp.sec = timestamp_ns // 1_000_000_000
        candidates.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        for candidate in candidates.candidates:
            candidate.header.stamp.sec = candidates.header.stamp.sec
            candidate.header.stamp.nanosec = candidates.header.stamp.nanosec
    return candidates


class GraspNetBaselineBackend:
    """Thin optional wrapper around an installed GraspNet baseline inference module.

    The upstream GraspNet baseline repository is a research codebase, so this
    adapter intentionally expects a tiny local wrapper module with a stable API:
    `GraspNetBaselineInference(model_root, checkpoint_path, device).infer(...)`.
    """

    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str = "",
        device: str = "cuda:0",
        module_name: str = "graspnet_baseline_inference",
    ) -> None:
        self.model_root = str(model_root).strip()
        self.checkpoint_path = str(checkpoint_path).strip()
        self.device = str(device).strip()
        self.module_name = str(module_name).strip()
        self._runner = None
        if not self.model_root:
            return
        root = Path(self.model_root)
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        module = importlib.import_module(self.module_name)
        runner_cls = getattr(module, "GraspNetBaselineInference")
        self._runner = runner_cls(
            model_root=self.model_root,
            checkpoint_path=self.checkpoint_path,
            device=self.device,
        )

    @property
    def available(self) -> bool:
        return self._runner is not None

    def infer(self, *, points: np.ndarray, colors: np.ndarray, max_grasps: int) -> list[GraspNetPrediction]:
        if self._runner is None:
            raise RuntimeError("GraspNet baseline backend is not configured")
        raw_predictions = self._runner.infer(points=points, colors=colors, max_grasps=max_grasps)
        return [_prediction_from_raw(item) for item in raw_predictions]


class InProcessGraspNetBackend:
    """Load the full RGB-D GraspNet runner directly inside the ROS candidate node.

    This is the Ubuntu production path.  It deliberately preserves the current
    full-scene collision cloud, object mask/depth separation, deterministic
    sampling, projection filter, and jaw-width filter implemented by the local
    runner while removing the localhost JSON/HTTP transport.
    """

    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str = "cuda:0",
        module_name: str = "graspnet_baseline_inference",
        module_path: str = "",
        num_point: int = 20_000,
    ) -> None:
        self.model_root = str(model_root).strip()
        self.checkpoint_path = str(checkpoint_path).strip()
        self.device = str(device).strip()
        self.module_name = str(module_name).strip()
        self.module_path = str(module_path).strip()
        self.num_point = int(num_point)
        self._runner = None
        self.backend_error = "model_root_or_checkpoint_unset"
        if not self.model_root or not self.checkpoint_path:
            return
        try:
            model_path = Path(self.model_root).expanduser()
            checkpoint = Path(self.checkpoint_path).expanduser()
            if not model_path.is_dir():
                raise FileNotFoundError(f"model root not found: {model_path}")
            if not checkpoint.is_file():
                raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
            module = _load_inprocess_backend_module(
                self.module_name,
                module_path=self.module_path,
            )
            runner_cls = getattr(module, "GraspNetBaselineInference")
            self._runner = runner_cls(
                model_root=str(model_path),
                checkpoint_path=str(checkpoint),
                device=self.device,
                num_point=self.num_point,
            )
            self.backend_error = ""
        except Exception as exc:
            self.backend_error = f"{type(exc).__name__}: {exc}"

    @property
    def available(self) -> bool:
        return self._runner is not None

    @property
    def last_stage_counts(self) -> dict[str, Any]:
        value = getattr(self._runner, "last_stage_counts", None)
        return dict(value) if isinstance(value, dict) else {}

    def infer(
        self,
        *,
        timestamp_ns: int,
        frame_id: str,
        color_bgr: np.ndarray,
        depth_m: np.ndarray,
        camera_info: dict[str, float],
        detection: dict[str, Any],
        max_grasps: int,
        max_jaw_width_m: float | None,
    ) -> dict[str, Any]:
        if self._runner is None:
            raise RuntimeError(
                f"GraspNet in-process backend is not configured: {self.backend_error}"
            )
        color = np.ascontiguousarray(color_bgr, dtype=np.uint8)
        depth = np.ascontiguousarray(depth_m, dtype=np.float32)
        if color.ndim != 3 or color.shape[2] != 3:
            raise ValueError("color_bgr must have shape HxWx3")
        if depth.ndim != 2 or color.shape[:2] != depth.shape:
            raise ValueError("depth_m must match color image dimensions")
        if not np.isfinite(depth).all() or np.any(depth < 0.0):
            raise ValueError("depth_m must contain finite non-negative values")
        if int(timestamp_ns) <= 0:
            raise ValueError("timestamp_ns must be positive")
        if not str(frame_id).strip():
            raise ValueError("frame_id must be non-empty")
        intrinsics = {
            key: float(camera_info[key]) for key in ("fx", "fy", "cx", "cy")
        }
        intrinsics["depth_scale_m"] = 1.0
        candidates = self._runner.infer(
            color_bgr=color,
            # The canonical runner keeps this historical argument name, but
            # depth_scale_m=1.0 makes the float32 array explicitly metric.
            depth_mm=depth,
            detections=[dict(detection)],
            camera_info=intrinsics,
            max_grasps=int(max_grasps),
            max_jaw_width_m=max_jaw_width_m,
        )
        return {
            "source": "ubuntu_inprocess_graspnet",
            "backend_configured": True,
            "stale": False,
            "timestamp_ns": int(timestamp_ns),
            "frame_id": str(frame_id),
            "class_name": str(detection.get("class_name", "")),
            "candidates": list(candidates),
        }


def _load_inprocess_backend_module(module_name: str, *, module_path: str = ""):
    configured_path = Path(module_path).expanduser() if module_path else None
    if configured_path is not None:
        return _load_python_module_from_path(module_name, configured_path)
    try:
        return importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        if exc.name != module_name:
            raise
        candidates: list[Path] = []
        try:
            from ament_index_python.packages import get_package_share_directory

            candidates.append(
                Path(get_package_share_directory("rebotarm_vision"))
                / "graspnet_backend"
                / f"{module_name}.py"
            )
        except Exception:
            pass
        source = Path(__file__).resolve()
        for parent in source.parents:
            candidates.append(parent / "tools" / f"{module_name}.py")
        for candidate in candidates:
            if candidate.is_file():
                return _load_python_module_from_path(module_name, candidate)
        raise


def _load_python_module_from_path(module_name: str, path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"backend module not found: {path}")
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load backend module from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _prediction_from_raw(item) -> GraspNetPrediction:
    if isinstance(item, GraspNetPrediction):
        return item
    if isinstance(item, dict):
        rotation = item.get("rotation_matrix", item.get("rotation"))
        translation = item.get("translation_xyz", item.get("translation"))
        return GraspNetPrediction(
            score=float(item.get("score", item.get("confidence", 0.0))),
            translation_xyz=_tuple3(translation),
            rotation_matrix=_matrix3(rotation),
            width_m=float(item.get("width_m", item.get("width", 0.0))),
            object_length_m=float(item.get("object_length_m", item.get("object_length", 0.0))),
        )
    return GraspNetPrediction(
        score=float(getattr(item, "score", getattr(item, "confidence", 0.0))),
        translation_xyz=_tuple3(getattr(item, "translation_xyz", getattr(item, "translation", None))),
        rotation_matrix=_matrix3(getattr(item, "rotation_matrix", getattr(item, "rotation", None))),
        width_m=float(getattr(item, "width_m", getattr(item, "width", 0.0))),
        object_length_m=float(getattr(item, "object_length_m", getattr(item, "object_length", 0.0))),
    )


def _tuple3(values) -> tuple[float, float, float]:
    arr = np.asarray(values, dtype=np.float64).reshape(-1)
    if arr.size < 3:
        raise ValueError("expected at least 3 values")
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _matrix3(values) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    arr = np.asarray(values, dtype=np.float64).reshape(3, 3)
    return tuple(tuple(float(v) for v in row) for row in arr)  # type: ignore[return-value]


def _rotation_matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (float(qx / norm), float(qy / norm), float(qz / norm), float(qw / norm))
