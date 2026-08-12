from __future__ import annotations

from pathlib import Path
import importlib.util
import os
import sys
import types
from typing import Any

import numpy as np


DEFAULT_NUM_POINT = 20000
DEFAULT_WORKSPACE_MIN_DEPTH_M = 0.05
DEFAULT_WORKSPACE_MAX_DEPTH_M = 1.5


def install_training_knn_fallback() -> None:
    """Provide the training-only knn import without building the legacy extension.

    The official model imports label-generation helpers even in eval mode. Those
    helpers import ``knn_modules``, although inference never calls them. A small
    torch implementation keeps that import valid and is also correct if a caller
    explicitly uses the helper on modest tensors.
    """
    if "knn_modules" in sys.modules:
        return
    import torch

    module = types.ModuleType("knn_modules")

    def knn(ref, query, k=1):
        ref_points = ref.transpose(1, 2).contiguous()
        query_points = query.transpose(1, 2).contiguous()
        distances = torch.cdist(query_points.float(), ref_points.float())
        return torch.topk(distances, int(k), dim=-1, largest=False).indices.transpose(1, 2) + 1

    module.knn = knn
    sys.modules["knn_modules"] = module


def load_grasp_group(model_root: str):
    """Load GraspGroup without importing graspnetAPI's evaluation-only stack."""
    configured = os.environ.get("GRASPNET_API_ROOT", "").strip()
    candidates = []
    if configured:
        candidates.append(Path(configured).expanduser())
    candidates.append(Path(model_root).resolve().parent / "graspnetAPI" / "graspnetAPI")
    for api_root in candidates:
        grasp_path = api_root / "grasp.py"
        if not grasp_path.is_file():
            continue
        package = types.ModuleType("graspnetAPI")
        package.__path__ = [str(api_root)]
        package.__package__ = "graspnetAPI"
        sys.modules["graspnetAPI"] = package
        spec = importlib.util.spec_from_file_location("graspnetAPI.grasp", grasp_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load GraspNetAPI from {grasp_path}")
        module = importlib.util.module_from_spec(spec)
        sys.modules["graspnetAPI.grasp"] = module
        spec.loader.exec_module(module)
        return module.GraspGroup
    from graspnetAPI import GraspGroup

    return GraspGroup


def add_windows_dll_directories() -> None:
    if os.name != "nt" or not hasattr(os, "add_dll_directory"):
        return
    candidates = [
        Path(sys.prefix) / "Lib" / "site-packages" / "torch" / "lib",
        Path(os.environ.get("CUDA_PATH", "")) / "bin",
        Path(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.6\bin"),
        Path(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.1\bin"),
    ]
    for path in candidates:
        if path.is_dir():
            os.add_dll_directory(str(path))


def build_scene_cloud(
    *,
    color_bgr: np.ndarray,
    depth_mm: np.ndarray,
    camera_info: dict[str, Any],
) -> tuple[np.ndarray, np.ndarray]:
    depth = np.asarray(depth_mm)
    color = np.asarray(color_bgr)
    if depth.ndim != 2:
        raise ValueError("depth_mm must be a 2D image")
    if color.shape[:2] != depth.shape:
        raise ValueError("color_bgr and depth_mm must have the same image size")

    height, width = depth.shape
    z = depth.astype(np.float32) * float(camera_info.get("depth_scale_m", 0.001))
    min_depth_m = float(camera_info.get("workspace_min_depth_m", DEFAULT_WORKSPACE_MIN_DEPTH_M))
    max_depth_m = float(camera_info.get("workspace_max_depth_m", DEFAULT_WORKSPACE_MAX_DEPTH_M))
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    v, u = np.nonzero(valid)
    if len(u) == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    z_values = z[v, u]
    x = (u.astype(np.float32) - cx) * z_values / fx
    y = (v.astype(np.float32) - cy) * z_values / fy
    points = np.column_stack((x, y, z_values)).astype(np.float32)
    colors = color[v, u, :3].astype(np.float32)[:, ::-1] / 255.0
    return points, colors.astype(np.float32)


def sample_cloud(points: np.ndarray, colors: np.ndarray, *, num_point: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) == 0:
        return points, colors
    count = int(num_point)
    if len(points) >= count:
        indices = np.random.choice(len(points), count, replace=False)
    else:
        extra = np.random.choice(len(points), count - len(points), replace=True)
        indices = np.concatenate([np.arange(len(points)), extra], axis=0)
    return points[indices].astype(np.float32), colors[indices].astype(np.float32)


def graspnet_array_to_candidates(
    grasp_array,
    *,
    class_name: str,
    max_grasps: int,
    target_detection: dict[str, Any] | None = None,
    camera_info: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    array = np.asarray(grasp_array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if target_detection is not None and camera_info is not None:
        array = filter_grasp_array_by_detection_projection(array, target_detection=target_detection, camera_info=camera_info)
    candidates: list[dict[str, Any]] = []
    for row in array[: max(0, int(max_grasps))]:
        if row.size < 16:
            continue
        rotation = row[4:13].reshape(3, 3)
        translation = row[13:16]
        candidates.append(
            {
                "source": "windows_graspnet_baseline",
                "class_name": str(class_name),
                "score": float(row[0]),
                "width_m": float(row[1]),
                "height_m": float(row[2]),
                "depth_m": float(row[3]),
                "rotation_matrix": rotation.astype(float).tolist(),
                "translation_xyz": translation.astype(float).tolist(),
                "object_length_m": float(row[2]),
                "target_filter": "yolo_projection" if target_detection is not None and camera_info is not None else "",
            }
        )
    return candidates


def filter_grasp_array_by_detection_projection(
    grasp_array,
    *,
    target_detection: dict[str, Any],
    camera_info: dict[str, Any],
) -> np.ndarray:
    array = np.asarray(grasp_array, dtype=np.float32)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    if array.size == 0:
        return array
    translations = array[:, 13:16]
    u, v = project_points_to_image(translations, camera_info=camera_info)
    keep = detection_contains_pixels(target_detection, u=u, v=v)
    return array[keep]


def project_points_to_image(points_xyz: np.ndarray, *, camera_info: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray(points_xyz, dtype=np.float32).reshape(-1, 3)
    z = points[:, 2]
    fx = float(camera_info["fx"])
    fy = float(camera_info["fy"])
    cx = float(camera_info["cx"])
    cy = float(camera_info["cy"])
    valid_z = np.where(np.abs(z) > 1e-6, z, np.nan)
    u = fx * points[:, 0] / valid_z + cx
    v = fy * points[:, 1] / valid_z + cy
    return u, v


def detection_contains_pixels(target_detection: dict[str, Any], *, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    u_values = np.asarray(u, dtype=np.float32).reshape(-1)
    v_values = np.asarray(v, dtype=np.float32).reshape(-1)
    finite = np.isfinite(u_values) & np.isfinite(v_values)
    polygon = _detection_polygon(target_detection)
    if polygon is not None:
        return finite & _points_in_polygon(u_values, v_values, polygon)
    x_min, y_min, x_max, y_max = _detection_bbox(target_detection)
    return finite & (u_values >= x_min) & (u_values <= x_max) & (v_values >= y_min) & (v_values <= y_max)


def _detection_polygon(detection: dict[str, Any]) -> np.ndarray | None:
    raw = None
    mask = detection.get("mask")
    if isinstance(mask, dict):
        raw = mask.get("polygon_xy")
    if raw is None:
        raw = detection.get("mask_polygon_xy")
    if raw is None:
        return None
    arr = np.asarray(raw, dtype=np.float32).reshape(-1, 2)
    if arr.shape[0] < 3:
        return None
    return arr


def _detection_bbox(detection: dict[str, Any]) -> tuple[float, float, float, float]:
    if "bbox_xyxy" in detection:
        raw = np.asarray(detection.get("bbox_xyxy"), dtype=np.float32).reshape(-1)
        if raw.size >= 4:
            return float(raw[0]), float(raw[1]), float(raw[2]), float(raw[3])
    return (
        float(detection.get("x_min", 0.0)),
        float(detection.get("y_min", 0.0)),
        float(detection.get("x_max", 0.0)),
        float(detection.get("y_max", 0.0)),
    )


def _points_in_polygon(u: np.ndarray, v: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    x = np.asarray(u, dtype=np.float32)
    y = np.asarray(v, dtype=np.float32)
    px = polygon[:, 0]
    py = polygon[:, 1]
    inside = np.zeros(x.shape, dtype=bool)
    j = len(polygon) - 1
    for i in range(len(polygon)):
        crosses = ((py[i] > y) != (py[j] > y)) & (
            x < (px[j] - px[i]) * (y - py[i]) / ((py[j] - py[i]) + 1e-12) + px[i]
        )
        inside ^= crosses
        j = i
    return inside


class GraspNetBaselineInference:
    def __init__(
        self,
        *,
        model_root: str,
        checkpoint_path: str,
        device: str = "cuda:0",
        num_point: int = DEFAULT_NUM_POINT,
        collision_thresh: float = 0.01,
        voxel_size: float = 0.01,
    ) -> None:
        self.model_root = str(model_root)
        self.checkpoint_path = str(checkpoint_path)
        self.device = str(device)
        self.num_point = int(num_point)
        self.collision_thresh = float(collision_thresh)
        self.voxel_size = float(voxel_size)
        self._torch = None
        self._GraspGroup = None
        self._ModelFreeCollisionDetector = None
        self._pred_decode = None
        self.net = self._load_network()

    def _load_network(self):
        add_windows_dll_directories()
        root = Path(self.model_root)
        for child in ("models", "dataset", "utils"):
            path = str(root / child)
            if path not in sys.path:
                sys.path.insert(0, path)
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        import torch
        install_training_knn_fallback()
        from graspnet import GraspNet, pred_decode
        GraspGroup = load_grasp_group(self.model_root)

        self._torch = torch
        self._pred_decode = pred_decode
        self._GraspGroup = GraspGroup
        try:
            from collision_detector import ModelFreeCollisionDetector

            self._ModelFreeCollisionDetector = ModelFreeCollisionDetector
        except Exception:
            self._ModelFreeCollisionDetector = None

        net = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            cylinder_radius=0.05,
            hmin=-0.02,
            hmax_list=[0.01, 0.02, 0.03, 0.04],
            is_training=False,
        )
        checkpoint = torch.load(self.checkpoint_path, map_location=self.device)
        state_dict = checkpoint.get("model_state_dict", checkpoint)
        net.load_state_dict(state_dict)
        net.to(self.device)
        net.eval()
        return net

    def infer(
        self,
        *,
        color_bgr: np.ndarray,
        depth_mm: np.ndarray,
        detections: list[dict[str, Any]],
        camera_info: dict[str, Any],
        max_grasps: int,
    ) -> list[dict[str, Any]]:
        if not detections:
            return []
        detection = max(detections, key=lambda item: float(item.get("confidence", 0.0)))
        points, colors = build_scene_cloud(
            color_bgr=color_bgr,
            depth_mm=depth_mm,
            camera_info=camera_info,
        )
        if len(points) == 0:
            return []
        points_sampled, colors_sampled = sample_cloud(points, colors, num_point=self.num_point)
        grasp_array = self._infer_grasp_array(points_sampled, colors_sampled, full_points=points)
        return graspnet_array_to_candidates(
            grasp_array,
            class_name=str(detection.get("class_name", "")),
            max_grasps=max_grasps,
            target_detection=detection,
            camera_info=camera_info,
        )

    def _infer_grasp_array(self, points: np.ndarray, colors: np.ndarray, *, full_points: np.ndarray):
        torch = self._torch
        if torch is None or self._pred_decode is None or self._GraspGroup is None:
            raise RuntimeError("GraspNet backend is not loaded")
        cloud = torch.from_numpy(points[np.newaxis].astype(np.float32)).to(self.device)
        color = torch.from_numpy(colors[np.newaxis].astype(np.float32)).to(self.device)
        end_points = {"point_clouds": cloud, "cloud_colors": color}
        with torch.no_grad():
            end_points = self.net(end_points)
            decoded = self._pred_decode(end_points)
        grasp_group = self._GraspGroup(decoded[0].detach().cpu().numpy())
        if self._ModelFreeCollisionDetector is not None and self.collision_thresh > 0:
            detector = self._ModelFreeCollisionDetector(full_points, voxel_size=self.voxel_size)
            collision_mask = detector.detect(grasp_group, approach_dist=0.05, collision_thresh=self.collision_thresh)
            grasp_group = grasp_group[~collision_mask]
        try:
            grasp_group = grasp_group.nms()
        except ModuleNotFoundError:
            pass
        grasp_group = grasp_group.sort_by_score()
        return getattr(grasp_group, "grasp_group_array", np.asarray(grasp_group))
