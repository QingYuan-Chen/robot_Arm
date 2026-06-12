from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def _load_wrapper():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "tools" / "graspnet_baseline_inference.py"
    spec = importlib.util.spec_from_file_location("graspnet_baseline_inference", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_graspnet_wrapper_converts_graspnet_array_rows_to_json_candidates():
    module = _load_wrapper()
    row = np.array(
        [
            0.91,
            0.042,
            0.02,
            0.03,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.10,
            -0.02,
            0.35,
            3.0,
        ],
        dtype=np.float32,
    )

    candidates = module.graspnet_array_to_candidates(
        np.asarray([row]),
        class_name="bottle",
        max_grasps=5,
    )

    assert candidates[0]["class_name"] == "bottle"
    assert candidates[0]["score"] == pytest.approx(0.91)
    assert candidates[0]["width_m"] == pytest.approx(0.042)
    assert candidates[0]["translation_xyz"] == pytest.approx([0.10, -0.02, 0.35])
    assert np.asarray(candidates[0]["rotation_matrix"]) == pytest.approx(np.eye(3))


def test_graspnet_wrapper_builds_scene_cloud_from_full_depth_image():
    module = _load_wrapper()
    depth_mm = np.array(
        [
            [0, 0, 0, 0],
            [0, 500, 500, 0],
            [0, 500, 500, 0],
            [0, 0, 0, 0],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 2.0, "depth_scale_m": 0.001}

    points, colors = module.build_scene_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
    )

    assert points.shape == (4, 3)
    assert colors.shape == (4, 3)
    assert points[:, 2].tolist() == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_graspnet_wrapper_scene_cloud_rejects_far_depth_outliers():
    module = _load_wrapper()
    depth_mm = np.array(
        [
            [500, 500, 4000],
            [500, 65511, 500],
        ],
        dtype=np.uint16,
    )
    color_bgr = np.zeros((2, 3, 3), dtype=np.uint8)
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 1.0, "cy": 1.0, "depth_scale_m": 0.001}

    points, _colors = module.build_scene_cloud(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        camera_info=camera_info,
    )

    assert points.shape[0] == 4
    assert points[:, 2].max() == pytest.approx(0.5)


def test_graspnet_inference_defaults_to_scene_cloud_not_detection_crop(monkeypatch):
    module = _load_wrapper()

    class FakeInference(module.GraspNetBaselineInference):
        def _load_network(self):
            self._torch = object()
            self._pred_decode = object()
            self._GraspGroup = object()
            return object()

        def _infer_grasp_array(self, points, colors, *, full_points):
            self.seen_point_count = len(points)
            self.seen_full_point_count = len(full_points)
            return np.asarray(
                [
                    [
                        0.9,
                        0.04,
                        0.02,
                        0.03,
                        1,
                        0,
                        0,
                        0,
                        1,
                        0,
                        0,
                        0,
                        1,
                        0.0,
                        0.0,
                        0.5,
                        -1,
                    ]
                ],
                dtype=np.float32,
            )

    monkeypatch.setattr(
        module,
        "sample_cloud",
        lambda points, colors, *, num_point: (points.astype(np.float32), colors.astype(np.float32)),
    )

    depth_mm = np.ones((4, 4), dtype=np.uint16) * 500
    color_bgr = np.zeros((4, 4, 3), dtype=np.uint8)
    detection = {"bbox_xyxy": [1.0, 1.0, 3.0, 3.0], "class_name": "bottle", "confidence": 0.9}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 2.0, "cy": 2.0, "depth_scale_m": 0.001}

    backend = FakeInference(model_root="", checkpoint_path="")
    candidates = backend.infer(
        color_bgr=color_bgr,
        depth_mm=depth_mm,
        detections=[detection],
        camera_info=camera_info,
        max_grasps=1,
    )

    assert backend.seen_point_count == 16
    assert backend.seen_full_point_count == 16
    assert candidates[0]["class_name"] == "bottle"


def test_graspnet_candidates_are_filtered_by_target_bbox_projection():
    module = _load_wrapper()
    grasp_array = np.asarray(
        [
            [
                0.9,
                0.04,
                0.02,
                0.03,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
                0.0,
                0.0,
                1.0,
                -1,
            ],
            [
                0.8,
                0.04,
                0.02,
                0.03,
                1,
                0,
                0,
                0,
                1,
                0,
                0,
                0,
                1,
                1.0,
                0.0,
                1.0,
                -1,
            ],
        ],
        dtype=np.float32,
    )
    detection = {"bbox_xyxy": [90.0, 90.0, 110.0, 110.0], "class_name": "bottle"}
    camera_info = {"fx": 100.0, "fy": 100.0, "cx": 100.0, "cy": 100.0}

    candidates = module.graspnet_array_to_candidates(
        grasp_array,
        class_name="bottle",
        max_grasps=2,
        target_detection=detection,
        camera_info=camera_info,
    )

    assert len(candidates) == 1
    assert candidates[0]["score"] == pytest.approx(0.9)
    assert candidates[0]["target_filter"] == "yolo_projection"
