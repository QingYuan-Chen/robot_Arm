from __future__ import annotations

import math
import threading

import numpy as np
import pytest

from rebotarm_simulation.virtual_camera import (
    VirtualCameraConfig,
    VirtualCameraFrame,
    VirtualCameraRenderer,
    VirtualCameraWorker,
    VirtualObjectAnnotation,
    bounding_box_from_mask,
    camera_optical_transform,
    metric_depth_to_millimeters,
    pinhole_intrinsics,
    segmentation_mask,
)


def test_virtual_camera_config_normalizes_names_and_rejects_unsafe_values():
    config = VirtualCameraConfig(
        camera_name=" fixed_camera ",
        frame_id="/mujoco_camera_optical_frame/",
        annotation_bodies=("bottle",),
    )
    assert config.camera_name == "fixed_camera"
    assert config.frame_id == "mujoco_camera_optical_frame"
    assert config.parent_body_name == "base_link"
    assert config.parent_frame_id == "base_link"

    with pytest.raises(ValueError, match="rate_hz"):
        VirtualCameraConfig(rate_hz=0.0)
    with pytest.raises(ValueError, match="duplicates"):
        VirtualCameraConfig(annotation_bodies=("bottle", "bottle"))
    with pytest.raises(ValueError, match="must differ"):
        VirtualCameraConfig(frame_id="base_link", parent_frame_id="base_link")


def test_pinhole_intrinsics_use_mujoco_vertical_field_of_view():
    intrinsics = pinhole_intrinsics(640, 480, 45.0)
    expected_focal = 240.0 / math.tan(math.radians(22.5))
    assert intrinsics.fx == pytest.approx(expected_focal)
    assert intrinsics.fy == pytest.approx(expected_focal)
    assert intrinsics.cx == pytest.approx(319.5)
    assert intrinsics.cy == pytest.approx(239.5)


def test_camera_optical_transform_converts_mujoco_axis_convention():
    transform = camera_optical_transform(
        camera_position_world=(0.85, -0.9, 0.55),
        camera_rotation_world_mujoco=np.eye(3),
        parent_position_world=(0.0, 0.0, 0.0),
        parent_rotation_world=np.eye(3),
        parent_frame_id="base_link",
        child_frame_id="camera_optical",
    )
    assert transform.translation_xyz == pytest.approx((0.85, -0.9, 0.55))
    # diag(1,-1,-1) is a pi rotation around +X.
    assert transform.rotation_xyzw == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_metric_depth_conversion_rejects_background_and_out_of_range_pixels():
    depth_m = np.array([[0.5, np.nan, 3.0], [0.0, 1.2344, 1.2346]])
    geometry_mask = np.array([[True, True, True], [True, False, True]])
    depth_mm = metric_depth_to_millimeters(
        depth_m, max_depth_m=2.0, valid_mask=geometry_mask
    )
    assert depth_mm.dtype == np.uint16
    assert depth_mm.tolist() == [[500, 0, 0], [0, 0, 1235]]


def test_segmentation_mask_and_bbox_follow_mujoco_object_id_type_order():
    segmentation = np.full((4, 5, 2), -1, dtype=np.int32)
    segmentation[1:4, 2:4, 0] = 26
    segmentation[1:4, 2:4, 1] = 5
    segmentation[0, 0] = (26, 1)
    mask = segmentation_mask(segmentation, (26,), geom_object_type=5)
    assert mask.sum() == 6
    assert bounding_box_from_mask(mask) == (2, 1, 3, 3)
    assert bounding_box_from_mask(np.zeros((2, 2), dtype=bool)) is None


def test_virtual_annotation_exposes_detection_compatible_rectangle_mask():
    annotation = VirtualObjectAnnotation("bottle", 2, 4, 8, 10)
    assert annotation.center_u == 5
    assert annotation.center_v == 7
    assert annotation.mask_polygon_xy == (2.0, 4.0, 8.0, 4.0, 8.0, 10.0, 2.0, 10.0)


def test_virtual_camera_worker_keeps_renderer_lifecycle_on_one_thread(monkeypatch):
    lifecycle_threads = []
    ready = threading.Event()
    received = threading.Event()
    errors = []
    intrinsics = pinhole_intrinsics(16, 16, 45.0)
    frame = VirtualCameraFrame(
        np.zeros((16, 16, 3), dtype=np.uint8),
        np.zeros((16, 16), dtype=np.uint16),
        (),
    )

    class FakeRenderer:
        def __init__(self):
            self.intrinsics = intrinsics
            self.extrinsics = camera_optical_transform(
                camera_position_world=(0.0, 0.0, 0.0),
                camera_rotation_world_mujoco=np.eye(3),
                parent_position_world=(0.0, 0.0, 0.0),
                parent_rotation_world=np.eye(3),
                parent_frame_id="base_link",
                child_frame_id="camera_optical",
            )

        def render(self):
            lifecycle_threads.append(("render", threading.get_ident()))
            return frame

        def close(self):
            lifecycle_threads.append(("close", threading.get_ident()))

    fake_renderer = FakeRenderer()

    def fake_from_simulation(_cls, _simulation, _config):
        lifecycle_threads.append(("create", threading.get_ident()))
        return fake_renderer

    monkeypatch.setattr(
        VirtualCameraRenderer,
        "from_simulation",
        classmethod(fake_from_simulation),
    )

    class Access:
        @staticmethod
        def run(operation):
            return operation(object())

    worker = VirtualCameraWorker(
        Access(),
        VirtualCameraConfig(width=16, height=16),
        on_frame=lambda actual, _intrinsics, stamp: (
            received.set()
            if actual is frame and stamp == (1, 2)
            else errors.append(RuntimeError("unexpected frame callback"))
        ),
        on_ready=lambda _intrinsics, _extrinsics: ready.set(),
        on_error=errors.append,
    )
    assert ready.wait(1.0)
    assert worker.submit((1, 2)) is True
    assert received.wait(1.0)
    assert worker.close() is True
    assert errors == []
    thread_ids = {thread_id for _event, thread_id in lifecycle_threads}
    assert len(thread_ids) == 1
    assert thread_ids != {threading.get_ident()}
