import base64
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import time

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_graspnet_requirements_pin_isolated_cuda_stack() -> None:
    requirements = _read("requirements-graspnet.txt")

    assert "numpy==1.26.4" in requirements
    assert "scipy==1.11.4" in requirements
    assert "open3d==0.19.0" in requirements
    assert "torch==2.11.0+cu128" in requirements
    assert "torchvision==0.26.0+cu128" in requirements


def test_graspnet_setup_uses_independent_venv_and_no_model_download() -> None:
    script = _read("tools/setup_ubuntu_graspnet.sh")

    assert 'python3 -m venv "${venv_dir}"' in script
    assert "--system-site-packages" not in script
    assert "env PYTHONPATH=" in script
    assert '"${python_bin}" -m pip check' in script
    assert "GRASPNET_MODEL_ROOT" in script
    assert "not downloaded" in script


def test_visual_grasp_launch_uses_isolated_graspnet_python_without_http() -> None:
    launch = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert ".venv-graspnet" in launch
    assert 'prefix=graspnet_python_executable' in launch
    assert '"graspnet_python_executable"' in launch
    assert "default_value=_graspnet_python_executable()" in launch
    assert '"vision_yolo_model_path"' in launch
    assert '"yolo_model_path": vision_yolo_model_path' in launch


def test_graspnet_full_scene_viewer_reuses_pre_sampling_cloud_builder() -> None:
    viewer = _read("tools/view_graspnet_scene_cloud.py")

    assert "from graspnet_baseline_inference import build_scene_cloud" in viewer
    assert "sample_cloud" not in viewer
    assert 'default="/camera/color/image_raw"' in viewer
    assert 'default="/camera/depth/image_raw"' in viewer
    assert 'default="/camera/depth/camera_info"' in viewer
    assert "0.05-1.5 m full scene" in viewer


def _scene_viewer_module():
    path = ROOT / "tools/view_graspnet_scene_cloud.py"
    spec = importlib.util.spec_from_file_location("view_graspnet_scene_cloud", path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    tools_path = str(ROOT / "tools")
    added_path = tools_path not in sys.path
    if added_path:
        sys.path.insert(0, tools_path)
    try:
        spec.loader.exec_module(module)
    finally:
        if added_path:
            sys.path.remove(tools_path)
    return module


def _candidate(
    *,
    score: float,
    xyz: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
):
    return SimpleNamespace(
        confidence=score,
        jaw_width=0.073,
        object_length=0.021,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
            orientation=SimpleNamespace(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        ),
    )


def test_graspnet_scene_viewer_converts_ros_candidates_for_open3d_in_top_n_order() -> None:
    module = _scene_viewer_module()
    half_sqrt = np.sqrt(0.5)
    message = SimpleNamespace(
        candidates=[
            _candidate(
                score=0.91,
                xyz=(0.10, -0.02, 0.35),
                quaternion=(0.0, 0.0, half_sqrt, half_sqrt),
            ),
            _candidate(
                score=0.72,
                xyz=(0.11, -0.01, 0.36),
                quaternion=(0.0, 0.0, 0.0, 1.0),
            ),
        ]
    )

    converted = module.candidate_array_to_visualizer_candidates(message, top_n=1)

    assert len(converted) == 1
    assert converted[0]["score"] == pytest.approx(0.91)
    assert converted[0]["width_m"] == pytest.approx(0.073)
    assert converted[0]["height_m"] == pytest.approx(0.021)
    assert converted[0]["translation_xyz"] == pytest.approx([0.10, -0.02, 0.35])
    np.testing.assert_allclose(
        converted[0]["rotation_matrix"],
        np.asarray([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]),
        atol=1e-6,
    )


def test_graspnet_scene_viewer_rejects_invalid_quaternion_without_losing_valid_candidate() -> None:
    module = _scene_viewer_module()
    message = SimpleNamespace(
        candidates=[
            _candidate(score=0.95, xyz=(0.0, 0.0, 0.3), quaternion=(0.0, 0.0, 0.0, 0.0)),
            _candidate(score=0.80, xyz=(0.1, 0.0, 0.3), quaternion=(0.0, 0.0, 0.0, 1.0)),
        ]
    )

    converted = module.candidate_array_to_visualizer_candidates(message, top_n=2)

    assert len(converted) == 1
    assert converted[0]["score"] == pytest.approx(0.80)
    assert converted[0]["translation_xyz"] == pytest.approx([0.1, 0.0, 0.3])


def test_graspnet_scene_viewer_matches_candidate_to_nearest_rgbd_frames_with_skew_gate() -> None:
    module = _scene_viewer_module()
    colors = [(100, "old_color"), (205, "matched_color"), (390, "future_color")]
    depths = [(90, "old_depth"), (210, "matched_depth"), (410, "future_depth")]

    matched = module.match_candidate_rgbd_frames(
        candidate_stamp_ns=200,
        color_frames=colors,
        depth_frames=depths,
        max_skew_ns=15,
    )

    assert matched == ((205, "matched_color"), (210, "matched_depth"))
    assert module.match_candidate_rgbd_frames(
        candidate_stamp_ns=300,
        color_frames=colors,
        depth_frames=depths,
        max_skew_ns=15,
    ) is None


def test_graspnet_scene_viewer_matches_timestamped_camera_info_and_rejects_frame_mismatch() -> None:
    module = _scene_viewer_module()
    message = SimpleNamespace(header=SimpleNamespace(frame_id="camera_depth_frame"))
    depth = SimpleNamespace(header=SimpleNamespace(frame_id="camera_depth_frame"))
    info = SimpleNamespace(
        header=SimpleNamespace(frame_id="camera_depth_frame"),
        width=640,
        height=480,
        k=[500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
    )

    matched = module.match_candidate_sensor_frames(
        candidate_stamp_ns=200,
        color_frames=[(201, "color")],
        depth_frames=[(202, depth)],
        camera_info_frames=[(203, info)],
        max_skew_ns=5,
    )

    assert matched == ((201, "color"), (202, depth), (203, info))
    assert module.overlay_geometry_is_consistent(
        candidate_message=message,
        depth_message=depth,
        camera_info_message=info,
        depth_shape=(480, 640),
    )
    info.header.frame_id = "wrong_frame"
    assert not module.overlay_geometry_is_consistent(
        candidate_message=message,
        depth_message=depth,
        camera_info_message=info,
        depth_shape=(480, 640),
    )


def test_graspnet_scene_viewer_decodes_row_padded_ros_images() -> None:
    module = _scene_viewer_module()
    color = SimpleNamespace(
        encoding="bgr8",
        height=2,
        width=2,
        step=8,
        data=bytes([1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88]),
    )
    depth = SimpleNamespace(
        encoding="16UC1",
        height=2,
        width=2,
        step=6,
        is_bigendian=False,
        data=bytes([1, 0, 2, 0, 99, 99, 3, 0, 4, 0, 88, 88]),
    )

    np.testing.assert_array_equal(
        module._color_bgr(color),
        np.asarray([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]),
    )
    np.testing.assert_array_equal(
        module._depth_mm(depth),
        np.asarray([[1, 2], [3, 4]], dtype=np.uint16),
    )


def test_graspnet_scene_viewer_only_shuts_down_live_rclpy_context() -> None:
    module = _scene_viewer_module()

    class FakeRclpy:
        def __init__(self, live: bool) -> None:
            self.live = live
            self.shutdown_calls = 0

        def ok(self) -> bool:
            return self.live

        def shutdown(self) -> None:
            self.shutdown_calls += 1

    already_stopped = FakeRclpy(live=False)
    module.shutdown_rclpy_if_live(already_stopped)
    assert already_stopped.shutdown_calls == 0

    live = FakeRclpy(live=True)
    module.shutdown_rclpy_if_live(live)
    assert live.shutdown_calls == 1


def test_graspnet_scene_viewer_cleanup_continues_when_window_close_fails() -> None:
    module = _scene_viewer_module()
    events = []

    class FakeVisualizer:
        def close(self) -> None:
            events.append("close")
            raise RuntimeError("window close failed")

    class FakeNode:
        def destroy_node(self) -> None:
            events.append("destroy_node")

    class FakeRclpy:
        @staticmethod
        def ok() -> bool:
            return True

        @staticmethod
        def shutdown() -> None:
            events.append("shutdown")

    with pytest.raises(RuntimeError, match="window close failed"):
        module.cleanup_viewer_runtime(
            visualizer=FakeVisualizer(),
            node=FakeNode(),
            rclpy_module=FakeRclpy,
        )

    assert events == ["close", "destroy_node", "shutdown"]


def test_ubuntu_graspnet_ros_profile_runs_inprocess_without_http() -> None:
    config = _read("src/rebotarm_vision/config/graspnet_ubuntu.yaml")
    node = _read("src/rebotarm_vision/rebotarm_vision/graspnet_baseline_node.py")
    setup = _read("src/rebotarm_vision/setup.py")

    assert "source_mode: in_process" in config
    assert "http://" not in config
    assert "depth_scale_m_per_unit: 0.001" in config
    assert "max_jaw_width_m: 0.085" in config
    assert '"in_process"' in node
    assert "InProcessGraspNetBackend" in node
    assert "LocalGraspNetClient" not in node
    assert "max_input_skew_ms" in node
    assert 'self.declare_parameter("max_jaw_width_m", 0.085)' in node
    assert 'max_jaw_width_m=float(self.get_parameter("max_jaw_width_m").value)' in node
    assert "closest_timestamped_frame" in node
    assert "QoSReliabilityPolicy.BEST_EFFORT" in node
    assert "depth=1" in node
    assert '"config/graspnet_ubuntu.yaml"' in setup


def test_inprocess_graspnet_selects_cached_frame_nearest_detection_timestamp() -> None:
    from rebotarm_vision.graspnet_baseline_adapter import closest_timestamped_frame

    frames = [(100, "old"), (220, "match"), (400, "future")]

    assert closest_timestamped_frame(frames, 210) == (220, "match")
    assert closest_timestamped_frame(frames, 0) is None
    assert closest_timestamped_frame([], 210) is None


def test_graspnet_environment_check_treats_empty_paths_as_unset() -> None:
    checker = _read("tools/check_ubuntu_graspnet_env.py")

    assert 'os.environ.get("GRASPNET_MODEL_ROOT", "").strip()' in checker
    assert 'Path(model_root_raw).expanduser() if model_root_raw else None' in checker
    assert 'model_root={\'set\' if model_root is not None else \'unset\'}' in checker
    assert 'checkpoint={\'set\' if checkpoint is not None else \'unset\'}' in checker


def _service_module():
    path = ROOT / "tools/ubuntu_graspnet_service.py"
    spec = importlib.util.spec_from_file_location("ubuntu_graspnet_service", path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _contract_payload() -> dict:
    color = np.zeros((2, 3, 3), dtype=np.uint8)
    depth = np.full((2, 3), 0.30, dtype=np.float32)
    return {
        "contract_version": "1.0",
        "timestamp_ns": 123456789,
        "frame_id": "camera_depth_frame",
        "rgb": {
            "encoding": "bgr8",
            "width": 3,
            "height": 2,
            "data_base64": base64.b64encode(color.tobytes()).decode("ascii"),
        },
        "depth": {
            "encoding": "32FC1",
            "unit": "m",
            "width": 3,
            "height": 2,
            "data_base64": base64.b64encode(depth.tobytes()).decode("ascii"),
        },
        "intrinsics": {"fx": 300.0, "fy": 301.0, "cx": 1.0, "cy": 0.5},
        "bbox": {
            "x_min": 0,
            "y_min": 0,
            "x_max": 3,
            "y_max": 2,
            "confidence": 0.9,
            "class_name": "box",
        },
        "mask": {
            "polygon_xy": [0.25, 0.25, 2.75, 0.25, 2.75, 1.75, 0.25, 1.75],
        },
        "max_grasps": 10,
        "max_jaw_width_m": 0.082,
    }


def test_graspnet_contract_preserves_metric_depth_intrinsics_bbox_and_header() -> None:
    from rebotarm_vision.graspnet_service_contract import decode_inference_request, encode_inference_request

    request = decode_inference_request(_contract_payload())
    encoded = encode_inference_request(
        timestamp_ns=request.timestamp_ns,
        sent_at_unix_ns=987654321,
        frame_id=request.frame_id,
        color_bgr=request.color_bgr,
        depth_m=request.depth_m,
        intrinsics=request.camera_info,
        bbox=request.detection,
        max_grasps=request.max_grasps,
        max_jaw_width_m=request.max_jaw_width_m,
        mask_polygon_xy=request.detection.get("mask_polygon_xy"),
    )
    round_trip = decode_inference_request(encoded)

    assert request.timestamp_ns == 123456789
    assert request.frame_id == "camera_depth_frame"
    assert request.depth_m.dtype == np.float32
    assert request.depth_m[0, 0] == pytest.approx(0.30)
    assert request.camera_info == {"fx": 300.0, "fy": 301.0, "cx": 1.0, "cy": 0.5, "depth_scale_m": 1.0}
    assert request.detection["class_name"] == "box"
    assert request.max_jaw_width_m == pytest.approx(0.082)
    assert request.detection["mask_polygon_xy"] == pytest.approx(
        [0.25, 0.25, 2.75, 0.25, 2.75, 1.75, 0.25, 1.75]
    )
    assert round_trip.timestamp_ns == request.timestamp_ns
    assert round_trip.max_jaw_width_m == pytest.approx(0.082)
    assert round_trip.sent_at_unix_ns == 987654321
    assert np.array_equal(round_trip.color_bgr, request.color_bgr)
    assert np.array_equal(round_trip.depth_m, request.depth_m)
    assert round_trip.detection["mask_polygon_xy"] == pytest.approx(
        request.detection["mask_polygon_xy"]
    )


def test_graspnet_contract_rejects_non_metric_depth() -> None:
    from rebotarm_vision.graspnet_service_contract import ContractError, decode_inference_request

    payload = _contract_payload()
    payload["depth"]["unit"] = "mm"
    with pytest.raises(ContractError, match="depth.unit must be m"):
        decode_inference_request(payload)


def test_graspnet_contract_rejects_non_positive_jaw_width_limit() -> None:
    from rebotarm_vision.graspnet_service_contract import ContractError, decode_inference_request

    payload = _contract_payload()
    payload["max_jaw_width_m"] = 0.0
    with pytest.raises(ContractError, match="max_jaw_width_m must be positive"):
        decode_inference_request(payload)


def test_local_service_health_and_unconfigured_inference_fail_closed() -> None:
    module = _service_module()
    service = module.GraspNetService(
        model_root="", checkpoint_path="", device="cuda:0", backend_module="unused", max_input_age_ms=0
    )

    assert service.health()["status"] == "unconfigured"
    assert service.health()["backend_configured"] is False
    with pytest.raises(RuntimeError, match="not configured"):
        service.infer(_contract_payload())


def test_local_service_response_preserves_timestamp_and_frame_id() -> None:
    module = _service_module()
    service = module.GraspNetService(
        model_root="", checkpoint_path="", device="cuda:0", backend_module="unused", max_input_age_ms=0
    )

    class FakeBackend:
        last_stage_counts = {
            "raw": 20,
            "after_collision": 14,
            "after_nms": 12,
            "after_score_sort": 12,
            "after_projection": 7,
            "after_jaw_width": 3,
            "published": 1,
            "empty_reason": "",
        }

        def infer(self, **kwargs):
            assert kwargs["depth_mm"][0, 0] == pytest.approx(0.30)
            assert kwargs["camera_info"]["depth_scale_m"] == 1.0
            assert kwargs["max_jaw_width_m"] == pytest.approx(0.082)
            return [{"score": 0.8, "translation_xyz": [0.0, 0.0, 0.3]}]

    service.backend = FakeBackend()
    response = service.infer(_contract_payload())

    assert response["backend_configured"] is True
    assert response["stale"] is False
    assert response["timestamp_ns"] == 123456789
    assert response["frame_id"] == "camera_depth_frame"
    assert len(response["candidates"]) == 1


def test_local_service_logs_structured_graspnet_stage_counts(capsys) -> None:
    module = _service_module()
    service = module.GraspNetService(
        model_root="", checkpoint_path="", device="cuda:0", backend_module="unused", max_input_age_ms=0
    )

    class FakeBackend:
        last_stage_counts = {
            "scene_points": 1000,
            "object_points": 200,
            "raw": 20,
            "after_collision": 14,
            "after_nms": 12,
            "after_score_sort": 12,
            "after_projection": 7,
            "after_jaw_width": 3,
            "published": 1,
            "empty_reason": "",
        }

        def infer(self, **_kwargs):
            return [{"score": 0.8, "translation_xyz": [0.0, 0.0, 0.3]}]

    service.backend = FakeBackend()
    service.infer(_contract_payload())
    record = json.loads(capsys.readouterr().out)

    assert record["event"] == "graspnet_stage_counts"
    assert record["class_name"] == "box"
    assert record["raw"] == 20
    assert record["after_collision"] == 14
    assert record["after_nms"] == 12
    assert record["after_projection"] == 7
    assert record["after_jaw_width"] == 3
    assert record["published"] == 1
    assert record["published_top_score"] == pytest.approx(0.8)


def test_local_service_rejects_stale_input_without_calling_backend() -> None:
    module = _service_module()
    service = module.GraspNetService(
        model_root="", checkpoint_path="", device="cuda:0", backend_module="unused", max_input_age_ms=100
    )

    class FailIfCalled:
        def infer(self, **kwargs):
            raise AssertionError("stale input must not reach inference")

    service.backend = FailIfCalled()
    payload = _contract_payload()
    payload["timestamp_ns"] = time.time_ns() - 1_000_000_000
    with pytest.raises(module.StaleInputError, match="exceeds limit") as exc_info:
        service.infer(payload)

    assert exc_info.value.request.frame_id == "camera_depth_frame"


def test_local_service_uses_transport_time_when_ros_stamp_is_sim_time() -> None:
    module = _service_module()
    service = module.GraspNetService(
        model_root="", checkpoint_path="", device="cuda:0", backend_module="unused", max_input_age_ms=100
    )

    class FakeBackend:
        def infer(self, **_kwargs):
            return []

    service.backend = FakeBackend()
    payload = _contract_payload()
    payload["timestamp_ns"] = 80_000_000_000
    payload["sent_at_unix_ns"] = time.time_ns()

    response = service.infer(payload)

    assert response["stale"] is False
    assert response["timestamp_ns"] == 80_000_000_000
