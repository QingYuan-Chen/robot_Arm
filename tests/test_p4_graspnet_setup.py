import base64
import importlib.util
from pathlib import Path
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


def test_graspnet_run_script_uses_isolated_environment_and_localhost_service() -> None:
    script = _read("tools/run_ubuntu_graspnet_service.sh")

    assert ".venv-graspnet/bin/python" in script
    assert "src/rebotarm_vision" in script
    assert "ubuntu_graspnet_service.py" in script


def test_ubuntu_graspnet_ros_profile_posts_rgbd_to_local_service() -> None:
    config = _read("src/rebotarm_vision/config/graspnet_ubuntu.yaml")
    node = _read("src/rebotarm_vision/rebotarm_vision/graspnet_baseline_node.py")
    setup = _read("src/rebotarm_vision/setup.py")

    assert "source_mode: local_service" in config
    assert "local_infer_url: http://127.0.0.1:8081/infer" in config
    assert "depth_scale_m_per_unit: 0.001" in config
    assert '"local_service"' in node
    assert "LocalGraspNetClient" in node
    assert "max_input_skew_ms" in node
    assert "closest_timestamped_frame" in node
    assert "QoSReliabilityPolicy.BEST_EFFORT" in node
    assert "depth=1" in node
    assert '"config/graspnet_ubuntu.yaml"' in setup


def test_local_graspnet_selects_cached_frame_nearest_detection_timestamp() -> None:
    from rebotarm_vision.local_graspnet_client import closest_timestamped_frame

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
        "max_grasps": 10,
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
    )
    round_trip = decode_inference_request(encoded)

    assert request.timestamp_ns == 123456789
    assert request.frame_id == "camera_depth_frame"
    assert request.depth_m.dtype == np.float32
    assert request.depth_m[0, 0] == pytest.approx(0.30)
    assert request.camera_info == {"fx": 300.0, "fy": 301.0, "cx": 1.0, "cy": 0.5, "depth_scale_m": 1.0}
    assert request.detection["class_name"] == "box"
    assert round_trip.timestamp_ns == request.timestamp_ns
    assert round_trip.sent_at_unix_ns == 987654321
    assert np.array_equal(round_trip.color_bgr, request.color_bgr)
    assert np.array_equal(round_trip.depth_m, request.depth_m)


def test_graspnet_contract_rejects_non_metric_depth() -> None:
    from rebotarm_vision.graspnet_service_contract import ContractError, decode_inference_request

    payload = _contract_payload()
    payload["depth"]["unit"] = "mm"
    with pytest.raises(ContractError, match="depth.unit must be m"):
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
        def infer(self, **kwargs):
            assert kwargs["depth_mm"][0, 0] == pytest.approx(0.30)
            assert kwargs["camera_info"]["depth_scale_m"] == 1.0
            return [{"score": 0.8, "translation_xyz": [0.0, 0.0, 0.3]}]

    service.backend = FakeBackend()
    response = service.infer(_contract_payload())

    assert response["backend_configured"] is True
    assert response["stale"] is False
    assert response["timestamp_ns"] == 123456789
    assert response["frame_id"] == "camera_depth_frame"
    assert len(response["candidates"]) == 1


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
