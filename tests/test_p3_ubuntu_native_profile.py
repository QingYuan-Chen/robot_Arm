from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_visual_grasp_bringup_exposes_explicit_ubuntu_native_profile() -> None:
    launch = _read("src/rebotarm_bringup/launch/visual_grasp_system.launch.py")

    assert '"vision_profile"' in launch
    assert 'default_value="network"' in launch
    assert 'choices=["network", "ubuntu_native"]' in launch
    assert '"config", "camera_ubuntu.yaml"' in launch
    assert '"models", "yolo26s-seg.pt"' in launch
    assert "vision_profile" in launch and "ubuntu_native" in launch


def test_ubuntu_native_camera_profile_has_no_network_inputs() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.type: gemini2" in config
    assert "ros.enable_network_detection: false" in config
    assert "camera.network_" in config
    assert "http://" not in config
    assert "https://" not in config


def test_vision_node_latches_camera_failure_and_publishes_empty_detections() -> None:
    node = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert "camera.max_empty_frames" in node
    assert "_publish_empty_detection" in node
    assert "_record_frame_failure" in node
    assert "if not frame_complete:" in node
    assert "empty detections will continue" in node


def test_plan_freshness_rejects_unset_old_and_accepts_recent_stamps() -> None:
    from rebotarm_vision.message_freshness import is_message_fresh, message_age_sec

    unset = SimpleNamespace(sec=0, nanosec=0)
    old = SimpleNamespace(sec=98, nanosec=0)
    recent = SimpleNamespace(sec=100, nanosec=0)

    assert message_age_sec(unset, now_ns=100_000_000_000) is None
    assert not is_message_fresh(unset, now_ns=100_000_000_000, max_age_sec=1.0)
    assert not is_message_fresh(old, now_ns=100_000_000_000, max_age_sec=1.0)
    assert is_message_fresh(recent, now_ns=100_000_000_000, max_age_sec=1.0)


def test_network_failure_returns_empty_payload_instead_of_stale_candidates(monkeypatch) -> None:
    import rebotarm_vision.network_graspnet_client as graspnet_client
    import rebotarm_vision.detector.network_detection_client as detection_client

    def fail(*_args, **_kwargs):
        raise TimeoutError("test timeout")

    monkeypatch.setattr(graspnet_client, "urlopen", fail)
    monkeypatch.setattr(detection_client, "urlopen", fail)

    grasp = graspnet_client.NetworkGraspNetClient(
        graspnet_client.NetworkGraspNetConfig("http://127.0.0.1:1/candidates", 10)
    )
    detection = detection_client.NetworkDetectionClient(
        detection_client.NetworkDetectionConfig("http://127.0.0.1:1/detections", 10)
    )

    assert grasp.fetch()["candidates"] == []
    assert grasp.fetch()["stale"] is True
    assert detection.fetch()["detections"] == []
    assert detection.fetch()["stale"] is True


def test_ubuntu_native_uses_reliable_qos_for_large_image_payloads() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")
    source = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert "ros.image_reliability: reliable" in config
    assert 'self.declare_parameter("ros.image_reliability", "best_effort")' in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "self._image_qos_profile()" in source
