from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


def test_ubuntu_camera_profile_uses_native_gemini_and_local_yolo() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.type: gemini2" in config
    assert "camera.enable_depth: true" in config
    assert "yolo.device: \"0\"" in config
    assert "ros.enable_detection: true" in config
    assert "ros.enable_network_detection: false" in config
    assert "http://" not in config
    assert "https://" not in config


def test_ubuntu_launch_uses_installed_config_and_model() -> None:
    launch = _read("src/rebotarm_vision/launch/vision_ubuntu.launch.py")
    setup = _read("src/rebotarm_vision/setup.py")

    assert '"config" / "camera_ubuntu.yaml"' in launch
    assert '"models" / "yolo11n-seg.pt"' in launch
    assert '"launch/vision_ubuntu.launch.py"' in setup
    assert '"config/camera_ubuntu.yaml"' in setup
    assert '"models/yolo11n-seg.pt"' in setup


def test_vision_dependencies_preserve_ros_numpy_abi() -> None:
    requirements = _read("requirements-vision.txt")

    assert "numpy==1.26.4" in requirements
    assert "pyorbbecsdk2==2.0.18" in requirements
    assert "pyorbbecsdk2==2.1.1" not in requirements
