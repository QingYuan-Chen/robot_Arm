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


def test_ubuntu_camera_config_uses_verified_hw_alignment_profiles() -> None:
    config = _read("src/rebotarm_vision/config/camera_ubuntu.yaml")

    assert "camera.color_width: 640" in config
    assert "camera.color_height: 480" in config
    assert "camera.depth_width: 640" in config
    assert "camera.depth_height: 400" in config
    assert "camera.enable_align: true" in config


def test_vision_node_publishes_camera_info_for_both_rgb_and_depth() -> None:
    node = _read("src/rebotarm_vision/rebotarm_vision/vision_node.py")

    assert '"/camera/color/camera_info"' in node
    assert '"/camera/depth/camera_info"' in node
    assert 'self._camera_info_payload("color"' in node
    assert 'self._camera_info_payload("depth"' in node
    assert "except KeyboardInterrupt:" in node
    assert "if rclpy.ok():" in node


def test_ubuntu_launch_uses_installed_config_and_model() -> None:
    launch = _read("src/rebotarm_vision/launch/vision_ubuntu.launch.py")
    setup = _read("src/rebotarm_vision/setup.py")

    assert '"config" / "camera_ubuntu.yaml"' in launch
    assert '"models" / "yolo26s-seg.pt"' in launch
    assert 'DeclareLaunchArgument(' in launch
    assert '"yolo_model_path"' in launch
    assert '"launch/vision_ubuntu.launch.py"' in setup
    assert '"config/camera_ubuntu.yaml"' in setup
    assert 'Path("../../tools/yolo26s-seg.pt")' in setup


def test_vision_dependencies_preserve_ros_numpy_abi() -> None:
    requirements = _read("requirements-vision.txt")

    assert "numpy==1.26.4" in requirements
    assert "pyorbbecsdk2==2.0.18" in requirements
    assert "pyorbbecsdk2==2.1.1" not in requirements


def test_ubuntu_vision_launcher_exports_venv_dependencies_for_ros_entrypoints() -> None:
    launcher = _read("tools/run_ubuntu_vision.sh")

    assert 'sysconfig.get_path("purelib")' in launcher
    assert 'export PYTHONPATH="${vision_site_packages}${PYTHONPATH:+:${PYTHONPATH}}"' in launcher
    assert launcher.index("export PYTHONPATH=") < launcher.index(
        "exec ros2 launch rebotarm_vision"
    )


def test_ubuntu_vision_setup_activates_venv_before_colcon_build() -> None:
    setup = _read("tools/setup_ubuntu_vision.sh")

    activate = 'source "${venv_dir}/bin/activate"'
    build = '"${python_bin}" -m colcon'
    assert activate in setup
    assert build in setup
    assert setup.index(activate) < setup.index(build)
