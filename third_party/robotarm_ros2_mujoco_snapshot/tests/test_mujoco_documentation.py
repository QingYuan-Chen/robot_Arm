from pathlib import Path


ROOT = Path(__file__).parents[1]
README = ROOT / "src" / "rebotarm_simulation" / "README_mujoco.md"
COMMANDS = ROOT / "docs" / "rebotarm_feature_commands.md"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_mujoco_readme_documents_reproducible_install_and_runtime_commands():
    text = _text(README)
    required = (
        "Ubuntu 24.04",
        "ROS 2 Jazzy",
        "Python 3.12",
        "python3 -m venv .venv-mujoco-ros --system-site-packages",
        "source /opt/ros/jazzy/setup.bash",
        "pip install -r src/rebotarm_simulation/requirements-mujoco.txt",
        "setuptools>=68,<80",
        ".venv-mujoco-ros/bin/python -m colcon build --symlink-install --packages-select rebotarm_simulation",
        "MUJOCO_GL=egl",
        "rebotarm_mujoco_health --renderer-timeout",
        "rebotarm_mujoco_cli --headless --duration",
        '"requested_duration"',
        '"achieved_duration"',
        "DISPLAY",
        "XAUTHORITY",
        "rebotarm_mujoco_viewer --duration",
        "PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_health",
        "PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_cli",
        "PYTHONPATH=src/rebotarm_simulation .venv-mujoco-ros/bin/python -m rebotarm_simulation.mujoco_viewer",
    )
    for value in required:
        assert value in text


def test_mujoco_readme_documents_ros_interfaces_examples_and_moveit_safety():
    text = _text(README)
    required = (
        "ros2 launch rebotarm_simulation mujoco_sim.launch.py",
        "/rebotarm/follow_joint_trajectory",
        "/rebotarm/joint_states",
        "/rebotarm/gripper/set",
        "/rebotarm/gripper/state",
        "/rebotarm/trajectory_stop",
        "/clock",
        "ros2 action send_goal",
        "ros2 service call /rebotarm/gripper/set",
        "ros2 service call /rebotarm/trajectory_stop",
        "use_hardware:=false",
        "MoveIt",
        "ros2 launch rebotarm_bringup interactive_system.launch.py use_moveit_preview:=true use_hardware:=false use_moveit_fake_joint_states:=false start_passive_joint_state_publisher:=false use_sim_time:=true",
        "use_local_rviz:=true",
        "fake joint state publisher",
        "passive joint state publisher",
    )
    for value in required:
        assert value in text


def test_mujoco_readme_documents_api_architecture_sync_and_troubleshooting():
    text = _text(README)
    required = (
        "RebotArmMujoco",
        "get_state()",
        "XYZW",
        "save_state()",
        "restore_state()",
        "set_object_pose",
        "云服务器",
        "本地 Ubuntu VM",
        "Gymnasium",
        "奖励函数",
        "训练",
        "备份",
        "哈希",
        "--delete",
        "重复节点",
        "SSH",
        "EGL",
        "goal tolerance",
        "self-collision",
        "use_hardware",
        "rebotarmcontroller",
        "CAN",
        "串口",
        "真实夹爪",
        "shebang",
        "仅结束 launch 父进程",
        "ros2 node list",
        "pgrep -af rebotarm_mujoco_node",
        "ps -fp <confirmed-pid>",
        "kill -TERM <confirmed-pid>",
    )
    for value in required:
        assert value in text
    assert "pkill" not in text
    assert "尚未实现强化学习训练" in text
    assert "已实现强化学习训练" not in text


def test_feature_commands_links_to_mujoco_readme_and_safe_entry_points():
    text = _text(COMMANDS)
    assert "../src/rebotarm_simulation/README_mujoco.md" in text
    assert "ros2 launch rebotarm_simulation mujoco_sim.launch.py" in text
    assert "rebotarm_mujoco_cli --headless --duration" in text
    assert "use_hardware:=false" in text


def test_mujoco_readme_documents_local_urdf_to_mjcf_workflow() -> None:
    text = _text(README)
    required = (
        "src/rebotarm_moveit_config/config/rebotarm.urdf",
        "src/rebotarm_simulation/models/rebotarm/robot.xml",
        "rebotarm_urdf_to_mjcf --repo-root",
        "rebotarm_urdf_to_mjcf --repo-root . --check",
        "mujoco>=3.3,<4",
        "原始 STL",
        "Get-FileHash",
        "sha256sum",
        "不连接实机",
    )
    for value in required:
        assert value in text
