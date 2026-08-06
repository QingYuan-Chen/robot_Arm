from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_mujoco_adapter_entrypoint_and_launch_are_installed():
    setup_text = (ROOT / "src/rebotarm_simulation/setup.py").read_text(encoding="utf-8")
    package_text = (ROOT / "src/rebotarm_simulation/package.xml").read_text(encoding="utf-8")

    assert "rebotarm_mujoco_adapter = rebotarm_simulation.mujoco_ros_adapter_node:main" in setup_text
    assert 'glob("launch/*.launch.py")' in setup_text
    assert "<exec_depend>control_msgs</exec_depend>" in package_text
    assert "<exec_depend>rebotarm_msgs</exec_depend>" in package_text


def test_mujoco_moveit_launch_starts_adapter_and_moveit_without_fake_joint_states():
    launch_text = (
        ROOT / "src/rebotarm_simulation/launch/mujoco_moveit_sim.launch.py"
    ).read_text(encoding="utf-8")

    assert 'executable="rebotarm_mujoco_adapter"' in launch_text
    assert 'demo.launch.py' in launch_text
    assert '"use_fake_joint_states": "false"' in launch_text
    assert 'DeclareLaunchArgument("model_xml"' in launch_text
    assert 'DeclareLaunchArgument("metrics_dir"' in launch_text
    assert 'DeclareLaunchArgument("metrics_sample_stride"' in launch_text
    assert 'DeclareLaunchArgument("use_mujoco_viewer"' in launch_text
    assert 'DeclareLaunchArgument("python_executable"' in launch_text
    assert "prefix=python_executable" in launch_text


def test_mujoco_ros_adapter_node_owns_required_ros_interfaces():
    node_text = (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/mujoco_ros_adapter_node.py"
    ).read_text(encoding="utf-8")

    assert "ActionServer(" in node_text
    assert "FollowJointTrajectory" in node_text
    assert '"/{self._arm_namespace}/joint_states"' in node_text
    assert '"/{self._arm_namespace}/follow_joint_trajectory"' in node_text
    assert '"/{self._arm_namespace}/trajectory_stop"' in node_text
    assert '"/{self._arm_namespace}/gripper/set"' in node_text
    assert '"/{self._arm_namespace}/gripper/state"' in node_text
    assert '"path_tolerance_rad"' in node_text
    assert '"goal_tolerance_rad"' in node_text
    assert '"metrics_sample_stride"' in node_text
    assert '"use_mujoco_viewer"' in node_text
    assert "mujoco.viewer" in node_text


def test_mujoco_cli_exposes_step_response_suite_command():
    cli_text = (
        ROOT / "src/rebotarm_simulation/rebotarm_simulation/mujoco_cli.py"
    ).read_text(encoding="utf-8")

    assert '"step-response-suite"' in cli_text
    assert "default_step_response_targets" in cli_text
    assert "max_rms_error" in cli_text
