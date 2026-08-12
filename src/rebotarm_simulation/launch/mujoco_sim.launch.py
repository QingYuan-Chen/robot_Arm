"""Start only the explicitly selected, headless MuJoCo ROS adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def _default_python_executable() -> str:
    workspace_root = Path(__file__).resolve().parents[3]
    pinned = workspace_root / "third_party" / "rebotarm_mujoco_venv" / "bin" / "python"
    return str(pinned) if pinned.is_file() else "python3"


def generate_launch_description():
    config = Path(get_package_share_directory("rebotarm_simulation")) / "config" / "mujoco_sim.yaml"
    python_executable = LaunchConfiguration("python_executable")
    mujoco_arm_namespace = LaunchConfiguration("mujoco_arm_namespace")
    initial_joint_positions = LaunchConfiguration("initial_joint_positions")
    enable_virtual_camera = LaunchConfiguration("enable_virtual_camera")
    virtual_camera_width = LaunchConfiguration("virtual_camera_width")
    virtual_camera_height = LaunchConfiguration("virtual_camera_height")
    virtual_camera_rate_hz = LaunchConfiguration("virtual_camera_rate_hz")
    virtual_camera_frame_id = LaunchConfiguration("virtual_camera_frame_id")
    virtual_camera_parent_frame_id = LaunchConfiguration(
        "virtual_camera_parent_frame_id"
    )
    mujoco_gl = LaunchConfiguration("mujoco_gl")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "python_executable",
                default_value=_default_python_executable(),
                description="Python interpreter containing MuJoCo and ROS 2 dependencies",
            ),
            DeclareLaunchArgument("mujoco_arm_namespace", default_value="rebotarm"),
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            DeclareLaunchArgument("enable_virtual_camera", default_value="false"),
            DeclareLaunchArgument("virtual_camera_width", default_value="640"),
            DeclareLaunchArgument("virtual_camera_height", default_value="480"),
            DeclareLaunchArgument("virtual_camera_rate_hz", default_value="15.0"),
            DeclareLaunchArgument(
                "virtual_camera_frame_id",
                default_value="mujoco_fixed_camera_optical_frame",
            ),
            DeclareLaunchArgument(
                "virtual_camera_parent_frame_id", default_value="base_link"
            ),
            DeclareLaunchArgument("mujoco_gl", default_value="egl"),
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                prefix=python_executable,
                parameters=[
                    str(config),
                    {
                        "backend": "mujoco",
                        "headless": True,
                        "arm_namespace": mujoco_arm_namespace,
                        "initial_joint_positions": initial_joint_positions,
                        "virtual_camera.enabled": ParameterValue(
                            enable_virtual_camera, value_type=bool
                        ),
                        "virtual_camera.width": ParameterValue(
                            virtual_camera_width, value_type=int
                        ),
                        "virtual_camera.height": ParameterValue(
                            virtual_camera_height, value_type=int
                        ),
                        "virtual_camera.rate_hz": ParameterValue(
                            virtual_camera_rate_hz, value_type=float
                        ),
                        "virtual_camera.frame_id": virtual_camera_frame_id,
                        "virtual_camera.parent_frame_id": virtual_camera_parent_frame_id,
                    },
                ],
                additional_env={"MUJOCO_GL": mujoco_gl},
            )
        ]
    )
