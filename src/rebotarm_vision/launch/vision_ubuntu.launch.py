from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vision_share = Path(get_package_share_directory("rebotarm_vision"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("yolo_device", default_value="0"),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=str(vision_share / "models" / "yolo26s-seg.pt"),
            ),
            DeclareLaunchArgument(
                "handeye_config",
                default_value=str(vision_share / "config" / "handeye.yaml"),
            ),
            DeclareLaunchArgument("start_ordinary_grasp", default_value="false"),
            DeclareLaunchArgument("ordinary_grasp_root", default_value=""),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(vision_share / "launch" / "vision.launch.py")
                ),
                launch_arguments={
                    "camera_config": str(
                        vision_share / "config" / "camera_ubuntu.yaml"
                    ),
                    "yolo_model_path": LaunchConfiguration("yolo_model_path"),
                    "yolo_device": LaunchConfiguration("yolo_device"),
                    "handeye_config": LaunchConfiguration("handeye_config"),
                    "start_ordinary_grasp": LaunchConfiguration("start_ordinary_grasp"),
                    "ordinary_grasp_root": LaunchConfiguration("ordinary_grasp_root"),
                    "ordinary_depth_quality_enabled": LaunchConfiguration(
                        "ordinary_depth_quality_enabled"
                    ),
                }.items(),
            )
        ]
    )
