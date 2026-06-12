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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(vision_share / "launch" / "vision.launch.py")
                ),
                launch_arguments={
                    "camera_config": str(
                        vision_share / "config" / "camera_ubuntu.yaml"
                    ),
                    "yolo_model_path": str(
                        vision_share / "models" / "yolo11n-seg.pt"
                    ),
                    "yolo_device": LaunchConfiguration("yolo_device"),
                    "start_ordinary_grasp": "false",
                }.items(),
            )
        ]
    )
