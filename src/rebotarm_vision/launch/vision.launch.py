from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rebotarm_vision.handeye_config import load_handeye_config


def _launch_setup(context):
    camera_config = LaunchConfiguration("camera_config").perform(context)
    handeye_config = LaunchConfiguration("handeye_config").perform(context)
    ordinary_grasp_root = LaunchConfiguration("ordinary_grasp_root").perform(context)
    handeye = load_handeye_config(Path(handeye_config).expanduser())

    common_environment = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_QPA_FONTDIR": "/usr/share/fonts/truetype/dejavu",
    }

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_handeye_static_tf",
            output="screen",
            arguments=handeye.as_static_transform_arguments(),
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_vision_node",
            name="rebotarm_vision_node",
            output="screen",
            parameters=[camera_config],
            additional_env=common_environment,
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_ordinary_grasp_node",
            name="rebotarm_ordinary_grasp_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_ordinary_grasp")),
            parameters=[
                camera_config,
                {
                    "ordinary_grasp.root": ordinary_grasp_root,
                    "ordinary_grasp.candidates_topic": "/grasp/candidates",
                    "depth_quality.override_enabled": True,
                    "depth_quality.override_value": ParameterValue(
                        LaunchConfiguration("ordinary_depth_quality_enabled"),
                        value_type=bool,
                    ),
                },
            ],
            additional_env=common_environment,
        ),
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_tcp_frame",
            name="rebotarm_grasp_tcp_frame",
            output="screen",
            parameters=[camera_config],
            additional_env=common_environment,
        ),
    ]


def generate_launch_description():
    vision_share = Path(get_package_share_directory("rebotarm_vision"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_config",
                default_value=str(vision_share / "config" / "camera.yaml"),
            ),
            DeclareLaunchArgument(
                "handeye_config",
                default_value=str(vision_share / "config" / "handeye.yaml"),
            ),
            DeclareLaunchArgument("ordinary_grasp_root", default_value=""),
            DeclareLaunchArgument("start_ordinary_grasp", default_value="false"),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
