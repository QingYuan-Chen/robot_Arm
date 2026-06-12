from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from rebotarm_vision.handeye_config import load_handeye_config


def generate_launch_description():
    ordinary_depth_quality_enabled = LaunchConfiguration("ordinary_depth_quality_enabled")
    handeye = load_handeye_config("/home/u24/robotarm_ros2/src/rebotarm_vision/config/handeye.yaml")
    environment_setup = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/u24/robotarm_ros2/install/setup.bash && "
        "source /home/u24/venvs/rebotarm_vision/bin/activate && "
        "export QT_QPA_PLATFORM=xcb && "
        "export QT_QPA_FONTDIR=/usr/share/fonts/truetype/dejavu && "
        "export PYTHONPATH="
        "/home/u24/venvs/rebotarm_vision/lib/python3.12/site-packages:"
        "/home/u24/rebot_grasp_vendor/pyorbbecsdk:"
        "/home/u24/rebot_grasp:"
        "/home/u24/robotarm_ros2/src/rebotarm_vision:${PYTHONPATH} && "
    )
    vision_command = (
        environment_setup
        +
        "exec python3 -m rebotarm_vision.vision_node "
        "--ros-args -r __node:=rebotarm_vision_node "
        "--params-file /home/u24/robotarm_ros2/src/rebotarm_vision/config/camera.yaml"
    )
    ordinary_grasp_command = (
        environment_setup
        +
        "exec python3 -m rebotarm_vision.ordinary_grasp_node "
        "--ros-args -r __node:=rebotarm_ordinary_grasp_node "
        "-p ordinary_grasp.candidates_topic:=/grasp/candidates "
        "--params-file /home/u24/robotarm_ros2/src/rebotarm_vision/config/camera.yaml"
    )
    grasp_tcp_frame_command = (
        environment_setup
        +
        "exec python3 -m rebotarm_vision.grasp_tcp_frame_node "
        "--ros-args -r __node:=rebotarm_grasp_tcp_frame "
        "--params-file /home/u24/robotarm_ros2/src/rebotarm_vision/config/camera.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="rebotarm_handeye_static_tf",
                output="screen",
                arguments=handeye.as_static_transform_arguments(),
            ),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            ExecuteProcess(
                cmd=["bash", "-lc", vision_command],
                name="rebotarm_vision_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    [
                        ordinary_grasp_command,
                        " -p depth_quality.override_enabled:=true",
                        " -p depth_quality.override_value:=",
                        ordinary_depth_quality_enabled,
                    ],
                ],
                name="rebotarm_ordinary_grasp_node",
                output="screen",
            ),
            ExecuteProcess(
                cmd=["bash", "-lc", grasp_tcp_frame_command],
                name="rebotarm_grasp_tcp_frame",
                output="screen",
            ),
        ]
    )
