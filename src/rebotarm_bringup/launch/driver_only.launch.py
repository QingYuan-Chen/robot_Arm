"""兼容入口：仅启动真实机械臂控制器。

控制器定义和全部硬件默认参数由 ``hardware_controller.launch.py`` 统一维护。本文件保留
原有命令名，供现有脚本和操作习惯继续使用，不增加任何节点或行为。
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "launch", "hardware_controller.launch.py"]
    )
    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(hardware_launch))]
    )
