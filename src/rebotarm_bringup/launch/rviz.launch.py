# 璇ュ惎鍔ㄦ枃浠朵粎鍚姩鏈哄櫒浜虹姸鎬佸彂甯冨櫒鍜孯Viz鍙鍖栵紝閫傜敤浜庨渶瑕佹煡鐪嬫満姊拌噦鐘舵€佸拰妯″瀷鐨勫満鏅€?
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    arm_namespace = LaunchConfiguration("arm_namespace")
    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    rviz_config = PathJoinSubstitution([bringup_share, "rviz", "rebotarm.rviz"])
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            Node(
                package="rebotarm_teleop",
                executable="GripperVisualJointStateNode",
                name="gripper_visual_joint_state_node",
                output="screen",
                parameters=[{"arm_namespace": arm_namespace}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", ["/", arm_namespace, "/visual_joint_states"])],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
