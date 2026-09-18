# 真实机械臂 + MoveIt 启动文件：真机上"规划—执行"的标准入口。
#
#用途
#    一次拉起真机硬件控制器、示教录制节点，以及 MoveIt 的规划与可视化栈，用于在真实机械臂上做规划与轨迹执行。
#
#节点组合
#    1. 硬件控制器：唯一持有电机总线与执行安全的入口；
#    2. 示教录制节点：默认不自动开始录制，只等待操作者显式开始；
#    3. 包含 MoveIt 配置包提供的 demo 启动文件，由它拉起 move_group、RViz 等规划栈；
#       并传入 "use_fake_joint_states="false""，即关节状态取真机反馈而不是假发布器。
#
#真实/仿真后端选择
#    本文件是"真机"入口：控制器无条件启动，没有 use_hardware 开关。
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    """按包名 + 相对路径读取安装后的 YAML 文件内容。

    路径通过 ament 资源索引解析，因此不依赖源码树位置。本文件目前未直接调用它，
    保留为同目录启动文件共用的读取工具（调用方需自行保证文件存在）。
    """
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    """构造真机 MoveIt 启动描述：控制器 + 示教录制 + MoveIt 规划栈。"""
    bringup_share = FindPackageShare("rebotarm_bringup")
    moveit_share = FindPackageShare("rebotarm_moveit_config")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    arm_namespace = LaunchConfiguration("arm_namespace")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    teach_record_path = LaunchConfiguration("teach_record_path")
    teach_record_rate_hz = LaunchConfiguration("teach_record_rate_hz")
    common_config = LaunchConfiguration("common_config")
    teach_config = LaunchConfiguration("teach_config")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    frame_id = LaunchConfiguration("frame_id")
    ee_frame_id = LaunchConfiguration("ee_frame_id")
    use_rviz = LaunchConfiguration("use_rviz")

    demo_launch = PathJoinSubstitution([moveit_share, "launch", "demo.launch.py"])

    return LaunchDescription(
        [
            # arm_config：机械臂电机配置（电机 ID/型号/增益/限速）。
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # gripper_config：夹爪电机配置。
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution(
                    [bringup_share, "config", "gripper.yaml"]
                ),
            ),
            # arm_namespace：话题/服务/动作命名空间前缀，必须与 MoveIt 配置一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # channel：电机总线串口设备；空串表示由上面的配置文件决定。
            DeclareLaunchArgument("channel", default_value=""),
            # joint_state_rate：关节状态发布频率（Hz）；越高越实时，总线负载越大。
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # hardware_feedback_rate_hz：硬件层反馈刷新频率上限（Hz），必须在 [20, 100]。
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            # gripper_position_torque_cap_nm：夹爪位置指令的力矩上限（N·m），允许范围[0.05, 1.5]；调大夹持更牢但堵转发热与夹伤风险更高。
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            # gripper_position_max_speed_rad_s：夹爪开合角速度上限（rad/s），硬件层允许范围 [0.05, 3.0]；调大更快但冲击更大。控制器内置默认是 0.5，本文件取 1.5。
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            # gripper_position_timeout_margin_sec：夹爪到位超时余量（s），实际超时 = 行程/速度 + 该余量；允许范围 [0.1, 10.0]。
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            # gripper_feedback_stale_timeout_sec：夹爪反馈新鲜度阈值（s），超过即判反馈过期并拒绝新的夹爪命令；允许范围 [0.05, 2.0]。
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            # teach_record_path：示教录制输出文件（JSONL）。
            DeclareLaunchArgument("teach_record_path", default_value="teleop_records/teach_record.jsonl"),
            # teach_record_rate_hz：示教采样率（Hz），与录制/回放预处理采样率一致。
            DeclareLaunchArgument("teach_record_rate_hz", default_value="150.0"),
            # common_config / teach_config：示教录制节点按消费者分别加载公共和示教参数。
            DeclareLaunchArgument(
                "common_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "operator_common.yaml"]),
            ),
            DeclareLaunchArgument(
                "teach_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "teach_control.yaml"]),
            ),
            # cmd_arbitration：轨迹运行期间收到单关节透传指令时的仲裁策略。"reject"=直接拒绝（默认，安全优先）；"preempt"=先停下轨迹再执行透传指令。
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # frame_id / ee_frame_id：对外位姿的参考坐标系与末端坐标系名称，需与 URDF 一致。
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("ee_frame_id", default_value="end_link"),
            # use_rviz：是否由 MoveIt 的 demo 启动文件打开 RViz。
            DeclareLaunchArgument("use_rviz", default_value="true"),
            # 真实硬件由唯一底层片段启动；本文件只叠加示教与 MoveIt。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, "launch", "hardware_controller.launch.py"]
                    )
                ),
                launch_arguments={
                    "arm_config": arm_config,
                    "gripper_config": gripper_config,
                    "channel": channel,
                    "joint_state_rate": joint_state_rate,
                    "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                    "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                    "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                    "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                    "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
                    "cmd_arbitration": cmd_arbitration,
                    "arm_namespace": arm_namespace,
                    "frame_id": frame_id,
                    "ee_frame_id": ee_frame_id,
                }.items(),
            ),
            # 示教录制节点：默认不自动开始录制，也不绑定键盘退出键。
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                parameters=[
                    common_config,
                    teach_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": teach_record_path,
                        "sample_rate_hz": teach_record_rate_hz,
                        "start_on_launch": False,
                        "keyboard_quit_enabled": False,
                    },
                ],
            ),
            # MoveIt 规划/可视化栈。use_fake_joint_states 固定为 "false"：关节状态必须来自真机控制器，否则 RViz 与规划器看到的是假姿态，规划结果没有意义。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch),
                launch_arguments={
                    "use_rviz": use_rviz,
                    "arm_namespace": arm_namespace,
                    "use_fake_joint_states": "false",
                }.items(),
            ),
        ]
    )
