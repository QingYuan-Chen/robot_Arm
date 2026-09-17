#仅启动硬件控制器的最小启动文件（真机驱动专用）。也是类似bringup的子启动文件（已测）
#
#原文件说明：该启动文件仅启动机械臂控制器节点，适用于需要单独控制机械臂的场景。
#
#启动用途：只拉起机械臂硬件控制器这一个节点，不启动机器人状态发布器、关节状态发布器、RViz 或任何上层操作接口。
#适合以下场景：由外部系统自行提供 TF 与可视化；用命令行直接调服务/动作调试电机；或在别的启动组合已经提供状态发布链路时避免重复占用。
#
#节点组合：只有 reBotArmController 一个节点。因此本文件不会与其它入口争抢/robot_description 与 /joint_states 这类全局资源，但操作者也就看不到 TF 与模型。
#
#真实/仿真后端选择逻辑：没有分支，本文件只面向真实硬件。仿真执行后端由仿真包与对应的仿真启动组合提供；两者互斥，不要同时对同一个关节链路下发命令。
#
#参数来源：arm_config / gripper_config 默认指向启动组合包 config/ 下的 YAML（总线通道、电机 ID 与整定增益），下面的夹爪安全参数以 launch 默认值的形式给出，可以在不修改YAML 的情况下按命令行覆盖。
#
#安全默认值：cmd_arbitration="reject"（轨迹期间拒绝单关节透传指令，安全优先）；
#夹爪位置命令的力矩上限 1.0 N·m、角速度上限 1.5 rad/s、到位超时余量 1.5 s、反馈新鲜度阈值 0.15 s、抓取保持上限 30 s——这些值的统一含义是“宁可慢一点、也不长时间堵转顶住工件”，
#而控制器节点自身对同类参数的默认值更保守（速度上限 0.5 rad/s），若需要与控制器内置默认一致请显式覆盖。真机上电后仍处于失能态，必须显式调用 enable 服务才会运动。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    hardware_feedback_rate_hz = LaunchConfiguration("hardware_feedback_rate_hz")
    gripper_position_torque_cap_nm = LaunchConfiguration("gripper_position_torque_cap_nm")
    gripper_position_max_speed_rad_s = LaunchConfiguration("gripper_position_max_speed_rad_s")
    gripper_position_timeout_margin_sec = LaunchConfiguration("gripper_position_timeout_margin_sec")
    gripper_feedback_stale_timeout_sec = LaunchConfiguration("gripper_feedback_stale_timeout_sec")
    grasp_hold_timeout_sec = LaunchConfiguration("grasp_hold_timeout_sec")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")

    return LaunchDescription(
        [
           # arm_config：六关节硬件配置文件。
            DeclareLaunchArgument( 
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # gripper_config：夹爪硬件配置文件。
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # channel：总线通道覆盖值；空串表示沿用配置文件中的 channel。
            DeclareLaunchArgument("channel", default_value=""),
            # joint_state_rate（Hz）：/joint_states 发布频率，默认 100 Hz。
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # hardware_feedback_rate_hz（Hz）：反馈刷新频率上限；硬件层要求落在 [20, 100]，默认 50 Hz。调高反馈更及时，但会挤压同一条总线上的命令带宽。
            DeclareLaunchArgument("hardware_feedback_rate_hz", default_value="50.0"),
            # gripper_position_torque_cap_nm（N·m）：夹爪位置移动期间的电机侧力矩上限，硬件层限制在 [0.05, 1.5]，默认 1.0——过小会因传动阻力而堵转到位不准，过大则有夹伤风险。
            DeclareLaunchArgument("gripper_position_torque_cap_nm", default_value="1.0"),
            # gripper_position_max_speed_rad_s（rad/s）：夹爪位置命令的角速度上限，硬件层限制在 [0.05, 3.0]，默认 1.5；调大加快开合但冲击与堵转力矩更大。
            DeclareLaunchArgument("gripper_position_max_speed_rad_s", default_value="1.5"),
            # gripper_position_timeout_margin_sec（s）：夹爪到位超时的附加余量；实际超时 ≈ 行程 / 速度 + 该余量，默认 1.5 s。
            DeclareLaunchArgument("gripper_position_timeout_margin_sec", default_value="1.5"),
            # gripper_feedback_stale_timeout_sec（s）：夹爪反馈新鲜度阈值，超时即判过期并拒绝新的夹爪命令；硬件层限制在 [0.05, 2.0]，默认 0.15 s。
            DeclareLaunchArgument("gripper_feedback_stale_timeout_sec", default_value="0.15"),
            # grasp_hold_timeout_sec（s）：抓取保持的最长时间，到期自动松开，避免电机持续堵转发热；硬件层限制在 [0.1, 120.0]，默认 30 s。
            DeclareLaunchArgument("grasp_hold_timeout_sec", default_value="30.0"),
            # cmd_arbitration：轨迹运行期间收到单关节透传指令时的仲裁策略；默认 "reject" 直接拒绝（安全优先），"preempt" 会先停下轨迹再执行透传指令。
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # arm_namespace：话题/服务/动作命名空间段，去首尾斜杠后拼成 /{ns}/...。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # 硬件控制器节点：本文件唯一节点。启动后处于失能态，必须先有新鲜反馈并显式调用 enable 服务才允许运动；上面所有参数均按名转发给它。
            Node(
                package="rebotarmcontroller",
                executable="reBotArmController",
                name="reBotArmController",
                output="screen",
                parameters=[
                    {
                        "arm_config": arm_config,
                        "gripper_config": gripper_config,
                        "channel": channel,
                        "joint_state_rate": joint_state_rate,
                        "hardware_feedback_rate_hz": hardware_feedback_rate_hz,
                        "gripper_position_torque_cap_nm": gripper_position_torque_cap_nm,
                        "gripper_position_max_speed_rad_s": gripper_position_max_speed_rad_s,
                        "gripper_position_timeout_margin_sec": gripper_position_timeout_margin_sec,
                        "gripper_feedback_stale_timeout_sec": gripper_feedback_stale_timeout_sec,
                        "grasp_hold_timeout_sec": grasp_hold_timeout_sec,
                        "cmd_arbitration": cmd_arbitration,
                        "arm_namespace": arm_namespace,
                    }
                ],
            ),
        ]
    )
