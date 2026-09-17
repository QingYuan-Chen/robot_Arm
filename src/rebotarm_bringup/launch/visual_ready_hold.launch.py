"""视觉就绪位姿启动文件：真机控制器 + 一次性摆位 + 常驻位姿服务（不含规划栈）。

用途
    把机械臂摆到固定的"视觉观察位姿"。相机视角与手眼关系正是在该位姿下标定，上层视觉
    抓取链路随后在该位姿开始检测与抓取规划，因此本文件只负责"到位"，不做任何感知。

节点组合（共三个）
    1. 硬件控制器：真机唯一硬件入口，本文件无条件启动（没有 use_hardware 开关）；
    2. 一次性启动摆位节点：延迟后自动执行一次观察位姿移动，**移动流程结束即退出进程**；
    3. 常驻服务节点：不自动运动，只提供 visual_ready/move 服务，供界面或操作者在需要时
       把机械臂重新摆回观察位姿。

两个实例如何串联
    监听"启动摆位节点退出"事件，进程一退出才启动常驻服务节点。注意启动实例无论移动
    成败都会退出（成败只体现在日志里），所以常驻服务的启动不会被移动结果阻塞。

参数来源与安全默认值
    - 观察位姿参数文件来自运动包 config/visual_ready.yaml，两个实例共用同一组位姿；
    - 下面的 launch 参数会覆盖参数文件中的同名值（参数字典排在参数文件之后），例如起始
      偏差保护 max_start_delta_rad 在此默认 2.5，而 YAML 里是 1.0；
    - ``shutdown_safe_home`` 默认 ``false``：退出时只断开连接、不产生任何运动；若要在
      退出前条件性回安全位，必须显式打开，且控制器还会额外做一次状态安全检查。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造视觉就绪启动描述：控制器 + 自动摆位进程 + 常驻服务进程。"""
    bringup_share = FindPackageShare("rebotarm_bringup")

    arm_config = LaunchConfiguration("arm_config")
    gripper_config = LaunchConfiguration("gripper_config")
    channel = LaunchConfiguration("channel")
    shutdown_safe_home = LaunchConfiguration("shutdown_safe_home")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    cmd_arbitration = LaunchConfiguration("cmd_arbitration")
    arm_namespace = LaunchConfiguration("arm_namespace")
    visual_ready_joint_positions = LaunchConfiguration("visual_ready_joint_positions")
    visual_ready_duration_sec = LaunchConfiguration("visual_ready_duration_sec")
    visual_ready_wait_timeout_sec = LaunchConfiguration("visual_ready_wait_timeout_sec")
    visual_ready_max_start_delta_rad = LaunchConfiguration("visual_ready_max_start_delta_rad")
    visual_ready_startup_delay_sec = LaunchConfiguration("visual_ready_startup_delay_sec")
    # 位姿/时长/超时/起始偏差保护都取自运动包的观察位姿参数文件，两个实例共用。
    visual_ready_params = PathJoinSubstitution([FindPackageShare("rebotarm_motion"), "config", "visual_ready.yaml"])

    # 硬件控制器：与视觉链路共用同一个命名空间，使摆位动作能落到真机上。
    controller = Node(
        package="rebotarmcontroller",
        executable="reBotArmController",
        name="reBotArmController",
        output="screen",
        parameters=[
            {
                "arm_config": arm_config,
                "gripper_config": gripper_config,
                "channel": channel,
                "shutdown_safe_home": shutdown_safe_home,
                "joint_state_rate": joint_state_rate,
                "cmd_arbitration": cmd_arbitration,
                "arm_namespace": arm_namespace,
            }
        ],
    )
    # 启动实例：auto_move_on_start=true 自动摆位，exit_after_startup_move=true 摆完即退出，
    # 供下面的事件处理器串联常驻实例。startup_delay_sec 用于等控制器与底层驱动就绪。
    visual_ready_startup = Node(
        package="rebotarm_motion",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready_startup",
        output="screen",
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": True,
                "exit_after_startup_move": True,
                "startup_delay_sec": visual_ready_startup_delay_sec,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )
    # 常驻实例：auto_move_on_start=false，只注册 visual_ready/move 服务，按需摆位。
    visual_ready_service = Node(
        package="rebotarm_motion",
        executable="rebotarm_visual_ready",
        name="rebotarm_visual_ready",
        output="screen",
        parameters=[
            visual_ready_params,
            {
                "arm_namespace": arm_namespace,
                "auto_move_on_start": False,
                "joint_positions": visual_ready_joint_positions,
                "duration_sec": visual_ready_duration_sec,
                "wait_timeout_sec": visual_ready_wait_timeout_sec,
                "max_start_delta_rad": visual_ready_max_start_delta_rad,
            }
        ],
    )

    return LaunchDescription(
        [
            # arm_config：机械臂电机配置（电机 ID/型号/增益/限速）。
            DeclareLaunchArgument(
                "arm_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "arm.yaml"]),
            ),
            # gripper_config：夹爪电机配置；就绪位姿只动 6 个手臂关节，夹爪不受影响。
            DeclareLaunchArgument(
                "gripper_config",
                default_value=PathJoinSubstitution([bringup_share, "config", "gripper.yaml"]),
            ),
            # channel：电机总线串口设备。默认 "auto"（与空串等价）表示由厂商 SDK 自动选择，
            #   适合单臂场景；多设备时建议显式指定 /dev/ttyACM* 或 can0。
            DeclareLaunchArgument("channel", default_value="auto"),
            # shutdown_safe_home：退出前是否在失能之前条件性回安全位。默认 false，避免把
            #   "节点退出"和"机械臂必须移动"隐式绑定。
            DeclareLaunchArgument("shutdown_safe_home", default_value="false"),
            # joint_state_rate：关节状态发布频率（Hz）；越高越实时，总线负载越大。
            DeclareLaunchArgument("joint_state_rate", default_value="100.0"),
            # cmd_arbitration：轨迹运行期间收到单关节透传指令时的仲裁策略；"reject" 为
            #   安全优先的默认，摆位过程中不接受透传抢占。
            DeclareLaunchArgument("cmd_arbitration", default_value="reject"),
            # arm_namespace：话题/服务/动作命名空间前缀，必须与视觉链路一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # visual_ready_joint_positions：观察位姿的 6 个手臂关节角（rad，joint1..joint6）。
            #   默认 joint1=-π/2，使机械臂朝向视觉工作区；改动会使标定的手眼关系失效，
            #   非必要不要改。
            DeclareLaunchArgument(
                "visual_ready_joint_positions",
                default_value="[-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            # visual_ready_duration_sec：摆位轨迹总时长（s），节点内部有 0.2 s 下限；
            #   调小会提高关节速度与加速度，调大更平缓。
            DeclareLaunchArgument("visual_ready_duration_sec", default_value="4.0"),
            # visual_ready_wait_timeout_sec：等待首帧关节反馈与等待动作服务上线的时间上限
            #   （s）；超时即失败返回，不自动重试（等待动作结果本身没有超时）。
            DeclareLaunchArgument("visual_ready_wait_timeout_sec", default_value="12.0"),
            # visual_ready_max_start_delta_rad：起始偏差保护（rad）。当前关节角与目标之间的
            #   单关节最大偏差超过该值就拒绝移动，避免上电位置离观察位太远时直接插值横扫；
            #   <=0 表示显式关闭保护。注意它覆盖了 YAML 中的 1.0，放宽到 2.5 更宽松。
            DeclareLaunchArgument("visual_ready_max_start_delta_rad", default_value="2.5"),
            # visual_ready_startup_delay_sec：启动摆位前的等待时间（s），0.0 表示不等待；
            #   只推迟动作开始时刻，不改变轨迹时长。
            DeclareLaunchArgument("visual_ready_startup_delay_sec", default_value="0.0"),
            controller,
            visual_ready_startup,
            # 事件串联：启动摆位进程退出后才启动常驻服务进程（见文件头部说明）。
            RegisterEventHandler(
                OnProcessExit(
                    target_action=visual_ready_startup,
                    on_exit=[visual_ready_service],
                )
            ),
        ]
    )
