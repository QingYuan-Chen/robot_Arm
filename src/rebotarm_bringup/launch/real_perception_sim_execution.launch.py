#真实感知 + MuJoCo 仿真执行（混合链路）启动文件。
#
#用途
#    用真实相机与真实感知（Ubuntu 原生视觉输入、GraspNet 抓取候选）驱动MuJoCo 物理仿真中的机械臂执行抓取，在没有真实机械臂的条件下端到端验证整条视觉抓取链路。真机控制器绝不会被本文件拉起："use_hardware" 被固定传成"false"。
#
#节点组合
#    1. MuJoCo 仿真后端节点：headless 运行，占用独立命名空间（"sim_arm_namespace"，默认 rebotarm_sim），参数取自仿真包安装目录下的 mujoco_sim.yaml；
#    2. 通过 IncludeLaunchDescription 引入「视觉抓取系统」启动文件，由它再拉起相机、检测、GraspNet、候选 IK 筛选、抓取执行器与可选 RViz。
#
#真实/仿真后端选择逻辑
#    整条链路包在同一个进程组（GroupAction）里，统一设置 "use_sim_time" 并显式覆盖被包含启动文件的默认值，使"谁拥有该命名空间的执行权"没有歧义：
#      - use_hardware="false"：不启动真机硬件控制器；
#      - start_sim_trajectory_controller="false"：不启动那个"仅 RViz 的运动学动作服务"，因为此处的执行权已经交给 MuJoCo 节点；
#      - execution_mode="execute"：允许真正下发运动（只不过下发给仿真的机械臂）；
#      - start_visual_ready="false"：仿真侧初始位形已由 "initial_joint_positions"给定，不再额外触发视觉就绪摆位动作。
#
#安全默认值
#    MuJoCo 节点以 "headless=True" 启动，不弹图形界面；仿真命名空间与真机默认命名空间隔离，便于两条链路并存而互不干扰。

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml


def _mujoco_parameters() -> dict:
    """读取仿真包安装目录下 mujoco_sim.yaml 中本节点使用的 ros__parameters。

    返回的是浅拷贝，调用方可以安全地覆盖少数键而不影响解析结果本身。节点参数段缺失
    时返回空字典（走节点默认值）；参数段存在但类型不是字典（YAML 层级写错）时抛
    RuntimeError，避免把错误配置静默带进仿真。
    """
    config_path = (
        Path(get_package_share_directory("rebotarm_simulation"))
        / "config"
        / "mujoco_sim.yaml"
    )
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    node_payload = payload.get("rebotarm_mujoco_node", {})
    parameters = node_payload.get("ros__parameters", {}) if isinstance(node_payload, dict) else {}
    if not isinstance(parameters, dict):
        raise RuntimeError(f"invalid MuJoCo parameters: {config_path}")
    return dict(parameters)


def generate_launch_description():
    """构造混合链路启动描述：MuJoCo 仿真后端 + 真实感知的视觉抓取系统。"""
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    sim_arm_namespace = LaunchConfiguration("sim_arm_namespace")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_joint_positions = LaunchConfiguration("initial_joint_positions")
    mujoco_python_executable = LaunchConfiguration("mujoco_python_executable")
    mujoco_parameters = _mujoco_parameters()
    # 这四个键以启动参数为准覆盖 YAML 同名值：backend 固定为 MuJoCo 物理后端；headless
    # 强制无窗口运行；arm_namespace 与 initial_joint_positions 必须与上层视觉链路一致，
    # 否则关节状态、TF 与执行动作会对不上命名空间和初始位形。
    mujoco_parameters.update(
        {
            "backend": "mujoco",
            "headless": True,
            "arm_namespace": sim_arm_namespace,
            "initial_joint_positions": initial_joint_positions,
        }
    )

    # MuJoCo 需要独立的 Python 环境：默认使用工作目录 third_party 下的虚拟环境解释器；
    # 可用环境变量 REBOTARM_MUJOCO_PYTHON 覆盖，最终还可由命令行参数覆盖。
    default_mujoco_python = PathJoinSubstitution(
        [EnvironmentVariable("PWD", default_value="."), "third_party", "rebotarm_mujoco_venv", "bin", "python"]
    )
    return LaunchDescription(
        [
            # sim_arm_namespace：仿真机械臂的命名空间。默认 rebotarm_sim，与真机默认的
            #   rebotarm 隔离，避免两条链路同时运行时话题/动作重名。
            DeclareLaunchArgument("sim_arm_namespace", default_value="rebotarm_sim"),
            # use_local_rviz：是否启动本链路自带的轻量 RViz（只读可视化，不含 MoveIt 面板）。
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            # use_sim_time：是否使用 /clock 仿真时间。MuJoCo 后端发布仿真时钟，此项必须为
            #   true，否则 TF、点云与候选的时间戳会与实际仿真时刻错位。
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # initial_joint_positions：仿真开始时 6 个手臂关节角（rad，顺序 joint1..joint6）。
            #   默认 joint1=-π/2，使机械臂朝向视觉工作区；改错可能让首个抓取目标不可达。
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            # mujoco_python_executable：MuJoCo 节点使用的 Python 解释器（含 MuJoCo 依赖）。
            #   优先级：命令行参数 > 环境变量 REBOTARM_MUJOCO_PYTHON > 上面的虚拟环境默认值。
            DeclareLaunchArgument(
                "mujoco_python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_MUJOCO_PYTHON",
                    default_value=default_mujoco_python,
                ),
            ),
            # vision_python_executable：视觉节点使用的解释器；默认系统 python3，
            #   可用 REBOTARM_VISION_PYTHON 或命令行覆盖。
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # graspnet_python_executable：GraspNet 推理节点使用的解释器（通常有独立环境）；
            #   默认系统 python3，可用 GRASPNET_PYTHON 或命令行覆盖。
            DeclareLaunchArgument(
                "graspnet_python_executable",
                default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
            ),
            # 进程组：先统一 use_sim_time，再启动 MuJoCo 节点并包含视觉抓取系统启动文件，
            # 保证被包含的节点都继承仿真时间与同一命名空间。
            GroupAction(
                [
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    # 仿真后端节点：必须由独立的 MuJoCo 解释器运行，故用 prefix 指定
                    # 可执行文件之前的解释器；参数为上面读取并覆盖过的 YAML 参数。
                    Node(
                        package="rebotarm_simulation",
                        executable="rebotarm_mujoco_node",
                        name="rebotarm_mujoco_node",
                        output="screen",
                        prefix=mujoco_python_executable,
                        parameters=[mujoco_parameters],
                    ),
                    # 引入「视觉抓取系统」启动文件，并把仿真后端相关的选择全部显式钉死。
                    # 这里每个 launch_arguments 键都会覆盖被包含文件的同名默认值；凡是与
                    # 被包含文件默认值不同的取值，都在注释里说明了原因。
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [bringup_share, "launch", "visual_grasp_system.launch.py"]
                            )
                        ),
                        launch_arguments={
                            # 与 MuJoCo 节点共用同一个仿真命名空间。
                            "arm_namespace": sim_arm_namespace,
                            # 透传两个解释器选择，保证视觉/GraspNet 用各自独立的环境。
                            "vision_python_executable": LaunchConfiguration("vision_python_executable"),
                            "graspnet_python_executable": LaunchConfiguration("graspnet_python_executable"),
                            # 执行后端选择：不启真机控制器；也不启动"仅 RViz 的运动学动作
                            # 服务"（默认 true），因为执行权归 MuJoCo 节点。
                            "use_hardware": "false",
                            "start_sim_trajectory_controller": "false",
                            "use_local_rviz": use_local_rviz,
                            # 允许真正下发运动——但目标是仿真机械臂；默认 plan_only。
                            "execution_mode": "execute",
                            # 感知开关：启动视觉与 GraspNet，使用 Ubuntu 原生配置。
                            "start_vision": "true",
                            "vision_profile": "ubuntu_native",
                            # 仿真侧初始位形由 initial_joint_positions 决定，不做就绪摆位
                            # （默认 true 会额外触发一次观察位姿移动）。
                            "start_visual_ready": "false",
                            "ordinary_depth_quality_enabled": "true",
                            "start_graspnet_baseline": "true",
                            "graspnet_config": PathJoinSubstitution(
                                [vision_share, "config", "graspnet_ubuntu.yaml"]
                            ),
                            # 候选来源与筛选：本地 GraspNet 候选直接进 IK 筛选，保留候选
                            # 自身姿态，每帧最多 20 个候选并启用工作空间门控。
                            "candidate_ik_input_topic": "/grasp/graspnet_candidates",
                            "start_candidate_ik_filter": "true",
                            "candidate_pose_policy": "preserve_candidate_pose",
                            "candidate_max_candidates_per_frame": "20",
                            "candidate_workspace_gate_enabled": "true",
                            # 关闭 joint6 大幅翻转的硬性否决（被包含文件默认 1.5708 rad，即
                            # 末轴转角超过约 90° 直接否掉该候选）；0.0 表示不做这项检查，
                            # 让更多候选进入后续排序。
                            "candidate_max_joint6_delta_rad": "0.0",
                            # 候选筛选读取 MuJoCo 发布的视觉关节状态，而不是真机反馈。
                            "candidate_joint_state_topic": [
                                "/",
                                sim_arm_namespace,
                                "/visual_joint_states",
                            ],
                            # 当前仿真模型本身已经把实测的 ee_site=-0.04 m 表达在模型里，
                            # 因此这里不能再叠加一次 TCP 偏移，否则会重复平移 4 cm。
                            "tcp_offset_xyz": "[0.0, 0.0, 0.0]",
                            # 执行器输入与轨迹预检。
                            "executor_input_topic": "/grasp/filtered_plan",
                            "trajectory_precheck_enabled": "true",
                            # 本机实测 GraspNet + IK 延迟约 1.0~1.4 s。真机链路默认仍为 1.0 s，
                            # 只在这条混合链路上放宽计划有效期，避免刚算出的计划被判过期丢弃。
                            "max_plan_age_sec": "3.0",
                            # 夹爪策略：接近前先张开，宽度/力矩按候选自动推导；但**不执行**
                            # 夹爪抓取动作（仿真不放宽"抓取保持"语义），也关闭抓取验证。
                            "open_before_approach": "true",
                            "auto_gripper_width": "true",
                            "auto_gripper_effort": "true",
                            "gripper_grasp_enabled": "false",
                            "grasp_verification_enabled": "false",
                            "grasp_verification_require_contact": "false",
                            # 安全撤退：抬起至少 0.12 m 再水平退出，避免拖拽仿真物块。
                            "safe_retreat_enabled": "true",
                            "safe_retreat_min_lift_z_m": "0.12",
                            "lift_z_m": "0.04",
                            # 规划时间 8 s、尝试 5 次，与真机视觉链路的默认值一致，这里显式
                            # 写出以固定仿真链路的规划预算。
                            "moveit_planning_time": "8.0",
                            "moveit_num_planning_attempts": "5",
                            "base_pregrasp_distance_m": "0.06",
                            # 抓取后不回安全零位，便于连续观察/调试仿真结果。
                            "safe_home_after_grasp": "false",
                        }.items(),
                    ),
                ]
            ),
        ]
    )
