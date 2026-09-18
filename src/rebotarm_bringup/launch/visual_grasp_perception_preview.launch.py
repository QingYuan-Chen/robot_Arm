#视觉抓取感知链路的“只读预览”启动文件：不起控制器、不做规划、不使能硬件。
#
#用途
#把“相机 → YOLO 检测 → GraspNet 候选 → 逆解过滤 → 可视化标记”这条感知与候选链路单独拉起来，配合本地 RViz 看抓取候选与夹爪位姿。用于装机与标定后的只读验收：确认相机、深度、检测、候选与坐标系朝向是否合理，不代表抓取一定可执行。
#
#启动内容（节点组合）
#1. 主视觉链路：以子启动方式引入视觉包的视觉启动文件（start_vision，默认 true），参数透传相机配置、手眼配置、YOLO 权重与普通抓取开关；
#2. 抓取候选推理节点：以 graspnet_python_executable 指定的独立解释器前缀运行，模型路径由环境变量或工作区相对路径给出（start_graspnet_baseline，默认 true）；
#3. 候选逆解过滤节点：先订阅候选话题做逆解与（可选）碰撞检查，再发布过滤后的候选与抓取计划（start_candidate_ik_filter，默认 true）；该节点只调用逆解与状态有效性服务，不产生运动；
#4. 可视化标记节点：把抓取计划画成标记数组，供 RViz 查看（start_visual_grasp_markers，默认 true）；
#5. 本地 RViz（use_local_rviz，默认 true），加载本包的抓取预览配置文件。
#
#真实/仿真后端选择逻辑
#本文件刻意不启动任何执行后端：既不启动真实硬件控制器，也不启动仿真的轨迹控制器、规划节点或任何执行入口。
#唯一的"后端相关性"来自两个坐标/话题约定：候选过滤节点的目标坐标系固定为"base_link"，关节状态来自 "candidate_joint_state_topic" 指向的话题（默认/rebotarm/visual_joint_states，由外部真机或仿真侧提供）。
#因此本预览在真机与仿真下都能用，但都必须保证该话题有有效关节反馈，否则逆解会因缺少种子状态而失败。
#
#参数来源与优先级
#- 参数文件：抓取姿态策略、台面安全两份配置加载到逆解过滤节点，GraspNet 配置单独加载到推理节点；参数文件里的键若在下面的节点参数字典中再次出现，以本文件的取值为准；
#- 本文件对参数的显式设定中，几处安全相关的关键值分别是"candidate_workspace_gate_enabled=true"（工作空间盒闸门，比参数文件里的 false 更严格）、"candidate_min_confidence=0.4"（过滤低置信度候选）、
#  ``candidate_max_joint6_delta_rad=0.0``（不允许候选改变末轴角度）、
#  ``collision_check_enabled=false``（对齐 ``candidate_collision_check_enabled``；本预览不依赖规划场景服务，只做逆解可行性判断）；
#- 话题名、坐标系名与参数键都是对外接口，改动会同时影响本文件与下游消费方，禁止随手改名。
#
#安全默认值
#----------
#- 预览链路不产生运动，默认全开也只是“看图”；要真正执行抓取必须使用带执行门的启动入口；
#- ``start_ordinary_grasp`` 默认 false：普通抓取通道（YOLO + 深度）只在显式指定 root 时才启用；
#- 预览不启动硬件，故没有“使能/失能”状态；真机测试前仍需按项目规则人工确认现场安全。
#
#说明：本文件不启动任何执行后端，也没有实现电机控制、运动规划或感知算法，这些分别属于控制器、运动、视觉包与标定包的职责。

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _workspace_path(environment_name: str, relative_path: str) -> str:
    """解析模型/权重路径：优先取环境变量，其次在源码所在目录树上按相对路径查找。

    参数：
        environment_name: 保存绝对路径的环境变量名（留空或空白视为未设置）。
        relative_path: 相对工作区根目录的路径，例如模型目录或权重文件。

    返回：
        命中则返回路径字符串，未命中返回空串——空串会让下游节点以“模型未配置”的正常
        失败路径处理（发布空候选），而不是拼出一个不存在的路径。
    """
    configured = os.environ.get(environment_name, "").strip()
    if configured:
        return configured
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidate = parent / relative_path
        if candidate.exists():
            return str(candidate)
    return ""


def generate_launch_description():
    """组装本预览链路的启动描述：声明全部启动参数，再按条件启动 4 个节点与本地 RViz。

    参数与节点组合见模块 docstring；``generate_launch_description`` 在解析阶段执行一次，
    因此这里只做订阅替换（如 ``PathJoinSubstitution``）与参数声明，具体路径在启动时才展开。
    """
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")

    # 以下三份参数文件按顺序加载到逆解过滤节点：策略/台面安全在后、GraspNet 配置单独给推理节点；
    # 节点参数字典里的显式项会覆盖文件中的同名键（例如工作空间闸门与置信度下限）
    grasp_pose_policy_params = PathJoinSubstitution([vision_share, "config", "grasp_pose_policy.yaml"])
    table_safety_params = PathJoinSubstitution([vision_share, "config", "table_safety.yaml"])
    graspnet_ubuntu_params = PathJoinSubstitution([vision_share, "config", "graspnet_ubuntu.yaml"])

    use_local_rviz = LaunchConfiguration("use_local_rviz")
    ordinary_depth_quality_enabled = LaunchConfiguration("ordinary_depth_quality_enabled")
    start_vision = LaunchConfiguration("start_vision")
    vision_camera_config = LaunchConfiguration("vision_camera_config")
    vision_handeye_config = LaunchConfiguration("vision_handeye_config")
    vision_yolo_model_path = LaunchConfiguration("vision_yolo_model_path")
    start_ordinary_grasp = LaunchConfiguration("start_ordinary_grasp")
    ordinary_grasp_root = LaunchConfiguration("ordinary_grasp_root")
    start_graspnet_baseline = LaunchConfiguration("start_graspnet_baseline")
    start_candidate_ik_filter = LaunchConfiguration("start_candidate_ik_filter")
    start_visual_grasp_markers = LaunchConfiguration("start_visual_grasp_markers")

    # GraspNet 推理输出与逆解过滤输入的话题（默认 /grasp/graspnet_candidates），两者必须同名衔接
    graspnet_candidates_topic = LaunchConfiguration("graspnet_candidates_topic")
    graspnet_python_executable = LaunchConfiguration("graspnet_python_executable")
    graspnet_model_root = LaunchConfiguration("graspnet_model_root")
    graspnet_checkpoint_path = LaunchConfiguration("graspnet_checkpoint_path")
    graspnet_device = LaunchConfiguration("graspnet_device")
    graspnet_backend_module = LaunchConfiguration("graspnet_backend_module")
    graspnet_max_grasps = LaunchConfiguration("graspnet_max_grasps")
    graspnet_max_points = LaunchConfiguration("graspnet_max_points")

    filtered_candidates_topic = LaunchConfiguration("filtered_candidates_topic")
    filtered_plan_topic = LaunchConfiguration("filtered_plan_topic")
    executor_input_topic = LaunchConfiguration("executor_input_topic")
    # 逆解种子与关节代价所需的关节状态来源；默认话题由真机或仿真侧提供，缺失即整条链路无法求解
    candidate_joint_state_topic = LaunchConfiguration("candidate_joint_state_topic")
    candidate_filter_service_timeout_sec = LaunchConfiguration("candidate_filter_service_timeout_sec")
    candidate_collision_check_enabled = LaunchConfiguration("candidate_collision_check_enabled")
    candidate_collision_check_service = LaunchConfiguration("candidate_collision_check_service")
    candidate_collision_group_name = LaunchConfiguration("candidate_collision_group_name")
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    # 固定抓取姿态（四元数 x,y,z,w）与基座接近轴：本站安装把上游 +X 工作区绕 Z 旋转 -90° 到 -Y，
    # 两者是同一朝向的两种表达，必须同时改才自洽
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    base_approach_axis_xyz = LaunchConfiguration("base_approach_axis_xyz")
    base_pregrasp_distance_m = LaunchConfiguration("base_pregrasp_distance_m")
    candidate_orientation_yaw_offsets_rad = LaunchConfiguration("candidate_orientation_yaw_offsets_rad")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    candidate_max_candidates_per_frame = LaunchConfiguration("candidate_max_candidates_per_frame")
    candidate_min_confidence = LaunchConfiguration("candidate_min_confidence")
    candidate_min_jaw_width_m = LaunchConfiguration("candidate_min_jaw_width_m")
    candidate_max_jaw_width_m = LaunchConfiguration("candidate_max_jaw_width_m")
    candidate_min_grasp_z_m = LaunchConfiguration("candidate_min_grasp_z_m")
    candidate_safe_lift_min_z_m = LaunchConfiguration("candidate_safe_lift_min_z_m")
    candidate_workspace_gate_enabled = LaunchConfiguration("candidate_workspace_gate_enabled")
    candidate_workspace_min_xyz = LaunchConfiguration("candidate_workspace_min_xyz")
    candidate_workspace_max_xyz = LaunchConfiguration("candidate_workspace_max_xyz")
    candidate_max_grasp_to_object_center_m = LaunchConfiguration("candidate_max_grasp_to_object_center_m")
    candidate_score_joint_distance_weight = LaunchConfiguration("candidate_score_joint_distance_weight")
    candidate_score_joint6_weight = LaunchConfiguration("candidate_score_joint6_weight")
    candidate_max_joint6_delta_rad = LaunchConfiguration("candidate_max_joint6_delta_rad")
    candidate_joint6_symmetry_enabled = LaunchConfiguration("candidate_joint6_symmetry_enabled")
    candidate_joint6_symmetry_angle_rad = LaunchConfiguration("candidate_joint6_symmetry_angle_rad")

    # 末端连杆到抓取 TCP 的固定偏移（单位 m，沿末端 X 轴 -4 cm）；标记与逆解共用同一偏移，
    # 改这里会同时移动画出来的 TCP 与真正求解的目标点
    tcp_offset_xyz = LaunchConfiguration("tcp_offset_xyz")
    target_base_offset_xyz = LaunchConfiguration("target_base_offset_xyz")
    base_z_offset_m = LaunchConfiguration("base_z_offset_m")
    grasp_base_z_offset_m = LaunchConfiguration("grasp_base_z_offset_m")
    lift_z_m = LaunchConfiguration("lift_z_m")
    gripper_open_axis_local_xyz = LaunchConfiguration("gripper_open_axis_local_xyz")
    show_tcp_markers = LaunchConfiguration("show_tcp_markers")
    show_approach_arrow = LaunchConfiguration("show_approach_arrow")
    show_gripper_open_axis = LaunchConfiguration("show_gripper_open_axis")

    return LaunchDescription(
        [
            # ---- 界面与主视觉链路开关 ----
            DeclareLaunchArgument("use_local_rviz", default_value="true"),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument(
                "vision_camera_config",
                default_value=PathJoinSubstitution([vision_share, "config", "camera_ubuntu.yaml"]),
            ),
            DeclareLaunchArgument(
                "vision_handeye_config",
                default_value=PathJoinSubstitution([vision_share, "config", "handeye.yaml"]),
            ),
            DeclareLaunchArgument(
                "vision_yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                        vision_share,
                        "models",
                        "yolo26s-seg.pt",
                    ]
                ),
            ),
            # ---- 可选：普通抓取（YOLO + 深度）通道，默认关闭，只在显式给出工作区根目录时启用 ----
            DeclareLaunchArgument("start_ordinary_grasp", default_value="false"),
            DeclareLaunchArgument("ordinary_grasp_root", default_value=""),
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            # ---- 感知与候选链路的三个节点开关，默认全开（只读预览） ----
            DeclareLaunchArgument("start_graspnet_baseline", default_value="true"),
            DeclareLaunchArgument("start_candidate_ik_filter", default_value="true"),
            DeclareLaunchArgument("start_visual_grasp_markers", default_value="true"),
            # ---- GraspNet 推理：话题、解释器、模型路径与设备 ----
            DeclareLaunchArgument("graspnet_candidates_topic", default_value="/grasp/graspnet_candidates"),
            DeclareLaunchArgument(
                "graspnet_python_executable",
                default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
            ),
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            DeclareLaunchArgument(
                "graspnet_model_root",
                default_value=_workspace_path(
                    "GRASPNET_MODEL_ROOT", ".local-models/graspnet-baseline"
                ),
            ),
            DeclareLaunchArgument(
                "graspnet_checkpoint_path",
                default_value=_workspace_path(
                    "GRASPNET_CHECKPOINT_PATH",
                    ".local-models/checkpoints/checkpoint-rs.tar",
                ),
            ),
            DeclareLaunchArgument("graspnet_device", default_value="cuda:0"),
            DeclareLaunchArgument("graspnet_backend_module", default_value="graspnet_baseline_inference"),
            DeclareLaunchArgument("graspnet_max_grasps", default_value="10"),
            DeclareLaunchArgument("graspnet_max_points", default_value="20000"),
            # ---- 候选过滤：输出话题与执行器输入话题；默认执行器输入为过滤后的抓取计划 ----
            DeclareLaunchArgument("filtered_candidates_topic", default_value="/grasp/filtered_candidates"),
            DeclareLaunchArgument("filtered_plan_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("executor_input_topic", default_value="/grasp/filtered_plan"),
            DeclareLaunchArgument("candidate_joint_state_topic", default_value="/rebotarm/visual_joint_states"),
            # ---- 逆解/碰撞检查服务与超时（单位 s）；本预览默认关闭碰撞检查，只做逆解可行性 ----
            DeclareLaunchArgument("candidate_filter_service_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("candidate_collision_check_enabled", default_value="false"),
            DeclareLaunchArgument("candidate_collision_check_service", default_value="/check_state_validity"),
            DeclareLaunchArgument("candidate_collision_group_name", default_value="arm_with_gripper"),
            # ---- 候选姿态策略与姿态/位置变体；变体以列表给出，元素越多逆解调用次数越多 ----
            DeclareLaunchArgument("candidate_pose_policy", default_value="preserve_candidate_pose"),
            DeclareLaunchArgument(
                "fixed_grasp_orientation_xyzw",
                default_value="[0.0, 0.0, -0.707106781, 0.707106781]",
            ),
            DeclareLaunchArgument("base_approach_axis_xyz", default_value="[0.0, -1.0, 0.0]"),
            DeclareLaunchArgument("base_pregrasp_distance_m", default_value="0.06"),
            DeclareLaunchArgument("candidate_orientation_yaw_offsets_rad", default_value="[0.0]"),
            DeclareLaunchArgument("candidate_grasp_z_offsets_m", default_value="[0.0]"),
            # ---- 每帧候选上限与置信度下限，用于限制单帧逆解调用量 ----
            DeclareLaunchArgument("candidate_max_candidates_per_frame", default_value="20"),
            DeclareLaunchArgument("candidate_min_confidence", default_value="0.4"),
            # ---- 夹爪行程与最低抓取高度（单位 m）；超出行程或贴地的候选直接丢弃 ----
            DeclareLaunchArgument("candidate_min_jaw_width_m", default_value="0.006"),
            DeclareLaunchArgument("candidate_max_jaw_width_m", default_value="0.085"),
            DeclareLaunchArgument("candidate_min_grasp_z_m", default_value="0.0"),
            # 抬升安全高度下限（m）：本预览只声明不使用，由执行侧读取同名参数生效
            DeclareLaunchArgument("candidate_safe_lift_min_z_m", default_value="0.120"),
            # ---- 工作空间盒闸门：本预览显式打开（参数文件里是关闭），抓取点必须落在下面的盒内 ----
            DeclareLaunchArgument("candidate_workspace_gate_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_workspace_min_xyz", default_value="[-0.35, -0.64, 0.0]"),
            DeclareLaunchArgument("candidate_workspace_max_xyz", default_value="[0.35, -0.18, 0.45]"),
            # 抓取点到物体中心的最大偏差（m）：超出说明抓取点已偏离物体，判为不可信
            DeclareLaunchArgument("candidate_max_grasp_to_object_center_m", default_value="0.15"),
            # ---- 候选打分权重：关节总位移与末轴（joint6）旋转各自的代价系数，越大越偏好动作小 ----
            DeclareLaunchArgument("candidate_score_joint_distance_weight", default_value="0.15"),
            DeclareLaunchArgument("candidate_score_joint6_weight", default_value="0.35"),
            # 末轴最大允许角差（rad）：本预览取 0.0，即要求候选不改变末轴角度，避免腕部大幅翻转
            DeclareLaunchArgument("candidate_max_joint6_delta_rad", default_value="0.0"),
            DeclareLaunchArgument("candidate_joint6_symmetry_enabled", default_value="true"),
            DeclareLaunchArgument("candidate_joint6_symmetry_angle_rad", default_value="3.141592653589793"),
            # ---- TCP 偏移与目标平移补偿（单位 m），以及预览用抬升量 ----
            DeclareLaunchArgument("tcp_offset_xyz", default_value="[-0.04, 0.0, 0.0]"),
            DeclareLaunchArgument("target_base_offset_xyz", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("base_z_offset_m", default_value="0.05"),
            DeclareLaunchArgument("grasp_base_z_offset_m", default_value="0.0"),
            DeclareLaunchArgument("lift_z_m", default_value="0.04"),
            # ---- 标记显示开关：TCP 标记、接近箭头、夹爪开合轴；三者只影响 RViz 显示 ----
            DeclareLaunchArgument("gripper_open_axis_local_xyz", default_value="[0.0, 1.0, 0.0]"),
            DeclareLaunchArgument("show_tcp_markers", default_value="true"),
            DeclareLaunchArgument("show_approach_arrow", default_value="true"),
            DeclareLaunchArgument("show_gripper_open_axis", default_value="true"),
            # 子启动：视觉包的主视觉链路（相机/检测/普通抓取）。仅在 start_vision 为真时引入，
            # 参数按下面字典透传；未列出的启动参数沿用其自身默认值
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([vision_share, "launch", "vision.launch.py"])
                ),
                condition=IfCondition(start_vision),
                launch_arguments={
                    "camera_config": vision_camera_config,
                    "vision_python_executable": LaunchConfiguration("vision_python_executable"),
                    "handeye_config": vision_handeye_config,
                    "yolo_model_path": vision_yolo_model_path,
                    "start_ordinary_grasp": start_ordinary_grasp,
                    "ordinary_grasp_root": ordinary_grasp_root,
                    "ordinary_depth_quality_enabled": ordinary_depth_quality_enabled,
                }.items(),
            ),
            # GraspNet 候选推理节点：独立进程内推理，输入彩色/深度/检测，输出候选数组。
            # 用单独的解释器前缀是为了让带推理依赖的虚拟环境与系统解释器解耦
            Node(
                package="rebotarm_vision",
                executable="rebotarm_graspnet_baseline_node",
                name="rebotarm_graspnet_baseline_node",
                output="screen",
                prefix=graspnet_python_executable,
                condition=IfCondition(start_graspnet_baseline),
                parameters=[
                    graspnet_ubuntu_params,
                    {
                        "input_color_topic": "/camera/color/image_raw",
                        "input_depth_topic": "/camera/depth/image_raw",
                        "input_detections_topic": "/grasp/detections",
                        "output_candidates_topic": graspnet_candidates_topic,
                        # 输出位姿坐标系：深度光学坐标系，与深度图 frame_id 一致
                        "output_frame_id": "camera_depth_frame",
                        "model_root": graspnet_model_root,
                        "checkpoint_path": graspnet_checkpoint_path,
                        "device": graspnet_device,
                        "backend_module": graspnet_backend_module,
                        "max_grasps": graspnet_max_grasps,
                        # 夹爪最大开口：与候选过滤共用同一个上限，保证两侧判据一致
                        "max_jaw_width_m": candidate_max_jaw_width_m,
                        "max_points": graspnet_max_points,
                    },
                ],
            ),
            # 候选逆解过滤节点：把候选换算到目标坐标系，逐个求逆解并（可选）做状态有效性检查，
            # 只发布过滤结果与抓取计划，绝不触发运动；关节状态来自外部反馈话题
            Node(
                package="rebotarm_vision",
                executable="rebotarm_grasp_candidate_ik_filter",
                name="rebotarm_grasp_candidate_ik_filter",
                output="screen",
                condition=IfCondition(start_candidate_ik_filter),
                parameters=[
                    grasp_pose_policy_params,
                    table_safety_params,
                    {
                        "input_topic": graspnet_candidates_topic,
                        "output_topic": filtered_candidates_topic,
                        "output_plan_topic": filtered_plan_topic,
                        "joint_state_topic": candidate_joint_state_topic,
                        # 逆解与碰撞检查超时（单位 s）；超时即判该候选不可行，宁可少给候选也不误判
                        "service_timeout_sec": candidate_filter_service_timeout_sec,
                        # 所有位姿统一换算到基座系再求解；下方服务名与分组名必须与规划侧配置一致
                        "target_frame": "base_link",
                        "moveit_ik_service": "/compute_ik",
                        "collision_check_enabled": candidate_collision_check_enabled,
                        "collision_check_service": candidate_collision_check_service,
                        "collision_group_name": candidate_collision_group_name,
                        # 逆解用的规划组（只含手臂关节，不含夹爪）与逆解参考连杆
                        "moveit_group_name": "arm",
                        "ee_frame_id": "end_link",
                        "pose_policy": candidate_pose_policy,
                        "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                        "base_approach_axis_xyz": base_approach_axis_xyz,
                        "base_pregrasp_distance_m": base_pregrasp_distance_m,
                        "orientation_yaw_offsets_rad": candidate_orientation_yaw_offsets_rad,
                        "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                        "max_candidates_per_frame": candidate_max_candidates_per_frame,
                        "candidate_min_confidence": candidate_min_confidence,
                        "lift_z_m": lift_z_m,
                        "candidate_min_jaw_width_m": candidate_min_jaw_width_m,
                        "candidate_max_jaw_width_m": candidate_max_jaw_width_m,
                        "candidate_min_grasp_z_m": candidate_min_grasp_z_m,
                        "candidate_safe_lift_min_z_m": candidate_safe_lift_min_z_m,
                        "candidate_workspace_gate_enabled": candidate_workspace_gate_enabled,
                        "candidate_workspace_min_xyz": candidate_workspace_min_xyz,
                        "candidate_workspace_max_xyz": candidate_workspace_max_xyz,
                        "candidate_max_grasp_to_object_center_m": candidate_max_grasp_to_object_center_m,
                        "candidate_score_joint_distance_weight": candidate_score_joint_distance_weight,
                        "candidate_score_joint6_weight": candidate_score_joint6_weight,
                        "candidate_max_joint6_delta_rad": candidate_max_joint6_delta_rad,
                        "candidate_joint6_symmetry_enabled": candidate_joint6_symmetry_enabled,
                        "candidate_joint6_symmetry_angle_rad": candidate_joint6_symmetry_angle_rad,
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "target_base_offset_xyz": target_base_offset_xyz,
                        # 接近点额外抬升与抓取点高度偏移（单位 m）：保证从物体上方进入而不是平推
                        "pregrasp_base_z_offset_m": base_z_offset_m,
                        "grasp_base_z_offset_m": grasp_base_z_offset_m,
                    },
                ],
            ),
            # 可视化标记节点：把抓取计划画成标记数组（物体、接近点、抓取点、TCP、接近轴、开合轴）。
            # object_min_* 是标记的显示下限，只影响 RViz 观感，不改变真实抓取尺寸
            Node(
                package="rebotarm_vision",
                executable="rebotarm_visual_grasp_markers",
                name="rebotarm_visual_grasp_markers",
                output="screen",
                condition=IfCondition(start_visual_grasp_markers),
                parameters=[
                    grasp_pose_policy_params,
                    {
                        "input_topic": executor_input_topic,
                        # 输入为执行器输入话题（默认为过滤后的抓取计划），输出标记数组供 RViz 订阅
                        "output_topic": "/grasp/visual_markers",
                        "target_frame": "base_link",
                        "object_min_diameter_m": 0.06,
                        "object_min_height_m": 0.12,
                        "upright_object_marker": True,
                        "tcp_offset_xyz": tcp_offset_xyz,
                        "gripper_open_axis_local_xyz": gripper_open_axis_local_xyz,
                        "show_tcp_markers": show_tcp_markers,
                        "show_approach_arrow": show_approach_arrow,
                        "show_gripper_open_axis": show_gripper_open_axis,
                    },
                ],
            ),
            # 本地 RViz：加载本包的抓取预览配置（仅机器人模型、TF、标记数组，不含运动规划面板）
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_visual_grasp_preview",
                output="screen",
                condition=IfCondition(use_local_rviz),
                arguments=["-d", PathJoinSubstitution([bringup_share, "rviz", "visual_grasp.rviz"])],
            ),
        ]
    )
