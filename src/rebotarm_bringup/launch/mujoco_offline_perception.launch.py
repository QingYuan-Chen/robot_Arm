#把 MuJoCo 虚拟 RGB-D 相机与本机「只规划」感知链路组合起来的离线启动文件。
#
#用途：在没有真实相机、也不驱动真实机械臂的前提下，用 MuJoCo 物理模型渲染出虚拟RGB-D 图像与真值标注，喂给本机 YOLO 与 GraspNet 候选生成，再走一遍候选 IK/碰撞过滤与夹爪几何闸门，从而离线验证整条视觉抓取链路。
#
#节点组合与数据流：
#
#1. 仿真节点（可执行文件 "rebotarm_mujoco_node"）：以 "backend=mujoco"、"headless=True" 运行，提供仿真关节状态与 "FollowJointTrajectory" 后端，并按 "virtual_camera.*" 参数发布彩色/深度图与真值检测；
#2. 离线 YOLO 节点（可执行文件 "rebotarm_offline_yolo_node"）：订阅虚拟彩图，发布检测结果，用于替代真机视觉前端；
#3. 本包内的视觉抓取系统（"visual_grasp_system.launch.py"）：只启用 GraspNet候选生成 + 候选 IK/碰撞过滤，其它视觉入口一律关闭。
#
#后端选择与安全默认值：本文件固定不启动真实硬件控制器（向下传递"use_hardware=false"），"execution_mode" 默认 "plan_only"，
#"start_visual_grasp_executor" 与 "start_motion_execution" 默认均为 "false"，即默认连仿真轨迹下发都不开启，只做规划干跑；
#"use_sim_time" 默认 "true"，保证整棵树共用仿真时钟。

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _mujoco_parameters() -> dict:
    """读取仿真包随包安装的 ``config/mujoco_sim.yaml``，取出仿真节点的 ROS 参数。

    返回 ``rebotarm_mujoco_node`` 一节下的 ``ros__parameters`` 副本（浅拷贝），供下面
    用启动参数覆盖其中的后端、命名空间与虚拟相机字段。文件缺失/结构不是字典时直接
    抛错而不静默兜底：宁可启动失败，也不让仿真节点带着未知默认值运行。
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
    """构建本启动文件的 LaunchDescription。

    先把所有会向下传递的启动参数固化为 LaunchConfiguration 对象，再对 ``mujoco_sim.yaml``
    读出的参数做定点覆盖，最后声明参数默认值并组合节点与内嵌启动文件。
    """
    bringup_share = FindPackageShare("rebotarm_bringup")
    vision_share = FindPackageShare("rebotarm_vision")
    # 仿真端命名空间：与真机命名空间（rebotarm）隔离，避免仿真话题与真机话题同名冲突
    sim_arm_namespace = LaunchConfiguration("sim_arm_namespace")
    use_local_rviz = LaunchConfiguration("use_local_rviz")
    # 全树统一使用仿真时钟：MuJoCo 的仿真时间与墙上时间不同，混用会导致 TF/轨迹过期误判
    use_sim_time = LaunchConfiguration("use_sim_time")
    # 仿真初始关节角（6 个转动关节，单位 rad），首关节 -pi/2 为视觉可达的初始姿态
    initial_joint_positions = LaunchConfiguration("initial_joint_positions")
    mujoco_python_executable = LaunchConfiguration("mujoco_python_executable")
    virtual_camera_width = LaunchConfiguration("virtual_camera_width")
    virtual_camera_height = LaunchConfiguration("virtual_camera_height")
    virtual_camera_rate_hz = LaunchConfiguration("virtual_camera_rate_hz")
    # 虚拟相机的光学坐标系名，必须与下游 GraspNet 输出坐标系一致（见 graspnet_output_frame_id）
    virtual_camera_frame_id = LaunchConfiguration("virtual_camera_frame_id")
    virtual_camera_annotation_topic = LaunchConfiguration(
        "virtual_camera_annotation_topic"
    )
    offline_yolo_enabled = LaunchConfiguration("offline_yolo_enabled")
    offline_yolo_model_path = LaunchConfiguration("offline_yolo_model_path")
    offline_yolo_device = LaunchConfiguration("offline_yolo_device")
    offline_yolo_target_classes = LaunchConfiguration("offline_yolo_target_classes")
    # 真值标注 / 检测结果是否使用世界坐标系输出（false 表示相机光学坐标系，与虚拟相机帧一致）
    offline_yolo_use_world = LaunchConfiguration("offline_yolo_use_world")
    offline_yolo_detection_topic = LaunchConfiguration("offline_yolo_detection_topic")
    offline_yolo_conf_threshold = LaunchConfiguration("offline_yolo_conf_threshold")
    start_visual_grasp_executor = LaunchConfiguration("start_visual_grasp_executor")
    start_motion_execution = LaunchConfiguration("start_motion_execution")
    execution_mode = LaunchConfiguration("execution_mode")
    execute_gripper = LaunchConfiguration("execute_gripper")
    # 规划结果的最大有效期（s）：超过该时长的计划被视为过期，执行器拒绝下发
    max_plan_age_sec = LaunchConfiguration("max_plan_age_sec")
    grasp_verification_enabled = LaunchConfiguration("grasp_verification_enabled")
    grasp_verification_require_contact = LaunchConfiguration(
        "grasp_verification_require_contact"
    )
    candidate_pose_policy = LaunchConfiguration("candidate_pose_policy")
    fixed_grasp_orientation_xyzw = LaunchConfiguration("fixed_grasp_orientation_xyzw")
    candidate_grasp_z_offsets_m = LaunchConfiguration("candidate_grasp_z_offsets_m")
    # 以 yaml 中真机/通用参数为底，再覆盖为「纯仿真 + 开启虚拟相机」的一套参数
    mujoco_parameters = _mujoco_parameters()
    mujoco_parameters.update(
        {
            "backend": "mujoco",
            # 无显示器环境（CI/桌面无关）运行，渲染走后端离屏通道
            "headless": True,
            "arm_namespace": sim_arm_namespace,
            "initial_joint_positions": initial_joint_positions,
            # 虚拟相机默认关闭，本启动文件显式打开
            "virtual_camera.enabled": True,
            "virtual_camera.width": ParameterValue(virtual_camera_width, value_type=int),
            "virtual_camera.height": ParameterValue(virtual_camera_height, value_type=int),
            "virtual_camera.rate_hz": ParameterValue(virtual_camera_rate_hz, value_type=float),
            "virtual_camera.frame_id": virtual_camera_frame_id,
            # 相机固定挂在模型基座坐标系下
            "virtual_camera.parent_frame_id": "base_link",
            "virtual_camera.annotation_topic": virtual_camera_annotation_topic,
        }
    )

    # 默认 MuJoCo 解释器：工作区内的独立虚拟环境；可用环境变量 REBOTARM_MUJOCO_PYTHON 覆盖
    default_mujoco_python = PathJoinSubstitution(
        [EnvironmentVariable("PWD", default_value="."), "third_party", "rebotarm_mujoco_venv", "bin", "python"]
    )
    return LaunchDescription(
        [
            # 仿真臂命名空间：整条感知链路都挂在该命名空间下，与真机话题隔离
            DeclareLaunchArgument("sim_arm_namespace", default_value="rebotarm_sim"),
            # 是否在本文件内额外拉起本地 RViz（默认 false，交由外层/其它终端观看）
            DeclareLaunchArgument("use_local_rviz", default_value="false"),
            # 使用仿真时钟：虚拟相机、轨迹与 TF 全部按 MuJoCo 时间推进
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # 仿真初始关节角（rad，6 关节）：略偏离零位以避免奇异位形，便于观察规划结果
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[-1.5707963267948966, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            # MuJoCo 节点解释器：默认取工作区 third_party 虚拟环境，可用环境变量覆盖
            DeclareLaunchArgument(
                "mujoco_python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_MUJOCO_PYTHON",
                    default_value=default_mujoco_python,
                ),
            ),
            # 视觉节点解释器：默认系统 python3，可用环境变量覆盖
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # GraspNet 推理解释器：与视觉解释器分开配置，便于使用不同依赖环境
            DeclareLaunchArgument(
                "graspnet_python_executable",
                default_value=EnvironmentVariable("GRASPNET_PYTHON", default_value="python3"),
            ),
            # 虚拟彩色/深度图分辨率（像素）：越大越接近真实相机，但渲染与推理更慢
            DeclareLaunchArgument("virtual_camera_width", default_value="640"),
            DeclareLaunchArgument("virtual_camera_height", default_value="480"),
            # 虚拟相机发布频率（Hz）：需与离线 YOLO 的处理能力匹配，过高会积压画面
            DeclareLaunchArgument("virtual_camera_rate_hz", default_value="10.0"),
            # 虚拟相机光学坐标系：GraspNet 候选的默认输出坐标系与之一致
            DeclareLaunchArgument(
                "virtual_camera_frame_id",
                default_value="mujoco_fixed_camera_optical_frame",
            ),
            # 虚拟相机发布的真值检测话题：用于与 YOLO 结果对照，不参与抓取执行
            DeclareLaunchArgument(
                "virtual_camera_annotation_topic",
                default_value="/grasp/ground_truth_detections",
            ),
            # 是否启用离线 YOLO 节点（关掉则只用真值标注）
            DeclareLaunchArgument("offline_yolo_enabled", default_value="true"),
            # YOLO 分割模型权重路径，默认取视觉包 models 目录下的随包权重
            DeclareLaunchArgument(
                "offline_yolo_model_path",
                default_value=PathJoinSubstitution(
                    [
                        vision_share,
                        "models",
                        "yolo26s-seg.pt",
                    ]
                ),
            ),
            # 推理设备编号："0" 表示第一块 GPU，"cpu" 表示 CPU（更慢）
            DeclareLaunchArgument("offline_yolo_device", default_value="0"),
            # 目标类别白名单：只保留这些类别，空列表表示不过滤
            DeclareLaunchArgument("offline_yolo_target_classes", default_value="['bottle']"),
            # 检测结果是否输出到世界坐标系：false 表示输出在相机坐标系（与虚拟相机帧一致）
            DeclareLaunchArgument("offline_yolo_use_world", default_value="false"),
            # 检测结果话题（供下游候选生成订阅）
            DeclareLaunchArgument(
                "offline_yolo_detection_topic",
                default_value="/grasp/detections",
            ),
            # 置信度阈值：调低召回更多候选但噪声更大，离线验证默认放宽到 0.05
            DeclareLaunchArgument("offline_yolo_conf_threshold", default_value="0.05"),
            # 是否启动视觉抓取执行器；默认关闭，只做规划干跑，避免误下发
            DeclareLaunchArgument("start_visual_grasp_executor", default_value="false"),
            # 是否启动点到点运动执行节点；默认关闭，保持链路只读
            DeclareLaunchArgument("start_motion_execution", default_value="false"),
            # 执行模式：plan_only 只规划不下发；execute/real 才真正执行
            DeclareLaunchArgument("execution_mode", default_value="plan_only"),
            # 是否允许夹爪动作：plan_only 下本身不会下发，这里再显式置 false 作为双保险
            DeclareLaunchArgument("execute_gripper", default_value="false"),
            # 计划有效期（s）：超过该时长的规划结果被判定过期并拒绝执行
            DeclareLaunchArgument("max_plan_age_sec", default_value="1.0"),
            # 抓取结果校验：通过夹爪闭合量判断是否真的夹住
            DeclareLaunchArgument("grasp_verification_enabled", default_value="true"),
            # 校验是否必须检测到接触，缺失接触即判定抓取失败
            DeclareLaunchArgument(
                "grasp_verification_require_contact",
                default_value="true",
            ),
            # 候选位姿策略：preserve_candidate_pose 表示保留 GraspNet 给出的原始姿态
            DeclareLaunchArgument(
                "candidate_pose_policy",
                default_value="preserve_candidate_pose",
            ),
            # 固定抓取姿态（x,y,z,w 四元数）：仅当候选姿态策略要求固定姿态时生效
            DeclareLaunchArgument(
                "fixed_grasp_orientation_xyzw",
                default_value="[0.0, 0.0, 0.0, 1.0]",
            ),
            # 抓取点在 z 方向的候选偏移（m）：多值表示对同一候选展开多组变体
            DeclareLaunchArgument(
                "candidate_grasp_z_offsets_m",
                default_value="[0.0]",
            ),
            GroupAction(
                [
                    # 让 MuJoCo 使用 EGL 离屏渲染（无 X11 显示也能出图）
                    SetEnvironmentVariable(name="MUJOCO_GL", value="egl"),
                    # 组内所有节点统一使用仿真时钟
                    SetParameter(name="use_sim_time", value=use_sim_time),
                    # 仿真后端节点：发布仿真关节状态与虚拟相机图像，并提供仿真轨迹执行动作服务
                    Node(
                        package="rebotarm_simulation",
                        executable="rebotarm_mujoco_node",
                        name="rebotarm_mujoco_node",
                        output="screen",
                        prefix=mujoco_python_executable,
                        parameters=[mujoco_parameters],
                    ),
                    # 离线 YOLO 节点：订阅虚拟彩图，输出检测结果与带标注图像
                    Node(
                        package="rebotarm_vision",
                        executable="rebotarm_offline_yolo_node",
                        prefix=LaunchConfiguration("vision_python_executable"),
                        name="rebotarm_offline_yolo_node",
                        output="screen",
                        condition=IfCondition(offline_yolo_enabled),
                        parameters=[
                            {
                                # 输入即虚拟相机发布的彩色图；输出话题与真机视觉保持同名，便于复用下游
                                "offline_yolo.input_topic": "/camera/color/image_raw",
                                "offline_yolo.raw_detection_topic": "/grasp/offline_yolo/raw_detections",
                                "offline_yolo.detection_topic": offline_yolo_detection_topic,
                                "offline_yolo.annotated_topic": "/camera/color/offline_yolo_annotated",
                                "offline_yolo.model_path": offline_yolo_model_path,
                                "offline_yolo.device": offline_yolo_device,
                                "offline_yolo.target_classes": ParameterValue(
                                    offline_yolo_target_classes, value_type=str
                                ),
                                # 字符串 "true"/"false" 先规整为小写再比较，避免大小写写法导致误判
                                "offline_yolo.use_world": ParameterValue(
                                    PythonExpression(
                                        ["'", offline_yolo_use_world, "'.lower() == 'true'"]
                                    ),
                                    value_type=bool,
                                ),
                                "offline_yolo.conf_threshold": ParameterValue(
                                    offline_yolo_conf_threshold, value_type=float
                                ),
                                # NMS 的 IoU 阈值：0.45 为常用折中，偏大保留更多重叠框
                                "offline_yolo.iou_threshold": 0.45,
                                # 发布带框图像，便于 RViz/图像工具人工核对
                                "offline_yolo.publish_annotated": True,
                            }
                        ],
                    ),
                    # 复用视觉抓取系统启动文件，但按离线仿真场景裁剪：只保留感知与候选过滤链路
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [bringup_share, "launch", "visual_grasp_system.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "arm_namespace": sim_arm_namespace,
                            "vision_python_executable": LaunchConfiguration("vision_python_executable"),
                            "graspnet_python_executable": LaunchConfiguration("graspnet_python_executable"),
                            # 明确关闭真实硬件后端：本文件只跑仿真
                            "use_hardware": "false",
                            # 退出时不自动回安全位（仿真场景无需真机回零流程）
                            "shutdown_safe_home": "false",
                            # 仿真轨迹控制器由本文件上方的仿真节点承担，避免同名动作服务重复注册
                            "start_sim_trajectory_controller": "false",
                            "use_local_rviz": use_local_rviz,
                            "execution_mode": execution_mode,
                            # 关闭真机视觉前端与普通物体抓取入口，改由虚拟相机 + 离线 YOLO 供数
                            "start_vision": "false",
                            "start_ordinary_grasp": "false",
                            "start_visual_ready": "false",
                            # 打开 GraspNet 候选生成，并指向本机随包的 Ubuntu 原生配置
                            "start_graspnet_baseline": "true",
                            "graspnet_config": PathJoinSubstitution(
                                [vision_share, "config", "graspnet_ubuntu.yaml"]
                            ),
                            # 候选位姿输出坐标系即虚拟相机光学坐标系，保证与虚拟相机图像一致
                            "graspnet_output_frame_id": virtual_camera_frame_id,
                            "candidate_ik_input_topic": "/grasp/graspnet_candidates",
                            # 打开候选 IK/碰撞过滤，并给定候选姿态与夹爪高度策略
                            "start_candidate_ik_filter": "true",
                            "candidate_pose_policy": candidate_pose_policy,
                            "fixed_grasp_orientation_xyzw": fixed_grasp_orientation_xyzw,
                            "candidate_grasp_z_offsets_m": candidate_grasp_z_offsets_m,
                            # 过滤节点用仿真命名空间下的关节状态做 IK 种子
                            "candidate_joint_state_topic": [
                                "/",
                                sim_arm_namespace,
                                "/visual_joint_states",
                            ],
                            # 工作空间闸门（基座坐标系，m）：只接受桌面正前方的可取候选
                            "candidate_workspace_gate_enabled": "true",
                            "candidate_workspace_min_xyz": "[-0.10, -0.15, 0.0]",
                            "candidate_workspace_max_xyz": "[0.60, 0.15, 0.50]",
                            # 预抓取点与安全抬升点的最低高度（m），避免贴桌碰撞
                            "candidate_pregrasp_min_z_m": "0.05",
                            "candidate_safe_lift_min_z_m": "0.08",
                            # 基座坐标系下的接近方向：-z 即自上而下接近
                            "base_approach_axis_xyz": "[0.0, 0.0, -1.0]",
                            # 限制 joint6 变化量为 0：由位姿策略固定腕部旋转，减少抖动候选
                            "candidate_max_joint6_delta_rad": "0.0",
                            # 候选必须通过碰撞检查
                            "candidate_collision_check_enabled": "true",
                            # 工具中心点相对末端法兰的偏移（m），沿末端 x 轴前伸
                            "tcp_offset_xyz": "[-0.04, 0.0, 0.0]",
                            # 离线场景不启动干跑预览、标记与执行链
                            "start_grasp_preview": "false",
                            "start_visual_grasp_markers": "false",
                            "start_visual_grasp_executor": start_visual_grasp_executor,
                            "start_motion_execution": start_motion_execution,
                            "execution_mode": execution_mode,
                            "execute_gripper": execute_gripper,
                            "max_plan_age_sec": max_plan_age_sec,
                            "grasp_verification_enabled": grasp_verification_enabled,
                            "grasp_verification_require_contact": grasp_verification_require_contact,
                        }.items(),
                    ),
                ]
            ),
        ]
    )
