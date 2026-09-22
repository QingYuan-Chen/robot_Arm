"""视觉包的主启动文件：真实相机路径下启动感知链路所需的节点组合。

启动内容共 4 个节点：

1. 手眼静态 TF：按 ``handeye.yaml`` 把相机坐标系挂到机械臂坐标系上（静态变换，不随
   关节运动改变）；
2. 视觉主节点 ``rebotarm_vision_node``：彩色/深度采集、检测与深度融合，参数来自
   ``camera_config`` 中同名段；
3. 普通抓取节点 ``rebotarm_ordinary_grasp_node``（可选，默认不启动）：只输出候选位姿，
   不执行运动；
4. 夹爪 TCP 静态帧节点 ``rebotarm_grasp_tcp_frame``：发布末端到抓取 TCP 的固定偏移，
   供抓取位姿换算使用。

参数来源：节点参数统一从 ``camera_config`` 指向的 YAML 读取，再按启动参数覆盖。三个
节点都用 ``vision_python_executable`` 作为解释器前缀（默认取环境变量
``REBOTARM_VISION_PYTHON``，未设置时用 ``python3``），便于把带推理依赖的独立虚拟环境
解释器固定下来。

安全边界：本文件只负责感知，不启动运动执行、不使能硬件。任何基于视觉结果的运动都必须
由上层在通过规划、碰撞检查与执行门控后下发。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rebotarm_vision.handeye_config import load_handeye_config


def _node_parameters(config_path: str, node_name: str) -> dict:
    """从 YAML 中取出指定节点的 ``ros__parameters`` 段并转成普通 dict。

    只做读取与类型检查，不填充默认值：文件为空或缺该节点段时返回空字典，交由节点的
    ``declare_parameter`` 兜底；``ros__parameters`` 不是映射时抛 ``RuntimeError``——
    宁可启动失败，也不要带着读错的参数跑起来。
    """

    payload = yaml.safe_load(Path(config_path).read_text(encoding="utf-8")) or {}
    node_payload = payload.get(node_name, {})
    parameters = node_payload.get("ros__parameters", {}) if isinstance(node_payload, dict) else {}
    if not isinstance(parameters, dict):
        raise RuntimeError(f"invalid ros__parameters for {node_name}: {config_path}")
    return dict(parameters)


def _launch_setup(context):
    """延迟执行的回调：等所有启动参数（含环境变量默认值）确定后再组装节点。

    流程：读取启动参数 → 载入手眼标定配置 → 组装公共环境变量 → 用 ``yolo_model_path``
    / ``yolo_device`` 覆盖视觉节点的 YOLO 参数 → 组装三个节点的参数 → 返回节点列表。

    普通抓取节点的参数在 YAML 之外还有两处显式覆盖：
    ``ordinary_grasp.candidates_topic`` 固定为 /grasp/candidates（与上层消费方约定），
    ``depth_quality.override_enabled`` 置 True 并绑定 ``ordinary_depth_quality_enabled``，
    即由启动开关强制决定深度质量门控状态，忽略配置文件里的取值。
    """

    camera_config = LaunchConfiguration("camera_config").perform(context)
    handeye_config = LaunchConfiguration("handeye_config").perform(context)
    ordinary_grasp_root = LaunchConfiguration("ordinary_grasp_root").perform(context)
    yolo_model_path = LaunchConfiguration("yolo_model_path").perform(context)
    yolo_device = LaunchConfiguration("yolo_device").perform(context)
    handeye = load_handeye_config(Path(handeye_config).expanduser())

    # OpenCV highgui 的 Qt 后端需要指定平台插件与字体目录，否则在无 Wayland/缺字体的
    # 主机上创建预览窗口会失败；这两个键有测试锁定，不要改动字面量。
    common_environment = {
        "QT_QPA_PLATFORM": "xcb",
        "QT_QPA_FONTDIR": "/usr/share/fonts/truetype/dejavu",
    }
    # 只有显式传入非空启动参数才覆盖 YAML，空串表示沿用配置文件里的取值。
    vision_overrides = {}
    if yolo_model_path:
        vision_overrides["yolo.model_path"] = yolo_model_path
    if yolo_device:
        vision_overrides["yolo.device"] = yolo_device
    vision_parameters = _node_parameters(camera_config, "rebotarm_vision_node")
    vision_parameters.update(vision_overrides)
    ordinary_parameters = _node_parameters(camera_config, "rebotarm_ordinary_grasp_node")
    # 候选话题与深度质量门控由启动层统一决定，避免与上层配置不一致；
    # override_value 走 ParameterValue(..., value_type=bool) 是为了把启动参数转成真正的
    # 布尔值，而不是字符串 "true"/"false"。
    ordinary_parameters.update(
        {
            "ordinary_grasp.root": ordinary_grasp_root,
            "ordinary_grasp.candidates_topic": "/grasp/candidates",
            "depth_quality.override_enabled": True,
            "depth_quality.override_value": ParameterValue(
                LaunchConfiguration("ordinary_depth_quality_enabled"),
                value_type=bool,
            ),
        }
    )
    tcp_parameters = _node_parameters(camera_config, "rebotarm_grasp_tcp_frame")

    return [
        # 手眼静态 TF：参数顺序沿用 static_transform_publisher 的位置参数约定
        # （x y z qx qy qz qw 父坐标系 子坐标系），数值全部来自 handeye.yaml。
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rebotarm_handeye_static_tf",
            output="screen",
            arguments=handeye.as_static_transform_arguments(),
        ),
        # 视觉主节点：负责取图、检测与深度融合，不下发任何运动指令。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_vision_node",
            prefix=LaunchConfiguration("vision_python_executable"),
            name="rebotarm_vision_node",
            output="screen",
            parameters=[vision_parameters],
            additional_env=common_environment,
        ),
        # 普通抓取节点：默认不启动（start_ordinary_grasp=false），只发布候选位姿。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_ordinary_grasp_node",
            prefix=LaunchConfiguration("vision_python_executable"),
            name="rebotarm_ordinary_grasp_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_ordinary_grasp")),
            parameters=[ordinary_parameters],
            additional_env=common_environment,
        ),
        # 夹爪 TCP 静态帧：发布末端到抓取 TCP 的固定偏移（父子坐标系与偏移量取自 YAML）。
        Node(
            package="rebotarm_vision",
            executable="rebotarm_grasp_tcp_frame",
            prefix=LaunchConfiguration("vision_python_executable"),
            name="rebotarm_grasp_tcp_frame",
            output="screen",
            parameters=[tcp_parameters],
            additional_env=common_environment,
        ),
    ]


def generate_launch_description():
    """声明全部启动参数，并注册延迟执行的 ``_launch_setup``。

    默认值的含义与安全考虑：

    - ``camera_config`` / ``handeye_config``：默认取安装后的包内配置，避免机器相关绝对
      路径；整体替换即可切换到别的相机或标定结果；
    - ``ordinary_grasp_root``：普通抓取的资源根目录，默认空串表示由节点按自身规则解析；
    - ``vision_python_executable``：解释器前缀，优先取环境变量
      ``REBOTARM_VISION_PYTHON``，未设置时用 ``python3``；
    - ``start_ordinary_grasp`` 默认 "false"：普通抓取节点默认不启动，需要时显式开启；
    - ``ordinary_depth_quality_enabled`` 默认 "true"：普通抓取的深度质量门控默认开启，
      只有显式传 false 才关闭——深度不合格时不应继续算抓取；
    - ``yolo_model_path`` / ``yolo_device`` 默认空串：表示不覆盖 YAML 里的模型路径与
      推理设备。
    """

    vision_share = Path(get_package_share_directory("rebotarm_vision"))

    return LaunchDescription(
        [
            # 相机与节点参数配置文件（内部按节点名分段）。
            DeclareLaunchArgument(
                "camera_config",
                default_value=str(vision_share / "config" / "camera_ubuntu.yaml"),
            ),
            # 手眼标定结果（父/子坐标系与平移、四元数），决定相机在机械臂下的位姿。
            DeclareLaunchArgument(
                "handeye_config",
                default_value=str(vision_share / "config" / "handeye.yaml"),
            ),
            # 普通抓取资源根目录；空串表示不指定。
            DeclareLaunchArgument("ordinary_grasp_root", default_value=""),
            # 视觉节点解释器：环境变量优先，缺省 python3。
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # 是否启动普通抓取节点；默认关闭，避免无意中发布抓取候选。
            DeclareLaunchArgument("start_ordinary_grasp", default_value="false"),
            # 普通抓取的深度质量门控开关；默认开启。
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            # YOLO 模型路径与推理设备覆盖值；空串表示沿用配置文件。
            DeclareLaunchArgument("yolo_model_path", default_value=""),
            DeclareLaunchArgument("yolo_device", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
