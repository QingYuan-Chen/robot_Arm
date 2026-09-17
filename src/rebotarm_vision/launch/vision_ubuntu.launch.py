"""Ubuntu 原生视觉链路的启动入口（vision_profile 的 Ubuntu 原生 profile）。

职责：只做"选参数 + 转交"，把 Ubuntu 主机上经过验证的默认取值整理好后，
原样包含同目录下的通用启动文件 vision.launch.py，真正的节点组合都在那里。

为什么需要这一层：通用启动文件接受较多参数（相机配置、模型路径、解释器路径等），
命令行逐个传容易漏项或传错；本文件固化 Ubuntu 上的已验证组合：
    camera_config  -> share 目录下的 camera_ubuntu.yaml（Gemini 2 原生 D2C profile）
    yolo_model_path-> share 目录下的 yolo26s-seg.pt（安装期打包的检测权重）
    handeye_config -> share 目录下的 handeye.yaml（手眼外参，决定相机与末端的 TF 关系）

启动参数（均可在命令行覆盖）：
    yolo_device:                  YOLO 推理设备，"0" 为第 0 号 GPU，"cpu" 为纯 CPU。
    vision_python_executable:     运行视觉节点的解释器。默认取环境变量 REBOTARM_VISION_PYTHON，
                                  未设置时回退 python3 —— 视觉依赖（如 TensorRT）通常装在独立解释器里，
                                  用错解释器会以缺失依赖的形式在运行期才暴露。
    yolo_model_path:              检测权重完整路径，默认 share/rebotarm_vision/models/yolo26s-seg.pt。
    handeye_config:               手眼标定 YAML 路径，默认 share/rebotarm_vision/config/handeye.yaml。
    start_ordinary_grasp:         是否启动传统"检测框 + 深度"抓取规划链路，默认 false（不启动）。
    ordinary_grasp_root:          传统链路所需的模型/资源根目录，默认空串（未配置）。仅当上项为 true 时必填。
    ordinary_depth_quality_enabled: 传统链路是否启用深度质量门控，默认 true。

以上参数只是转交给 vision.launch.py，本文件不直接构造任何节点，也不决定真实/仿真后端。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    """构造启动描述：声明参数并把它们转交给通用视觉启动文件。"""
    # 用安装后的 share 目录（而非源码树）定位配置与模型，保证 install 空间自洽
    vision_share = Path(get_package_share_directory("rebotarm_vision"))

    return LaunchDescription(
        [
            # GPU 设备号，默认 "0"
            DeclareLaunchArgument("yolo_device", default_value="0"),
            # 视觉解释器：优先环境变量 REBOTARM_VISION_PYTHON，未设置时回退 python3
            DeclareLaunchArgument(
                "vision_python_executable",
                default_value=EnvironmentVariable("REBOTARM_VISION_PYTHON", default_value="python3"),
            ),
            # 检测权重路径：默认指向本包安装目录下的 yolo26s-seg.pt
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value=str(
                    vision_share
                    / "models"
                    / "yolo26s-seg.pt"
                ),
            ),
            # 手眼标定文件路径
            DeclareLaunchArgument(
                "handeye_config",
                default_value=str(vision_share / "config" / "handeye.yaml"),
            ),
            # 传统抓取链路默认关闭：Ubuntu 原生 profile 走 GraspNet 候选链路
            DeclareLaunchArgument("start_ordinary_grasp", default_value="false"),
            # 传统链路资源根目录，默认空串表示未配置（启用传统链路时必须显式提供）
            DeclareLaunchArgument("ordinary_grasp_root", default_value=""),
            # 传统链路深度质量门控开关
            DeclareLaunchArgument("ordinary_depth_quality_enabled", default_value="true"),
            # 包含通用启动文件：相机配置固定为 Ubuntu 原生 profile，其余参数透传
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(vision_share / "launch" / "vision.launch.py")
                ),
                launch_arguments={
                    "camera_config": str(
                        vision_share / "config" / "camera_ubuntu.yaml"
                    ),
                    "yolo_model_path": LaunchConfiguration("yolo_model_path"),
                    "yolo_device": LaunchConfiguration("yolo_device"),
                    "vision_python_executable": LaunchConfiguration("vision_python_executable"),
                    "handeye_config": LaunchConfiguration("handeye_config"),
                    "start_ordinary_grasp": LaunchConfiguration("start_ordinary_grasp"),
                    "ordinary_grasp_root": LaunchConfiguration("ordinary_grasp_root"),
                    "ordinary_depth_quality_enabled": LaunchConfiguration(
                        "ordinary_depth_quality_enabled"
                    ),
                }.items(),
            )
        ]
    )
