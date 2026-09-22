"""只启动被显式选中的无头 MuJoCo 仿真 ROS 适配节点。

启动内容与用途：本文件只拉起一个节点（``rebotarm_mujoco_node``），把 MuJoCo 物理仿真暴露成
与真实控制器一致的 ROS 接口（关节状态、轨迹动作、夹爪服务、``/clock`` 等），供运动规划、
示教回放等上层程序在“仿真后端”上做不带硬件的联调与回归。

后端选择逻辑：节点参数里把 ``backend`` 固定为 ``mujoco``、``headless`` 固定为 ``True``，
也就是说本 launch 不会打开图形化 viewer，也不会选择真实电机后端；需要界面时由上层组合
其它启动文件并在那里再开启 viewer/RViz。仿真启动不打开任何硬件通道。

参数来源与安全默认值：

- 参数先读安装目录下的 ``config/mujoco_sim.yaml``，再用本文件里的字典覆盖少数需要由命令行
  决定的项（命名空间、初始关节角、虚拟相机）；YAML 里的节点默认值仍然生效；
- 虚拟相机默认关闭：离屏渲染需要 EGL/GPU 支持，纯运动联调不应被图形依赖拖累；
- 初始关节角是一个小幅预弯姿态而不是零位，避免上电瞬间处于奇异构型；
- ``python_executable`` 默认指向工作目录下的 MuJoCo 虚拟环境，保证能 import 到 MuJoCo 与 ROS 2
  绑定；可用环境变量 ``REBOTARM_MUJOCO_PYTHON`` 或启动参数覆盖。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # 节点参数基线：安装目录下的 mujoco_sim.yaml（仿真节拍、容差、虚拟相机等默认值都在那里）。
    config = Path(get_package_share_directory("rebotarm_simulation")) / "config" / "mujoco_sim.yaml"
    # 以下 LaunchConfiguration 均对应本文件声明的同名启动参数，用于把命令行取值注入节点参数。
    python_executable = LaunchConfiguration("python_executable")
    mujoco_arm_namespace = LaunchConfiguration("mujoco_arm_namespace")
    initial_joint_positions = LaunchConfiguration("initial_joint_positions")
    enable_virtual_camera = LaunchConfiguration("enable_virtual_camera")
    virtual_camera_width = LaunchConfiguration("virtual_camera_width")
    virtual_camera_height = LaunchConfiguration("virtual_camera_height")
    virtual_camera_rate_hz = LaunchConfiguration("virtual_camera_rate_hz")
    virtual_camera_frame_id = LaunchConfiguration("virtual_camera_frame_id")
    virtual_camera_parent_frame_id = LaunchConfiguration(
        "virtual_camera_parent_frame_id"
    )
    mujoco_gl = LaunchConfiguration("mujoco_gl")
    # 兜底的解释器路径：工作目录下的 MuJoCo 虚拟环境；相对于 PWD 解析，找不到会由进程启动阶段报错。
    default_mujoco_python = PathJoinSubstitution(
        [EnvironmentVariable("PWD", default_value="."), "third_party", "rebotarm_mujoco_venv", "bin", "python"]
    )
    return LaunchDescription(
        [
            # 运行本节点的 Python 解释器：必须同时具备 MuJoCo 与 ROS 2 依赖。
            # 优先级为 启动参数 > 环境变量 REBOTARM_MUJOCO_PYTHON > 工作目录下的虚拟环境。
            DeclareLaunchArgument(
                "python_executable",
                default_value=EnvironmentVariable(
                    "REBOTARM_MUJOCO_PYTHON",
                    default_value=default_mujoco_python,
                ),
                description="Python interpreter containing MuJoCo and ROS 2 dependencies",
            ),
            # 仿真对外的话题/服务/动作命名空间（默认 rebotarm），必须与上层期望的前缀一致。
            DeclareLaunchArgument("mujoco_arm_namespace", default_value="rebotarm"),
            # 上电初始的六个手臂关节角（rad，按 joint1..joint6 顺序）：小幅预弯姿态，避免零位奇异。
            DeclareLaunchArgument(
                "initial_joint_positions",
                default_value="[0.0, -0.1, -0.2, 0.2, 0.0, 0.0]",
            ),
            # 虚拟 RGB-D 相机开关：默认关闭，纯运动联调不需要离屏渲染与 EGL。
            DeclareLaunchArgument("enable_virtual_camera", default_value="false"),
            # 虚拟相机图像宽度（像素）；仅在开启虚拟相机时生效。
            DeclareLaunchArgument("virtual_camera_width", default_value="640"),
            # 虚拟相机图像高度（像素）；仅在开启虚拟相机时生效。
            DeclareLaunchArgument("virtual_camera_height", default_value="480"),
            # 虚拟相机帧率（Hz）；调高会增加渲染与发布开销，调低则感知链路更迟钝。
            DeclareLaunchArgument("virtual_camera_rate_hz", default_value="15.0"),
            # 虚拟相机光学坐标系名（遵循光学惯例：z 向前、x 向右、y 向下）。
            DeclareLaunchArgument(
                "virtual_camera_frame_id",
                default_value="mujoco_fixed_camera_optical_frame",
            ),
            # 虚拟相机静态外参的父坐标系；相机在该坐标系下的位姿由节点从模型中的安装位姿读取。
            DeclareLaunchArgument(
                "virtual_camera_parent_frame_id", default_value="base_link"
            ),
            # MuJoCo 离屏渲染后端（MUJOCO_GL）：egl 适合无显示器的服务器，桌面环境可改用 glfw，软件渲染用 osmesa。
            DeclareLaunchArgument("mujoco_gl", default_value="egl"),
            # 唯一的仿真节点：backend/headless 在此固定为 mujoco/true，保证本 launch 只驱动仿真、
            # 不打开 viewer、也不会启用任何真实硬件通道。
            Node(
                package="rebotarm_simulation",
                executable="rebotarm_mujoco_node",
                name="rebotarm_mujoco_node",
                output="screen",
                # 用上面的解释器前缀启动，确保该进程能 import 到 MuJoCo。
                prefix=python_executable,
                parameters=[
                    str(config),
                    {
                        # 仿真后端标识；本文件只允许 mujoco，真实电机后端由别的启动组合选择。
                        "backend": "mujoco",
                        # 无头模式：不创建可视化窗口，可在服务器/CI 上运行。
                        "headless": True,
                        # 命名空间覆盖，来自启动参数。
                        "arm_namespace": mujoco_arm_namespace,
                        # 初始关节角覆盖，来自启动参数（rad，joint1..joint6 顺序）。
                        "initial_joint_positions": initial_joint_positions,
                        # 以下为虚拟相机参数覆盖；enabled 为 false 时其余参数不产生渲染开销。
                        "virtual_camera.enabled": ParameterValue(
                            enable_virtual_camera, value_type=bool
                        ),
                        "virtual_camera.width": ParameterValue(
                            virtual_camera_width, value_type=int
                        ),
                        "virtual_camera.height": ParameterValue(
                            virtual_camera_height, value_type=int
                        ),
                        "virtual_camera.rate_hz": ParameterValue(
                            virtual_camera_rate_hz, value_type=float
                        ),
                        "virtual_camera.frame_id": virtual_camera_frame_id,
                        "virtual_camera.parent_frame_id": virtual_camera_parent_frame_id,
                    },
                ],
                # 只给该子进程注入 MUJOCO_GL，避免污染父进程与其它节点。
                additional_env={"MUJOCO_GL": mujoco_gl},
            )
        ]
    )
