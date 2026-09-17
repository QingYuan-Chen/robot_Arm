"""语音控制节点的默认启动文件（演练模式 dry_run，安全默认入口）。

用途：启动 ``rebotarm_voice_control_node`` 并保持演练模式。演练模式下命令只做
"意图 → 接口"的映射，路由结果带 dry_run 标志，执行器据此拒绝真正下发；因此
这是唯一可以"随便启动、不会动机器人"的语音控制入口，适合配置校验与联调。

节点组合：只启动一个语音控制常驻节点，不启动仿真动作服务端、不启动真实运动或
控制器接口；需要仿真闭环时使用同包的仿真启动文件，需要真实闭环时使用真实启动
文件（并另行打开 safety_limits.yaml 的真实调用开关）。

后端选择：``execution_mode`` 默认 "dry_run"；可覆盖为 "sim"（改写为
/rebotarm/sim 前缀的仿真动作）或 "real"（仍受 allow_real_ros_calls 二次拦截）。
模式优先级：构造参数 > safety_limits.yaml 的 execution_mode 键 > "dry_run"。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """装配演练模式节点：声明 execution_mode 参数并透传给节点参数表。"""
    execution_mode = LaunchConfiguration("execution_mode")
    return LaunchDescription(
        [
            # 执行模式启动参数，默认 "dry_run"：不产生任何真实调用，是安全默认值。
            DeclareLaunchArgument("execution_mode", default_value="dry_run"),
            # 语音控制常驻节点：package/executable/name 同名，均为对外契约，
            # 与 setup.py 注册的 console_scripts 入口一致；output="screen" 便于
            # 在终端直接观察节点启动日志。
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_voice_control_node",
                name="rebotarm_voice_control_node",
                output="screen",
                parameters=[
                    {
                        # 传入执行模式；与真实/仿真启动文件保持同一参数键名。
                        "execution_mode": execution_mode,
                    }
                ],
            )
        ]
    )
