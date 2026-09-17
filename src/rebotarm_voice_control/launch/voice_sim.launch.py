"""语音控制仿真模式启动文件。

用途：在仿真后端下把语音控制栈拉起来，不接触真实机械臂。

启动的节点组合：
    1. rebotarm_voice_control_node        —— 语音控制常驻节点（骨架/就绪标志）；
    2. rebotarm_sim_move_relative_action  —— 仿真相对位移动作服务端，
       提供语音 move_relative 指令所需的仿真动作接口。

后端选择逻辑：通过 ``execution_mode`` 启动参数选择，本文件安全默认值为 "sim"；
执行模式路由位于语音控制包内，会把命令目标重写到仿真命名空间下的动作
（见 config/sim_config.yaml 中的动作映射）。真机模式另有独立启动文件，本文件
不会打开真实 ROS 调用开关。

安全默认值：默认 "sim" 意味着即使误启动本文件也不会下发真实硬件指令；若显式
传入 execution_mode:=real，仍会被 safety_limits.yaml 的 allow_real_ros_calls
开关拦截。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 由启动参数决定的执行模式；默认 sim，仅用于仿真验证。
    execution_mode = LaunchConfiguration("execution_mode")
    return LaunchDescription(
        [
            # 可覆盖参数：execution_mode（字符串），默认 "sim"。
            DeclareLaunchArgument("execution_mode", default_value="sim"),
            # 语音控制常驻节点：接收 execution_mode 参数以决定路由目标后端。
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_voice_control_node",
                name="rebotarm_voice_control_node",
                output="screen",
                parameters=[
                    {
                        "execution_mode": execution_mode,
                    }
                ],
            ),
            # 仿真相对位移动作服务端：为语音 move_relative 提供仿真后端。
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_sim_move_relative_action",
                name="rebotarm_sim_move_relative_action",
                output="screen",
            ),
        ]
    )
