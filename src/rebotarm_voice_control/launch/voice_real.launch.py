"""真实机械臂语音控制的启动文件（把执行模式设为 real 的最小入口）。

用途：启动 ``rebotarm_voice_control_node`` 并把它托管给语音控制链路的真实执行
模式。它本身只是"把模式拨到 real"的入口，真正的安全闸门不在本文件，而在
safety_limits.yaml：只有该文件的 allow_real_ros_calls 显式为 true 时，real
模式才可能下发真实调用；仓库默认该键为 false，因此即使本文件被启动，命令也会
被判为安全违规而不是静默降级或直接执行。

节点组合：只启动一个语音控制常驻节点，不启动运动、控制器或仿真节点；因此该
节点必须与真实后端（上层启动组合包提供的运动/控制器接口）配合使用才具备完整
的闭环能力。

后端选择：``execution_mode`` 默认值 "real" 是本文件与演练/仿真启动文件的唯一
区别；它作为 ROS 参数以 "execution_mode" 键传入节点。execution_mode 的完整
优先级为：构造参数 > safety_limits.yaml 的 execution_mode 键 > "dry_run"。

安全默认值说明：默认值取 real 是把"模式选择"显式交给操作者，并不等于默认打开
真实调用；allow_real_ros_calls 的第二道闸门必须由人工在配置中打开，这样避免了
"启动即运动"这类误操作。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """装配真实模式节点：声明 execution_mode 参数并透传给节点参数表。"""
    execution_mode = LaunchConfiguration("execution_mode")
    return LaunchDescription(
        [
            # 执行模式启动参数，默认 "real"（真实后端）；
            # 可覆盖为 "dry_run"/"sim" 以便用同一份文件做降级演练或仿真联调。
            DeclareLaunchArgument("execution_mode", default_value="real"),
            # 语音控制常驻节点：package/executable/name 同名，均为对外契约，
            # 与 setup.py 注册的 console_scripts 入口一致；output="screen" 便于
            # 在终端直接观察节点日志。
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_voice_control_node",
                name="rebotarm_voice_control_node",
                output="screen",
                parameters=[
                    {
                        # 传入执行模式；节点当前尚未 declare 该参数，但该键是
                        # 真实/仿真/演练三条启动路径共用的统一接口，不可改名。
                        "execution_mode": execution_mode,
                    }
                ],
            ),
        ]
    )
