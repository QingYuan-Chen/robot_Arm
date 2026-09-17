"""实时语音事件网关的启动文件（事件流回放，不接麦克风）。

用途：启动 ``rebotarm_realtime_gateway`` 可执行文件，从 JSONL 事件源逐条读取
"实时语音事件"，经工具白名单校验与执行模式路由后，把每条成功路由的结果以
JSON 列表打印到标准输出。该入口不访问麦克风或网络，因此可在离线环境复现
整条实时链路（事件 → 工具调用 → 路由），也是实时相关测试使用的入口。

节点组合：本文件只启动一个网关节点，不启动解析、仿真动作或真实运动节点；
若需要闭环执行，需另行启动对应后端（仿真动作服务端或上层启动组合包）。

后端选择：``execution_mode`` 参数即后端选择开关，逐级过滤——
  dry_run（本文件默认）：只做接口映射，不下发任何调用；
  sim：路由到 /rebotarm/sim 前缀的仿真动作；
  real：仍会被 safety_limits.yaml 的 allow_real_ros_calls 二次拦截，
        该键为 false（仓库默认）时直接判为安全违规。
默认值刻意取 dry_run，保证"不带参数直接启动"不会产生任何真实动作。

参数来源：两个启动参数均为命令行可覆盖的声明式参数；``event_jsonl`` 默认空
字符串，因此必须显式传入事件文件路径才能得到非空回放结果。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """装配网关节点：声明启动参数并把它们透传给可执行文件的命令行。"""
    execution_mode = LaunchConfiguration("execution_mode")
    event_jsonl = LaunchConfiguration("event_jsonl")
    return LaunchDescription(
        [
            # 执行模式启动参数，默认 dry_run（只路由、不下发）；
            # 可选值 "dry_run" / "sim" / "real" 对应网关的 --mode 取值。
            DeclareLaunchArgument("execution_mode", default_value="dry_run"),
            # 实时事件源文件路径：JSONL 格式，每行一条事件；默认空表示未提供，
            # 网关的 --event-jsonl 为必填参数，留空会以空路径打开而失败。
            DeclareLaunchArgument("event_jsonl", default_value=""),
            # 实时事件网关节点：package/executable/name 三者同名，均为对外契约，
            # 与 setup.py 注册的 console_scripts 入口保持一致，改名会同时影响
            # 启动、测试与监控；output="screen" 便于在终端直接看到回放结果 JSON。
            Node(
                package="rebotarm_voice_control",
                executable="rebotarm_realtime_gateway",
                name="rebotarm_realtime_gateway",
                output="screen",
                arguments=[
                    "--event-jsonl",
                    event_jsonl,
                    "--mode",
                    execution_mode,
                ],
            ),
        ]
    )
