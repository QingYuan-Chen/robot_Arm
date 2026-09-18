#真实硬件下的 RViz 末端拖动入口（MoveIt 原生交互标记 + 真机执行）。（已测）
#
#用途：在 RViz 里用 MoveIt 原生的交互标记拖动机械臂末端，由规划层生成轨迹后交给真实控制器执行。这是“人给出末端目标 → 规划 → 真机运动”的完整链路入口，不接仿真执行后端。
#
#启动组合：
#    1. 本包内的交互系统共享启动文件（真机 / 仿真二选一的后端选择逻辑都在其中），这里显式选择「真机」一侧；
#    2. 真机控制器节点由该共享启动文件拉起，串口通道由本文件解析后传入。
#
#真实/仿真后端选择逻辑："use_hardware=true" 打开真实电机通道；"use_local_rviz=true"让共享启动文件拉起 RViz（末端拖动界面所在）；"use_moveit_preview=true" 使用规划包自带的预览栈提供规划能力；
#"start_passive_joint_state_publisher=false" 抑制额外的关节状态源，"use_moveit_fake_joint_states=false" 让规划器读取真实控制器的关节状态——两者必须同时为假，否则规划器会看到与真机不一致的关节状态。
#
#通道解析："channel" 默认 "auto"，按 /dev/ttyACM0、/dev/ttyACM1 的顺序探测第一个存在的串口桥；都找不到时回落为 /dev/ttyACM0，由控制器在连接阶段如实报错退出，而不是静默换口。
#
#安全默认值：本文件不做任何自动使能。共享启动文件侧的默认值仍是 "cmd_arbitration=reject"与 "shutdown_safe_home=false"：命令冲突时拒绝，退出时不自动回安全位。真机上电后必须由操作者显式调用 enable 服务，且在确认工作空间无人后才会真正运动。

from __future__ import annotations

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_channel(context: LaunchContext) -> str:
    """解析真实硬件使用的串口通道。

    参数：``context`` 为启动上下文，用于把 ``channel`` 这个延迟求值的替换表达式求成
    具体字符串。
    返回：设备路径字符串。``channel`` 非空且不是 ``auto`` 时原样返回；否则按
    /dev/ttyACM0、/dev/ttyACM1 的顺序返回第一个存在的设备；都不存在时返回
    /dev/ttyACM0，让控制器自己在连接阶段报错，避免在这里悄悄换到别的设备。
    """
    channel = LaunchConfiguration("channel").perform(context)
    if channel and channel != "auto":
        return channel
    for candidate in ("/dev/ttyACM0", "/dev/ttyACM1"):
        if Path(candidate).exists():
            return candidate
    return "/dev/ttyACM0"


def _include_system(context: LaunchContext):
    """构建并返回被包含的交互系统启动描述（延后到启动阶段执行）。

    用 OpaqueFunction 包装的原因是通道探测需要依赖运行机上的真实设备节点：参数替换阶段
    还拿不到 ``channel`` 的最终值，必须等到启动上下文可用时再解析。
    """
    bringup_share = FindPackageShare("rebotarm_bringup")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "interactive_system.launch.py"])
            ),
            # use_hardware=true：打开真实电机通道（本入口与仿真入口的唯一分野）；
            # use_local_rviz=true：由共享启动文件拉起 RViz 拖动界面；
            # use_moveit_preview=true：规划能力来自规划包自带的预览栈；
            # start_passive_joint_state_publisher=false 且 use_moveit_fake_joint_states=false：
            # 真机控制器是本入口唯一的执行侧关节状态来源，不要再叠加假状态源。
            launch_arguments={
                "channel": _resolve_channel(context),
                "use_hardware": "true",
                "use_local_rviz": "true",
                "use_moveit_preview": "true",
                "start_passive_joint_state_publisher": "false",
                "use_moveit_fake_joint_states": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    """构建本入口的 LaunchDescription：声明通道参数并延后包含交互系统启动文件。"""
    return LaunchDescription(
        [
            # channel：电机总线通道。默认 auto = 按 /dev/ttyACM0、/dev/ttyACM1 顺序探测；
            # 显式给出设备路径（或 can0/can1）时优先使用该值。
            DeclareLaunchArgument("channel", default_value="auto"),
            OpaqueFunction(function=_include_system),
        ]
    )
