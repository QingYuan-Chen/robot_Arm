"""示教录制专用启动文件（只起示教录制节点，不附带键盘/面板/可视化）。

启动用途：在已经存在真实硬件控制器（或已在别处完成重力补偿使能）的前提下，单独拉起
示教录制节点，把操作者手拖机械臂得到的关节样本写进 JSONL 记录文件。

节点组合：只有 TeachRecorderNode 一个节点，因此本文件不与任何启动组合争抢状态发布、
RViz 或关节状态话题；它依赖外部已经把机械臂控制起来并进入重力补偿模式。

真实/仿真后端选择逻辑：没有分支，本文件既不启动硬件控制器也不启动仿真控制器——录制
节点是“纯观察者”，只订阅状态与关节反馈。也就是说本文件不会让机械臂运动，但也不会替
操作者使能电机：真机上电后仍是失能态，必须先显式调用 enable，再显式进入重力补偿。

参数来源：common_config 提供关节名顺序，teach_config 提供录制与重力补偿参数；launch
参数只覆盖命名空间、记录路径与是否自动启动重力补偿。

安全默认值：auto_start_gravity_comp=false——是否松开抱闸交由操作者显式决定，避免节点
一启动就让机械臂失去刚性；record_path 为相对路径，默认写到工作目录下的
teleop_records/teach_record.jsonl，父目录不存在时会自动创建。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 延迟求值的 LaunchConfiguration 句柄：真实取值在启动阶段由下方默认值或命令行覆盖决定。
    arm_namespace = LaunchConfiguration("arm_namespace")
    record_path = LaunchConfiguration("record_path")
    auto_start_gravity_comp = LaunchConfiguration("auto_start_gravity_comp")
    common_config = LaunchConfiguration("common_config")
    teach_config = LaunchConfiguration("teach_config")
    keyboard_prefix = LaunchConfiguration("keyboard_prefix")
    config_share = FindPackageShare("rebotarm_bringup")

    return LaunchDescription(
        [
            # arm_namespace：话题/服务命名空间段，必须与控制器一致，否则取不到关节反馈。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # record_path：示教记录 JSONL 输出路径；相对路径按节点工作目录解析，父目录会
            # 自动创建。改成绝对路径可避免受启动目录影响。
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            # auto_start_gravity_comp（bool）：为真时录制节点自行周期性调用重力补偿启动
            # 服务，省去手动使能步骤；默认 false，是否松开抱闸必须由操作者决定。
            DeclareLaunchArgument("auto_start_gravity_comp", default_value="false"),
            # keyboard_prefix：进程启动前缀。留空表示不额外包装；由上层组合启动时通常传入
            # 把标准输入接到 /dev/tty 的 bash 包装（录制节点监听退出键需要真实 TTY）。
            DeclareLaunchArgument(
                "keyboard_prefix",
                default_value="",
            ),
            # common_config：公共命名空间、关节名称和限位。
            DeclareLaunchArgument(
                "common_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "operator_common.yaml"]
                ),
            ),
            # teach_config：仅示教录制/回放参数。
            DeclareLaunchArgument(
                "teach_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "teach_control.yaml"]
                ),
            ),
            # 示教录制节点：订阅关节反馈与整臂状态，按 sample_rate_hz 写 JSONL 样本。
            # 覆盖项含义：record_path 指定输出文件；auto_start_gravity_comp 决定是否由本
            # 节点代为触发重力补偿（默认否）。其余录制参数来自 teach_config。
            Node(
                package="rebotarm_teach",
                executable="TeachRecorderNode",
                name="teach_recorder_node",
                output="screen",
                prefix=keyboard_prefix,
                parameters=[
                    common_config,
                    teach_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "auto_start_gravity_comp": auto_start_gravity_comp,
                    },
                ],
            ),
        ]
    )
