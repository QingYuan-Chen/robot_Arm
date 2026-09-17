"""示教回放启动文件：把录制好的关节轨迹安全地下发到唯一的执行后端。

用途
    加载一条示教记录（JSONL），依次做质量分析、预处理（平滑/低通滤波/重采样/重定时）、
    起点对齐判定、碰撞预检与运行期跟踪监控，最后按门控结果决定是否真正执行。
    本文件只负责启动组合与参数传递，回放算法全部在示教包的 TeachReplayNode 内实现。

节点组合
    只启动一个节点：示教包的 TeachReplayNode（可执行名 TeachReplayNode，节点名
    teach_replay_node）。它通过 ``arm_namespace`` 与底层控制器或仿真后端对接，因此本
    文件本身不区分真实/仿真——唯一执行后端由上层启动文件选择，这里只递参数。

参数来源（后者覆盖前者）
    1. ``common_config``（默认本包 config/operator_common.yaml）：提供关节名称、命名空间
       与公共限位；
    2. ``teach_config``（默认本包 config/teach_control.yaml）：提供示教回放专用参数；
    3. 下方逐个 DeclareLaunchArgument：可在命令行用 ``参数:=值`` 覆盖。
    节点 ``parameters`` 列表中参数字典排在参数文件之后，故这里的取值优先于 YAML。

安全默认值（改动前请确认后果）
    - ``dry_run`` 默认 ``true``：只加载、预处理与校验，绝不下发运动；真机回放必须由
      操作者显式传 ``dry_run:=false``。
    - 平滑、滤波、重采样、碰撞预检、运行期跟踪监控默认全部开启，任何一项置 false
      都会减少一层保护。
    - 起点对齐默认走规划服务（``use_moveit_start_align=true``），避免起点偏差较大时
      直接插值造成大幅横扫。
    - ``max_tracking_error_rad`` / ``max_live_velocity_rad_s`` 是运行期停机阈值，
      调大等于放宽保护。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """构造示教回放的启动描述：声明全部可覆盖参数并启动唯一的回放节点。"""
    arm_namespace = LaunchConfiguration("arm_namespace")
    record_path = LaunchConfiguration("record_path")
    dry_run = LaunchConfiguration("dry_run")
    speed = LaunchConfiguration("speed")
    yellow_max_speed = LaunchConfiguration("yellow_max_speed")
    max_replay_velocity_rad_s = LaunchConfiguration("max_replay_velocity_rad_s")
    max_replay_acceleration_rad_s2 = LaunchConfiguration("max_replay_acceleration_rad_s2")
    max_replay_jerk_rad_s3 = LaunchConfiguration("max_replay_jerk_rad_s3")
    large_motion_span_rad = LaunchConfiguration("large_motion_span_rad")
    large_motion_total_rad = LaunchConfiguration("large_motion_total_rad")
    large_motion_max_speed = LaunchConfiguration("large_motion_max_speed")
    start_hold_sec = LaunchConfiguration("start_hold_sec")
    soft_start_duration = LaunchConfiguration("soft_start_duration")
    soft_start_steps = LaunchConfiguration("soft_start_steps")
    first_hold_sec = LaunchConfiguration("first_hold_sec")
    final_hold_sec = LaunchConfiguration("final_hold_sec")
    use_moveit_start_align = LaunchConfiguration("use_moveit_start_align")
    moveit_start_skip_threshold = LaunchConfiguration("moveit_start_skip_threshold")
    moveit_planning_service = LaunchConfiguration("moveit_planning_service")
    moveit_planning_time = LaunchConfiguration("moveit_planning_time")
    collision_check_enabled = LaunchConfiguration("collision_check_enabled")
    collision_check_service = LaunchConfiguration("collision_check_service")
    collision_check_max_samples = LaunchConfiguration("collision_check_max_samples")
    collision_check_timeout_sec = LaunchConfiguration("collision_check_timeout_sec")
    smoothing_enabled = LaunchConfiguration("smoothing_enabled")
    smoothing_window = LaunchConfiguration("smoothing_window")
    filter_enabled = LaunchConfiguration("filter_enabled")
    filter_cutoff_hz = LaunchConfiguration("filter_cutoff_hz")
    filter_sample_rate_hz = LaunchConfiguration("filter_sample_rate_hz")
    resample_enabled = LaunchConfiguration("resample_enabled")
    resample_rate_hz = LaunchConfiguration("resample_rate_hz")
    time_parameterization_method = LaunchConfiguration("time_parameterization_method")
    max_prepared_jump_rad = LaunchConfiguration("max_prepared_jump_rad")
    replay_monitor_enabled = LaunchConfiguration("replay_monitor_enabled")
    replay_monitor_period_sec = LaunchConfiguration("replay_monitor_period_sec")
    replay_monitor_start_grace_sec = LaunchConfiguration("replay_monitor_start_grace_sec")
    replay_monitor_violation_grace_sec = LaunchConfiguration("replay_monitor_violation_grace_sec")
    max_tracking_error_rad = LaunchConfiguration("max_tracking_error_rad")
    max_live_velocity_rad_s = LaunchConfiguration("max_live_velocity_rad_s")
    common_config = LaunchConfiguration("common_config")
    teach_config = LaunchConfiguration("teach_config")
    # 默认参数文件从本启动组合包的 share 目录解析，保证安装后不依赖源码树路径。
    config_share = FindPackageShare("rebotarm_bringup")

    return LaunchDescription(
        [
            # ---- 基础目标与执行模式 ----
            # arm_namespace：话题/服务/动作的命名空间前缀；决定记录里的关节名与回放目标
            #   后端如何解析，必须与已使能的控制器或仿真后端一致。
            DeclareLaunchArgument("arm_namespace", default_value="rebotarm"),
            # record_path：示教记录文件路径（JSONL，每行一个采样帧）。相对路径按启动时
            #   的工作目录解析；文件不存在或格式非法时回放流程直接失败，不会产生运动。
            DeclareLaunchArgument("record_path", default_value="teleop_records/teach_record.jsonl"),
            # dry_run：true=只做加载/预处理/门控校验，不下发轨迹（默认，安全）；真机回放
            #   必须显式置 false，否则流程会停在"仅校验"状态。
            DeclareLaunchArgument("dry_run", default_value="true"),
            # ---- 速度与风险门控 ----
            # speed：回放速度倍率（无量纲）。1.0=按录制原速；预处理内部夹到 [0.01, 1.0]，
            #   因此大于 1.0 不会加速，只会按 1.0 处理；调小则整体放慢。
            DeclareLaunchArgument("speed", default_value="1.0"),
            # yellow_max_speed：质量分析判为"黄色"（跳变偏大）的记录所允许的最大回放速度
            #   倍率；真实回放时若预处理后的有效速度超过该值即拒绝执行。
            DeclareLaunchArgument("yellow_max_speed", default_value="0.6"),
            # ---- 动力学上限（质量分析与重定时共用）----
            # max_replay_velocity_rad_s：关节速度上限标量（rad/s）。仅当逐关节列表
            #   max_replay_velocity_rad_s_by_joint 长度与记录关节数不匹配时作为回退值；
            #   逐关节上限默认由 teach_control.yaml 提供（前 3 轴 3.0、后 3 轴 1.8）。
            DeclareLaunchArgument("max_replay_velocity_rad_s", default_value="3.0"),
            # max_replay_acceleration_rad_s2：关节加速度上限（rad/s²）；调小使加减速更缓。
            DeclareLaunchArgument("max_replay_acceleration_rad_s2", default_value="5.0"),
            # max_replay_jerk_rad_s3：关节加加速度（jerk）上限（rad/s³）；限制冲击，
            #   与 MoveIt 的 max_jerk 20.0 保持同一量级。
            DeclareLaunchArgument("max_replay_jerk_rad_s3", default_value="20.0"),
            # ---- 大范围运动判定 ----
            # large_motion_span_rad：单关节最大行程阈值（rad），超过即判定为大范围运动。
            DeclareLaunchArgument("large_motion_span_rad", default_value="0.8"),
            # large_motion_total_rad：各关节累计行程阈值（rad，按关节求和），超过即判定为
            #   大范围运动；两个条件满足其一即触发。
            DeclareLaunchArgument("large_motion_total_rad", default_value="2.5"),
            # large_motion_max_speed：大范围运动的推荐速度上限（倍率）。它只随预处理结果与
            #   状态上报，本身不再二次限速；真正的限速走 yellow_max_speed 分支。
            DeclareLaunchArgument("large_motion_max_speed", default_value="1.0"),
            # ---- 起停保持段与软启动 ----
            # start_hold_sec：轨迹开头先保持"当前位形"的时长（s），给出明确的首个目标点。
            DeclareLaunchArgument("start_hold_sec", default_value="0.8"),
            # soft_start_duration：起点判为 direct（直接回放）时软启动插值的时长（s）。
            DeclareLaunchArgument("soft_start_duration", default_value="1.0"),
            # soft_start_steps：软启动插值的点数（含首尾），越多越平滑、轨迹越长。
            DeclareLaunchArgument("soft_start_steps", default_value="30"),
            # first_hold_sec：到达记录首点后的保持时长（s），两条起点对齐路径都会追加。
            DeclareLaunchArgument("first_hold_sec", default_value="0.3"),
            # final_hold_sec：轨迹末尾保持末点位置的时长（s）；0 表示不追加保持段。
            DeclareLaunchArgument("final_hold_sec", default_value="1.0"),
            # ---- 起点对齐（改用 MoveIt 规划服务）----
            # use_moveit_start_align：true=用规划服务把当前位形对齐到记录首点；
            #   false=退回关节空间插值（更快但可能大幅横扫）。
            DeclareLaunchArgument("use_moveit_start_align", default_value="true"),
            # moveit_start_skip_threshold：当前位形与首点的最大关节误差小于该值（rad）时
            #   跳过规划，只保留保持点，避免为微小偏差做一次规划。
            DeclareLaunchArgument("moveit_start_skip_threshold", default_value="0.005"),
            # moveit_planning_service：规划服务名，由 MoveIt 的 move_group 提供。
            DeclareLaunchArgument("moveit_planning_service", default_value="/plan_kinematic_path"),
            # moveit_planning_time：单次规划允许的时长（s）；调大提高成功率，但会更久阻塞。
            DeclareLaunchArgument("moveit_planning_time", default_value="3.0"),
            # ---- 碰撞预检 ----
            # collision_check_enabled：是否对预处理后的轨迹做状态有效性抽样检查；false 时
            #   预检直接返回 disabled，不会拦截真实回放（削弱保护，慎用）。
            DeclareLaunchArgument("collision_check_enabled", default_value="true"),
            # collision_check_service：状态有效性服务名，同样由 move_group 提供。
            DeclareLaunchArgument("collision_check_service", default_value="/check_state_validity"),
            # collision_check_max_samples：轨迹上最多抽样的位形点数，内部下限 1；调大覆盖
            #   更密但服务调用耗时更长。
            DeclareLaunchArgument("collision_check_max_samples", default_value="80"),
            # collision_check_timeout_sec：碰撞预检的整体超时（s），内部下限 0.1；超时按
            #   "结论不可用"处理，真实回放会被拦截（dry_run 不受影响）。
            DeclareLaunchArgument("collision_check_timeout_sec", default_value="2.0"),
            # ---- 预处理：平滑 / 滤波 / 重采样 / 重定时 ----
            # smoothing_enabled / smoothing_window：是否做滑动平均平滑，以及窗口点数
            #   （点数越大越平滑，但会削弱细节、增大跟踪滞后）。
            DeclareLaunchArgument("smoothing_enabled", default_value="true"),
            DeclareLaunchArgument("smoothing_window", default_value="7"),
            # filter_enabled / filter_cutoff_hz / filter_sample_rate_hz：是否做低通滤波、
            #   截止频率（Hz）与滤波使用的采样率（Hz）。截止频率必须低于采样率的一半
            #   （5.0 < 150.0/2），否则滤波参数无效。
            DeclareLaunchArgument("filter_enabled", default_value="true"),
            DeclareLaunchArgument("filter_cutoff_hz", default_value="5.0"),
            DeclareLaunchArgument("filter_sample_rate_hz", default_value="150.0"),
            # resample_enabled / resample_rate_hz：是否重采样及目标采样率（Hz）。150 Hz 与
            #   控制器 100 Hz 反馈、示教录制 150 Hz 对齐，避免插值引入额外抖动。
            DeclareLaunchArgument("resample_enabled", default_value="true"),
            DeclareLaunchArgument("resample_rate_hz", default_value="150.0"),
            # time_parameterization_method：重定时方法，取值 "auto" /
            #   "current_jerk_retime" / "ruckig"；非法值会回落为 "auto"。
            DeclareLaunchArgument("time_parameterization_method", default_value="auto"),
            # max_prepared_jump_rad：预处理后相邻点允许的最大跳变（rad），真实回放的门控
            #   之一；超过即拒绝下发，用于拦住预处理没能压平的突变。
            DeclareLaunchArgument("max_prepared_jump_rad", default_value="0.02"),
            # ---- 运行期跟踪监控 ----
            # replay_monitor_enabled：是否在回放期间比较"目标位形 vs 实际反馈"。
            DeclareLaunchArgument("replay_monitor_enabled", default_value="true"),
            # replay_monitor_period_sec：监控定时器周期（s），默认 0.05 即 20 Hz，
            #   内部下限 0.02。周期越短发现异常越快，CPU 负载越高。
            DeclareLaunchArgument("replay_monitor_period_sec", default_value="0.05"),
            # replay_monitor_start_grace_sec：起步宽限期（s），期间不判超差，避免起步瞬间
            #   反馈尚未跟上导致误停。
            DeclareLaunchArgument("replay_monitor_start_grace_sec", default_value="1.0"),
            # replay_monitor_violation_grace_sec：连续超差需持续超过该时长（s）才触发停止，
            #   用于滤掉单帧抖动造成的误停；调大更宽容也更晚停。
            DeclareLaunchArgument("replay_monitor_violation_grace_sec", default_value="0.30"),
            # max_tracking_error_rad：允许的关节跟踪误差（目标位形与反馈之差，rad）。
            DeclareLaunchArgument("max_tracking_error_rad", default_value="0.25"),
            # max_live_velocity_rad_s：允许的反馈关节速度（rad/s）；超过同样计入超差。
            DeclareLaunchArgument("max_live_velocity_rad_s", default_value="3.0"),
            # common_config：公共命名空间、关节名称和关节限位。
            DeclareLaunchArgument(
                "common_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "operator_common.yaml"]
                ),
            ),
            # teach_config：示教回放、MoveIt 对齐、碰撞和跟踪保护参数。
            DeclareLaunchArgument(
                "teach_config",
                default_value=PathJoinSubstitution(
                    [config_share, "config", "teach_control.yaml"]
                ),
            ),
            # 唯一的回放节点。参数文件先加载，下面的启动参数字典再覆盖同名键。
            Node(
                package="rebotarm_teach",
                executable="TeachReplayNode",
                name="teach_replay_node",
                output="screen",
                parameters=[
                    common_config,
                    teach_config,
                    {
                        "arm_namespace": arm_namespace,
                        "record_path": record_path,
                        "dry_run": dry_run,
                        "speed": speed,
                        "yellow_max_speed": yellow_max_speed,
                        "max_replay_velocity_rad_s": max_replay_velocity_rad_s,
                        "max_replay_acceleration_rad_s2": max_replay_acceleration_rad_s2,
                        "max_replay_jerk_rad_s3": max_replay_jerk_rad_s3,
                        "large_motion_span_rad": large_motion_span_rad,
                        "large_motion_total_rad": large_motion_total_rad,
                        "large_motion_max_speed": large_motion_max_speed,
                        "start_hold_sec": start_hold_sec,
                        "soft_start_duration": soft_start_duration,
                        "soft_start_steps": soft_start_steps,
                        "first_hold_sec": first_hold_sec,
                        "final_hold_sec": final_hold_sec,
                        "use_moveit_start_align": use_moveit_start_align,
                        "moveit_start_skip_threshold": moveit_start_skip_threshold,
                        "moveit_planning_service": moveit_planning_service,
                        "moveit_planning_time": moveit_planning_time,
                        "collision_check_enabled": collision_check_enabled,
                        "collision_check_service": collision_check_service,
                        "collision_check_max_samples": collision_check_max_samples,
                        "collision_check_timeout_sec": collision_check_timeout_sec,
                        "smoothing_enabled": smoothing_enabled,
                        "smoothing_window": smoothing_window,
                        "filter_enabled": filter_enabled,
                        "filter_cutoff_hz": filter_cutoff_hz,
                        "filter_sample_rate_hz": filter_sample_rate_hz,
                        "resample_enabled": resample_enabled,
                        "resample_rate_hz": resample_rate_hz,
                        "time_parameterization_method": time_parameterization_method,
                        "max_prepared_jump_rad": max_prepared_jump_rad,
                        "replay_monitor_enabled": replay_monitor_enabled,
                        "replay_monitor_period_sec": replay_monitor_period_sec,
                        "replay_monitor_start_grace_sec": replay_monitor_start_grace_sec,
                        "replay_monitor_violation_grace_sec": replay_monitor_violation_grace_sec,
                        "max_tracking_error_rad": max_tracking_error_rad,
                        "max_live_velocity_rad_s": max_live_velocity_rad_s,
                    },
                ],
            ),
        ]
    )
